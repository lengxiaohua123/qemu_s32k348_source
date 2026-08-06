/*
 * NXP S32K3xx eDMA QEMU device model
 *
 * Functional 32-channel eDMA.  Simple (non-scatter-gather) transfers:
 * writing TCD_CSR.START (or SERQ/SSRT) performs the CITER loop of
 * NBYTES copies from SADDR to DADDR with SOFF/DOFF updates, then sets
 * DONE and raises the channel interrupt when INTMAJOR is set.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/dma/s32k3_edma.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "qemu/log.h"
#include "qemu/module.h"

static uint32_t edma_tcd_r32(S32K3EdmaState *s, int ch, int off)
{
    const uint8_t *t = s->tcd[ch] + off;
    return (uint32_t)t[0] | ((uint32_t)t[1] << 8) |
           ((uint32_t)t[2] << 16) | ((uint32_t)t[3] << 24);
}

static uint16_t edma_tcd_r16(S32K3EdmaState *s, int ch, int off)
{
    const uint8_t *t = s->tcd[ch] + off;
    return (uint16_t)t[0] | ((uint16_t)t[1] << 8);
}

static void edma_tcd_w16(S32K3EdmaState *s, int ch, int off, uint16_t v)
{
    s->tcd[ch][off]     = v & 0xff;
    s->tcd[ch][off + 1] = (v >> 8) & 0xff;
}

static void s32k3_edma_update_irq(S32K3EdmaState *s, int ch)
{
    bool level = (s->intr & (1u << ch)) && (s->eei & (1u << ch));
    qemu_set_irq(s->irq[ch], level);
}

/* 完成当前通道 major loop：地址调整、中断、linking */
static void s32k3_edma_transfer(S32K3EdmaState *s, int ch);
static void s32k3_edma_finish_channel(S32K3EdmaState *s, int ch)
{
    uint16_t csr;

    /* major loop 完成：源地址加 SLAST 回绕，目标按 DLASTSGA */
    s->ch_saddr += s->ch_slast;
    s->ch_daddr += s->ch_dlastsga;

    /* 写回 TCD */
    s->tcd[ch][TCD_SADDR]     = s->ch_saddr & 0xff;
    s->tcd[ch][TCD_SADDR + 1] = (s->ch_saddr >> 8) & 0xff;
    s->tcd[ch][TCD_SADDR + 2] = (s->ch_saddr >> 16) & 0xff;
    s->tcd[ch][TCD_SADDR + 3] = (s->ch_saddr >> 24) & 0xff;
    s->tcd[ch][TCD_DADDR]     = s->ch_daddr & 0xff;
    s->tcd[ch][TCD_DADDR + 1] = (s->ch_daddr >> 8) & 0xff;
    s->tcd[ch][TCD_DADDR + 2] = (s->ch_daddr >> 16) & 0xff;
    s->tcd[ch][TCD_DADDR + 3] = (s->ch_daddr >> 24) & 0xff;
    csr = edma_tcd_r16(s, ch, TCD_CSR);
    csr |= TCD_CSR_DONE;
    csr &= ~TCD_CSR_START;
    edma_tcd_w16(s, ch, TCD_CSR, csr);

    /* CITER 递减写回；无 MAJORELINK 时 reload BITER */
    edma_tcd_w16(s, ch, TCD_CITER, 0);
    if (!(csr & TCD_CSR_MAJORELINK)) {
        edma_tcd_w16(s, ch, TCD_CITER, s->ch_biter);
    }

    if (csr & TCD_CSR_INTMAJOR) {
        s->intr |= 1u << ch;
        s32k3_edma_update_irq(s, ch);
    }

    s->active_ch = -1;

    /* MAJORELINK：major 完成触发目标通道请求 */
    if (csr & TCD_CSR_MAJORELINK) {
        int link_ch = (csr & TCD_CSR_MAJORLINKCH_MASK) >>
                      TCD_CSR_MAJORLINKCH_SHIFT;
        if (link_ch < S32K3_EDMA_CHANNELS && link_ch != ch) {
            s32k3_edma_transfer(s, link_ch);
        }
    }
}

/* 一个 minor loop 节拍：搬运 NBYTES 并步进 */
static void s32k3_edma_minor_tick(void *opaque);
static void s32k3_edma_req_set(void *opaque, int line, int level);
static void s32k3_edma_minor_step(S32K3EdmaState *s)
{
    int ch = s->active_ch;
    uint8_t *buf;

    if (ch < 0) {
        return;
    }
    buf = g_malloc(s->ch_nbytes);
    if (address_space_read(&s->as, s->ch_saddr, MEMTXATTRS_UNSPECIFIED,
                           buf, s->ch_nbytes) != MEMTX_OK ||
        address_space_write(&s->as, s->ch_daddr, MEMTXATTRS_UNSPECIFIED,
                            buf, s->ch_nbytes) != MEMTX_OK) {
        s->err |= 1u << ch;
        g_free(buf);
        qemu_irq_raise(s->err_irq);
        s->active_ch = -1;
        return;
    }
    g_free(buf);
    s->ch_saddr += s->ch_soff;
    s->ch_daddr += s->ch_doff;

    if (--s->ch_citer == 0) {
        s32k3_edma_finish_channel(s, ch);
    } else {
        /* 继续下一 minor loop */
        ptimer_transaction_begin(s->timer);
        ptimer_run(s->timer, 1);
        ptimer_transaction_commit(s->timer);
    }
}

/* 周期化传输：配置通道状态，按模块时钟逐 minor loop 节拍 */
static void s32k3_edma_transfer(S32K3EdmaState *s, int ch)
{
    uint32_t nbytes = edma_tcd_r32(s, ch, TCD_NBYTES);
    uint16_t citer = edma_tcd_r16(s, ch, TCD_CITER) & 0x7fff;
    uint16_t biter = edma_tcd_r16(s, ch, TCD_BITER) & 0x7fff;

    if (nbytes == 0 || nbytes > 4096) {
        s->err |= 1u << ch;
        qemu_irq_raise(s->err_irq);
        return;
    }
    if (citer == 0) {
        citer = biter ? biter : 1;
    }

    /* 装载通道状态 */
    s->active_ch = ch;
    s->ch_saddr = edma_tcd_r32(s, ch, TCD_SADDR);
    s->ch_daddr = edma_tcd_r32(s, ch, TCD_DADDR);
    s->ch_nbytes = nbytes;
    s->ch_soff = (int16_t)edma_tcd_r16(s, ch, TCD_SOFF);
    s->ch_doff = (int16_t)edma_tcd_r16(s, ch, TCD_DOFF);
    s->ch_slast = (int32_t)edma_tcd_r32(s, ch, TCD_SLAST);
    s->ch_dlastsga = edma_tcd_r32(s, ch, TCD_DLASTSGA);
    s->ch_citer = citer;
    s->ch_biter = biter;

    /* 每 minor loop 一拍（模块时钟） */
    ptimer_transaction_begin(s->timer);
    ptimer_set_freq(s->timer, clock_get_hz(s->module_clk) ?
                    clock_get_hz(s->module_clk) : 1);
    ptimer_set_count(s->timer, 1);
    ptimer_run(s->timer, 1);
    ptimer_transaction_commit(s->timer);
}

static void s32k3_edma_reset(DeviceState *dev)
{
    S32K3EdmaState *s = S32K3_EDMA(dev);
    int i;

    s->cr = 0;
    s->es = 0;
    s->erq = 0;
    s->eei = 0;
    s->intr = 0;
    s->err = 0;
    s->hrs = 0;
    memset(s->ch_csr, 0, sizeof(s->ch_csr));
    memset(s->ch_es, 0, sizeof(s->ch_es));
    memset(s->ch_int, 0, sizeof(s->ch_int));
    memset(s->ch_sbr, 0, sizeof(s->ch_sbr));
    memset(s->tcd, 0, sizeof(s->tcd));
    for (i = 0; i < S32K3_EDMA_CHANNELS; i++) {
        qemu_irq_lower(s->irq[i]);
    }
    qemu_irq_lower(s->err_irq);
}

static void s32k3_edma_write8_cmd(S32K3EdmaState *s, hwaddr addr, uint32_t v)
{
    int ch = v & 0x1f;

    switch (addr) {
    case EDMA_SEEI:
        s->eei |= 1u << ch;
        break;
    case EDMA_CEEI:
        s->eei &= ~(1u << ch);
        break;
    case EDMA_SERQ:
        s->erq |= 1u << ch;
        s32k3_edma_transfer(s, ch);
        break;
    case EDMA_CERQ:
        s->erq &= ~(1u << ch);
        break;
    case EDMA_SSRT:
        s32k3_edma_transfer(s, ch);
        break;
    case EDMA_CDNE:
        s->intr &= ~(1u << ch);
        break;
    case EDMA_CINT:
        s->intr &= ~(1u << ch);
        break;
    case EDMA_CERR:
        s->err &= ~(1u << ch);
        break;
    }
    s32k3_edma_update_irq(s, ch);
}

static uint64_t s32k3_edma_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3EdmaState *s = opaque;
    int ch;

    if (addr >= EDMA_CH_BASE &&
        addr < EDMA_CH_BASE + S32K3_EDMA_CHANNELS * EDMA_CH_STRIDE) {
        ch = (addr - EDMA_CH_BASE) / EDMA_CH_STRIDE;
        switch ((addr - EDMA_CH_BASE) % EDMA_CH_STRIDE) {
        case 0x0:
            return s->ch_csr[ch];
        case 0x4:
            return s->ch_es[ch];
        case 0x8:
            return s->ch_int[ch];
        case 0xC:
            return s->ch_sbr[ch];
        }
    }

    switch (addr) {
    case EDMA_CR:
        return s->cr;
    case EDMA_ES:
        return s->es;
    case EDMA_ERQ:
        return s->erq;
    case EDMA_EEI:
        return s->eei;
    case EDMA_INT:
        return s->intr;
    case EDMA_ERR:
        return s->err;
    case EDMA_HRS:
        return s->hrs;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_edma: read of unimplemented reg 0x%03" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void s32k3_edma_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
    S32K3EdmaState *s = opaque;
    uint32_t v = value;
    int ch;

    if (addr >= EDMA_CH_BASE &&
        addr < EDMA_CH_BASE + S32K3_EDMA_CHANNELS * EDMA_CH_STRIDE) {
        ch = (addr - EDMA_CH_BASE) / EDMA_CH_STRIDE;
        switch ((addr - EDMA_CH_BASE) % EDMA_CH_STRIDE) {
        case 0x0:
            s->ch_csr[ch] = v;
            return;
        case 0x4:
            s->ch_es[ch] = v;
            return;
        case 0x8:
            s->ch_int[ch] &= ~v;   /* W1C */
            return;
        case 0xC:
            s->ch_sbr[ch] = v;
            return;
        }
        return;
    }

    /* 8-bit command registers */
    if (addr >= EDMA_CEEI && addr <= EDMA_CINT) {
        s32k3_edma_write8_cmd(s, addr, v);
        return;
    }

    switch (addr) {
    case EDMA_CR:
        s->cr = v;
        break;
    case EDMA_ERQ:
        s->erq = v;
        break;
    case EDMA_EEI:
        s->eei = v;
        for (ch = 0; ch < S32K3_EDMA_CHANNELS; ch++) {
            s32k3_edma_update_irq(s, ch);
        }
        break;
    case EDMA_INT:
        s->intr &= ~v;
        for (ch = 0; ch < S32K3_EDMA_CHANNELS; ch++) {
            s32k3_edma_update_irq(s, ch);
        }
        break;
    case EDMA_ERR:
        s->err &= ~v;
        if (!s->err) {
            qemu_irq_lower(s->err_irq);
        }
        break;
    case EDMA_ES:
        s->es &= ~v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_edma: write of unimplemented reg 0x%03" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_edma_ops = {
    .read = s32k3_edma_read,
    .write = s32k3_edma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* ---------------- TCD regions ---------------- */

static uint64_t s32k3_edma_tcd_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3EdmaState *s = opaque;
    int ch = addr / TCD_SIZE;
    int off = addr % TCD_SIZE;
    uint32_t r = 0;
    int i;

    if (ch >= S32K3_EDMA_CHANNELS) {
        return 0;
    }
    for (i = size - 1; i >= 0; i--) {
        r = (r << 8) | s->tcd[ch][off + i];
    }
    return r;
}

static void s32k3_edma_tcd_write(void *opaque, hwaddr addr,
                                 uint64_t value, unsigned size)
{
    S32K3EdmaState *s = opaque;
    int ch = addr / TCD_SIZE;
    int off = addr % TCD_SIZE;
    int i;

    if (ch >= S32K3_EDMA_CHANNELS) {
        return;
    }
    for (i = 0; i < size; i++) {
        s->tcd[ch][off + i] = (value >> (8 * i)) & 0xff;
    }
    /* START bit triggers the transfer */
    if (off <= TCD_CSR && off + size > TCD_CSR) {
        uint16_t csr = edma_tcd_r16(s, ch, TCD_CSR);
        if (csr & TCD_CSR_START) {
            s32k3_edma_transfer(s, ch);
        }
    }
}

static const MemoryRegionOps s32k3_edma_tcd_ops = {
    .read = s32k3_edma_tcd_read,
    .write = s32k3_edma_tcd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void s32k3_edma_init(Object *obj)
{
    S32K3EdmaState *s = S32K3_EDMA(obj);
    int i;

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_edma_ops, s,
                          TYPE_S32K3_EDMA, 0x4000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    memory_region_init_io(&s->tcd1, obj, &s32k3_edma_tcd_ops, s,
                          TYPE_S32K3_EDMA ".tcd1", 12 * TCD_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->tcd1);
    memory_region_init_io(&s->tcd2, obj, &s32k3_edma_tcd_ops, s,
                          TYPE_S32K3_EDMA ".tcd2", 20 * TCD_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->tcd2);

    for (i = 0; i < S32K3_EDMA_CHANNELS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq[i]);
    }
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->err_irq);
    qdev_init_gpio_in(DEVICE(s), s32k3_edma_req_set, S32K3_EDMA_CHANNELS);
    s->timer = ptimer_init(s32k3_edma_minor_tick, s,
                           PTIMER_POLICY_LEGACY);
    s->active_ch = -1;
}

/* DMAMUX 外设请求输入：电平触发对应通道 eDMA 传输 */
static void s32k3_edma_req_set(void *opaque, int line, int level)
{
    S32K3EdmaState *s = opaque;

    if (line < 0 || line >= S32K3_EDMA_CHANNELS) {
        return;
    }
    if (level && s->active_ch < 0) {
        uint16_t csr = edma_tcd_r16(s, line, TCD_CSR);
        if (csr & TCD_CSR_START) {
            s32k3_edma_transfer(s, line);
        }
    }
}

/* ptimer 回调：一个 minor loop 节拍 */
static void s32k3_edma_minor_tick(void *opaque)
{
    S32K3EdmaState *s = opaque;

    s32k3_edma_minor_step(s);
}

static void s32k3_edma_realize(DeviceState *dev, Error **errp)
{
    S32K3EdmaState *s = S32K3_EDMA(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_edma: module_clk must be connected");
        return;
    }
    if (!s->mem) {
        s->mem = get_system_memory();
    }
    address_space_init(&s->as, s->mem, "s32k3-edma");
    s32k3_edma_reset(dev);
}

static const Property s32k3_edma_properties[] = {
    DEFINE_PROP_LINK("memory", S32K3EdmaState, mem,
                     TYPE_MEMORY_REGION, MemoryRegion *),
};

static void s32k3_edma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_edma_reset);
    dc->realize = s32k3_edma_realize;
    device_class_set_props(dc, s32k3_edma_properties);
    dc->desc = "NXP S32K3xx eDMA (32 channels, simple transfers)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_edma_types[] = {
    {
        .name          = TYPE_S32K3_EDMA,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3EdmaState),
        .instance_init = s32k3_edma_init,
        .class_init    = s32k3_edma_class_init,
    },
};

DEFINE_TYPES(s32k3_edma_types)
