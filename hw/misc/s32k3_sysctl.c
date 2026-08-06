/*
 * NXP S32K3xx system/security control blocks QEMU placeholder model
 *
 * One device with a "kind" property covering:
 *   0 = STCU  (Self-Test Control Unit, LBIST/MBIST status) @ 0x403A0000
 *   1 = MSCM  (Misc System Control, CPN/CFG)              @ 0x40260000
 *   2 = RTC   (Real-Time Clock, time counters)            @ 0x40288000
 *   3 = CRC   (Cyclic Redundancy Check, ctrl/data)        @ 0x40380000
 *
 * Each reports sensible reset values so real firmware init does not
 * misread the block state (STCU BIST "done/ok", MSCM single core,
 * RTC counting, CRC idle).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_S32K3_SYSCTL "s32k3-sysctl"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3SysctlState, S32K3_SYSCTL)

/* STCU: status registers (RM 54) */
#define STCU_LBIST_CTRL  0x00
#define STCU_MBIST_CTRL  0x04
#define STCU_STATUS      0x08   /* BIST complete/ok status */
#define STCU_ERR_STAT    0x0C

/* MSCM: misc system control (RM 7) */
#define MSCM_CPN         0x00   /* core processor number */
#define MSCM_CFG         0x04   /* core config */
#define MSCM_XLDOVR      0x08
#define MSCM_ENDES       0x0C
#define MSCM_ENEDC       0x10

/* RTC: time registers (RM) */
#define RTC_TSR          0x00   /* time seconds */
#define RTC_TPR          0x04   /* time prescaler */
#define RTC_TAR          0x08   /* time alarm */
#define RTC_TCR          0x0C   /* time compensation */
#define RTC_CR           0x10   /* control */
#define RTC_SR           0x14   /* status */

/* CRC: registers */
#define CRC_CTRL         0x00
#define CRC_STATUS       0x04
#define CRC_ACCESS_DATAL 0x08
#define CRC_ACCESS_DATAH 0x0C

struct S32K3SysctlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *module_clk;

    uint32_t kind;      /* 0=STCU 1=MSCM 2=RTC 3=CRC */
    uint32_t regs[0x100];
};

static void s32k3_sysctl_reset(DeviceState *dev)
{
    S32K3SysctlState *s = S32K3_SYSCTL(dev);

    memset(s->regs, 0, sizeof(s->regs));
    switch (s->kind) {
    case 0:   /* STCU: LBIST/MBIST done + ok */
        s->regs[STCU_STATUS / 4] = 0x3;   /* LBIST+MBIST complete */
        break;
    case 1:   /* MSCM: CFG 复位 0（固件 core0 读 MSCM_CFG==0 走完整
               * init_data_bss 拷贝；非 0 会被当成非主核走 core2 部分拷贝，
               * 导致 .sram_data 不拷、函数指针表为 0） */
        s->regs[MSCM_CPN / 4] = 0;
        s->regs[MSCM_CFG / 4] = 0;
        break;
    case 2:   /* RTC: counting (TSR increments) */
        s->regs[RTC_CR / 4] = 0x00000002; /* SUP=1 */
        s->regs[RTC_SR / 4] = 0;
        break;
    case 3:   /* CRC: idle */
        s->regs[CRC_CTRL / 4] = 0;
        s->regs[CRC_STATUS / 4] = 0;
        break;
    case 4:   /* CONFIGURATION_GPR (RM): CONFIG_REG_GPR[31:29] APP_CORE_ACC=5
               * 允许应用核写 FIRC 分频等；固件读 !=5 会报时钟写保护错误 */
        s->regs[0x64 / 4] = 5u << 29;
        break;
    }
}

static uint64_t s32k3_sysctl_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3SysctlState *s = opaque;
    uint32_t r = 0;

    if (addr >= sizeof(s->regs)) {
        return 0;
    }
    r = s->regs[addr / 4];

    /* RTC: TSR/TPR 自增（简化：每次读 +1，模拟计数） */
    if (s->kind == 2 && (addr == RTC_TSR || addr == RTC_TPR)) {
        r = s->regs[addr / 4]++;
    }
    return r;
}

static void s32k3_sysctl_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    S32K3SysctlState *s = opaque;
    uint32_t v = value;

    if (addr >= sizeof(s->regs)) {
        return;
    }
    s->regs[addr / 4] = v;

    /* RTC: 写 CR 使能时 TSR 开始自增（简化：SUP=1 置位即启动） */
    if (s->kind == 2 && addr == RTC_CR && (v & 0x2)) {
        s->regs[RTC_TSR / 4] = 0;   /* 从 0 开始 */
    }
}

static const MemoryRegionOps s32k3_sysctl_ops = {
    .read = s32k3_sysctl_read,
    .write = s32k3_sysctl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void s32k3_sysctl_init(Object *obj)
{
    S32K3SysctlState *s = S32K3_SYSCTL(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_sysctl_ops, s,
                          TYPE_S32K3_SYSCTL, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void s32k3_sysctl_realize(DeviceState *dev, Error **errp)
{
    S32K3SysctlState *s = S32K3_SYSCTL(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_sysctl: module_clk must be connected");
        return;
    }
    s32k3_sysctl_reset(dev);
}

static const Property s32k3_sysctl_properties[] = {
    DEFINE_PROP_UINT32("kind", S32K3SysctlState, kind, 0),
};

static void s32k3_sysctl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_sysctl_reset);
    device_class_set_props(dc, s32k3_sysctl_properties);
    dc->realize = s32k3_sysctl_realize;
    dc->desc = "NXP S32K3xx system/security control (STCU/MSCM/RTC/CRC)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_sysctl_types[] = {
    {
        .name          = TYPE_S32K3_SYSCTL,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3SysctlState),
        .instance_init = s32k3_sysctl_init,
        .class_init    = s32k3_sysctl_class_init,
    },
};

DEFINE_TYPES(s32k3_sysctl_types)
