/*
 * S32K3 SEMA42 semaphore (RM 17: Semaphores2)
 *
 * 16 gates, each an 8-bit byte at offset 0 + (n + 3 - 2*(n mod 4)).
 * Gate value: 0=unlocked, else locked by domain (value-1).
 * Only the locking domain may unlock (write 0).
 * RSTGT @0x42 (16-bit): two-step secure reset.
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/misc/s32k3_sema42.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define SEMA42_NUM_GATES 16
#define SEMA42_RSTGT_OFF 0x42

/* gate n byte offset: 0 + (n + 3 - 2*(n mod 4)) */
static inline hwaddr s32k3_sema42_gate_off(int n)
{
    return n + 3 - 2 * (n % 4);
}

static uint64_t s32k3_sema42_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3Sema42State *s = opaque;

    if (addr < 0x40) {
        for (int n = 0; n < SEMA42_NUM_GATES; n++) {
            if (s32k3_sema42_gate_off(n) == addr) {
                return s->gate[n];   /* 8-bit */
            }
        }
    }
    if (addr == SEMA42_RSTGT_OFF) {
        return s->rstgt_r;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: read @0x%lx size %u\n",
                  __func__, addr, size);
    return 0;
}

static void s32k3_sema42_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    S32K3Sema42State *s = opaque;
    uint32_t v = value;

    if (addr < 0x40) {
        for (int n = 0; n < SEMA42_NUM_GATES; n++) {
            if (s32k3_sema42_gate_off(n) == addr) {
                uint8_t w = v & 0xff;
                /* 写 0 = 解锁（任意域写 0 即解锁——手册：gate 0=unlocked）；
                 * 写 1..15 = 锁给域 (w-1)；其他值 NOP */
                if (w == 0) {
                    s->gate[n] = 0;
                } else if (w <= 15 && s->gate[n] == 0) {
                    s->gate[n] = w;
                }
                return;
            }
        }
    }
    if (addr == SEMA42_RSTGT_OFF) {
        /* 两次写序列：第一写 RSTGDP=0x1D；第二写 RSTGDP=0xE2 + RSTGTN */
        if ((v >> 8) == 0x1D) {
            s->rstgt_step = 1;
        } else if ((v >> 8) == 0xE2 && s->rstgt_step == 1) {
            uint8_t n = v & 0xff;
            if (n < SEMA42_NUM_GATES) {
                s->gate[n] = 0;
            } else {
                memset(s->gate, 0, sizeof(s->gate));
            }
            s->rstgt_step = 0;
        }
        s->rstgt_r = v & 0x3FFF;   /* RSTGSM/RSTGMS/RSTGTN 可读 */
        return;
    }
}

static const MemoryRegionOps s32k3_sema42_ops = {
    .read  = s32k3_sema42_read,
    .write = s32k3_sema42_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void s32k3_sema42_reset(DeviceState *dev)
{
    S32K3Sema42State *s = S32K3_SEMA42(dev);

    memset(s->gate, 0, sizeof(s->gate));
    s->rstgt_r = 0;
    s->rstgt_step = 0;
}

static void s32k3_sema42_init(Object *obj)
{
    S32K3Sema42State *s = S32K3_SEMA42(obj);

    memory_region_init_io(&s->iomem, obj, &s32k3_sema42_ops, s,
                          TYPE_S32K3_SEMA42, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void s32k3_sema42_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, s32k3_sema42_reset);
}

static const TypeInfo s32k3_sema42_info = {
    .name          = TYPE_S32K3_SEMA42,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(S32K3Sema42State),
    .instance_init = s32k3_sema42_init,
    .class_init    = s32k3_sema42_class_init,
};

static void s32k3_sema42_register_types(void)
{
    type_register_static(&s32k3_sema42_info);
}

type_init(s32k3_sema42_register_types)
