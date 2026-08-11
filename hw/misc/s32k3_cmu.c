/*
 * s32k3_cmu.c — S32K3 Clock Monitoring Unit (CMU_FC) minimal model
 *
 * RM Ch56 CMU_FC @ 0x402BC000 (CMU_FC_0; CMU_FC_1-5 are aliases in the
 * same 0x4000 window).  RTD Clock_Ip_Monitor enables frequency check
 * (GCR.FCE) then polls SR.RS until it is no longer STOPPED (0).
 * Model clocks take effect immediately, so RS is set whenever FCE=1.
 *
 * Registers: GCR@0x00, RCCR@0x04, HTCR@0x08, LTCR@0x0C, SR@0x10, IER@0x14
 */
#include "qemu/osdep.h"
#include "hw/misc/s32k3_cmu.h"
#include "qemu/log.h"

#define CMU_GCR_FCE   (1u << 0)
#define CMU_SR_RS     (1u << 4)   /* run status: 1 = frequency check running */

static uint64_t s32k3_cmu_read(void *opaque, hwaddr offset, unsigned size)
{
    S32K3CmuState *s = S32K3_CMU(opaque);

    switch (offset) {
    case 0x00:
        return s->gcr;
    case 0x04:
        return s->rccr;
    case 0x08:
        return s->htcr;
    case 0x0C:
        return s->ltcr;
    case 0x10:
        /* SR.RS: running while frequency check is enabled (instant in QEMU) */
        return (s->gcr & CMU_GCR_FCE) ? CMU_SR_RS : 0;
    case 0x14:
        return s->ier;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read offset 0x%"
                       HWADDR_PRIx "\n", __func__, offset);
        return 0;
    }
}

static void s32k3_cmu_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    S32K3CmuState *s = S32K3_CMU(opaque);
    uint32_t v = value;

    switch (offset) {
    case 0x00:
        s->gcr = v;
        break;
    case 0x04:
        s->rccr = v;
        break;
    case 0x08:
        s->htcr = v;
        break;
    case 0x0C:
        s->ltcr = v;
        break;
    case 0x14:
        s->ier = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad write offset 0x%"
                       HWADDR_PRIx "\n", __func__, offset);
        break;
    }
}

static const MemoryRegionOps s32k3_cmu_ops = {
    .read = s32k3_cmu_read,
    .write = s32k3_cmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void s32k3_cmu_init(Object *obj)
{
    S32K3CmuState *s = S32K3_CMU(obj);

    memory_region_init_io(&s->iomem, obj, &s32k3_cmu_ops, s,
                          TYPE_S32K3_CMU, 0x4000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void s32k3_cmu_reset(DeviceState *dev)
{
    S32K3CmuState *s = S32K3_CMU(dev);

    s->gcr = 0;
    s->rccr = 0;
    s->htcr = 0;
    s->ltcr = 0;
    s->ier = 0;
}

static void s32k3_cmu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, s32k3_cmu_reset);
}

static const TypeInfo s32k3_cmu_info = {
    .name = TYPE_S32K3_CMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(S32K3CmuState),
    .instance_init = s32k3_cmu_init,
    .class_init = s32k3_cmu_class_init,
};

static void s32k3_cmu_register_types(void)
{
    type_register_static(&s32k3_cmu_info);
}

type_init(s32k3_cmu_register_types)
