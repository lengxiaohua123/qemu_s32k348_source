/*
 * s32k3_cmu.h — S32K3 Clock Monitoring Unit (CMU_FC)
 *
 * Minimal model: GCR.FCE -> SR.RS running (see s32k3_cmu.c).
 */
#ifndef S32K3_CMU_H
#define S32K3_CMU_H

#include "hw/core/sysbus.h"

#define TYPE_S32K3_CMU "s32k3-cmu"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3CmuState, S32K3_CMU)

struct S32K3CmuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t gcr;    /* Global Configuration (FCE bit0) */
    uint32_t rccr;
    uint32_t htcr;
    uint32_t ltcr;
    uint32_t ier;
};

#endif /* S32K3_CMU_H */
