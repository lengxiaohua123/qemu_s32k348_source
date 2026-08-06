#ifndef S32K3_SEMA42_H
#define S32K3_SEMA42_H

#include "hw/core/sysbus.h"

#define TYPE_S32K3_SEMA42 "s32k3-sema42"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3Sema42State, S32K3_SEMA42)

struct S32K3Sema42State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint8_t      gate[16];   /* 0=unlocked, else domain+1 */
    uint16_t     rstgt_r;
    uint8_t      rstgt_step;
};

#endif /* S32K3_SEMA42_H */
