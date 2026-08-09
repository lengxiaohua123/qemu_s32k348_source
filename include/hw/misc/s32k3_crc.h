#ifndef S32K3_CRC_H
#define S32K3_CRC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_S32K3_CRC "s32k3-crc"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3CRCState, S32K3_CRC)

struct S32K3CRCState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t ctrl;
    uint32_t gpoly;
    uint32_t seed;
    uint32_t crc;
};

#endif /* S32K3_CRC_H */
