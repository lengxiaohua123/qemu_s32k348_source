#ifndef S32K3_RTC_H
#define S32K3_RTC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_S32K3_RTC "s32k3-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3RTCState, S32K3_RTC)

struct S32K3RTCState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    ptimer_state *ptimer;

    uint32_t rtcsupv;
    uint32_t rtcc;
    uint32_t rtcs;
    uint32_t rtccnt;
    uint32_t apival;
    uint32_t rtcval;
};

#endif /* S32K3_RTC_H */
