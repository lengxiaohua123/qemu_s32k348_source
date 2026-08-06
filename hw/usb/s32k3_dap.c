/*
 * S32K348EVB USB debug probe (CMSIS-DAP v1, HID)
 *
 * Emulates a CMSIS-DAP debug adapter on the QEMU host USB bus.  The host
 * OS enumerates it as a standard USB HID device (no driver needed):
 *
 *   Linux:    lsusb shows "S32K348EVB CMSIS-DAP"; hidraw node appears
 *   Windows:  Device Manager -> Human Interface Devices
 *
 * OpenOCD (interface/cmsis-dap.cfg) and pyOCD attach to it out of the box.
 * SWD DP/AP transactions are translated to reads/writes of the emulated
 * S32K348 address space, so `load`/`flash write_image` program the board
 * code flash (flash regions are RAM-backed in this machine model).
 *
 * Note: a genuine SEGGER J-Link cannot be emulated (closed protocol with
 * license checks).  CMSIS-DAP is the open, tool-supported equivalent.
 *
 * Usage:
 *   qemu-system-arm -M s32k348evb -device s32k3-dap
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/usb/usb.h"
#include "desc.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "system/runstate.h"
#include "system/system.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "hw/core/qdev-properties.h"

#define TYPE_S32K3_DAP "s32k3-dap"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3DapState, S32K3_DAP)

#define DAP_PACKET_SIZE   64
#define DAP_PACKET_COUNT  4

/* CMSIS-DAP commands */
#define DAP_CMD_INFO            0x00
#define DAP_CMD_HOST_STATUS     0x01
#define DAP_CMD_CONNECT         0x02
#define DAP_CMD_DISCONNECT      0x03
#define DAP_CMD_TRANSFER_CFG    0x04
#define DAP_CMD_TRANSFER        0x05
#define DAP_CMD_TRANSFER_BLOCK  0x06
#define DAP_CMD_TRANSFER_ABORT  0x07
#define DAP_CMD_WRITE_ABORT     0x08
#define DAP_CMD_DELAY           0x09
#define DAP_CMD_RESET_TARGET    0x0A
#define DAP_CMD_SWJ_PINS        0x11
#define DAP_CMD_SWJ_CLOCK       0x1D
#define DAP_CMD_SWJ_SEQUENCE    0x1E
#define DAP_CMD_SWD_CONFIGURE   0x13
#define DAP_CMD_SWD_SEQINFO     0x1C

/* DP register addresses (A[3:2]) */
#define DP_DPIDR        0x0     /* read */
#define DP_ABORT        0x0     /* write */
#define DP_CTRL_STAT    0x4
#define DP_SELECT       0x8     /* write */
#define DP_RDBUFF       0xC     /* read */

/* AP (MEM-AP) register offsets inside bank */
#define AP_CSW          0x0
#define AP_TAR          0x4
#define AP_DRW          0xC

#define DPIDR_VALUE     0x6BA02477  /* ADIv5 JTAG/SWD DP */

struct S32K3DapState {
    USBDevice dev;

    /* link to the board memory (defaults to system memory) */
    MemoryRegion *mem;
    AddressSpace *as;
    AddressSpace  own_as;

    /* response fifo (responses are always one 64-byte packet) */
    uint8_t  resp[DAP_PACKET_SIZE];
    uint32_t resp_len;
    bool     resp_pending;

    /* DAP / debug-port state */
    bool     connected;
    uint8_t  port_mode;     /* 1 = SWD */
    uint32_t swj_clock;
    uint32_t dpidr;
    uint32_t ctrl_stat;
    uint32_t select;
    uint32_t abort_reg;
    uint32_t ap_csw;
    uint32_t ap_tar;
    uint32_t last_ap_read;
};

/* ------------------------------------------------------------------ */
/* Emulated SWD DP/AP access                                           */
/* ------------------------------------------------------------------ */

static bool dap_mem_read(S32K3DapState *s, hwaddr addr, uint32_t size,
                         uint32_t *val)
{
    uint8_t buf[4] = {0, 0, 0, 0};
    MemTxResult r;

    if (size > 4) {
        size = 4;
    }
    r = address_space_read(s->as, addr, MEMTXATTRS_UNSPECIFIED, buf, size);
    *val = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    return r == MEMTX_OK;
}

static bool dap_mem_write(S32K3DapState *s, hwaddr addr, uint32_t size,
                          uint32_t val)
{
    uint8_t buf[4];
    MemTxResult r;

    if (size > 4) {
        size = 4;
    }
    buf[0] = val & 0xff;
    buf[1] = (val >> 8) & 0xff;
    buf[2] = (val >> 16) & 0xff;
    buf[3] = (val >> 24) & 0xff;
    r = address_space_write(s->as, addr, MEMTXATTRS_UNSPECIFIED, buf, size);
    return r == MEMTX_OK;
}

static uint32_t dap_ap_size(S32K3DapState *s)
{
    static const uint32_t sz[4] = { 1, 2, 4, 4 };
    return sz[s->ap_csw & 3];
}

static void dap_ap_autoinc(S32K3DapState *s)
{
    uint32_t inc_mode = (s->ap_csw >> 4) & 3;

    if (inc_mode == 1) { /* auto-increment single */
        s->ap_tar += dap_ap_size(s);
    }
}

/* returns DAP response code: 1=OK, 2=WAIT, 4=FAULT */
static uint8_t dap_dp_access(S32K3DapState *s, bool read, uint8_t a2,
                             uint32_t *data)
{
    uint8_t addr = a2 * 4;

    if (read) {
        switch (addr) {
        case DP_DPIDR:
            *data = s->dpidr;
            break;
        case DP_CTRL_STAT:
            *data = s->ctrl_stat;
            break;
        case DP_RDBUFF:
            *data = s->last_ap_read;
            break;
        default:
            *data = 0;
            break;
        }
    } else {
        switch (addr) {
        case DP_ABORT:
            s->abort_reg = *data;
            break;
        case DP_CTRL_STAT:
            s->ctrl_stat = *data & 0xf0000030; /* keep sticky bits */
            break;
        case DP_SELECT:
            s->select = *data;
            break;
        default:
            break;
        }
    }
    return 0x01;
}

static uint8_t dap_ap_access(S32K3DapState *s, bool read, uint8_t a2,
                             uint32_t *data)
{
    uint8_t bank = (s->select >> 4) & 0xf;
    uint8_t addr = a2 * 4;

    /* only bank 0 modeled (standard MEM-AP) */
    if (bank != 0) {
        if (read) {
            *data = 0;
        }
        return 0x01;
    }

    switch (addr) {
    case AP_CSW:
        if (read) {
            *data = s->ap_csw;
        } else {
            s->ap_csw = *data;
        }
        break;
    case AP_TAR:
        if (read) {
            *data = s->ap_tar;
        } else {
            s->ap_tar = *data;
        }
        break;
    case AP_DRW: {
        bool ok;
        if (read) {
            ok = dap_mem_read(s, s->ap_tar, dap_ap_size(s), data);
            s->last_ap_read = *data;
        } else {
            ok = dap_mem_write(s, s->ap_tar, dap_ap_size(s), *data);
        }
        dap_ap_autoinc(s);
        return ok ? 0x01 : 0x04;
    }
    default:
        if (read) {
            *data = 0;
        }
        break;
    }
    return 0x01;
}

/* ------------------------------------------------------------------ */
/* CMSIS-DAP command processing                                        */
/* ------------------------------------------------------------------ */

static void dap_respond(S32K3DapState *s, const uint8_t *buf, uint32_t len)
{
    memset(s->resp, 0, DAP_PACKET_SIZE);
    if (len > DAP_PACKET_SIZE) {
        len = DAP_PACKET_SIZE;
    }
    memcpy(s->resp, buf, len);
    s->resp_len = DAP_PACKET_SIZE;
    s->resp_pending = true;
}

static void dap_cmd_info(S32K3DapState *s, const uint8_t *in)
{
    uint8_t out[DAP_PACKET_SIZE] = {0};
    const char *str = NULL;
    uint8_t id = in[1];
    uint8_t b;

    out[0] = DAP_CMD_INFO;
    switch (id) {
    case 0x01:
        str = "Moonshot";
        break;
    case 0x02:
        str = "S32K348EVB CMSIS-DAP";
        break;
    case 0x03:
        str = "S32K348-QEMU-0001";
        break;
    case 0x04:
        str = "1.10";
        break;
    case 0x00:  /* capabilities: SWD supported */
    case 0xF0:
        out[1] = 1;
        out[2] = 0x01;
        break;
    case 0xFE:  /* packet count */
        out[1] = 1;
        out[2] = DAP_PACKET_COUNT;
        break;
    case 0xFF:  /* packet size */
        out[1] = 1;
        out[2] = DAP_PACKET_SIZE;
        break;
    default:
        out[1] = 0;
        break;
    }
    if (str) {
        b = strlen(str) + 1;
        out[1] = b;
        memcpy(&out[2], str, b - 1);
        out[2 + b - 1] = 0;
    }
    dap_respond(s, out, DAP_PACKET_SIZE);
}

static void dap_cmd_transfer(S32K3DapState *s, const uint8_t *in)
{
    uint8_t out[DAP_PACKET_SIZE] = {0};
    uint8_t count = in[2];
    uint32_t ipos = 3, opos = 3;
    uint8_t done = 0, resp_code = 0x01;
    uint32_t i;

    out[0] = DAP_CMD_TRANSFER;

    for (i = 0; i < count; i++) {
        uint8_t req, a2;
        bool read, ap, vmatch, mmask;
        uint32_t data = 0, match_val = 0, match_mask = 0;
        uint8_t rc = 0x01;

        if (ipos >= DAP_PACKET_SIZE) {
            break;
        }
        req = in[ipos++];
        read  = req & 0x01;
        ap    = req & 0x02;
        a2    = (req >> 2) & 3;
        vmatch = req & 0x10;
        mmask  = req & 0x20;

        if (!read && ipos + 4 <= DAP_PACKET_SIZE) {
            memcpy(&data, &in[ipos], 4);
            ipos += 4;
        }

        if (ap) {
            rc = dap_ap_access(s, read, a2, &data);
        } else {
            rc = dap_dp_access(s, read, a2, &data);
        }

        if (read && vmatch) {
            /* value match: compare against match value */
            if (ipos + 4 <= DAP_PACKET_SIZE) {
                memcpy(&match_val, &in[ipos], 4);
                ipos += 4;
            }
            if (mmask && ipos + 4 <= DAP_PACKET_SIZE) {
                memcpy(&match_mask, &in[ipos], 4);
                ipos += 4;
            }
            if ((data & (mmask ? match_mask : 0xffffffff)) != match_val) {
                rc |= 0x08; /* value mismatch */
            }
        }

        if (read && opos + 4 <= DAP_PACKET_SIZE) {
            memcpy(&out[opos], &data, 4);
            opos += 4;
        }

        done++;
        resp_code = rc;
        if (rc != 0x01) {
            break;  /* stop on fault/wait */
        }
    }

    out[1] = done;
    out[2] = resp_code;
    dap_respond(s, out, DAP_PACKET_SIZE);
}

static void dap_cmd_transfer_block(S32K3DapState *s, const uint8_t *in)
{
    uint8_t out[DAP_PACKET_SIZE] = {0};
    uint16_t count = in[2] | (in[3] << 8);
    uint8_t req = in[4];
    bool read = req & 0x01;
    bool ap = req & 0x02;
    uint8_t a2 = (req >> 2) & 3;
    uint32_t ipos = 5, opos = 4;
    uint16_t done = 0;
    uint8_t resp_code = 0x01;
    uint16_t max = read ? (DAP_PACKET_SIZE - 4) / 4 :
                          (DAP_PACKET_SIZE - 5) / 4;
    uint32_t i;

    if (count > max) {
        count = max;
    }

    out[0] = DAP_CMD_TRANSFER_BLOCK;

    for (i = 0; i < count; i++) {
        uint32_t data = 0;
        uint8_t rc;

        if (!read) {
            memcpy(&data, &in[ipos], 4);
            ipos += 4;
        }
        rc = ap ? dap_ap_access(s, read, a2, &data)
                : dap_dp_access(s, read, a2, &data);
        if (read) {
            memcpy(&out[opos], &data, 4);
            opos += 4;
        }
        done++;
        resp_code = rc;
        if (rc != 0x01) {
            break;
        }
    }

    out[1] = done & 0xff;
    out[2] = (done >> 8) & 0xff;
    out[3] = resp_code;
    dap_respond(s, out, DAP_PACKET_SIZE);
}

static void s32k3_dap_process(S32K3DapState *s, const uint8_t *in, int len)
{
    uint8_t out[DAP_PACKET_SIZE] = {0};

    if (len < 1) {
        return;
    }

    switch (in[0]) {
    case DAP_CMD_INFO:
        dap_cmd_info(s, in);
        break;
    case DAP_CMD_HOST_STATUS:
        out[0] = in[0];
        out[1] = 0x00;
        dap_respond(s, out, DAP_PACKET_SIZE);
        break;
    case DAP_CMD_CONNECT:
        s->connected = true;
        s->port_mode = 1;               /* SWD */
        s->ap_csw = 0x00000002;         /* 32-bit transfers */
        out[0] = in[0];
        out[1] = s->port_mode;
        dap_respond(s, out, DAP_PACKET_SIZE);
        break;
    case DAP_CMD_DISCONNECT:
        s->connected = false;
        out[0] = in[0];
        out[1] = 0x00;
        dap_respond(s, out, DAP_PACKET_SIZE);
        break;
    case DAP_CMD_TRANSFER_CFG:
    case DAP_CMD_TRANSFER_ABORT:
    case DAP_CMD_WRITE_ABORT:
    case DAP_CMD_DELAY:
    case DAP_CMD_SWJ_CLOCK:
    case DAP_CMD_SWJ_SEQUENCE:
    case DAP_CMD_SWD_CONFIGURE:
    case DAP_CMD_SWD_SEQINFO:
        if (in[0] == DAP_CMD_WRITE_ABORT && len >= 6) {
            s->abort_reg = ((uint32_t)in[2]) |
                           ((uint32_t)in[3] << 8) |
                           ((uint32_t)in[4] << 16) |
                           ((uint32_t)in[5] << 24);
        }
        if (in[0] == DAP_CMD_SWJ_CLOCK && len >= 5) {
            s->swj_clock = ((uint32_t)in[1]) |
                           ((uint32_t)in[2] << 8) |
                           ((uint32_t)in[3] << 16) |
                           ((uint32_t)in[4] << 24);
        }
        out[0] = in[0];
        out[1] = 0x00;
        dap_respond(s, out, DAP_PACKET_SIZE);
        break;
    case DAP_CMD_SWJ_PINS:
        out[0] = in[0];
        out[1] = (len > 1) ? in[1] : 0;  /* echo pin state */
        dap_respond(s, out, DAP_PACKET_SIZE);
        break;
    case DAP_CMD_RESET_TARGET:
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        out[0] = in[0];
        out[1] = 0x00;      /* execute ok */
        dap_respond(s, out, DAP_PACKET_SIZE);
        break;
    case DAP_CMD_TRANSFER:
        dap_cmd_transfer(s, in);
        break;
    case DAP_CMD_TRANSFER_BLOCK:
        dap_cmd_transfer_block(s, in);
        break;
    default:
        out[0] = 0xFF;      /* command not supported */
        dap_respond(s, out, DAP_PACKET_SIZE);
        break;
    }
}

/* ------------------------------------------------------------------ */
/* USB descriptors                                                     */
/* ------------------------------------------------------------------ */

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings desc_strings = {
    [STR_MANUFACTURER - 1] = "Moonshot",
    [STR_PRODUCT - 1]      = "S32K348EVB CMSIS-DAP",
    [STR_SERIALNUMBER - 1] = "S32K348-QEMU-0001",
};

static const USBDescIface desc_iface_dap = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 2,
    .bInterfaceClass               = USB_CLASS_HID,
    .bInterfaceSubClass            = 0,
    .bInterfaceProtocol            = 0,
    .ndesc                         = 1,
    .descs = (USBDescOther[]) {
        {
            /* HID class descriptor (bDescriptorType 0x21) */
            .data = (uint8_t[]) {
                0x09,           /* length */
                0x21,           /* HID descriptor */
                0x11, 0x01,     /* bcdHID 1.11 */
                0x00,           /* country code */
                0x01,           /* num descriptors */
                0x22,           /* report descriptor */
                0x21, 0x00,     /* report descriptor length = 33 */
            },
        },
    },
    .eps = (USBDescEndpoint[]) {
        {
            .bEndpointAddress      = USB_DIR_IN | 0x01,
            .bmAttributes          = USB_ENDPOINT_XFER_INT,
            .wMaxPacketSize        = DAP_PACKET_SIZE,
            .bInterval             = 1,
        },
        {
            .bEndpointAddress      = USB_DIR_OUT | 0x01,
            .bmAttributes          = USB_ENDPOINT_XFER_INT,
            .wMaxPacketSize        = DAP_PACKET_SIZE,
            .bInterval             = 1,
        },
    },
};

static const USBDescDevice desc_device_dap = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE | USB_CFG_ATT_SELFPOWER,
            .bMaxPower             = 50,
            .nif = 1,
            .ifs = (USBDescIface[]) { desc_iface_dap },
        },
    },
};

static const USBDesc desc_dap = {
    .id = {
        .idVendor          = 0xC251,   /* Keil Software (CMSIS-DAP ref) */
        .idProduct         = 0xF000,   /* CMSIS-DAP */
        .bcdDevice         = 0x0100,
        .iManufacturer     = STR_MANUFACTURER,
        .iProduct          = STR_PRODUCT,
        .iSerialNumber     = STR_SERIALNUMBER,
    },
    .full  = &desc_device_dap,
    .high  = &desc_device_dap,
    .str   = desc_strings,
};

/* HID report descriptor: vendor-defined 64-byte input/output reports */
static const uint8_t dap_report_descriptor[] = {
    0x06, 0x00, 0xFF,       /* Usage Page (Vendor Defined 0xFF00) */
    0x09, 0x01,             /* Usage (0x01) */
    0xA1, 0x01,             /* Collection (Application) */
    0x15, 0x00,             /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00,       /*   Logical Maximum (255) */
    0x75, 0x08,             /*   Report Size (8) */
    0x95, DAP_PACKET_SIZE,  /*   Report Count (64) */
    0x09, 0x01,             /*   Usage (0x01) */
    0x81, 0x02,             /*   Input (Data,Var,Abs) */
    0x95, DAP_PACKET_SIZE,  /*   Report Count (64) */
    0x09, 0x01,             /*   Usage (0x01) */
    0x91, 0x02,             /*   Output (Data,Var,Abs) */
    0xC0,                   /* End Collection */
};

/* ------------------------------------------------------------------ */
/* USB request handling                                                */
/* ------------------------------------------------------------------ */

#define HID_GET_REPORT   0x01
#define HID_GET_IDLE     0x02
#define HID_GET_PROTOCOL 0x03
#define HID_SET_REPORT   0x09
#define HID_SET_IDLE     0x0A
#define HID_SET_PROTOCOL 0x0B

#define USB_DT_HID       0x21
#define USB_DT_REPORT    0x22

static void s32k3_dap_handle_reset(USBDevice *dev)
{
    S32K3DapState *s = S32K3_DAP(dev);

    s->resp_pending = false;
    s->resp_len = 0;
    s->connected = false;
    s->dpidr = DPIDR_VALUE;
    s->ctrl_stat = 0;
    s->select = 0;
    s->ap_csw = 0x00000002;
    s->ap_tar = 0;
    s->last_ap_read = 0;
}

static void s32k3_dap_handle_control(USBDevice *dev, USBPacket *p,
                                     int request, int value, int index,
                                     int length, uint8_t *data)
{
    S32K3DapState *s = S32K3_DAP(dev);
    int ret;

    /* standard descriptors (device/config/string) handled by core */
    ret = usb_desc_handle_control(dev, p, request, value, index, length, data);
    if (ret >= 0) {
        return;
    }

    switch (request) {
    case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
        if ((value >> 8) == USB_DT_REPORT) {
            int n = MIN(length, (int)sizeof(dap_report_descriptor));
            memcpy(data, dap_report_descriptor, n);
            p->actual_length = n;
            return;
        }
        if ((value >> 8) == USB_DT_HID) {
            int n = MIN(length, 9);
            data[0] = 0x09;
            data[1] = USB_DT_HID;
            data[2] = 0x11; data[3] = 0x01;
            data[4] = 0x00;
            data[5] = 0x01;
            data[6] = USB_DT_REPORT;
            data[7] = sizeof(dap_report_descriptor) & 0xff;
            data[8] = sizeof(dap_report_descriptor) >> 8;
            p->actual_length = n;
            return;
        }
        break;

    /* HID class requests */
    case ClassInterfaceRequest | HID_GET_REPORT:
        if (s->resp_pending) {
            int n = MIN(length, s->resp_len);
            memcpy(data, s->resp, n);
            s->resp_pending = false;
            p->actual_length = n;
        } else {
            memset(data, 0, MIN(length, DAP_PACKET_SIZE));
            p->actual_length = MIN(length, DAP_PACKET_SIZE);
        }
        return;
    case ClassInterfaceOutRequest | HID_SET_REPORT:
        s32k3_dap_process(s, data, length);
        p->actual_length = length;
        return;
    case ClassInterfaceOutRequest | HID_SET_IDLE:
    case ClassInterfaceOutRequest | HID_SET_PROTOCOL:
        p->actual_length = 0;
        return;
    case ClassInterfaceRequest | HID_GET_IDLE:
    case ClassInterfaceRequest | HID_GET_PROTOCOL:
        data[0] = 0;
        p->actual_length = 1;
        return;
    default:
        break;
    }

    p->status = USB_RET_STALL;
}

static void s32k3_dap_handle_data(USBDevice *dev, USBPacket *p)
{
    S32K3DapState *s = S32K3_DAP(dev);
    uint8_t buf[DAP_PACKET_SIZE];

    switch (p->pid) {
    case USB_TOKEN_OUT:
        if (p->ep->nr != 1) {
            p->status = USB_RET_STALL;
            return;
        }
        if (p->iov.size > DAP_PACKET_SIZE) {
            p->status = USB_RET_STALL;
            return;
        }
        memset(buf, 0, sizeof(buf));
        usb_packet_copy(p, buf, p->iov.size);
        s32k3_dap_process(s, buf, p->iov.size);
        p->actual_length = p->iov.size;
        break;
    case USB_TOKEN_IN:
        if (p->ep->nr != 1) {
            p->status = USB_RET_STALL;
            return;
        }
        if (!s->resp_pending) {
            p->status = USB_RET_NAK;
            return;
        }
        usb_packet_addbuf(p, s->resp, s->resp_len);
        p->actual_length = s->resp_len;
        s->resp_pending = false;
        break;
    default:
        p->status = USB_RET_STALL;
        break;
    }
}

static void s32k3_dap_realize(USBDevice *dev, Error **errp)
{
    S32K3DapState *s = S32K3_DAP(dev);

    if (!s->mem) {
        /* default: whole system memory of the machine */
        s->as = &address_space_memory;
    } else {
        memory_region_ref(s->mem);
        address_space_init(&s->own_as, s->mem, "s32k3-dap");
        s->as = &s->own_as;
    }

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    s32k3_dap_handle_reset(dev);
}

static const Property s32k3_dap_properties[] = {
    DEFINE_PROP_LINK("mem", S32K3DapState, mem,
                     TYPE_MEMORY_REGION, MemoryRegion *),
};

static void s32k3_dap_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

    uc->product_desc   = "S32K348EVB CMSIS-DAP debug probe";
    uc->usb_desc       = &desc_dap;
    uc->realize        = s32k3_dap_realize;
    uc->handle_reset   = s32k3_dap_handle_reset;
    uc->handle_control = s32k3_dap_handle_control;
    uc->handle_data    = s32k3_dap_handle_data;

    device_class_set_props(dc, s32k3_dap_properties);
    dc->desc = "S32K348EVB CMSIS-DAP debug probe (USB HID)";
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static const TypeInfo s32k3_dap_types[] = {
    {
        .name          = TYPE_S32K3_DAP,
        .parent        = TYPE_USB_DEVICE,
        .instance_size = sizeof(S32K3DapState),
        .class_init    = s32k3_dap_class_init,
    },
};

DEFINE_TYPES(s32k3_dap_types)
