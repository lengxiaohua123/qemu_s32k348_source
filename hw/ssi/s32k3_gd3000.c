/*
 * S32K3 GD3000 (MC33937) pre-driver SPI slave model
 *
 * 用于 S32K344_example BLDC 工程：TPP SDK 通过 LPSPI1 配置 MC34GD3000
 * 并读回状态寄存器（SR0-SR3）。模型按 MC33937 SPI 命令解析：
 *
 *   bit7-5 命令码：000 NULL 读 / 010 MASK0 / 011 MASK1 / 100 MODE /
 *                  110 CLINT0 / 111 CLINT1 / 1000 DEADTIME
 *   bit4-0 数据或寄存器地址
 *
 * 读时序（MC33937 延迟一拍）：NULL+addr 命令后，下一次 SPI 传输返回
 * 该寄存器内容。RTD TPP_GetStatusRegister 正是利用这一点：
 *   发 NULL+SR2（准备）-> 发 NULL+SR0（返回 SR2 内容）。
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/ssi/ssi.h"

#define TYPE_S32K3_GD3000 "s32k3-gd3000"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3Gd3000State, S32K3_GD3000)

/* 命令码 */
#define GD_CMD_NULL     0x00
#define GD_CMD_MASK0    0x20
#define GD_CMD_MASK1    0x30
#define GD_CMD_MODE     0x40
#define GD_CMD_CLINT0   0x60
#define GD_CMD_CLINT1   0x70
#define GD_CMD_DEADTIME 0x80

struct S32K3Gd3000State {
    SSIPeripheral parent_obj;

    uint8_t sr[4];     /* SR0 事件 / SR1 设置 / SR2 中断设置 / SR3 deadtime */
    uint8_t pending;   /* 最近 NULL 读命令请求的寄存器地址 */
};

static uint32_t s32k3_gd3000_transfer(SSIPeripheral *ssp, uint32_t val)
{
    S32K3Gd3000State *s = S32K3_GD3000(ssp);
    uint8_t cmd = val & 0xFF;
    uint8_t ret = s->sr[s->pending];

    if (cmd & 0x80) {                    /* DEADTIME 0x80-0xBF */
        s->sr[3] = cmd & 0x0F;
    } else if ((cmd & 0x70) == 0x70) {   /* CLINT1 0x70-0x7F */
        s->sr[0] = 0;
    } else if ((cmd & 0x70) == 0x60) {   /* CLINT0 0x60-0x6F */
        s->sr[0] = 0;
    } else if ((cmd & 0x70) == 0x40) {   /* MODE 0x40-0x5F */
        s->sr[1] = (cmd & 0x03) | ((cmd & 0x08) << 3);
    } else if ((cmd & 0x30) == 0x30) {   /* MASK1 0x30-0x3F */
        s->sr[2] = (s->sr[2] & 0x0F) | ((cmd & 0x0F) << 4);
    } else if ((cmd & 0x20) == 0x20) {   /* MASK0 0x20-0x2F */
        s->sr[2] = (s->sr[2] & 0xF0) | (cmd & 0x0F);
    } else {                             /* NULL 读 0x00-0x1F */
        s->pending = cmd & 0x07;
    }

    return ret;
}

static void s32k3_gd3000_realize(SSIPeripheral *ssp, Error **errp)
{
    S32K3Gd3000State *s = S32K3_GD3000(ssp);

    memset(s->sr, 0, sizeof(s->sr));
    s->pending = 0;
}

static void s32k3_gd3000_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = s32k3_gd3000_realize;
    k->transfer = s32k3_gd3000_transfer;
    k->cs_polarity = SSI_CS_LOW;
    dc->desc = "NXP MC34GD3000 pre-driver SPI slave";
}

static const TypeInfo s32k3_gd3000_info = {
    .name = TYPE_S32K3_GD3000,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(S32K3Gd3000State),
    .class_init = s32k3_gd3000_class_init,
};

static void s32k3_gd3000_register_types(void)
{
    type_register_static(&s32k3_gd3000_info);
}

type_init(s32k3_gd3000_register_types)
