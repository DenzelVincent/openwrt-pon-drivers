/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _AIROHA_XPON_GPON_REGS_H_
#define _AIROHA_XPON_GPON_REGS_H_

#include <linux/bits.h>

/*
 * AN7581 GPON MAC offsets within the dedicated 0x1fb64000 window.
 *
 * 窗口基准：DT "gpon" 资源就是 0x1fb64000/0x3e8（an7581.dtsi），devm_ioremap
 * 后 gpon_base 即指 0x1fb64000。原厂 xpon.ko 把整片 MAC ioremap 到 0x1fb60000，
 * 其表里的 GPON 偏移是 0x4000..0x43e4；本文件一律换算成窗口内偏移
 * （原厂偏移 - 0x4000），物理地址 = 0x1fb64000 + 本文件偏移。
 *
 * 每个偏移都同时满足两条证据：原厂 xpon.ko 的 GPON 寄存器表里有命名条目
 * （gponDevDumpCsr 等），且存在对应的原厂访问器函数；位域只收反汇编能读到
 * 的 mask/shift，未验证的位一律不写名字（见计划文档 4.3 "未验位域不许猜"）。
 */

#define AIROHA_GPON_ONU_ID            0x0000
#define AIROHA_GPON_ONU_GLOBAL_CONFIG 0x0004
#define AIROHA_GPON_INT_STATUS        0x0008
#define AIROHA_GPON_INT_ENABLE        0x000c
/* 8 个寄存器、每个 2 个 12-bit Alloc-ID 字段：[11:0] 与 [27:16]。 */
#define AIROHA_GPON_TCONT_ALLOC_BASE 0x0020
#define AIROHA_GPON_TCONT_ALLOC_REGS 8
#define AIROHA_GPON_PLOAMU_FIFO_STS  0x0050
#define AIROHA_GPON_PLOAMU_WDATA     0x0054
#define AIROHA_GPON_PLOAMD_FIFO_STS  0x0058
#define AIROHA_GPON_PLOAMD_RDATA     0x005c
#define AIROHA_GPON_EQD              0x00a8
#define AIROHA_GPON_ACTIVATION_PARAM 0x00ac
#define AIROHA_GPON_VENDOR_ID        0x00b0
#define AIROHA_GPON_VS_SN            0x00b4
#define AIROHA_GPON_ACTIVATION_ST    0x00bc
#define AIROHA_GPON_MBI_MPI_STOP     0x0160
#define AIROHA_GPON_DBG_SIG_GROUP0   0x0228
#define AIROHA_GPON_DBG_SIG_GROUP1   0x022c
#define AIROHA_GPON_RX_GEM_CNT       0x0300
#define AIROHA_GPON_RX_ETH_CRC_CNT   0x0304
#define AIROHA_GPON_RX_GTC_CNT       0x0308
#define AIROHA_GPON_TX_GEM_CNT       0x030c
#define AIROHA_GPON_TX_BST_CNT       0x0310
#define AIROHA_GPON_RX_GEM_HDR_1BIT  0x0330
#define AIROHA_GPON_RX_GEM_HDR_2BIT  0x0334
#define AIROHA_GPON_RX_GEM_HDR_UC    0x0338
#define AIROHA_GPON_DS_SF_CNT        0x0358
#define AIROHA_GPON_SW_RESYNC        0x0364
#define AIROHA_GPON_MAC_SET          0x03a0
#define AIROHA_GPON_O3O4_PLOAM_CTRL  0x03c4
/* gpon_dev_init() 关闭 sniff，避免正常接收被旁路。 */
#define AIROHA_GPON_SNIFF_DA_SA      0x0390
#define AIROHA_GPON_SNIFF_GEM_CNT    0x0398

/*
 * ONU-ID 低 8 bit + bit15 valid；gponDevDeactiveOnu 清 valid 并把低字节写成
 * 0xFF，因此失效值用 0xFF（不是 XG-PON 的 10-bit 0x3ff）。
 */
#define AIROHA_GPON_ONU_ID_MASK    GENMASK(7, 0)
#define AIROHA_GPON_ONU_ID_VALID   BIT(15)
#define AIROHA_GPON_ONU_ID_INVALID 0xff

/*
 * MBI（MAC↔GDM/CDM）与 MPI（接数字 PON PHY）的停口位。厂商 gponDevMbiStop
 * 把 bit0/bit8 同时置位或同时清零，gponDevMpiStop 同样成对操作 bit16/bit24，
 * 没有按收/发方向单独控制的接口。
 */
#define AIROHA_GPON_MBI_STOP   BIT(0)
#define AIROHA_GPON_MBI_TX_STOP BIT(8)
#define AIROHA_GPON_MPI_RX_STOP BIT(16)
#define AIROHA_GPON_MPI_TX_STOP BIT(24)
#define AIROHA_GPON_MBI_STOP_MASK \
	(AIROHA_GPON_MBI_STOP | AIROHA_GPON_MBI_TX_STOP)
#define AIROHA_GPON_MPI_STOP_MASK \
	(AIROHA_GPON_MPI_RX_STOP | AIROHA_GPON_MPI_TX_STOP)
/* 停止 MPI 后必须等 bit30 与 bit31 同时置位；MBI 没有 done 位。 */
#define AIROHA_GPON_MPI_STOP_DONE_MASK (BIT(30) | BIT(31))

/*
 * T-CONT Alloc-ID 字段是 12 bit，失效值 0xFF；gponDevResetAllocId 对 8 个寄存器
 * 都写 0x00FF00FF（低/高字段各 0xFF）。
 */
#define AIROHA_GPON_TCONT_ALLOC_LOW_MASK  GENMASK(11, 0)
#define AIROHA_GPON_TCONT_ALLOC_HIGH_MASK GENMASK(27, 16)
#define AIROHA_GPON_TCONT_ALLOC_INVALID   0x00ff00ff

/*
 * GPON_MAC_SET bit0：gponDevSwReset 先清零、延时 1us、再置位，作为 MAC 软复位
 * 脉冲；本驱动把"清零"当作复位保持、"置位"当作放行。
 */
#define AIROHA_GPON_MAC_SET_RELEASED BIT(0)

/*
 * SW_RESYNC：置 bit31 准备，1us 后置 bit0 触发，保持 1000us 再清 bit31。
 * GPON 没有 XG-PON 的 TX_SYNC_READY 回读位，厂商也只按时序收尾。
 */
#define AIROHA_GPON_SW_RESYNC_ARM      BIT(31)
#define AIROHA_GPON_SW_RESYNC_START    BIT(0)
#define AIROHA_GPON_SW_RESYNC_DELAY_US 1000

/* ONU 激活状态在 bit[2:0]，取值 1..7 对应 O1..O7。 */
#define AIROHA_GPON_ACTIVATION_STATE_MASK GENMASK(2, 0)

/* O3/O4 US PLOAM 控制：bit0=1 表示软件代答；bit8 为发 PLOAMu 等待模式。 */
#define AIROHA_GPON_O3O4_SOFTWARE_REPLY BIT(0)
#define AIROHA_GPON_O3O4_SEND_WAIT_MODE BIT(8)

#endif
