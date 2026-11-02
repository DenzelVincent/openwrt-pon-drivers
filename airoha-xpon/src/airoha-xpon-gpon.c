// SPDX-License-Identifier: GPL-2.0-only

/*
 * Airoha AN7581 GPON (ITU-T G.984) MAC implementation.
 *
 * AN7581 exposes three separate MAC windows: GPON (0x1fb64000), XG-PON
 * (0x1fb65000) and EPON (0x1fb66000). The GPON path must use the dedicated
 * "gpon" window for every MAC access: once the PCS selects GPON framing the
 * xgpon window reads back a constant 0xDEADBEEF on real hardware.
 *
 * 共享层（airoha-xpon-ploam.c / -xgpon.c / -crypto.c）仍是 XG-PON 实现，
 * 本文件不调用其中会写 XG-PON 窗口的函数；GPON 自己的 T-CONT/ONU-ID/
 * 错误清除都就地实现。协议栈（PLOAMd 解析、消息编号、Key 交换）留待
 * S3-2b/2c，本阶段只保证 MAC 块访问、GTC 同步判据和 O2_3 状态可达。
 *
 * Implementation is an independent, from-scratch driver based on ITU-T
 * G.984.3 (GTC layer) and G.984.4 (OMCI) provisions. Register semantics
 * are validated against the public EN7581 SDK header (xepon_mac_c_header)
 * and the vendor xpon.ko GPON register table; no proprietary code is copied
 * verbatim.
 */

#include <airoha-pon-frontend.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>

#include "airoha-xpon-private.h"
#include "airoha-xpon-omci.h"
#include "airoha-xpon-gpon.h"
#include "airoha-xpon-gpon-regs.h"

/*
 * Digital PHY RX mode: GPON selects a different demodulation/framing path
 * than XG-PON. The XG-PON driver uses bit[8] in RX_SYNC_CTRL; GPON uses
 * bit[9] as confirmed by the SDK rx_sync mode table.
 */
#define AIROHA_GPON_PHY_RX_MODE_GPON BIT(9)

/*
 * Synchronization state: GPON GTC hunt/pre-sync/in-sync/re-sync uses the
 * same 4-state machine as XG-PON, with identical values.
 */

/*
 * GPON shares the 1.25G GEPON PHY block with 1G-EPON (XEPON_1G pairs the
 * 10G MAC with that PHY block). PHYSET10 bit[31] selects the GPON framing;
 * the EPON 1G path explicitly clears it in airoha-xpon-epon.c, so the GPON
 * path has to set it before the PCS profile is programmed.
 */
#define AIROHA_GEPON_PHYSET10  0x124
#define AIROHA_GEPON_GPON_MODE BIT(31)

/* 计数器表里"本制式没有该计数器"的哨兵；GPON 没有 PLOAMd/PLOAMu 计数器。 */
#define AIROHA_GPON_COUNTER_NONE 0

/*
 * 软件侧对齐共享 airoha_xpon_clear_assignment()，但不能直接调用它：它写
 * XG-PON ONU_ID/INT_ENABLE 并通过 reset_data_key_exchange 写 XG-PON AES 槽，
 * 这些地址在 GPON 制式下都属于死块。数据密钥只有在 GPON PLOAM 栈（S3-2c）
 * 落地后才可能生效，本阶段没有硬件密钥需要撤销。
 */
static void airoha_xpon_gpon_clear_assignment(struct airoha_xpon *xpon)
{
	airoha_xpon_omci_set_link(xpon->omci, false,
	                          AIROHA_GPON_ONU_ID_INVALID, 0);
	/* 对齐 gponDevDeactiveOnu：清 bit15 并把低字节写成 0xFF。 */
	airoha_gpon_update_bits(xpon, AIROHA_GPON_ONU_ID,
	                        AIROHA_GPON_ONU_ID_VALID |
	                                AIROHA_GPON_ONU_ID_MASK,
	                        AIROHA_GPON_ONU_ID_INVALID);
	WRITE_ONCE(xpon->tx_armed, false);
	if (xpon->pon_netdev) {
		airoha_xpon_set_data_path_link(xpon, false);
		airoha_eth_pon_configure(xpon->pon_netdev, 0, 0, false);
	}
	xpon->onu_id = AIROHA_GPON_ONU_ID_INVALID;
	xpon->eqd = 0;
	xpon->service_ready = false;
	xpon->data_path.count = 0;
	WRITE_ONCE(xpon->data_path.pending_count, 0);
	atomic_set(&xpon->data_path_retry_pending, 0);
	xpon->data_path.configured = false;
}

/* gponDevResetAllocId 的等价操作：8 个寄存器全部写回无效值。 */
static void airoha_xpon_gpon_reset_alloc_ids(struct airoha_xpon *xpon)
{
	unsigned int i;

	for (i = 0; i < AIROHA_GPON_TCONT_ALLOC_REGS; i++)
		airoha_gpon_write(xpon, AIROHA_GPON_TCONT_ALLOC_BASE + i * 4,
		                  AIROHA_GPON_TCONT_ALLOC_INVALID);
}

/*
 * 0x4228/0x422c 是厂商 gponDevSetErrStatusClear 的"写全 1 清除"口。
 * 具体错误位域尚未抽（S4 第 3 步欠账），这里只做整组清除，不解释位含义。
 */
static void airoha_xpon_gpon_clear_errors(struct airoha_xpon *xpon,
                                          bool clear_software_latch)
{
	airoha_gpon_write(xpon, AIROHA_GPON_DBG_SIG_GROUP0, U32_MAX);
	airoha_gpon_write(xpon, AIROHA_GPON_DBG_SIG_GROUP1, U32_MAX);

	if (clear_software_latch) {
		WRITE_ONCE(xpon->stats.latched_fifo_error, 0);
		WRITE_ONCE(xpon->stats.latched_tx_error, 0);
		WRITE_ONCE(xpon->stats.latched_rx_error, 0);
		WRITE_ONCE(xpon->stats.latched_bwm_check, 0);
	}
}

/*
 * GPON 的激活状态在 0x40bc bit[2:0]，取值与 XG-PON 的 O1..O7 编码一致；
 * 写完后回读一行，区分"驱动自己记到了 O2_3"和"硬件真的进到 O2_3"。
 */
static void airoha_xpon_gpon_set_onu_state(struct airoha_xpon *xpon,
                                           enum airoha_xgpon_onu_state state)
{
	enum airoha_xgpon_onu_state prev = xpon->onu_state;
	u32 activation;

	airoha_gpon_update_bits(xpon, AIROHA_GPON_ACTIVATION_ST,
	                        AIROHA_GPON_ACTIVATION_STATE_MASK,
	                        FIELD_PREP(AIROHA_GPON_ACTIVATION_STATE_MASK,
	                                   state));
	xpon->onu_state = state;
	airoha_xpon_leds_update(xpon);

	activation = airoha_gpon_read(xpon, AIROHA_GPON_ACTIVATION_ST);
	airoha_xpon_trace(xpon, AIROHA_XPON_TRACE_SEG_PLOAM, "onu-state",
			  "state=%s prev=%s onu_id=%u eqd=%u activation=0x%08x\n",
			  airoha_xpon_onu_state_name(state),
			  airoha_xpon_onu_state_name(prev), xpon->onu_id,
			  xpon->eqd, activation);
}

static void airoha_xpon_gpon_release_mac_interfaces(struct airoha_xpon *xpon)
{
	/* 与厂商顺序一致：先放 MPI、再放 MBI（gponDevMpiStop(0)/gponDevMbiStop(0)）。 */
	airoha_gpon_update_bits(xpon, AIROHA_GPON_MBI_MPI_STOP,
	                        AIROHA_GPON_MPI_STOP_MASK, 0);
	airoha_gpon_update_bits(xpon, AIROHA_GPON_MBI_MPI_STOP,
	                        AIROHA_GPON_MBI_STOP_MASK, 0);
}

static void airoha_xpon_gpon_stop_mac_interfaces(struct airoha_xpon *xpon,
                                                 const char *reason)
{
	u32 value;
	int ret;

	/*
	 * MBI 没有 done 位：厂商在置位后固定延时 1us，再等 GDM/CDM 停口
	 * 50us。本驱动没有 feDevGdm2Cdm2Stop 钩子，用同样的固定延时覆盖。
	 */
	airoha_gpon_update_bits(xpon, AIROHA_GPON_MBI_MPI_STOP,
	                        AIROHA_GPON_MBI_STOP_MASK,
	                        AIROHA_GPON_MBI_STOP_MASK);
	udelay(1);
	udelay(50);

	/* MPI 停口必须等 bit30 与 bit31 同时置位。 */
	airoha_gpon_update_bits(xpon, AIROHA_GPON_MBI_MPI_STOP,
	                        AIROHA_GPON_MPI_STOP_MASK,
	                        AIROHA_GPON_MPI_STOP_MASK);
	ret = readl_poll_timeout_atomic(
		xpon->gpon_base + AIROHA_GPON_MBI_MPI_STOP, value,
		(value & AIROHA_GPON_MPI_STOP_DONE_MASK) ==
			AIROHA_GPON_MPI_STOP_DONE_MASK,
		1, 3000);
	if (ret)
		dev_warn_ratelimited(
			xpon->dev,
			"timed out stopping GPON MPI (%s), raw=0x%08x\n",
			reason, value);
}

static void airoha_xpon_gpon_program_identity(struct airoha_xpon *xpon)
{
	const struct airoha_xpon_identity *identity =
		&xpon->identity.active_identity;

	/*
	 * GPON 身份只有 8 字节 SN：0x40b0 放 SN[0..3]、0x40b4 放 SN[4..7]，
	 * 各按大端拼成 u32（与 gponDevSetSerialNumber 一致）。GPON 没有
	 * Registration-ID，XG-PON 的 RGS_ID 寄存器组不在这里翻译。
	 */
	if (identity->serial_configured) {
		airoha_gpon_write(xpon, AIROHA_GPON_VENDOR_ID,
		                  airoha_xpon_bytes_to_u32(identity->serial_number));
		airoha_gpon_write(xpon, AIROHA_GPON_VS_SN,
		                  airoha_xpon_bytes_to_u32(identity->serial_number + 4));
	}
}

static int airoha_xpon_gpon_resync_tx_locked(struct airoha_xpon *xpon)
{
	u32 value;
	int ret;

	lockdep_assert_held(&xpon->state_lock);
	mutex_lock(&xpon->ploam_lock);

	/*
	 * GPON 没有按收/发分开的停口接口，也没有 XG-PON 的 US_PROF_VLD /
	 * TX_SYNC_READY 回读；停 MBI/MPI 后直接走厂商的 SW_RESYNC 时序：
	 * bit31 准备、bit0 触发、保持 1ms 后撤 bit31。
	 */
	airoha_gpon_update_bits(xpon, AIROHA_GPON_MBI_MPI_STOP,
	                        AIROHA_GPON_MBI_STOP_MASK,
	                        AIROHA_GPON_MBI_STOP_MASK);
	udelay(1);
	udelay(50);

	airoha_gpon_update_bits(xpon, AIROHA_GPON_MBI_MPI_STOP,
	                        AIROHA_GPON_MPI_STOP_MASK,
	                        AIROHA_GPON_MPI_STOP_MASK);
	ret = readl_poll_timeout_atomic(
		xpon->gpon_base + AIROHA_GPON_MBI_MPI_STOP, value,
		(value & AIROHA_GPON_MPI_STOP_DONE_MASK) ==
			AIROHA_GPON_MPI_STOP_DONE_MASK,
		1, 3000);
	if (ret)
		goto out_release;

	airoha_gpon_write(xpon, AIROHA_GPON_EQD, xpon->eqd);
	airoha_gpon_update_bits(xpon, AIROHA_GPON_SW_RESYNC,
	                        AIROHA_GPON_SW_RESYNC_ARM,
	                        AIROHA_GPON_SW_RESYNC_ARM);
	udelay(1);
	airoha_gpon_update_bits(xpon, AIROHA_GPON_SW_RESYNC,
	                        AIROHA_GPON_SW_RESYNC_START,
	                        AIROHA_GPON_SW_RESYNC_START);
	udelay(AIROHA_GPON_SW_RESYNC_DELAY_US);
	airoha_gpon_update_bits(xpon, AIROHA_GPON_SW_RESYNC,
	                        AIROHA_GPON_SW_RESYNC_ARM, 0);

out_release:
	/* 与厂商 gponDevMpiStop(0) / gponDevMbiStop(0) 的顺序一致：先 MPI 后 MBI。 */
	airoha_gpon_update_bits(xpon, AIROHA_GPON_MBI_MPI_STOP,
	                        AIROHA_GPON_MPI_STOP_MASK, 0);
	airoha_gpon_update_bits(xpon, AIROHA_GPON_MBI_MPI_STOP,
	                        AIROHA_GPON_MBI_STOP_MASK, 0);
	mutex_unlock(&xpon->ploam_lock);

	return ret;
}

static int airoha_xpon_gpon_enable_digital_rx(struct airoha_xpon *xpon)
{
	u32 rx_ctrl;

	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_SFP_VLD_LEVEL,
	                     xpon->frontend_link.xgpon_sfp_valid_level);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_INT_ENABLE, 0);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_INT_STATUS,
	                     AIROHA_XGPON_PHY_INT_RX_MASK);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_INT_ENABLE,
	                     AIROHA_XGPON_PHY_INT_RX_MASK);

	rx_ctrl = airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_RX_SYNC_CTRL);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_RX_SYNC_CTRL,
	                     (rx_ctrl & ~AIROHA_XGPON_PHY_RX_MODE_MASK) |
	                             AIROHA_GPON_PHY_RX_MODE_GPON |
	                             AIROHA_XGPON_PHY_RX_ENABLE);
	rx_ctrl = airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_RX_SYNC_CTRL);

	return (rx_ctrl &
	        (AIROHA_XGPON_PHY_RX_MODE_MASK | AIROHA_XGPON_PHY_RX_ENABLE)) ==
	                       (AIROHA_GPON_PHY_RX_MODE_GPON |
	                        AIROHA_XGPON_PHY_RX_ENABLE) ?
	               0 :
	               -EIO;
}

static void airoha_xpon_gpon_hold_digital_rx_reset(struct airoha_xpon *xpon)
{
	u32 rx_ctrl;

	rx_ctrl = airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_RESET_CTRL);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_RESET_CTRL,
	                     rx_ctrl & ~AIROHA_XGPON_PHY_RESET_MASK);
}

static int airoha_xpon_gpon_release_digital_rx_reset(struct airoha_xpon *xpon)
{
	u32 rx_ctrl;

	rx_ctrl = airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_RESET_CTRL);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_RESET_CTRL,
	                     rx_ctrl | AIROHA_XGPON_PHY_RESET_RELEASED);
	usleep_range(1000, 1200);
	rx_ctrl = airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_RESET_CTRL);
	if ((rx_ctrl & AIROHA_XGPON_PHY_RESET_MASK) !=
	    AIROHA_XGPON_PHY_RESET_RELEASED)
		return -EIO;

	return 0;
}

static int airoha_xpon_gpon_reset_and_enable_digital_rx(struct airoha_xpon *xpon)
{
	int ret;

	airoha_xpon_gpon_hold_digital_rx_reset(xpon);
	usleep_range(1000, 1200);
	ret = airoha_xpon_gpon_release_digital_rx_reset(xpon);
	if (ret)
		return ret;

	return airoha_xpon_gpon_enable_digital_rx(xpon);
}

static int airoha_xpon_gpon_start_control(struct airoha_xpon *xpon)
{
	if (READ_ONCE(xpon->stopping))
		return -ESHUTDOWN;

	/*
	 * GTC 同步后放行 MAC 复位。
	 *
	 * GPON 没有 XG-PON 的 US_PROF_VLD 和 TX-late 自动重同步位；RSP_TIME
	 * （0x40ac ONU activation parameter）的取值来自板级校准，本阶段不写。
	 * MAC 中断布局只验证了 bit1(PLOAMu)/bit10(1PPS)/bit30(BWmap stop)，
	 * 没有 PLOAMd 收包位，因此 INT_ENABLE 保持 0：第 5 段判据由 250ms
	 * 轮询完成，第 6 段要等 GPON PLOAM 栈（S3-2c）再启用中断。
	 */
	airoha_gpon_write(xpon, AIROHA_GPON_INT_ENABLE, 0);
	airoha_gpon_update_bits(xpon, AIROHA_GPON_MAC_SET,
	                        AIROHA_GPON_MAC_SET_RELEASED,
	                        AIROHA_GPON_MAC_SET_RELEASED);
	airoha_xpon_gpon_counters_start(xpon);
	airoha_xpon_gpon_release_mac_interfaces(xpon);
	/* 0 = 硬件代答 O3/O4（与厂商默认一致）；软件代答要等 PLOAM 栈。 */
	airoha_gpon_update_bits(xpon, AIROHA_GPON_O3O4_PLOAM_CTRL,
	                        AIROHA_GPON_O3O4_SOFTWARE_REPLY, 0);
	airoha_pon_phy_update_bits(xpon, AIROHA_XGPON_PHY_XG_CONTINUE_CTRL,
	                           AIROHA_XGPON_PHY_CONTINUE_MASK, 0);
	airoha_xpon_gpon_program_identity(xpon);

	xpon->crypto.default_ploam_ik_programmed = false;
	xpon->crypto.activation_keys_programmed = false;
	xpon->crypto.o4_key_switch_observed = false;
	xpon->crypto.generation_runs = 0;
	xpon->crypto.generation_failed_type = 0;
	xpon->crypto.key_control_rx_count = 0;
	xpon->crypto.key_report_tx_count = 0;
	xpon->crypto.key_exchange_completed_count = 0;
	xpon->crypto.key_exchange_error_count = 0;
	airoha_xpon_gpon_clear_assignment(xpon);
	airoha_xpon_gpon_reset_alloc_ids(xpon);
	airoha_xpon_gpon_set_onu_state(xpon, AIROHA_XGPON_O2_3);
	airoha_xpon_gpon_clear_errors(xpon, true);
	/* 清掉复位前残留的中断状态；厂商 gpon_set_ploamu_int 也是写全 1。 */
	airoha_gpon_write(xpon, AIROHA_GPON_INT_STATUS, U32_MAX);

	xpon->xgpon_irq_enabled = false;
	xpon->mac_initialized = true;
	xpon->lifecycle = AIROHA_XPON_PROTOCOL_ACTIVATING;

	dev_info(
		xpon->dev,
		"GTC in sync; MAC entered O2_3, SN=%s, BEN/upstream TX remains disabled\n",
		xpon->identity.active_identity.serial_configured ?
			"configured" :
			"missing");

	return 0;
}

irqreturn_t airoha_xpon_gpon_phy_irq(struct airoha_xpon *xpon)
{
	u32 events, status, sync;

	status = airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_INT_STATUS);
	events = status & AIROHA_XGPON_PHY_INT_RX_MASK;
	if (!events)
		return IRQ_NONE;

	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_INT_STATUS, events);
	if (READ_ONCE(xpon->stopping))
		return IRQ_HANDLED;

	/* Disambiguate SYNC_OK vs. LOF coexistence per current GTC sync state. */
	if ((events &
	     (AIROHA_XGPON_PHY_INT_RX_SYNC_OK | AIROHA_XGPON_PHY_INT_RX_LOF)) ==
	    (AIROHA_XGPON_PHY_INT_RX_SYNC_OK | AIROHA_XGPON_PHY_INT_RX_LOF)) {
		sync = airoha_pon_phy_read(xpon,
		                           AIROHA_XGPON_PHY_DBG_RX_SYNC_ST);
		if (FIELD_GET(AIROHA_XGPON_PHY_RX_SYNC_MASK, sync) ==
		    AIROHA_XGPON_PHY_RX_SYNC_IN)
			events &= ~AIROHA_XGPON_PHY_INT_RX_LOF;
		else
			events &= ~AIROHA_XGPON_PHY_INT_RX_SYNC_OK;
	}

	atomic_or(events, &xpon->phy_events);
	xpon->stats.phy_irq_count++;
	if (events & AIROHA_XGPON_PHY_INT_RX_RDY)
		xpon->stats.phy_rx_ready_count++;
	if (events & AIROHA_XGPON_PHY_INT_RX_SYNC_OK)
		xpon->stats.phy_sync_count++;
	if (events & AIROHA_XGPON_PHY_INT_RX_LOS)
		xpon->stats.phy_los_count++;
	if (events & AIROHA_XGPON_PHY_INT_RX_LOF)
		xpon->stats.phy_lof_count++;

	if (events &
	    (AIROHA_XGPON_PHY_INT_RX_LOS | AIROHA_XGPON_PHY_INT_RX_RDY))
		mod_delayed_work(system_wq, &xpon->link_work, 0);
	else
		queue_delayed_work(system_wq, &xpon->link_work, 0);
	return IRQ_HANDLED;
}

irqreturn_t airoha_xpon_gpon_irq(struct airoha_xpon *xpon)
{
	u32 enabled, status;

	/*
	 * GPON 的 MAC 中断位只验证了 bit1(PLOAMu)/bit10(1PPS)/bit30(BWmap
	 * stop)，没有可用的 PLOAMd 收包位，本阶段 INT_ENABLE 恒为 0。
	 * 保留处理函数只为在误触发时清状态，不做任何 XG-PON 解码。
	 */
	if (!xpon->gpon_base)
		return IRQ_NONE;

	status = airoha_gpon_read(xpon, AIROHA_GPON_INT_STATUS);
	enabled = airoha_gpon_read(xpon, AIROHA_GPON_INT_ENABLE);
	if (!(status & enabled))
		return IRQ_NONE;

	airoha_gpon_write(xpon, AIROHA_GPON_INT_STATUS, U32_MAX);
	return IRQ_HANDLED;
}

void airoha_xpon_gpon_mask_irqs(struct airoha_xpon *xpon)
{
	if (xpon->gpon_base)
		airoha_gpon_write(xpon, AIROHA_GPON_INT_ENABLE, 0);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_INT_ENABLE, 0);
}

bool airoha_xpon_gpon_los(struct airoha_xpon *xpon)
{
	return !!(airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_SFP_STA) &
	          AIROHA_XGPON_PHY_SFP_RX_LOS);
}

int airoha_xpon_gpon_prepare(struct airoha_xpon *xpon)
{
	u32 onu_id_prev;

	if (!xpon->gpon_base)
		return dev_err_probe(xpon->dev, -ENODEV,
		                     "GPON MAC window is not mapped\n");

	/* Hold the GPON MAC in reset during PMA initialization. */
	airoha_gpon_write(xpon, AIROHA_GPON_INT_ENABLE, 0);
	airoha_xpon_gpon_stop_mac_interfaces(xpon, "prepare");
	airoha_gpon_update_bits(xpon, AIROHA_GPON_MAC_SET,
	                        AIROHA_GPON_MAC_SET_RELEASED, 0);
	/*
	 * MAC 复位不会清 ONU-ID；从上一轮（含 XG-PON）残留的 valid 身份会让
	 * 第 6 段误判 O2/O4。GPON 只有低 8 bit + bit15 valid，失效值写 0xFF。
	 */
	onu_id_prev = airoha_gpon_read(xpon, AIROHA_GPON_ONU_ID);
	airoha_gpon_update_bits(xpon, AIROHA_GPON_ONU_ID,
	                        AIROHA_GPON_ONU_ID_VALID |
	                                AIROHA_GPON_ONU_ID_MASK,
	                        AIROHA_GPON_ONU_ID_INVALID);
	xpon->onu_id = AIROHA_GPON_ONU_ID_INVALID;
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_SFP_VLD_LEVEL,
	                     xpon->frontend_link.xgpon_sfp_valid_level);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_INT_ENABLE, 0);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_INT_STATUS,
	                     AIROHA_XGPON_PHY_INT_RX_MASK);
	xpon->lifecycle = AIROHA_XPON_WAIT_OPTICAL_SIGNAL;
	/* 新的 MAC 周期：同步态取证位图重新计数。 */
	xpon->trace_sync_seen = 0;

	/*
	 * onu_id_prev 是清零前的硬件原值，也是本段唯一的"上一轮残留"证据：
	 * GPON ONU-ID 是 8 bit + bit15 valid，从 XGS 切过来时若 valid=1 且
	 * id 非失效值，第 6 段的认证会被旧身份带偏。
	 */
	airoha_xpon_trace(xpon, AIROHA_XPON_TRACE_SEG_MAC, "mac-prepared",
			  "onu_id_prev=0x%08x mac_set=0x%08x phy_int_en=0x%08x\n",
			  onu_id_prev,
			  airoha_gpon_read(xpon, AIROHA_GPON_MAC_SET),
			  airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_INT_ENABLE));

	return 0;
}

int airoha_xpon_gpon_start_rx(struct airoha_xpon *xpon)
{
	bool retrain;
	u32 onu_id;
	u32 physet10;
	int ret;

	/* Select GPON framing on the 1.25G GEPON PHY block. */
	airoha_pon_phy_update_bits(xpon, AIROHA_GEPON_PHYSET10,
	                           AIROHA_GEPON_GPON_MODE,
	                           AIROHA_GEPON_GPON_MODE);

	/*
	 * GPON 的制式落在 GEPON PHY 的 bit 上，而不是 MAC 侧；这一行回读是"真的切到
	 * GPON 帧格式"的直接证据，legacy bringup 缺校准时也靠它排除制式没切对。
	 */
	physet10 = airoha_pon_phy_read(xpon, AIROHA_GEPON_PHYSET10);
	airoha_xpon_trace(xpon, AIROHA_XPON_TRACE_SEG_MAC, "gpon-framing",
			  "physet10=0x%08x gpon_mode=%d\n", physet10,
			  !!(physet10 & AIROHA_GEPON_GPON_MODE));

	/* GPON uses the dedicated GPON PCS profile (2.488/1.244 Gbps) */
	retrain = xpon->pcs_profile_valid &&
	          xpon->pcs_profile == AIROHA_PCS_PON_MODE_GPON;
	airoha_xpon_trace(xpon, AIROHA_XPON_TRACE_SEG_MAC, "pcs-path",
			  "retrain=%d profile=%u profile_valid=%d setting0=0x%08x\n",
			  retrain, xpon->pcs_profile, xpon->pcs_profile_valid,
			  xpon->frontend_link.pma_xpon_setting0);
	if (!retrain) {
		ret = airoha_pcs_pon_config_rx(
			xpon->pcs, AIROHA_PCS_PON_MODE_GPON,
			xpon->frontend_link.pma_xpon_setting0,
			xpon->frontend_link.pma_xpon_setting1);
		if (!ret) {
			xpon->pcs_profile = AIROHA_PCS_PON_MODE_GPON;
			xpon->pcs_profile_valid = true;
		} else {
			airoha_xpon_fail(xpon, AIROHA_XPON_TRACE_SEG_ANA,
					 "pcs-config", ret, "profile=%d\n",
					 AIROHA_PCS_PON_MODE_GPON);
		}
	} else {
		usleep_range(2000, 2500);
		airoha_xpon_gpon_hold_digital_rx_reset(xpon);
		usleep_range(1000, 1200);
		ret = airoha_pcs_pon_retrain_rx(xpon->pcs);
		if (ret) {
			airoha_xpon_fail(xpon, AIROHA_XPON_TRACE_SEG_PMA,
					 "pcs-retrain", ret, "profile=%u\n",
					 xpon->pcs_profile);
			airoha_xpon_gpon_release_digital_rx_reset(xpon);
			return ret;
		}
		usleep_range(1000, 1200);
	}
	if (ret)
		return ret;
	xpon->lifecycle = AIROHA_XPON_PMA_CONFIGURED;

	if (retrain) {
		ret = airoha_xpon_gpon_release_digital_rx_reset(xpon);
		if (!ret)
			ret = airoha_xpon_gpon_enable_digital_rx(xpon);
		if (!ret)
			usleep_range(8000, 9000);
	} else {
		ret = airoha_xpon_gpon_reset_and_enable_digital_rx(xpon);
	}
	if (ret) {
		airoha_xpon_fail(xpon, AIROHA_XPON_TRACE_SEG_MAC, "digital-rx",
				 ret, "retrain=%d\n", retrain);
		return ret;
	}

	xpon->lifecycle = AIROHA_XPON_WAIT_LINE_SYNC;
	xpon->rx_active = true;
	xpon->stats.rx_start_count++;
	xpon->no_sync_since = jiffies;

	onu_id = airoha_gpon_read(xpon, AIROHA_GPON_ONU_ID);
	dev_info(
		xpon->dev,
		"GPON RX started, irq=%d, ONU-ID=%lu (%s), burst TX remains disabled\n",
		xpon->mac_irq, FIELD_GET(AIROHA_GPON_ONU_ID_MASK, onu_id),
		onu_id & AIROHA_GPON_ONU_ID_VALID ? "valid" : "invalid");
	airoha_xpon_trace(xpon, AIROHA_XPON_TRACE_SEG_MAC, "rx-started",
			  "retrain=%d rx_starts=%u physet10=0x%08x onu_id_raw=0x%08x\n",
			  retrain, xpon->stats.rx_start_count,
			  airoha_pon_phy_read(xpon, AIROHA_GEPON_PHYSET10), onu_id);

	return 0;
}

/*
 * GPON 块只有 GTC/GEM/BST 四类计数器，没有 PLOAMd/PLOAMu 计数寄存器；缺的两项
 * 记 0 并在采样时跳过，避免把 XG-PON 的偏移误读成 GPON 数据。
 */
static const u32 airoha_xpon_gpon_counter_registers[AIROHA_XPON_MIB_COUNT] = {
	[AIROHA_XPON_MIB_XGTC_RX] = AIROHA_GPON_RX_GTC_CNT,
	[AIROHA_XPON_MIB_BURSTS_TX] = AIROHA_GPON_TX_BST_CNT,
	[AIROHA_XPON_MIB_PLOAMD_RX] = AIROHA_GPON_COUNTER_NONE,
	[AIROHA_XPON_MIB_PLOAMU_TX] = AIROHA_GPON_COUNTER_NONE,
	[AIROHA_XPON_MIB_XGEM_RX] = AIROHA_GPON_RX_GEM_CNT,
	[AIROHA_XPON_MIB_XGEM_TX] = AIROHA_GPON_TX_GEM_CNT,
};

void airoha_xpon_gpon_counters_start(struct airoha_xpon *xpon)
{
	unsigned int i;

	lockdep_assert_held(&xpon->state_lock);
	for (i = 0; i < ARRAY_SIZE(airoha_xpon_gpon_counter_registers); i++) {
		u32 reg = airoha_xpon_gpon_counter_registers[i];

		xpon->stats.mib_last[i] =
			reg == AIROHA_GPON_COUNTER_NONE ? 0 :
							 airoha_gpon_read(xpon, reg);
	}
}

void airoha_xpon_gpon_counters_update(struct airoha_xpon *xpon)
{
	unsigned int i;

	lockdep_assert_held(&xpon->state_lock);
	if (!xpon->rx_active || !xpon->mac_initialized)
		return;

	for (i = 0; i < ARRAY_SIZE(airoha_xpon_gpon_counter_registers); i++) {
		u32 reg = airoha_xpon_gpon_counter_registers[i];
		u32 value;

		if (reg == AIROHA_GPON_COUNTER_NONE)
			continue;
		value = airoha_gpon_read(xpon, reg);
		xpon->stats.mib_total[i] +=
			(u32)(value - xpon->stats.mib_last[i]);
		xpon->stats.mib_last[i] = value;
	}
}

void airoha_xpon_gpon_stop_rx(struct airoha_xpon *xpon)
{
	u32 rx_ctrl;
	int ret;

	atomic_set(&xpon->tx_resync_pending, 0);
	if (xpon->xgpon_irq_enabled) {
		airoha_gpon_write(xpon, AIROHA_GPON_INT_ENABLE, 0);
		xpon->xgpon_irq_enabled = false;
		synchronize_irq(xpon->mac_irq);
	}

	airoha_xpon_gpon_counters_update(xpon);
	ret = airoha_pcs_pon_stop_rx(xpon->pcs);
	if (ret)
		dev_warn_ratelimited(xpon->dev, "PON PMA PLUG_OUT failed: %d\n",
		                     ret);
	rx_ctrl = airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_RX_SYNC_CTRL);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_RX_SYNC_CTRL,
	                     rx_ctrl & ~AIROHA_XGPON_PHY_RX_ENABLE);
	if (xpon->mac_initialized)
		airoha_xpon_gpon_stop_mac_interfaces(xpon, "rx-deactivate");
	/* 0x43a0 bit0 清零 = 厂商 gponDevSwReset 的复位保持阶段。 */
	if (xpon->gpon_base)
		airoha_gpon_update_bits(xpon, AIROHA_GPON_MAC_SET,
		                        AIROHA_GPON_MAC_SET_RELEASED, 0);
	airoha_xpon_gpon_clear_assignment(xpon);
	airoha_xpon_gpon_set_onu_state(xpon, AIROHA_XGPON_O1);
	xpon->mac_initialized = false;
	xpon->crypto.default_ploam_ik_programmed = false;
	xpon->crypto.activation_keys_programmed = false;
	xpon->crypto.o4_key_switch_observed = false;
	xpon->burst_profile_programmed = false;
	xpon->rx_active = false;
	xpon->lifecycle = AIROHA_XPON_WAIT_OPTICAL_SIGNAL;

	/* 恢复阶梯的每一轮 stop 都留一行，sync_seen 是刚结束这个周期里见过的同步态。 */
	airoha_xpon_trace(xpon, AIROHA_XPON_TRACE_SEG_MAC, "rx-stopped",
			  "pcs_stop_ret=%d rx_starts=%u recoveries=%u sync_seen=0x%02x\n",
			  ret, xpon->stats.rx_start_count,
			  xpon->stats.no_ready_recovery_count,
			  xpon->trace_sync_seen);
	xpon->trace_sync_seen = 0;
}

void airoha_xpon_gpon_restart_mac(struct airoha_xpon *xpon, const char *reason)
{
	airoha_xpon_gpon_counters_update(xpon);
	atomic_set(&xpon->tx_resync_pending, 0);
	airoha_gpon_write(xpon, AIROHA_GPON_INT_ENABLE, 0);
	xpon->xgpon_irq_enabled = false;
	synchronize_irq(xpon->mac_irq);
	if (xpon->mac_initialized)
		airoha_xpon_gpon_stop_mac_interfaces(xpon, reason);
	if (xpon->gpon_base)
		airoha_gpon_update_bits(xpon, AIROHA_GPON_MAC_SET,
		                        AIROHA_GPON_MAC_SET_RELEASED, 0);
	airoha_xpon_gpon_clear_assignment(xpon);
	airoha_xpon_gpon_set_onu_state(xpon, AIROHA_XGPON_O1);
	xpon->mac_initialized = false;
	xpon->crypto.default_ploam_ik_programmed = false;
	xpon->crypto.activation_keys_programmed = false;
	xpon->crypto.o4_key_switch_observed = false;
	xpon->burst_profile_programmed = false;
	xpon->lifecycle = AIROHA_XPON_WAIT_LINE_SYNC;

	dev_info(xpon->dev, "GPON MAC epoch reset (%s); PHY RX retained\n",
	         reason);
	/* 只重置 MAC，不动 PHY RX：同步态位图重新计，PMA 档位保持不变。 */
	airoha_xpon_trace(xpon, AIROHA_XPON_TRACE_SEG_MAC, "mac-restart",
			  "reason=%s sync_seen=0x%02x rx_starts=%u\n", reason,
			  xpon->trace_sync_seen, xpon->stats.rx_start_count);
	xpon->trace_sync_seen = 0;
}

static void airoha_xpon_gpon_recover_no_sync(struct airoha_xpon *xpon,
                                             unsigned long *delay)
{
	unsigned long deadline;
	bool full_reinit;
	int ret;

	if (!xpon->no_sync_since)
		xpon->no_sync_since = jiffies;
	deadline = xpon->no_sync_since +
	           msecs_to_jiffies(AIROHA_XPON_NO_SYNC_RECOVERY_MS);
	if (time_before(jiffies, deadline)) {
		if (time_before(deadline, jiffies + *delay))
			*delay = deadline - jiffies;
		return;
	}

	xpon->stats.no_ready_recovery_count++;
	full_reinit = (xpon->stats.no_ready_recovery_count + 3) %
	                      AIROHA_XPON_FULL_REINIT_PERIOD ==
	              0;
	/*
	 * 恢复阶梯原有的 dev_warn_ratelimited 会被限流丢掉，计数只能靠这一行不受限流的
	 * pon-trace 复原；sync_seen 能区分"从没见过同步态"和"见过但没到 in"。
	 */
	airoha_xpon_trace(xpon, AIROHA_XPON_TRACE_SEG_RX, "recover-no-sync",
			  "recovery=%u full_reinit=%d sync_seen=0x%02x rx_starts=%u\n",
			  xpon->stats.no_ready_recovery_count, full_reinit,
			  xpon->trace_sync_seen, xpon->stats.rx_start_count);
	airoha_xpon_gpon_stop_rx(xpon);
	if (full_reinit) {
		airoha_xpon_gpon_hold_digital_rx_reset(xpon);
		xpon->pcs_profile_valid = false;
		xpon->stats.full_reinit_count++;
		dev_warn_ratelimited(
			xpon->dev,
			"optical signal present without GTC sync; running full PON PHY reinitialization (recovery %u)\n",
			xpon->stats.no_ready_recovery_count);
	} else {
		dev_warn_ratelimited(
			xpon->dev,
			"optical signal present without GTC sync; running short PMA recovery (recovery %u)\n",
			xpon->stats.no_ready_recovery_count);
	}

	ret = airoha_xpon_gpon_start_rx(xpon);
	if (ret) {
		xpon->last_start_error = ret;
		xpon->lifecycle = AIROHA_XPON_ERROR;
		airoha_xpon_fail(xpon, AIROHA_XPON_TRACE_SEG_RX,
				 "recovery-start-rx", ret,
				 "recovery=%u full_reinit=%d\n",
				 xpon->stats.no_ready_recovery_count, full_reinit);
		*delay = msecs_to_jiffies(AIROHA_XPON_RETRAIN_RETRY_MS);
	} else {
		xpon->last_start_error = 0;
	}
}

static void airoha_xpon_gpon_advance_gtc(struct airoha_xpon *xpon,
                                         unsigned long *delay)
{
	u32 sync = airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_DBG_RX_SYNC_ST);
	u32 sync_state = FIELD_GET(AIROHA_XGPON_PHY_RX_SYNC_MASK, sync);
	int ret;

	/*
	 * 250ms 轮询每轮都读同步态，逐轮打印会冲掉时间线；同步态只有 2bit，所以按值
	 * 置位、每个新状态只出一行。GPON 到底会不会离开 hunt 是 P3 的判据，raw 一并
	 * 留下，便于事后确认这个寄存器在 GPON 下是不是根本不变。
	 */
	if (!(xpon->trace_sync_seen & BIT(sync_state))) {
		xpon->trace_sync_seen |= BIT(sync_state);
		airoha_xpon_trace(xpon, AIROHA_XPON_TRACE_SEG_GTC, "gtc-sync-state",
				  "state=%u raw=0x%08x mac_init=%d rx_active=%d physet10=0x%08x gtc_cnt=0x%08x activation=0x%08x\n",
				  sync_state, sync, xpon->mac_initialized,
				  xpon->rx_active,
				  airoha_pon_phy_read(xpon, AIROHA_GEPON_PHYSET10),
				  airoha_gpon_read(xpon, AIROHA_GPON_RX_GTC_CNT),
				  airoha_gpon_read(xpon, AIROHA_GPON_ACTIVATION_ST));
	}

	if (sync_state == AIROHA_XGPON_PHY_RX_SYNC_IN) {
		xpon->no_sync_since = 0;
		xpon->stats.no_ready_recovery_count = 0;
		if (!xpon->mac_initialized) {
			ret = airoha_xpon_gpon_start_control(xpon);
			if (ret) {
				xpon->last_start_error = ret;
				xpon->lifecycle = AIROHA_XPON_ERROR;
				airoha_xpon_fail(xpon, AIROHA_XPON_TRACE_SEG_GTC,
						 "start-control", ret,
						 "sync_state=%u\n", sync_state);
				*delay = msecs_to_jiffies(
					AIROHA_XPON_RETRAIN_RETRY_MS);
				return;
			}
		}
		/*
		 * 上游 BEN 的放行依赖 GPON PLOAMu 栈（S3-2c）：现在没有发送通路，
		 * 强行调用共享 airoha_xpon_try_arm_upstream() 只会写死块并使能 BEN。
		 * 本阶段保持 receive-only，等 2c 落地后再补 arm 时序。
		 */
		return;
	}

	if (xpon->mac_initialized) {
		unsigned long deadline;

		if (!xpon->no_sync_since)
			xpon->no_sync_since = jiffies;
		deadline = xpon->no_sync_since +
		           msecs_to_jiffies(AIROHA_XPON_SYNC_LOSS_CONFIRM_MS);
		if (time_before(jiffies, deadline)) {
			if (time_before(deadline, jiffies + *delay))
				*delay = deadline - jiffies;
			return;
		}
		dev_warn_ratelimited(
			xpon->dev,
			"GTC sync lost; resetting MAC epoch while retaining PHY RX\n");
		airoha_xpon_gpon_restart_mac(xpon, "PHY LOF");
		*delay = msecs_to_jiffies(AIROHA_XPON_LINK_POLL_MS);
		return;
	}

	airoha_xpon_gpon_recover_no_sync(xpon, delay);
}

void airoha_xpon_gpon_advance(struct airoha_xpon *xpon, u32 phy_events,
                              unsigned long *delay)
{
	int ret;

	if (atomic_xchg(&xpon->tx_resync_pending, 0) && xpon->mac_initialized &&
	    xpon->tx_armed) {
		xpon->stats.tx_resync_count++;
		ret = airoha_xpon_gpon_resync_tx_locked(xpon);
		if (ret) {
			xpon->stats.tx_resync_failure_count++;
			dev_warn_ratelimited(
				xpon->dev, "GPON TX late resync failed: %d\n",
				ret);
		} else {
			dev_info_ratelimited(
				xpon->dev, "GPON TX late resync completed\n");
		}
	}

	if ((phy_events & AIROHA_XGPON_PHY_INT_RX_RDY) && xpon->rx_active &&
	    !xpon->mac_initialized)
		airoha_xpon_gpon_stop_rx(xpon);

	if (!xpon->rx_active) {
		ret = airoha_xpon_gpon_start_rx(xpon);
		if (ret) {
			xpon->last_start_error = ret;
			xpon->lifecycle = AIROHA_XPON_ERROR;
			*delay = msecs_to_jiffies(AIROHA_XPON_RETRAIN_RETRY_MS);
			dev_err_ratelimited(
				xpon->dev,
				"failed to start PON RX with optical signal present: %d; BEN remains disabled, retrying\n",
				ret);
		} else {
			xpon->last_start_error = 0;
		}
		return;
	}

	airoha_xpon_gpon_advance_gtc(xpon, delay);
}

bool airoha_xpon_mode_is_gpon(enum airoha_xpon_mode mode)
{
	return mode == AIROHA_XPON_MODE_GPON;
}

/*
 * GPON MAC register forensics. 1.6/1.9 already settled that the GPON MAC is
 * the dedicated 0x1fb64000 block and that the xgpon window reads 0xDEADBEEF
 * under GPON framing, so the blind whole-window dump is gone. What a bundle
 * still needs is a named, repeatable view of the registers that decide the
 * stage-5/6 verdict, sampled twice in the same run so "does it move" can be
 * answered without pasting logs by hand.
 *
 * P7: the old dump was gated on mac_initialized, which only becomes true after
 * GTC sync, i.e. exactly never in the mode the dump existed for. Only the
 * gpon_base check and per-register window bounds remain.
 */
struct airoha_gpon_debug_csr {
	const char *name;
	u32 reg;
};

static const struct airoha_gpon_debug_csr airoha_xpon_gpon_debug_csrs[] = {
	{ "ONU_ID", AIROHA_GPON_ONU_ID },
	{ "ONU_GLOBAL_CONFIG", AIROHA_GPON_ONU_GLOBAL_CONFIG },
	{ "INT_STATUS", AIROHA_GPON_INT_STATUS },
	{ "INT_ENABLE", AIROHA_GPON_INT_ENABLE },
	{ "TCONT_ALLOC0", AIROHA_GPON_TCONT_ALLOC_BASE + 0 * 4 },
	{ "TCONT_ALLOC1", AIROHA_GPON_TCONT_ALLOC_BASE + 1 * 4 },
	{ "TCONT_ALLOC2", AIROHA_GPON_TCONT_ALLOC_BASE + 2 * 4 },
	{ "TCONT_ALLOC3", AIROHA_GPON_TCONT_ALLOC_BASE + 3 * 4 },
	{ "TCONT_ALLOC4", AIROHA_GPON_TCONT_ALLOC_BASE + 4 * 4 },
	{ "TCONT_ALLOC5", AIROHA_GPON_TCONT_ALLOC_BASE + 5 * 4 },
	{ "TCONT_ALLOC6", AIROHA_GPON_TCONT_ALLOC_BASE + 6 * 4 },
	{ "TCONT_ALLOC7", AIROHA_GPON_TCONT_ALLOC_BASE + 7 * 4 },
	{ "PLOAMU_FIFO_STS", AIROHA_GPON_PLOAMU_FIFO_STS },
	{ "PLOAMD_FIFO_STS", AIROHA_GPON_PLOAMD_FIFO_STS },
	{ "EQD", AIROHA_GPON_EQD },
	{ "ACTIVATION_PARAM", AIROHA_GPON_ACTIVATION_PARAM },
	{ "VENDOR_ID", AIROHA_GPON_VENDOR_ID },
	{ "VS_SN", AIROHA_GPON_VS_SN },
	{ "ACTIVATION_ST", AIROHA_GPON_ACTIVATION_ST },
	{ "MBI_MPI_STOP", AIROHA_GPON_MBI_MPI_STOP },
	{ "DBG_SIG_GROUP0", AIROHA_GPON_DBG_SIG_GROUP0 },
	{ "DBG_SIG_GROUP1", AIROHA_GPON_DBG_SIG_GROUP1 },
	{ "RX_GEM_CNT", AIROHA_GPON_RX_GEM_CNT },
	{ "RX_ETH_CRC_CNT", AIROHA_GPON_RX_ETH_CRC_CNT },
	{ "RX_GTC_CNT", AIROHA_GPON_RX_GTC_CNT },
	{ "TX_GEM_CNT", AIROHA_GPON_TX_GEM_CNT },
	{ "TX_BST_CNT", AIROHA_GPON_TX_BST_CNT },
	{ "RX_GEM_HDR_1BIT", AIROHA_GPON_RX_GEM_HDR_1BIT },
	{ "RX_GEM_HDR_2BIT", AIROHA_GPON_RX_GEM_HDR_2BIT },
	{ "RX_GEM_HDR_UC", AIROHA_GPON_RX_GEM_HDR_UC },
	{ "DS_SF_CNT", AIROHA_GPON_DS_SF_CNT },
	{ "SW_RESYNC", AIROHA_GPON_SW_RESYNC },
	{ "MAC_SET", AIROHA_GPON_MAC_SET },
	{ "O3O4_PLOAM_CTRL", AIROHA_GPON_O3O4_PLOAM_CTRL },
};

/*
 * 0x0054/0x005c 是 PLOAM FIFO 数据口（读 0x005c 会取走一个 PLOAMd），
 * 不进取证表；其余寄存器读侧无副作用。
 */
void airoha_xpon_gpon_debug_window(struct airoha_xpon *xpon,
                                   struct seq_file *m)
{
	u32 first[ARRAY_SIZE(airoha_xpon_gpon_debug_csrs)];
	u32 second[ARRAY_SIZE(airoha_xpon_gpon_debug_csrs)];
	unsigned int i;

	seq_printf(m,
		   "phy_raw: sync_ctrl=0x%08x dbg_rx_sync_st=0x%08x physet10=0x%08x int_status=0x%08x int_enable=0x%08x phy_sta=0x%08x\n",
		   airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_RX_SYNC_CTRL),
		   airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_DBG_RX_SYNC_ST),
		   airoha_pon_phy_read(xpon, AIROHA_GEPON_PHYSET10),
		   airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_INT_STATUS),
		   airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_INT_ENABLE),
		   airoha_pon_phy_read(xpon, AIROHA_XGPON_PHY_XG_PHY_STA));

	seq_printf(m, "gpon_window: %s len: 0x%zx\n",
		   xpon->gpon_base ? "mapped" : "absent",
		   xpon->gpon_window_len);
	if (!xpon->gpon_base)
		return;

	for (i = 0; i < ARRAY_SIZE(airoha_xpon_gpon_debug_csrs); i++) {
		u32 reg = airoha_xpon_gpon_debug_csrs[i].reg;

		first[i] = reg + sizeof(u32) <= xpon->gpon_window_len ?
				   airoha_gpon_read(xpon, reg) :
				   0;
	}
	/* 2ms 足够让 GTC/GEM 计数在收到帧时走出可辨差值。 */
	usleep_range(2000, 2500);
	for (i = 0; i < ARRAY_SIZE(airoha_xpon_gpon_debug_csrs); i++) {
		u32 reg = airoha_xpon_gpon_debug_csrs[i].reg;

		second[i] = reg + sizeof(u32) <= xpon->gpon_window_len ?
				    airoha_gpon_read(xpon, reg) :
				    0;
	}

	for (i = 0; i < ARRAY_SIZE(airoha_xpon_gpon_debug_csrs); i++) {
		const struct airoha_gpon_debug_csr *csr =
			&airoha_xpon_gpon_debug_csrs[i];

		if (csr->reg + sizeof(u32) > xpon->gpon_window_len) {
			seq_printf(m, "gpon %-18s 0x%03x: out of window\n",
				   csr->name, csr->reg);
			continue;
		}
		seq_printf(m,
			   "gpon %-18s 0x%03x: 0x%08x -> 0x%08x delta=%d\n",
			   csr->name, csr->reg, first[i], second[i],
			   (int)(second[i] - first[i]));
	}

	seq_printf(m,
		   "decode: onu_id=%lu valid=%u activation=0x%08x act_state=%lu alloc0=0x%08x\n",
		   FIELD_GET(AIROHA_GPON_ONU_ID_MASK,
			     airoha_gpon_read(xpon, AIROHA_GPON_ONU_ID)),
		   (unsigned int)!!(airoha_gpon_read(xpon, AIROHA_GPON_ONU_ID) &
		                    AIROHA_GPON_ONU_ID_VALID),
		   airoha_gpon_read(xpon, AIROHA_GPON_ACTIVATION_ST),
		   FIELD_GET(AIROHA_GPON_ACTIVATION_STATE_MASK,
			     airoha_gpon_read(xpon, AIROHA_GPON_ACTIVATION_ST)),
		   airoha_gpon_read(xpon,
				    AIROHA_GPON_TCONT_ALLOC_BASE));
}
