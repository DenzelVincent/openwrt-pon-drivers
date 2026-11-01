// SPDX-License-Identifier: GPL-2.0-only

/*
 * Airoha AN7581 GPON (ITU-T G.984) MAC implementation.
 *
 * The PON MAC block on AN7581 exposes separate "gpon" and "xgpon" register
 * windows; this file currently drives gigabit GPON (2.488/1.244 Gbps) through
 * the XG-PON window while switching the PCS profile and the digital PHY RX
 * mode, and reuses the common MAC reset / MBI-MPI stop / T-CONT/GEM
 * programming logic from airoha-xpon-ploam.c and airoha-xpon-control.c.
 *
 * Whether GPON status lives in the dedicated "gpon" window is not established
 * by any local source; airoha_xpon_gpon_debug_window() dumps both so a field
 * capture can decide.
 *
 * Implementation is an independent, from-scratch driver based on ITU-T
 * G.984.3 (GTC layer) and G.984.4 (OMCI) provisions. Register semantics
 * are validated against the public EN7581 SDK header (xepon_mac_c_header)
 * and against the XG-PON driver that ships with PonWRT; no proprietary
 * code is copied verbatim.
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

/*
 * The MAC control path reuses the XG-PON register offsets. That mapping is an
 * assumption, not vendor-documented fact; airoha_xpon_gpon_debug_window()
 * prints the dedicated GPON window alongside so real hardware can confirm
 * which block reports GTC sync and PLOAM activity.
 */

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
 * GPON downstream PLOAM is 13 bytes (1 byte ONU-ID + 1 byte Message ID +
 * 10 bytes payload + 1 byte CRC) per G.984.3 §8.1.3. In the MAC FIFO the
 * hardware presents PLOAMd as 32-bit words; we read the same 13-word
 * width used by XG-PON for DMA alignment and only interpret the first
 * 13 bytes per the GTC framing rules.
 */
#define AIROHA_GPON_PLOAMD_BYTES 13
/* GPON upstream PLOAM is 13 bytes (1 byte ONU-ID + 1 byte Message ID +
 * 10 bytes payload + 1 byte CRC), matching the 11-word FIFO width. */
#define AIROHA_GPON_PLOAMU_BYTES 13

/* GTC counter offset is shared with the XGTC counter at 0x0940 because
 * the MAC block is mode-multiplexed. */
#define AIROHA_GPON_RX_GTC_CNT    AIROHA_XGPON_RX_XGTC_CNT
#define AIROHA_GPON_TX_BURST_CNT  AIROHA_XGPON_TX_BURST_CNT
#define AIROHA_GPON_RX_PLOAMD_CNT AIROHA_XGPON_RX_PLOAMD_CNT
#define AIROHA_GPON_TX_PLOAMU_CNT AIROHA_XGPON_TX_PLOAMU_CNT
#define AIROHA_GPON_RX_GEM_CNT    AIROHA_XGPON_RX_XGEM_CNT
#define AIROHA_GPON_TX_GEM_CNT    AIROHA_XGPON_TX_XGEM_CNT

/*
 * GPON shares the 1.25G GEPON PHY block with 1G-EPON (XEPON_1G pairs the
 * 10G MAC with that PHY block). PHYSET10 bit[31] selects the GPON framing;
 * the EPON 1G path explicitly clears it in airoha-xpon-epon.c, so the GPON
 * path has to set it before the PCS profile is programmed.
 */
#define AIROHA_GEPON_PHYSET10  0x124
#define AIROHA_GEPON_GPON_MODE BIT(31)

/* GPON activation states are G.984 O1..O7. The hardware ACTIVATION_ST[3:0]
 * values match the XG-PON encoding. */
static void airoha_xpon_gpon_release_mac_interfaces(struct airoha_xpon *xpon)
{
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_MBI_MPI_STOP,
	                         AIROHA_XGPON_MBI_MPI_STOP_MASK, 0);
}

static void airoha_xpon_gpon_stop_mac_interfaces(struct airoha_xpon *xpon,
                                                 const char *reason)
{
	u32 value;
	int ret;

	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_MBI_MPI_STOP,
	                         AIROHA_XGPON_MBI_MPI_STOP_MASK,
	                         AIROHA_XGPON_MBI_MPI_STOP_MASK);
	ret = readl_poll_timeout_atomic(
		xpon->xgpon_base + AIROHA_XGPON_MBI_MPI_STOP, value,
		(value & AIROHA_XGPON_MBI_MPI_STOP_DONE_MASK) ==
			AIROHA_XGPON_MBI_MPI_STOP_DONE_MASK,
		1, 3000);
	if (ret)
		dev_warn_ratelimited(
			xpon->dev,
			"timed out stopping GPON MBI/MPI (%s), raw=0x%08x\n",
			reason, value);
}

static void airoha_xpon_gpon_program_identity(struct airoha_xpon *xpon)
{
	const struct airoha_xpon_identity *identity =
		&xpon->identity.active_identity;
	unsigned int i;

	/* GPON SN is 8 bytes: 4-byte VENDOR_ID + 4-byte VS_SN (vendor-specific). */
	if (identity->serial_configured) {
		airoha_xgpon_write(xpon, AIROHA_XGPON_VENDOR_ID,
		                   airoha_xpon_bytes_to_u32(identity->serial_number));
		airoha_xgpon_write(xpon, AIROHA_XGPON_VS_SN,
		                   airoha_xpon_bytes_to_u32(identity->serial_number + 4));
	}

	/* Registration-ID: 36 bytes (10 words) mapped in reverse order,
	 * matching the XG-PON RGS_ID register layout that the same MAC exposes. */
	for (i = 0; i < 9; i++)
		airoha_xgpon_write(xpon, AIROHA_XGPON_RGS_ID3_0 + i * 4,
		                   airoha_xpon_bytes_to_u32(identity->registration_id +
		                                           (8 - i) * 4));
}

static int airoha_xpon_gpon_set_tx_interface_stop(struct airoha_xpon *xpon,
                                                  u32 stop_bit, u32 done_bit,
                                                  bool stop)
{
	u32 value;

	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_MBI_MPI_STOP, stop_bit,
	                         stop ? stop_bit : 0);
	if (!stop)
		return 0;

	return readl_poll_timeout_atomic(xpon->xgpon_base +
	                                         AIROHA_XGPON_MBI_MPI_STOP,
	                                 value, value & done_bit, 1, 3000);
}

static int airoha_xpon_gpon_resync_tx_locked(struct airoha_xpon *xpon)
{
	u32 profile_valid, value;
	bool mbi_stopped = false;
	bool mpi_stopped = false;
	int ret;

	lockdep_assert_held(&xpon->state_lock);
	mutex_lock(&xpon->ploam_lock);

	profile_valid = airoha_xgpon_read(xpon, AIROHA_XGPON_US_PROF_VLD);
	airoha_xgpon_write(xpon, AIROHA_XGPON_US_PROF_VLD,
	                   profile_valid & ~AIROHA_XGPON_US_PROFILE_VALID_MASK);

	mbi_stopped = true;
	ret = airoha_xpon_gpon_set_tx_interface_stop(
		xpon, AIROHA_XGPON_MBI_TX_STOP, AIROHA_XGPON_MBI_TX_STOP_DONE,
		true);
	if (ret)
		goto out_restore;

	mpi_stopped = true;
	ret = airoha_xpon_gpon_set_tx_interface_stop(
		xpon, AIROHA_XGPON_MPI_TX_STOP, AIROHA_XGPON_MPI_TX_STOP_DONE,
		true);
	if (ret)
		goto out_restore;

	airoha_xgpon_write(xpon, AIROHA_XGPON_EQD, xpon->eqd);
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_DBG_RESYNC,
	                         AIROHA_XGPON_SW_RESYNC_MASK,
	                         AIROHA_XGPON_SW_RESYNC_MASK);
	readl_poll_timeout_atomic(xpon->xgpon_base + AIROHA_XGPON_DBG_RESYNC,
	                          value, value & AIROHA_XGPON_TX_SYNC_READY, 1,
	                          3000);

	airoha_xpon_gpon_set_tx_interface_stop(xpon, AIROHA_XGPON_MPI_TX_STOP,
	                                       AIROHA_XGPON_MPI_TX_STOP_DONE,
	                                       false);
	mpi_stopped = false;
	ret = readl_poll_timeout_atomic(
		xpon->xgpon_base + AIROHA_XGPON_DBG_RESYNC, value,
		value & AIROHA_XGPON_TX_SYNC_READY, 1, 200);

out_restore:
	if (mpi_stopped)
		airoha_xpon_gpon_set_tx_interface_stop(
			xpon, AIROHA_XGPON_MPI_TX_STOP,
			AIROHA_XGPON_MPI_TX_STOP_DONE, false);
	if (mbi_stopped)
		airoha_xpon_gpon_set_tx_interface_stop(
			xpon, AIROHA_XGPON_MBI_TX_STOP,
			AIROHA_XGPON_MBI_TX_STOP_DONE, false);
	airoha_xgpon_write(xpon, AIROHA_XGPON_US_PROF_VLD, profile_valid);
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

	/* Release MAC reset after GTC synchronization, before O2_3. */
	airoha_xgpon_write(xpon, AIROHA_XGPON_INT_ENABLE, 0);
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_SW_RST,
	                         AIROHA_XGPON_MAC_RESET_RELEASED,
	                         AIROHA_XGPON_MAC_RESET_RELEASED);
	airoha_xpon_gpon_counters_start(xpon);
	airoha_xpon_gpon_release_mac_interfaces(xpon);
	airoha_xgpon_write(xpon, AIROHA_XGPON_US_PROF_VLD, 0);
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_RSP_TIME,
	                         AIROHA_XGPON_RESPONSE_TIME_MASK, 0x551);
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_O23_O4_PLOAMU_CTRL,
	                         AIROHA_XGPON_O23_O4_SOFTWARE_REPLY, 0);
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_DBG_CAP_SETTING,
	                         AIROHA_XGPON_DBG_O52_IDLE_ONLY |
	                                 AIROHA_XGPON_DBG_HW_US_OMCI_MIC |
	                                 AIROHA_XGPON_DBG_HW_DS_OMCI_MIC,
	                         AIROHA_XGPON_DBG_O52_IDLE_ONLY |
	                                 AIROHA_XGPON_DBG_HW_US_OMCI_MIC |
	                                 AIROHA_XGPON_DBG_HW_DS_OMCI_MIC);
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_DBG_RESYNC,
	                         AIROHA_XGPON_TX_LATE_AUTO_RESYNC, 0);
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
	airoha_xpon_clear_assignment(xpon);
	airoha_xpon_clear_tconts(xpon);
	airoha_xpon_set_onu_state(xpon, AIROHA_XGPON_O2_3);

	airoha_xpon_clear_mac_errors(xpon, true);
	airoha_xgpon_write(xpon, AIROHA_XGPON_INT_STATUS,
	                   AIROHA_XGPON_INT_ACTIVATION_MASK |
	                           AIROHA_XGPON_INT_ERROR_MASK);
	airoha_xgpon_write(xpon, AIROHA_XGPON_INT_ENABLE,
	                   AIROHA_XGPON_INT_ACTIVATION_MASK |
	                           AIROHA_XGPON_INT_FIFO_ERROR);
	xpon->xgpon_irq_enabled = true;
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
	u32 enabled, events, status;
	u32 fifo_error = 0, tx_error = 0, rx_error = 0, bwm_check = 0;

	status = airoha_xgpon_read(xpon, AIROHA_XGPON_INT_STATUS);
	enabled = airoha_xgpon_read(xpon, AIROHA_XGPON_INT_ENABLE);
	events = status & enabled;
	if (!events)
		return IRQ_NONE;

	airoha_xgpon_write(xpon, AIROHA_XGPON_INT_STATUS, events);
	if (READ_ONCE(xpon->stopping))
		return IRQ_HANDLED;

	if (events & AIROHA_XGPON_INT_SN_REQUEST)
		xpon->stats.sn_request_count++;
	if (events & AIROHA_XGPON_INT_SN_SENT)
		xpon->stats.sn_sent_count++;
	if (events & AIROHA_XGPON_INT_RANGING_REQUEST) {
		u32 key_index;

		xpon->stats.ranging_request_count++;
		key_index = airoha_xgpon_read(xpon, AIROHA_XGPON_CUR_KIDX);
		if (!(key_index & AIROHA_XGPON_CUR_PIK_INDEX) &&
		    !(key_index & AIROHA_XGPON_CUR_OIK_INDEX))
			WRITE_ONCE(xpon->crypto.o4_key_switch_observed, true);
	}
	if (events & AIROHA_XGPON_INT_REGISTRATION_SENT)
		xpon->stats.registration_sent_count++;

	if (events & AIROHA_XGPON_INT_ERROR_MASK) {
		fifo_error = airoha_xgpon_read(xpon, AIROHA_XGPON_FIFO_ERR_STS);
		tx_error = airoha_xgpon_read(xpon, AIROHA_XGPON_TX_ERR_STS);
		rx_error = airoha_xgpon_read(xpon, AIROHA_XGPON_RX_ERR_STS);
		bwm_check =
			airoha_xgpon_read(xpon, AIROHA_XGPON_DBG_BWM_CHK_STS);
		WRITE_ONCE(xpon->stats.latched_fifo_error,
		           READ_ONCE(xpon->stats.latched_fifo_error) |
		                   fifo_error);
		WRITE_ONCE(xpon->stats.latched_tx_error,
		           READ_ONCE(xpon->stats.latched_tx_error) | tx_error);
		WRITE_ONCE(xpon->stats.latched_rx_error,
		           READ_ONCE(xpon->stats.latched_rx_error) | rx_error);
		WRITE_ONCE(xpon->stats.latched_bwm_check,
		           READ_ONCE(xpon->stats.latched_bwm_check) |
		                   bwm_check);
		xpon->stats.mac_error_irq_count++;
		if (fifo_error & AIROHA_XGPON_FIFO_ERR_RX_MBI_HEADER_OVERRUN)
			xpon->stats.rx_mbi_header_overrun_irq_count++;
		if (fifo_error & AIROHA_XGPON_FIFO_ERR_RX_MBI_PAYLOAD_OVERRUN)
			xpon->stats.rx_mbi_payload_overrun_irq_count++;

		airoha_xgpon_write(xpon, AIROHA_XGPON_FIFO_ERR_STS, U32_MAX);
		airoha_xgpon_write(xpon, AIROHA_XGPON_TX_ERR_STS, U32_MAX);
		airoha_xgpon_write(xpon, AIROHA_XGPON_RX_ERR_STS, U32_MAX);
		airoha_xgpon_write(xpon, AIROHA_XGPON_DBG_BWM_CHK_STS, U32_MAX);
		dev_warn_ratelimited(
			xpon->dev,
			"GPON MAC error: int=0x%08x fifo=0x%08x tx=0x%08x rx=0x%08x bwm=0x%08x\n",
			(u32)(events & AIROHA_XGPON_INT_ERROR_MASK), fifo_error,
			tx_error, rx_error, bwm_check);

		if ((events & AIROHA_XGPON_INT_TX_ERROR) &&
		    (tx_error & AIROHA_XGPON_TX_ERR_LATE_START)) {
			xpon->stats.tx_late_error_count++;
			atomic_set(&xpon->tx_resync_pending, 1);
			mod_delayed_work(system_wq, &xpon->link_work, 0);
		}
	}

	if ((events & AIROHA_XGPON_INT_PLOAMD_RECV) &&
	    airoha_xpon_drain_ploamd(xpon) == AIROHA_XGPON_PLOAMD_MAX_DRAIN)
		dev_warn_ratelimited(
			xpon->dev,
			"PLOAMd FIFO drain reached the per-IRQ safety limit\n");
	if (atomic_read(&xpon->mac_restart_pending) ||
	    atomic_read(&xpon->data_path_retry_pending))
		mod_delayed_work(system_wq, &xpon->link_work, 0);

	return IRQ_HANDLED;
}

void airoha_xpon_gpon_mask_irqs(struct airoha_xpon *xpon)
{
	airoha_xgpon_write(xpon, AIROHA_XGPON_INT_ENABLE, 0);
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

	/* Hold the GPON MAC in reset during PMA initialization. */
	airoha_xgpon_write(xpon, AIROHA_XGPON_INT_ENABLE, 0);
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_MBI_MPI_STOP,
	                         AIROHA_XGPON_MBI_MPI_STOP_MASK,
	                         AIROHA_XGPON_MBI_MPI_STOP_MASK);
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_SW_RST,
	                         AIROHA_XGPON_MAC_RESET_RELEASED, 0);
	/*
	 * MAC reset leaves ONU_ID behind, so an assignment from an earlier XG-PON
	 * session would still read back as valid and hide the real O2/O4 state.
	 */
	onu_id_prev = airoha_xgpon_read(xpon, AIROHA_XGPON_ONU_ID);
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_ONU_ID,
	                         AIROHA_XGPON_ONU_ID_VALID |
	                                 AIROHA_XGPON_ONU_ID_MASK,
	                         AIROHA_XGPON_BROADCAST_ID);
	xpon->onu_id = AIROHA_XGPON_BROADCAST_ID;
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_SFP_VLD_LEVEL,
	                     xpon->frontend_link.xgpon_sfp_valid_level);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_INT_ENABLE, 0);
	airoha_pon_phy_write(xpon, AIROHA_XGPON_PHY_INT_STATUS,
	                     AIROHA_XGPON_PHY_INT_RX_MASK);
	xpon->lifecycle = AIROHA_XPON_WAIT_OPTICAL_SIGNAL;
	/* 新的 MAC 周期：同步态取证位图重新计数。 */
	xpon->trace_sync_seen = 0;

	/*
	 * onu_id_prev 是清零前的硬件原值，也是本段唯一的"上一轮残留"证据：GPON 的
	 * ONU-ID 只有 10bit(驱动按 8bit 用)，从 XGS 切过来时若 valid=1 且 id 非广播，
	 * 第 6 段的认证会被旧身份带偏。
	 */
	airoha_xpon_trace(xpon, AIROHA_XPON_TRACE_SEG_MAC, "mac-prepared",
			  "onu_id_prev=0x%08x sw_rst=0x%08x phy_int_en=0x%08x\n",
			  onu_id_prev,
			  airoha_xgpon_read(xpon, AIROHA_XGPON_SW_RST),
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

	onu_id = airoha_xgpon_read(xpon, AIROHA_XGPON_ONU_ID);
	dev_info(
		xpon->dev,
		"GPON RX started, irq=%d, ONU-ID=%lu (%s), burst TX remains disabled\n",
		xpon->mac_irq, FIELD_GET(AIROHA_XGPON_ONU_ID_MASK, onu_id),
		onu_id & AIROHA_XGPON_ONU_ID_VALID ? "valid" : "invalid");
	airoha_xpon_trace(xpon, AIROHA_XPON_TRACE_SEG_MAC, "rx-started",
			  "retrain=%d rx_starts=%u physet10=0x%08x onu_id_raw=0x%08x\n",
			  retrain, xpon->stats.rx_start_count,
			  airoha_pon_phy_read(xpon, AIROHA_GEPON_PHYSET10), onu_id);

	return 0;
}

static const u32 airoha_xpon_gpon_counter_registers[AIROHA_XPON_MIB_COUNT] = {
	[AIROHA_XPON_MIB_XGTC_RX] = AIROHA_GPON_RX_GTC_CNT,
	[AIROHA_XPON_MIB_BURSTS_TX] = AIROHA_GPON_TX_BURST_CNT,
	[AIROHA_XPON_MIB_PLOAMD_RX] = AIROHA_GPON_RX_PLOAMD_CNT,
	[AIROHA_XPON_MIB_PLOAMU_TX] = AIROHA_GPON_TX_PLOAMU_CNT,
	[AIROHA_XPON_MIB_XGEM_RX] = AIROHA_GPON_RX_GEM_CNT,
	[AIROHA_XPON_MIB_XGEM_TX] = AIROHA_GPON_TX_GEM_CNT,
};

void airoha_xpon_gpon_counters_start(struct airoha_xpon *xpon)
{
	unsigned int i;

	lockdep_assert_held(&xpon->state_lock);
	for (i = 0; i < ARRAY_SIZE(airoha_xpon_gpon_counter_registers); i++)
		xpon->stats.mib_last[i] = airoha_xgpon_read(
			xpon, airoha_xpon_gpon_counter_registers[i]);
}

void airoha_xpon_gpon_counters_update(struct airoha_xpon *xpon)
{
	unsigned int i;

	lockdep_assert_held(&xpon->state_lock);
	if (!xpon->rx_active || !xpon->mac_initialized)
		return;

	for (i = 0; i < ARRAY_SIZE(airoha_xpon_gpon_counter_registers); i++) {
		u32 value = airoha_xgpon_read(
			xpon, airoha_xpon_gpon_counter_registers[i]);

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
		airoha_xgpon_write(xpon, AIROHA_XGPON_INT_ENABLE, 0);
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
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_SW_RST,
	                         AIROHA_XGPON_MAC_RESET_RELEASED, 0);
	airoha_xpon_clear_assignment(xpon);
	xpon->onu_state = AIROHA_XGPON_O1;
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
	airoha_xgpon_write(xpon, AIROHA_XGPON_INT_ENABLE, 0);
	xpon->xgpon_irq_enabled = false;
	synchronize_irq(xpon->mac_irq);
	if (xpon->mac_initialized)
		airoha_xpon_gpon_stop_mac_interfaces(xpon, reason);
	airoha_xgpon_update_bits(xpon, AIROHA_XGPON_SW_RST,
	                         AIROHA_XGPON_MAC_RESET_RELEASED, 0);
	airoha_xpon_clear_assignment(xpon);
	airoha_xpon_set_onu_state(xpon, AIROHA_XGPON_O1);
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
				  "state=%u raw=0x%08x mac_init=%d rx_active=%d physet10=0x%08x\n",
				  sync_state, sync, xpon->mac_initialized,
				  xpon->rx_active,
				  airoha_pon_phy_read(xpon, AIROHA_GEPON_PHYSET10));
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
		ret = airoha_xpon_try_arm_upstream(xpon);
		if (ret) {
			xpon->last_start_error = ret;
			airoha_xpon_fail(xpon, AIROHA_XPON_TRACE_SEG_CTRL,
					 "arm-upstream", ret,
					 "sync_state=%u\n", sync_state);
			dev_warn_ratelimited(
				xpon->dev,
				"failed to arm upstream BEN: %d; remaining receive-only and retrying\n",
				ret);
		}
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
 * GPON MAC register forensics. The device tree exposes a dedicated "gpon"
 * window that no driver in this tree consumes, and the local sources do not
 * document whether GPON GTC status appears there or in the shared XG-PON
 * window. Dump both raw views so a field capture shows which block actually
 * tracks the incoming frames.
 */
void airoha_xpon_gpon_debug_window(struct airoha_xpon *xpon,
                                   struct seq_file *m)
{
	unsigned int i, words;

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
	/*
	 * Only walk the window once the MAC has answered reset polling, otherwise a
	 * block without a clock would hang the read.
	 */
	if (!xpon->mac_initialized) {
		seq_puts(m, "gpon_window: skipped, mac not initialized\n");
		return;
	}

	words = (xpon->gpon_window_len / 4) & ~7U;
	for (i = 0; i < words; i += 8)
		seq_printf(m,
			   "gpon 0x%03x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			   i * 4,
			   readl(xpon->gpon_base + (i + 0) * 4),
			   readl(xpon->gpon_base + (i + 1) * 4),
			   readl(xpon->gpon_base + (i + 2) * 4),
			   readl(xpon->gpon_base + (i + 3) * 4),
			   readl(xpon->gpon_base + (i + 4) * 4),
			   readl(xpon->gpon_base + (i + 5) * 4),
			   readl(xpon->gpon_base + (i + 6) * 4),
			   readl(xpon->gpon_base + (i + 7) * 4));
}
