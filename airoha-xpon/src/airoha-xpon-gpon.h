/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __AIROHA_XPON_GPON_H
#define __AIROHA_XPON_GPON_H

#include <linux/interrupt.h>
#include <linux/types.h>

struct airoha_xpon;

/*
 * ITU-T G.984 GPON PLOAM message types (downstream).
 * Values per G.984.3 Table 8-4.
 */
enum airoha_gpon_ploam_msg_type {
	AIROHA_GPON_PLOAM_UPSTREAM_OVERHEAD   = 0x01,
	AIROHA_GPON_PLOAM_ASSIGN_ONU_ID       = 0x03,
	AIROHA_GPON_PLOAM_RANGING_TIME        = 0x04,
	AIROHA_GPON_PLOAM_REQUEST_PASSWORD    = 0x07,
	AIROHA_GPON_PLOAM_ASSIGN_ALLOC_ID     = 0x09,
	AIROHA_GPON_PLOAM_NO_MESSAGE          = 0x0a,
	AIROHA_GPON_PLOAM_POPUP               = 0x0c,
	AIROHA_GPON_PLOAM_KEY_CONTROL         = 0x0d,
	AIROHA_GPON_PLOAM_SLEEP_ALLOW         = 0x0f,
	AIROHA_GPON_PLOAM_DEACTIVATE_ONU_ID   = 0x13,
	AIROHA_GPON_PLOAM_DISABLE_SERIAL_NUM  = 0x15,
	AIROHA_GPON_PLOAM_REQUEST_REGISTRATION= 0x1d,
};

/*
 * ITU-T G.984 GPON PLOAM upstream message types.
 */
enum airoha_gpon_ploam_up_type {
	AIROHA_GPON_PLOAM_SERIAL_NUMBER_ONU   = 0x02,
	AIROHA_GPON_PLOAM_PASSWORD            = 0x05,
	AIROHA_GPON_PLOAM_ACKNOWLEDGE         = 0x06,
	AIROHA_GPON_PLOAM_KEY_REPORT          = 0x0e,
	AIROHA_GPON_PLOAM_POPUP_ACK           = 0x10,
	AIROHA_GPON_PLOAM_REGISTRATION        = 0x1e,
};

int airoha_xpon_gpon_prepare(struct airoha_xpon *xpon);
int airoha_xpon_gpon_start_rx(struct airoha_xpon *xpon);
void airoha_xpon_gpon_stop_rx(struct airoha_xpon *xpon);
void airoha_xpon_gpon_mask_irqs(struct airoha_xpon *xpon);
bool airoha_xpon_gpon_los(struct airoha_xpon *xpon);
bool airoha_xpon_mode_is_gpon(enum airoha_xpon_mode mode);
irqreturn_t airoha_xpon_gpon_irq(struct airoha_xpon *xpon);
irqreturn_t airoha_xpon_gpon_phy_irq(struct airoha_xpon *xpon);
void airoha_xpon_gpon_restart_mac(struct airoha_xpon *xpon,
                                  const char *reason);
void airoha_xpon_gpon_advance(struct airoha_xpon *xpon, u32 phy_events,
                              unsigned long *delay);
void airoha_xpon_gpon_counters_start(struct airoha_xpon *xpon);
void airoha_xpon_gpon_counters_update(struct airoha_xpon *xpon);

#endif
