/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)	"[drm-dp] %s: " fmt, __func__

#include <linux/delay.h>

#include "dp_catalog.h"
#include "dp_reg.h"

#define dp_read(offset) readl_relaxed((offset))
#define dp_write(offset, data) writel_relaxed((data), (offset))

#define dp_catalog_get_priv(x) { \
	struct dp_catalog *dp_catalog; \
	dp_catalog = container_of(x, struct dp_catalog, x); \
	catalog = container_of(dp_catalog, struct dp_catalog_private, \
				dp_catalog); \
}

#define DP_INTERRUPT_STATUS1 \
	(DP_INTR_AUX_I2C_DONE| \
	DP_INTR_WRONG_ADDR | DP_INTR_TIMEOUT | \
	DP_INTR_NACK_DEFER | DP_INTR_WRONG_DATA_CNT | \
	DP_INTR_I2C_NACK | DP_INTR_I2C_DEFER | \
	DP_INTR_PLL_UNLOCKED | DP_INTR_AUX_ERROR)

#define DP_INTR_MASK1		(DP_INTERRUPT_STATUS1 << 2)

#define DP_INTERRUPT_STATUS2 \
	(DP_INTR_READY_FOR_VIDEO | DP_INTR_IDLE_PATTERN_SENT | \
	DP_INTR_FRAME_END | DP_INTR_CRC_UPDATED)

#define DP_INTR_MASK2		(DP_INTERRUPT_STATUS2 << 2)

static u8 const vm_pre_emphasis[4][4] = {
	{0x00, 0x0B, 0x12, 0xFF},       /* pe0, 0 db */
	{0x00, 0x0A, 0x12, 0xFF},       /* pe1, 3.5 db */
	{0x00, 0x0C, 0xFF, 0xFF},       /* pe2, 6.0 db */
	{0xFF, 0xFF, 0xFF, 0xFF}        /* pe3, 9.5 db */
};

/* voltage swing, 0.2v and 1.0v are not support */
static u8 const vm_voltage_swing[4][4] = {
	{0x07, 0x0F, 0x14, 0xFF}, /* sw0, 0.4v  */
	{0x11, 0x1D, 0x1F, 0xFF}, /* sw1, 0.6 v */
	{0x18, 0x1F, 0xFF, 0xFF}, /* sw1, 0.8 v */
	{0xFF, 0xFF, 0xFF, 0xFF}  /* sw1, 1.2 v, optional */
};

struct dp_catalog_private {
	struct device *dev;
	struct dp_io *io;
	struct dp_catalog dp_catalog;
};

/* aux related catalog functions */
static u32 dp_catalog_aux_read_data(struct dp_catalog_aux *aux)
{
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!aux) {
		pr_err("invalid input\n");
		goto end;
	}

	dp_catalog_get_priv(aux);
	base = catalog->io->ctrl_io.base;

	return dp_read(base + DP_AUX_DATA);
end:
	return 0;
}

static int dp_catalog_aux_write_data(struct dp_catalog_aux *aux)
{
	int rc = 0;
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!aux) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto end;
	}

	dp_catalog_get_priv(aux);
	base = catalog->io->ctrl_io.base;

	dp_write(base + DP_AUX_DATA, aux->data);
end:
	return rc;
}

static int dp_catalog_aux_write_trans(struct dp_catalog_aux *aux)
{
	int rc = 0;
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!aux) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto end;
	}

	dp_catalog_get_priv(aux);
	base = catalog->io->ctrl_io.base;

	dp_write(base + DP_AUX_TRANS_CTRL, aux->data);
end:
	return rc;
}

static void dp_catalog_aux_reset(struct dp_catalog_aux *aux)
{
	u32 aux_ctrl;
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!aux) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(aux);
	base = catalog->io->ctrl_io.base;

	aux_ctrl = dp_read(base + DP_AUX_CTRL);

	aux_ctrl |= BIT(1);
	dp_write(base + DP_AUX_CTRL, aux_ctrl);
	usleep_range(1000, 1010); /* h/w recommended delay */

	aux_ctrl &= ~BIT(1);
	dp_write(base + DP_AUX_CTRL, aux_ctrl);
}

static void dp_catalog_aux_enable(struct dp_catalog_aux *aux, bool enable)
{
	u32 aux_ctrl;
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!aux) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(aux);
	base = catalog->io->ctrl_io.base;

	aux_ctrl = dp_read(base + DP_AUX_CTRL);

	if (enable) {
		dp_write(base + DP_TIMEOUT_COUNT, 0xffff);
		dp_write(base + DP_AUX_LIMITS, 0xffff);
		aux_ctrl |= BIT(0);
	} else {
		aux_ctrl &= ~BIT(0);
	}

	dp_write(base + DP_AUX_CTRL, aux_ctrl);
}

static void dp_catalog_aux_setup(struct dp_catalog_aux *aux, u32 *aux_cfg)
{
	struct dp_catalog_private *catalog;

	if (!aux || !aux_cfg) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(aux);

	dp_write(catalog->io->phy_io.base + DP_PHY_PD_CTL, 0x02);
	wmb(); /* make sure PD programming happened */
	dp_write(catalog->io->phy_io.base + DP_PHY_PD_CTL, 0x7d);

	/* Turn on BIAS current for PHY/PLL */
	dp_write(catalog->io->dp_pll_io.base +
		QSERDES_COM_BIAS_EN_CLKBUFLR_EN, 0x3f);

	/* DP AUX CFG register programming */
	dp_write(catalog->io->phy_io.base + DP_PHY_AUX_CFG0, aux_cfg[0]);
	dp_write(catalog->io->phy_io.base + DP_PHY_AUX_CFG1, aux_cfg[1]);
	dp_write(catalog->io->phy_io.base + DP_PHY_AUX_CFG2, aux_cfg[2]);
	dp_write(catalog->io->phy_io.base + DP_PHY_AUX_CFG3, aux_cfg[3]);
	dp_write(catalog->io->phy_io.base + DP_PHY_AUX_CFG4, aux_cfg[4]);
	dp_write(catalog->io->phy_io.base + DP_PHY_AUX_CFG5, aux_cfg[5]);
	dp_write(catalog->io->phy_io.base + DP_PHY_AUX_CFG6, aux_cfg[6]);
	dp_write(catalog->io->phy_io.base + DP_PHY_AUX_CFG7, aux_cfg[7]);
	dp_write(catalog->io->phy_io.base + DP_PHY_AUX_CFG8, aux_cfg[8]);
	dp_write(catalog->io->phy_io.base + DP_PHY_AUX_CFG9, aux_cfg[9]);

	dp_write(catalog->io->phy_io.base + DP_PHY_AUX_INTERRUPT_MASK, 0x1F);
}

static void dp_catalog_aux_get_irq(struct dp_catalog_aux *aux, bool cmd_busy)
{
	u32 ack;
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!aux) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(aux);
	base = catalog->io->ctrl_io.base;

	if (cmd_busy)
		dp_write(base + DP_AUX_TRANS_CTRL, 0x0);

	aux->isr = dp_read(base + DP_INTR_STATUS);
	aux->isr &= ~DP_INTR_MASK1;
	ack = aux->isr & DP_INTERRUPT_STATUS1;
	ack <<= 1;
	ack |= DP_INTR_MASK1;
	dp_write(base + DP_INTR_STATUS, ack);
}

/* controller related catalog functions */
static u32 dp_catalog_ctrl_read_hdcp_status(struct dp_catalog_ctrl *ctrl)
{
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return -EINVAL;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	return dp_read(base + DP_HDCP_STATUS);
}

static void dp_catalog_ctrl_update_transfer_unit(struct dp_catalog_ctrl *ctrl)
{
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	dp_write(base + DP_VALID_BOUNDARY, ctrl->valid_boundary);
	dp_write(base + DP_TU, ctrl->dp_tu);
	dp_write(base + DP_VALID_BOUNDARY_2, ctrl->valid_boundary2);
}

static void dp_catalog_ctrl_state_ctrl(struct dp_catalog_ctrl *ctrl, u32 state)
{
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	dp_write(base + DP_STATE_CTRL, state);
}

static void dp_catalog_ctrl_config_ctrl(struct dp_catalog_ctrl *ctrl, u32 cfg)
{
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	pr_debug("DP_CONFIGURATION_CTRL=0x%x\n", cfg);

	dp_write(base + DP_CONFIGURATION_CTRL, cfg);
	dp_write(base + DP_MAINLINK_LEVELS, 0xa08);
	dp_write(base + MMSS_DP_ASYNC_FIFO_CONFIG, 0x1);
}

static void dp_catalog_ctrl_lane_mapping(struct dp_catalog_ctrl *ctrl)
{
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	dp_write(base + DP_LOGICAL2PHYSCIAL_LANE_MAPPING, 0xe4);
}

static void dp_catalog_ctrl_mainlink_ctrl(struct dp_catalog_ctrl *ctrl,
						bool enable)
{
	u32 mainlink_ctrl;
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	if (enable) {
		dp_write(base + DP_MAINLINK_CTRL, 0x02000000);
		wmb(); /* make sure mainlink is turned off before reset */
		dp_write(base + DP_MAINLINK_CTRL, 0x02000002);
		wmb(); /* make sure mainlink entered reset */
		dp_write(base + DP_MAINLINK_CTRL, 0x02000000);
		wmb(); /* make sure mainlink reset done */
		dp_write(base + DP_MAINLINK_CTRL, 0x02000001);
		wmb(); /* make sure mainlink turned on */
	} else {
		mainlink_ctrl = dp_read(base + DP_MAINLINK_CTRL);
		mainlink_ctrl &= ~BIT(0);
		dp_write(base + DP_MAINLINK_CTRL, mainlink_ctrl);
	}
}

static void dp_catalog_ctrl_config_misc(struct dp_catalog_ctrl *ctrl,
					u32 cc, u32 tb)
{
	u32 misc_val = cc;
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	misc_val |= (tb << 5);
	misc_val |= BIT(0); /* Configure clock to synchronous mode */

	pr_debug("misc settings = 0x%x\n", misc_val);
	dp_write(base + DP_MISC1_MISC0, misc_val);
}

static void dp_catalog_ctrl_config_msa(struct dp_catalog_ctrl *ctrl,
					u32 rate)
{
	u32 pixel_m, pixel_n;
	u32 mvid, nvid;
	u32 const link_rate = 540000;
	struct dp_catalog_private *catalog;
	void __iomem *base_cc, *base_ctrl;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base_cc = catalog->io->dp_cc_io.base;
	base_ctrl = catalog->io->ctrl_io.base;

	pixel_m = dp_read(base_cc + MMSS_DP_PIXEL_M);
	pixel_n = dp_read(base_cc + MMSS_DP_PIXEL_N);
	pr_debug("pixel_m=0x%x, pixel_n=0x%x\n", pixel_m, pixel_n);

	mvid = (pixel_m & 0xFFFF) * 5;
	nvid = (0xFFFF & (~pixel_n)) + (pixel_m & 0xFFFF);

	pr_debug("rate = %d\n", rate);

	if (link_rate == rate)
		nvid *= 2;

	pr_debug("mvid=0x%x, nvid=0x%x\n", mvid, nvid);
	dp_write(base_ctrl + DP_SOFTWARE_MVID, mvid);
	dp_write(base_ctrl + DP_SOFTWARE_NVID, nvid);
}

static void dp_catalog_ctrl_set_pattern(struct dp_catalog_ctrl *ctrl,
					u32 pattern)
{
	int bit, cnt = 10;
	u32 data;
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	bit = 1;
	bit <<= (pattern - 1);
	pr_debug("hw: bit=%d train=%d\n", bit, pattern);
	dp_write(base + DP_STATE_CTRL, bit);

	bit = 8;
	bit <<= (pattern - 1);

	while (cnt--) {
		data = dp_read(base + DP_MAINLINK_READY);
		if (data & bit)
			break;
	}

	if (cnt == 0)
		pr_err("set link_train=%d failed\n", pattern);
}

static void dp_catalog_ctrl_reset(struct dp_catalog_ctrl *ctrl)
{
	u32 sw_reset;
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	sw_reset = dp_read(base + DP_SW_RESET);

	sw_reset |= BIT(0);
	dp_write(base + DP_SW_RESET, sw_reset);
	usleep_range(1000, 1010); /* h/w recommended delay */

	sw_reset &= ~BIT(0);
	dp_write(base + DP_SW_RESET, sw_reset);
}

static bool dp_catalog_ctrl_mainlink_ready(struct dp_catalog_ctrl *ctrl)
{
	u32 data;
	int cnt = 10;
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		goto end;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	while (--cnt) {
		/* DP_MAINLINK_READY */
		data = dp_read(base + DP_MAINLINK_READY);
		if (data & BIT(0))
			return true;

		usleep_range(1000, 1010); /* 1ms wait before next reg read */
	}
	pr_err("mainlink not ready\n");
end:
	return false;
}

static void dp_catalog_ctrl_enable_irq(struct dp_catalog_ctrl *ctrl,
						bool enable)
{
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	if (enable) {
		dp_write(base + DP_INTR_STATUS, DP_INTR_MASK1);
		dp_write(base + DP_INTR_STATUS2, DP_INTR_MASK2);
	} else {
		dp_write(base + DP_INTR_STATUS, 0x00);
		dp_write(base + DP_INTR_STATUS2, 0x00);
	}
}

static void dp_catalog_ctrl_hpd_config(struct dp_catalog_ctrl *ctrl, bool en)
{
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	if (en) {
		u32 reftimer = dp_read(base + DP_DP_HPD_REFTIMER);

		dp_write(base + DP_DP_HPD_INT_ACK, 0xF);
		dp_write(base + DP_DP_HPD_INT_MASK, 0xF);

		/* Enabling REFTIMER */
		reftimer |= BIT(16);
		dp_write(base + DP_DP_HPD_REFTIMER, 0xF);
		/* Enable HPD */
		dp_write(base + DP_DP_HPD_CTRL, 0x1);
	} else {
		/*Disable HPD */
		dp_write(base + DP_DP_HPD_CTRL, 0x0);
	}
}

static void dp_catalog_ctrl_get_interrupt(struct dp_catalog_ctrl *ctrl)
{
	u32 ack = 0;
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	ctrl->isr = dp_read(base + DP_INTR_STATUS2);
	ctrl->isr &= ~DP_INTR_MASK2;
	ack = ctrl->isr & DP_INTERRUPT_STATUS2;
	ack <<= 1;
	ack |= DP_INTR_MASK2;
	dp_write(base + DP_INTR_STATUS2, ack);
}

static void dp_catalog_ctrl_phy_reset(struct dp_catalog_ctrl *ctrl)
{
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base = catalog->io->ctrl_io.base;

	dp_write(base + DP_PHY_CTRL, 0x5); /* bit 0 & 2 */
	usleep_range(1000, 1010); /* h/w recommended delay */
	dp_write(base + DP_PHY_CTRL, 0x0);
	wmb(); /* make sure PHY reset done */
}

static void dp_catalog_ctrl_phy_lane_cfg(struct dp_catalog_ctrl *ctrl,
		bool flipped, u8 ln_cnt)
{
	u32 info = 0x0;
	struct dp_catalog_private *catalog;
	u8 orientation = BIT(!!flipped);

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);

	info |= (ln_cnt & 0x0F);
	info |= ((orientation & 0x0F) << 4);
	pr_debug("Shared Info = 0x%x\n", info);

	dp_write(catalog->io->phy_io.base + DP_PHY_SPARE0, info);
}

static void dp_catalog_ctrl_update_vx_px(struct dp_catalog_ctrl *ctrl,
		u8 v_level, u8 p_level)
{
	struct dp_catalog_private *catalog;
	void __iomem *base0, *base1;
	u8 value0, value1;

	if (!ctrl) {
		pr_err("invalid input\n");
		return;
	}

	dp_catalog_get_priv(ctrl);
	base0 = catalog->io->ln_tx0_io.base;
	base1 = catalog->io->ln_tx1_io.base;

	pr_debug("hw: v=%d p=%d\n", v_level, p_level);

	value0 = vm_voltage_swing[v_level][p_level];
	value1 = vm_pre_emphasis[v_level][p_level];

	/* program default setting first */
	dp_write(base0 + TXn_TX_DRV_LVL, 0x2A);
	dp_write(base1 + TXn_TX_DRV_LVL, 0x2A);
	dp_write(base0 + TXn_TX_EMP_POST1_LVL, 0x20);
	dp_write(base1 + TXn_TX_EMP_POST1_LVL, 0x20);

	/* Enable MUX to use Cursor values from these registers */
	value0 |= BIT(5);
	value1 |= BIT(5);

	/* Configure host and panel only if both values are allowed */
	if (value0 != 0xFF && value1 != 0xFF) {
		dp_write(base0 + TXn_TX_DRV_LVL, value0);
		dp_write(base1 + TXn_TX_DRV_LVL, value0);
		dp_write(base0 + TXn_TX_EMP_POST1_LVL, value1);
		dp_write(base1 + TXn_TX_EMP_POST1_LVL, value1);

		pr_debug("hw: vx_value=0x%x px_value=0x%x\n",
			value0, value1);
	} else {
		pr_err("invalid vx (0x%x=0x%x), px (0x%x=0x%x\n",
			v_level, value0, p_level, value1);
	}
}

/* panel related catalog functions */
static int dp_catalog_panel_timing_cfg(struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	void __iomem *base;

	if (!panel) {
		pr_err("invalid input\n");
		goto end;
	}

	dp_catalog_get_priv(panel);
	base = catalog->io->ctrl_io.base;

	dp_write(base + DP_TOTAL_HOR_VER, panel->total);
	dp_write(base + DP_START_HOR_VER_FROM_SYNC, panel->sync_start);
	dp_write(base + DP_HSYNC_VSYNC_WIDTH_POLARITY, panel->width_blanking);
	dp_write(base + DP_ACTIVE_HOR_VER, panel->dp_active);
end:
	return 0;
}
 /* audio related catalog functions */
static int dp_catalog_audio_acr_ctrl(struct dp_catalog_audio *audio)
{
	return 0;
}

static int dp_catalog_audio_stream_sdp(struct dp_catalog_audio *audio)
{
	return 0;
}

static int dp_catalog_audio_timestamp_sdp(struct dp_catalog_audio *audio)
{
	return 0;
}

static int dp_catalog_audio_infoframe_sdp(struct dp_catalog_audio *audio)
{
	return 0;
}

static int dp_catalog_audio_copy_mgmt_sdp(struct dp_catalog_audio *audio)
{
	return 0;
}

static int dp_catalog_audio_isrc_sdp(struct dp_catalog_audio *audio)
{
	return 0;
}

static int dp_catalog_audio_setup_sdp(struct dp_catalog_audio *audio)
{
	return 0;
}

struct dp_catalog *dp_catalog_get(struct device *dev, struct dp_io *io)
{
	int rc = 0;
	struct dp_catalog *dp_catalog;
	struct dp_catalog_private *catalog;
	struct dp_catalog_aux aux = {
		.read_data     = dp_catalog_aux_read_data,
		.write_data    = dp_catalog_aux_write_data,
		.write_trans   = dp_catalog_aux_write_trans,
		.reset         = dp_catalog_aux_reset,
		.enable        = dp_catalog_aux_enable,
		.setup         = dp_catalog_aux_setup,
		.get_irq       = dp_catalog_aux_get_irq,
	};
	struct dp_catalog_ctrl ctrl = {
		.state_ctrl     = dp_catalog_ctrl_state_ctrl,
		.config_ctrl    = dp_catalog_ctrl_config_ctrl,
		.lane_mapping   = dp_catalog_ctrl_lane_mapping,
		.mainlink_ctrl  = dp_catalog_ctrl_mainlink_ctrl,
		.config_misc    = dp_catalog_ctrl_config_misc,
		.config_msa     = dp_catalog_ctrl_config_msa,
		.set_pattern    = dp_catalog_ctrl_set_pattern,
		.reset          = dp_catalog_ctrl_reset,
		.mainlink_ready = dp_catalog_ctrl_mainlink_ready,
		.enable_irq     = dp_catalog_ctrl_enable_irq,
		.hpd_config     = dp_catalog_ctrl_hpd_config,
		.phy_reset      = dp_catalog_ctrl_phy_reset,
		.phy_lane_cfg   = dp_catalog_ctrl_phy_lane_cfg,
		.update_vx_px   = dp_catalog_ctrl_update_vx_px,
		.get_interrupt  = dp_catalog_ctrl_get_interrupt,
		.update_transfer_unit = dp_catalog_ctrl_update_transfer_unit,
		.read_hdcp_status     = dp_catalog_ctrl_read_hdcp_status,
	};
	struct dp_catalog_audio audio = {
		.acr_ctrl      = dp_catalog_audio_acr_ctrl,
		.stream_sdp    = dp_catalog_audio_stream_sdp,
		.timestamp_sdp = dp_catalog_audio_timestamp_sdp,
		.infoframe_sdp = dp_catalog_audio_infoframe_sdp,
		.copy_mgmt_sdp = dp_catalog_audio_copy_mgmt_sdp,
		.isrc_sdp      = dp_catalog_audio_isrc_sdp,
		.setup_sdp     = dp_catalog_audio_setup_sdp,
	};
	struct dp_catalog_panel panel = {
		.timing_cfg = dp_catalog_panel_timing_cfg,
	};

	if (!io) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto error;
	}

	catalog  = devm_kzalloc(dev, sizeof(*catalog), GFP_KERNEL);
	if (!catalog) {
		rc = -ENOMEM;
		goto error;
	}

	catalog->dev = dev;
	catalog->io = io;

	dp_catalog = &catalog->dp_catalog;

	dp_catalog->aux   = aux;
	dp_catalog->ctrl  = ctrl;
	dp_catalog->audio = audio;
	dp_catalog->panel = panel;

	return dp_catalog;
error:
	return ERR_PTR(rc);
}

void dp_catalog_put(struct dp_catalog *dp_catalog)
{
	struct dp_catalog_private *catalog;

	if (!dp_catalog)
		return;

	catalog = container_of(dp_catalog, struct dp_catalog_private,
				dp_catalog);

	devm_kfree(catalog->dev, catalog);
}
