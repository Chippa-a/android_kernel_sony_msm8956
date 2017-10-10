/*
 * Copyright (c) 2012-2017, The Linux Foundation. All rights reserved.
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

#include "dp_panel.h"

#define DP_PANEL_DEFAULT_BPP 24
#define DP_MAX_DS_PORT_COUNT 1

enum {
	DP_LINK_RATE_MULTIPLIER = 27000000,
};

struct dp_panel_private {
	struct device *dev;
	struct dp_panel dp_panel;
	struct dp_aux *aux;
	struct dp_link *link;
	struct dp_catalog_panel *catalog;
	bool aux_cfg_update_done;
};

static const struct dp_panel_info fail_safe = {
	.h_active = 640,
	.v_active = 480,
	.h_back_porch = 48,
	.h_front_porch = 16,
	.h_sync_width = 96,
	.h_active_low = 0,
	.v_back_porch = 33,
	.v_front_porch = 10,
	.v_sync_width = 2,
	.v_active_low = 0,
	.h_skew = 0,
	.refresh_rate = 60,
	.pixel_clk_khz = 25200,
	.bpp = 24,
};

static int dp_panel_read_dpcd(struct dp_panel *dp_panel)
{
	int rlen, rc = 0;
	struct dp_panel_private *panel;
	struct drm_dp_link *link_info;
	u8 *dpcd, major = 0, minor = 0;
	u32 dfp_count = 0;
	unsigned long caps = DP_LINK_CAP_ENHANCED_FRAMING;

	if (!dp_panel) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto end;
	}

	dpcd = dp_panel->dpcd;

	panel = container_of(dp_panel, struct dp_panel_private, dp_panel);
	link_info = &dp_panel->link_info;

	rlen = drm_dp_dpcd_read(panel->aux->drm_aux, DP_DPCD_REV,
		dpcd, (DP_RECEIVER_CAP_SIZE + 1));
	if (rlen < (DP_RECEIVER_CAP_SIZE + 1)) {
		pr_err("dpcd read failed, rlen=%d\n", rlen);
		rc = -EINVAL;
		goto end;
	}

	link_info->revision = dp_panel->dpcd[DP_DPCD_REV];

	major = (link_info->revision >> 4) & 0x0f;
	minor = link_info->revision & 0x0f;
	pr_debug("version: %d.%d\n", major, minor);

	link_info->rate =
		drm_dp_bw_code_to_link_rate(dp_panel->dpcd[DP_MAX_LINK_RATE]);
	pr_debug("link_rate=%d\n", link_info->rate);

	link_info->num_lanes = dp_panel->dpcd[DP_MAX_LANE_COUNT] &
				DP_MAX_LANE_COUNT_MASK;

	pr_debug("lane_count=%d\n", link_info->num_lanes);

	if (drm_dp_enhanced_frame_cap(dpcd))
		link_info->capabilities |= caps;

	dfp_count = dpcd[DP_DOWN_STREAM_PORT_COUNT] &
						DP_DOWN_STREAM_PORT_COUNT;

	if ((dpcd[DP_DOWNSTREAMPORT_PRESENT] & DP_DWN_STRM_PORT_PRESENT)
		&& (dpcd[DP_DPCD_REV] > 0x10)) {
		rlen = drm_dp_dpcd_read(panel->aux->drm_aux,
			DP_DOWNSTREAM_PORT_0, dp_panel->ds_ports,
			DP_MAX_DOWNSTREAM_PORTS);
		if (rlen < DP_MAX_DOWNSTREAM_PORTS) {
			pr_err("ds port status failed, rlen=%d\n", rlen);
			rc = -EINVAL;
			goto end;
		}
	}

	if (dfp_count > DP_MAX_DS_PORT_COUNT)
		pr_debug("DS port count %d greater that max (%d) supported\n",
			dfp_count, DP_MAX_DS_PORT_COUNT);

end:
	return rc;
}

static int dp_panel_set_default_link_params(struct dp_panel *dp_panel)
{
	struct drm_dp_link *link_info;
	const int default_bw_code = 162000;
	const int default_num_lanes = 1;

	if (!dp_panel) {
		pr_err("invalid input\n");
		return -EINVAL;
	}
	link_info = &dp_panel->link_info;
	link_info->rate = default_bw_code;
	link_info->num_lanes = default_num_lanes;
	pr_debug("link_rate=%d num_lanes=%d\n",
		link_info->rate, link_info->num_lanes);
	return 0;
}

static int dp_panel_read_edid(struct dp_panel *dp_panel,
	struct drm_connector *connector)
{
	int retry_cnt = 0;
	const int max_retry = 10;
	struct dp_panel_private *panel;

	if (!dp_panel) {
		pr_err("invalid input\n");
		return -EINVAL;
	}

	panel = container_of(dp_panel, struct dp_panel_private, dp_panel);

	do {
		sde_get_edid(connector, &panel->aux->drm_aux->ddc,
			(void **)&dp_panel->edid_ctrl);
		if (!dp_panel->edid_ctrl->edid) {
			pr_err("EDID read failed\n");
			retry_cnt++;
			panel->aux->reconfig(panel->aux);
			panel->aux_cfg_update_done = true;
		} else {
			return 0;
		}
	} while (retry_cnt < max_retry);

	return -EINVAL;
}

static int dp_panel_read_sink_caps(struct dp_panel *dp_panel,
	struct drm_connector *connector)
{
	int rc = 0;
	struct dp_panel_private *panel;

	if (!dp_panel || !connector) {
		pr_err("invalid input\n");
		return -EINVAL;
	}

	panel = container_of(dp_panel, struct dp_panel_private, dp_panel);

	rc = dp_panel_read_dpcd(dp_panel);
	if (rc || !is_link_rate_valid(drm_dp_link_rate_to_bw_code(
		dp_panel->link_info.rate)) || !is_lane_count_valid(
		dp_panel->link_info.num_lanes) ||
		((drm_dp_link_rate_to_bw_code(dp_panel->link_info.rate)) >
		dp_panel->max_bw_code)) {
		pr_err("panel dpcd read failed/incorrect, set default params\n");
		dp_panel_set_default_link_params(dp_panel);
	}

	rc = dp_panel_read_edid(dp_panel, connector);
	if (rc) {
		pr_err("panel edid read failed, set failsafe mode\n");
		return rc;
	}

	if (panel->aux_cfg_update_done) {
		pr_debug("read DPCD with updated AUX config\n");
		dp_panel_read_dpcd(dp_panel);
		panel->aux_cfg_update_done = false;
	}

	return 0;
}

static u32 dp_panel_get_max_pclk(struct dp_panel *dp_panel)
{
	struct drm_dp_link *link_info;
	const u8 num_components = 3;
	u32 bpc = 0, bpp = 0, max_data_rate_khz = 0, max_pclk_rate_khz = 0;

	if (!dp_panel) {
		pr_err("invalid input\n");
		return 0;
	}

	link_info = &dp_panel->link_info;

	bpc = sde_get_sink_bpc(dp_panel->edid_ctrl);
	bpp = bpc * num_components;
	if (!bpp)
		bpp = DP_PANEL_DEFAULT_BPP;

	max_data_rate_khz = (link_info->num_lanes * link_info->rate * 8);
	max_pclk_rate_khz = max_data_rate_khz / bpp;

	pr_debug("bpp=%d, max_lane_cnt=%d\n", bpp, link_info->num_lanes);
	pr_debug("max_data_rate=%dKHz, max_pclk_rate=%dKHz\n",
		max_data_rate_khz, max_pclk_rate_khz);

	return max_pclk_rate_khz;
}

static void dp_panel_set_test_mode(struct dp_panel_private *panel,
		struct dp_display_mode *mode)
{
	struct dp_panel_info *pinfo = NULL;
	struct dp_link_test_video *test_info = NULL;

	if (!panel) {
		pr_err("invalid params\n");
		return;
	}

	pinfo = &mode->timing;
	test_info = &panel->link->test_video;

	pinfo->h_active = test_info->test_h_width;
	pinfo->h_sync_width = test_info->test_hsync_width;
	pinfo->h_back_porch = test_info->test_h_start -
		test_info->test_hsync_width;
	pinfo->h_front_porch = test_info->test_h_total -
		(test_info->test_h_start + test_info->test_h_width);

	pinfo->v_active = test_info->test_v_height;
	pinfo->v_sync_width = test_info->test_vsync_width;
	pinfo->v_back_porch = test_info->test_v_start -
		test_info->test_vsync_width;
	pinfo->v_front_porch = test_info->test_v_total -
		(test_info->test_v_start + test_info->test_v_height);

	pinfo->bpp = dp_link_bit_depth_to_bpp(test_info->test_bit_depth);
	pinfo->h_active_low = test_info->test_hsync_pol;
	pinfo->v_active_low = test_info->test_vsync_pol;

	pinfo->refresh_rate = test_info->test_rr_n;
	pinfo->pixel_clk_khz = test_info->test_h_total *
		test_info->test_v_total * pinfo->refresh_rate;

	if (test_info->test_rr_d == 0)
		pinfo->pixel_clk_khz /= 1000;
	else
		pinfo->pixel_clk_khz /= 1001;

	if (test_info->test_h_width == 640)
		pinfo->pixel_clk_khz = 25170;
}

static int dp_panel_get_modes(struct dp_panel *dp_panel,
	struct drm_connector *connector, struct dp_display_mode *mode)
{
	struct dp_panel_private *panel;

	if (!dp_panel) {
		pr_err("invalid input\n");
		return -EINVAL;
	}

	panel = container_of(dp_panel, struct dp_panel_private, dp_panel);

	if (dp_panel->video_test) {
		dp_panel_set_test_mode(panel, mode);
		return 1;
	} else if (dp_panel->edid_ctrl->edid) {
		return _sde_edid_update_modes(connector, dp_panel->edid_ctrl);
	} else { /* fail-safe mode */
		memcpy(&mode->timing, &fail_safe,
			sizeof(fail_safe));
		return 1;
	}
}

static void dp_panel_handle_sink_request(struct dp_panel *dp_panel)
{
	struct dp_panel_private *panel;

	if (!dp_panel) {
		pr_err("invalid input\n");
		return;
	}

	panel = container_of(dp_panel, struct dp_panel_private, dp_panel);

	if (panel->link->sink_request & DP_TEST_LINK_EDID_READ) {
		u8 checksum = sde_get_edid_checksum(dp_panel->edid_ctrl);

		panel->link->send_edid_checksum(panel->link, checksum);
		panel->link->send_test_response(panel->link);
	}
}

static int dp_panel_timing_cfg(struct dp_panel *dp_panel)
{
	int rc = 0;
	u32 data, total_ver, total_hor;
	struct dp_catalog_panel *catalog;
	struct dp_panel_private *panel;
	struct dp_panel_info *pinfo;

	if (!dp_panel) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto end;
	}

	panel = container_of(dp_panel, struct dp_panel_private, dp_panel);
	catalog = panel->catalog;
	pinfo = &panel->dp_panel.pinfo;

	pr_debug("width=%d hporch= %d %d %d\n",
		pinfo->h_active, pinfo->h_back_porch,
		pinfo->h_front_porch, pinfo->h_sync_width);

	pr_debug("height=%d vporch= %d %d %d\n",
		pinfo->v_active, pinfo->v_back_porch,
		pinfo->v_front_porch, pinfo->v_sync_width);

	total_hor = pinfo->h_active + pinfo->h_back_porch +
		pinfo->h_front_porch + pinfo->h_sync_width;

	total_ver = pinfo->v_active + pinfo->v_back_porch +
			pinfo->v_front_porch + pinfo->v_sync_width;

	data = total_ver;
	data <<= 16;
	data |= total_hor;

	catalog->total = data;

	data = (pinfo->v_back_porch + pinfo->v_sync_width);
	data <<= 16;
	data |= (pinfo->h_back_porch + pinfo->h_sync_width);

	catalog->sync_start = data;

	data = pinfo->v_sync_width;
	data <<= 16;
	data |= (pinfo->v_active_low << 31);
	data |= pinfo->h_sync_width;
	data |= (pinfo->h_active_low << 15);

	catalog->width_blanking = data;

	data = pinfo->v_active;
	data <<= 16;
	data |= pinfo->h_active;

	catalog->dp_active = data;

	panel->catalog->timing_cfg(catalog);
end:
	return rc;
}

static int dp_panel_edid_register(struct dp_panel *dp_panel)
{
	int rc = 0;

	if (!dp_panel) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto end;
	}

	dp_panel->edid_ctrl = sde_edid_init();
	if (!dp_panel->edid_ctrl) {
		pr_err("sde edid init for DP failed\n");
		rc = -ENOMEM;
		goto end;
	}
end:
	return rc;
}

static void dp_panel_edid_deregister(struct dp_panel *dp_panel)
{
	if (!dp_panel) {
		pr_err("invalid input\n");
		return;
	}

	sde_edid_deinit((void **)&dp_panel->edid_ctrl);
}

static int dp_panel_init_panel_info(struct dp_panel *dp_panel)
{
	int rc = 0;
	struct dp_panel_info *pinfo;

	if (!dp_panel) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto end;
	}

	pinfo = &dp_panel->pinfo;

	/*
	 * print resolution info as this is a result
	 * of user initiated action of cable connection
	 */
	pr_info("SET NEW RESOLUTION:\n");
	pr_info("%dx%d@%dfps\n", pinfo->h_active,
		pinfo->v_active, pinfo->refresh_rate);
	pr_info("h_porches(back|front|width) = (%d|%d|%d)\n",
			pinfo->h_back_porch,
			pinfo->h_front_porch,
			pinfo->h_sync_width);
	pr_info("v_porches(back|front|width) = (%d|%d|%d)\n",
			pinfo->v_back_porch,
			pinfo->v_front_porch,
			pinfo->v_sync_width);
	pr_info("pixel clock (KHz)=(%d)\n", pinfo->pixel_clk_khz);
	pr_info("bpp = %d\n", pinfo->bpp);
	pr_info("active low (h|v)=(%d|%d)\n", pinfo->h_active_low,
		pinfo->v_active_low);

	pinfo->bpp = max_t(u32, 18, min_t(u32, pinfo->bpp, 30));
	pr_info("updated bpp = %d\n", pinfo->bpp);
end:
	return rc;
}

static u32 dp_panel_get_min_req_link_rate(struct dp_panel *dp_panel)
{
	const u32 encoding_factx10 = 8;
	u32 min_link_rate_khz = 0, lane_cnt;
	struct dp_panel_info *pinfo;

	if (!dp_panel) {
		pr_err("invalid input\n");
		goto end;
	}

	lane_cnt = dp_panel->link_info.num_lanes;
	pinfo = &dp_panel->pinfo;

	/* num_lanes * lane_count * 8 >= pclk * bpp * 10 */
	min_link_rate_khz = pinfo->pixel_clk_khz /
				(lane_cnt * encoding_factx10);
	min_link_rate_khz *= pinfo->bpp;

	pr_debug("min lclk req=%d khz for pclk=%d khz, lanes=%d, bpp=%d\n",
		min_link_rate_khz, pinfo->pixel_clk_khz, lane_cnt,
		pinfo->bpp);
end:
	return min_link_rate_khz;
}

struct dp_panel *dp_panel_get(struct dp_panel_in *in)
{
	int rc = 0;
	struct dp_panel_private *panel;
	struct dp_panel *dp_panel;

	if (!in->dev || !in->catalog || !in->aux || !in->link) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto error;
	}

	panel = devm_kzalloc(in->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel) {
		rc = -ENOMEM;
		goto error;
	}

	panel->dev = in->dev;
	panel->aux = in->aux;
	panel->catalog = in->catalog;
	panel->link = in->link;

	dp_panel = &panel->dp_panel;
	panel->aux_cfg_update_done = false;
	dp_panel->max_bw_code = DP_LINK_BW_8_1;

	dp_panel->sde_edid_register = dp_panel_edid_register;
	dp_panel->sde_edid_deregister = dp_panel_edid_deregister;
	dp_panel->init_info = dp_panel_init_panel_info;
	dp_panel->timing_cfg = dp_panel_timing_cfg;
	dp_panel->read_sink_caps = dp_panel_read_sink_caps;
	dp_panel->get_min_req_link_rate = dp_panel_get_min_req_link_rate;
	dp_panel->get_max_pclk = dp_panel_get_max_pclk;
	dp_panel->get_modes = dp_panel_get_modes;
	dp_panel->handle_sink_request = dp_panel_handle_sink_request;

	return dp_panel;
error:
	return ERR_PTR(rc);
}

void dp_panel_put(struct dp_panel *dp_panel)
{
	struct dp_panel_private *panel;

	if (!dp_panel)
		return;

	panel = container_of(dp_panel, struct dp_panel_private, dp_panel);

	devm_kfree(panel->dev, panel);
}
