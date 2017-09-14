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

#ifndef _DP_LINK_H_
#define _DP_LINK_H_

#include "dp_aux.h"

#define DS_PORT_STATUS_CHANGED 0x200
#define DP_TEST_BIT_DEPTH_UNKNOWN 0xFFFFFFFF
#define DP_LINK_ENUM_STR(x)		#x

enum dp_link_voltage_level {
	DP_LINK_VOLTAGE_LEVEL_0	= 0,
	DP_LINK_VOLTAGE_LEVEL_1	= 1,
	DP_LINK_VOLTAGE_LEVEL_2	= 2,
	DP_LINK_VOLTAGE_MAX	= DP_LINK_VOLTAGE_LEVEL_2,
};

enum dp_link_preemaphasis_level {
	DP_LINK_PRE_EMPHASIS_LEVEL_0	= 0,
	DP_LINK_PRE_EMPHASIS_LEVEL_1	= 1,
	DP_LINK_PRE_EMPHASIS_LEVEL_2	= 2,
	DP_LINK_PRE_EMPHASIS_MAX	= DP_LINK_PRE_EMPHASIS_LEVEL_2,
};

struct dp_link_sink_count {
	u32 count;
	bool cp_ready;
};

struct dp_link_test_video {
	u32 test_video_pattern;
	u32 test_bit_depth;
	u32 test_dyn_range;
	u32 test_h_total;
	u32 test_v_total;
	u32 test_h_start;
	u32 test_v_start;
	u32 test_hsync_pol;
	u32 test_hsync_width;
	u32 test_vsync_pol;
	u32 test_vsync_width;
	u32 test_h_width;
	u32 test_v_height;
	u32 test_rr_d;
	u32 test_rr_n;
};

struct dp_link_test_audio {
	u32 test_audio_sampling_rate;
	u32 test_audio_channel_count;
	u32 test_audio_pattern_type;
	u32 test_audio_period_ch_1;
	u32 test_audio_period_ch_2;
	u32 test_audio_period_ch_3;
	u32 test_audio_period_ch_4;
	u32 test_audio_period_ch_5;
	u32 test_audio_period_ch_6;
	u32 test_audio_period_ch_7;
	u32 test_audio_period_ch_8;
};

struct dp_link_phy_params {
	u32 phy_test_pattern_sel;
	u8 v_level;
	u8 p_level;
};

struct dp_link_params {
	u32 lane_count;
	u32 bw_code;
};

struct dp_link {
	u32 sink_request;
	u32 test_response;

	struct dp_link_sink_count sink_count;
	struct dp_link_test_video test_video;
	struct dp_link_test_audio test_audio;
	struct dp_link_phy_params phy_params;
	struct dp_link_params link_params;

	u32 (*get_test_bits_depth)(struct dp_link *dp_link, u32 bpp);
	int (*process_request)(struct dp_link *dp_link);
	int (*get_colorimetry_config)(struct dp_link *dp_link);
	int (*adjust_levels)(struct dp_link *dp_link, u8 *link_status);
	int (*send_psm_request)(struct dp_link *dp_link, bool req);
	void (*send_test_response)(struct dp_link *dp_link);
};

static inline char *dp_link_get_phy_test_pattern(u32 phy_test_pattern_sel)
{
	switch (phy_test_pattern_sel) {
	case DP_TEST_PHY_PATTERN_NONE:
		return DP_LINK_ENUM_STR(DP_TEST_PHY_PATTERN_NONE);
	case DP_TEST_PHY_PATTERN_D10_2_NO_SCRAMBLING:
		return DP_LINK_ENUM_STR(
			DP_TEST_PHY_PATTERN_D10_2_NO_SCRAMBLING);
	case DP_TEST_PHY_PATTERN_SYMBOL_ERR_MEASUREMENT_CNT:
		return DP_LINK_ENUM_STR(
			DP_TEST_PHY_PATTERN_SYMBOL_ERR_MEASUREMENT_CNT);
	case DP_TEST_PHY_PATTERN_PRBS7:
		return DP_LINK_ENUM_STR(DP_TEST_PHY_PATTERN_PRBS7);
	case DP_TEST_PHY_PATTERN_80_BIT_CUSTOM_PATTERN:
		return DP_LINK_ENUM_STR(
			DP_TEST_PHY_PATTERN_80_BIT_CUSTOM_PATTERN);
	case DP_TEST_PHY_PATTERN_HBR2_CTS_EYE_PATTERN:
		return DP_LINK_ENUM_STR(
			DP_TEST_PHY_PATTERN_HBR2_CTS_EYE_PATTERN);
	default:
		return "unknown";
	}
}

/**
 * dp_link_get() - get the functionalities of dp test module
 *
 *
 * return: a pointer to dp_link struct
 */
struct dp_link *dp_link_get(struct device *dev, struct dp_aux *aux);

/**
 * dp_link_put() - releases the dp test module's resources
 *
 * @dp_link: an instance of dp_link module
 *
 */
void dp_link_put(struct dp_link *dp_link);

#endif /* _DP_LINK_H_ */
