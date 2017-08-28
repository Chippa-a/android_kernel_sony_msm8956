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

#define DS_PORT_STATUS_CHANGED 0x200
#define DP_TEST_BIT_DEPTH_UNKNOWN 0xFFFFFFFF

struct dp_link_sink_count {
	u32 count;
	bool cp_ready;
};

struct dp_link {
	u32 test_requested;

	u32 lane_count;
	u32 bw_code;
	u32 v_level;
	u32 p_level;

	struct dp_link_sink_count sink_count;

	u32 (*get_test_bits_depth)(struct dp_link *dp_link, u32 bpp);
	int (*process_request)(struct dp_link *dp_link);
	int (*get_sink_count)(struct dp_link *dp_link);
	int (*get_colorimetry_config)(struct dp_link *dp_link);
	int (*adjust_levels)(struct dp_link *dp_link, u8 *link_status);
	int (*send_psm_request)(struct dp_link *dp_link, bool req);
	bool (*phy_pattern_requested)(struct dp_link *dp_link);
};

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
