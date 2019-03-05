/* Copyright (c) 2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include "board-dt.h"
#include <asm/mach/map.h>
#include <asm/mach/arch.h>

static const char *msm8976_dt_match[] __initconst = {
	"qcom,msm8976",
	"qcom,apq8076",
	NULL
};

static const char *msm8956_dt_match[] __initconst = {
	"qcom,msm8956",
	"qcom,apq8056",
	NULL
};

static void __init msm8976_init(void)
{
	board_dt_populate(NULL);
}

DT_MACHINE_START(MSM8976_DT,
	"Qualcomm Technologies, Inc. MSM 8976 (Flattened Device Tree)")
	.init_machine = msm8976_init,
	.dt_compat = msm8976_dt_match,
MACHINE_END

DT_MACHINE_START(MSM8956_DT,
	"Qualcomm Technologies, Inc. MSM 8956 (Flattened Device Tree)")
	.init_machine = msm8976_init,
	.dt_compat = msm8956_dt_match,
MACHINE_END
