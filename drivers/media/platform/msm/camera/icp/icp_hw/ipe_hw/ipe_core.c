/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "IPE-CORE %s:%d " fmt, __func__, __LINE__

#include <linux/of.h>
#include <linux/debugfs.h>
#include <linux/videodev2.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include "cam_io_util.h"
#include "cam_hw.h"
#include "cam_hw_intf.h"
#include "ipe_core.h"
#include "ipe_soc.h"
#include "cam_soc_util.h"
#include "cam_io_util.h"
#include "cam_ipe_hw_intf.h"
#include "cam_icp_hw_mgr_intf.h"
#include "cam_cpas_api.h"

static int cam_ipe_caps_vote(struct cam_ipe_device_core_info *core_info,
	struct cam_icp_cpas_vote *cpas_vote)
{
	int rc = 0;

	if (cpas_vote->ahb_vote_valid)
		rc = cam_cpas_update_ahb_vote(core_info->cpas_handle,
			&cpas_vote->ahb_vote);
	if (cpas_vote->axi_vote_valid)
		rc = cam_cpas_update_axi_vote(core_info->cpas_handle,
			&cpas_vote->axi_vote);

	if (rc < 0)
		pr_err("cpas vote is failed: %d\n", rc);

	return rc;
}

int cam_ipe_init_hw(void *device_priv,
	void *init_hw_args, uint32_t arg_size)
{
	struct cam_hw_info *ipe_dev = device_priv;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_ipe_device_core_info *core_info = NULL;
	struct cam_icp_cpas_vote cpas_vote;
	int rc = 0;

	if (!device_priv) {
		pr_err("Invalid cam_dev_info\n");
		return -EINVAL;
	}

	soc_info = &ipe_dev->soc_info;
	core_info = (struct cam_ipe_device_core_info *)ipe_dev->core_info;

	if ((!soc_info) || (!core_info)) {
		pr_err("soc_info = %pK core_info = %pK\n", soc_info, core_info);
		return -EINVAL;
	}

	cpas_vote.ahb_vote.type = CAM_VOTE_ABSOLUTE;
	cpas_vote.ahb_vote.vote.level = CAM_TURBO_VOTE;
	cpas_vote.axi_vote.compressed_bw = ICP_TURBO_VOTE;
	cpas_vote.axi_vote.uncompressed_bw = ICP_TURBO_VOTE;

	rc = cam_cpas_start(core_info->cpas_handle,
		&cpas_vote.ahb_vote, &cpas_vote.axi_vote);
	if (rc < 0) {
		pr_err("cpass start failed: %d\n", rc);
		return rc;
	}

	rc = cam_ipe_enable_soc_resources(soc_info);
	if (rc < 0) {
		pr_err("soc enable is failed\n");
		rc = cam_cpas_stop(core_info->cpas_handle);
	}

	return rc;
}

int cam_ipe_deinit_hw(void *device_priv,
	void *init_hw_args, uint32_t arg_size)
{
	struct cam_hw_info *ipe_dev = device_priv;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_ipe_device_core_info *core_info = NULL;
	int rc = 0;

	if (!device_priv) {
		pr_err("Invalid cam_dev_info\n");
		return -EINVAL;
	}

	soc_info = &ipe_dev->soc_info;
	core_info = (struct cam_ipe_device_core_info *)ipe_dev->core_info;
	if ((!soc_info) || (!core_info)) {
		pr_err("soc_info = %pK core_info = %pK\n", soc_info, core_info);
		return -EINVAL;
	}

	rc = cam_ipe_disable_soc_resources(soc_info);
	if (rc < 0)
		pr_err("soc enable is failed\n");

	rc = cam_cpas_stop(core_info->cpas_handle);
	if (rc < 0)
		pr_err("cpas stop is failed: %d\n", rc);

	return rc;
}

int cam_ipe_process_cmd(void *device_priv, uint32_t cmd_type,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_hw_info *ipe_dev = device_priv;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_ipe_device_core_info *core_info = NULL;
	struct cam_ipe_device_hw_info *hw_info = NULL;
	int rc = 0;

	if (!device_priv) {
		pr_err("Invalid arguments\n");
		return -EINVAL;
	}

	if (cmd_type >= CAM_ICP_IPE_CMD_MAX) {
		pr_err("Invalid command : %x\n", cmd_type);
		return -EINVAL;
	}

	soc_info = &ipe_dev->soc_info;
	core_info = (struct cam_ipe_device_core_info *)ipe_dev->core_info;
	hw_info = core_info->ipe_hw_info;

	switch (cmd_type) {
	case CAM_ICP_IPE_CMD_VOTE_CPAS: {
		struct cam_icp_cpas_vote *cpas_vote = cmd_args;

		if (!cmd_args)
			return -EINVAL;

		cam_ipe_caps_vote(core_info, cpas_vote);
		break;
	}

	case CAM_ICP_IPE_CMD_CPAS_START: {
		struct cam_icp_cpas_vote *cpas_vote = cmd_args;

		if (!cmd_args)
			return -EINVAL;

		rc = cam_cpas_start(core_info->cpas_handle,
			&cpas_vote->ahb_vote, &cpas_vote->axi_vote);
		break;
	}

	case CAM_ICP_IPE_CMD_CPAS_STOP:
		cam_cpas_stop(core_info->cpas_handle);
		break;
	default:
		break;
	}
	return rc;
}

irqreturn_t cam_ipe_irq(int irq_num, void *data)
{
	return IRQ_HANDLED;
}
