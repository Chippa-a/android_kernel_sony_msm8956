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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/slab.h>
#include "cam_cpas_api.h"
#include "cam_vfe_soc.h"

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

static int cam_vfe_get_dt_properties(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	rc = cam_soc_util_get_dt_properties(soc_info);
	if (rc) {
		pr_err("Error! get DT properties failed\n");
		return rc;
	}

	return rc;
}

static int cam_vfe_request_platform_resource(
	struct cam_hw_soc_info *soc_info,
	irq_handler_t vfe_irq_handler, void *irq_data)
{
	int rc = 0;

	rc = cam_soc_util_request_platform_resource(soc_info, vfe_irq_handler,
		irq_data);

	return rc;
}

int cam_vfe_init_soc_resources(struct cam_hw_soc_info *soc_info,
	irq_handler_t vfe_irq_handler, void *irq_data)
{
	int                               rc = 0;
	struct cam_vfe_soc_private       *soc_private;
	struct cam_cpas_register_params   cpas_register_param;

	soc_private = kzalloc(sizeof(struct cam_vfe_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		CDBG("Error! soc_private Alloc Failed\n");
		return -ENOMEM;
	}
	soc_info->soc_private = soc_private;

	rc = cam_vfe_get_dt_properties(soc_info);
	if (rc < 0) {
		pr_err("Error! Get DT properties failed\n");
		goto free_soc_private;
	}

	rc = cam_vfe_request_platform_resource(soc_info, vfe_irq_handler,
		irq_data);
	if (rc < 0) {
		pr_err("Error! Request platform resources failed\n");
		goto free_soc_private;
	}

	memset(&cpas_register_param, 0, sizeof(cpas_register_param));
	strlcpy(cpas_register_param.identifier, "ife",
		CAM_HW_IDENTIFIER_LENGTH);
	cpas_register_param.cell_index = soc_info->index;
	cpas_register_param.dev = &soc_info->pdev->dev;
	rc = cam_cpas_register_client(&cpas_register_param);
	if (rc) {
		pr_err("CPAS registration failed\n");
		goto release_soc;
	} else {
		soc_private->cpas_handle = cpas_register_param.client_handle;
	}

	return rc;

release_soc:
	cam_soc_util_release_platform_resource(soc_info);
free_soc_private:
	kfree(soc_private);

	return rc;
}

int cam_vfe_enable_soc_resources(struct cam_hw_soc_info *soc_info)
{
	int                               rc = 0;
	struct cam_vfe_soc_private       *soc_private;
	struct cam_ahb_vote               ahb_vote;
	struct cam_axi_vote               axi_vote;

	if (!soc_info) {
		pr_err("Error! Invalid params\n");
		rc = -EINVAL;
		goto end;
	}
	soc_private = soc_info->soc_private;

	ahb_vote.type       = CAM_VOTE_ABSOLUTE;
	ahb_vote.vote.level = CAM_SVS_VOTE;

	axi_vote.compressed_bw   = 10640000000L;
	axi_vote.uncompressed_bw = 10640000000L;

	rc = cam_cpas_start(soc_private->cpas_handle, &ahb_vote, &axi_vote);
	if (rc) {
		pr_err("Error! CPAS start failed.\n");
		rc = -EFAULT;
		goto end;
	}

	rc = cam_soc_util_enable_platform_resource(soc_info, true,
		CAM_TURBO_VOTE, true);
	if (rc) {
		pr_err("Error! enable platform failed\n");
		goto stop_cpas;
	}

	return rc;

stop_cpas:
	cam_cpas_stop(soc_private->cpas_handle);
end:
	return rc;
}


int cam_vfe_disable_soc_resources(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;
	struct cam_vfe_soc_private       *soc_private;

	if (!soc_info) {
		pr_err("Error! Invalid params\n");
		rc = -EINVAL;
		return rc;
	}
	soc_private = soc_info->soc_private;

	rc = cam_soc_util_disable_platform_resource(soc_info, true, true);
	if (rc) {
		pr_err("%s: disable platform failed\n", __func__);
		return rc;
	}

	rc = cam_cpas_stop(soc_private->cpas_handle);
	if (rc) {
		pr_err("Error! CPAS stop failed.\n");
		return rc;
	}

	return rc;
}
