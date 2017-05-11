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

#ifndef _CAM_IFE_CSID_SOC_H_
#define _CAM_IFE_CSID_SOC_H_

#include "cam_isp_hw.h"

/**
 * struct csid_device_soc_info - CSID SOC info object
 *
 * @csi_vdd_voltage:       csi vdd voltage value
 *
 */
struct csid_device_soc_info {
	int                             csi_vdd_voltage;
};

/**
 * cam_ife_csid_init_soc_resources()
 *
 * @brief:                 csid initialization function for the soc info
 *
 * @soc_info:              soc info structure pointer
 * @csid_irq_handler:      irq handler function to be registered
 * @irq_data:              irq data for the callback function
 *
 */
int cam_ife_csid_init_soc_resources(struct cam_hw_soc_info *soc_info,
	irq_handler_t csid_irq_handler, void *irq_data);

/**
 * cam_ife_csid_enable_soc_resources()
 *
 * @brief:                 csid soc resource enable function
 *
 * @soc_info:              soc info structure pointer
 *
 */
int cam_ife_csid_enable_soc_resources(struct cam_hw_soc_info  *soc_info);

/**
 * cam_ife_csid_disable_soc_resources()
 *
 * @brief:                 csid soc resource disable function
 *
 * @soc_info:              soc info structure pointer
 *
 */
int cam_ife_csid_disable_soc_resources(struct cam_hw_soc_info *soc_info);

#endif
