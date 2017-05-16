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

#ifndef CAM_IPE_CORE_H
#define CAM_IPE_CORE_H

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/dma-buf.h>

struct cam_ipe_device_hw_info {
	uint32_t reserved;
};

struct cam_ipe_device_core_info {
	struct cam_ipe_device_hw_info *ipe_hw_info;
	uint32_t cpas_handle;
};

int cam_ipe_init_hw(void *device_priv,
	void *init_hw_args, uint32_t arg_size);
int cam_ipe_deinit_hw(void *device_priv,
	void *init_hw_args, uint32_t arg_size);
int cam_ipe_process_cmd(void *device_priv, uint32_t cmd_type,
	void *cmd_args, uint32_t arg_size);
irqreturn_t cam_ipe_irq(int irq_num, void *data);

#endif /* CAM_IPE_CORE_H */
