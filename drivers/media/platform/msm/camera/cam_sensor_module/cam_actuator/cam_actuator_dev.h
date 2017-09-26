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

#ifndef _CAM_ACTUATOR_DEV_H_
#define _CAM_ACTUATOR_DEV_H_

#include <cam_sensor_io.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/irqreturn.h>
#include <linux/ion.h>
#include <linux/iommu.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <cam_cci_dev.h>
#include <cam_sensor_cmn_header.h>
#include <cam_subdev.h>
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_debug_util.h"

#define NUM_MASTERS 2
#define NUM_QUEUES 2

#define TRUE  1
#define FALSE 0

#define ACTUATOR_DRIVER_I2C "i2c_actuator"
#define CAMX_ACTUATOR_DEV_NAME "cam-actuator-driver"

#define MSM_ACTUATOR_MAX_VREGS (10)
#define ACTUATOR_MAX_POLL_COUNT 10


enum msm_actuator_state_t {
	ACT_APPLY_SETTINGS_NOW,
	ACT_APPLY_SETTINGS_LATER,
};

/**
 * struct intf_params
 * @device_hdl: Device Handle
 * @session_hdl: Session Handle
 * @ops: KMD operations
 * @crm_cb: Callback API pointers
 */
struct intf_params {
	int32_t device_hdl;
	int32_t session_hdl;
	int32_t link_hdl;
	struct cam_req_mgr_kmd_ops ops;
	struct cam_req_mgr_crm_cb *crm_cb;
};

/**
 * struct cam_actuator_ctrl_t
 * @i2c_driver: I2C device info
 * @pdev: Platform device
 * @cci_i2c_master: I2C structure
 * @io_master_info: Information about the communication master
 * @actuator_mutex: Actuator mutex
 * @act_apply_state: Actuator settings aRegulator config
 * @gconf: GPIO config
 * @pinctrl_info: Pinctrl information
 * @v4l2_dev_str: V4L2 device structure
 * @i2c_data: I2C register settings structure
 * @act_info: Sensor query cap structure
 * @of_node: Node ptr
 * @device_name: Device name
 */
struct cam_actuator_ctrl_t {
	struct i2c_driver *i2c_driver;
	enum cci_i2c_master_t cci_i2c_master;
	struct camera_io_master io_master_info;
	struct cam_hw_soc_info soc_info;
	struct mutex actuator_mutex;
	enum msm_actuator_state_t act_apply_state;
	struct msm_camera_gpio_num_info *gpio_num_info;
	uint8_t cam_pinctrl_status;
	struct cam_subdev v4l2_dev_str;
	struct i2c_data_settings i2c_data;
	struct cam_actuator_query_cap act_info;
	struct intf_params bridge_intf;
	char device_name[20];
};

#endif /* _CAM_ACTUATOR_DEV_H_ */
