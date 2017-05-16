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

#include "cam_cci_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_cci_soc.h"
#include "cam_cci_core.h"

#define CCI_MAX_DELAY 1000000
#define CCI_TIMEOUT msecs_to_jiffies(500)

static struct v4l2_subdev *g_cci_subdev;

struct v4l2_subdev *cam_cci_get_subdev(void)
{
	return g_cci_subdev;
}

static long cam_cci_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int32_t rc = 0;

	switch (cmd) {
	case VIDIOC_MSM_CCI_CFG:
		rc = cam_cci_core_cfg(sd, arg);
		break;
	default:
		pr_err("%s:%d Invalid ioctl cmd: %d\n",
			__func__, __LINE__, cmd);
		rc = -ENOIOCTLCMD;
	}

	return rc;
}

irqreturn_t cam_cci_irq(int irq_num, void *data)
{
	uint32_t irq;
	struct cci_device *cci_dev = data;

	irq = cam_io_r_mb(cci_dev->base + CCI_IRQ_STATUS_0_ADDR);
	cam_io_w_mb(irq, cci_dev->base + CCI_IRQ_CLEAR_0_ADDR);
	cam_io_w_mb(0x1, cci_dev->base + CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);

	if (irq & CCI_IRQ_STATUS_0_RST_DONE_ACK_BMSK) {
		if (cci_dev->cci_master_info[MASTER_0].reset_pending == TRUE) {
			cci_dev->cci_master_info[MASTER_0].reset_pending =
				FALSE;
			complete(&cci_dev->cci_master_info[MASTER_0].
				reset_complete);
		}
		if (cci_dev->cci_master_info[MASTER_1].reset_pending == TRUE) {
			cci_dev->cci_master_info[MASTER_1].reset_pending =
				FALSE;
			complete(&cci_dev->cci_master_info[MASTER_1].
				reset_complete);
		}
	}
	if (irq & CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK) {
		cci_dev->cci_master_info[MASTER_0].status = 0;
		complete(&cci_dev->cci_master_info[MASTER_0].reset_complete);
	}
	if (irq & CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK) {
		struct cam_cci_master_info *cci_master_info;

		cci_master_info = &cci_dev->cci_master_info[MASTER_0];
		atomic_set(&cci_master_info->q_free[QUEUE_0], 0);
		cci_master_info->status = 0;
		if (atomic_read(&cci_master_info->done_pending[QUEUE_0]) == 1) {
			complete(&cci_master_info->report_q[QUEUE_0]);
			atomic_set(&cci_master_info->done_pending[QUEUE_0], 0);
		}
	}
	if (irq & CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK) {
		struct cam_cci_master_info *cci_master_info;

		cci_master_info = &cci_dev->cci_master_info[MASTER_0];
		atomic_set(&cci_master_info->q_free[QUEUE_1], 0);
		cci_master_info->status = 0;
		if (atomic_read(&cci_master_info->done_pending[QUEUE_1]) == 1) {
			complete(&cci_master_info->report_q[QUEUE_1]);
			atomic_set(&cci_master_info->done_pending[QUEUE_1], 0);
		}
	}
	if (irq & CCI_IRQ_STATUS_0_I2C_M1_RD_DONE_BMSK) {
		cci_dev->cci_master_info[MASTER_1].status = 0;
		complete(&cci_dev->cci_master_info[MASTER_1].reset_complete);
	}
	if (irq & CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT_BMSK) {
		struct cam_cci_master_info *cci_master_info;

		cci_master_info = &cci_dev->cci_master_info[MASTER_1];
		atomic_set(&cci_master_info->q_free[QUEUE_0], 0);
		cci_master_info->status = 0;
		if (atomic_read(&cci_master_info->done_pending[QUEUE_0]) == 1) {
			complete(&cci_master_info->report_q[QUEUE_0]);
			atomic_set(&cci_master_info->done_pending[QUEUE_0], 0);
		}
	}
	if (irq & CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT_BMSK) {
		struct cam_cci_master_info *cci_master_info;

		cci_master_info = &cci_dev->cci_master_info[MASTER_1];
		atomic_set(&cci_master_info->q_free[QUEUE_1], 0);
		cci_master_info->status = 0;
		if (atomic_read(&cci_master_info->done_pending[QUEUE_1]) == 1) {
			complete(&cci_master_info->report_q[QUEUE_1]);
			atomic_set(&cci_master_info->done_pending[QUEUE_1], 0);
		}
	}
	if (irq & CCI_IRQ_STATUS_0_I2C_M0_Q0Q1_HALT_ACK_BMSK) {
		cci_dev->cci_master_info[MASTER_0].reset_pending = TRUE;
		cam_io_w_mb(CCI_M0_RESET_RMSK,
			cci_dev->base + CCI_RESET_CMD_ADDR);
	}
	if (irq & CCI_IRQ_STATUS_0_I2C_M1_Q0Q1_HALT_ACK_BMSK) {
		cci_dev->cci_master_info[MASTER_1].reset_pending = TRUE;
		cam_io_w_mb(CCI_M1_RESET_RMSK,
			cci_dev->base + CCI_RESET_CMD_ADDR);
	}
	if (irq & CCI_IRQ_STATUS_0_I2C_M0_ERROR_BMSK) {
		pr_err("%s:%d MASTER_0 error 0x%x\n", __func__, __LINE__, irq);
		cci_dev->cci_master_info[MASTER_0].status = -EINVAL;
		cam_io_w_mb(CCI_M0_HALT_REQ_RMSK,
			cci_dev->base + CCI_HALT_REQ_ADDR);
	}
	if (irq & CCI_IRQ_STATUS_0_I2C_M1_ERROR_BMSK) {
		pr_err("%s:%d MASTER_1 error 0x%x\n", __func__, __LINE__, irq);
		cci_dev->cci_master_info[MASTER_1].status = -EINVAL;
		cam_io_w_mb(CCI_M1_HALT_REQ_RMSK,
			cci_dev->base + CCI_HALT_REQ_ADDR);
	}
	return IRQ_HANDLED;
}

static int cam_cci_irq_routine(struct v4l2_subdev *sd, u32 status,
	bool *handled)
{
	struct cci_device *cci_dev = v4l2_get_subdevdata(sd);
	irqreturn_t ret;

	ret = cam_cci_irq(cci_dev->irq->start, cci_dev);
	*handled = TRUE;
	return 0;
}

static struct v4l2_subdev_core_ops cci_subdev_core_ops = {
	.ioctl = cam_cci_subdev_ioctl,
	.interrupt_service_routine = cam_cci_irq_routine,
};

static const struct v4l2_subdev_ops cci_subdev_ops = {
	.core = &cci_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cci_subdev_intern_ops;

static int cam_cci_platform_probe(struct platform_device *pdev)
{
	struct cam_cpas_register_params cpas_parms;
	struct cci_device *new_cci_dev;
	int rc = 0;

	new_cci_dev = kzalloc(sizeof(struct cci_device),
		GFP_KERNEL);
	if (!new_cci_dev)
		return -ENOMEM;

	new_cci_dev->v4l2_dev_str.pdev = pdev;

	rc = cam_cci_parse_dt_info(pdev, new_cci_dev);
	if (rc < 0) {
		pr_err("%s: %d Resource get Failed: %d\n",
			__func__, __LINE__, rc);
		goto cci_no_resource;
	}

	new_cci_dev->v4l2_dev_str.internal_ops =
		&cci_subdev_intern_ops;
	new_cci_dev->v4l2_dev_str.ops =
		&cci_subdev_ops;
	strlcpy(new_cci_dev->device_name, CAMX_CCI_DEV_NAME,
		sizeof(new_cci_dev->device_name));
	new_cci_dev->v4l2_dev_str.name =
		new_cci_dev->device_name;
	new_cci_dev->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	new_cci_dev->v4l2_dev_str.ent_function =
		CAM_CCI_DEVICE_TYPE;
	new_cci_dev->v4l2_dev_str.token =
		new_cci_dev;

	rc = cam_register_subdev(&(new_cci_dev->v4l2_dev_str));
	if (rc < 0) {
		pr_err("%s:%d :Error: Fail with cam_register_subdev\n",
			__func__, __LINE__);
		goto cci_no_resource;
	}

	platform_set_drvdata(pdev, &(new_cci_dev->v4l2_dev_str.sd));
	v4l2_set_subdevdata(&new_cci_dev->v4l2_dev_str.sd, new_cci_dev);
	g_cci_subdev = &new_cci_dev->v4l2_dev_str.sd;

	cpas_parms.cam_cpas_client_cb = NULL;
	cpas_parms.cell_index = 0;
	cpas_parms.dev = &pdev->dev;
	cpas_parms.userdata = new_cci_dev;
	strlcpy(cpas_parms.identifier, "cci", CAM_HW_IDENTIFIER_LENGTH);
	rc = cam_cpas_register_client(&cpas_parms);
	if (rc) {
		pr_err("%s:%d CPAS registration failed\n", __func__, __LINE__);
		goto cci_no_resource;
	}
	CDBG("CPAS registration successful handle=%d\n",
		cpas_parms.client_handle);
	new_cci_dev->cpas_handle = cpas_parms.client_handle;

	return rc;
cci_no_resource:
	kfree(new_cci_dev);
	return rc;
}

static int cam_cci_device_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *subdev = platform_get_drvdata(pdev);
	struct cci_device *cci_dev =
		v4l2_get_subdevdata(subdev);

	cam_cpas_unregister_client(cci_dev->cpas_handle);
	cam_cci_soc_remove(pdev, cci_dev);
	devm_kfree(&pdev->dev, cci_dev);
	return 0;
}

static const struct of_device_id cam_cci_dt_match[] = {
	{.compatible = "qcom,cci"},
	{}
};

MODULE_DEVICE_TABLE(of, cam_cci_dt_match);

static struct platform_driver cci_driver = {
	.probe = cam_cci_platform_probe,
	.remove = cam_cci_device_remove,
	.driver = {
		.name = CAMX_CCI_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cam_cci_dt_match,
	},
};

static int __init cam_cci_init_module(void)
{
	return platform_driver_register(&cci_driver);
}

static void __exit cam_cci_exit_module(void)
{
	platform_driver_unregister(&cci_driver);
}

module_init(cam_cci_init_module);
module_exit(cam_cci_exit_module);
MODULE_DESCRIPTION("MSM CCI driver");
MODULE_LICENSE("GPL v2");
