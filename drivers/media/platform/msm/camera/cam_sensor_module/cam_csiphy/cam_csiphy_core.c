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

#include <linux/module.h>
#include "cam_csiphy_core.h"
#include "cam_csiphy_dev.h"
#include "cam_csiphy_soc.h"
#include <cam_mem_mgr.h>

void cam_csiphy_query_cap(struct csiphy_device *csiphy_dev,
	struct cam_csiphy_query_cap *csiphy_cap)
{
	struct cam_hw_soc_info *soc_info = &csiphy_dev->soc_info;

	csiphy_cap->slot_info = soc_info->index;
	csiphy_cap->version = csiphy_dev->hw_version;
	csiphy_cap->clk_lane = csiphy_dev->clk_lane;
}

void cam_csiphy_reset(struct csiphy_device *csiphy_dev)
{
	int32_t  i;
	void __iomem *base = NULL;
	uint32_t size =
		csiphy_dev->ctrl_reg->csiphy_reg.csiphy_reset_array_size;
	struct cam_hw_soc_info *soc_info = &csiphy_dev->soc_info;

	base = soc_info->reg_map[0].mem_base;

	for (i = 0; i < size; i++) {
		cam_io_w(
			csiphy_dev->ctrl_reg->
			csiphy_reset_reg[i].reg_data,
			base +
			csiphy_dev->ctrl_reg->
			csiphy_reset_reg[i].reg_addr);

		usleep_range(csiphy_dev->ctrl_reg->
			csiphy_reset_reg[i].delay * 100,
			csiphy_dev->ctrl_reg->
			csiphy_reset_reg[i].delay * 100 + 1000);
	}
}

int32_t cam_cmd_buf_parser(struct csiphy_device *csiphy_dev,
	struct cam_config_dev_cmd *cfg_dev)
{
	int32_t                 rc = 0;
	uint64_t                generic_ptr;
	struct cam_packet       *csl_packet = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	uint32_t                *cmd_buf = NULL;
	struct cam_csiphy_info  *cam_cmd_csiphy_info = NULL;
	size_t                  len;

	if (!cfg_dev || !csiphy_dev) {
		pr_err("%s:%d Invalid Args\n", __func__, __LINE__);
		return -EINVAL;
	}

	csiphy_dev->csiphy_info =
		kzalloc(sizeof(struct cam_csiphy_info), GFP_KERNEL);
	if (!csiphy_dev->csiphy_info)
		return -ENOMEM;

	rc = cam_mem_get_cpu_buf((int32_t) cfg_dev->packet_handle,
		(uint64_t *)&generic_ptr, &len);
	if (rc < 0) {
		pr_err("%s:%d :ERROR: Failed to get packet Mem address: %d\n",
			__func__, __LINE__, rc);
		kfree(csiphy_dev->csiphy_info);
		csiphy_dev->csiphy_info = NULL;
		return rc;
	}

	if (cfg_dev->offset > len) {
		pr_err("%s: %d offset is out of bounds: offset: %lld len: %zu\n",
			__func__, __LINE__, cfg_dev->offset, len);
		kfree(csiphy_dev->csiphy_info);
		csiphy_dev->csiphy_info = NULL;
		return -EINVAL;
	}

	csl_packet = (struct cam_packet *)(generic_ptr + cfg_dev->offset);

	cmd_desc = (struct cam_cmd_buf_desc *)
		((uint32_t *)&csl_packet->payload +
		csl_packet->cmd_buf_offset / 4);

	rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle,
		(uint64_t *)&generic_ptr, &len);
	if (rc < 0) {
		pr_err("%s:%d :ERROR: Failed to get cmd buf Mem address : %d\n",
			__func__, __LINE__, rc);
		kfree(csiphy_dev->csiphy_info);
		csiphy_dev->csiphy_info = NULL;
		return rc;
	}

	cmd_buf = (uint32_t *)generic_ptr;
	cmd_buf += cmd_desc->offset / 4;
	cam_cmd_csiphy_info = (struct cam_csiphy_info *)cmd_buf;

	csiphy_dev->csiphy_info->lane_cnt = cam_cmd_csiphy_info->lane_cnt;
	csiphy_dev->csiphy_info->lane_mask = cam_cmd_csiphy_info->lane_mask;
	csiphy_dev->csiphy_info->csiphy_3phase =
		cam_cmd_csiphy_info->csiphy_3phase;
	csiphy_dev->csiphy_info->combo_mode = cam_cmd_csiphy_info->combo_mode;
	csiphy_dev->csiphy_info->settle_time = cam_cmd_csiphy_info->settle_time;
	csiphy_dev->csiphy_info->data_rate = cam_cmd_csiphy_info->data_rate;

	return rc;
}

void cam_csiphy_cphy_irq_config(struct csiphy_device *csiphy_dev)
{
	int32_t i;
	void __iomem *csiphybase =
		csiphy_dev->soc_info.reg_map[0].mem_base;

	for (i = 0; i < csiphy_dev->num_irq_registers; i++)
		cam_io_w(csiphy_dev->ctrl_reg->
			csiphy_irq_reg[i].reg_data,
			csiphybase +
			csiphy_dev->ctrl_reg->
			csiphy_irq_reg[i].reg_addr);
}

void cam_csiphy_cphy_irq_disable(struct csiphy_device *csiphy_dev)
{
	int32_t i;
	void __iomem *csiphybase =
		csiphy_dev->soc_info.reg_map[0].mem_base;

	for (i = 0; i < csiphy_dev->num_irq_registers; i++)
		cam_io_w(0x0,
			csiphybase +
			csiphy_dev->ctrl_reg->
			csiphy_irq_reg[i].reg_addr);
}

irqreturn_t cam_csiphy_irq(int irq_num, void *data)
{
	uint32_t irq;
	uint8_t i;
	struct csiphy_device *csiphy_dev =
		(struct csiphy_device *)data;
	struct cam_hw_soc_info *soc_info = NULL;
	void __iomem *base = NULL;

	if (!csiphy_dev) {
		pr_err("%s:%d Invalid Args\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	soc_info = &csiphy_dev->soc_info;
	base =  csiphy_dev->soc_info.reg_map[0].mem_base;

	for (i = 0; i < csiphy_dev->num_irq_registers; i++) {
		irq = cam_io_r(
			base +
			csiphy_dev->ctrl_reg->csiphy_reg.
			mipi_csiphy_interrupt_status0_addr + 0x4*i);
		cam_io_w(irq,
			base +
			csiphy_dev->ctrl_reg->csiphy_reg.
			mipi_csiphy_interrupt_clear0_addr + 0x4*i);
		pr_err_ratelimited(
			"%s CSIPHY%d_IRQ_STATUS_ADDR%d = 0x%x\n",
			__func__, soc_info->index, i, irq);
		cam_io_w(0x0,
			base +
			csiphy_dev->ctrl_reg->csiphy_reg.
			mipi_csiphy_interrupt_clear0_addr + 0x4*i);
	}
	cam_io_w(0x1, base +
		csiphy_dev->ctrl_reg->
		csiphy_reg.mipi_csiphy_glbl_irq_cmd_addr);
	cam_io_w(0x0, base +
		csiphy_dev->ctrl_reg->
		csiphy_reg.mipi_csiphy_glbl_irq_cmd_addr);

	return IRQ_HANDLED;
}

int32_t cam_csiphy_config_dev(struct csiphy_device *csiphy_dev)
{
	int32_t      rc = 0;
	uint32_t     lane_enable = 0, mask = 1, size = 0;
	uint16_t     lane_mask = 0, i = 0, cfg_size = 0;
	uint8_t      settle_cnt, lane_cnt, lane_pos = 0;
	void __iomem *csiphybase;
	struct csiphy_reg_t (*reg_array)[MAX_SETTINGS_PER_LANE];

	if (csiphy_dev->csiphy_info == NULL) {
		pr_err("%s:%d csiphy_info is NULL, No/Fail CONFIG_DEV ?\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	lane_cnt = csiphy_dev->csiphy_info->lane_cnt;
	lane_mask = csiphy_dev->csiphy_info->lane_mask & 0x1f;
	settle_cnt = (csiphy_dev->csiphy_info->settle_time / 200000000);
	csiphybase = csiphy_dev->soc_info.reg_map[0].mem_base;

	if (!csiphybase) {
		pr_err("%s: csiphybase NULL\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < MAX_DPHY_DATA_LN; i++) {
		if (mask == 0x2) {
			if (lane_mask & mask)
				lane_enable |= 0x80;
			i--;
		} else if (lane_mask & mask) {
			lane_enable |= 0x1 << (i<<1);
		}
		mask <<= 1;
	}

	if (!csiphy_dev->csiphy_info->csiphy_3phase) {
		if (csiphy_dev->csiphy_info->combo_mode == 1)
			reg_array =
				csiphy_dev->ctrl_reg->csiphy_2ph_combo_mode_reg;
		else
			reg_array =
				csiphy_dev->ctrl_reg->csiphy_2ph_reg;
		csiphy_dev->num_irq_registers = 11;
		cfg_size = csiphy_dev->ctrl_reg->csiphy_reg.
			csiphy_2ph_config_array_size;
	} else {
		if (csiphy_dev->csiphy_info->combo_mode == 1)
			reg_array =
				csiphy_dev->ctrl_reg->csiphy_2ph_3ph_mode_reg;
		else
			reg_array =
				csiphy_dev->ctrl_reg->csiphy_3ph_reg;
		csiphy_dev->num_irq_registers = 20;
		cfg_size = csiphy_dev->ctrl_reg->csiphy_reg.
			csiphy_3ph_config_array_size;
	}

	size = csiphy_dev->ctrl_reg->csiphy_reg.csiphy_common_array_size;

	for (i = 0; i < size; i++) {
		switch (csiphy_dev->ctrl_reg->
			csiphy_common_reg[i].csiphy_param_type) {
			case CSIPHY_LANE_ENABLE:
				cam_io_w(lane_enable,
					csiphybase +
					csiphy_dev->ctrl_reg->
					csiphy_common_reg[i].reg_addr);
			break;
			case CSIPHY_DEFAULT_PARAMS:
				cam_io_w(csiphy_dev->ctrl_reg->
					csiphy_common_reg[i].reg_data,
					csiphybase +
					csiphy_dev->ctrl_reg->
					csiphy_common_reg[i].reg_addr);
			break;
			default:
			break;
		}
	}

	while (lane_mask & 0x1f) {
		if (!(lane_mask & 0x1)) {
			lane_pos++;
			lane_mask >>= 1;
			continue;
		}

		for (i = 0; i < cfg_size; i++) {
			switch (reg_array[lane_pos][i].csiphy_param_type) {
			case CSIPHY_LANE_ENABLE:
				cam_io_w(lane_enable,
					csiphybase +
					reg_array[lane_pos][i].reg_addr);
			break;
			case CSIPHY_DEFAULT_PARAMS:
				cam_io_w(reg_array[lane_pos][i].reg_data,
					csiphybase +
					reg_array[lane_pos][i].reg_addr);
			break;
			case CSIPHY_SETTLE_CNT_LOWER_BYTE:
				cam_io_w(settle_cnt & 0xFF,
					csiphybase +
					reg_array[lane_pos][i].reg_addr);
			break;
			case CSIPHY_SETTLE_CNT_HIGHER_BYTE:
				cam_io_w((settle_cnt >> 8) & 0xFF,
					csiphybase +
					reg_array[lane_pos][i].reg_addr);
			break;
			default:
				CDBG("%s: %d Do Nothing\n", __func__, __LINE__);
			break;
			}
			usleep_range(reg_array[lane_pos][i].delay*1000,
				reg_array[lane_pos][i].delay*1000 + 1000);
		}
		lane_mask >>= 1;
		lane_pos++;
	}

	cam_csiphy_cphy_irq_config(csiphy_dev);

	return rc;
}

int32_t cam_csiphy_core_cfg(void *phy_dev,
			void *arg)
{
	struct csiphy_device *csiphy_dev =
		(struct csiphy_device *)phy_dev;
	struct cam_control   *cmd = (struct cam_control *)arg;
	int32_t              rc = 0;

	if (!csiphy_dev || !cmd) {
		pr_err("%s:%d Invalid input args\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	pr_debug("%s:%d Opcode received: %d\n", __func__, __LINE__,
		cmd->op_code);
	mutex_lock(&csiphy_dev->mutex);
	switch (cmd->op_code) {
	case CAM_ACQUIRE_DEV: {
		struct cam_sensor_acquire_dev csiphy_acq_dev;
		struct cam_csiphy_acquire_dev_info csiphy_acq_params;

		struct cam_create_dev_hdl bridge_params;

		rc = copy_from_user(&csiphy_acq_dev,
			(void __user *)cmd->handle,
			sizeof(csiphy_acq_dev));
		if (rc < 0) {
			pr_err("%s:%d :ERROR: Failed copying from User\n",
				__func__, __LINE__);
			goto release_mutex;
		}

		csiphy_acq_params.combo_mode = 0;

		if (csiphy_dev->acquire_count == 2) {
			pr_err("%s:%d CSIPHY device do not allow more than 2 acquires\n",
				__func__, __LINE__);
			rc = -EINVAL;
			goto release_mutex;
		}

		bridge_params.ops = NULL;
		bridge_params.session_hdl = csiphy_acq_dev.session_handle;
		bridge_params.v4l2_sub_dev_flag = 0;
		bridge_params.media_entity_flag = 0;
		bridge_params.priv = csiphy_dev;

		csiphy_acq_dev.device_handle =
			cam_create_device_hdl(&bridge_params);
		csiphy_dev->bridge_intf.
			device_hdl[csiphy_acq_params.combo_mode] =
				csiphy_acq_dev.device_handle;
		csiphy_dev->bridge_intf.
			session_hdl[csiphy_acq_params.combo_mode] =
			csiphy_acq_dev.session_handle;

		if (copy_to_user((void __user *)cmd->handle,
				&csiphy_acq_dev,
				sizeof(struct cam_sensor_acquire_dev))) {
			pr_err("%s:%d :ERROR: Failed copying from User\n",
				__func__, __LINE__);
			rc = -EINVAL;
			goto release_mutex;
		}
		if (csiphy_acq_params.combo_mode == 1)
			csiphy_dev->is_acquired_dev_combo_mode = 1;
		csiphy_dev->acquire_count++;
	}
		break;
	case CAM_QUERY_CAP: {
		struct cam_csiphy_query_cap csiphy_cap;

		cam_csiphy_query_cap(csiphy_dev, &csiphy_cap);
		if (copy_to_user((void __user *)cmd->handle,
			&csiphy_cap, sizeof(struct cam_csiphy_query_cap))) {
			pr_err("%s:%d :ERROR: Failed copying from User\n",
				__func__, __LINE__);
			rc = -EINVAL;
			goto release_mutex;
		}
	}
		break;
	case CAM_STOP_DEV: {
		rc = cam_csiphy_disable_hw(csiphy_dev);
		if (rc < 0) {
			pr_err("%s:%d Failed in csiphy release\n",
				__func__, __LINE__);
			cam_cpas_stop(csiphy_dev->cpas_handle);
			goto release_mutex;
		}
		rc = cam_cpas_stop(csiphy_dev->cpas_handle);
		if (rc < 0) {
			pr_err("%s:%d :Error: de-voting CPAS: %d\n",
				__func__, __LINE__, rc);
			goto release_mutex;
		}
	}
		break;
	case CAM_RELEASE_DEV: {
		struct cam_release_dev_cmd release;

		if (!csiphy_dev->acquire_count) {
			pr_err("%s:%d No valid devices to release\n",
				__func__, __LINE__);
			rc = -EINVAL;
			goto release_mutex;
		}

		if (copy_from_user(&release, (void __user *) cmd->handle,
			sizeof(release))) {
			rc = -EFAULT;
			goto release_mutex;
		}

		rc = cam_destroy_device_hdl(release.dev_handle);
		if (rc < 0)
			pr_err("%s:%d :ERROR: destroying the device hdl\n",
				__func__, __LINE__);
		if (release.dev_handle ==
			csiphy_dev->bridge_intf.device_hdl[0]) {
			csiphy_dev->bridge_intf.device_hdl[0] = -1;
			csiphy_dev->bridge_intf.link_hdl[0] = -1;
			csiphy_dev->bridge_intf.session_hdl[0] = -1;
		} else {
			csiphy_dev->bridge_intf.device_hdl[1] = -1;
			csiphy_dev->bridge_intf.link_hdl[1] = -1;
			csiphy_dev->bridge_intf.
				session_hdl[1] = -1;
		}
		csiphy_dev->acquire_count--;
	}
		break;
	case CAM_CONFIG_DEV: {
		struct cam_config_dev_cmd config;

		if (copy_from_user(&config, (void __user *)cmd->handle,
					sizeof(config))) {
			rc = -EFAULT;
		} else {
			rc = cam_cmd_buf_parser(csiphy_dev, &config);
			if (rc < 0) {
				pr_err("%s:%d Fail in cmd buf parser\n",
					__func__, __LINE__);
				goto release_mutex;
			}
		}
		break;
	}
	case CAM_START_DEV: {
		struct cam_ahb_vote ahb_vote;
		struct cam_axi_vote axi_vote;

		ahb_vote.type = CAM_VOTE_ABSOLUTE;
		ahb_vote.vote.level = CAM_SVS_VOTE;
		axi_vote.compressed_bw = CAM_CPAS_DEFAULT_AXI_BW;
		axi_vote.uncompressed_bw = CAM_CPAS_DEFAULT_AXI_BW;

		rc = cam_cpas_start(csiphy_dev->cpas_handle,
			&ahb_vote, &axi_vote);
		if (rc < 0) {
			pr_err("%s:%d :Error: voting CPAS: %d\n",
				__func__, __LINE__, rc);
			goto release_mutex;
		}

		rc = cam_csiphy_enable_hw(csiphy_dev);
		if (rc != 0) {
			pr_err("%s: %d cam_csiphy_enable_hw failed\n",
				__func__, __LINE__);
			cam_cpas_stop(csiphy_dev->cpas_handle);
			goto release_mutex;
		}
		rc = cam_csiphy_config_dev(csiphy_dev);
		if (rc < 0) {
			pr_err("%s: %d cam_csiphy_config_dev failed\n",
				__func__, __LINE__);
			cam_cpas_stop(csiphy_dev->cpas_handle);
			goto release_mutex;
		}
	}
		break;
	case CAM_SD_SHUTDOWN:
		break;
	default:
		pr_err("%s:%d :Error: Invalid Opcode: %d\n",
			__func__, __LINE__, cmd->op_code);
		rc = -EINVAL;
		goto release_mutex;
	}

release_mutex:
	mutex_unlock(&csiphy_dev->mutex);

	return rc;
}
