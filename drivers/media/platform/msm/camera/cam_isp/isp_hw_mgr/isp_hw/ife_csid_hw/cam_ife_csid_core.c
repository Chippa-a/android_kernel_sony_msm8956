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

#include <linux/iopoll.h>
#include <linux/slab.h>
#include <uapi/media/cam_isp.h>
#include <uapi/media/cam_defs.h>

#include "cam_ife_csid_core.h"
#include "cam_isp_hw.h"
#include "cam_soc_util.h"
#include "cam_io_util.h"

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)


/* Timeout value in msec */
#define IFE_CSID_TIMEOUT                               1000

/* TPG VC/DT values */
#define CAM_IFE_CSID_TPG_VC_VAL                        0xA
#define CAM_IFE_CSID_TPG_DT_VAL                        0x2B

/* Timeout values in usec */
#define CAM_IFE_CSID_TIMEOUT_SLEEP_US                  1000
#define CAM_IFE_CSID_TIMEOUT_ALL_US                    1000000

static int cam_ife_csid_is_ipp_format_supported(
				uint32_t decode_fmt)
{
	int rc = -EINVAL;

	switch (decode_fmt) {
	case CAM_FORMAT_MIPI_RAW_6:
	case CAM_FORMAT_MIPI_RAW_8:
	case CAM_FORMAT_MIPI_RAW_10:
	case CAM_FORMAT_MIPI_RAW_12:
	case CAM_FORMAT_MIPI_RAW_14:
	case CAM_FORMAT_MIPI_RAW_16:
	case CAM_FORMAT_MIPI_RAW_20:
	case CAM_FORMAT_DPCM_10_6_10:
	case CAM_FORMAT_DPCM_10_8_10:
	case CAM_FORMAT_DPCM_12_6_12:
	case CAM_FORMAT_DPCM_12_8_12:
	case CAM_FORMAT_DPCM_14_8_14:
	case CAM_FORMAT_DPCM_14_10_14:
		rc = 0;
		break;
	default:
		break;
	}
	return rc;
}

static int cam_ife_csid_get_format(uint32_t  res_id,
	uint32_t decode_fmt, uint32_t *path_fmt, uint32_t *plain_fmt)
{
	int rc = 0;

	if (res_id >= CAM_IFE_PIX_PATH_RES_RDI_0 &&
		res_id <= CAM_IFE_PIX_PATH_RES_RDI_3) {
		*path_fmt = 0xf;
		return 0;
	}

	switch (decode_fmt) {
	case CAM_FORMAT_MIPI_RAW_6:
		*path_fmt  = 0;
		*plain_fmt = 0;
		break;
	case CAM_FORMAT_MIPI_RAW_8:
		*path_fmt  = 1;
		*plain_fmt = 0;
		break;
	case CAM_FORMAT_MIPI_RAW_10:
		*path_fmt  = 2;
		*plain_fmt = 1;
		break;
	case CAM_FORMAT_MIPI_RAW_12:
		*path_fmt  = 3;
		*plain_fmt = 1;
		break;
	case CAM_FORMAT_MIPI_RAW_14:
		*path_fmt  = 4;
		*plain_fmt = 1;
		break;
	case CAM_FORMAT_MIPI_RAW_16:
		*path_fmt  = 5;
		*plain_fmt = 1;
		break;
	case CAM_FORMAT_MIPI_RAW_20:
		*path_fmt  = 6;
		*plain_fmt = 2;
		break;
	case CAM_FORMAT_DPCM_10_6_10:
		*path_fmt  = 7;
		*plain_fmt = 1;
		break;
	case CAM_FORMAT_DPCM_10_8_10:
		*path_fmt  = 8;
		*plain_fmt = 1;
		break;
	case CAM_FORMAT_DPCM_12_6_12:
		*path_fmt  = 9;
		*plain_fmt = 1;
		break;
	case CAM_FORMAT_DPCM_12_8_12:
		*path_fmt  = 0xA;
		*plain_fmt = 1;
		break;
	case CAM_FORMAT_DPCM_14_8_14:
		*path_fmt  = 0xB;
		*plain_fmt = 1;
		break;
	case CAM_FORMAT_DPCM_14_10_14:
		*path_fmt  = 0xC;
		*plain_fmt = 1;
		break;
	default:
		pr_err("%s:%d:CSID:%d un supported format\n",
		__func__, __LINE__, decode_fmt);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_ife_csid_cid_get(struct cam_ife_csid_hw *csid_hw,
	struct cam_isp_resource_node **res, int32_t vc, uint32_t dt,
	uint32_t res_type)
{
	int  rc = 0;
	struct cam_ife_csid_cid_data    *cid_data;
	uint32_t  i = 0, j = 0;

	for (i = 0; i < CAM_IFE_CSID_CID_RES_MAX; i++) {
		if (csid_hw->cid_res[i].res_state >=
			CAM_ISP_RESOURCE_STATE_RESERVED) {
			cid_data = (struct cam_ife_csid_cid_data *)
				csid_hw->cid_res[i].res_priv;
			if (res_type == CAM_ISP_IFE_IN_RES_TPG) {
				if (cid_data->tpg_set) {
					cid_data->cnt++;
					*res = &csid_hw->cid_res[i];
					break;
				}
			} else {
				if (cid_data->vc == vc && cid_data->dt == dt) {
					cid_data->cnt++;
					*res = &csid_hw->cid_res[i];
					break;
				}
			}
		}
	}

	if (i == CAM_IFE_CSID_CID_RES_MAX) {
		if (res_type == CAM_ISP_IFE_IN_RES_TPG) {
			pr_err("%s:%d:CSID:%d TPG CID not available\n",
				__func__, __LINE__, csid_hw->hw_intf->hw_idx);
			rc = -EINVAL;
		}

		for (j = 0; j < CAM_IFE_CSID_CID_RES_MAX; j++) {
			if (csid_hw->cid_res[j].res_state ==
				CAM_ISP_RESOURCE_STATE_AVAILABLE) {
				cid_data = (struct cam_ife_csid_cid_data *)
					csid_hw->cid_res[j].res_priv;
				cid_data->vc  = vc;
				cid_data->dt  = dt;
				cid_data->cnt = 1;
				csid_hw->cid_res[j].res_state =
					CAM_ISP_RESOURCE_STATE_RESERVED;
				*res = &csid_hw->cid_res[j];
				CDBG("%s:%d:CSID:%d CID %d allocated\n",
					__func__, __LINE__,
					csid_hw->hw_intf->hw_idx,
					csid_hw->cid_res[j].res_id);
				break;
			}
		}

		if (j == CAM_IFE_CSID_CID_RES_MAX) {
			pr_err("%s:%d:CSID:%d Free cid is not available\n",
				__func__, __LINE__, csid_hw->hw_intf->hw_idx);
			rc = -EINVAL;
		}
	}

	return rc;
}


static int cam_ife_csid_global_reset(struct cam_ife_csid_hw *csid_hw)
{
	struct cam_hw_soc_info          *soc_info;
	struct cam_ife_csid_reg_offset  *csid_reg;
	int rc = 0;
	uint32_t i, irq_mask_rx, irq_mask_ipp = 0,
		irq_mask_rdi[CAM_IFE_CSID_RDI_MAX];

	soc_info = &csid_hw->hw_info->soc_info;
	csid_reg = csid_hw->csid_info->csid_reg;

	if (csid_hw->hw_info->hw_state != CAM_HW_STATE_POWER_UP) {
		pr_err("%s:%d:CSID:%d Invalid HW State:%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx,
			csid_hw->hw_info->hw_state);
		return -EINVAL;
	}

	CDBG("%s:%d:CSID:%d Csid reset\n", __func__, __LINE__,
		csid_hw->hw_intf->hw_idx);

	init_completion(&csid_hw->csid_top_complete);

	/* Save interrupt mask registers values*/
	irq_mask_rx = cam_io_r_mb(soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->csid_csi2_rx_irq_mask_addr);

	if (csid_reg->cmn_reg->no_pix)
		irq_mask_ipp = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_irq_mask_addr);

	for (i = 0; i < csid_reg->cmn_reg->no_rdis; i++) {
		irq_mask_rdi[i] = cam_io_r_mb(soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[i]->csid_rdi_irq_mask_addr);
	}

	/* Mask all interrupts */
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->csid_csi2_rx_irq_mask_addr);

	if (csid_reg->cmn_reg->no_pix)
		cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_irq_mask_addr);

	for (i = 0; i < csid_reg->cmn_reg->no_rdis; i++)
		cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[i]->csid_rdi_irq_mask_addr);

	/* clear all interrupts */
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_top_irq_clear_addr);

	cam_io_w_mb(csid_reg->csi2_reg->csi2_irq_mask_all,
		soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->csid_csi2_rx_irq_clear_addr);

	if (csid_reg->cmn_reg->no_pix)
		cam_io_w_mb(csid_reg->cmn_reg->ipp_irq_mask_all,
			soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_irq_clear_addr);

	for (i = 0 ; i < csid_reg->cmn_reg->no_rdis; i++)
		cam_io_w_mb(csid_reg->cmn_reg->rdi_irq_mask_all,
			soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[i]->csid_rdi_irq_clear_addr);

	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_irq_cmd_addr);

	cam_io_w_mb(0x80, soc_info->reg_map[0].mem_base +
		csid_hw->csid_info->csid_reg->csi2_reg->csid_csi2_rx_cfg1_addr);

	/* enable the IPP and RDI format measure */
	if (csid_reg->cmn_reg->no_pix)
		cam_io_w_mb(0x1, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_cfg0_addr);

	for (i = 0; i < csid_reg->cmn_reg->no_rdis; i++)
		cam_io_w_mb(0x2, soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[i]->csid_rdi_cfg0_addr);

	/* perform the top CSID HW reset */
	cam_io_w_mb(csid_reg->cmn_reg->csid_rst_stb,
		soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_rst_strobes_addr);

	CDBG("%s:%d: Waiting for reset complete from irq handler\n",
		__func__, __LINE__);

	rc = wait_for_completion_timeout(&csid_hw->csid_top_complete,
		msecs_to_jiffies(IFE_CSID_TIMEOUT));
	if (rc <= 0) {
		pr_err("%s:%d:CSID:%d reset completion in fail rc = %d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx, rc);
		if (rc == 0)
			rc = -ETIMEDOUT;
	} else {
		rc = 0;
	}

	/*restore all interrupt masks */
	cam_io_w_mb(irq_mask_rx, soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->csid_csi2_rx_irq_mask_addr);

	if (csid_reg->cmn_reg->no_pix)
		cam_io_w_mb(irq_mask_ipp, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_irq_mask_addr);

	for (i = 0; i < csid_reg->cmn_reg->no_rdis; i++)
		cam_io_w_mb(irq_mask_rdi[i], soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[i]->csid_rdi_irq_mask_addr);

	return rc;
}

static int cam_ife_csid_path_reset(struct cam_ife_csid_hw *csid_hw,
	struct cam_csid_reset_cfg_args  *reset)
{
	int rc = 0;
	struct cam_hw_soc_info              *soc_info;
	struct cam_isp_resource_node        *res;
	struct cam_ife_csid_reg_offset      *csid_reg;
	uint32_t  reset_strb_addr, reset_strb_val, val, id;
	struct completion  *complete;

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	res      = reset->node_res;

	if (csid_hw->hw_info->hw_state != CAM_HW_STATE_POWER_UP) {
		pr_err("%s:%d:CSID:%d Invalid hw state :%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx,
			csid_hw->hw_info->hw_state);
		return -EINVAL;
	}

	if (res->res_id >= CAM_IFE_PIX_PATH_RES_MAX) {
		CDBG("%s:%d:CSID:%d Invalid res id%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_id);
		rc = -EINVAL;
		goto end;
	}

	CDBG("%s:%d:CSID:%d resource:%d\n", __func__, __LINE__,
		csid_hw->hw_intf->hw_idx, res->res_id);

	if (res->res_id == CAM_IFE_PIX_PATH_RES_IPP) {
		if (!csid_reg->ipp_reg) {
			pr_err("%s:%d:CSID:%d IPP not supported :%d\n",
				__func__, __LINE__, csid_hw->hw_intf->hw_idx,
				res->res_id);
			return -EINVAL;
		}

		reset_strb_addr = csid_reg->ipp_reg->csid_ipp_rst_strobes_addr;
		complete = &csid_hw->csid_ipp_complete;

		/* Enable path reset done interrupt */
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_irq_mask_addr);
		val |= CSID_PATH_INFO_RST_DONE;
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			 csid_reg->ipp_reg->csid_ipp_irq_mask_addr);

	} else {
		id = res->res_id;
		if (!csid_reg->rdi_reg[id]) {
			pr_err("%s:%d:CSID:%d RDI res not supported :%d\n",
				__func__, __LINE__, csid_hw->hw_intf->hw_idx,
				res->res_id);
			return -EINVAL;
		}

		reset_strb_addr =
			csid_reg->rdi_reg[id]->csid_rdi_rst_strobes_addr;
		complete =
			&csid_hw->csid_rdin_complete[id];

		/* Enable path reset done interrupt */
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_irq_mask_addr);
		val |= CSID_PATH_INFO_RST_DONE;
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_irq_mask_addr);
	}

	init_completion(complete);
	reset_strb_val = csid_reg->cmn_reg->path_rst_stb_all;

	/* Enable the Test gen before reset */
	cam_io_w_mb(1,	csid_hw->hw_info->soc_info.reg_map[0].mem_base +
		csid_reg->tpg_reg->csid_tpg_ctrl_addr);

	/* Reset the corresponding ife csid path */
	cam_io_w_mb(reset_strb_val, soc_info->reg_map[0].mem_base +
				reset_strb_addr);

	rc = wait_for_completion_timeout(complete,
		msecs_to_jiffies(IFE_CSID_TIMEOUT));
	if (rc <= 0) {
		pr_err("%s:%d CSID:%d Res id %d fail rc = %d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_id,  rc);
		if (rc == 0)
			rc = -ETIMEDOUT;
	}

	/* Disable Test Gen after reset*/
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->tpg_reg->csid_tpg_ctrl_addr);

end:
	return rc;

}

static int cam_ife_csid_cid_reserve(struct cam_ife_csid_hw *csid_hw,
	struct cam_csid_hw_reserve_resource_args  *cid_reserv)
{
	int rc = 0;
	struct cam_ife_csid_cid_data       *cid_data;

	CDBG("%s:%d CSID:%d res_sel:%d Lane type:%d lane_num:%d dt:%d vc:%d\n",
		__func__, __LINE__, csid_hw->hw_intf->hw_idx,
		cid_reserv->in_port->res_type,
		cid_reserv->in_port->lane_type,
		cid_reserv->in_port->lane_num,
		cid_reserv->in_port->dt,
		cid_reserv->in_port->vc);

	if (cid_reserv->in_port->res_type >= CAM_ISP_IFE_IN_RES_MAX) {
		pr_err("%s:%d:CSID:%d  Invalid phy sel %d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx,
			cid_reserv->in_port->res_type);
		rc = -EINVAL;
		goto end;
	}

	if (cid_reserv->in_port->lane_type >= CAM_ISP_LANE_TYPE_MAX &&
		cid_reserv->in_port->res_type != CAM_ISP_IFE_IN_RES_TPG) {
		pr_err("%s:%d:CSID:%d  Invalid lane type %d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx,
			cid_reserv->in_port->lane_type);
		rc = -EINVAL;
		goto end;
	}

	if ((cid_reserv->in_port->lane_type ==  CAM_ISP_LANE_TYPE_DPHY &&
		cid_reserv->in_port->lane_num > 4) &&
		cid_reserv->in_port->res_type != CAM_ISP_IFE_IN_RES_TPG) {
		pr_err("%s:%d:CSID:%d Invalid lane num %d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx,
			cid_reserv->in_port->lane_num);
		rc = -EINVAL;
		goto end;
	}
	if ((cid_reserv->in_port->lane_type == CAM_ISP_LANE_TYPE_CPHY &&
		cid_reserv->in_port->lane_num > 3) &&
		cid_reserv->in_port->res_type != CAM_ISP_IFE_IN_RES_TPG) {
		pr_err("%s:%d: CSID:%d Invalid lane type %d & num %d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			cid_reserv->in_port->lane_type,
			cid_reserv->in_port->lane_num);
		rc = -EINVAL;
		goto end;
	}

	/* CSID  CSI2 v2.0 supports 31 vc  */
	if (cid_reserv->in_port->dt > 0x3f ||
		cid_reserv->in_port->vc > 0x1f) {
		pr_err("%s:%d:CSID:%d Invalid vc:%d dt %d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx,
			cid_reserv->in_port->vc, cid_reserv->in_port->dt);
		rc = -EINVAL;
		goto end;
	}

	if (cid_reserv->in_port->res_type == CAM_ISP_IFE_IN_RES_TPG && (
		(cid_reserv->in_port->format < CAM_FORMAT_MIPI_RAW_8 &&
		cid_reserv->in_port->format > CAM_FORMAT_MIPI_RAW_16))) {
		pr_err("%s:%d: CSID:%d Invalid tpg decode fmt %d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			cid_reserv->in_port->format);
		rc = -EINVAL;
		goto end;
	}

	if (csid_hw->csi2_reserve_cnt) {
		/* current configure res type should match requested res type */
		if (csid_hw->res_type != cid_reserv->in_port->res_type) {
			rc = -EINVAL;
			goto end;
		}

		if (cid_reserv->in_port->res_type != CAM_ISP_IFE_IN_RES_TPG) {
			if (csid_hw->csi2_rx_cfg.lane_cfg !=
				cid_reserv->in_port->lane_cfg  ||
				csid_hw->csi2_rx_cfg.lane_type !=
				cid_reserv->in_port->lane_type ||
				csid_hw->csi2_rx_cfg.lane_num !=
				cid_reserv->in_port->lane_num) {
				rc = -EINVAL;
				goto end;
				}
		} else {
			if (csid_hw->tpg_cfg.decode_fmt !=
				cid_reserv->in_port->format     ||
				csid_hw->tpg_cfg.width !=
				cid_reserv->in_port->left_width ||
				csid_hw->tpg_cfg.height !=
				cid_reserv->in_port->height     ||
				csid_hw->tpg_cfg.test_pattern !=
				cid_reserv->in_port->test_pattern) {
				rc = -EINVAL;
				goto end;
			}
		}
	}

	if (!csid_hw->csi2_reserve_cnt) {
		csid_hw->res_type = cid_reserv->in_port->res_type;
		/* Take the first CID resource*/
		csid_hw->cid_res[0].res_state = CAM_ISP_RESOURCE_STATE_RESERVED;
		cid_data = (struct cam_ife_csid_cid_data *)
				csid_hw->cid_res[0].res_priv;

		csid_hw->csi2_rx_cfg.lane_cfg =
			cid_reserv->in_port->lane_cfg;
		csid_hw->csi2_rx_cfg.lane_type =
			cid_reserv->in_port->lane_type;
		csid_hw->csi2_rx_cfg.lane_num =
			cid_reserv->in_port->lane_num;

		if (cid_reserv->in_port->res_type == CAM_ISP_IFE_IN_RES_TPG) {
			csid_hw->csi2_rx_cfg.phy_sel = 0;
			if (cid_reserv->in_port->format >
			    CAM_FORMAT_MIPI_RAW_16) {
				pr_err("%s:%d: Wrong TPG format\n", __func__,
					__LINE__);
				rc = -EINVAL;
				goto end;
			}
			csid_hw->tpg_cfg.decode_fmt =
				cid_reserv->in_port->format;
			csid_hw->tpg_cfg.width =
				cid_reserv->in_port->left_width;
			csid_hw->tpg_cfg.height = cid_reserv->in_port->height;
			csid_hw->tpg_cfg.test_pattern =
				cid_reserv->in_port->test_pattern;
			cid_data->tpg_set = 1;
		} else {
			csid_hw->csi2_rx_cfg.phy_sel =
				(cid_reserv->in_port->res_type & 0xFF) - 1;
		}

		cid_data->vc = cid_reserv->in_port->vc;
		cid_data->dt = cid_reserv->in_port->dt;
		cid_data->cnt = 1;
		cid_reserv->node_res = &csid_hw->cid_res[0];
		csid_hw->csi2_reserve_cnt++;

		CDBG("%s:%d:CSID:%d CID :%d resource acquired successfully\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			cid_reserv->node_res->res_id);
	} else {
		rc = cam_ife_csid_cid_get(csid_hw, &cid_reserv->node_res,
			cid_reserv->in_port->vc, cid_reserv->in_port->dt,
			cid_reserv->in_port->res_type);
		/* if success then increment the reserve count */
		if (!rc) {
			if (csid_hw->csi2_reserve_cnt == UINT_MAX) {
				pr_err("%s:%d:CSID%d reserve cnt reached max\n",
					__func__, __LINE__,
					csid_hw->hw_intf->hw_idx);
				rc = -EINVAL;
			} else {
				csid_hw->csi2_reserve_cnt++;
				CDBG("%s:%d:CSID:%d CID:%d acquired\n",
					__func__, __LINE__,
					csid_hw->hw_intf->hw_idx,
					cid_reserv->node_res->res_id);
			}
		}
	}

end:
	return rc;
}


static int cam_ife_csid_path_reserve(struct cam_ife_csid_hw *csid_hw,
	struct cam_csid_hw_reserve_resource_args  *reserve)
{
	int rc = 0;
	struct cam_ife_csid_path_cfg    *path_data;
	struct cam_isp_resource_node    *res;

	/* CSID  CSI2 v2.0 supports 31 vc */
	if (reserve->in_port->dt > 0x3f || reserve->in_port->vc > 0x1f ||
		(reserve->sync_mode >= CAM_ISP_HW_SYNC_MAX)) {
		pr_err("%s:%d:CSID:%d Invalid vc:%d dt %d mode:%d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			reserve->in_port->vc, reserve->in_port->dt,
			reserve->sync_mode);
		rc = -EINVAL;
		goto end;
	}

	switch (reserve->res_id) {
	case CAM_IFE_PIX_PATH_RES_IPP:
		if (csid_hw->ipp_res.res_state !=
			CAM_ISP_RESOURCE_STATE_AVAILABLE) {
			CDBG("%s:%d:CSID:%d IPP resource not available %d\n",
				__func__, __LINE__, csid_hw->hw_intf->hw_idx,
				csid_hw->ipp_res.res_state);
			rc = -EINVAL;
			goto end;
		}

		if (cam_ife_csid_is_ipp_format_supported(
				reserve->in_port->format)) {
			pr_err("%s:%d:CSID:%d res id:%d un support format %d\n",
				__func__, __LINE__,
				csid_hw->hw_intf->hw_idx, reserve->res_id,
				reserve->in_port->format);
			rc = -EINVAL;
			goto end;
		}

		/* assign the IPP resource */
		res = &csid_hw->ipp_res;
		CDBG("%s:%d:CSID:%d IPP resource:%d acquired successfully\n",
			__func__, __LINE__,
			csid_hw->hw_intf->hw_idx, res->res_id);

			break;
	case CAM_IFE_PIX_PATH_RES_RDI_0:
	case CAM_IFE_PIX_PATH_RES_RDI_1:
	case CAM_IFE_PIX_PATH_RES_RDI_2:
	case CAM_IFE_PIX_PATH_RES_RDI_3:
		if (csid_hw->rdi_res[reserve->res_id].res_state !=
			CAM_ISP_RESOURCE_STATE_AVAILABLE) {
			CDBG("%s:%d:CSID:%d RDI:%d resource not available %d\n",
				__func__, __LINE__, csid_hw->hw_intf->hw_idx,
				reserve->res_id,
				csid_hw->rdi_res[reserve->res_id].res_state);
			rc = -EINVAL;
			goto end;
		} else {
			res = &csid_hw->rdi_res[reserve->res_id];
			CDBG("%s:%d:CSID:%d RDI resource:%d acquire success\n",
				__func__, __LINE__, csid_hw->hw_intf->hw_idx,
				res->res_id);
		}

		break;
	default:
		pr_err("%s:%d:CSID:%d Invalid res id:%d\n",
			__func__, __LINE__,
			csid_hw->hw_intf->hw_idx, reserve->res_id);
		rc = -EINVAL;
		goto end;
	}

	res->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;
	path_data = (struct cam_ife_csid_path_cfg   *)res->res_priv;

	path_data->cid = reserve->cid;
	path_data->decode_fmt = reserve->in_port->format;
	path_data->master_idx = reserve->master_idx;
	path_data->sync_mode = reserve->sync_mode;
	path_data->height  = reserve->in_port->height;
	path_data->start_line = reserve->in_port->line_start;
	if (reserve->in_port->res_type == CAM_ISP_IFE_IN_RES_TPG) {
		path_data->dt = CAM_IFE_CSID_TPG_DT_VAL;
		path_data->vc = CAM_IFE_CSID_TPG_VC_VAL;
	} else {
		path_data->dt = reserve->in_port->dt;
		path_data->vc = reserve->in_port->vc;
	}

	if (reserve->sync_mode == CAM_ISP_HW_SYNC_MASTER) {
		path_data->crop_enable = 1;
		path_data->start_pixel = reserve->in_port->left_start;
		path_data->width  = reserve->in_port->left_width;
	} else if (reserve->sync_mode == CAM_ISP_HW_SYNC_SLAVE) {
		path_data->crop_enable = 1;
		path_data->start_pixel = reserve->in_port->right_start;
		path_data->width  = reserve->in_port->right_width;
	} else
		path_data->crop_enable = 0;

	reserve->node_res = res;

end:
	return rc;
}

static int cam_ife_csid_enable_hw(struct cam_ife_csid_hw  *csid_hw)
{
	int rc = 0;
	struct cam_ife_csid_reg_offset      *csid_reg;
	struct cam_hw_soc_info              *soc_info;
	uint32_t i, status, val;

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	/* overflow check before increment */
	if (csid_hw->hw_info->open_count == UINT_MAX) {
		pr_err("%s:%d:CSID:%d Open count reached max\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx);
		return -EINVAL;
	}

	/* Increment ref Count */
	csid_hw->hw_info->open_count++;
	if (csid_hw->hw_info->open_count > 1) {
		CDBG("%s:%d: CSID hw has already been enabled\n",
			__func__, __LINE__);
		return rc;
	}

	CDBG("%s:%d:CSID:%d init CSID HW\n", __func__, __LINE__,
		csid_hw->hw_intf->hw_idx);

	rc = cam_ife_csid_enable_soc_resources(soc_info);
	if (rc) {
		pr_err("%s:%d:CSID:%d Enable SOC failed\n", __func__, __LINE__,
			csid_hw->hw_intf->hw_idx);
		goto err;
	}


	CDBG("%s:%d:CSID:%d enable top irq interrupt\n", __func__, __LINE__,
		csid_hw->hw_intf->hw_idx);

	csid_hw->hw_info->hw_state = CAM_HW_STATE_POWER_UP;
	/* Enable the top IRQ interrupt */
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_top_irq_mask_addr);

	rc = cam_ife_csid_global_reset(csid_hw);
	if (rc) {
		pr_err("%s:%d CSID:%d csid_reset fail rc = %d\n",
			 __func__, __LINE__, csid_hw->hw_intf->hw_idx, rc);
		rc = -ETIMEDOUT;
		goto disable_soc;
	}

	/*
	 * Reset the SW registers
	 * SW register reset also reset the mask irq, so poll the irq status
	 * to check the reset complete.
	 */
	CDBG("%s:%d:CSID:%d Reset Software registers\n", __func__, __LINE__,
			csid_hw->hw_intf->hw_idx);

	cam_io_w_mb(csid_reg->cmn_reg->csid_rst_stb_sw_all,
		soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_rst_strobes_addr);

	rc = readl_poll_timeout(soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_top_irq_status_addr,
			status, (status & 0x1) == 0x1,
		CAM_IFE_CSID_TIMEOUT_SLEEP_US, CAM_IFE_CSID_TIMEOUT_ALL_US);
	if (rc < 0) {
		pr_err("%s:%d: software register reset timeout.....\n",
			__func__, __LINE__);
		rc = -ETIMEDOUT;
		goto disable_soc;
	}

	/* clear all interrupts */
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_top_irq_clear_addr);

	cam_io_w_mb(csid_reg->csi2_reg->csi2_irq_mask_all,
		soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->csid_csi2_rx_irq_clear_addr);

	if (csid_reg->cmn_reg->no_pix)
		cam_io_w_mb(csid_reg->cmn_reg->ipp_irq_mask_all,
			soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_irq_clear_addr);

	for (i = 0; i < csid_reg->cmn_reg->no_rdis; i++)
		cam_io_w_mb(csid_reg->cmn_reg->rdi_irq_mask_all,
			soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[i]->csid_rdi_irq_clear_addr);

	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_irq_cmd_addr);

	/* Enable the top IRQ interrupt */
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
			csid_reg->cmn_reg->csid_top_irq_mask_addr);

	val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->cmn_reg->csid_hw_version_addr);
	CDBG("%s:%d:CSID:%d CSID HW version: 0x%x\n", __func__, __LINE__,
		csid_hw->hw_intf->hw_idx, val);

	return 0;

disable_soc:
	cam_ife_csid_disable_soc_resources(soc_info);
	csid_hw->hw_info->hw_state = CAM_HW_STATE_POWER_DOWN;
err:
	csid_hw->hw_info->open_count--;
	return rc;
}

static int cam_ife_csid_disable_hw(struct cam_ife_csid_hw *csid_hw)
{
	int rc = 0;
	struct cam_hw_soc_info             *soc_info;
	struct cam_ife_csid_reg_offset     *csid_reg;


	/*  Decrement ref Count */
	if (csid_hw->hw_info->open_count)
		csid_hw->hw_info->open_count--;
	if (csid_hw->hw_info->open_count)
		return rc;

	soc_info = &csid_hw->hw_info->soc_info;
	csid_reg = csid_hw->csid_info->csid_reg;

	CDBG("%s:%d:CSID:%d De-init CSID HW\n", __func__, __LINE__,
		csid_hw->hw_intf->hw_idx);

	/*disable the top IRQ interrupt */
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_top_irq_mask_addr);

	rc = cam_ife_csid_disable_soc_resources(soc_info);
	if (rc)
		pr_err("%s:%d:CSID:%d Disable CSID SOC failed\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx);

	csid_hw->hw_info->hw_state = CAM_HW_STATE_POWER_DOWN;
	return rc;
}


static int cam_ife_csid_tpg_start(struct cam_ife_csid_hw   *csid_hw,
	struct cam_isp_resource_node       *res)
{
	uint32_t  val = 0;
	struct cam_hw_soc_info    *soc_info;

	csid_hw->tpg_start_cnt++;
	if (csid_hw->tpg_start_cnt == 1) {
		/*Enable the TPG */
		CDBG("%s:%d CSID:%d start CSID TPG\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx);

		soc_info = &csid_hw->hw_info->soc_info;
		{
			uint32_t val;
			uint32_t i;
			uint32_t base = 0x600;

			CDBG("%s:%d: ================== TPG ===============\n",
				__func__, __LINE__);
			for (i = 0; i < 16; i++) {
				val = cam_io_r_mb(
					soc_info->reg_map[0].mem_base +
					base + i * 4);
				CDBG("%s:%d reg 0x%x = 0x%x\n",
					__func__, __LINE__,
					(base + i*4), val);
			}

			CDBG("%s:%d: ================== IPP ===============\n",
				__func__, __LINE__);
			base = 0x200;
			for (i = 0; i < 10; i++) {
				val = cam_io_r_mb(
					soc_info->reg_map[0].mem_base +
					base + i * 4);
				CDBG("%s:%d reg 0x%x = 0x%x\n",
					__func__, __LINE__,
					(base + i*4), val);
			}

			CDBG("%s:%d: ================== RX ===============\n",
				__func__, __LINE__);
			base = 0x100;
			for (i = 0; i < 5; i++) {
				val = cam_io_r_mb(
					soc_info->reg_map[0].mem_base +
					base + i * 4);
				CDBG("%s:%d reg 0x%x = 0x%x\n",
					__func__, __LINE__,
					(base + i*4), val);
			}
		}

		CDBG("%s:%d: =============== TPG control ===============\n",
			__func__, __LINE__);
		val = (4 << 20);
		val |= (0x80 << 8);
		val |= (((csid_hw->csi2_rx_cfg.lane_num - 1) & 0x3) << 4);
		val |= 7;
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_hw->csid_info->csid_reg->tpg_reg->
			csid_tpg_ctrl_addr);

		val = cam_io_r_mb(soc_info->reg_map[0].mem_base + 0x600);
		CDBG("%s:%d reg 0x%x = 0x%x\n", __func__, __LINE__,
			0x600, val);
	}

	return 0;
}

static int cam_ife_csid_tpg_stop(struct cam_ife_csid_hw   *csid_hw,
	struct cam_isp_resource_node       *res)
{
	struct cam_hw_soc_info    *soc_info;

	if (csid_hw->tpg_start_cnt)
		csid_hw->tpg_start_cnt--;

	if (csid_hw->tpg_start_cnt)
		return 0;

	soc_info = &csid_hw->hw_info->soc_info;

	/* disable the TPG */
	if (!csid_hw->tpg_start_cnt) {
		CDBG("%s:%d CSID:%d stop CSID TPG\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx);

		/*stop the TPG */
		cam_io_w_mb(0,  soc_info->reg_map[0].mem_base +
		csid_hw->csid_info->csid_reg->tpg_reg->csid_tpg_ctrl_addr);
	}

	return 0;
}


static int cam_ife_csid_config_tpg(struct cam_ife_csid_hw   *csid_hw,
	struct cam_isp_resource_node       *res)
{
	struct cam_ife_csid_reg_offset *csid_reg;
	struct cam_hw_soc_info         *soc_info;
	uint32_t val = 0;

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	CDBG("%s:%d CSID:%d TPG config\n", __func__,
		__LINE__, csid_hw->hw_intf->hw_idx);

	/* configure one DT, infinite frames */
	val = (0 << 16) | (1 << 10) | CAM_IFE_CSID_TPG_VC_VAL;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->tpg_reg->csid_tpg_vc_cfg0_addr);

	/* vertical blanking count = 0x740, horzontal blanking count = 0x740*/
	val = (0x740 << 12) | 0x740;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->tpg_reg->csid_tpg_vc_cfg1_addr);

	cam_io_w_mb(0x12345678, soc_info->reg_map[0].mem_base +
		csid_hw->csid_info->csid_reg->tpg_reg->csid_tpg_lfsr_seed_addr);

	val = csid_hw->tpg_cfg.width << 16 |
		csid_hw->tpg_cfg.height;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->tpg_reg->csid_tpg_dt_n_cfg_0_addr);

	cam_io_w_mb(CAM_IFE_CSID_TPG_DT_VAL, soc_info->reg_map[0].mem_base +
		csid_reg->tpg_reg->csid_tpg_dt_n_cfg_1_addr);

	/*
	 * decode_fmt is the same as the input resource format.
	 * it is one larger than the register spec format.
	 */
	val = ((csid_hw->tpg_cfg.decode_fmt - 1) << 16) | 0x8;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->tpg_reg->csid_tpg_dt_n_cfg_2_addr);

	/* select rotate period as  5 frame */
	val =  5 << 8;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->tpg_reg->csid_tpg_color_bars_cfg_addr);
	/* config pix pattern */
	cam_io_w_mb(csid_hw->tpg_cfg.test_pattern,
		soc_info->reg_map[0].mem_base +
		csid_reg->tpg_reg->csid_tpg_common_gen_cfg_addr);

	return 0;
}

static int cam_ife_csid_enable_csi2(
	struct cam_ife_csid_hw          *csid_hw,
	struct cam_isp_resource_node    *res)
{
	int rc = 0;
	struct cam_ife_csid_reg_offset       *csid_reg;
	struct cam_hw_soc_info               *soc_info;
	struct cam_ife_csid_cid_data         *cid_data;
	uint32_t val = 0;

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	CDBG("%s:%d CSID:%d count:%d config csi2 rx\n", __func__,
		__LINE__, csid_hw->hw_intf->hw_idx, csid_hw->csi2_cfg_cnt);

	/* overflow check before increment */
	if (csid_hw->csi2_cfg_cnt == UINT_MAX) {
		pr_err("%s:%d:CSID:%d Open count reached max\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx);
		return -EINVAL;
	}

	cid_data = (struct cam_ife_csid_cid_data *)res->res_priv;

	res->res_state  = CAM_ISP_RESOURCE_STATE_STREAMING;
	csid_hw->csi2_cfg_cnt++;
	if (csid_hw->csi2_cfg_cnt > 1)
		return rc;

	/* rx cfg0 */
	val = (csid_hw->csi2_rx_cfg.lane_num - 1)  |
		(csid_hw->csi2_rx_cfg.lane_cfg << 4) |
		(csid_hw->csi2_rx_cfg.lane_type << 24);
	val |= csid_hw->csi2_rx_cfg.phy_sel & 0x3;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->csid_csi2_rx_cfg0_addr);

	/* rx cfg1*/
	val = (1 << csid_reg->csi2_reg->csi2_misr_enable_shift_val);
	/* if VC value is more than 3 than set full width of VC */
	if (cid_data->vc > 3)
		val |= (1 << csid_reg->csi2_reg->csi2_vc_mode_shift_val);

	/* enable packet ecc correction */
	val |= 1;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->csid_csi2_rx_cfg1_addr);

	if (csid_hw->res_type == CAM_ISP_IFE_IN_RES_TPG) {
		/* Config the TPG */
		rc = cam_ife_csid_config_tpg(csid_hw, res);
		if (rc) {
			res->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;
			return rc;
		}
	}

	/*Enable the CSI2 rx inerrupts */
	val = CSID_CSI2_RX_INFO_RST_DONE |
		CSID_CSI2_RX_ERROR_TG_FIFO_OVERFLOW |
		CSID_CSI2_RX_ERROR_LANE0_FIFO_OVERFLOW |
		CSID_CSI2_RX_ERROR_LANE1_FIFO_OVERFLOW |
		CSID_CSI2_RX_ERROR_LANE2_FIFO_OVERFLOW |
		CSID_CSI2_RX_ERROR_LANE3_FIFO_OVERFLOW |
		CSID_CSI2_RX_ERROR_CPHY_EOT_RECEPTION |
		CSID_CSI2_RX_ERROR_CPHY_SOT_RECEPTION |
		CSID_CSI2_RX_ERROR_CPHY_PH_CRC;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->csid_csi2_rx_irq_mask_addr);

	return 0;
}

static int cam_ife_csid_disable_csi2(
	struct cam_ife_csid_hw          *csid_hw,
	struct cam_isp_resource_node    *res)
{
	struct cam_ife_csid_reg_offset      *csid_reg;
	struct cam_hw_soc_info              *soc_info;

	if (res->res_id >= CAM_IFE_CSID_CID_MAX) {
		pr_err("%s:%d CSID:%d Invalid res id :%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_id);
		return -EINVAL;
	}

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	CDBG("%s:%d CSID:%d cnt : %d Disable csi2 rx\n", __func__,
		__LINE__, csid_hw->hw_intf->hw_idx, csid_hw->csi2_cfg_cnt);

	if (csid_hw->csi2_cfg_cnt)
		csid_hw->csi2_cfg_cnt--;

	if (csid_hw->csi2_cfg_cnt)
		return 0;

	/*Disable the CSI2 rx inerrupts */
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->csid_csi2_rx_irq_mask_addr);

	res->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;

	return 0;
}

static int cam_ife_csid_init_config_ipp_path(
	struct cam_ife_csid_hw          *csid_hw,
	struct cam_isp_resource_node    *res)
{
	int rc = 0;
	struct cam_ife_csid_path_cfg           *path_data;
	struct cam_ife_csid_reg_offset         *csid_reg;
	struct cam_hw_soc_info                 *soc_info;
	uint32_t path_format = 0, plain_format = 0, val = 0;

	path_data = (struct cam_ife_csid_path_cfg  *) res->res_priv;
	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	if (!csid_reg->ipp_reg) {
		pr_err("%s:%d CSID:%d IPP:%d is not supported on HW\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_id);
		return -EINVAL;
	}

	CDBG("%s:%d: Enabled IPP Path.......\n", __func__, __LINE__);
	rc = cam_ife_csid_get_format(res->res_id,
		path_data->decode_fmt, &path_format, &plain_format);
	if (rc)
		return rc;

	/**
	 * configure the IPP and enable the time stamp capture.
	 * enable the HW measrurement blocks
	 */
	val = (path_data->vc << csid_reg->cmn_reg->vc_shift_val) |
		(path_data->dt << csid_reg->cmn_reg->dt_shift_val) |
		(path_data->cid << csid_reg->cmn_reg->dt_id_shift_val) |
		(path_format << csid_reg->cmn_reg->fmt_shift_val) |
		(path_data->crop_enable & 1 <<
		csid_reg->cmn_reg->crop_h_en_shift_val) |
		(path_data->crop_enable & 1 <<
		csid_reg->cmn_reg->crop_v_en_shift_val) |
		(1 << 1) | 1;
	val |= (1 << csid_reg->ipp_reg->pix_store_en_shift_val);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_cfg0_addr);

	if (path_data->crop_enable) {
		val = ((path_data->width +
			path_data->start_pixel) & 0xFFFF <<
			csid_reg->cmn_reg->crop_shift) |
			(path_data->start_pixel & 0xFFFF);

		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_hcrop_addr);

		val = ((path_data->height +
			path_data->start_line) & 0xFFFF <<
			csid_reg->cmn_reg->crop_shift) |
			(path_data->start_line & 0xFFFF);

		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_vcrop_addr);
	}

	/* set frame drop pattern to 0 and period to 1 */
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_frm_drop_period_addr);
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_frm_drop_pattern_addr);
	/* set irq sub sample pattern to 0 and period to 1 */
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_irq_subsample_period_addr);
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_irq_subsample_pattern_addr);
	/* set pixel drop pattern to 0 and period to 1 */
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_pix_drop_pattern_addr);
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_pix_drop_period_addr);
	/* set line drop pattern to 0 and period to 1 */
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_line_drop_pattern_addr);
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_line_drop_period_addr);

	/*Set master or slave IPP */
	if (path_data->sync_mode == CAM_ISP_HW_SYNC_MASTER)
		/*Set halt mode as master */
		val = CSID_HALT_MODE_MASTER << 2;
	else if (path_data->sync_mode == CAM_ISP_HW_SYNC_SLAVE)
		/*Set halt mode as slave and set master idx */
		val = path_data->master_idx  << 4 | CSID_HALT_MODE_SLAVE << 2;
	else
		/* Default is internal halt mode */
		val = 0;

	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_ctrl_addr);

	/* Enable the IPP path */
	val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_cfg0_addr);
	val |= (1 << csid_reg->cmn_reg->path_en_shift_val);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_cfg0_addr);

	res->res_state = CAM_ISP_RESOURCE_STATE_INIT_HW;

	return rc;
}

static int cam_ife_csid_deinit_ipp_path(
	struct cam_ife_csid_hw          *csid_hw,
	struct cam_isp_resource_node    *res)
{
	int rc = 0;
	struct cam_ife_csid_reg_offset      *csid_reg;
	struct cam_hw_soc_info              *soc_info;
	uint32_t val = 0;

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	if (res->res_state != CAM_ISP_RESOURCE_STATE_INIT_HW) {
		pr_err("%s:%d:CSID:%d Res type %d res_id:%d in wrong state %d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_type, res->res_id, res->res_state);
		rc = -EINVAL;
	}

	if (!csid_reg->ipp_reg) {
		pr_err("%s:%d:CSID:%d IPP %d is not supported on HW\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_id);
		rc = -EINVAL;
	}

	/* Disable the IPP path */
	val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_cfg0_addr);
	val &= ~(1 << csid_reg->cmn_reg->path_en_shift_val);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_cfg0_addr);

	res->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;
	return rc;
}

static int cam_ife_csid_enable_ipp_path(
	struct cam_ife_csid_hw          *csid_hw,
	struct cam_isp_resource_node    *res)
{
	struct cam_ife_csid_reg_offset    *csid_reg;
	struct cam_hw_soc_info            *soc_info;
	struct cam_ife_csid_path_cfg      *path_data;
	uint32_t val = 0;

	path_data = (struct cam_ife_csid_path_cfg   *) res->res_priv;
	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	if (res->res_state != CAM_ISP_RESOURCE_STATE_INIT_HW) {
		pr_err("%s:%d:CSID:%d res type:%d res_id:%d Invalid state%d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_type, res->res_id, res->res_state);
		return -EINVAL;
	}

	if (!csid_reg->ipp_reg) {
		pr_err("%s:%d:CSID:%d IPP %d not supported on HW\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_id);
		return -EINVAL;
	}

	CDBG("%s:%d: enable IPP path.......\n", __func__, __LINE__);

	/*Resume at frame boundary */
	if (path_data->sync_mode == CAM_ISP_HW_SYNC_MASTER) {
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_ctrl_addr);
		val |= CAM_CSID_RESUME_AT_FRAME_BOUNDARY;
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_ctrl_addr);
	} else if (path_data->sync_mode == CAM_ISP_HW_SYNC_NONE) {
		cam_io_w_mb(CAM_CSID_RESUME_AT_FRAME_BOUNDARY,
			soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_ctrl_addr);
	}
	/* for slave mode, not need to resume for slave device */

	/* Enable the required ipp interrupts */
	val = CSID_PATH_INFO_RST_DONE | CSID_PATH_ERROR_FIFO_OVERFLOW|
		CSID_PATH_INFO_INPUT_SOF|CSID_PATH_INFO_INPUT_EOF;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->ipp_reg->csid_ipp_irq_mask_addr);

	res->res_state = CAM_ISP_RESOURCE_STATE_STREAMING;

	return 0;
}

static int cam_ife_csid_disable_ipp_path(
	struct cam_ife_csid_hw          *csid_hw,
	struct cam_isp_resource_node    *res,
	enum cam_ife_csid_halt_cmd       stop_cmd)
{
	int rc = 0;
	struct cam_ife_csid_reg_offset       *csid_reg;
	struct cam_hw_soc_info               *soc_info;
	struct cam_ife_csid_path_cfg         *path_data;
	uint32_t val = 0;

	path_data = (struct cam_ife_csid_path_cfg   *) res->res_priv;
	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	if (res->res_id >= CAM_IFE_PIX_PATH_RES_MAX) {
		CDBG("%s:%d:CSID:%d Invalid res id%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_id);
		return -EINVAL;
	}

	if (res->res_state == CAM_ISP_RESOURCE_STATE_INIT_HW ||
		res->res_state == CAM_ISP_RESOURCE_STATE_RESERVED) {
		CDBG("%s:%d:CSID:%d Res:%d already in stopped state:%d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_id, res->res_state);
		return rc;
	}

	if (res->res_state != CAM_ISP_RESOURCE_STATE_STREAMING) {
		CDBG("%s:%d:CSID:%d Res:%d Invalid state%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_id,
			res->res_state);
		return -EINVAL;
	}

	if (!csid_reg->ipp_reg) {
		pr_err("%s:%d:CSID:%d IPP%d is not supported on HW\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_id);
		return -EINVAL;
	}

	if (stop_cmd != CAM_CSID_HALT_AT_FRAME_BOUNDARY &&
		stop_cmd != CAM_CSID_HALT_IMMEDIATELY) {
		pr_err("%s:%d:CSID:%d un supported stop command:%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, stop_cmd);
		return -EINVAL;
	}

	CDBG("%s:%d CSID:%d res_id:%d\n", __func__, __LINE__,
		csid_hw->hw_intf->hw_idx, res->res_id);

	if (path_data->sync_mode == CAM_ISP_HW_SYNC_MASTER) {
		/* configure Halt */
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_ctrl_addr);
		val &= ~0x3;
		val |= stop_cmd;
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_ctrl_addr);
	} else if (path_data->sync_mode == CAM_ISP_HW_SYNC_NONE)
		cam_io_w_mb(stop_cmd, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_ctrl_addr);

	/* For slave mode, halt command should take it from master */

	/* Enable the EOF interrupt for resume at boundary case */
	if (stop_cmd != CAM_CSID_HALT_IMMEDIATELY) {
		init_completion(&csid_hw->csid_ipp_complete);
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
				csid_reg->ipp_reg->csid_ipp_irq_mask_addr);
		val |= CSID_PATH_INFO_INPUT_EOF;
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_irq_mask_addr);
	} else {
		val &= ~(CSID_PATH_INFO_RST_DONE |
				CSID_PATH_ERROR_FIFO_OVERFLOW);
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_irq_mask_addr);
	}

	return rc;
}


static int cam_ife_csid_init_config_rdi_path(
	struct cam_ife_csid_hw          *csid_hw,
	struct cam_isp_resource_node    *res)
{
	int rc = 0;
	struct cam_ife_csid_path_cfg           *path_data;
	struct cam_ife_csid_reg_offset         *csid_reg;
	struct cam_hw_soc_info                 *soc_info;
	uint32_t path_format = 0, plain_fmt = 0, val = 0, id;

	path_data = (struct cam_ife_csid_path_cfg   *) res->res_priv;
	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	id = res->res_id;
	if (!csid_reg->rdi_reg[id]) {
		pr_err("%s:%d CSID:%d RDI:%d is not supported on HW\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx, id);
		return -EINVAL;
	}

	rc = cam_ife_csid_get_format(res->res_id,
		path_data->decode_fmt, &path_format, &plain_fmt);
	if (rc)
		return rc;

	/**
	 * RDI path config and enable the time stamp capture
	 * Enable the measurement blocks
	 */
	val = (path_data->vc << csid_reg->cmn_reg->vc_shift_val) |
		(path_data->dt << csid_reg->cmn_reg->dt_shift_val) |
		(path_data->cid << csid_reg->cmn_reg->dt_id_shift_val) |
		(path_format << csid_reg->cmn_reg->fmt_shift_val) |
		(plain_fmt << csid_reg->cmn_reg->plain_fmt_shit_val) |
		(path_data->crop_enable & 1 <<
			csid_reg->cmn_reg->crop_h_en_shift_val) |
		(path_data->crop_enable & 1 <<
		csid_reg->cmn_reg->crop_v_en_shift_val) |
		(1 << 2) | 3;

	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_cfg0_addr);

	if (path_data->crop_enable) {
		val = ((path_data->width +
			path_data->start_pixel) & 0xFFFF <<
			csid_reg->cmn_reg->crop_shift) |
			(path_data->start_pixel & 0xFFFF);

		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_rpp_hcrop_addr);

		val = ((path_data->height +
			path_data->start_line) & 0xFFFF <<
			csid_reg->cmn_reg->crop_shift) |
			(path_data->start_line & 0xFFFF);

		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_rpp_vcrop_addr);
	}
	/* set frame drop pattern to 0 and period to 1 */
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_frm_drop_period_addr);
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_frm_drop_pattern_addr);
	/* set IRQ sum sabmple */
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_irq_subsample_period_addr);
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_irq_subsample_pattern_addr);

	/* set pixel drop pattern to 0 and period to 1 */
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_rpp_pix_drop_pattern_addr);
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_rpp_pix_drop_period_addr);
	/* set line drop pattern to 0 and period to 1 */
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_rpp_line_drop_pattern_addr);
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_rpp_line_drop_period_addr);

	/* Configure the halt mode */
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_ctrl_addr);

	/* Enable the RPP path */
	val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_cfg0_addr);
	val |= (1 << csid_reg->cmn_reg->path_en_shift_val);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_cfg0_addr);

	res->res_state = CAM_ISP_RESOURCE_STATE_INIT_HW;

	return rc;
}

static int cam_ife_csid_deinit_rdi_path(
	struct cam_ife_csid_hw          *csid_hw,
	struct cam_isp_resource_node    *res)
{
	int rc = 0;
	struct cam_ife_csid_reg_offset      *csid_reg;
	struct cam_hw_soc_info              *soc_info;
	uint32_t val = 0, id;

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	id = res->res_id;

	if (res->res_id > CAM_IFE_PIX_PATH_RES_RDI_3 ||
		res->res_state != CAM_ISP_RESOURCE_STATE_INIT_HW ||
		!csid_reg->rdi_reg[id]) {
		pr_err("%s:%d:CSID:%d Invalid res id%d state:%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_id,
			res->res_state);
		return -EINVAL;
	}

	/* Disable the RDI path */
	val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_cfg0_addr);
	val &= ~(1 << csid_reg->cmn_reg->path_en_shift_val);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_cfg0_addr);

	res->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;
	return rc;
}

static int cam_ife_csid_enable_rdi_path(
	struct cam_ife_csid_hw          *csid_hw,
	struct cam_isp_resource_node    *res)
{
	struct cam_ife_csid_reg_offset      *csid_reg;
	struct cam_hw_soc_info              *soc_info;
	uint32_t id, val;

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	id = res->res_id;

	if (res->res_state != CAM_ISP_RESOURCE_STATE_INIT_HW ||
		res->res_id > CAM_IFE_PIX_PATH_RES_RDI_3 ||
		!csid_reg->rdi_reg[id]) {
		pr_err("%s:%d:CSID:%d invalid res type:%d res_id:%d state%d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_type, res->res_id, res->res_state);
		return -EINVAL;
	}

	/*resume at frame boundary */
	cam_io_w_mb(CAM_CSID_RESUME_AT_FRAME_BOUNDARY,
			soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_ctrl_addr);

	/* Enable the required RDI interrupts */
	val = (CSID_PATH_INFO_RST_DONE | CSID_PATH_ERROR_FIFO_OVERFLOW|
		CSID_PATH_INFO_INPUT_SOF | CSID_PATH_INFO_INPUT_EOF);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[id]->csid_rdi_irq_mask_addr);

	res->res_state = CAM_ISP_RESOURCE_STATE_STREAMING;

	return 0;
}


static int cam_ife_csid_disable_rdi_path(
	struct cam_ife_csid_hw          *csid_hw,
	struct cam_isp_resource_node    *res,
	enum cam_ife_csid_halt_cmd                stop_cmd)
{
	int rc = 0;
	struct cam_ife_csid_reg_offset       *csid_reg;
	struct cam_hw_soc_info               *soc_info;
	uint32_t  val = 0, id;

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	id = res->res_id;

	if (res->res_id >= CAM_IFE_PIX_PATH_RES_MAX ||
		!csid_reg->rdi_reg[res->res_id]) {
		CDBG("%s:%d:CSID:%d Invalid res id%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_id);
		return -EINVAL;
	}

	if (res->res_state == CAM_ISP_RESOURCE_STATE_INIT_HW ||
		res->res_state == CAM_ISP_RESOURCE_STATE_RESERVED) {
		CDBG("%s:%d:CSID:%d Res:%d already in stopped state:%d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_id, res->res_state);
		return rc;
	}

	if (res->res_state != CAM_ISP_RESOURCE_STATE_STREAMING) {
		CDBG("%s:%d:CSID:%d Res:%d Invalid res_state%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_id,
			res->res_state);
		return -EINVAL;
	}

	if (stop_cmd != CAM_CSID_HALT_AT_FRAME_BOUNDARY &&
		stop_cmd != CAM_CSID_HALT_IMMEDIATELY) {
		pr_err("%s:%d:CSID:%d un supported stop command:%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, stop_cmd);
		return -EINVAL;
	}


	CDBG("%s:%d CSID:%d res_id:%d\n", __func__, __LINE__,
		csid_hw->hw_intf->hw_idx, res->res_id);

	init_completion(&csid_hw->csid_rdin_complete[id]);

	if (stop_cmd != CAM_CSID_HALT_IMMEDIATELY) {
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_irq_mask_addr);
		val |= CSID_PATH_INFO_INPUT_EOF;
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_irq_mask_addr);
	} else {
		val &= ~(CSID_PATH_INFO_RST_DONE |
				CSID_PATH_ERROR_FIFO_OVERFLOW);
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_irq_mask_addr);
	}

	/*Halt the RDI path */
	cam_io_w_mb(stop_cmd, soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_ctrl_addr);

	return rc;
}

static int cam_ife_csid_get_time_stamp(
		struct cam_ife_csid_hw   *csid_hw, void *cmd_args)
{
	struct cam_csid_get_time_stamp_args  *time_stamp;
	struct cam_isp_resource_node         *res;
	struct cam_ife_csid_reg_offset       *csid_reg;
	struct cam_hw_soc_info               *soc_info;
	uint32_t  time_32, id;

	time_stamp = (struct cam_csid_get_time_stamp_args  *)cmd_args;
	res = time_stamp->node_res;
	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	if (res->res_type != CAM_ISP_RESOURCE_PIX_PATH ||
		res->res_id >= CAM_IFE_PIX_PATH_RES_MAX) {
		CDBG("%s:%d:CSID:%d Invalid res_type:%d res id%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_type,
			res->res_id);
		return -EINVAL;
	}

	if (csid_hw->hw_info->hw_state != CAM_HW_STATE_POWER_UP) {
		pr_err("%s:%d:CSID:%d Invalid dev state :%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx,
			csid_hw->hw_info->hw_state);
		return -EINVAL;
	}

	if (res->res_id == CAM_IFE_PIX_PATH_RES_IPP) {
		time_32 = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_timestamp_curr1_sof_addr);
		time_stamp->time_stamp_val = time_32;
		time_stamp->time_stamp_val = time_stamp->time_stamp_val << 32;
		time_32 = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_timestamp_curr0_sof_addr);
		time_stamp->time_stamp_val |= time_32;
	} else {
		id = res->res_id;
		time_32 = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->
			csid_rdi_timestamp_curr1_sof_addr);
		time_stamp->time_stamp_val = time_32;
		time_stamp->time_stamp_val = time_stamp->time_stamp_val << 32;

		time_32 = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->
			csid_rdi_timestamp_curr0_sof_addr);
		time_stamp->time_stamp_val |= time_32;
	}

	return 0;
}
static int cam_ife_csid_res_wait_for_halt(
	struct cam_ife_csid_hw          *csid_hw,
	struct cam_isp_resource_node    *res)
{
	int rc = 0;
	struct cam_ife_csid_reg_offset      *csid_reg;
	struct cam_hw_soc_info              *soc_info;

	struct completion  *complete;
	uint32_t val = 0, id;

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	if (res->res_id >= CAM_IFE_PIX_PATH_RES_MAX) {
		CDBG("%s:%d:CSID:%d Invalid res id%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_id);
		return -EINVAL;
	}

	if (res->res_state == CAM_ISP_RESOURCE_STATE_INIT_HW ||
		res->res_state == CAM_ISP_RESOURCE_STATE_RESERVED) {
		CDBG("%s:%d:CSID:%d Res:%d already in stopped state:%d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_id, res->res_state);
		return rc;
	}

	if (res->res_state != CAM_ISP_RESOURCE_STATE_STREAMING) {
		CDBG("%s:%d:CSID:%d Res:%d Invalid state%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_id,
			res->res_state);
		return -EINVAL;
	}

	if (res->res_id == CAM_IFE_PIX_PATH_RES_IPP)
		complete = &csid_hw->csid_ipp_complete;
	else
		complete =  &csid_hw->csid_rdin_complete[res->res_id];

	rc = wait_for_completion_timeout(complete,
		msecs_to_jiffies(IFE_CSID_TIMEOUT));
	if (rc <= 0) {
		pr_err("%s:%d:CSID%d stop at frame boundary failid:%drc:%d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_id, rc);
		if (rc == 0)
			/* continue even have timeout */
			rc = -ETIMEDOUT;
	}

	/* Disable the interrupt */
	if (res->res_id == CAM_IFE_PIX_PATH_RES_IPP) {
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
				csid_reg->ipp_reg->csid_ipp_irq_mask_addr);
		val &= ~(CSID_PATH_INFO_INPUT_EOF | CSID_PATH_INFO_RST_DONE |
				CSID_PATH_ERROR_FIFO_OVERFLOW);
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_irq_mask_addr);
	} else {
		id = res->res_id;
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_irq_mask_addr);
		val &= ~(CSID_PATH_INFO_INPUT_EOF | CSID_PATH_INFO_RST_DONE |
			CSID_PATH_ERROR_FIFO_OVERFLOW);
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[id]->csid_rdi_irq_mask_addr);
	}
	/* set state to init HW */
	res->res_state = CAM_ISP_RESOURCE_STATE_INIT_HW;
	return rc;
}

static int cam_ife_csid_get_hw_caps(void *hw_priv,
	void *get_hw_cap_args, uint32_t arg_size)
{
	int rc = 0;
	struct cam_ife_csid_hw_caps     *hw_caps;
	struct cam_ife_csid_hw          *csid_hw;
	struct cam_hw_info              *csid_hw_info;
	struct cam_ife_csid_reg_offset  *csid_reg;

	if (!hw_priv || !get_hw_cap_args) {
		pr_err("%s:%d:CSID: Invalid args\n", __func__, __LINE__);
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct cam_ife_csid_hw   *)csid_hw_info->core_info;
	csid_reg = csid_hw->csid_info->csid_reg;
	hw_caps = (struct cam_ife_csid_hw_caps *) get_hw_cap_args;

	hw_caps->no_rdis = csid_reg->cmn_reg->no_rdis;
	hw_caps->no_pix = csid_reg->cmn_reg->no_pix;
	hw_caps->major_version = csid_reg->cmn_reg->major_version;
	hw_caps->minor_version = csid_reg->cmn_reg->minor_version;
	hw_caps->version_incr = csid_reg->cmn_reg->version_incr;

	CDBG("%s:%d:CSID:%d No rdis:%d, no pix:%d, major:%d minor:%d ver :%d\n",
		__func__, __LINE__, csid_hw->hw_intf->hw_idx, hw_caps->no_rdis,
		hw_caps->no_pix, hw_caps->major_version, hw_caps->minor_version,
		hw_caps->version_incr);

	return rc;
}

static int cam_ife_csid_reset(void *hw_priv,
	void *reset_args, uint32_t arg_size)
{
	struct cam_ife_csid_hw          *csid_hw;
	struct cam_hw_info              *csid_hw_info;
	struct cam_csid_reset_cfg_args  *reset;
	int rc = 0;

	if (!hw_priv || !reset_args || (arg_size !=
		sizeof(struct cam_csid_reset_cfg_args))) {
		pr_err("%s:%d:CSID:Invalid args\n", __func__, __LINE__);
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct cam_ife_csid_hw   *)csid_hw_info->core_info;
	reset   = (struct cam_csid_reset_cfg_args  *)reset_args;

	switch (reset->reset_type) {
	case CAM_IFE_CSID_RESET_GLOBAL:
		rc = cam_ife_csid_global_reset(csid_hw);
		break;
	case CAM_IFE_CSID_RESET_PATH:
		rc = cam_ife_csid_path_reset(csid_hw, reset);
		break;
	default:
		pr_err("%s:%d:CSID:Invalid reset type :%d\n", __func__,
			__LINE__, reset->reset_type);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int cam_ife_csid_reserve(void *hw_priv,
	void *reserve_args, uint32_t arg_size)
{
	int rc = 0;
	struct cam_ife_csid_hw                    *csid_hw;
	struct cam_hw_info                        *csid_hw_info;
	struct cam_csid_hw_reserve_resource_args  *reserv;

	if (!hw_priv || !reserve_args || (arg_size !=
		sizeof(struct cam_csid_hw_reserve_resource_args))) {
		pr_err("%s:%d:CSID: Invalid args\n", __func__, __LINE__);
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct cam_ife_csid_hw   *)csid_hw_info->core_info;
	reserv = (struct cam_csid_hw_reserve_resource_args  *)reserve_args;

	mutex_lock(&csid_hw->hw_info->hw_mutex);
	switch (reserv->res_type) {
	case CAM_ISP_RESOURCE_CID:
		rc = cam_ife_csid_cid_reserve(csid_hw, reserv);
		break;
	case CAM_ISP_RESOURCE_PIX_PATH:
		rc = cam_ife_csid_path_reserve(csid_hw, reserv);
		break;
	default:
		pr_err("%s:%d:CSID:%d Invalid res type :%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, reserv->res_type);
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&csid_hw->hw_info->hw_mutex);
	return rc;
}

static int cam_ife_csid_release(void *hw_priv,
	void *release_args, uint32_t arg_size)
{
	int rc = 0;
	struct cam_ife_csid_hw          *csid_hw;
	struct cam_hw_info              *csid_hw_info;
	struct cam_isp_resource_node    *res;
	struct cam_ife_csid_cid_data    *cid_data;

	if (!hw_priv || !release_args ||
		(arg_size != sizeof(struct cam_isp_resource_node))) {
		pr_err("%s:%d:CSID: Invalid args\n", __func__, __LINE__);
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct cam_ife_csid_hw   *)csid_hw_info->core_info;
	res = (struct cam_isp_resource_node *)release_args;

	mutex_lock(&csid_hw->hw_info->hw_mutex);
	if ((res->res_type == CAM_ISP_RESOURCE_CID &&
		res->res_id >= CAM_IFE_CSID_CID_MAX) ||
		(res->res_type == CAM_ISP_RESOURCE_PIX_PATH &&
		res->res_id >= CAM_IFE_PIX_PATH_RES_MAX)) {
		pr_err("%s:%d:CSID:%d Invalid res type:%d res id%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_type,
			res->res_id);
		rc = -EINVAL;
		goto end;
	}

	if (res->res_state == CAM_ISP_RESOURCE_STATE_AVAILABLE) {
		CDBG("%s:%d:CSID:%d res type:%d Res %d  in released state\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_type, res->res_id);
		goto end;
	}

	if (res->res_type == CAM_ISP_RESOURCE_PIX_PATH &&
		res->res_state != CAM_ISP_RESOURCE_STATE_RESERVED) {
		CDBG("%s:%d:CSID:%d res type:%d Res id:%d invalid state:%d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_type, res->res_id, res->res_state);
		rc = -EINVAL;
		goto end;
	}

	CDBG("%s:%d:CSID:%d res type :%d Resource id:%d\n", __func__, __LINE__,
			csid_hw->hw_intf->hw_idx, res->res_type, res->res_id);

	switch (res->res_type) {
	case CAM_ISP_RESOURCE_CID:
		cid_data = (struct cam_ife_csid_cid_data    *) res->res_priv;
		if (cid_data->cnt)
			cid_data->cnt--;

		if (!cid_data->cnt)
			res->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;

		if (csid_hw->csi2_reserve_cnt)
			csid_hw->csi2_reserve_cnt--;

		if (!csid_hw->csi2_reserve_cnt)
			memset(&csid_hw->csi2_rx_cfg, 0,
				sizeof(struct cam_ife_csid_csi2_rx_cfg));

		CDBG("%s:%d:CSID:%d res id :%d cnt:%d reserv cnt:%d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_id, cid_data->cnt, csid_hw->csi2_reserve_cnt);

		break;
	case CAM_ISP_RESOURCE_PIX_PATH:
		res->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;
		break;
	default:
		pr_err("%s:%d:CSID:%d Invalid res type:%d res id%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_type,
			res->res_id);
		rc = -EINVAL;
		break;
	}

end:
	mutex_unlock(&csid_hw->hw_info->hw_mutex);
	return rc;
}

static int cam_ife_csid_init_hw(void *hw_priv,
	void *init_args, uint32_t arg_size)
{
	int rc = 0;
	struct cam_ife_csid_hw                 *csid_hw;
	struct cam_hw_info                     *csid_hw_info;
	struct cam_isp_resource_node           *res;
	struct cam_ife_csid_reg_offset         *csid_reg;

	if (!hw_priv || !init_args ||
		(arg_size != sizeof(struct cam_isp_resource_node))) {
		pr_err("%s:%d:CSID: Invalid args\n", __func__, __LINE__);
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct cam_ife_csid_hw   *)csid_hw_info->core_info;
	res      = (struct cam_isp_resource_node *)init_args;
	csid_reg = csid_hw->csid_info->csid_reg;

	mutex_lock(&csid_hw->hw_info->hw_mutex);
	if ((res->res_type == CAM_ISP_RESOURCE_CID &&
		res->res_id >= CAM_IFE_CSID_CID_MAX) ||
		(res->res_type == CAM_ISP_RESOURCE_PIX_PATH &&
		res->res_id >= CAM_IFE_PIX_PATH_RES_MAX)) {
		pr_err("%s:%d:CSID:%d Invalid res tpe:%d res id%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_type,
			res->res_id);
		rc = -EINVAL;
		goto end;
	}


	if ((res->res_type == CAM_ISP_RESOURCE_PIX_PATH) &&
		(res->res_state != CAM_ISP_RESOURCE_STATE_RESERVED)) {
		pr_err("%s:%d:CSID:%d res type:%d res_id:%dInvalid state %d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_type, res->res_id, res->res_state);
		rc = -EINVAL;
		goto end;
	}

	CDBG("%s:%d CSID:%d res type :%d res_id:%d\n", __func__, __LINE__,
		csid_hw->hw_intf->hw_idx, res->res_type, res->res_id);


	/* Initialize the csid hardware */
	rc = cam_ife_csid_enable_hw(csid_hw);
	if (rc)
		goto end;

	switch (res->res_type) {
	case CAM_ISP_RESOURCE_CID:
		rc = cam_ife_csid_enable_csi2(csid_hw, res);
		break;
	case CAM_ISP_RESOURCE_PIX_PATH:
		if (res->res_id == CAM_IFE_PIX_PATH_RES_IPP)
			rc = cam_ife_csid_init_config_ipp_path(csid_hw, res);
		else
			rc = cam_ife_csid_init_config_rdi_path(csid_hw, res);

		break;
	default:
		pr_err("%s:%d:CSID:%d Invalid res type state %d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_type);
		break;
	}

	if (rc)
		cam_ife_csid_disable_hw(csid_hw);
end:
	mutex_unlock(&csid_hw->hw_info->hw_mutex);
	return rc;
}

static int cam_ife_csid_deinit_hw(void *hw_priv,
	void *deinit_args, uint32_t arg_size)
{
	int rc = 0;
	struct cam_ife_csid_hw                 *csid_hw;
	struct cam_hw_info                     *csid_hw_info;
	struct cam_isp_resource_node           *res;

	if (!hw_priv || !deinit_args ||
		(arg_size != sizeof(struct cam_isp_resource_node))) {
		pr_err("%s:%d:CSID:Invalid arguments\n", __func__, __LINE__);
		return -EINVAL;
	}

	res = (struct cam_isp_resource_node *)deinit_args;
	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct cam_ife_csid_hw   *)csid_hw_info->core_info;

	mutex_lock(&csid_hw->hw_info->hw_mutex);
	if (res->res_state == CAM_ISP_RESOURCE_STATE_RESERVED) {
		CDBG("%s:%d:CSID:%d Res:%d already in De-init state\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_id);
		goto end;
	}

	switch (res->res_type) {
	case CAM_ISP_RESOURCE_CID:
		rc = cam_ife_csid_disable_csi2(csid_hw, res);
		break;
	case CAM_ISP_RESOURCE_PIX_PATH:
		if (res->res_id == CAM_IFE_PIX_PATH_RES_IPP)
			rc = cam_ife_csid_deinit_ipp_path(csid_hw, res);
		else
			rc = cam_ife_csid_deinit_rdi_path(csid_hw, res);

		break;
	default:
		pr_err("%s:%d:CSID:%d Invalid Res type %d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_type);
		goto end;
	}

	/* Disable CSID HW */
	cam_ife_csid_disable_hw(csid_hw);

end:
	mutex_unlock(&csid_hw->hw_info->hw_mutex);
	return rc;
}

static int cam_ife_csid_start(void *hw_priv, void *start_args,
			uint32_t arg_size)
{
	int rc = 0;
	struct cam_ife_csid_hw                 *csid_hw;
	struct cam_hw_info                     *csid_hw_info;
	struct cam_isp_resource_node           *res;
	struct cam_ife_csid_reg_offset         *csid_reg;

	if (!hw_priv || !start_args ||
		(arg_size != sizeof(struct cam_isp_resource_node))) {
		pr_err("%s:%d:CSID: Invalid args\n", __func__, __LINE__);
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct cam_ife_csid_hw   *)csid_hw_info->core_info;
	res = (struct cam_isp_resource_node *)start_args;
	csid_reg = csid_hw->csid_info->csid_reg;

	mutex_lock(&csid_hw->hw_info->hw_mutex);
	if ((res->res_type == CAM_ISP_RESOURCE_CID &&
		res->res_id >= CAM_IFE_CSID_CID_MAX) ||
		(res->res_type == CAM_ISP_RESOURCE_PIX_PATH &&
		res->res_id >= CAM_IFE_PIX_PATH_RES_MAX)) {
		CDBG("%s:%d:CSID:%d Invalid res tpe:%d res id:%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, res->res_type,
			res->res_id);
		rc = -EINVAL;
		goto end;
	}

	CDBG("%s:%d CSID:%d res_type :%d res_id:%d\n", __func__, __LINE__,
		csid_hw->hw_intf->hw_idx, res->res_type, res->res_id);

	switch (res->res_type) {
	case CAM_ISP_RESOURCE_CID:
		if (csid_hw->res_type ==  CAM_ISP_IFE_IN_RES_TPG)
			rc = cam_ife_csid_tpg_start(csid_hw, res);
		break;
	case CAM_ISP_RESOURCE_PIX_PATH:
		if (res->res_id == CAM_IFE_PIX_PATH_RES_IPP)
			rc = cam_ife_csid_enable_ipp_path(csid_hw, res);
		else
			rc = cam_ife_csid_enable_rdi_path(csid_hw, res);
		break;
	default:
		pr_err("%s:%d:CSID:%d Invalid res type%d\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx,
			res->res_type);
		break;
	}
end:
	mutex_unlock(&csid_hw->hw_info->hw_mutex);
	return rc;
}

static int cam_ife_csid_stop(void *hw_priv,
	void *stop_args, uint32_t arg_size)
{
	int rc = 0;
	struct cam_ife_csid_hw               *csid_hw;
	struct cam_hw_info                   *csid_hw_info;
	struct cam_isp_resource_node         *res;
	struct cam_csid_hw_stop_args         *csid_stop;
	uint32_t  i;

	if (!hw_priv || !stop_args ||
		(arg_size != sizeof(struct cam_csid_hw_stop_args))) {
		pr_err("%s:%d:CSID: Invalid args\n", __func__, __LINE__);
		return -EINVAL;
	}
	csid_stop = (struct cam_csid_hw_stop_args  *) stop_args;
	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct cam_ife_csid_hw   *)csid_hw_info->core_info;

	mutex_lock(&csid_hw->hw_info->hw_mutex);
	/* Stop the resource first */
	for (i = 0; i < csid_stop->num_res; i++) {
		res = csid_stop->node_res[i];
		switch (res->res_type) {
		case CAM_ISP_RESOURCE_CID:
			if (csid_hw->res_type == CAM_ISP_IFE_IN_RES_TPG)
				rc = cam_ife_csid_tpg_stop(csid_hw, res);
			break;
		case CAM_ISP_RESOURCE_PIX_PATH:
			if (res->res_id == CAM_IFE_PIX_PATH_RES_IPP)
				rc = cam_ife_csid_disable_ipp_path(csid_hw,
						res, csid_stop->stop_cmd);
			else
				rc = cam_ife_csid_disable_rdi_path(csid_hw,
						res, csid_stop->stop_cmd);

			break;
		default:
			pr_err("%s:%d:CSID:%d Invalid res type%d\n", __func__,
				__LINE__, csid_hw->hw_intf->hw_idx,
				res->res_type);
			break;
		}
	}

	/*wait for the path to halt */
	for (i = 0; i < csid_stop->num_res; i++) {
		res = csid_stop->node_res[i];
		if (res->res_type == CAM_ISP_RESOURCE_PIX_PATH &&
			csid_stop->stop_cmd == CAM_CSID_HALT_AT_FRAME_BOUNDARY)
			rc = cam_ife_csid_res_wait_for_halt(csid_hw, res);
	}

	mutex_unlock(&csid_hw->hw_info->hw_mutex);
	return rc;

}

static int cam_ife_csid_read(void *hw_priv,
	void *read_args, uint32_t arg_size)
{
	pr_err("%s:%d:CSID: un supported\n", __func__, __LINE__);

	return -EINVAL;
}

static int cam_ife_csid_write(void *hw_priv,
	void *write_args, uint32_t arg_size)
{
	pr_err("%s:%d:CSID: un supported\n", __func__, __LINE__);
	return -EINVAL;
}

static int cam_ife_csid_process_cmd(void *hw_priv,
	uint32_t cmd_type, void *cmd_args, uint32_t arg_size)
{
	int rc = 0;
	struct cam_ife_csid_hw               *csid_hw;
	struct cam_hw_info                   *csid_hw_info;

	if (!hw_priv || !cmd_args) {
		pr_err("%s:%d:CSID: Invalid arguments\n", __func__, __LINE__);
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct cam_ife_csid_hw   *)csid_hw_info->core_info;

	mutex_lock(&csid_hw->hw_info->hw_mutex);
	switch (cmd_type) {
	case CAM_IFE_CSID_CMD_GET_TIME_STAMP:
		rc = cam_ife_csid_get_time_stamp(csid_hw, cmd_args);
		break;
	default:
		pr_err("%s:%d:CSID:%d un supported cmd:%d\n", __func__,
			__LINE__, csid_hw->hw_intf->hw_idx, cmd_type);
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&csid_hw->hw_info->hw_mutex);

	return rc;

}

irqreturn_t cam_ife_csid_irq(int irq_num, void *data)
{
	struct cam_ife_csid_hw          *csid_hw;
	struct cam_hw_soc_info          *soc_info;
	struct cam_ife_csid_reg_offset  *csid_reg;
	uint32_t i, irq_status_top, irq_status_rx, irq_status_ipp = 0,
		irq_status_rdi[4];

	csid_hw = (struct cam_ife_csid_hw *)data;

	CDBG("%s:%d:CSID %d IRQ Handling\n", __func__, __LINE__,
		csid_hw->hw_intf->hw_idx);

	if (!data) {
		pr_err("%s:%d:CSID: Invalid arguments\n", __func__, __LINE__);
		return IRQ_HANDLED;
	}

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	/* read */
	irq_status_top = cam_io_r_mb(soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_top_irq_status_addr);

	irq_status_rx = cam_io_r_mb(soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->csid_csi2_rx_irq_status_addr);

	if (csid_reg->cmn_reg->no_pix)
		irq_status_ipp = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_irq_status_addr);


	for (i = 0; i < csid_reg->cmn_reg->no_rdis; i++)
		irq_status_rdi[i] = cam_io_r_mb(soc_info->reg_map[0].mem_base +
		csid_reg->rdi_reg[i]->csid_rdi_irq_status_addr);

	/* clear */
	cam_io_w_mb(irq_status_top, soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_top_irq_clear_addr);
	cam_io_w_mb(irq_status_rx, soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->csid_csi2_rx_irq_clear_addr);
	if (csid_reg->cmn_reg->no_pix)
		cam_io_w_mb(irq_status_ipp, soc_info->reg_map[0].mem_base +
			csid_reg->ipp_reg->csid_ipp_irq_clear_addr);

	for (i = 0; i < csid_reg->cmn_reg->no_rdis; i++) {
		cam_io_w_mb(irq_status_rdi[i], soc_info->reg_map[0].mem_base +
			csid_reg->rdi_reg[i]->csid_rdi_irq_clear_addr);
	}
	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
		csid_reg->cmn_reg->csid_irq_cmd_addr);

	CDBG("%s:%d: irq_status_rx = 0x%x\n", __func__, __LINE__,
		irq_status_rx);
	CDBG("%s:%d: irq_status_ipp = 0x%x\n", __func__, __LINE__,
		irq_status_ipp);

	if (irq_status_top) {
		CDBG("%s:%d: CSID global reset complete......Exit\n",
			__func__, __LINE__);
		complete(&csid_hw->csid_top_complete);
		return IRQ_HANDLED;
	}


	if (irq_status_rx & BIT(csid_reg->csi2_reg->csi2_rst_done_shift_val)) {
		CDBG("%s:%d: csi rx reset complete\n", __func__, __LINE__);
		complete(&csid_hw->csid_csi2_complete);
	}

	if (irq_status_rx & CSID_CSI2_RX_ERROR_LANE0_FIFO_OVERFLOW) {
		pr_err_ratelimited("%s:%d:CSID:%d lane 0 over flow\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx);
	}
	if (irq_status_rx & CSID_CSI2_RX_ERROR_LANE1_FIFO_OVERFLOW) {
		pr_err_ratelimited("%s:%d:CSID:%d lane 1 over flow\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx);
	}
	if (irq_status_rx & CSID_CSI2_RX_ERROR_LANE2_FIFO_OVERFLOW) {
		pr_err_ratelimited("%s:%d:CSID:%d lane 2 over flow\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx);
	}
	if (irq_status_rx & CSID_CSI2_RX_ERROR_LANE3_FIFO_OVERFLOW) {
		pr_err_ratelimited("%s:%d:CSID:%d lane 3 over flow\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx);
	}
	if (irq_status_rx & CSID_CSI2_RX_ERROR_TG_FIFO_OVERFLOW) {
		pr_err_ratelimited("%s:%d:CSID:%d TG OVER  FLOW\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx);
	}
	if (irq_status_rx & CSID_CSI2_RX_ERROR_CPHY_EOT_RECEPTION) {
		pr_err_ratelimited("%s:%d:CSID:%d CPHY_EOT_RECEPTION\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx);
	}
	if (irq_status_rx & CSID_CSI2_RX_ERROR_CPHY_SOT_RECEPTION) {
		pr_err_ratelimited("%s:%d:CSID:%d CPHY_SOT_RECEPTION\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx);
	}
	if (irq_status_rx & CSID_CSI2_RX_ERROR_CPHY_PH_CRC) {
		pr_err_ratelimited("%s:%d:CSID:%d CPHY_PH_CRC\n",
			__func__, __LINE__, csid_hw->hw_intf->hw_idx);
	}

	/*read the IPP errors */
	if (csid_reg->cmn_reg->no_pix) {
		/* IPP reset done bit */
		if (irq_status_ipp &
			BIT(csid_reg->cmn_reg->path_rst_done_shift_val)) {
			CDBG("%s%d: CSID IPP reset complete\n",
				__func__, __LINE__);
			complete(&csid_hw->csid_ipp_complete);
		}
		if (irq_status_ipp & CSID_PATH_INFO_INPUT_SOF)
			CDBG("%s: CSID IPP SOF received\n", __func__);
		if (irq_status_ipp & CSID_PATH_INFO_INPUT_SOL)
			CDBG("%s: CSID IPP SOL received\n", __func__);
		if (irq_status_ipp & CSID_PATH_INFO_INPUT_EOL)
			CDBG("%s: CSID IPP EOL received\n", __func__);
		if (irq_status_ipp & CSID_PATH_INFO_INPUT_EOF)
			CDBG("%s: CSID IPP EOF received\n", __func__);

		if (irq_status_ipp & CSID_PATH_INFO_INPUT_EOF)
			complete(&csid_hw->csid_ipp_complete);

		if (irq_status_ipp & CSID_PATH_ERROR_FIFO_OVERFLOW) {
			pr_err("%s:%d:CSID:%d IPP fifo over flow\n",
				__func__, __LINE__,
				csid_hw->hw_intf->hw_idx);
			/*Stop IPP path immediately */
			cam_io_w_mb(CAM_CSID_HALT_IMMEDIATELY,
				soc_info->reg_map[0].mem_base +
				csid_reg->ipp_reg->csid_ipp_ctrl_addr);
		}
	}

	for (i = 0; i < csid_reg->cmn_reg->no_rdis; i++) {
		if (irq_status_rdi[i] &
			BIT(csid_reg->cmn_reg->path_rst_done_shift_val)) {
			CDBG("%s:%d: CSID rdi%d reset complete\n",
				__func__, __LINE__, i);
			complete(&csid_hw->csid_rdin_complete[i]);
		}

		if (irq_status_rdi[i] & CSID_PATH_INFO_INPUT_EOF)
			complete(&csid_hw->csid_rdin_complete[i]);

		if (irq_status_rdi[i] & CSID_PATH_ERROR_FIFO_OVERFLOW) {
			pr_err("%s:%d:CSID:%d RDI fifo over flow\n",
				__func__, __LINE__,
				csid_hw->hw_intf->hw_idx);
			/*Stop RDI path immediately */
			cam_io_w_mb(CAM_CSID_HALT_IMMEDIATELY,
				soc_info->reg_map[0].mem_base +
				csid_reg->rdi_reg[i]->csid_rdi_ctrl_addr);
		}
	}

	CDBG("%s:%d:IRQ Handling exit\n", __func__, __LINE__);
	return IRQ_HANDLED;
}

int cam_ife_csid_hw_probe_init(struct cam_hw_intf  *csid_hw_intf,
	uint32_t csid_idx)
{
	int rc = -EINVAL;
	uint32_t i;
	struct cam_ife_csid_path_cfg         *path_data;
	struct cam_ife_csid_cid_data         *cid_data;
	struct cam_hw_info                   *csid_hw_info;
	struct cam_ife_csid_hw               *ife_csid_hw = NULL;

	if (csid_idx >= CAM_IFE_CSID_HW_RES_MAX) {
		pr_err("%s:%d: Invalid csid index:%d\n", __func__, __LINE__,
			csid_idx);
		return rc;
	}

	csid_hw_info = (struct cam_hw_info  *) csid_hw_intf->hw_priv;
	ife_csid_hw  = (struct cam_ife_csid_hw  *) csid_hw_info->core_info;

	ife_csid_hw->hw_intf = csid_hw_intf;
	ife_csid_hw->hw_info = csid_hw_info;

	CDBG("%s:%d: type %d index %d\n", __func__, __LINE__,
		ife_csid_hw->hw_intf->hw_type, csid_idx);


	ife_csid_hw->hw_info->hw_state = CAM_HW_STATE_POWER_DOWN;
	mutex_init(&ife_csid_hw->hw_info->hw_mutex);
	spin_lock_init(&ife_csid_hw->hw_info->hw_lock);
	init_completion(&ife_csid_hw->hw_info->hw_complete);

	init_completion(&ife_csid_hw->csid_top_complete);
	init_completion(&ife_csid_hw->csid_csi2_complete);
	init_completion(&ife_csid_hw->csid_ipp_complete);
	for (i = 0; i < CAM_IFE_CSID_RDI_MAX; i++)
		init_completion(&ife_csid_hw->csid_rdin_complete[i]);


	rc = cam_ife_csid_init_soc_resources(&ife_csid_hw->hw_info->soc_info,
			cam_ife_csid_irq, ife_csid_hw);
	if (rc < 0) {
		pr_err("%s:%d:CSID:%d Failed to init_soc\n", __func__, __LINE__,
			csid_idx);
		goto err;
	}

	ife_csid_hw->hw_intf->hw_ops.get_hw_caps = cam_ife_csid_get_hw_caps;
	ife_csid_hw->hw_intf->hw_ops.init        = cam_ife_csid_init_hw;
	ife_csid_hw->hw_intf->hw_ops.deinit      = cam_ife_csid_deinit_hw;
	ife_csid_hw->hw_intf->hw_ops.reset       = cam_ife_csid_reset;
	ife_csid_hw->hw_intf->hw_ops.reserve     = cam_ife_csid_reserve;
	ife_csid_hw->hw_intf->hw_ops.release     = cam_ife_csid_release;
	ife_csid_hw->hw_intf->hw_ops.start       = cam_ife_csid_start;
	ife_csid_hw->hw_intf->hw_ops.stop        = cam_ife_csid_stop;
	ife_csid_hw->hw_intf->hw_ops.read        = cam_ife_csid_read;
	ife_csid_hw->hw_intf->hw_ops.write       = cam_ife_csid_write;
	ife_csid_hw->hw_intf->hw_ops.process_cmd = cam_ife_csid_process_cmd;

	/*Initialize the CID resoure */
	for (i = 0; i < CAM_IFE_CSID_CID_RES_MAX; i++) {
		ife_csid_hw->cid_res[i].res_type = CAM_ISP_RESOURCE_CID;
		ife_csid_hw->cid_res[i].res_id = i;
		ife_csid_hw->cid_res[i].res_state  =
					CAM_ISP_RESOURCE_STATE_AVAILABLE;
		ife_csid_hw->cid_res[i].hw_intf = ife_csid_hw->hw_intf;

		cid_data = kzalloc(sizeof(struct cam_ife_csid_cid_data),
					GFP_KERNEL);
		if (!cid_data) {
			rc = -ENOMEM;
			goto err;
		}
		ife_csid_hw->cid_res[i].res_priv = cid_data;
	}

	/* Initialize the IPP resources */
	if (ife_csid_hw->csid_info->csid_reg->cmn_reg->no_pix) {
		ife_csid_hw->ipp_res.res_type = CAM_ISP_RESOURCE_PIX_PATH;
		ife_csid_hw->ipp_res.res_id = CAM_IFE_PIX_PATH_RES_IPP;
		ife_csid_hw->ipp_res.res_state =
			CAM_ISP_RESOURCE_STATE_AVAILABLE;
		ife_csid_hw->ipp_res.hw_intf = ife_csid_hw->hw_intf;
		path_data = kzalloc(sizeof(struct cam_ife_csid_path_cfg),
					GFP_KERNEL);
		if (!path_data) {
			rc = -ENOMEM;
			goto err;
		}
		ife_csid_hw->ipp_res.res_priv = path_data;
	}

	/* Initialize the RDI resource */
	for (i = 0; i < ife_csid_hw->csid_info->csid_reg->cmn_reg->no_rdis;
				i++) {
		/* res type is from RDI 0 to RDI3 */
		ife_csid_hw->rdi_res[i].res_type =
			CAM_ISP_RESOURCE_PIX_PATH;
		ife_csid_hw->rdi_res[i].res_id = i;
		ife_csid_hw->rdi_res[i].res_state =
			CAM_ISP_RESOURCE_STATE_AVAILABLE;
		ife_csid_hw->rdi_res[i].hw_intf = ife_csid_hw->hw_intf;

		path_data = kzalloc(sizeof(struct cam_ife_csid_path_cfg),
					GFP_KERNEL);
		if (!path_data) {
			rc = -ENOMEM;
			goto err;
		}
		ife_csid_hw->rdi_res[i].res_priv = path_data;
	}

	return 0;
err:
	if (rc) {
		kfree(ife_csid_hw->ipp_res.res_priv);
		for (i = 0; i <
			ife_csid_hw->csid_info->csid_reg->cmn_reg->no_rdis; i++)
			kfree(ife_csid_hw->rdi_res[i].res_priv);

		for (i = 0; i < CAM_IFE_CSID_CID_RES_MAX; i++)
			kfree(ife_csid_hw->cid_res[i].res_priv);

	}

	return rc;
}


int cam_ife_csid_hw_deinit(struct cam_ife_csid_hw *ife_csid_hw)
{
	int rc = -EINVAL;
	uint32_t i;

	if (!ife_csid_hw) {
		pr_err("%s:%d: Invalid param\n", __func__, __LINE__);
		return rc;
	}

	/* release the privdate data memory from resources */
	kfree(ife_csid_hw->ipp_res.res_priv);
	for (i = 0; i <
		ife_csid_hw->csid_info->csid_reg->cmn_reg->no_rdis;
		i++) {
		kfree(ife_csid_hw->rdi_res[i].res_priv);
	}
	for (i = 0; i < CAM_IFE_CSID_CID_RES_MAX; i++)
		kfree(ife_csid_hw->cid_res[i].res_priv);


	return 0;
}


