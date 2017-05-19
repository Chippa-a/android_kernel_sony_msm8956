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

#include <linux/debugfs.h>
#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "cam_isp_context.h"
#include "cam_isp_log.h"
#include "cam_mem_mgr.h"
#include "cam_sync_api.h"

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

static int __cam_isp_ctx_handle_buf_done_in_activated_state(
	struct cam_isp_context *ctx_isp,
	struct cam_isp_hw_done_event_data *done,
	uint32_t bubble_state)
{
	int rc = 0;
	int i, j;
	struct cam_ctx_request  *req;
	struct cam_isp_ctx_req  *req_isp;
	struct cam_context *ctx = ctx_isp->base;

	if (list_empty(&ctx->active_req_list)) {
		CDBG("Buf done with no active request!\n");
		goto end;
	}

	CDBG("%s: Enter with bubble_state %d\n", __func__, bubble_state);

	req = list_first_entry(&ctx->active_req_list,
			struct cam_ctx_request, list);
	req_isp = (struct cam_isp_ctx_req *) req->req_priv;
	for (i = 0; i < done->num_handles; i++) {
		for (j = 0; j < req_isp->num_fence_map_out; j++) {
			if (done->resource_handle[i] ==
				req_isp->fence_map_out[j].resource_handle)
			break;
		}

		if (j == req_isp->num_fence_map_out) {
			pr_err("Can not find matching lane handle 0x%x!\n",
				done->resource_handle[i]);
			rc = -EINVAL;
			continue;
		}

		if (!bubble_state) {
			CDBG("%s: Sync with success: fd 0x%x\n", __func__,
				   req_isp->fence_map_out[j].sync_id);
			rc = cam_sync_signal(req_isp->fence_map_out[j].sync_id,
				CAM_SYNC_STATE_SIGNALED_SUCCESS);
			if (rc)
				pr_err("%s: Sync failed with rc = %d\n",
					__func__, rc);

		} else if (!req_isp->bubble_report) {
			CDBG("%s: Sync with failure: fd 0x%x\n", __func__,
				   req_isp->fence_map_out[j].sync_id);
			rc = cam_sync_signal(req_isp->fence_map_out[j].sync_id,
				CAM_SYNC_STATE_SIGNALED_ERROR);
			if (rc)
				pr_err("%s: Sync failed with rc = %d\n",
					__func__, rc);
		} else {
			/*
			 * Ignore the buffer done if bubble detect is on
			 * In most case, active list should be empty when
			 * bubble detects. But for safety, we just move the
			 * current active request to the pending list here.
			 */
			list_del_init(&req->list);
			list_add(&req->list, &ctx->pending_req_list);
			continue;
		}

		CDBG("%s: req %lld, reset sync id 0x%x\n", __func__,
			   req->request_id,
			   req_isp->fence_map_out[j].sync_id);
		req_isp->num_acked++;
		req_isp->fence_map_out[j].sync_id = -1;
	}

	if (req_isp->num_acked == req_isp->num_fence_map_out) {
		list_del_init(&req->list);
		list_add_tail(&req->list, &ctx->free_req_list);
	}

end:
	return rc;
}

static int __cam_isp_ctx_reg_upd_in_activated_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_ctx_request  *req;
	struct cam_context      *ctx = ctx_isp->base;
	struct cam_isp_ctx_req  *req_isp;

	if (list_empty(&ctx->pending_req_list)) {
		pr_err("Reg upd ack with no pending request\n");
		goto end;
	}
	req = list_first_entry(&ctx->pending_req_list,
			struct cam_ctx_request, list);
	list_del_init(&req->list);

	req_isp = (struct cam_isp_ctx_req *) req->req_priv;
	if (req_isp->num_fence_map_out != 0) {
		CDBG("%s: move request %lld to active list\n", __func__,
			req->request_id);
		if (!list_empty(&ctx->active_req_list))
			pr_err("%s: More than one entry in active list\n",
				__func__);
		list_add_tail(&req->list, &ctx->active_req_list);
	} else {
		/* no io config, so the request is completed. */
		list_add_tail(&req->list, &ctx->free_req_list);
	}

	/*
	 * This function only called directly from applied and bubble applied
	 * state so change substate here.
	 */
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_EPOCH;
	CDBG("%s: next substate %d\n", __func__, ctx_isp->substate_activated);

end:
	return rc;
}

static int __cam_isp_ctx_notify_sof_in_actived_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_req_mgr_sof_notify  notify;
	struct cam_context *ctx = ctx_isp->base;

	/* notify reqmgr with sof  signal */
	if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_sof) {
		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.frame_id = ctx_isp->frame_id;

		ctx->ctx_crm_intf->notify_sof(&notify);
		CDBG("%s: Notify CRM  SOF frame %lld\n", __func__,
			ctx_isp->frame_id);
	} else {
		pr_err("%s: Can not notify SOF to CRM\n", __func__);
	}

	return rc;
}


static int __cam_isp_ctx_sof_in_sof(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;

	CDBG("%s: Enter\n", __func__);
	ctx_isp->frame_id++;

	return rc;
}

static int __cam_isp_ctx_reg_upd_in_sof(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;
	struct cam_ctx_request *req;
	struct cam_isp_ctx_req *req_isp;
	struct cam_context *ctx = ctx_isp->base;

	if (ctx->state != CAM_CTX_ACTIVATED) {
		CDBG("%s: invalid RUP\n", __func__);
		goto end;
	}

	/*
	 * This is for the first update. The initial setting will
	 * cause the reg_upd in the first frame.
	 */
	if (!list_empty(&ctx->pending_req_list)) {
		req = list_first_entry(&ctx->pending_req_list,
			struct cam_ctx_request, list);
		list_del_init(&req->list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		if (req_isp->num_fence_map_out == req_isp->num_acked)
			list_add_tail(&req->list, &ctx->free_req_list);
		else {
			/* need to handle the buf done */
			list_add_tail(&req->list, &ctx->active_req_list);
			ctx_isp->substate_activated =
				CAM_ISP_CTX_ACTIVATED_EPOCH;
		}
	}
end:
	return rc;
}

static int __cam_isp_ctx_epoch_in_applied(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;
	struct cam_ctx_request    *req;
	struct cam_isp_ctx_req    *req_isp;
	struct cam_context        *ctx = ctx_isp->base;

	if (list_empty(&ctx->pending_req_list)) {
		/*
		 * If no pending req in epoch, this is an error case.
		 * The recovery is to go back to sof state
		 */
		pr_err("%s: No pending request\n", __func__);
		ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;
		goto end;
	}

	req = list_first_entry(&ctx->pending_req_list, struct cam_ctx_request,
		list);
	req_isp = (struct cam_isp_ctx_req *)req->req_priv;

	CDBG("Report Bubble flag %d\n", req_isp->bubble_report);
	if (req_isp->bubble_report && ctx->ctx_crm_intf &&
		ctx->ctx_crm_intf->notify_err) {
		struct cam_req_mgr_error_notify notify;

		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.req_id = req->request_id;
		notify.error = CRM_KMD_ERR_BUBBLE;
		ctx->ctx_crm_intf->notify_err(&notify);
		CDBG("%s: Notify CRM about Bubble frame %lld\n", __func__,
			ctx_isp->frame_id);
	} else {
		/*
		 * Since can not bubble report, always move the request to
		 * active list.
		 */
		list_del_init(&req->list);
		list_add_tail(&req->list, &ctx->active_req_list);
		req_isp->bubble_report = 0;
	}

	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_BUBBLE;
	CDBG("%s: next substate %d\n", __func__,
		ctx_isp->substate_activated);
end:
	return rc;
}


static int __cam_isp_ctx_buf_done_in_applied(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;
	struct cam_isp_hw_done_event_data *done =
		(struct cam_isp_hw_done_event_data *) evt_data;

	rc = __cam_isp_ctx_handle_buf_done_in_activated_state(ctx_isp, done, 0);
	return rc;
}


static int __cam_isp_ctx_sof_in_epoch(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;

	ctx_isp->frame_id++;
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;
	CDBG("%s: next substate %d\n", __func__,
		ctx_isp->substate_activated);

	return rc;
}

static int __cam_isp_ctx_buf_done_in_epoch(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;
	struct cam_isp_hw_done_event_data *done =
		(struct cam_isp_hw_done_event_data *) evt_data;

	rc = __cam_isp_ctx_handle_buf_done_in_activated_state(ctx_isp, done, 0);
	return rc;
}


static int __cam_isp_ctx_sof_in_bubble(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	ctx_isp->frame_id++;
	return 0;
}

static int __cam_isp_ctx_buf_done_in_bubble(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_isp_hw_done_event_data *done =
		(struct cam_isp_hw_done_event_data *) evt_data;

	rc = __cam_isp_ctx_handle_buf_done_in_activated_state(ctx_isp, done, 1);
	return rc;
}

static int __cam_isp_ctx_sof_in_bubble_applied(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	ctx_isp->frame_id++;
	return 0;
}


static int __cam_isp_ctx_epoch_in_bubble_applied(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	struct cam_ctx_request    *req;
	struct cam_isp_ctx_req    *req_isp;
	struct cam_context        *ctx = ctx_isp->base;

	/*
	 * This means we missed the reg upd ack. So we need to
	 * transition to BUBBLE state again.
	 */

	if (list_empty(&ctx->pending_req_list)) {
		/*
		 * If no pending req in epoch, this is an error case.
		 * Just go back to the bubble state.
		 */
		pr_err("%s: No pending request.\n", __func__);
		ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_BUBBLE;
		goto end;
	}

	req = list_first_entry(&ctx->pending_req_list, struct cam_ctx_request,
		list);
	req_isp = (struct cam_isp_ctx_req *)req->req_priv;

	if (req_isp->bubble_report && ctx->ctx_crm_intf &&
		ctx->ctx_crm_intf->notify_err) {
		struct cam_req_mgr_error_notify notify;

		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.req_id = req->request_id;
		notify.error = CRM_KMD_ERR_BUBBLE;
		ctx->ctx_crm_intf->notify_err(&notify);
		CDBG("%s: Notify CRM about Bubble frame %lld\n", __func__,
			ctx_isp->frame_id);
	} else {
		/*
		 * If we can not report bubble, then treat it as if no bubble
		 * report. Just move the req to active list.
		 */
		list_del_init(&req->list);
		list_add_tail(&req->list, &ctx->active_req_list);
		req_isp->bubble_report = 0;
	}

	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_BUBBLE;
	CDBG("%s: next substate %d\n", __func__, ctx_isp->substate_activated);
end:
	return 0;
}

static int __cam_isp_ctx_buf_done_in_bubble_applied(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_isp_hw_done_event_data *done =
		(struct cam_isp_hw_done_event_data *) evt_data;

	rc = __cam_isp_ctx_handle_buf_done_in_activated_state(ctx_isp, done, 1);
	return rc;
}

static int __cam_isp_ctx_handle_error(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int                              rc = 0;
	struct cam_ctx_request          *req;
	struct cam_req_mgr_error_notify  notify;

	struct cam_context *ctx = ctx_isp->base;
	struct cam_isp_hw_error_event_data  *error_event_data =
			(struct cam_isp_hw_error_event_data *)evt_data;

	uint32_t error_type = error_event_data->error_type;

	CDBG("%s: Enter error_type = %d\n", __func__, error_type);
	if ((error_type == CAM_ISP_HW_ERROR_OVERFLOW) ||
		(error_type == CAM_ISP_HW_ERROR_BUSIF_OVERFLOW))
		notify.error = CRM_KMD_ERR_FATAL;

	/*
	 * Need to check the active req
	 * move all of them to the pending request list
	 * Note this funciton need revisit!
	 */

	if (list_empty(&ctx->active_req_list)) {
		pr_err("handling error with no active request!\n");
		rc = -EINVAL;
		goto end;
	}

	req = list_first_entry(&ctx->active_req_list,
				struct cam_ctx_request, list);

	if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_err) {
		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.req_id = req->request_id;

		ctx->ctx_crm_intf->notify_err(&notify);
		pr_err("%s: Notify CRM about ERROR frame %lld\n", __func__,
			ctx_isp->frame_id);
	} else {
		pr_err("%s: Can not notify ERRROR to CRM\n", __func__);
		rc = -EFAULT;
	}

	list_del_init(&req->list);
	list_add(&req->list, &ctx->pending_req_list);
	/* might need to check if active list is empty */

end:
	CDBG("%s: Exit\n", __func__);
	return rc;
}

static struct cam_isp_ctx_irq_ops
	cam_isp_ctx_activated_state_machine_irq[CAM_ISP_CTX_ACTIVATED_MAX] = {
	/* SOF */
	{
		.irq_ops = {
			NULL,
			__cam_isp_ctx_sof_in_sof,
			__cam_isp_ctx_reg_upd_in_sof,
			__cam_isp_ctx_notify_sof_in_actived_state,
			NULL,
			NULL,
		},
	},
	/* APPLIED */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_sof,
			__cam_isp_ctx_reg_upd_in_activated_state,
			__cam_isp_ctx_epoch_in_applied,
			NULL,
			__cam_isp_ctx_buf_done_in_applied,
		},
	},
	/* EPOCH */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_epoch,
			NULL,
			__cam_isp_ctx_notify_sof_in_actived_state,
			NULL,
			__cam_isp_ctx_buf_done_in_epoch,
		},
	},
	/* BUBBLE */
	{
		.irq_ops = {
			NULL,
			__cam_isp_ctx_sof_in_bubble,
			NULL,
			__cam_isp_ctx_notify_sof_in_actived_state,
			NULL,
			__cam_isp_ctx_buf_done_in_bubble,
		},
	},
	/* Bubble Applied */
	{
		.irq_ops = {
			NULL,
			__cam_isp_ctx_sof_in_bubble_applied,
			__cam_isp_ctx_reg_upd_in_activated_state,
			__cam_isp_ctx_epoch_in_bubble_applied,
			NULL,
			__cam_isp_ctx_buf_done_in_bubble_applied,
		},
	},
	/* HALT */
	{
	},
};

static int __cam_isp_ctx_apply_req_in_activated_state(
	struct cam_context *ctx, struct cam_req_mgr_apply_request *apply,
	uint32_t next_state)
{
	int rc = 0;
	int cnt = 0;
	struct cam_ctx_request          *req;
	struct cam_isp_ctx_req          *req_isp;
	struct cam_isp_context          *ctx_isp;
	struct cam_hw_config_args        cfg;

	if (list_empty(&ctx->pending_req_list)) {
		pr_err("%s: No available request for Apply id %lld\n",
			__func__, apply->request_id);
		rc = -EFAULT;
		goto end;
	}

	/*
	 * When the pipeline has issue, the requests can be queued up in the
	 * pipeline. In this case, we should reject the additional request.
	 * The maximum number of request allowed to be outstanding is 2.
	 *
	 */
	list_for_each_entry(req, &ctx->active_req_list, list) {
		if (++cnt > 2) {
			pr_err("%s: Apply failed due to pipeline congestion\n",
				__func__);
			rc = -EFAULT;
			goto end;
		}
	}

	req = list_first_entry(&ctx->pending_req_list, struct cam_ctx_request,
		list);

	/*
	 * Check whehter the request id is matching the tip, if not, this means
	 * we are in the middle of the error handling. Need to reject this apply
	 */
	if (req->request_id != apply->request_id) {
		rc = -EFAULT;
		goto end;
	}

	CDBG("%s: Apply request %lld\n", __func__, req->request_id);
	req_isp = (struct cam_isp_ctx_req *) req->req_priv;
	ctx_isp = (struct cam_isp_context *) ctx->ctx_priv;

	req_isp->bubble_report = apply->report_if_bubble;

	cfg.ctxt_to_hw_map = ctx_isp->hw_ctx;
	cfg.hw_update_entries = req_isp->cfg;
	cfg.num_hw_update_entries = req_isp->num_cfg;

	rc = ctx->hw_mgr_intf->hw_config(ctx->hw_mgr_intf->hw_mgr_priv, &cfg);
	if (rc) {
		pr_err("%s: Can not apply the configuration\n", __func__);
	} else {
		spin_lock(&ctx->lock);
		ctx_isp->substate_activated = next_state;
		CDBG("%s: new state %d\n", __func__, next_state);
		spin_unlock(&ctx->lock);
	}
end:
	return rc;
}

static int __cam_isp_ctx_apply_req_in_sof(
	struct cam_context *ctx, struct cam_req_mgr_apply_request *apply)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	CDBG("%s: current substate %d\n", __func__,
		ctx_isp->substate_activated);
	rc = __cam_isp_ctx_apply_req_in_activated_state(ctx, apply,
		CAM_ISP_CTX_ACTIVATED_APPLIED);
	CDBG("%s: new substate %d\n", __func__, ctx_isp->substate_activated);

	return rc;
}

static int __cam_isp_ctx_apply_req_in_epoch(
	struct cam_context *ctx, struct cam_req_mgr_apply_request *apply)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	CDBG("%s: current substate %d\n", __func__,
		ctx_isp->substate_activated);
	rc = __cam_isp_ctx_apply_req_in_activated_state(ctx, apply,
		CAM_ISP_CTX_ACTIVATED_APPLIED);
	CDBG("%s: new substate %d\n", __func__, ctx_isp->substate_activated);

	return rc;
}

static int __cam_isp_ctx_apply_req_in_bubble(
	struct cam_context *ctx, struct cam_req_mgr_apply_request *apply)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	CDBG("%s: current substate %d\n", __func__,
		ctx_isp->substate_activated);
	rc = __cam_isp_ctx_apply_req_in_activated_state(ctx, apply,
		CAM_ISP_CTX_ACTIVATED_BUBBLE_APPLIED);
	CDBG("%s: new substate %d\n", __func__, ctx_isp->substate_activated);

	return rc;
}

static struct cam_ctx_ops
	cam_isp_ctx_activated_state_machine[CAM_ISP_CTX_ACTIVATED_MAX] = {
	/* SOF */
	{
		.ioctl_ops = {},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_apply_req_in_sof,
		},
		.irq_ops = NULL,
	},
	/* APPLIED */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* EPOCH */
	{
		.ioctl_ops = {},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_apply_req_in_epoch,
		},
		.irq_ops = NULL,
	},
	/* BUBBLE */
	{
		.ioctl_ops = {},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_apply_req_in_bubble,
		},
		.irq_ops = NULL,
	},
	/* Bubble Applied */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* HALT */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
};


/* top level state machine */
static int __cam_isp_ctx_release_dev_in_top_state(struct cam_context *ctx,
	struct cam_release_dev_cmd *cmd)
{
	int rc = 0;
	int i;
	struct cam_hw_release_args       rel_arg;
	struct cam_ctx_request	        *req;
	struct cam_isp_ctx_req	        *req_isp;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	if (ctx_isp->hw_ctx) {
		rel_arg.ctxt_to_hw_map = ctx_isp->hw_ctx;
		ctx->hw_mgr_intf->hw_release(ctx->hw_mgr_intf->hw_mgr_priv,
			&rel_arg);
		ctx_isp->hw_ctx = NULL;
	}

	ctx->session_hdl = 0;
	ctx->dev_hdl = 0;
	ctx->link_hdl = 0;
	ctx->ctx_crm_intf = NULL;
	ctx_isp->frame_id = 0;

	/*
	 * Ideally, we should never have any active request here.
	 * But we still add some sanity check code here to help the debug
	 */
	if (!list_empty(&ctx->active_req_list))
		pr_err("%s: Active list is empty.\n", __func__);

	/* flush the pending list */
	while (!list_empty(&ctx->pending_req_list)) {
		req = list_first_entry(&ctx->pending_req_list,
			struct cam_ctx_request, list);
		list_del_init(&req->list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		pr_err("%s: signal fence in pending list. fence num %d\n",
			__func__, req_isp->num_fence_map_out);
		for (i = 0; i < req_isp->num_fence_map_out; i++) {
			if (req_isp->fence_map_out[i].sync_id != -1) {
				cam_sync_signal(
					req_isp->fence_map_out[i].sync_id,
					CAM_SYNC_STATE_SIGNALED_ERROR);
			}
		}
		list_add_tail(&req->list, &ctx->free_req_list);
	}
	ctx->state = CAM_CTX_AVAILABLE;
	CDBG("%s: next state %d\n", __func__, ctx->state);
	return rc;
}

static int __cam_isp_ctx_config_dev_in_top_state(
	struct cam_context *ctx, struct cam_config_dev_cmd *cmd)
{
	int rc = 0;
	struct cam_ctx_request           *req = NULL;
	struct cam_isp_ctx_req           *req_isp;
	uint64_t                          packet_addr;
	struct cam_packet                *packet;
	size_t                            len = 0;
	struct cam_hw_prepare_update_args cfg;
	struct cam_req_mgr_add_request    add_req;
	struct cam_isp_context           *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	CDBG("%s: get free request object......\n", __func__);

	/* get free request */
	spin_lock(&ctx->lock);
	if (!list_empty(&ctx->free_req_list)) {
		req = list_first_entry(&ctx->free_req_list,
				struct cam_ctx_request, list);
		list_del_init(&req->list);
	}
	spin_unlock(&ctx->lock);

	if (!req) {
		pr_err("%s: No more request obj free\n", __func__);
		rc = -ENOMEM;
		goto end;
	}

	req_isp = (struct cam_isp_ctx_req *) req->req_priv;

	/* for config dev, only memory handle is supported */
	/* map packet from the memhandle */
	rc = cam_mem_get_cpu_buf((int32_t) cmd->packet_handle,
		(uint64_t *) &packet_addr, &len);
	if (rc != 0) {
		pr_err("%s: Can not get packet address\n", __func__);
		rc = -EINVAL;
		goto free_req;
	}

	packet = (struct cam_packet *) (packet_addr + cmd->offset);
	CDBG("%s: pack_handle %llx\n", __func__, cmd->packet_handle);
	CDBG("%s: packet address is 0x%llx\n", __func__, packet_addr);
	CDBG("%s: packet with length %zu, offset 0x%llx\n", __func__,
		len, cmd->offset);
	CDBG("%s: Packet request id 0x%llx\n", __func__,
		packet->header.request_id);
	CDBG("%s: Packet size 0x%x\n", __func__, packet->header.size);
	CDBG("%s: packet op %d\n", __func__, packet->header.op_code);

	/* preprocess the configuration */
	memset(&cfg, 0, sizeof(cfg));
	cfg.packet = packet;
	cfg.ctxt_to_hw_map = ctx_isp->hw_ctx;
	cfg.max_hw_update_entries = CAM_ISP_CTX_CFG_MAX;
	cfg.hw_update_entries = req_isp->cfg;
	cfg.max_out_map_entries = CAM_ISP_CTX_RES_MAX;
	cfg.max_in_map_entries = CAM_ISP_CTX_RES_MAX;
	cfg.out_map_entries = req_isp->fence_map_out;
	cfg.in_map_entries = req_isp->fence_map_in;

	CDBG("%s: try to prepare config packet......\n", __func__);

	rc = ctx->hw_mgr_intf->hw_prepare_update(
		ctx->hw_mgr_intf->hw_mgr_priv, &cfg);
	if (rc != 0) {
		pr_err("%s: Prepare config packet failed in HW layer\n",
			__func__);
		rc = -EFAULT;
		goto free_req;
	}
	req_isp->num_cfg = cfg.num_hw_update_entries;
	req_isp->num_fence_map_out = cfg.num_out_map_entries;
	req_isp->num_fence_map_in = cfg.num_in_map_entries;
	req_isp->num_acked = 0;

	CDBG("%s: num_entry: %d, num fence out: %d, num fence in: %d\n",
		__func__, req_isp->num_cfg, req_isp->num_fence_map_out,
		req_isp->num_fence_map_in);

	req->request_id = packet->header.request_id;
	req->status = 1;

	if (ctx->state == CAM_CTX_ACTIVATED && ctx->ctx_crm_intf->add_req) {
		add_req.link_hdl = ctx->link_hdl;
		add_req.dev_hdl  = ctx->dev_hdl;
		add_req.req_id   = req->request_id;
		rc = ctx->ctx_crm_intf->add_req(&add_req);
		if (rc) {
			pr_err("%s: Error: Adding request id=%llu\n", __func__,
				req->request_id);
				goto free_req;
		}
	}

	CDBG("%s: Packet request id 0x%llx\n", __func__,
		packet->header.request_id);

	spin_lock(&ctx->lock);
	list_add_tail(&req->list, &ctx->pending_req_list);
	spin_unlock(&ctx->lock);

	CDBG("%s: Preprocessing Config %lld successful\n", __func__,
		req->request_id);

	return rc;

free_req:
	spin_lock(&ctx->lock);
	list_add_tail(&req->list, &ctx->free_req_list);
	spin_unlock(&ctx->lock);
end:
	return rc;
}

static int __cam_isp_ctx_acquire_dev_in_available(struct cam_context *ctx,
	struct cam_acquire_dev_cmd *cmd)
{
	int rc = 0;
	struct cam_hw_acquire_args       param;
	struct cam_isp_resource         *isp_res = NULL;
	struct cam_create_dev_hdl        req_hdl_param;
	struct cam_hw_release_args       release;
	struct cam_isp_context          *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	if (!ctx->hw_mgr_intf) {
		pr_err("HW interface is not ready!\n");
		rc = -EFAULT;
		goto end;
	}

	CDBG("%s: session_hdl 0x%x, num_resources %d, hdl type %d, res %lld\n",
		 __func__, cmd->session_handle, cmd->num_resources,
		cmd->handle_type, cmd->resource_hdl);

	if (cmd->num_resources > CAM_ISP_CTX_RES_MAX) {
		pr_err("Too much resources in the acquire!\n");
		rc = -ENOMEM;
		goto end;
	}

	/* for now we only support user pointer */
	if (cmd->handle_type != 1)  {
		pr_err("%s: Only user pointer is supported!", __func__);
		rc = -EINVAL;
		goto end;
	}

	isp_res = kzalloc(
		sizeof(*isp_res)*cmd->num_resources, GFP_KERNEL);
	if (!isp_res) {
		rc = -ENOMEM;
		goto end;
	}

	CDBG("%s: start copy %d resources from user\n",
		__func__, cmd->num_resources);

	if (copy_from_user(isp_res, (void __user *)cmd->resource_hdl,
		sizeof(*isp_res)*cmd->num_resources)) {
		rc = -EFAULT;
		goto free_res;
	}

	param.context_data = ctx;
	param.event_cb = ctx->irq_cb_intf;
	param.num_acq = cmd->num_resources;
	param.acquire_info = (uint64_t) isp_res;

	/* call HW manager to reserve the resource */
	rc = ctx->hw_mgr_intf->hw_acquire(ctx->hw_mgr_intf->hw_mgr_priv,
		&param);
	if (rc != 0) {
		pr_err("Acquire device failed\n");
		goto free_res;
	}

	ctx_isp->hw_ctx = param.ctxt_to_hw_map;

	req_hdl_param.session_hdl = cmd->session_handle;
	/* bridge is not ready for these flags. so false for now */
	req_hdl_param.v4l2_sub_dev_flag = 0;
	req_hdl_param.media_entity_flag = 0;
	req_hdl_param.ops = ctx->crm_ctx_intf;
	req_hdl_param.priv = ctx;

	CDBG("%s: get device handle form bridge\n", __func__);
	ctx->dev_hdl = cam_create_device_hdl(&req_hdl_param);
	if (ctx->dev_hdl <= 0) {
		rc = -EFAULT;
		pr_err("Can not create device handle\n");
		goto free_hw;
	}
	cmd->dev_handle = ctx->dev_hdl;

	/* store session information */
	ctx->session_hdl = cmd->session_handle;

	ctx->state = CAM_CTX_ACQUIRED;

	CDBG("%s:%d: Acquire success.\n", __func__, __LINE__);
	kfree(isp_res);
	return rc;

free_hw:
	release.ctxt_to_hw_map = ctx_isp->hw_ctx;
	ctx->hw_mgr_intf->hw_release(ctx->hw_mgr_intf->hw_mgr_priv, &release);
	ctx_isp->hw_ctx = NULL;
free_res:
	kfree(isp_res);
end:
	return rc;
}

static int __cam_isp_ctx_config_dev_in_acquired(struct cam_context *ctx,
	struct cam_config_dev_cmd *cmd)
{
	int rc = 0;

	rc = __cam_isp_ctx_config_dev_in_top_state(ctx, cmd);

	if (!rc && ctx->link_hdl)
		ctx->state = CAM_CTX_READY;

	CDBG("%s: next state %d\n", __func__, ctx->state);
	return rc;
}

static int __cam_isp_ctx_link_in_acquired(struct cam_context *ctx,
	struct cam_req_mgr_core_dev_link_setup *link)
{
	int rc = 0;

	CDBG("%s:%d: Enter.........\n", __func__, __LINE__);

	ctx->link_hdl = link->link_hdl;
	ctx->ctx_crm_intf = link->crm_cb;

	/* change state only if we had the init config */
	if (!list_empty(&ctx->pending_req_list))
		ctx->state = CAM_CTX_READY;

	CDBG("%s: next state %d\n", __func__, ctx->state);

	return rc;
}

static int __cam_isp_ctx_unlink_in_acquired(struct cam_context *ctx,
	struct cam_req_mgr_core_dev_link_setup *unlink)
{
	int rc = 0;

	ctx->link_hdl = 0;
	ctx->ctx_crm_intf = NULL;

	return rc;
}

static int __cam_isp_ctx_get_dev_info_in_acquired(struct cam_context *ctx,
	struct cam_req_mgr_device_info *dev_info)
{
	int rc = 0;

	dev_info->dev_hdl = ctx->dev_hdl;
	strlcpy(dev_info->name, CAM_ISP_DEV_NAME, sizeof(dev_info->name));
	dev_info->dev_id = CAM_REQ_MGR_DEVICE_IFE;
	dev_info->p_delay = 1;

	return rc;
}

static int __cam_isp_ctx_start_dev_in_ready(struct cam_context *ctx,
	struct cam_start_stop_dev_cmd *cmd)
{
	int rc = 0;
	struct cam_hw_start_args         arg;
	struct cam_ctx_request          *req;
	struct cam_isp_ctx_req          *req_isp;
	struct cam_isp_context          *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	if (cmd->session_handle != ctx->session_hdl ||
		cmd->dev_handle != ctx->dev_hdl) {
		rc = -EPERM;
		goto end;
	}

	if (list_empty(&ctx->pending_req_list)) {
		/* should never happen */
		pr_err("%s: Start device with empty configuration\n",
			__func__);
		rc = -EFAULT;
		goto end;
	} else {
		req = list_first_entry(&ctx->pending_req_list,
			struct cam_ctx_request, list);
	}
	req_isp = (struct cam_isp_ctx_req *) req->req_priv;

	if (!ctx_isp->hw_ctx) {
		pr_err("%s:%d: Wrong hw context pointer.\n",
			__func__, __LINE__);
		rc = -EFAULT;
		goto end;
	}
	arg.ctxt_to_hw_map = ctx_isp->hw_ctx;
	arg.hw_update_entries = req_isp->cfg;
	arg.num_hw_update_entries = req_isp->num_cfg;

	ctx_isp->frame_id = 0;
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;

	/*
	 * Only place to change state before calling the hw due to
	 * hardware tasklet has higher priority that can cause the
	 * irq handling comes early
	 */
	ctx->state = CAM_CTX_ACTIVATED;
	rc = ctx->hw_mgr_intf->hw_start(ctx->hw_mgr_intf->hw_mgr_priv, &arg);
	if (rc) {
		/* HW failure. user need to clean up the resource */
		pr_err("Start HW failed\n");
		ctx->state = CAM_CTX_READY;
		goto end;
	}
	CDBG("%s: start device success\n", __func__);
end:
	return rc;
}

static int __cam_isp_ctx_unlink_in_ready(struct cam_context *ctx,
	struct cam_req_mgr_core_dev_link_setup *unlink)
{
	int rc = 0;

	ctx->link_hdl = 0;
	ctx->ctx_crm_intf = NULL;
	ctx->state = CAM_CTX_ACQUIRED;

	return rc;
}

static int __cam_isp_ctx_stop_dev_in_activated_unlock(
	struct cam_context *ctx)
{
	int rc = 0;
	uint32_t i;
	struct cam_hw_stop_args          stop;
	struct cam_ctx_request          *req;
	struct cam_isp_ctx_req          *req_isp;
	struct cam_isp_context          *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	/* Mask off all the incoming hardware events */
	spin_lock(&ctx->lock);
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_HALT;
	spin_unlock(&ctx->lock);
	CDBG("%s: next substate %d", __func__, ctx_isp->substate_activated);

	/* stop hw first */
	if (ctx_isp->hw_ctx) {
		stop.ctxt_to_hw_map = ctx_isp->hw_ctx;
		ctx->hw_mgr_intf->hw_stop(ctx->hw_mgr_intf->hw_mgr_priv,
			&stop);
	}

	while (!list_empty(&ctx->pending_req_list)) {
		req = list_first_entry(&ctx->pending_req_list,
				struct cam_ctx_request, list);
		list_del_init(&req->list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		CDBG("%s: signal fence in pending list. fence num %d\n",
			__func__, req_isp->num_fence_map_out);
		for (i = 0; i < req_isp->num_fence_map_out; i++)
			if (req_isp->fence_map_out[i].sync_id != -1) {
				cam_sync_signal(
					req_isp->fence_map_out[i].sync_id,
					CAM_SYNC_STATE_SIGNALED_ERROR);
			}
		list_add_tail(&req->list, &ctx->free_req_list);
	}

	while (!list_empty(&ctx->active_req_list)) {
		req = list_first_entry(&ctx->active_req_list,
				struct cam_ctx_request, list);
		list_del_init(&req->list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		CDBG("%s: signal fence in active list. fence num %d\n",
			__func__, req_isp->num_fence_map_out);
		for (i = 0; i < req_isp->num_fence_map_out; i++)
			if (req_isp->fence_map_out[i].sync_id != -1) {
				cam_sync_signal(
					req_isp->fence_map_out[i].sync_id,
					CAM_SYNC_STATE_SIGNALED_ERROR);
			}
		list_add_tail(&req->list, &ctx->free_req_list);
	}
	ctx_isp->frame_id = 0;

	CDBG("%s: next state %d", __func__, ctx->state);
	return rc;
}

static int __cam_isp_ctx_stop_dev_in_activated(struct cam_context *ctx,
	struct cam_start_stop_dev_cmd *cmd)
{
	int rc = 0;

	__cam_isp_ctx_stop_dev_in_activated_unlock(ctx);
	ctx->state = CAM_CTX_ACQUIRED;
	return rc;
}

static int __cam_isp_ctx_release_dev_in_activated(struct cam_context *ctx,
	struct cam_release_dev_cmd *cmd)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	__cam_isp_ctx_stop_dev_in_activated_unlock(ctx);

	if (ctx_isp->hw_ctx) {
		struct cam_hw_release_args   arg;

		arg.ctxt_to_hw_map = ctx_isp->hw_ctx;
		ctx->hw_mgr_intf->hw_release(ctx->hw_mgr_intf->hw_mgr_priv,
			&arg);
		ctx_isp->hw_ctx = NULL;
	}

	ctx->session_hdl = 0;
	ctx->dev_hdl = 0;
	ctx->link_hdl = 0;
	ctx->ctx_crm_intf = NULL;

	ctx->state =  CAM_CTX_AVAILABLE;

	return rc;
}

static int __cam_isp_ctx_apply_req(struct cam_context *ctx,
	struct cam_req_mgr_apply_request *apply)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	CDBG("%s: Enter: apply req in Substate %d\n",
		__func__, ctx_isp->substate_activated);
	if (ctx_isp->substate_machine[ctx_isp->substate_activated].
		crm_ops.apply_req) {
		rc = ctx_isp->substate_machine[ctx_isp->substate_activated].
			crm_ops.apply_req(ctx, apply);
	} else {
		pr_err("%s: No handle function in activated substate %d\n",
			__func__, ctx_isp->substate_activated);
		rc = -EFAULT;
	}

	if (rc)
		pr_err("%s: Apply failed in active substate %d\n",
			__func__, ctx_isp->substate_activated);
	return rc;
}



static int __cam_isp_ctx_handle_irq_in_activated(void *context,
	uint32_t evt_id, void *evt_data)
{
	int rc = 0;
	struct cam_context *ctx = (struct cam_context *)context;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *)ctx->ctx_priv;

	spin_lock(&ctx->lock);
	CDBG("%s: Enter: State %d, Substate %d, evt id %d\n",
		__func__, ctx->state, ctx_isp->substate_activated, evt_id);
	if (ctx_isp->substate_machine_irq[ctx_isp->substate_activated].
		irq_ops[evt_id]) {
		rc = ctx_isp->substate_machine_irq[ctx_isp->substate_activated].
			irq_ops[evt_id](ctx_isp, evt_data);
	} else {
		CDBG("%s: No handle function for substate %d\n", __func__,
			ctx_isp->substate_activated);
	}
	CDBG("%s: Exit: State %d Substate %d\n",
		__func__, ctx->state, ctx_isp->substate_activated);
	spin_unlock(&ctx->lock);
	return rc;
}

/* top state machine */
static struct cam_ctx_ops
	cam_isp_ctx_top_state_machine[CAM_CTX_STATE_MAX] = {
	/* Uninit */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* Available */
	{
		.ioctl_ops = {
			.acquire_dev = __cam_isp_ctx_acquire_dev_in_available,
		},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* Acquired */
	{
		.ioctl_ops = {
			.release_dev = __cam_isp_ctx_release_dev_in_top_state,
			.config_dev = __cam_isp_ctx_config_dev_in_acquired,
		},
		.crm_ops = {
			.link = __cam_isp_ctx_link_in_acquired,
			.unlink = __cam_isp_ctx_unlink_in_acquired,
			.get_dev_info = __cam_isp_ctx_get_dev_info_in_acquired,
		},
		.irq_ops = NULL,
	},
	/* Ready */
	{
		.ioctl_ops = {
			.start_dev = __cam_isp_ctx_start_dev_in_ready,
			.release_dev = __cam_isp_ctx_release_dev_in_top_state,
			.config_dev = __cam_isp_ctx_config_dev_in_top_state,
		},
		.crm_ops = {
			.unlink = __cam_isp_ctx_unlink_in_ready,
		},
		.irq_ops = NULL,
	},
	/* Activated */
	{
		.ioctl_ops = {
			.stop_dev = __cam_isp_ctx_stop_dev_in_activated,
			.release_dev = __cam_isp_ctx_release_dev_in_activated,
			.config_dev = __cam_isp_ctx_config_dev_in_top_state,
		},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_apply_req,
		},
		.irq_ops = __cam_isp_ctx_handle_irq_in_activated,
	},
};


int cam_isp_context_init(struct cam_isp_context *ctx,
	struct cam_context *ctx_base,
	struct cam_req_mgr_kmd_ops *crm_node_intf,
	struct cam_hw_mgr_intf *hw_intf)

{
	int rc = -1;
	int i;

	if (!ctx || !ctx_base) {
		pr_err("%s: Invalid Context\n", __func__);
		goto err;
	}

	/* ISP context setup */
	memset(ctx, 0, sizeof(*ctx));

	ctx->base = ctx_base;
	ctx->frame_id = 0;
	ctx->hw_ctx = NULL;
	ctx->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;
	ctx->substate_machine = cam_isp_ctx_activated_state_machine;
	ctx->substate_machine_irq = cam_isp_ctx_activated_state_machine_irq;

	for (i = 0; i < CAM_CTX_REQ_MAX; i++) {
		ctx->req_base[i].req_priv = &ctx->req_isp[i];
		ctx->req_isp[i].base = &ctx->req_base[i];
	}

	/* camera context setup */
	rc = cam_context_init(ctx_base, crm_node_intf, hw_intf, ctx->req_base,
		CAM_CTX_REQ_MAX);
	if (rc) {
		pr_err("%s: Camera Context Base init failed\n", __func__);
		goto err;
	}

	/* link camera context with isp context */
	ctx_base->state_machine = cam_isp_ctx_top_state_machine;
	ctx_base->ctx_priv = ctx;

err:
	return rc;
}

int cam_isp_context_deinit(struct cam_isp_context *ctx)
{
	int rc = 0;

	if (ctx->base)
		cam_context_deinit(ctx->base);

	if (ctx->substate_activated != CAM_ISP_CTX_ACTIVATED_SOF)
		pr_err("%s: ISP context substate is invalid\n", __func__);

	memset(ctx, 0, sizeof(*ctx));
	return rc;
}

