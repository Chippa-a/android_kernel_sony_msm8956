/*
 * Copyright (c) 2014-2017, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__

#include <drm/drm_crtc.h>
#include <linux/debugfs.h>
#include <linux/of_irq.h>
#include <linux/dma-buf.h>

#include "msm_drv.h"
#include "msm_mmu.h"
#include "msm_gem.h"

#include "dsi_display.h"
#include "dsi_drm.h"
#include "sde_wb.h"
#include "dp_display.h"
#include "dp_drm.h"

#include "sde_kms.h"
#include "sde_core_irq.h"
#include "sde_formats.h"
#include "sde_hw_vbif.h"
#include "sde_vbif.h"
#include "sde_encoder.h"
#include "sde_plane.h"
#include "sde_crtc.h"
#include "sde_reg_dma.h"

#define CREATE_TRACE_POINTS
#include "sde_trace.h"

static const char * const iommu_ports[] = {
		"mdp_0",
};

/**
 * Controls size of event log buffer. Specified as a power of 2.
 */
#define SDE_EVTLOG_SIZE	1024

/*
 * To enable overall DRM driver logging
 * # echo 0x2 > /sys/module/drm/parameters/debug
 *
 * To enable DRM driver h/w logging
 * # echo <mask> > /sys/kernel/debug/dri/0/debug/hw_log_mask
 *
 * See sde_hw_mdss.h for h/w logging mask definitions (search for SDE_DBG_MASK_)
 */
#define SDE_DEBUGFS_DIR "msm_sde"
#define SDE_DEBUGFS_HWMASKNAME "hw_log_mask"

/**
 * sdecustom - enable certain driver customizations for sde clients
 *	Enabling this modifies the standard DRM behavior slightly and assumes
 *	that the clients have specific knowledge about the modifications that
 *	are involved, so don't enable this unless you know what you're doing.
 *
 *	Parts of the driver that are affected by this setting may be located by
 *	searching for invocations of the 'sde_is_custom_client()' function.
 *
 *	This is disabled by default.
 */
static bool sdecustom = true;
module_param(sdecustom, bool, 0400);
MODULE_PARM_DESC(sdecustom, "Enable customizations for sde clients");

static int sde_kms_hw_init(struct msm_kms *kms);
static int _sde_kms_mmu_destroy(struct sde_kms *sde_kms);
static int _sde_kms_register_events(struct msm_kms *kms,
		struct drm_mode_object *obj, u32 event, bool en);
bool sde_is_custom_client(void)
{
	return sdecustom;
}

#ifdef CONFIG_DEBUG_FS
static int _sde_danger_signal_status(struct seq_file *s,
		bool danger_status)
{
	struct sde_kms *kms = (struct sde_kms *)s->private;
	struct msm_drm_private *priv;
	struct sde_danger_safe_status status;
	int i;

	if (!kms || !kms->dev || !kms->dev->dev_private || !kms->hw_mdp) {
		SDE_ERROR("invalid arg(s)\n");
		return 0;
	}

	priv = kms->dev->dev_private;
	memset(&status, 0, sizeof(struct sde_danger_safe_status));

	sde_power_resource_enable(&priv->phandle, kms->core_client, true);
	if (danger_status) {
		seq_puts(s, "\nDanger signal status:\n");
		if (kms->hw_mdp->ops.get_danger_status)
			kms->hw_mdp->ops.get_danger_status(kms->hw_mdp,
					&status);
	} else {
		seq_puts(s, "\nSafe signal status:\n");
		if (kms->hw_mdp->ops.get_danger_status)
			kms->hw_mdp->ops.get_danger_status(kms->hw_mdp,
					&status);
	}
	sde_power_resource_enable(&priv->phandle, kms->core_client, false);

	seq_printf(s, "MDP     :  0x%x\n", status.mdp);

	for (i = SSPP_VIG0; i < SSPP_MAX; i++)
		seq_printf(s, "SSPP%d   :  0x%x  \t", i - SSPP_VIG0,
				status.sspp[i]);
	seq_puts(s, "\n");

	for (i = WB_0; i < WB_MAX; i++)
		seq_printf(s, "WB%d     :  0x%x  \t", i - WB_0,
				status.wb[i]);
	seq_puts(s, "\n");

	return 0;
}

#define DEFINE_SDE_DEBUGFS_SEQ_FOPS(__prefix)				\
static int __prefix ## _open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, __prefix ## _show, inode->i_private);	\
}									\
static const struct file_operations __prefix ## _fops = {		\
	.owner = THIS_MODULE,						\
	.open = __prefix ## _open,					\
	.release = single_release,					\
	.read = seq_read,						\
	.llseek = seq_lseek,						\
}

static int sde_debugfs_danger_stats_show(struct seq_file *s, void *v)
{
	return _sde_danger_signal_status(s, true);
}
DEFINE_SDE_DEBUGFS_SEQ_FOPS(sde_debugfs_danger_stats);

static int sde_debugfs_safe_stats_show(struct seq_file *s, void *v)
{
	return _sde_danger_signal_status(s, false);
}
DEFINE_SDE_DEBUGFS_SEQ_FOPS(sde_debugfs_safe_stats);

static void sde_debugfs_danger_destroy(struct sde_kms *sde_kms)
{
	debugfs_remove_recursive(sde_kms->debugfs_danger);
	sde_kms->debugfs_danger = NULL;
}

static int sde_debugfs_danger_init(struct sde_kms *sde_kms,
		struct dentry *parent)
{
	sde_kms->debugfs_danger = debugfs_create_dir("danger",
			parent);
	if (!sde_kms->debugfs_danger) {
		SDE_ERROR("failed to create danger debugfs\n");
		return -EINVAL;
	}

	debugfs_create_file("danger_status", 0600, sde_kms->debugfs_danger,
			sde_kms, &sde_debugfs_danger_stats_fops);
	debugfs_create_file("safe_status", 0600, sde_kms->debugfs_danger,
			sde_kms, &sde_debugfs_safe_stats_fops);

	return 0;
}

static int _sde_debugfs_show_regset32(struct seq_file *s, void *data)
{
	struct sde_debugfs_regset32 *regset;
	struct sde_kms *sde_kms;
	struct drm_device *dev;
	struct msm_drm_private *priv;
	void __iomem *base;
	uint32_t i, addr;

	if (!s || !s->private)
		return 0;

	regset = s->private;

	sde_kms = regset->sde_kms;
	if (!sde_kms || !sde_kms->mmio)
		return 0;

	dev = sde_kms->dev;
	if (!dev)
		return 0;

	priv = dev->dev_private;
	if (!priv)
		return 0;

	base = sde_kms->mmio + regset->offset;

	/* insert padding spaces, if needed */
	if (regset->offset & 0xF) {
		seq_printf(s, "[%x]", regset->offset & ~0xF);
		for (i = 0; i < (regset->offset & 0xF); i += 4)
			seq_puts(s, "         ");
	}

	if (sde_power_resource_enable(&priv->phandle,
				sde_kms->core_client, true)) {
		seq_puts(s, "failed to enable sde clocks\n");
		return 0;
	}

	/* main register output */
	for (i = 0; i < regset->blk_len; i += 4) {
		addr = regset->offset + i;
		if ((addr & 0xF) == 0x0)
			seq_printf(s, i ? "\n[%x]" : "[%x]", addr);
		seq_printf(s, " %08x", readl_relaxed(base + i));
	}
	seq_puts(s, "\n");
	sde_power_resource_enable(&priv->phandle, sde_kms->core_client, false);

	return 0;
}

static int sde_debugfs_open_regset32(struct inode *inode,
		struct file *file)
{
	return single_open(file, _sde_debugfs_show_regset32, inode->i_private);
}

static const struct file_operations sde_fops_regset32 = {
	.open =		sde_debugfs_open_regset32,
	.read =		seq_read,
	.llseek =	seq_lseek,
	.release =	single_release,
};

void sde_debugfs_setup_regset32(struct sde_debugfs_regset32 *regset,
		uint32_t offset, uint32_t length, struct sde_kms *sde_kms)
{
	if (regset) {
		regset->offset = offset;
		regset->blk_len = length;
		regset->sde_kms = sde_kms;
	}
}

void *sde_debugfs_create_regset32(const char *name, umode_t mode,
		void *parent, struct sde_debugfs_regset32 *regset)
{
	if (!name || !regset || !regset->sde_kms || !regset->blk_len)
		return NULL;

	/* make sure offset is a multiple of 4 */
	regset->offset = round_down(regset->offset, 4);

	return debugfs_create_file(name, mode, parent,
			regset, &sde_fops_regset32);
}

void *sde_debugfs_get_root(struct sde_kms *sde_kms)
{
	struct msm_drm_private *priv;

	if (!sde_kms || !sde_kms->dev || !sde_kms->dev->dev_private)
		return NULL;

	priv = sde_kms->dev->dev_private;
	return priv->debug_root;
}

static int _sde_debugfs_init(struct sde_kms *sde_kms)
{
	void *p;
	int rc;
	void *debugfs_root;

	p = sde_hw_util_get_log_mask_ptr();

	if (!sde_kms || !p)
		return -EINVAL;

	debugfs_root = sde_debugfs_get_root(sde_kms);
	if (!debugfs_root)
		return -EINVAL;

	/* allow debugfs_root to be NULL */
	debugfs_create_x32(SDE_DEBUGFS_HWMASKNAME, 0600, debugfs_root, p);

	(void) sde_debugfs_danger_init(sde_kms, debugfs_root);
	(void) sde_debugfs_vbif_init(sde_kms, debugfs_root);
	(void) sde_debugfs_core_irq_init(sde_kms, debugfs_root);

	rc = sde_core_perf_debugfs_init(&sde_kms->perf, debugfs_root);
	if (rc) {
		SDE_ERROR("failed to init perf %d\n", rc);
		return rc;
	}

	return 0;
}

static void _sde_debugfs_destroy(struct sde_kms *sde_kms)
{
	/* don't need to NULL check debugfs_root */
	if (sde_kms) {
		sde_debugfs_vbif_destroy(sde_kms);
		sde_debugfs_danger_destroy(sde_kms);
		sde_debugfs_core_irq_destroy(sde_kms);
	}
}
#else
static int _sde_debugfs_init(struct sde_kms *sde_kms)
{
	return 0;
}

static void _sde_debugfs_destroy(struct sde_kms *sde_kms)
{
}
#endif

static int sde_kms_enable_vblank(struct msm_kms *kms, struct drm_crtc *crtc)
{
	return sde_crtc_vblank(crtc, true);
}

static void sde_kms_disable_vblank(struct msm_kms *kms, struct drm_crtc *crtc)
{
	sde_crtc_vblank(crtc, false);
}

static void sde_kms_prepare_commit(struct msm_kms *kms,
		struct drm_atomic_state *state)
{
	struct sde_kms *sde_kms;
	struct msm_drm_private *priv;
	struct drm_device *dev;
	struct drm_encoder *encoder;

	if (!kms)
		return;
	sde_kms = to_sde_kms(kms);
	dev = sde_kms->dev;

	if (!dev || !dev->dev_private)
		return;
	priv = dev->dev_private;

	sde_power_resource_enable(&priv->phandle, sde_kms->core_client, true);

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head)
		if (encoder->crtc != NULL)
			sde_encoder_prepare_commit(encoder);

}

static void sde_kms_commit(struct msm_kms *kms,
		struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	int i;

	for_each_crtc_in_state(old_state, crtc, old_crtc_state, i) {
		if (crtc->state->active) {
			SDE_EVT32(DRMID(crtc));
			sde_crtc_commit_kickoff(crtc);
		}
	}
}

static void sde_kms_complete_commit(struct msm_kms *kms,
		struct drm_atomic_state *old_state)
{
	struct sde_kms *sde_kms;
	struct msm_drm_private *priv;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	int i;

	if (!kms || !old_state)
		return;
	sde_kms = to_sde_kms(kms);

	if (!sde_kms->dev || !sde_kms->dev->dev_private)
		return;
	priv = sde_kms->dev->dev_private;

	for_each_crtc_in_state(old_state, crtc, old_crtc_state, i)
		sde_crtc_complete_commit(crtc, old_crtc_state);
	sde_power_resource_enable(&priv->phandle, sde_kms->core_client, false);

	SDE_EVT32(SDE_EVTLOG_FUNC_EXIT);
}

static void sde_kms_wait_for_tx_complete(struct msm_kms *kms,
		struct drm_crtc *crtc)
{
	struct drm_encoder *encoder;
	struct drm_device *dev;
	int ret;

	if (!kms || !crtc || !crtc->state || !crtc->dev) {
		SDE_ERROR("invalid params\n");
		return;
	}

	if (!crtc->state->enable) {
		SDE_DEBUG("[crtc:%d] not enable\n", crtc->base.id);
		return;
	}

	if (!crtc->state->active) {
		SDE_DEBUG("[crtc:%d] not active\n", crtc->base.id);
		return;
	}

	dev = crtc->dev;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (encoder->crtc != crtc)
			continue;
		/*
		 * Video Mode - Wait for VSYNC
		 * Cmd Mode   - Wait for PP_DONE. Will be no-op if transfer is
		 *              complete
		 */
		SDE_EVT32_VERBOSE(DRMID(crtc));
		ret = sde_encoder_wait_for_event(encoder, MSM_ENC_TX_COMPLETE);
		if (ret && ret != -EWOULDBLOCK) {
			SDE_ERROR(
			"[crtc: %d][enc: %d] wait for commit done returned %d\n",
			crtc->base.id, encoder->base.id, ret);
			break;
		}
	}
}

static void sde_kms_wait_for_commit_done(struct msm_kms *kms,
		struct drm_crtc *crtc)
{
	struct drm_encoder *encoder;
	struct drm_device *dev = crtc->dev;
	int ret;

	if (!kms || !crtc || !crtc->state) {
		SDE_ERROR("invalid params\n");
		return;
	}

	if (!crtc->state->enable) {
		SDE_DEBUG("[crtc:%d] not enable\n", crtc->base.id);
		return;
	}

	if (!crtc->state->active) {
		SDE_DEBUG("[crtc:%d] not active\n", crtc->base.id);
		return;
	}

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (encoder->crtc != crtc)
			continue;
		/*
		 * Wait for post-flush if necessary to delay before
		 * plane_cleanup. For example, wait for vsync in case of video
		 * mode panels. This may be a no-op for command mode panels.
		 */
		SDE_EVT32_VERBOSE(DRMID(crtc));
		ret = sde_encoder_wait_for_event(encoder, MSM_ENC_COMMIT_DONE);
		if (ret && ret != -EWOULDBLOCK) {
			SDE_ERROR("wait for commit done returned %d\n", ret);
			break;
		}
	}
}

static void sde_kms_prepare_fence(struct msm_kms *kms,
		struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	int i, rc;

	if (!kms || !old_state || !old_state->dev || !old_state->acquire_ctx) {
		SDE_ERROR("invalid argument(s)\n");
		return;
	}

retry:
	/* attempt to acquire ww mutex for connection */
	rc = drm_modeset_lock(&old_state->dev->mode_config.connection_mutex,
			       old_state->acquire_ctx);

	if (rc == -EDEADLK) {
		drm_modeset_backoff(old_state->acquire_ctx);
		goto retry;
	}

	/* old_state actually contains updated crtc pointers */
	for_each_crtc_in_state(old_state, crtc, old_crtc_state, i)
		sde_crtc_prepare_commit(crtc, old_crtc_state);
}

/**
 * _sde_kms_get_displays - query for underlying display handles and cache them
 * @sde_kms:    Pointer to sde kms structure
 * Returns:     Zero on success
 */
static int _sde_kms_get_displays(struct sde_kms *sde_kms)
{
	int rc = -ENOMEM;

	if (!sde_kms) {
		SDE_ERROR("invalid sde kms\n");
		return -EINVAL;
	}

	/* dsi */
	sde_kms->dsi_displays = NULL;
	sde_kms->dsi_display_count = dsi_display_get_num_of_displays();
	if (sde_kms->dsi_display_count) {
		sde_kms->dsi_displays = kcalloc(sde_kms->dsi_display_count,
				sizeof(void *),
				GFP_KERNEL);
		if (!sde_kms->dsi_displays) {
			SDE_ERROR("failed to allocate dsi displays\n");
			goto exit_deinit_dsi;
		}
		sde_kms->dsi_display_count =
			dsi_display_get_active_displays(sde_kms->dsi_displays,
					sde_kms->dsi_display_count);
	}

	/* wb */
	sde_kms->wb_displays = NULL;
	sde_kms->wb_display_count = sde_wb_get_num_of_displays();
	if (sde_kms->wb_display_count) {
		sde_kms->wb_displays = kcalloc(sde_kms->wb_display_count,
				sizeof(void *),
				GFP_KERNEL);
		if (!sde_kms->wb_displays) {
			SDE_ERROR("failed to allocate wb displays\n");
			goto exit_deinit_wb;
		}
		sde_kms->wb_display_count =
			wb_display_get_displays(sde_kms->wb_displays,
					sde_kms->wb_display_count);
	}

	/* dp */
	sde_kms->dp_displays = NULL;
	sde_kms->dp_display_count = dp_display_get_num_of_displays();
	if (sde_kms->dp_display_count) {
		sde_kms->dp_displays = kcalloc(sde_kms->dp_display_count,
				sizeof(void *), GFP_KERNEL);
		if (!sde_kms->dp_displays) {
			SDE_ERROR("failed to allocate dp displays\n");
			goto exit_deinit_dp;
		}
		sde_kms->dp_display_count =
			dp_display_get_displays(sde_kms->dp_displays,
					sde_kms->dp_display_count);
	}
	return 0;

exit_deinit_dp:
	kfree(sde_kms->dp_displays);
	sde_kms->dp_display_count = 0;
	sde_kms->dp_displays = NULL;

exit_deinit_wb:
	kfree(sde_kms->wb_displays);
	sde_kms->wb_display_count = 0;
	sde_kms->wb_displays = NULL;

exit_deinit_dsi:
	kfree(sde_kms->dsi_displays);
	sde_kms->dsi_display_count = 0;
	sde_kms->dsi_displays = NULL;
	return rc;
}

/**
 * _sde_kms_release_displays - release cache of underlying display handles
 * @sde_kms:    Pointer to sde kms structure
 */
static void _sde_kms_release_displays(struct sde_kms *sde_kms)
{
	if (!sde_kms) {
		SDE_ERROR("invalid sde kms\n");
		return;
	}

	kfree(sde_kms->wb_displays);
	sde_kms->wb_displays = NULL;
	sde_kms->wb_display_count = 0;

	kfree(sde_kms->dsi_displays);
	sde_kms->dsi_displays = NULL;
	sde_kms->dsi_display_count = 0;
}

/**
 * _sde_kms_setup_displays - create encoders, bridges and connectors
 *                           for underlying displays
 * @dev:        Pointer to drm device structure
 * @priv:       Pointer to private drm device data
 * @sde_kms:    Pointer to sde kms structure
 * Returns:     Zero on success
 */
static int _sde_kms_setup_displays(struct drm_device *dev,
		struct msm_drm_private *priv,
		struct sde_kms *sde_kms)
{
	static const struct sde_connector_ops dsi_ops = {
		.post_init =  dsi_conn_post_init,
		.detect =     dsi_conn_detect,
		.get_modes =  dsi_connector_get_modes,
		.mode_valid = dsi_conn_mode_valid,
		.get_info =   dsi_display_get_info,
		.set_backlight = dsi_display_set_backlight,
		.soft_reset   = dsi_display_soft_reset,
		.pre_kickoff  = dsi_conn_pre_kickoff,
		.clk_ctrl = dsi_display_clk_ctrl,
		.get_topology = dsi_conn_get_topology
	};
	static const struct sde_connector_ops wb_ops = {
		.post_init =    sde_wb_connector_post_init,
		.detect =       sde_wb_connector_detect,
		.get_modes =    sde_wb_connector_get_modes,
		.set_property = sde_wb_connector_set_property,
		.get_info =     sde_wb_get_info,
		.soft_reset =   NULL,
		.get_topology = sde_wb_get_topology
	};
	static const struct sde_connector_ops dp_ops = {
		.post_init  = dp_connector_post_init,
		.detect     = dp_connector_detect,
		.get_modes  = dp_connector_get_modes,
		.mode_valid = dp_connector_mode_valid,
		.get_info   = dp_connector_get_info,
		.get_topology   = dp_connector_get_topology,
	};
	struct msm_display_info info;
	struct drm_encoder *encoder;
	void *display, *connector;
	int i, max_encoders;
	int rc = 0;

	if (!dev || !priv || !sde_kms) {
		SDE_ERROR("invalid argument(s)\n");
		return -EINVAL;
	}

	max_encoders = sde_kms->dsi_display_count + sde_kms->wb_display_count +
				sde_kms->dp_display_count;
	if (max_encoders > ARRAY_SIZE(priv->encoders)) {
		max_encoders = ARRAY_SIZE(priv->encoders);
		SDE_ERROR("capping number of displays to %d", max_encoders);
	}

	/* dsi */
	for (i = 0; i < sde_kms->dsi_display_count &&
		priv->num_encoders < max_encoders; ++i) {
		display = sde_kms->dsi_displays[i];
		encoder = NULL;

		memset(&info, 0x0, sizeof(info));
		rc = dsi_display_get_info(&info, display);
		if (rc) {
			SDE_ERROR("dsi get_info %d failed\n", i);
			continue;
		}

		encoder = sde_encoder_init(dev, &info);
		if (IS_ERR_OR_NULL(encoder)) {
			SDE_ERROR("encoder init failed for dsi %d\n", i);
			continue;
		}

		rc = dsi_display_drm_bridge_init(display, encoder);
		if (rc) {
			SDE_ERROR("dsi bridge %d init failed, %d\n", i, rc);
			sde_encoder_destroy(encoder);
			continue;
		}

		connector = sde_connector_init(dev,
					encoder,
					0,
					display,
					&dsi_ops,
					DRM_CONNECTOR_POLL_HPD,
					DRM_MODE_CONNECTOR_DSI);
		if (connector) {
			priv->encoders[priv->num_encoders++] = encoder;
		} else {
			SDE_ERROR("dsi %d connector init failed\n", i);
			dsi_display_drm_bridge_deinit(display);
			sde_encoder_destroy(encoder);
		}
	}

	/* wb */
	for (i = 0; i < sde_kms->wb_display_count &&
		priv->num_encoders < max_encoders; ++i) {
		display = sde_kms->wb_displays[i];
		encoder = NULL;

		memset(&info, 0x0, sizeof(info));
		rc = sde_wb_get_info(&info, display);
		if (rc) {
			SDE_ERROR("wb get_info %d failed\n", i);
			continue;
		}

		encoder = sde_encoder_init(dev, &info);
		if (IS_ERR_OR_NULL(encoder)) {
			SDE_ERROR("encoder init failed for wb %d\n", i);
			continue;
		}

		rc = sde_wb_drm_init(display, encoder);
		if (rc) {
			SDE_ERROR("wb bridge %d init failed, %d\n", i, rc);
			sde_encoder_destroy(encoder);
			continue;
		}

		connector = sde_connector_init(dev,
				encoder,
				0,
				display,
				&wb_ops,
				DRM_CONNECTOR_POLL_HPD,
				DRM_MODE_CONNECTOR_VIRTUAL);
		if (connector) {
			priv->encoders[priv->num_encoders++] = encoder;
		} else {
			SDE_ERROR("wb %d connector init failed\n", i);
			sde_wb_drm_deinit(display);
			sde_encoder_destroy(encoder);
		}
	}
	/* dp */
	for (i = 0; i < sde_kms->dp_display_count &&
			priv->num_encoders < max_encoders; ++i) {
		display = sde_kms->dp_displays[i];
		encoder = NULL;

		memset(&info, 0x0, sizeof(info));
		rc = dp_connector_get_info(&info, display);
		if (rc) {
			SDE_ERROR("dp get_info %d failed\n", i);
			continue;
		}

		encoder = sde_encoder_init(dev, &info);
		if (IS_ERR_OR_NULL(encoder)) {
			SDE_ERROR("dp encoder init failed %d\n", i);
			continue;
		}

		rc = dp_drm_bridge_init(display, encoder);
		if (rc) {
			SDE_ERROR("dp bridge %d init failed, %d\n", i, rc);
			sde_encoder_destroy(encoder);
			continue;
		}

		connector = sde_connector_init(dev,
					encoder,
					NULL,
					display,
					&dp_ops,
					DRM_CONNECTOR_POLL_HPD,
					DRM_MODE_CONNECTOR_DisplayPort);
		if (connector) {
			priv->encoders[priv->num_encoders++] = encoder;
		} else {
			SDE_ERROR("dp %d connector init failed\n", i);
			dp_drm_bridge_deinit(display);
			sde_encoder_destroy(encoder);
		}
	}

	return 0;
}

static void _sde_kms_drm_obj_destroy(struct sde_kms *sde_kms)
{
	struct msm_drm_private *priv;
	int i;

	if (!sde_kms) {
		SDE_ERROR("invalid sde_kms\n");
		return;
	} else if (!sde_kms->dev) {
		SDE_ERROR("invalid dev\n");
		return;
	} else if (!sde_kms->dev->dev_private) {
		SDE_ERROR("invalid dev_private\n");
		return;
	}
	priv = sde_kms->dev->dev_private;

	for (i = 0; i < priv->num_crtcs; i++)
		priv->crtcs[i]->funcs->destroy(priv->crtcs[i]);
	priv->num_crtcs = 0;

	for (i = 0; i < priv->num_planes; i++)
		priv->planes[i]->funcs->destroy(priv->planes[i]);
	priv->num_planes = 0;

	for (i = 0; i < priv->num_connectors; i++)
		priv->connectors[i]->funcs->destroy(priv->connectors[i]);
	priv->num_connectors = 0;

	for (i = 0; i < priv->num_encoders; i++)
		priv->encoders[i]->funcs->destroy(priv->encoders[i]);
	priv->num_encoders = 0;

	_sde_kms_release_displays(sde_kms);
}

static int _sde_kms_drm_obj_init(struct sde_kms *sde_kms)
{
	struct drm_device *dev;
	struct drm_plane *primary_planes[MAX_PLANES], *plane;
	struct drm_crtc *crtc;

	struct msm_drm_private *priv;
	struct sde_mdss_cfg *catalog;

	int primary_planes_idx = 0, i, ret;
	int max_crtc_count;

	u32 sspp_id[MAX_PLANES];
	u32 master_plane_id[MAX_PLANES];
	u32 num_virt_planes = 0;

	if (!sde_kms || !sde_kms->dev || !sde_kms->dev->dev) {
		SDE_ERROR("invalid sde_kms\n");
		return -EINVAL;
	}

	dev = sde_kms->dev;
	priv = dev->dev_private;
	catalog = sde_kms->catalog;

	ret = sde_core_irq_domain_add(sde_kms);
	if (ret)
		goto fail_irq;
	/*
	 * Query for underlying display drivers, and create connectors,
	 * bridges and encoders for them.
	 */
	if (!_sde_kms_get_displays(sde_kms))
		(void)_sde_kms_setup_displays(dev, priv, sde_kms);

	max_crtc_count = min(catalog->mixer_count, priv->num_encoders);

	/* Create the planes */
	for (i = 0; i < catalog->sspp_count; i++) {
		bool primary = true;

		if (catalog->sspp[i].features & BIT(SDE_SSPP_CURSOR)
			|| primary_planes_idx >= max_crtc_count)
			primary = false;

		plane = sde_plane_init(dev, catalog->sspp[i].id, primary,
				(1UL << max_crtc_count) - 1, 0);
		if (IS_ERR(plane)) {
			SDE_ERROR("sde_plane_init failed\n");
			ret = PTR_ERR(plane);
			goto fail;
		}
		priv->planes[priv->num_planes++] = plane;

		if (primary)
			primary_planes[primary_planes_idx++] = plane;

		if (sde_hw_sspp_multirect_enabled(&catalog->sspp[i]) &&
			sde_is_custom_client()) {
			int priority =
				catalog->sspp[i].sblk->smart_dma_priority;
			sspp_id[priority - 1] = catalog->sspp[i].id;
			master_plane_id[priority - 1] = plane->base.id;
			num_virt_planes++;
		}
	}

	/* Initialize smart DMA virtual planes */
	for (i = 0; i < num_virt_planes; i++) {
		plane = sde_plane_init(dev, sspp_id[i], false,
			(1UL << max_crtc_count) - 1, master_plane_id[i]);
		if (IS_ERR(plane)) {
			SDE_ERROR("sde_plane for virtual SSPP init failed\n");
			ret = PTR_ERR(plane);
			goto fail;
		}
		priv->planes[priv->num_planes++] = plane;
	}

	max_crtc_count = min(max_crtc_count, primary_planes_idx);

	/* Create one CRTC per encoder */
	for (i = 0; i < max_crtc_count; i++) {
		crtc = sde_crtc_init(dev, primary_planes[i]);
		if (IS_ERR(crtc)) {
			ret = PTR_ERR(crtc);
			goto fail;
		}
		priv->crtcs[priv->num_crtcs++] = crtc;
	}

	if (sde_is_custom_client()) {
		/* All CRTCs are compatible with all planes */
		for (i = 0; i < priv->num_planes; i++)
			priv->planes[i]->possible_crtcs =
				(1 << priv->num_crtcs) - 1;
	}

	/* All CRTCs are compatible with all encoders */
	for (i = 0; i < priv->num_encoders; i++)
		priv->encoders[i]->possible_crtcs = (1 << priv->num_crtcs) - 1;

	return 0;
fail:
	_sde_kms_drm_obj_destroy(sde_kms);
fail_irq:
	sde_core_irq_domain_fini(sde_kms);
	return ret;
}

/**
 * struct sde_kms_fbo_fb - framebuffer creation list
 * @list: list of framebuffer attached to framebuffer object
 * @fb: Pointer to framebuffer attached to framebuffer object
 */
struct sde_kms_fbo_fb {
	struct list_head list;
	struct drm_framebuffer *fb;
};

struct drm_framebuffer *sde_kms_fbo_create_fb(struct drm_device *dev,
		struct sde_kms_fbo *fbo)
{
	struct drm_framebuffer *fb = NULL;
	struct sde_kms_fbo_fb *fbo_fb;
	struct drm_mode_fb_cmd2 mode_cmd = {0};
	u32 base_offset = 0;
	int i, ret;

	if (!dev) {
		SDE_ERROR("invalid drm device node\n");
		return NULL;
	}

	fbo_fb = kzalloc(sizeof(struct sde_kms_fbo_fb), GFP_KERNEL);
	if (!fbo_fb)
		return NULL;

	mode_cmd.pixel_format = fbo->pixel_format;
	mode_cmd.width = fbo->width;
	mode_cmd.height = fbo->height;
	mode_cmd.flags = fbo->flags;

	for (i = 0; i < fbo->nplane; i++) {
		mode_cmd.offsets[i] = base_offset;
		mode_cmd.pitches[i] = fbo->layout.plane_pitch[i];
		mode_cmd.modifier[i] = fbo->modifier[i];
		base_offset += fbo->layout.plane_size[i];
		SDE_DEBUG("offset[%d]:%x\n", i, mode_cmd.offsets[i]);
	}

	fb = msm_framebuffer_init(dev, &mode_cmd, fbo->bo);
	if (IS_ERR(fb)) {
		ret = PTR_ERR(fb);
		fb = NULL;
		SDE_ERROR("failed to allocate fb %d\n", ret);
		goto fail;
	}

	/* need to take one reference for gem object */
	for (i = 0; i < fbo->nplane; i++)
		drm_gem_object_reference(fbo->bo[i]);

	SDE_DEBUG("register private fb:%d\n", fb->base.id);

	INIT_LIST_HEAD(&fbo_fb->list);
	fbo_fb->fb = fb;
	drm_framebuffer_reference(fbo_fb->fb);
	list_add_tail(&fbo_fb->list, &fbo->fb_list);

	return fb;

fail:
	kfree(fbo_fb);
	return NULL;
}

static void sde_kms_fbo_destroy(struct sde_kms_fbo *fbo)
{
	struct msm_drm_private *priv;
	struct sde_kms *sde_kms;
	struct drm_device *dev;
	struct sde_kms_fbo_fb *curr, *next;
	int i;

	if (!fbo) {
		SDE_ERROR("invalid drm device node\n");
		return;
	}
	dev = fbo->dev;

	if (!dev || !dev->dev_private) {
		SDE_ERROR("invalid drm device node\n");
		return;
	}
	priv = dev->dev_private;

	if (!priv->kms) {
		SDE_ERROR("invalid kms handle\n");
		return;
	}
	sde_kms = to_sde_kms(priv->kms);

	SDE_DEBUG("%dx%d@%c%c%c%c/%llx/%x\n", fbo->width, fbo->height,
			fbo->pixel_format >> 0, fbo->pixel_format >> 8,
			fbo->pixel_format >> 16, fbo->pixel_format >> 24,
			fbo->modifier[0], fbo->flags);

	list_for_each_entry_safe(curr, next, &fbo->fb_list, list) {
		SDE_DEBUG("unregister private fb:%d\n", curr->fb->base.id);
		drm_framebuffer_unregister_private(curr->fb);
		drm_framebuffer_unreference(curr->fb);
		list_del(&curr->list);
		kfree(curr);
	}

	for (i = 0; i < fbo->layout.num_planes; i++) {
		if (fbo->bo[i]) {
			mutex_lock(&dev->struct_mutex);
			drm_gem_object_unreference(fbo->bo[i]);
			mutex_unlock(&dev->struct_mutex);
			fbo->bo[i] = NULL;
		}
	}

	if (fbo->dma_buf) {
		dma_buf_put(fbo->dma_buf);
		fbo->dma_buf = NULL;
	}

	if (sde_kms->iclient && fbo->ihandle) {
		ion_free(sde_kms->iclient, fbo->ihandle);
		fbo->ihandle = NULL;
	}
}

static void sde_kms_set_gem_flags(struct msm_gem_object *msm_obj,
		uint32_t flags)
{
	if (msm_obj)
		msm_obj->flags |= flags;
}

struct sde_kms_fbo *sde_kms_fbo_alloc(struct drm_device *dev, u32 width,
		u32 height, u32 pixel_format, u64 modifier[4], u32 flags)
{
	struct msm_drm_private *priv;
	struct sde_kms *sde_kms;
	struct sde_kms_fbo *fbo;
	int i, ret;

	if (!dev || !dev->dev_private) {
		SDE_ERROR("invalid drm device node\n");
		return NULL;
	}
	priv = dev->dev_private;

	if (!priv->kms) {
		SDE_ERROR("invalid kms handle\n");
		return NULL;
	}
	sde_kms = to_sde_kms(priv->kms);

	SDE_DEBUG("%dx%d@%c%c%c%c/%llx/%x\n", width, height,
			pixel_format >> 0, pixel_format >> 8,
			pixel_format >> 16, pixel_format >> 24,
			modifier[0], flags);

	fbo = kzalloc(sizeof(struct sde_kms_fbo), GFP_KERNEL);
	if (!fbo)
		return NULL;

	atomic_set(&fbo->refcount, 0);
	INIT_LIST_HEAD(&fbo->fb_list);
	fbo->dev = dev;
	fbo->width = width;
	fbo->height = height;
	fbo->pixel_format = pixel_format;
	fbo->flags = flags;
	for (i = 0; i < ARRAY_SIZE(fbo->modifier); i++)
		fbo->modifier[i] = modifier[i];
	fbo->nplane = drm_format_num_planes(fbo->pixel_format);
	fbo->fmt = sde_get_sde_format_ext(fbo->pixel_format, fbo->modifier,
			fbo->nplane);
	if (!fbo->fmt) {
		ret = -EINVAL;
		SDE_ERROR("failed to find pixel format\n");
		goto done;
	}

	ret = sde_format_get_plane_sizes(fbo->fmt, fbo->width, fbo->height,
			&fbo->layout);
	if (ret) {
		SDE_ERROR("failed to get plane sizes\n");
		goto done;
	}

	/* allocate backing buffer object */
	if (sde_kms->iclient) {
		u32 heap_id = fbo->flags & DRM_MODE_FB_SECURE ?
				ION_HEAP(ION_SECURE_DISPLAY_HEAP_ID) :
				ION_HEAP(ION_SYSTEM_HEAP_ID);

		fbo->ihandle = ion_alloc(sde_kms->iclient,
				fbo->layout.total_size, SZ_4K, heap_id, 0);
		if (IS_ERR_OR_NULL(fbo->ihandle)) {
			SDE_ERROR("failed to alloc ion memory\n");
			ret = PTR_ERR(fbo->ihandle);
			fbo->ihandle = NULL;
			goto done;
		}

		fbo->dma_buf = ion_share_dma_buf(sde_kms->iclient,
				fbo->ihandle);
		if (IS_ERR(fbo->dma_buf)) {
			SDE_ERROR("failed to share ion memory\n");
			ret = -ENOMEM;
			fbo->dma_buf = NULL;
			goto done;
		}

		fbo->bo[0] = dev->driver->gem_prime_import(dev,
				fbo->dma_buf);
		if (IS_ERR(fbo->bo[0])) {
			SDE_ERROR("failed to import ion memory\n");
			ret = PTR_ERR(fbo->bo[0]);
			fbo->bo[0] = NULL;
			goto done;
		}

		/* insert extra bo flags */
		sde_kms_set_gem_flags(to_msm_bo(fbo->bo[0]), MSM_BO_KEEPATTRS);
	} else {
		mutex_lock(&dev->struct_mutex);
		fbo->bo[0] = msm_gem_new(dev, fbo->layout.total_size,
				MSM_BO_SCANOUT | MSM_BO_WC | MSM_BO_KEEPATTRS);
		if (IS_ERR(fbo->bo[0])) {
			mutex_unlock(&dev->struct_mutex);
			SDE_ERROR("failed to new gem buffer\n");
			ret = PTR_ERR(fbo->bo[0]);
			fbo->bo[0] = NULL;
			goto done;
		}
		mutex_unlock(&dev->struct_mutex);
	}

	mutex_lock(&dev->struct_mutex);
	for (i = 1; i < fbo->layout.num_planes; i++) {
		fbo->bo[i] = fbo->bo[0];
		drm_gem_object_reference(fbo->bo[i]);
	}
	mutex_unlock(&dev->struct_mutex);

done:
	if (ret) {
		sde_kms_fbo_destroy(fbo);
		kfree(fbo);
		fbo = NULL;
	} else {
		sde_kms_fbo_reference(fbo);
	}

	return fbo;
}

int sde_kms_fbo_reference(struct sde_kms_fbo *fbo)
{
	if (!fbo) {
		SDE_ERROR("invalid parameters\n");
		return -EINVAL;
	}

	SDE_DEBUG("%pS refcount:%d\n", __builtin_return_address(0),
			atomic_read(&fbo->refcount));

	atomic_inc(&fbo->refcount);

	return 0;
}

void sde_kms_fbo_unreference(struct sde_kms_fbo *fbo)
{
	if (!fbo) {
		SDE_ERROR("invalid parameters\n");
		return;
	}

	SDE_DEBUG("%pS refcount:%d\n", __builtin_return_address(0),
			atomic_read(&fbo->refcount));

	if (!atomic_read(&fbo->refcount)) {
		SDE_ERROR("invalid refcount\n");
		return;
	} else if (atomic_dec_return(&fbo->refcount) == 0) {
		sde_kms_fbo_destroy(fbo);
	}
}

static int sde_kms_postinit(struct msm_kms *kms)
{
	struct sde_kms *sde_kms = to_sde_kms(kms);
	struct drm_device *dev;
	int rc;

	if (!sde_kms || !sde_kms->dev || !sde_kms->dev->dev) {
		SDE_ERROR("invalid sde_kms\n");
		return -EINVAL;
	}

	dev = sde_kms->dev;

	rc = _sde_debugfs_init(sde_kms);
	if (rc)
		SDE_ERROR("sde_debugfs init failed: %d\n", rc);

	return rc;
}

static long sde_kms_round_pixclk(struct msm_kms *kms, unsigned long rate,
		struct drm_encoder *encoder)
{
	return rate;
}

static void _sde_kms_hw_destroy(struct sde_kms *sde_kms,
		struct platform_device *pdev)
{
	struct drm_device *dev;
	struct msm_drm_private *priv;
	int i;

	if (!sde_kms || !pdev)
		return;

	dev = sde_kms->dev;
	if (!dev)
		return;

	priv = dev->dev_private;
	if (!priv)
		return;

	if (sde_kms->hw_intr)
		sde_hw_intr_destroy(sde_kms->hw_intr);
	sde_kms->hw_intr = NULL;

	if (sde_kms->power_event)
		sde_power_handle_unregister_event(
				&priv->phandle, sde_kms->power_event);

	_sde_kms_release_displays(sde_kms);

	/* safe to call these more than once during shutdown */
	_sde_debugfs_destroy(sde_kms);
	_sde_kms_mmu_destroy(sde_kms);

	if (sde_kms->iclient) {
		ion_client_destroy(sde_kms->iclient);
		sde_kms->iclient = NULL;
	}

	if (sde_kms->catalog) {
		for (i = 0; i < sde_kms->catalog->vbif_count; i++) {
			u32 vbif_idx = sde_kms->catalog->vbif[i].id;

			if ((vbif_idx < VBIF_MAX) && sde_kms->hw_vbif[vbif_idx])
				sde_hw_vbif_destroy(sde_kms->hw_vbif[vbif_idx]);
		}
	}

	if (sde_kms->rm_init)
		sde_rm_destroy(&sde_kms->rm);
	sde_kms->rm_init = false;

	if (sde_kms->catalog)
		sde_hw_catalog_deinit(sde_kms->catalog);
	sde_kms->catalog = NULL;

	if (sde_kms->core_client)
		sde_power_client_destroy(&priv->phandle, sde_kms->core_client);
	sde_kms->core_client = NULL;

	if (sde_kms->vbif[VBIF_NRT])
		msm_iounmap(pdev, sde_kms->vbif[VBIF_NRT]);
	sde_kms->vbif[VBIF_NRT] = NULL;

	if (sde_kms->vbif[VBIF_RT])
		msm_iounmap(pdev, sde_kms->vbif[VBIF_RT]);
	sde_kms->vbif[VBIF_RT] = NULL;

	if (sde_kms->mmio)
		msm_iounmap(pdev, sde_kms->mmio);
	sde_kms->mmio = NULL;

	sde_reg_dma_deinit();
}

static void sde_kms_destroy(struct msm_kms *kms)
{
	struct sde_kms *sde_kms;
	struct drm_device *dev;

	if (!kms) {
		SDE_ERROR("invalid kms\n");
		return;
	}

	sde_kms = to_sde_kms(kms);
	dev = sde_kms->dev;
	if (!dev) {
		SDE_ERROR("invalid device\n");
		return;
	}

	_sde_kms_hw_destroy(sde_kms, dev->platformdev);
	kfree(sde_kms);
}

static void sde_kms_preclose(struct msm_kms *kms, struct drm_file *file)
{
	struct sde_kms *sde_kms = to_sde_kms(kms);
	struct drm_device *dev = sde_kms->dev;
	struct msm_drm_private *priv = dev->dev_private;
	unsigned int i;

	for (i = 0; i < priv->num_crtcs; i++)
		sde_crtc_cancel_pending_flip(priv->crtcs[i], file);
}

static int sde_kms_atomic_check(struct msm_kms *kms,
		struct drm_atomic_state *state)
{
	struct sde_kms *sde_kms = to_sde_kms(kms);
	struct drm_device *dev = sde_kms->dev;
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	int rc, i;

	if (!kms || !state)
		return -EINVAL;

	/*
	 * Add planes (and other affected DRM objects, if any) to new state
	 * if idle power collapse occurred since previous commit.
	 * Since atomic state is a delta from the last, if the user-space
	 * did not request any changes on a plane/connector, that object
	 * will not be included in the new atomic state. Idle power collapse
	 * is driver-autonomous, so the driver needs to ensure that all
	 * hardware is reprogrammed as the power comes back on by forcing
	 * the drm objects attached to the CRTC into the new atomic state.
	 */
	for_each_crtc_in_state(state, crtc, crtc_state, i) {
		struct sde_crtc_state *cstate = to_sde_crtc_state(crtc_state);
		struct sde_crtc_state *old_cstate =
				to_sde_crtc_state(crtc->state);

		if (cstate->idle_pc != old_cstate->idle_pc) {
			SDE_DEBUG("crtc%d idle_pc:%d/%d\n",
					crtc->base.id, cstate->idle_pc,
					old_cstate->idle_pc);
			SDE_EVT32(DRMID(crtc), cstate->idle_pc,
					old_cstate->idle_pc);
			rc = drm_atomic_add_affected_planes(state, crtc);
			if (rc)
				return rc;
		}
	}

	return drm_atomic_helper_check(dev, state);
}

static struct msm_gem_address_space*
_sde_kms_get_address_space(struct msm_kms *kms,
		unsigned int domain)
{
	struct sde_kms *sde_kms;

	if (!kms) {
		SDE_ERROR("invalid kms\n");
		return  NULL;
	}

	sde_kms = to_sde_kms(kms);
	if (!sde_kms) {
		SDE_ERROR("invalid sde_kms\n");
		return NULL;
	}

	if (domain >= MSM_SMMU_DOMAIN_MAX)
		return NULL;

	return sde_kms->aspace[domain];
}

static const struct msm_kms_funcs kms_funcs = {
	.hw_init         = sde_kms_hw_init,
	.postinit        = sde_kms_postinit,
	.irq_preinstall  = sde_irq_preinstall,
	.irq_postinstall = sde_irq_postinstall,
	.irq_uninstall   = sde_irq_uninstall,
	.irq             = sde_irq,
	.preclose        = sde_kms_preclose,
	.prepare_fence   = sde_kms_prepare_fence,
	.prepare_commit  = sde_kms_prepare_commit,
	.commit          = sde_kms_commit,
	.complete_commit = sde_kms_complete_commit,
	.wait_for_crtc_commit_done = sde_kms_wait_for_commit_done,
	.wait_for_tx_complete = sde_kms_wait_for_tx_complete,
	.enable_vblank   = sde_kms_enable_vblank,
	.disable_vblank  = sde_kms_disable_vblank,
	.check_modified_format = sde_format_check_modified_format,
	.atomic_check    = sde_kms_atomic_check,
	.get_format      = sde_get_msm_format,
	.round_pixclk    = sde_kms_round_pixclk,
	.destroy         = sde_kms_destroy,
	.register_events = _sde_kms_register_events,
	.get_address_space = _sde_kms_get_address_space,
};

/* the caller api needs to turn on clock before calling it */
static inline void _sde_kms_core_hw_rev_init(struct sde_kms *sde_kms)
{
	sde_kms->core_rev = readl_relaxed(sde_kms->mmio + 0x0);
}

static int _sde_kms_mmu_destroy(struct sde_kms *sde_kms)
{
	struct msm_mmu *mmu;
	int i;

	for (i = ARRAY_SIZE(sde_kms->aspace) - 1; i >= 0; i--) {
		if (!sde_kms->aspace[i])
			continue;

		mmu = sde_kms->aspace[i]->mmu;

		mmu->funcs->detach(mmu, (const char **)iommu_ports,
				ARRAY_SIZE(iommu_ports));
		msm_gem_address_space_destroy(sde_kms->aspace[i]);

		sde_kms->aspace[i] = NULL;
	}

	return 0;
}

static int _sde_kms_mmu_init(struct sde_kms *sde_kms)
{
	struct msm_mmu *mmu;
	int i, ret;

	for (i = 0; i < MSM_SMMU_DOMAIN_MAX; i++) {
		struct msm_gem_address_space *aspace;

		mmu = msm_smmu_new(sde_kms->dev->dev, i);
		if (IS_ERR(mmu)) {
			ret = PTR_ERR(mmu);
			SDE_DEBUG("failed to init iommu id %d: rc:%d\n",
								i, ret);
			continue;
		}

		aspace = msm_gem_smmu_address_space_create(sde_kms->dev->dev,
			mmu, "sde");
		if (IS_ERR(aspace)) {
			ret = PTR_ERR(aspace);
			mmu->funcs->destroy(mmu);
			goto fail;
		}

		sde_kms->aspace[i] = aspace;

		ret = mmu->funcs->attach(mmu, (const char **)iommu_ports,
				ARRAY_SIZE(iommu_ports));
		if (ret) {
			SDE_ERROR("failed to attach iommu %d: %d\n", i, ret);
			msm_gem_address_space_destroy(aspace);
			goto fail;
		}

	}

	return 0;
fail:
	_sde_kms_mmu_destroy(sde_kms);

	return ret;
}

static void sde_kms_handle_power_event(u32 event_type, void *usr)
{
	struct sde_kms *sde_kms = usr;

	if (!sde_kms)
		return;

	if (event_type == SDE_POWER_EVENT_POST_ENABLE)
		sde_vbif_init_memtypes(sde_kms);
}

static int sde_kms_hw_init(struct msm_kms *kms)
{
	struct sde_kms *sde_kms;
	struct drm_device *dev;
	struct msm_drm_private *priv;
	int i, rc = -EINVAL;

	if (!kms) {
		SDE_ERROR("invalid kms\n");
		goto end;
	}

	sde_kms = to_sde_kms(kms);
	dev = sde_kms->dev;
	if (!dev || !dev->platformdev) {
		SDE_ERROR("invalid device\n");
		goto end;
	}

	priv = dev->dev_private;
	if (!priv) {
		SDE_ERROR("invalid private data\n");
		goto end;
	}

	sde_kms->mmio = msm_ioremap(dev->platformdev, "mdp_phys", "mdp_phys");
	if (IS_ERR(sde_kms->mmio)) {
		rc = PTR_ERR(sde_kms->mmio);
		SDE_ERROR("mdp register memory map failed: %d\n", rc);
		sde_kms->mmio = NULL;
		goto error;
	}
	DRM_INFO("mapped mdp address space @%p\n", sde_kms->mmio);
	sde_kms->mmio_len = msm_iomap_size(dev->platformdev, "mdp_phys");

	rc = sde_dbg_reg_register_base(SDE_DBG_NAME, sde_kms->mmio,
			sde_kms->mmio_len);
	if (rc)
		SDE_ERROR("dbg base register kms failed: %d\n", rc);

	sde_kms->vbif[VBIF_RT] = msm_ioremap(dev->platformdev, "vbif_phys",
								"vbif_phys");
	if (IS_ERR(sde_kms->vbif[VBIF_RT])) {
		rc = PTR_ERR(sde_kms->vbif[VBIF_RT]);
		SDE_ERROR("vbif register memory map failed: %d\n", rc);
		sde_kms->vbif[VBIF_RT] = NULL;
		goto error;
	}
	sde_kms->vbif_len[VBIF_RT] = msm_iomap_size(dev->platformdev,
								"vbif_phys");
	rc = sde_dbg_reg_register_base("vbif_rt", sde_kms->vbif[VBIF_RT],
				sde_kms->vbif_len[VBIF_RT]);
	if (rc)
		SDE_ERROR("dbg base register vbif_rt failed: %d\n", rc);

	sde_kms->vbif[VBIF_NRT] = msm_ioremap(dev->platformdev, "vbif_nrt_phys",
								"vbif_nrt_phys");
	if (IS_ERR(sde_kms->vbif[VBIF_NRT])) {
		sde_kms->vbif[VBIF_NRT] = NULL;
		SDE_DEBUG("VBIF NRT is not defined");
	} else {
		sde_kms->vbif_len[VBIF_NRT] = msm_iomap_size(dev->platformdev,
							"vbif_nrt_phys");
		rc = sde_dbg_reg_register_base("vbif_nrt",
				sde_kms->vbif[VBIF_NRT],
				sde_kms->vbif_len[VBIF_NRT]);
		if (rc)
			SDE_ERROR("dbg base register vbif_nrt failed: %d\n",
					rc);
	}

	sde_kms->reg_dma = msm_ioremap(dev->platformdev, "regdma_phys",
								"regdma_phys");
	if (IS_ERR(sde_kms->reg_dma)) {
		sde_kms->reg_dma = NULL;
		SDE_DEBUG("REG_DMA is not defined");
	} else {
		sde_kms->reg_dma_len = msm_iomap_size(dev->platformdev,
								"regdma_phys");
		rc =  sde_dbg_reg_register_base("vbif_nrt",
				sde_kms->reg_dma,
				sde_kms->reg_dma_len);
		if (rc)
			SDE_ERROR("dbg base register reg_dma failed: %d\n",
					rc);
	}

	sde_kms->core_client = sde_power_client_create(&priv->phandle, "core");
	if (IS_ERR_OR_NULL(sde_kms->core_client)) {
		rc = PTR_ERR(sde_kms->core_client);
		if (!sde_kms->core_client)
			rc = -EINVAL;
		SDE_ERROR("sde power client create failed: %d\n", rc);
		sde_kms->core_client = NULL;
		goto error;
	}

	rc = sde_power_resource_enable(&priv->phandle, sde_kms->core_client,
		true);
	if (rc) {
		SDE_ERROR("resource enable failed: %d\n", rc);
		goto error;
	}

	_sde_kms_core_hw_rev_init(sde_kms);

	pr_info("sde hardware revision:0x%x\n", sde_kms->core_rev);

	sde_kms->catalog = sde_hw_catalog_init(dev, sde_kms->core_rev);
	if (IS_ERR_OR_NULL(sde_kms->catalog)) {
		rc = PTR_ERR(sde_kms->catalog);
		if (!sde_kms->catalog)
			rc = -EINVAL;
		SDE_ERROR("catalog init failed: %d\n", rc);
		sde_kms->catalog = NULL;
		goto power_error;
	}

	sde_dbg_init_dbg_buses(sde_kms->core_rev);

	/*
	 * Now we need to read the HW catalog and initialize resources such as
	 * clocks, regulators, GDSC/MMAGIC, ioremap the register ranges etc
	 */
	rc = _sde_kms_mmu_init(sde_kms);
	if (rc) {
		SDE_ERROR("sde_kms_mmu_init failed: %d\n", rc);
		goto power_error;
	}

	/* Initialize reg dma block which is a singleton */
	rc = sde_reg_dma_init(sde_kms->reg_dma, sde_kms->catalog,
			sde_kms->dev);
	if (rc) {
		SDE_ERROR("failed: reg dma init failed\n");
		goto power_error;
	}

	rc = sde_rm_init(&sde_kms->rm, sde_kms->catalog, sde_kms->mmio,
			sde_kms->dev);
	if (rc) {
		SDE_ERROR("rm init failed: %d\n", rc);
		goto power_error;
	}

	sde_kms->rm_init = true;

	sde_kms->hw_mdp = sde_rm_get_mdp(&sde_kms->rm);
	if (IS_ERR_OR_NULL(sde_kms->hw_mdp)) {
		rc = PTR_ERR(sde_kms->hw_mdp);
		if (!sde_kms->hw_mdp)
			rc = -EINVAL;
		SDE_ERROR("failed to get hw_mdp: %d\n", rc);
		sde_kms->hw_mdp = NULL;
		goto power_error;
	}

	for (i = 0; i < sde_kms->catalog->vbif_count; i++) {
		u32 vbif_idx = sde_kms->catalog->vbif[i].id;

		sde_kms->hw_vbif[i] = sde_hw_vbif_init(vbif_idx,
				sde_kms->vbif[vbif_idx], sde_kms->catalog);
		if (IS_ERR_OR_NULL(sde_kms->hw_vbif[vbif_idx])) {
			rc = PTR_ERR(sde_kms->hw_vbif[vbif_idx]);
			if (!sde_kms->hw_vbif[vbif_idx])
				rc = -EINVAL;
			SDE_ERROR("failed to init vbif %d: %d\n", vbif_idx, rc);
			sde_kms->hw_vbif[vbif_idx] = NULL;
			goto power_error;
		}
	}

	sde_kms->iclient = msm_ion_client_create(dev->unique);
	if (IS_ERR(sde_kms->iclient)) {
		rc = PTR_ERR(sde_kms->iclient);
		SDE_DEBUG("msm_ion_client not available: %d\n", rc);
		sde_kms->iclient = NULL;
	}


	rc = sde_core_perf_init(&sde_kms->perf, dev, sde_kms->catalog,
			&priv->phandle, priv->pclient, "core_clk");
	if (rc) {
		SDE_ERROR("failed to init perf %d\n", rc);
		goto perf_err;
	}

	sde_kms->hw_intr = sde_hw_intr_init(sde_kms->mmio, sde_kms->catalog);
	if (IS_ERR_OR_NULL(sde_kms->hw_intr)) {
		rc = PTR_ERR(sde_kms->hw_intr);
		SDE_ERROR("hw_intr init failed: %d\n", rc);
		sde_kms->hw_intr = NULL;
		goto hw_intr_init_err;
	}

	/*
	 * _sde_kms_drm_obj_init should create the DRM related objects
	 * i.e. CRTCs, planes, encoders, connectors and so forth
	 */
	rc = _sde_kms_drm_obj_init(sde_kms);
	if (rc) {
		SDE_ERROR("modeset init failed: %d\n", rc);
		goto drm_obj_init_err;
	}

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	/*
	 * max crtc width is equal to the max mixer width * 2 and max height is
	 * is 4K
	 */
	dev->mode_config.max_width = sde_kms->catalog->max_mixer_width * 2;
	dev->mode_config.max_height = 4096;

	/*
	 * Support format modifiers for compression etc.
	 */
	dev->mode_config.allow_fb_modifiers = true;

	/*
	 * Handle (re)initializations during power enable
	 */
	sde_kms_handle_power_event(SDE_POWER_EVENT_POST_ENABLE, sde_kms);
	sde_kms->power_event = sde_power_handle_register_event(&priv->phandle,
			SDE_POWER_EVENT_POST_ENABLE,
			sde_kms_handle_power_event, sde_kms, "kms");

	sde_power_resource_enable(&priv->phandle, sde_kms->core_client, false);
	return 0;

drm_obj_init_err:
	sde_core_perf_destroy(&sde_kms->perf);
hw_intr_init_err:
perf_err:
power_error:
	sde_power_resource_enable(&priv->phandle, sde_kms->core_client, false);
error:
	_sde_kms_hw_destroy(sde_kms, dev->platformdev);
end:
	return rc;
}

struct msm_kms *sde_kms_init(struct drm_device *dev)
{
	struct msm_drm_private *priv;
	struct sde_kms *sde_kms;

	if (!dev || !dev->dev_private) {
		SDE_ERROR("drm device node invalid\n");
		return ERR_PTR(-EINVAL);
	}

	priv = dev->dev_private;

	sde_kms = kzalloc(sizeof(*sde_kms), GFP_KERNEL);
	if (!sde_kms) {
		SDE_ERROR("failed to allocate sde kms\n");
		return ERR_PTR(-ENOMEM);
	}

	msm_kms_init(&sde_kms->base, &kms_funcs);
	sde_kms->dev = dev;

	return &sde_kms->base;
}

static int _sde_kms_register_events(struct msm_kms *kms,
		struct drm_mode_object *obj, u32 event, bool en)
{
	int ret = 0;
	struct drm_crtc *crtc = NULL;
	struct drm_connector *conn = NULL;
	struct sde_kms *sde_kms = NULL;

	if (!kms || !obj) {
		SDE_ERROR("invalid argument kms %pK obj %pK\n", kms, obj);
		return -EINVAL;
	}

	sde_kms = to_sde_kms(kms);
	switch (obj->type) {
	case DRM_MODE_OBJECT_CRTC:
		crtc = obj_to_crtc(obj);
		ret = sde_crtc_register_custom_event(sde_kms, crtc, event, en);
		break;
	case DRM_MODE_OBJECT_CONNECTOR:
		conn = obj_to_connector(obj);
		ret = sde_connector_register_custom_event(sde_kms, conn, event,
				en);
		break;
	}

	return ret;
}
