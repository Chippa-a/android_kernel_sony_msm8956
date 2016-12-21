/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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

#include <linux/sort.h>
#include <drm/drm_mode.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_flip_work.h>

#include "sde_kms.h"
#include "sde_hw_lm.h"
#include "sde_hw_mdp_ctl.h"

#define CRTC_DUAL_MIXERS	2
#define PENDING_FLIP		2

#define CRTC_HW_MIXER_MAXSTAGES(c, idx) ((c)->mixer[idx].sblk->maxblendstages)

struct sde_crtc_mixer {
	struct sde_hw_dspp  *hw_dspp;
	struct sde_hw_mixer *hw_lm;
	struct sde_hw_ctl   *hw_ctl;
	u32 flush_mask;
};

struct sde_crtc {
	struct drm_crtc base;
	char name[8];
	struct drm_plane *plane;
	struct drm_plane *planes[8];
	struct drm_encoder *encoder;
	int id;
	bool enabled;

	spinlock_t lm_lock;	/* protect registers */

	/* HW Resources reserved for the crtc */
	u32  num_ctls;
	u32  num_mixers;
	struct sde_crtc_mixer mixer[CRTC_DUAL_MIXERS];

	/*if there is a pending flip, these will be non-null */
	struct drm_pending_vblank_event *event;
};

#define to_sde_crtc(x) container_of(x, struct sde_crtc, base)

static struct sde_kms *get_kms(struct drm_crtc *crtc)
{
	struct msm_drm_private *priv = crtc->dev->dev_private;

	return to_sde_kms(to_mdp_kms(priv->kms));
}

static inline struct sde_hw_ctl *sde_crtc_rm_get_ctl_path(enum sde_ctl idx,
		void __iomem *addr,
		struct sde_mdss_cfg *m)
{
	/*
	 * This module keeps track of the requested hw resources state,
	 * if the requested resource is being used it returns NULL,
	 * otherwise it returns the hw driver struct
	 */
	return sde_hw_ctl_init(idx, addr, m);
}

static inline struct sde_hw_mixer *sde_crtc_rm_get_mixer(enum sde_lm idx,
		void __iomem *addr,
		struct sde_mdss_cfg *m)
{
	/*
	 * This module keeps track of the requested hw resources state,
	 * if the requested resource is being used it returns NULL,
	 * otherwise it returns the hw driver struct
	 */
	return sde_hw_lm_init(idx, addr, m);
}

static int sde_crtc_reserve_hw_resources(struct drm_crtc *crtc,
		struct drm_encoder *encoder)
{
	/*
	 * Assign CRTC resources
	 * num_ctls;
	 * num_mixers;
	 * sde_lm mixer[CRTC_MAX_PIPES];
	 * sde_ctl ctl[CRTC_MAX_PIPES];
	 */
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_kms *kms = get_kms(crtc);
	enum sde_lm lm_id[CRTC_DUAL_MIXERS];
	enum sde_ctl ctl_id[CRTC_DUAL_MIXERS];
	int i;

	if (!kms) {
		DBG("[%s] invalid kms\n", __func__);
		return -EINVAL;
	}

	if (!kms->mmio)
		return -EINVAL;

	/*
	 * simple check validate against catalog
	 */
	sde_crtc->num_ctls = 1;
	sde_crtc->num_mixers = 1;
	ctl_id[0] = CTL_0;
	lm_id[0] = LM_0;

	/*
	 * need to also enable MDP core clock and AHB CLK
	 * before touching HW driver
	 */
	DBG("%s Enable clocks\n", __func__);
	sde_enable(kms);
	for (i = 0; i < sde_crtc->num_ctls; i++) {
		sde_crtc->mixer[i].hw_ctl = sde_crtc_rm_get_ctl_path(ctl_id[i],
				kms->mmio, kms->catalog);
		if (!sde_crtc->mixer[i].hw_ctl) {
			DBG("[%s], Invalid ctl_path", __func__);
			return -EACCES;
		}
	}

	for (i = 0; i < sde_crtc->num_mixers; i++) {
		sde_crtc->mixer[i].hw_lm = sde_crtc_rm_get_mixer(lm_id[i],
				kms->mmio, kms->catalog);
		if (!sde_crtc->mixer[i].hw_lm) {
			DBG("[%s], Invalid ctl_path", __func__);
			return -EACCES;
		}
	}
	/*
	 * need to disable MDP core clock and AHB CLK
	 */
	sde_disable(kms);
	return 0;
}

static void sde_crtc_destroy(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);

	DBG("");
	drm_crtc_cleanup(crtc);
	kfree(sde_crtc);
}

static bool sde_crtc_mode_fixup(struct drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	DBG("");
	return true;
}

static void sde_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_crtc_mixer *mixer = sde_crtc->mixer;
	struct drm_device *dev = crtc->dev;
	struct sde_hw_mixer *lm;
	unsigned long flags;
	struct drm_display_mode *mode;
	struct sde_hw_mixer_cfg cfg;
	u32 mixer_width;
	int i;
	int rc;

	DBG("");
	if (WARN_ON(!crtc->state))
		return;

	mode = &crtc->state->adjusted_mode;

	DBG("%s: set mode: %d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x",
			sde_crtc->name, mode->base.id, mode->name,
			mode->vrefresh, mode->clock,
			mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal,
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal,
			mode->type, mode->flags);

	/*
	 * reserve mixer(s) if not already avaialable
	 * if dual mode, mixer_width = half mode width
	 * program mode configuration on mixer(s)
	 */
	if ((sde_crtc->num_ctls == 0) ||
		(sde_crtc->num_mixers == 0)) {
		rc = sde_crtc_reserve_hw_resources(crtc, sde_crtc->encoder);
		if (rc) {
			dev_err(dev->dev, " error reserving HW resource for this CRTC\n");
			return;
		}
	}

	if (sde_crtc->num_mixers == CRTC_DUAL_MIXERS)
		mixer_width = mode->hdisplay >> 1;
	else
		mixer_width = mode->hdisplay;

	spin_lock_irqsave(&sde_crtc->lm_lock, flags);

	for (i = 0; i < sde_crtc->num_mixers; i++) {
		lm = mixer[i].hw_lm;
		cfg.out_width = mixer_width;
		cfg.out_height = mode->vdisplay;
		cfg.right_mixer = (i == 0) ? false : true;
		cfg.flags = 0;
		lm->ops.setup_mixer_out(lm, &cfg);
	}

	spin_unlock_irqrestore(&sde_crtc->lm_lock, flags);
}

static void sde_crtc_get_blend_cfg(struct sde_hw_blend_cfg *cfg,
		struct sde_plane_state *pstate)
{
	const struct mdp_format *format;
	struct drm_plane *plane;

	format = to_mdp_format(
			msm_framebuffer_format(pstate->base.fb));
	plane = pstate->base.plane;

	cfg->fg.alpha_sel = ALPHA_FG_CONST;
	cfg->bg.alpha_sel = ALPHA_BG_CONST;
	cfg->fg.const_alpha = pstate->alpha;
	cfg->bg.const_alpha = 0xFF - pstate->alpha;

	if (format->alpha_enable && pstate->premultiplied) {
		cfg->fg.alpha_sel = ALPHA_FG_CONST;
		cfg->bg.alpha_sel = ALPHA_FG_PIXEL;
		if (pstate->alpha != 0xff) {
			cfg->bg.const_alpha = pstate->alpha;
			cfg->bg.inv_alpha_sel = 1;
			cfg->bg.mod_alpha = 1;
		} else {
			cfg->bg.inv_mode_alpha = 1;
		}
	} else if (format->alpha_enable) {
		cfg->fg.alpha_sel = ALPHA_FG_PIXEL;
		cfg->bg.alpha_sel = ALPHA_FG_PIXEL;
		if (pstate->alpha != 0xff) {
			cfg->bg.const_alpha = pstate->alpha;
			cfg->fg.mod_alpha = 1;
			cfg->bg.inv_alpha_sel = 1;
			cfg->bg.mod_alpha = 1;
			cfg->bg.inv_mode_alpha = 1;
		} else {
			cfg->bg.inv_mode_alpha = 1;
		}
	}
}

static void blend_setup(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_crtc_mixer *mixer = sde_crtc->mixer;
	struct drm_plane *plane;
	struct sde_plane_state *pstate, *pstates[SDE_STAGE_MAX] = {0};
	struct sde_hw_stage_cfg stage_cfg;
	struct sde_hw_blend_cfg blend;
	struct sde_hw_ctl *ctl;
	struct sde_hw_mixer *lm;
	u32 flush_mask = 0;
	unsigned long flags;
	int i, j, plane_cnt = 0;

	spin_lock_irqsave(&sde_crtc->lm_lock, flags);

	/* ctl could be reserved already */
	if (!sde_crtc->num_ctls)
		goto out;

	/* initialize stage cfg */
	memset(&stage_cfg, 0, sizeof(stage_cfg));
	memset(&blend, 0, sizeof(blend));

	/* Collect all plane information */
	drm_atomic_crtc_for_each_plane(plane, crtc) {
		pstate = to_sde_plane_state(plane->state);
		pstates[pstate->stage] = pstate;
		plane_cnt++;
		for (i = 0; i < sde_crtc->num_mixers; i++) {
			stage_cfg.stage[pstate->stage][i] =
				sde_plane_pipe(plane);

			/* Cache the flushmask for this layer
			 * sourcesplit is always enabled, so this layer will
			 * be staged on both the mixers
			 */
			ctl = mixer[i].hw_ctl;
			ctl->ops.get_bitmask_sspp(ctl, &flush_mask,
					sde_plane_pipe(plane));
		}
	}

	/*
	 * If there is no base layer, enable border color.
	 * currently border color is always black
	 */
	if ((stage_cfg.stage[SDE_STAGE_BASE][0] == SSPP_NONE) &&
		plane_cnt) {
		stage_cfg.border_enable = 1;
		DBG("Border Color is enabled\n");
	}

	/* Program hw */
	for (i = 0; i < sde_crtc->num_mixers; i++) {
		if (!mixer[i].hw_lm)
			continue;

		if (!mixer[i].hw_ctl)
			continue;

		ctl = mixer[i].hw_ctl;
		lm = mixer[i].hw_lm;

		/* stage config */
		ctl->ops.setup_blendstage(ctl, mixer[i].hw_lm->idx,
			&stage_cfg);
		/* stage config flush mask */
		mixer[i].flush_mask = flush_mask;
		/* get the flush mask for mixer */
		ctl->ops.get_bitmask_mixer(ctl, &mixer[i].flush_mask,
			mixer[i].hw_lm->idx);

		/* blend config */
		for (j = SDE_STAGE_0; j < SDE_STAGE_MAX; j++) {
			if (!pstates[j])
				continue;
			sde_crtc_get_blend_cfg(&blend, pstates[j]);
			blend.fg.alpha_sel = ALPHA_FG_CONST;
			blend.bg.alpha_sel = ALPHA_BG_CONST;
			blend.fg.const_alpha = pstate->alpha;
			blend.bg.const_alpha = 0xFF - pstate->alpha;
			lm->ops.setup_blend_config(lm, j, &blend);
		}
	}
out:
	spin_unlock_irqrestore(&sde_crtc->lm_lock, flags);
}

static void request_pending(struct drm_crtc *crtc, u32 pending)
{
	DBG("");
}
/**
 * Flush the CTL PATH
 */
static u32 crtc_flush_all(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_hw_ctl *ctl;
	int i;

	DBG("");

	for (i = 0; i < sde_crtc->num_ctls; i++) {
		/*
		 * Query flush_mask from encoder
		 * and append to the ctl_path flush_mask
		 */
		ctl = sde_crtc->mixer[i].hw_ctl;
		ctl->ops.get_bitmask_intf(ctl,
				&(sde_crtc->mixer[i].flush_mask),
				INTF_1);
		ctl->ops.setup_flush(ctl,
				sde_crtc->mixer[i].flush_mask);
	}

	return 0;
}

static void sde_crtc_atomic_begin(struct drm_crtc *crtc,
		struct drm_crtc_state *old_crtc_state)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	unsigned long flags;

	DBG("");

	WARN_ON(sde_crtc->event);

	spin_lock_irqsave(&dev->event_lock, flags);
	sde_crtc->event = crtc->state->event;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	/*
	 * If no CTL has been allocated in sde_crtc_atomic_check(),
	 * it means we are trying to flush a CRTC whose state is disabled:
	 * nothing else needs to be done.
	 */
	if (unlikely(!sde_crtc->num_ctls))
		return;

	blend_setup(crtc);

	/*
	 * PP_DONE irq is only used by command mode for now.
	 * It is better to request pending before FLUSH and START trigger
	 * to make sure no pp_done irq missed.
	 * This is safe because no pp_done will happen before SW trigger
	 * in command mode.
	 */
}

static void sde_crtc_atomic_flush(struct drm_crtc *crtc,
		struct drm_crtc_state *old_crtc_state)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	unsigned long flags;

	DBG("");

	WARN_ON(sde_crtc->event);

	spin_lock_irqsave(&dev->event_lock, flags);
	sde_crtc->event = crtc->state->event;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	/*
	 * If no CTL has been allocated in sde_crtc_atomic_check(),
	 * it means we are trying to flush a CRTC whose state is disabled:
	 * nothing else needs to be done.
	 */
	if (unlikely(!sde_crtc->num_ctls))
		return;

	crtc_flush_all(crtc);

	request_pending(crtc, PENDING_FLIP);
}

static int sde_crtc_set_property(struct drm_crtc *crtc,
		struct drm_property *property, uint64_t val)
{
	return -EINVAL;
}

static int sde_crtc_cursor_set(struct drm_crtc *crtc,
		struct drm_file *file, uint32_t handle,
		uint32_t width, uint32_t height)
{
	return 0;
}

static int sde_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	return 0;
}

static void sde_crtc_disable(struct drm_crtc *crtc)
{
	DBG("");
}

static void sde_crtc_enable(struct drm_crtc *crtc)
{
	DBG("");
}

struct plane_state {
	struct drm_plane *plane;
	struct sde_plane_state *state;
};

static int pstate_cmp(const void *a, const void *b)
{
	struct plane_state *pa = (struct plane_state *)a;
	struct plane_state *pb = (struct plane_state *)b;

	return pa->state->zpos - pb->state->zpos;
}

static int sde_crtc_atomic_check(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_kms *sde_kms = get_kms(crtc);
	struct drm_plane *plane;
	struct drm_device *dev = crtc->dev;
	struct plane_state pstates[SDE_STAGE_MAX];
	int max_stages = CRTC_HW_MIXER_MAXSTAGES(sde_kms->catalog, 0);
	int cnt = 0, i;

	DBG("%s: check", sde_crtc->name);

	/* verify that there are not too many planes attached to crtc
	 * and that we don't have conflicting mixer stages:
	 */
	drm_atomic_crtc_state_for_each_plane(plane, state) {
		struct drm_plane_state *pstate;

		if (cnt >= (max_stages)) {
			dev_err(dev->dev, "too many planes!\n");
			return -EINVAL;
		}

		pstate = state->state->plane_states[drm_plane_index(plane)];

		/* plane might not have changed, in which case take
		 * current state:
		 */
		if (!pstate)
			pstate = plane->state;
		pstates[cnt].plane = plane;
		pstates[cnt].state = to_sde_plane_state(pstate);

		cnt++;
	}

	/* assign a stage based on sorted zpos property */
	sort(pstates, cnt, sizeof(pstates[0]), pstate_cmp, NULL);

	for (i = 0; i < cnt; i++) {
		pstates[i].state->stage = SDE_STAGE_0 + i;
		DBG("%s: assign pipe %d on stage=%d", sde_crtc->name,
				sde_plane_pipe(pstates[i].plane),
				pstates[i].state->stage);
	}

	return 0;
}

static const struct drm_crtc_funcs sde_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.destroy = sde_crtc_destroy,
	.page_flip = drm_atomic_helper_page_flip,
	.set_property = sde_crtc_set_property,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.cursor_set = sde_crtc_cursor_set,
	.cursor_move = sde_crtc_cursor_move,
};

static const struct drm_crtc_helper_funcs sde_crtc_helper_funcs = {
	.mode_fixup = sde_crtc_mode_fixup,
	.mode_set_nofb = sde_crtc_mode_set_nofb,
	.disable = sde_crtc_disable,
	.enable = sde_crtc_enable,
	.atomic_check = sde_crtc_atomic_check,
	.atomic_begin = sde_crtc_atomic_begin,
	.atomic_flush = sde_crtc_atomic_flush,
};

uint32_t sde_crtc_vblank(struct drm_crtc *crtc)
{
	return 0;
}

void sde_crtc_cancel_pending_flip(struct drm_crtc *crtc, struct drm_file *file)
{
}

static void sde_crtc_install_properties(struct drm_crtc *crtc,
	struct drm_mode_object *obj)
{
}


/* initialize crtc */
struct drm_crtc *sde_crtc_init(struct drm_device *dev,
		struct drm_encoder *encoder,
		struct drm_plane *plane, int id)
{
	struct drm_crtc *crtc = NULL;
	struct sde_crtc *sde_crtc;
	int rc;

	sde_crtc = kzalloc(sizeof(*sde_crtc), GFP_KERNEL);
	if (!sde_crtc)
		return ERR_PTR(-ENOMEM);

	crtc = &sde_crtc->base;

	sde_crtc->id = id;
	sde_crtc->encoder = encoder;

	sde_crtc_install_properties(crtc, &crtc->base);

	drm_crtc_init_with_planes(dev, crtc, plane, NULL, &sde_crtc_funcs);

	drm_crtc_helper_add(crtc, &sde_crtc_helper_funcs);
	plane->crtc = crtc;

	rc = sde_crtc_reserve_hw_resources(crtc, encoder);
	if (rc) {
		dev_err(dev->dev, " error reserving HW resource for this CRTC\n");
		return ERR_PTR(-EINVAL);
	}

	DBG("%s: Successfully initialized crtc\n", __func__);
	return crtc;
}
