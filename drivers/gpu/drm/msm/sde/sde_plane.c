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
#include <linux/debugfs.h>
#include <uapi/drm/sde_drm.h>
#include "sde_kms.h"
#include "sde_formats.h"
#include "sde_hw_sspp.h"

#define DECIMATED_DIMENSION(dim, deci) (((dim) + ((1 << (deci)) - 1)) >> (deci))
#define PHASE_STEP_SHIFT	21
#define PHASE_STEP_UNIT_SCALE   ((int) (1 << PHASE_STEP_SHIFT))
#define PHASE_RESIDUAL		15

#define SHARP_STRENGTH_DEFAULT	32
#define SHARP_EDGE_THR_DEFAULT	112
#define SHARP_SMOOTH_THR_DEFAULT	8
#define SHARP_NOISE_THR_DEFAULT	2

#define SDE_NAME_SIZE  12

struct sde_plane {
	struct drm_plane base;

	int mmu_id;

	enum sde_sspp pipe;
	uint32_t features;      /* capabilities from catalog */
	uint32_t nformats;
	uint32_t formats[32];

	struct sde_hw_pipe *pipe_hw;
	struct sde_hw_pipe_cfg pipe_cfg;
	struct sde_hw_pixel_ext pixel_ext;
	struct sde_hw_sharp_cfg sharp_cfg;
	struct sde_hw_scaler3_cfg scaler3_cfg;

	const struct sde_sspp_sub_blks *pipe_sblk;

	char pipe_name[SDE_NAME_SIZE];

	/* debugfs related stuff */
	struct dentry *debugfs_root;
	struct sde_debugfs_regset32 debugfs_src;
	struct sde_debugfs_regset32 debugfs_scaler;
	struct sde_debugfs_regset32 debugfs_csc;
};
#define to_sde_plane(x) container_of(x, struct sde_plane, base)

static bool sde_plane_enabled(struct drm_plane_state *state)
{
	return state->fb && state->crtc;
}

static void _sde_plane_set_scanout(struct drm_plane *plane,
		struct sde_plane_state *pstate,
		struct sde_hw_pipe_cfg *pipe_cfg, struct drm_framebuffer *fb)
{
	struct sde_plane *psde = to_sde_plane(plane);
	unsigned int shift;
	int i;

	if (pipe_cfg && fb && psde->pipe_hw->ops.setup_sourceaddress) {
		/* stride */
		if (sde_plane_get_property(pstate, PLANE_PROP_SRC_CONFIG) &
				BIT(SDE_DRM_DEINTERLACE))
			shift = 1;
		else
			shift = 0;

		i = min_t(int, ARRAY_SIZE(fb->pitches), SDE_MAX_PLANES);
		while (i) {
			--i;
			pipe_cfg->src.ystride[i] = fb->pitches[i] << shift;
		}

		/* address */
		for (i = 0; i < ARRAY_SIZE(pipe_cfg->addr.plane); ++i)
			pipe_cfg->addr.plane[i] = msm_framebuffer_iova(fb,
					psde->mmu_id, i);

		/* hw driver */
		psde->pipe_hw->ops.setup_sourceaddress(psde->pipe_hw, pipe_cfg);
	}
}

static void _sde_plane_setup_scaler3(struct drm_plane *plane,
		uint32_t src_w, uint32_t src_h, uint32_t dst_w, uint32_t dst_h,
		struct sde_hw_scaler3_cfg *scale_cfg,
		struct sde_mdp_format_params *fmt,
		uint32_t chroma_subsmpl_h, uint32_t chroma_subsmpl_v)
{
}

static void _sde_plane_setup_scaler2(struct drm_plane *plane,
		uint32_t src, uint32_t dst, uint32_t *phase_steps,
		enum sde_hw_filter *filter, struct sde_mdp_format_params *fmt,
		uint32_t chroma_subsampling)
{
	/* calculate phase steps, leave init phase as zero */
	phase_steps[SDE_SSPP_COMP_0] =
		mult_frac(1 << PHASE_STEP_SHIFT, src, dst);
	phase_steps[SDE_SSPP_COMP_1_2] =
		phase_steps[SDE_SSPP_COMP_0] / chroma_subsampling;
	phase_steps[SDE_SSPP_COMP_2] = phase_steps[SDE_SSPP_COMP_1_2];
	phase_steps[SDE_SSPP_COMP_3] = phase_steps[SDE_SSPP_COMP_0];

	/* calculate scaler config, if necessary */
	if (fmt->is_yuv || src != dst) {
		filter[SDE_SSPP_COMP_3] =
			(src <= dst) ? SDE_MDP_SCALE_FILTER_BIL :
			SDE_MDP_SCALE_FILTER_PCMN;

		if (fmt->is_yuv) {
			filter[SDE_SSPP_COMP_0] = SDE_MDP_SCALE_FILTER_CA;
			filter[SDE_SSPP_COMP_1_2] = filter[SDE_SSPP_COMP_3];
		} else {
			filter[SDE_SSPP_COMP_0] = filter[SDE_SSPP_COMP_3];
			filter[SDE_SSPP_COMP_1_2] =
				SDE_MDP_SCALE_FILTER_NEAREST;
		}
	} else {
		/* disable scaler */
		filter[SDE_SSPP_COMP_0] = SDE_MDP_SCALE_FILTER_MAX;
		filter[SDE_SSPP_COMP_1_2] = SDE_MDP_SCALE_FILTER_MAX;
		filter[SDE_SSPP_COMP_3] = SDE_MDP_SCALE_FILTER_MAX;
	}
}

static void _sde_plane_setup_pixel_ext(struct drm_plane *plane,
		uint32_t src, uint32_t dst, uint32_t decimated_src,
		uint32_t *phase_steps, uint32_t *out_src, int *out_edge1,
		int *out_edge2, enum sde_hw_filter *filter,
		struct sde_mdp_format_params *fmt, uint32_t chroma_subsampling,
		bool post_compare)
{
	int64_t edge1, edge2, caf;
	uint32_t src_work;
	int i, tmp;

	if (plane && phase_steps && out_src && out_edge1 &&
			out_edge2 && filter && fmt) {
		/* handle CAF for YUV formats */
		if (fmt->is_yuv && SDE_MDP_SCALE_FILTER_CA == *filter)
			caf = PHASE_STEP_UNIT_SCALE;
		else
			caf = 0;

		for (i = 0; i < SDE_MAX_PLANES; i++) {
			src_work = decimated_src;
			if (i == SDE_SSPP_COMP_1_2 || i == SDE_SSPP_COMP_2)
				src_work /= chroma_subsampling;
			if (post_compare)
				src = src_work;
			if (!(fmt->is_yuv) && (src == dst)) {
				/* unity */
				edge1 = 0;
				edge2 = 0;
			} else if (dst >= src) {
				/* upscale */
				edge1 = (1 << PHASE_RESIDUAL);
				edge1 -= caf;
				edge2 = (1 << PHASE_RESIDUAL);
				edge2 += (dst - 1) * *(phase_steps + i);
				edge2 -= (src_work - 1) * PHASE_STEP_UNIT_SCALE;
				edge2 += caf;
				edge2 = -(edge2);
			} else {
				/* downscale */
				edge1 = 0;
				edge2 = (dst - 1) * *(phase_steps + i);
				edge2 -= (src_work - 1) * PHASE_STEP_UNIT_SCALE;
				edge2 += *(phase_steps + i);
				edge2 = -(edge2);
			}

			/* only enable CAF for luma plane */
			caf = 0;

			/* populate output arrays */
			*(out_src + i) = src_work;

			/* edge updates taken from __pxl_extn_helper */
			if (edge1 >= 0) {
				tmp = (uint32_t)edge1;
				tmp >>= PHASE_STEP_SHIFT;
				*(out_edge1 + i) = -tmp;
			} else {
				tmp = (uint32_t)(-edge1);
				*(out_edge1 + i) =
					(tmp + PHASE_STEP_UNIT_SCALE - 1) >>
					PHASE_STEP_SHIFT;
			}
			if (edge2 >= 0) {
				tmp = (uint32_t)edge2;
				tmp >>= PHASE_STEP_SHIFT;
				*(out_edge2 + i) = -tmp;
			} else {
				tmp = (uint32_t)(-edge2);
				*(out_edge2 + i) =
					(tmp + PHASE_STEP_UNIT_SCALE - 1) >>
					PHASE_STEP_SHIFT;
			}
		}
	}
}

static void *_sde_plane_get_blob(struct sde_plane_state *pstate,
		enum msm_mdp_plane_property property, size_t *byte_len)
{
	struct drm_property_blob *blob;
	size_t len = 0;
	void *ret = 0;

	if (!pstate || (property >= PLANE_PROP_BLOBCOUNT)) {
		DRM_ERROR("Invalid argument(s)\n");
	} else {
		blob = pstate->property_blobs[property];
		if (blob) {
			len = blob->length;
			ret = &blob->data;
		}
	}

	if (byte_len)
		*byte_len = len;

	return ret;
}

/**
 * _sde_plane_verify_blob - verify incoming blob is big enough to contain
 *                          sub-structure
 * @blob_ptr: Pointer to start of incoming blob data
 * @blob_size: Size of incoming blob data, in bytes
 * @sub_ptr: Pointer to start of desired sub-structure
 * @sub_size: Required size of sub-structure, in bytes
 */
static int _sde_plane_verify_blob(void *blob_ptr,
		size_t blob_size,
		void *sub_ptr,
		size_t sub_size)
{
	/*
	 * Use the blob size provided by drm to check if there are enough
	 * bytes from the start of versioned sub-structures to the end of
	 * blob data:
	 *
	 * e.g.,
	 * blob_ptr             --> struct blob_data {
	 *                                  uint32_t version;
	 * sub_ptr              -->         struct blob_data_v1 v1;
	 * sub_ptr + sub_size   -->         struct blob_stuff more_stuff;
	 * blob_ptr + blob_size --> };
	 *
	 * It's important to check the actual number of bytes from the start
	 * of the sub-structure to the end of the blob data, and not just rely
	 * on something like,
	 *
	 * sizeof(blob) - sizeof(blob->version) >= sizeof(sub-struct)
	 *
	 * This is because the start of the sub-structure can vary based on
	 * how the compiler pads the overall structure.
	 */
	if (blob_ptr && sub_ptr)
		/* return zero if end of blob >= end of sub-struct */
		return ((unsigned char *)blob_ptr + blob_size) <
			((unsigned char *)sub_ptr + sub_size);
	return -EINVAL;
}

static void _sde_plane_setup_csc(struct sde_plane *psde,
		struct sde_plane_state *pstate,
		struct sde_mdp_format_params *fmt)
{
	static const struct sde_csc_cfg sde_csc_YUV2RGB_601L = {
		{
			0x0254, 0x0000, 0x0331,
			0x0254, 0xff37, 0xfe60,
			0x0254, 0x0409, 0x0000,
		},
		{ 0xfff0, 0xff80, 0xff80,},
		{ 0x0, 0x0, 0x0,},
		{ 0x10, 0xeb, 0x10, 0xf0, 0x10, 0xf0,},
		{ 0x0, 0xff, 0x0, 0xff, 0x0, 0xff,},
	};

	static const struct sde_csc_cfg sde_csc_NOP = {
		{
			0x0200, 0x0000, 0x0000,
			0x0000, 0x0200, 0x0000,
			0x0000, 0x0000, 0x0200,
		},
		{ 0x0, 0x0, 0x0,},
		{ 0x0, 0x0, 0x0,},
		{ 0x0, 0xff, 0x0, 0xff, 0x0, 0xff,},
		{ 0x0, 0xff, 0x0, 0xff, 0x0, 0xff,},
	};
	struct sde_drm_csc *csc = NULL;
	size_t csc_size = 0;
	bool user_blob = false;

	if (!psde->pipe_hw->ops.setup_csc)
		return;

	/* check for user space override */
	csc = _sde_plane_get_blob(pstate, PLANE_PROP_CSC, &csc_size);
	if (csc) {
		struct sde_csc_cfg cfg;
		int i;

		/* user space override */
		memcpy(&cfg, &sde_csc_NOP, sizeof(struct sde_csc_cfg));
		switch (csc->version) {
		case SDE_DRM_CSC_V1:
			if (!_sde_plane_verify_blob(csc,
					csc_size,
					&csc->v1,
					sizeof(struct sde_drm_csc_v1))) {
				for (i = 0; i < SDE_CSC_MATRIX_COEFF_SIZE; ++i)
					cfg.csc_mv[i] =
						csc->v1.ctm_coeff[i] >> 23;
				for (i = 0; i < SDE_CSC_BIAS_SIZE; ++i) {
					cfg.csc_pre_bv[i] =
						csc->v1.pre_bias[i];
					cfg.csc_post_bv[i] =
						csc->v1.post_bias[i];
				}
				for (i = 0; i < SDE_CSC_CLAMP_SIZE; ++i) {
					cfg.csc_pre_lv[i] =
						csc->v1.pre_clamp[i];
					cfg.csc_post_lv[i] =
						csc->v1.post_clamp[i];
				}
				user_blob = true;
			}
			break;
		default:
			break;
		}

		if (!user_blob)
			DRM_ERROR("Invalid csc blob, v%lld\n", csc->version);
		else
			psde->pipe_hw->ops.setup_csc(psde->pipe_hw,
					(struct sde_csc_cfg *)&cfg);
	}

	if (user_blob) {
		DBG("User blobs override for CSC");
	/* revert to kernel default */
	} else if (fmt->is_yuv) {
		psde->pipe_hw->ops.setup_csc(psde->pipe_hw,
			(struct sde_csc_cfg *)&sde_csc_YUV2RGB_601L);
	} else {
		psde->pipe_hw->ops.setup_csc(psde->pipe_hw,
			(struct sde_csc_cfg *)&sde_csc_NOP);
	}
}

static int _sde_plane_mode_set(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		int crtc_x, int crtc_y,
		unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y,
		uint32_t src_w, uint32_t src_h)
{
	struct sde_plane *psde;
	struct sde_plane_state *pstate;
	const struct mdp_format *format;
	uint32_t nplanes, pix_format, tmp;
	uint32_t chroma_subsmpl_h, chroma_subsmpl_v;
	uint32_t src_fmt_flags;
	int i;
	struct sde_mdp_format_params *fmt;
	struct sde_hw_pixel_ext *pe;
	size_t sc_u_size = 0;
	struct sde_drm_scaler *sc_u = NULL;
	struct sde_drm_scaler_v1 *sc_u1 = NULL;

	DBG("");

	if (!plane || !plane->state) {
		DRM_ERROR("Invalid plane/state\n");
		return -EINVAL;
	}
	if (!crtc || !fb) {
		DRM_ERROR("Invalid crtc/fb\n");
		return -EINVAL;
	}
	psde = to_sde_plane(plane);
	pstate = to_sde_plane_state(plane->state);
	nplanes = drm_format_num_planes(fb->pixel_format);

	format = to_mdp_format(msm_framebuffer_format(fb));
	pix_format = format->base.pixel_format;

	/* src values are in Q16 fixed point, convert to integer */
	src_x = src_x >> 16;
	src_y = src_y >> 16;
	src_w = src_w >> 16;
	src_h = src_h >> 16;

	DBG("%s: FB[%u] %u,%u,%u,%u -> CRTC[%u] %d,%d,%u,%u", psde->pipe_name,
			fb->base.id, src_x, src_y, src_w, src_h,
			crtc->base.id, crtc_x, crtc_y, crtc_w, crtc_h);

	/* update format configuration */
	memset(&(psde->pipe_cfg), 0, sizeof(struct sde_hw_pipe_cfg));
	src_fmt_flags = 0;

	psde->pipe_cfg.src.format = sde_mdp_get_format_params(pix_format,
			fb->modifier[0]);
	psde->pipe_cfg.src.width = fb->width;
	psde->pipe_cfg.src.height = fb->height;
	psde->pipe_cfg.src.num_planes = nplanes;

	_sde_plane_set_scanout(plane, pstate, &psde->pipe_cfg, fb);

	/* decimation */
	psde->pipe_cfg.horz_decimation =
		sde_plane_get_property(pstate, PLANE_PROP_H_DECIMATE);
	psde->pipe_cfg.vert_decimation =
		sde_plane_get_property(pstate, PLANE_PROP_V_DECIMATE);

	/* flags */
	DBG("Flags 0x%llX, rotation 0x%llX",
			sde_plane_get_property(pstate, PLANE_PROP_SRC_CONFIG),
			sde_plane_get_property(pstate, PLANE_PROP_ROTATION));
	if (sde_plane_get_property(pstate, PLANE_PROP_ROTATION) &
		BIT(DRM_REFLECT_X))
		src_fmt_flags |= SDE_SSPP_FLIP_LR;
	if (sde_plane_get_property(pstate, PLANE_PROP_ROTATION) &
		BIT(DRM_REFLECT_Y))
		src_fmt_flags |= SDE_SSPP_FLIP_UD;
	if (sde_plane_get_property(pstate, PLANE_PROP_SRC_CONFIG) &
		BIT(SDE_DRM_DEINTERLACE)) {
		src_h /= 2;
		src_y  = DIV_ROUND_UP(src_y, 2);
		src_y &= ~0x1;
	}

	psde->pipe_cfg.src_rect.x = src_x;
	psde->pipe_cfg.src_rect.y = src_y;
	psde->pipe_cfg.src_rect.w = src_w;
	psde->pipe_cfg.src_rect.h = src_h;

	psde->pipe_cfg.dst_rect.x = crtc_x;
	psde->pipe_cfg.dst_rect.y = crtc_y;
	psde->pipe_cfg.dst_rect.w = crtc_w;
	psde->pipe_cfg.dst_rect.h = crtc_h;

	/* get sde pixel format definition */
	fmt = psde->pipe_cfg.src.format;

	/* don't chroma subsample if decimating */
	chroma_subsmpl_h = psde->pipe_cfg.horz_decimation ? 1 :
		drm_format_horz_chroma_subsampling(pix_format);
	chroma_subsmpl_v = psde->pipe_cfg.vert_decimation ? 1 :
		drm_format_vert_chroma_subsampling(pix_format);

	pe = &(psde->pixel_ext);
	memset(pe, 0, sizeof(struct sde_hw_pixel_ext));

	/* get scaler config from user space */
	sc_u = _sde_plane_get_blob(pstate, PLANE_PROP_SCALER, &sc_u_size);
	if (sc_u) {
		switch (sc_u->version) {
		case SDE_DRM_SCALER_V1:
			if (!_sde_plane_verify_blob(sc_u,
						sc_u_size,
						&sc_u->v1,
						sizeof(*sc_u1)))
				sc_u1 = &sc_u->v1;
			break;
		default:
			DBG("Unrecognized scaler blob v%lld", sc_u->version);
			break;
		}
	}

	/* update scaler */
	if (psde->features & BIT(SDE_SSPP_SCALER_QSEED3)) {
		if (sc_u1 && (sc_u1->enable & SDE_DRM_SCALER_SCALER_3))
			DBG("QSEED3 blob detected");
		else
			_sde_plane_setup_scaler3(plane, src_w, src_h, crtc_w,
					crtc_h, &psde->scaler3_cfg, fmt,
					chroma_subsmpl_h, chroma_subsmpl_v);
	} else {
		/* always calculate basic scaler config */
		if (sc_u1 && (sc_u1->enable & SDE_DRM_SCALER_SCALER_2)) {
			/* populate from user space */
			for (i = 0; i < SDE_MAX_PLANES; i++) {
				pe->init_phase_x[i] = sc_u1->init_phase_x[i];
				pe->phase_step_x[i] = sc_u1->phase_step_x[i];
				pe->init_phase_y[i] = sc_u1->init_phase_y[i];
				pe->phase_step_y[i] = sc_u1->phase_step_y[i];

				pe->horz_filter[i] = sc_u1->horz_filter[i];
				pe->vert_filter[i] = sc_u1->vert_filter[i];
			}
		} else {
			/* calculate phase steps */
			_sde_plane_setup_scaler2(plane, src_w, crtc_w,
					pe->phase_step_x,
					pe->horz_filter, fmt, chroma_subsmpl_h);
			_sde_plane_setup_scaler2(plane, src_h, crtc_h,
					pe->phase_step_y,
					pe->vert_filter, fmt, chroma_subsmpl_v);
		}
	}

	/* update pixel extensions */
	if (sc_u1 && (sc_u1->enable & SDE_DRM_SCALER_PIX_EXT)) {
		/* populate from user space */
		for (i = 0; i < SDE_MAX_PLANES; i++) {
			pe->num_ext_pxls_left[i] = sc_u1->lr.num_pxls_start[i];
			pe->num_ext_pxls_right[i] = sc_u1->lr.num_pxls_end[i];
			pe->left_ftch[i] = sc_u1->lr.ftch_start[i];
			pe->right_ftch[i] = sc_u1->lr.ftch_end[i];
			pe->left_rpt[i] = sc_u1->lr.rpt_start[i];
			pe->right_rpt[i] = sc_u1->lr.rpt_end[i];
			pe->roi_w[i] = sc_u1->lr.roi[i];

			pe->num_ext_pxls_top[i] = sc_u1->tb.num_pxls_start[i];
			pe->num_ext_pxls_btm[i] = sc_u1->tb.num_pxls_end[i];
			pe->top_ftch[i] = sc_u1->tb.ftch_start[i];
			pe->btm_ftch[i] = sc_u1->tb.ftch_end[i];
			pe->top_rpt[i] = sc_u1->tb.rpt_start[i];
			pe->btm_rpt[i] = sc_u1->tb.rpt_end[i];
			pe->roi_h[i] = sc_u1->tb.roi[i];
		}
	} else {
		/* calculate left/right/top/bottom pixel extensions */
		tmp = DECIMATED_DIMENSION(src_w,
				psde->pipe_cfg.horz_decimation);
		if (fmt->is_yuv)
			tmp &= ~0x1;
		_sde_plane_setup_pixel_ext(plane, src_w, crtc_w, tmp,
				pe->phase_step_x,
				pe->roi_w,
				pe->num_ext_pxls_left,
				pe->num_ext_pxls_right, pe->horz_filter, fmt,
				chroma_subsmpl_h, 0);

		tmp = DECIMATED_DIMENSION(src_h,
				psde->pipe_cfg.vert_decimation);
		_sde_plane_setup_pixel_ext(plane, src_h, crtc_h, tmp,
				pe->phase_step_y,
				pe->roi_h,
				pe->num_ext_pxls_top,
				pe->num_ext_pxls_btm, pe->vert_filter, fmt,
				chroma_subsmpl_v, 1);

		for (i = 0; i < SDE_MAX_PLANES; i++) {
			if (pe->num_ext_pxls_left[i] >= 0)
				pe->left_rpt[i] =
					pe->num_ext_pxls_left[i];
			else
				pe->left_ftch[i] =
					pe->num_ext_pxls_left[i];

			if (pe->num_ext_pxls_right[i] >= 0)
				pe->right_rpt[i] =
					pe->num_ext_pxls_right[i];
			else
				pe->right_ftch[i] =
					pe->num_ext_pxls_right[i];

			if (pe->num_ext_pxls_top[i] >= 0)
				pe->top_rpt[i] =
					pe->num_ext_pxls_top[i];
			else
				pe->top_ftch[i] =
					pe->num_ext_pxls_top[i];

			if (pe->num_ext_pxls_btm[i] >= 0)
				pe->btm_rpt[i] =
					pe->num_ext_pxls_btm[i];
			else
				pe->btm_ftch[i] =
					pe->num_ext_pxls_btm[i];
		}
	}

	if (psde->pipe_hw->ops.setup_format)
		psde->pipe_hw->ops.setup_format(psde->pipe_hw,
				&psde->pipe_cfg, src_fmt_flags);
	if (psde->pipe_hw->ops.setup_rects)
		psde->pipe_hw->ops.setup_rects(psde->pipe_hw,
				&psde->pipe_cfg, &psde->pixel_ext);

	/* update sharpening */
	psde->sharp_cfg.strength = SHARP_STRENGTH_DEFAULT;
	psde->sharp_cfg.edge_thr = SHARP_EDGE_THR_DEFAULT;
	psde->sharp_cfg.smooth_thr = SHARP_SMOOTH_THR_DEFAULT;
	psde->sharp_cfg.noise_thr = SHARP_NOISE_THR_DEFAULT;

	if (psde->pipe_hw->ops.setup_sharpening)
		psde->pipe_hw->ops.setup_sharpening(psde->pipe_hw,
			&psde->sharp_cfg);

	/* update csc */
	if (fmt->is_yuv)
		_sde_plane_setup_csc(psde, pstate, fmt);

	return 0;
}

static int sde_plane_prepare_fb(struct drm_plane *plane,
		const struct drm_plane_state *new_state)
{
	struct drm_framebuffer *fb = new_state->fb;
	struct sde_plane *psde = to_sde_plane(plane);

	if (!new_state->fb)
		return 0;

	DBG("%s: prepare: FB[%u]", psde->pipe_name, fb->base.id);
	return msm_framebuffer_prepare(fb, psde->mmu_id);
}

static void sde_plane_cleanup_fb(struct drm_plane *plane,
		const struct drm_plane_state *old_state)
{
	struct drm_framebuffer *fb = old_state->fb;
	struct sde_plane *psde = to_sde_plane(plane);

	if (!fb)
		return;

	DBG("%s: cleanup: FB[%u]", psde->pipe_name, fb->base.id);
	msm_framebuffer_cleanup(fb, psde->mmu_id);
}

static int sde_plane_atomic_check(struct drm_plane *plane,
		struct drm_plane_state *state)
{
	struct sde_plane *psde = to_sde_plane(plane);
	struct drm_plane_state *old_state = plane->state;
	const struct mdp_format *format;

	DBG("%s: check (%d -> %d)", psde->pipe_name,
			sde_plane_enabled(old_state), sde_plane_enabled(state));

	if (sde_plane_enabled(state)) {
		/* CIFIX: don't use mdp format? */
		format = to_mdp_format(msm_framebuffer_format(state->fb));
		if (MDP_FORMAT_IS_YUV(format) &&
			(!(psde->features & SDE_SSPP_SCALER) ||
			 !(psde->features & BIT(SDE_SSPP_CSC)))) {
			DRM_ERROR("Pipe doesn't support YUV\n");

			return -EINVAL;
		}

		if (!(psde->features & SDE_SSPP_SCALER) &&
			(((state->src_w >> 16) != state->crtc_w) ||
			((state->src_h >> 16) != state->crtc_h))) {
			DRM_ERROR(
				"Unsupported Pipe scaling (%dx%d -> %dx%d)\n",
				state->src_w >> 16, state->src_h >> 16,
				state->crtc_w, state->crtc_h);

			return -EINVAL;
		}
	}

	if (sde_plane_enabled(state) && sde_plane_enabled(old_state)) {
		/* we cannot change SMP block configuration during scanout: */
		bool full_modeset = false;

		if (state->fb->pixel_format != old_state->fb->pixel_format) {
			DBG("%s: pixel_format change!", psde->pipe_name);
			full_modeset = true;
		}
		if (state->src_w != old_state->src_w) {
			DBG("%s: src_w change!", psde->pipe_name);
			full_modeset = true;
		}
		if (to_sde_plane_state(old_state)->pending) {
			DBG("%s: still pending!", psde->pipe_name);
			full_modeset = true;
		}
		if (full_modeset) {
			struct drm_crtc_state *crtc_state =
				drm_atomic_get_crtc_state(state->state,
					state->crtc);
			crtc_state->mode_changed = true;
			to_sde_plane_state(state)->mode_changed = true;
		}
	} else {
		to_sde_plane_state(state)->mode_changed = true;
	}

	return 0;
}

static void sde_plane_atomic_update(struct drm_plane *plane,
				struct drm_plane_state *old_state)
{
	struct sde_plane *sde_plane;
	struct drm_plane_state *state;
	struct sde_plane_state *pstate;

	DBG("%s: update", sde_plane->pipe_name);

	if (!plane || !plane->state) {
		DRM_ERROR("Invalid plane/state\n");
		return;
	}

	sde_plane = to_sde_plane(plane);
	state = plane->state;
	pstate = to_sde_plane_state(state);

	if (!sde_plane_enabled(state)) {
		pstate->pending = true;
	} else if (pstate->mode_changed) {
		int ret;

		pstate->pending = true;
		ret = _sde_plane_mode_set(plane,
				state->crtc, state->fb,
				state->crtc_x, state->crtc_y,
				state->crtc_w, state->crtc_h,
				state->src_x,  state->src_y,
				state->src_w, state->src_h);
		/* atomic_check should have ensured that this doesn't fail */
		WARN_ON(ret < 0);
	} else {
		_sde_plane_set_scanout(plane, pstate,
				&sde_plane->pipe_cfg, state->fb);
	}
}

static void _sde_plane_install_range_property(struct drm_plane *plane,
		struct drm_device *dev, const char *name,
		uint64_t min, uint64_t max, uint64_t init,
		struct drm_property **prop)
{
	if (plane && dev && name && prop) {
		/* only create the property once */
		if (*prop == 0) {
			*prop = drm_property_create_range(dev,
					0 /* flags */, name, min, max);
			if (*prop == 0)
				DRM_ERROR("Create %s property failed\n", name);
		}

		/* always attach property, if created */
		if (*prop)
			drm_object_attach_property(&plane->base, *prop, init);
	}
}

static void _sde_plane_install_rotation_property(struct drm_plane *plane,
		struct drm_device *dev, struct drm_property **prop)
{
	if (plane && dev && prop) {
		/* only create the property once */
		if (*prop == 0) {
			*prop = drm_mode_create_rotation_property(dev,
					BIT(DRM_REFLECT_X) |
					BIT(DRM_REFLECT_Y));
			if (*prop == 0)
				DRM_ERROR("Create rotation property failed\n");
		}

		/* always attach property, if created */
		if (*prop)
			drm_object_attach_property(&plane->base, *prop, 0);
	}
}

static void _sde_plane_install_enum_property(struct drm_plane *plane,
		struct drm_device *dev, const char *name, int is_bitmask,
		const struct drm_prop_enum_list *values, int num_values,
		struct drm_property **prop)
{
	if (plane && dev && name && prop && values && num_values) {
		/* only create the property once */
		if (*prop == 0) {
			/* 'bitmask' is a special type of 'enum' */
			if (is_bitmask)
				*prop = drm_property_create_bitmask(dev,
						DRM_MODE_PROP_BITMASK, name,
						values, num_values, -1);
			else
				*prop = drm_property_create_enum(dev,
						DRM_MODE_PROP_ENUM, name,
						values, num_values);
			if (*prop == 0)
				DRM_ERROR("Create %s property failed\n", name);
		}

		/* always attach property, if created */
		if (*prop)
			drm_object_attach_property(&plane->base, *prop, 0);
	}
}

static void _sde_plane_install_blob_property(struct drm_plane *plane,
		struct drm_device *dev, const char *name,
		struct drm_property **prop)
{
	if (plane && dev && name && prop) {
		/* only create the property once */
		if (*prop == 0) {
			/* use 'create' for blob property place holder */
			*prop = drm_property_create(dev,
					DRM_MODE_PROP_BLOB, name, 0);
			if (*prop == 0)
				DRM_ERROR("Create %s property failed\n", name);
		}

		/* always attach property, if created */
		if (*prop)
			drm_object_attach_property(&plane->base, *prop, 0);
	}
}

static int _sde_plane_get_property_index(struct drm_plane *plane,
		struct drm_property *property)
{
	struct drm_property **prop_array;
	int idx = PLANE_PROP_COUNT;

	if (!plane) {
		DRM_ERROR("Invalid plane\n");
	} else if (!plane->dev || !plane->dev->dev_private) {
		/* don't access dev_private if !dev */
		DRM_ERROR("Invalid device\n");
	} else if (!property) {
		DRM_ERROR("Incoming property is NULL\n");
	} else {
		prop_array = ((struct msm_drm_private *)
				(plane->dev->dev_private))->plane_property;
		if (!prop_array)
			/* should never hit this */
			DRM_ERROR("Invalid property array\n");

		/* linear search is okay */
		for (idx = 0; idx < PLANE_PROP_COUNT; ++idx) {
			if (prop_array[idx] == property)
				break;
		}
	}

	return idx;
}

/* helper to install properties which are common to planes and crtcs */
static void _sde_plane_install_properties(struct drm_plane *plane,
		struct drm_mode_object *obj,
		struct sde_mdss_cfg *catalog)
{
	static const struct drm_prop_enum_list e_blend_op[] = {
		{SDE_DRM_BLEND_OP_NOT_DEFINED,    "not_defined"},
		{SDE_DRM_BLEND_OP_OPAQUE,         "opaque"},
		{SDE_DRM_BLEND_OP_PREMULTIPLIED,  "premultiplied"},
		{SDE_DRM_BLEND_OP_COVERAGE,       "coverage"}
	};
	static const struct drm_prop_enum_list e_src_config[] = {
		{SDE_DRM_DEINTERLACE, "deinterlace"}
	};
	struct sde_plane *psde = to_sde_plane(plane);
	struct drm_device *dev = plane->dev;
	struct msm_drm_private *dev_priv = dev->dev_private;

	DBG("");

	if (!psde || !psde->pipe_sblk || !catalog) {
		DRM_ERROR("Failed to identify catalog definition\n");
		return;
	}

	/* range properties */
	_sde_plane_install_range_property(plane, dev, "zpos", 0, 255, 1,
			&(dev_priv->plane_property[PLANE_PROP_ZPOS]));

	_sde_plane_install_range_property(plane, dev, "alpha", 0, 255, 255,
			&(dev_priv->plane_property[PLANE_PROP_ALPHA]));

	/* max range of first pipe will be used */
	_sde_plane_install_range_property(plane, dev, "h_decimate",
			0, psde->pipe_sblk->maxhdeciexp, 0,
			&(dev_priv->plane_property[PLANE_PROP_H_DECIMATE]));

	/* max range of first pipe will be used */
	_sde_plane_install_range_property(plane, dev, "v_decimate",
			0, psde->pipe_sblk->maxvdeciexp, 0,
			&(dev_priv->plane_property[PLANE_PROP_V_DECIMATE]));

	_sde_plane_install_range_property(plane, dev, "sync_fence", 0, ~0, 0,
			&(dev_priv->plane_property[PLANE_PROP_SYNC_FENCE]));

	/* standard properties */
	_sde_plane_install_rotation_property(plane, dev,
			&(dev_priv->plane_property[PLANE_PROP_ROTATION]));

		/* enum/bitmask properties */
	_sde_plane_install_enum_property(plane, dev, "blend_op", 0,
			e_blend_op, ARRAY_SIZE(e_blend_op),
			&(dev_priv->plane_property[PLANE_PROP_BLEND_OP]));
	_sde_plane_install_enum_property(plane, dev, "src_config", 1,
			e_src_config, ARRAY_SIZE(e_src_config),
			&(dev_priv->plane_property[PLANE_PROP_SRC_CONFIG]));

	/* blob properties */
	if (psde->features & SDE_SSPP_SCALER)
		_sde_plane_install_blob_property(plane, dev, "scaler",
			&(dev_priv->plane_property[PLANE_PROP_SCALER]));
	if (psde->features & BIT(SDE_SSPP_CSC))
		_sde_plane_install_blob_property(plane, dev, "csc",
				&(dev_priv->plane_property[PLANE_PROP_CSC]));
}

static int sde_plane_atomic_set_property(struct drm_plane *plane,
		struct drm_plane_state *state, struct drm_property *property,
		uint64_t val)
{
	struct sde_plane_state *pstate;
	struct drm_property_blob *blob, **pr_blob;
	int idx, ret = -EINVAL;

	DBG("");

	idx = _sde_plane_get_property_index(plane, property);
	if (!state) {
		DRM_ERROR("Invalid state\n");
	} else if (idx < PLANE_PROP_COUNT) {
		DBG("Set property %d <= %d", idx, (int)val);
		pstate = to_sde_plane_state(state);

		/* extra handling for incoming blob properties */
		if ((property->flags & DRM_MODE_PROP_BLOB) &&
			(idx < PLANE_PROP_BLOBCOUNT)) {
			/* DRM lookup also takes a reference */
			blob = drm_property_lookup_blob(plane->dev,
				(uint32_t)val);
			if (!blob) {
				DRM_ERROR("Blob not found\n");
				val = 0;
			} else {
				DBG("Blob %u saved", blob->base.id);
				val = blob->base.id;

				/* save blobs for later */
				pr_blob = &pstate->property_blobs[idx];
				/* need to clear previous reference */
				if (*pr_blob)
					drm_property_unreference_blob(*pr_blob);
				*pr_blob = blob;
			}
		}
		pstate->property_values[idx] = val;
		ret = 0;
	}

	return ret;
}

static int sde_plane_set_property(struct drm_plane *plane,
		struct drm_property *property, uint64_t val)
{
	int rc;

	DBG("");

	if (!plane)
		return -EINVAL;

	rc = sde_plane_atomic_set_property(plane, plane->state, property, val);
	return rc;
}

static int sde_plane_atomic_get_property(struct drm_plane *plane,
		const struct drm_plane_state *state,
		struct drm_property *property, uint64_t *val)
{
	struct sde_plane_state *pstate;
	int idx, ret = -EINVAL;

	DBG("");

	idx = _sde_plane_get_property_index(plane, property);
	if (!state) {
		DRM_ERROR("Invalid state\n");
	} else if (!val) {
		DRM_ERROR("Value pointer is NULL\n");
	} else if (idx < PLANE_PROP_COUNT) {
		pstate = to_sde_plane_state(state);

		*val = pstate->property_values[idx];
		DBG("Get property %d %lld", idx, *val);
		ret = 0;
	}

	return ret;
}

static void sde_plane_destroy(struct drm_plane *plane)
{
	struct sde_plane *psde;

	DBG("");

	if (plane) {
		psde = to_sde_plane(plane);

		debugfs_remove_recursive(psde->debugfs_root);

		drm_plane_helper_disable(plane);

		/* this will destroy the states as well */
		drm_plane_cleanup(plane);

		if (psde->pipe_hw)
			sde_hw_sspp_destroy(psde->pipe_hw);

		kfree(psde);
	}
}

static void sde_plane_destroy_state(struct drm_plane *plane,
		struct drm_plane_state *state)
{
	struct sde_plane_state *pstate;
	int i;

	DBG("");

	/* remove ref count for frame buffers */
	if (state->fb)
		drm_framebuffer_unreference(state->fb);

	pstate = to_sde_plane_state(state);

	/* remove ref count for blobs */
	for (i = 0; i < PLANE_PROP_BLOBCOUNT; ++i)
		if (pstate->property_blobs[i])
			drm_property_unreference_blob(
					pstate->property_blobs[i]);

	kfree(pstate);
}

static struct drm_plane_state *
sde_plane_duplicate_state(struct drm_plane *plane)
{
	struct sde_plane_state *pstate;
	int i;

	if (WARN_ON(!plane->state))
		return NULL;

	DBG("");
	pstate = kmemdup(to_sde_plane_state(plane->state),
			sizeof(*pstate), GFP_KERNEL);
	if (pstate) {
		/* add ref count for frame buffer */
		if (pstate->base.fb)
			drm_framebuffer_reference(pstate->base.fb);

		/* add ref count for blobs */
		for (i = 0; i < PLANE_PROP_BLOBCOUNT; ++i)
			if (pstate->property_blobs[i])
				drm_property_reference_blob(
						pstate->property_blobs[i]);

		pstate->mode_changed = false;
		pstate->pending = false;
	}

	return pstate ? &pstate->base : NULL;
}

static void sde_plane_reset(struct drm_plane *plane)
{
	struct sde_plane_state *pstate;

	DBG("");
	if (plane->state && plane->state->fb)
		drm_framebuffer_unreference(plane->state->fb);

	kfree(to_sde_plane_state(plane->state));
	pstate = kzalloc(sizeof(*pstate), GFP_KERNEL);

	/* assign default blend parameters */
	pstate->property_values[PLANE_PROP_ALPHA] = 255;

	if (plane->type == DRM_PLANE_TYPE_PRIMARY)
		pstate->property_values[PLANE_PROP_ZPOS] = STAGE_BASE;
	else
		pstate->property_values[PLANE_PROP_ZPOS] =
			STAGE0 + drm_plane_index(plane);

	pstate->base.plane = plane;

	plane->state = &pstate->base;
}

static const struct drm_plane_funcs sde_plane_funcs = {
		.update_plane = drm_atomic_helper_update_plane,
		.disable_plane = drm_atomic_helper_disable_plane,
		.destroy = sde_plane_destroy,
		.set_property = sde_plane_set_property,
		.atomic_set_property = sde_plane_atomic_set_property,
		.atomic_get_property = sde_plane_atomic_get_property,
		.reset = sde_plane_reset,
		.atomic_duplicate_state = sde_plane_duplicate_state,
		.atomic_destroy_state = sde_plane_destroy_state,
};

static const struct drm_plane_helper_funcs sde_plane_helper_funcs = {
		.prepare_fb = sde_plane_prepare_fb,
		.cleanup_fb = sde_plane_cleanup_fb,
		.atomic_check = sde_plane_atomic_check,
		.atomic_update = sde_plane_atomic_update,
};

enum sde_sspp sde_plane_pipe(struct drm_plane *plane)
{
	struct sde_plane *sde_plane = to_sde_plane(plane);

	return sde_plane->pipe;
}

static void _sde_plane_init_debugfs(struct sde_plane *psde, struct sde_kms *kms)
{
	const struct sde_sspp_sub_blks *sblk = 0;
	const struct sde_sspp_cfg *cfg = 0;

	if (psde && psde->pipe_hw)
		cfg = psde->pipe_hw->cap;
	if (cfg)
		sblk = cfg->sblk;

	if (kms && sblk) {
		/* create overall sub-directory for the pipe */
		psde->debugfs_root =
			debugfs_create_dir(psde->pipe_name,
					sde_debugfs_get_root(kms));
		if (psde->debugfs_root) {
			/* don't error check these */
			debugfs_create_x32("features", 0644,
					psde->debugfs_root, &psde->features);

			/* add register dump support */
			sde_debugfs_setup_regset32(&psde->debugfs_src,
					sblk->src_blk.base + cfg->base,
					sblk->src_blk.len,
					kms->mmio);
			sde_debugfs_create_regset32("src_blk", 0444,
					psde->debugfs_root, &psde->debugfs_src);

			sde_debugfs_setup_regset32(&psde->debugfs_scaler,
					sblk->scaler_blk.base + cfg->base,
					sblk->scaler_blk.len,
					kms->mmio);
			sde_debugfs_create_regset32("scaler_blk", 0444,
					psde->debugfs_root,
					&psde->debugfs_scaler);

			sde_debugfs_setup_regset32(&psde->debugfs_csc,
					sblk->csc_blk.base + cfg->base,
					sblk->csc_blk.len,
					kms->mmio);
			sde_debugfs_create_regset32("csc_blk", 0444,
					psde->debugfs_root, &psde->debugfs_csc);
		}
	}
}

/* initialize plane */
struct drm_plane *sde_plane_init(struct drm_device *dev,
		uint32_t pipe, bool primary_plane)
{
	struct drm_plane *plane = NULL;
	struct sde_plane *psde;
	struct msm_drm_private *priv;
	struct sde_kms *kms;
	enum drm_plane_type type;
	int ret = -EINVAL;

	if (!dev) {
		DRM_ERROR("[%u]Device is NULL\n", pipe);
		goto exit;
	}

	priv = dev->dev_private;
	if (!priv) {
		DRM_ERROR("[%u]Private data is NULL\n", pipe);
		goto exit;
	}

	if (!priv->kms) {
		DRM_ERROR("[%u]Invalid KMS reference\n", pipe);
		goto exit;
	}
	kms = to_sde_kms(priv->kms);

	if (!kms->catalog) {
		DRM_ERROR("[%u]Invalid catalog reference\n", pipe);
		goto exit;
	}

	/* create and zero local structure */
	psde = kzalloc(sizeof(*psde), GFP_KERNEL);
	if (!psde) {
		DRM_ERROR("[%u]Failed to allocate local plane struct\n", pipe);
		ret = -ENOMEM;
		goto exit;
	}

	/* cache local stuff for later */
	plane = &psde->base;
	psde->pipe = pipe;
	psde->mmu_id = kms->mmu_id;

	/* initialize underlying h/w driver */
	psde->pipe_hw = sde_hw_sspp_init(pipe, kms->mmio, kms->catalog);
	if (IS_ERR(psde->pipe_hw)) {
		DRM_ERROR("[%u]SSPP init failed\n", pipe);
		ret = PTR_ERR(psde->pipe_hw);
		goto clean_plane;
	} else if (!psde->pipe_hw->cap || !psde->pipe_hw->cap->sblk) {
		DRM_ERROR("[%u]SSPP init returned invalid cfg\n", pipe);
		goto clean_sspp;
	}

	/* cache features mask for later */
	psde->features = psde->pipe_hw->cap->features;
	psde->pipe_sblk = psde->pipe_hw->cap->sblk;

	/* add plane to DRM framework */
	psde->nformats = mdp_get_formats(psde->formats,
		ARRAY_SIZE(psde->formats),
		!(psde->features & BIT(SDE_SSPP_CSC)) ||
		!(psde->features & SDE_SSPP_SCALER));

	if (!psde->nformats) {
		DRM_ERROR("[%u]No valid formats for plane\n", pipe);
		goto clean_sspp;
	}

	if (psde->features & BIT(SDE_SSPP_CURSOR))
		type = DRM_PLANE_TYPE_CURSOR;
	else if (primary_plane)
		type = DRM_PLANE_TYPE_PRIMARY;
	else
		type = DRM_PLANE_TYPE_OVERLAY;
	ret = drm_universal_plane_init(dev, plane, 0xff, &sde_plane_funcs,
				psde->formats, psde->nformats,
				type);
	if (ret)
		goto clean_sspp;

	/* success! finalize initialization */
	drm_plane_helper_add(plane, &sde_plane_helper_funcs);

	_sde_plane_install_properties(plane, &plane->base, kms->catalog);

	/* save user friendly pipe name for later */
	snprintf(psde->pipe_name, SDE_NAME_SIZE, "plane%u", plane->base.id);

	_sde_plane_init_debugfs(psde, kms);

	DRM_INFO("[%u]Successfully created %s\n", pipe, psde->pipe_name);
	return plane;

clean_sspp:
	if (psde && psde->pipe_hw)
		sde_hw_sspp_destroy(psde->pipe_hw);
clean_plane:
	kfree(psde);
exit:
	return ERR_PTR(ret);
}
