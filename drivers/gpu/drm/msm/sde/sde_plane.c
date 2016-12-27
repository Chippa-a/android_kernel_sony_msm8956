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
#include "sde_fence.h"
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

#define SDE_STATE_CACHE_SIZE	2

struct sde_plane {
	struct drm_plane base;

	int mmu_id;

	struct mutex lock;

	enum sde_sspp pipe;
	uint32_t features;      /* capabilities from catalog */
	uint32_t nformats;
	uint32_t formats[32];

	struct sde_hw_pipe *pipe_hw;
	struct sde_hw_pipe_cfg pipe_cfg;
	struct sde_hw_pixel_ext pixel_ext;
	struct sde_hw_sharp_cfg sharp_cfg;
	struct sde_hw_scaler3_cfg scaler3_cfg;

	struct sde_csc_cfg csc_cfg;
	struct sde_csc_cfg *csc_ptr;

	const struct sde_sspp_sub_blks *pipe_sblk;

	char pipe_name[SDE_NAME_SIZE];

	/* cache property default values (for reset) */
	uint64_t property_defaults[PLANE_PROP_COUNT];

	/* cache for unused plane state structures */
	struct sde_plane_state *state_cache[SDE_STATE_CACHE_SIZE];
	int state_cache_size;

	/* debugfs related stuff */
	struct dentry *debugfs_root;
	struct sde_debugfs_regset32 debugfs_src;
	struct sde_debugfs_regset32 debugfs_scaler;
	struct sde_debugfs_regset32 debugfs_csc;
};
#define to_sde_plane(x) container_of(x, struct sde_plane, base)

static bool sde_plane_enabled(struct drm_plane_state *state)
{
	return state && state->fb && state->crtc;
}

/* helper to update a state's sync fence pointer from the property */
static void _sde_plane_update_sync_fence(struct drm_plane *plane,
		struct sde_plane_state *pstate, uint64_t fd)
{
	if (!plane || !pstate)
		return;

	/* clear previous reference */
	if (pstate->sync_fence)
		sde_sync_put(pstate->sync_fence);

	/* get fence pointer for later */
	pstate->sync_fence = sde_sync_get(fd);

	DBG("0x%llX", fd);
}

int sde_plane_wait_sync_fence(struct drm_plane *plane)
{
	struct sde_plane_state *pstate;
	void *sync_fence;
	long wait_ms;
	int ret = -EINVAL;

	if (!plane) {
		DRM_ERROR("Invalid plane\n");
	} else if (!plane->state) {
		DRM_ERROR("Invalid plane state\n");
	} else {
		pstate = to_sde_plane_state(plane->state);
		sync_fence = pstate->sync_fence;

		if (sync_fence) {
			wait_ms = (long)sde_plane_get_property(pstate,
					PLANE_PROP_SYNC_FENCE_TIMEOUT);

			DBG("%s", to_sde_plane(plane)->pipe_name);
			ret = sde_sync_wait(sync_fence, wait_ms);
			if (!ret)
				DBG("signaled");
			else if (ret == -ETIME)
				DRM_ERROR("timeout\n");
			else
				DRM_ERROR("error %d\n", ret);
		} else {
			ret = 0;
		}
	}
	return ret;
}

static void _sde_plane_set_scanout(struct drm_plane *plane,
		struct sde_plane_state *pstate,
		struct sde_hw_pipe_cfg *pipe_cfg, struct drm_framebuffer *fb)
{
	struct sde_plane *psde;
	unsigned int shift;
	int i;

	if (!plane || !pstate || !pipe_cfg || !fb)
		return;

	psde = to_sde_plane(plane);

	if (psde->pipe_hw && psde->pipe_hw->ops.setup_sourceaddress) {
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

static void _sde_plane_setup_scaler3(struct sde_plane *psde,
		uint32_t src_w, uint32_t src_h, uint32_t dst_w, uint32_t dst_h,
		struct sde_hw_scaler3_cfg *scale_cfg,
		const struct sde_format *fmt,
		uint32_t chroma_subsmpl_h, uint32_t chroma_subsmpl_v)
{
}

/**
 * _sde_plane_setup_scaler2(): Determine default scaler phase steps/filter type
 * @psde: Pointer to SDE plane object
 * @src: Source size
 * @dst: Destination size
 * @phase_steps: Pointer to output array for phase steps
 * @filter: Pointer to output array for filter type
 * @fmt: Pointer to format definition
 * @chroma_subsampling: Subsampling amount for chroma channel
 *
 * Returns: 0 on success
 */
static int _sde_plane_setup_scaler2(struct sde_plane *psde,
		uint32_t src, uint32_t dst, uint32_t *phase_steps,
		enum sde_hw_filter *filter, const struct sde_format *fmt,
		uint32_t chroma_subsampling)
{
	if (!psde || !phase_steps || !filter || !fmt) {
		DRM_ERROR("Invalid arguments\n");
		return -EINVAL;
	}

	/* calculate phase steps, leave init phase as zero */
	phase_steps[SDE_SSPP_COMP_0] =
		mult_frac(1 << PHASE_STEP_SHIFT, src, dst);
	phase_steps[SDE_SSPP_COMP_1_2] =
		phase_steps[SDE_SSPP_COMP_0] / chroma_subsampling;
	phase_steps[SDE_SSPP_COMP_2] = phase_steps[SDE_SSPP_COMP_1_2];
	phase_steps[SDE_SSPP_COMP_3] = phase_steps[SDE_SSPP_COMP_0];

	/* calculate scaler config, if necessary */
	if (SDE_FORMAT_IS_YUV(fmt) || src != dst) {
		filter[SDE_SSPP_COMP_3] =
			(src <= dst) ? SDE_SCALE_FILTER_BIL :
			SDE_SCALE_FILTER_PCMN;

		if (SDE_FORMAT_IS_YUV(fmt)) {
			filter[SDE_SSPP_COMP_0] = SDE_SCALE_FILTER_CA;
			filter[SDE_SSPP_COMP_1_2] = filter[SDE_SSPP_COMP_3];
		} else {
			filter[SDE_SSPP_COMP_0] = filter[SDE_SSPP_COMP_3];
			filter[SDE_SSPP_COMP_1_2] =
				SDE_SCALE_FILTER_NEAREST;
		}
	} else {
		/* disable scaler */
		DBG("Disable scaler");
		filter[SDE_SSPP_COMP_0] = SDE_SCALE_FILTER_MAX;
		filter[SDE_SSPP_COMP_1_2] = SDE_SCALE_FILTER_MAX;
		filter[SDE_SSPP_COMP_3] = SDE_SCALE_FILTER_MAX;
	}
	return 0;
}

/**
 * _sde_plane_setup_pixel_ext - determine default pixel extension values
 * @psde: Pointer to SDE plane object
 * @src: Source size
 * @dst: Destination size
 * @decimated_src: Source size after decimation, if any
 * @phase_steps: Pointer to output array for phase steps
 * @out_src: Output array for pixel extension values
 * @out_edge1: Output array for pixel extension first edge
 * @out_edge2: Output array for pixel extension second edge
 * @filter: Pointer to array for filter type
 * @fmt: Pointer to format definition
 * @chroma_subsampling: Subsampling amount for chroma channel
 * @post_compare: Whether to chroma subsampled source size for comparisions
 */
static void _sde_plane_setup_pixel_ext(struct sde_plane *psde,
		uint32_t src, uint32_t dst, uint32_t decimated_src,
		uint32_t *phase_steps, uint32_t *out_src, int *out_edge1,
		int *out_edge2, enum sde_hw_filter *filter,
		const struct sde_format *fmt, uint32_t chroma_subsampling,
		bool post_compare)
{
	int64_t edge1, edge2, caf;
	uint32_t src_work;
	int i, tmp;

	if (psde && phase_steps && out_src && out_edge1 &&
			out_edge2 && filter && fmt) {
		/* handle CAF for YUV formats */
		if (SDE_FORMAT_IS_YUV(fmt) && *filter == SDE_SCALE_FILTER_CA)
			caf = PHASE_STEP_UNIT_SCALE;
		else
			caf = 0;

		for (i = 0; i < SDE_MAX_PLANES; i++) {
			src_work = decimated_src;
			if (i == SDE_SSPP_COMP_1_2 || i == SDE_SSPP_COMP_2)
				src_work /= chroma_subsampling;
			if (post_compare)
				src = src_work;
			if (!SDE_FORMAT_IS_YUV(fmt) && (src == dst)) {
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
		const struct sde_format *fmt)
{
	static const struct sde_csc_cfg sde_csc_YUV2RGB_601L = {
		{
			/* S15.16 format */
			0x00012A00, 0x00000000, 0x00019880,
			0x00012A00, 0xFFFF9B80, 0xFFFF3000,
			0x00012A00, 0x00020480, 0x00000000,
		},
		/* signed bias */
		{ 0xfff0, 0xff80, 0xff80,},
		{ 0x0, 0x0, 0x0,},
		/* unsigned clamp */
		{ 0x10, 0xeb, 0x10, 0xf0, 0x10, 0xf0,},
		{ 0x00, 0xff, 0x00, 0xff, 0x00, 0xff,},
	};
	static const struct sde_csc_cfg sde_csc_NOP = {
		{
			/* identity matrix, S15.16 format */
			0x10000, 0x00000, 0x00000,
			0x00000, 0x10000, 0x00000,
			0x00000, 0x00000, 0x10000,
		},
		/* signed bias */
		{ 0x0, 0x0, 0x0,},
		{ 0x0, 0x0, 0x0,},
		/* unsigned clamp */
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
						csc->v1.ctm_coeff[i] >> 16;
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
		psde->csc_ptr = &psde->csc_cfg;
	/* revert to kernel default */
	} else if (SDE_FORMAT_IS_YUV(fmt)) {
		psde->csc_ptr = (struct sde_csc_cfg *)&sde_csc_YUV2RGB_601L;
	} else {
		psde->csc_ptr = (struct sde_csc_cfg *)&sde_csc_NOP;
	}

	psde->pipe_hw->ops.setup_csc(psde->pipe_hw, psde->csc_ptr);
}

static void _sde_plane_setup_scaler(struct sde_plane *psde,
		const struct sde_format *fmt,
		struct sde_plane_state *pstate)
{
	struct sde_hw_pixel_ext *pe = NULL;
	struct sde_drm_scaler *sc_u = NULL;
	struct sde_drm_scaler_v1 *sc_u1 = NULL;
	size_t sc_u_size = 0;
	uint32_t chroma_subsmpl_h, chroma_subsmpl_v;
	uint32_t tmp;
	int i;

	if (!psde || !fmt)
		return;

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

	/* decimation */
	if (sc_u1 && (sc_u1->enable & SDE_DRM_SCALER_DECIMATE)) {
		psde->pipe_cfg.horz_decimation = sc_u1->horz_decimate;
		psde->pipe_cfg.vert_decimation = sc_u1->vert_decimate;
	} else {
		psde->pipe_cfg.horz_decimation = 0;
		psde->pipe_cfg.vert_decimation = 0;
	}

	/* don't chroma subsample if decimating */
	chroma_subsmpl_h = psde->pipe_cfg.horz_decimation ? 1 :
		drm_format_horz_chroma_subsampling(fmt->base.pixel_format);
	chroma_subsmpl_v = psde->pipe_cfg.vert_decimation ? 1 :
		drm_format_vert_chroma_subsampling(fmt->base.pixel_format);

	/* update scaler */
	if (psde->features & BIT(SDE_SSPP_SCALER_QSEED3)) {
		if (sc_u1 && (sc_u1->enable & SDE_DRM_SCALER_SCALER_3))
			DBG("SCALER3 blob detected");
		else
			_sde_plane_setup_scaler3(psde,
					psde->pipe_cfg.src_rect.w,
					psde->pipe_cfg.src_rect.h,
					psde->pipe_cfg.dst_rect.w,
					psde->pipe_cfg.dst_rect.h,
					&psde->scaler3_cfg, fmt,
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
			_sde_plane_setup_scaler2(psde,
					psde->pipe_cfg.src_rect.w,
					psde->pipe_cfg.dst_rect.w,
					pe->phase_step_x,
					pe->horz_filter, fmt, chroma_subsmpl_h);
			_sde_plane_setup_scaler2(psde,
					psde->pipe_cfg.src_rect.h,
					psde->pipe_cfg.dst_rect.h,
					pe->phase_step_y,
					pe->vert_filter, fmt, chroma_subsmpl_v);
		}
	}

	/* update pixel extensions */
	if (sc_u1 && (sc_u1->enable & SDE_DRM_SCALER_PIX_EXT)) {
		/* populate from user space */
		DBG("PIXEXT blob detected");
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
		tmp = DECIMATED_DIMENSION(psde->pipe_cfg.src_rect.w,
				psde->pipe_cfg.horz_decimation);
		if (SDE_FORMAT_IS_YUV(fmt))
			tmp &= ~0x1;
		_sde_plane_setup_pixel_ext(psde, psde->pipe_cfg.src_rect.w,
				psde->pipe_cfg.dst_rect.w, tmp,
				pe->phase_step_x,
				pe->roi_w,
				pe->num_ext_pxls_left,
				pe->num_ext_pxls_right, pe->horz_filter, fmt,
				chroma_subsmpl_h, 0);

		tmp = DECIMATED_DIMENSION(psde->pipe_cfg.src_rect.h,
				psde->pipe_cfg.vert_decimation);
		_sde_plane_setup_pixel_ext(psde, psde->pipe_cfg.src_rect.h,
				psde->pipe_cfg.dst_rect.h, tmp,
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
}

int sde_plane_color_fill(struct drm_plane *plane,
		uint32_t color, uint32_t alpha)
{
	struct sde_plane *psde;
	const struct sde_format *fmt;

	if (!plane) {
		DRM_ERROR("Invalid plane\n");
		return -EINVAL;
	}

	psde = to_sde_plane(plane);
	if (!psde->pipe_hw) {
		DRM_ERROR("Invalid plane h/w pointer\n");
		return -EINVAL;
	}

	/*
	 * select fill format to match user property expectation,
	 * h/w only supports RGB variants
	 */
	fmt = sde_get_sde_format(DRM_FORMAT_ABGR8888);

	/* update sspp */
	if (fmt && psde->pipe_hw->ops.setup_solidfill) {
		psde->pipe_hw->ops.setup_solidfill(psde->pipe_hw,
				(color & 0xFFFFFF) | ((alpha & 0xFF) << 24));

		/* override scaler/decimation if solid fill */
		psde->pipe_cfg.src_rect.x = 0;
		psde->pipe_cfg.src_rect.y = 0;
		psde->pipe_cfg.src_rect.w = psde->pipe_cfg.dst_rect.w;
		psde->pipe_cfg.src_rect.h = psde->pipe_cfg.dst_rect.h;

		_sde_plane_setup_scaler(psde, fmt, 0);

		if (psde->pipe_hw->ops.setup_format)
			psde->pipe_hw->ops.setup_format(psde->pipe_hw,
					fmt, SDE_SSPP_SOLID_FILL);

		if (psde->pipe_hw->ops.setup_rects)
			psde->pipe_hw->ops.setup_rects(psde->pipe_hw,
					&psde->pipe_cfg, &psde->pixel_ext);
	}

	return 0;
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
	uint32_t nplanes, color_fill;
	uint32_t src_flags;
	const struct sde_format *fmt;

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

	fmt = to_sde_format(msm_framebuffer_format(fb));

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
	src_flags = 0;

	psde->pipe_cfg.src.format = fmt;
	psde->pipe_cfg.src.width = fb->width;
	psde->pipe_cfg.src.height = fb->height;
	psde->pipe_cfg.src.num_planes = nplanes;

	/* flags */
	DBG("Flags 0x%llX, rotation 0x%llX",
			sde_plane_get_property(pstate, PLANE_PROP_SRC_CONFIG),
			sde_plane_get_property(pstate, PLANE_PROP_ROTATION));
	if (sde_plane_get_property(pstate, PLANE_PROP_ROTATION) &
		BIT(DRM_REFLECT_X))
		src_flags |= SDE_SSPP_FLIP_LR;
	if (sde_plane_get_property(pstate, PLANE_PROP_ROTATION) &
		BIT(DRM_REFLECT_Y))
		src_flags |= SDE_SSPP_FLIP_UD;
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

	/* check for color fill */
	color_fill = (uint32_t)sde_plane_get_property(pstate,
			PLANE_PROP_COLOR_FILL);
	if (color_fill & BIT(31)) {
		/* force 100% alpha, stop other processing */
		return sde_plane_color_fill(plane, color_fill, 0xFF);
	}

	_sde_plane_set_scanout(plane, pstate, &psde->pipe_cfg, fb);

	_sde_plane_setup_scaler(psde, fmt, pstate);

	if (psde->pipe_hw->ops.setup_format)
		psde->pipe_hw->ops.setup_format(psde->pipe_hw,
				fmt, src_flags);
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
	if (SDE_FORMAT_IS_YUV(fmt))
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

	DBG("%s: FB[%u]", psde->pipe_name, fb->base.id);
	return msm_framebuffer_prepare(fb, psde->mmu_id);
}

static void sde_plane_cleanup_fb(struct drm_plane *plane,
		const struct drm_plane_state *old_state)
{
	struct drm_framebuffer *fb = old_state->fb;
	struct sde_plane *psde = to_sde_plane(plane);

	if (!fb)
		return;

	DBG("%s: FB[%u]", psde->pipe_name, fb->base.id);
	msm_framebuffer_cleanup(fb, psde->mmu_id);
}

static int _sde_plane_atomic_check_fb(struct sde_plane *psde,
		struct sde_plane_state *pstate,
		struct drm_framebuffer *fb)
{
	return 0;
}

static int sde_plane_atomic_check(struct drm_plane *plane,
		struct drm_plane_state *state)
{
	struct sde_plane *psde;
	struct sde_plane_state *pstate;
	struct drm_plane_state *old_state;
	const struct sde_format *fmt;
	size_t sc_u_size = 0;
	struct sde_drm_scaler *sc_u = NULL;
	int ret = 0;

	uint32_t src_x, src_y;
	uint32_t src_w, src_h;
	uint32_t deci_w, deci_h, src_deci_w, src_deci_h;
	uint32_t src_max_x, src_max_y, src_max_w, src_max_h;
	uint32_t upscale_max, downscale_max;

	DBG();

	if (!plane || !state) {
		DRM_ERROR("Invalid plane/state\n");
		ret = -EINVAL;
		goto exit;
	}

	psde = to_sde_plane(plane);
	pstate = to_sde_plane_state(state);
	old_state = plane->state;

	if (!psde->pipe_sblk) {
		DRM_ERROR("Invalid plane catalog\n");
		ret = -EINVAL;
		goto exit;
	}

	/* get decimation config from user space */
	deci_w = 0;
	deci_h = 0;
	sc_u = _sde_plane_get_blob(pstate, PLANE_PROP_SCALER, &sc_u_size);
	if (sc_u) {
		switch (sc_u->version) {
		case SDE_DRM_SCALER_V1:
			if (!_sde_plane_verify_blob(sc_u,
					sc_u_size,
					&sc_u->v1,
					sizeof(struct sde_drm_scaler_v1))) {
				deci_w = sc_u->v1.horz_decimate;
				deci_h = sc_u->v1.vert_decimate;
			}
			break;
		default:
			DBG("Unrecognized scaler blob v%lld", sc_u->version);
			break;
		}
	}

	/* src values are in Q16 fixed point, convert to integer */
	src_x = state->src_x >> 16;
	src_y = state->src_y >> 16;
	src_w = state->src_w >> 16;
	src_h = state->src_h >> 16;

	src_deci_w = DECIMATED_DIMENSION(src_w, deci_w);
	src_deci_h = DECIMATED_DIMENSION(src_h, deci_h);

	src_max_x = 0xFFFF;
	src_max_y = 0xFFFF;
	src_max_w = 0x3FFF;
	src_max_h = 0x3FFF;
	upscale_max   = psde->pipe_sblk->maxupscale;
	downscale_max = psde->pipe_sblk->maxdwnscale;

	/*
	 * Including checks from mdss
	 * - mdss_mdp_overlay_req_check()
	 */
	DBG("%s: check (%d -> %d)", psde->pipe_name,
			sde_plane_enabled(old_state), sde_plane_enabled(state));

	if (sde_plane_enabled(state)) {
		/* determine SDE format definition. State's fb is valid here. */
		fmt = to_sde_format(msm_framebuffer_format(state->fb));

		/* don't check for other errors after first failure */
		if (SDE_FORMAT_IS_YUV(fmt) &&
			(!(psde->features & SDE_SSPP_SCALER) ||
			 !(psde->features & BIT(SDE_SSPP_CSC)))) {
			DRM_ERROR("Pipe doesn't support YUV\n");
			ret = -EINVAL;

		/* verify source size/region */
		} else if (!src_w || !src_h ||
			(src_w > src_max_w) || (src_h > src_max_h) ||
			(src_x > src_max_x) || (src_y > src_max_y) ||
			(src_x + src_w > src_max_x) ||
			(src_y + src_h > src_max_y)) {
			DRM_ERROR("Invalid source (%u, %u) -> (%u, %u)\n",
					src_x, src_y, src_x + src_w,
					src_y + src_h);
			ret = -EINVAL;

		/* require even source for YUV */
		} else if (SDE_FORMAT_IS_YUV(fmt) &&
				((src_x & 0x1) || (src_y & 0x1) ||
				 (src_w & 0x1) || (src_h & 0x1))) {
			DRM_ERROR("Invalid odd src res/pos for YUV\n");
			ret = -EINVAL;

		/* verify scaler requirements */
		} else if (!(psde->features & SDE_SSPP_SCALER) &&
			((src_w != state->crtc_w) ||
			 (src_h != state->crtc_h))) {
			DRM_ERROR("Pipe doesn't support scaling %ux%u->%ux%u\n",
					src_w, src_h, state->crtc_w,
					state->crtc_h);
			ret = -EINVAL;

		/* check decimated source width */
		} else if (src_deci_w > psde->pipe_sblk->maxlinewidth) {
			DRM_ERROR("Invalid source [W:%u, Wd:%u] > %u\n",
					src_w, src_deci_w,
					psde->pipe_sblk->maxlinewidth);
			ret = -EINVAL;

		/* check max scaler capability */
		} else if (((src_deci_w * upscale_max) < state->crtc_w) ||
			((src_deci_h * upscale_max) < state->crtc_h) ||
			((state->crtc_w * downscale_max) < src_deci_w) ||
			((state->crtc_h * downscale_max) < src_deci_h)) {
			DRM_ERROR("Too much scaling requested %ux%u -> %ux%u\n",
					src_deci_w, src_deci_h,
					state->crtc_w, state->crtc_h);
			ret = -EINVAL;

		/* check frame buffer */
		} else if (_sde_plane_atomic_check_fb(
				psde, pstate, state->fb)) {
			ret = -EINVAL;
		}

		/* check decimation (and bwc/fetch mode) */
		if (!ret && (deci_w || deci_h)) {
			if (SDE_FORMAT_IS_UBWC(fmt)) {
				DRM_ERROR("No decimation with BWC\n");
				ret = -EINVAL;
			} else if ((deci_w > psde->pipe_sblk->maxhdeciexp) ||
				(deci_h > psde->pipe_sblk->maxvdeciexp)) {
				DRM_ERROR("Too much decimation requested\n");
				ret = -EINVAL;
			} else if (fmt->fetch_mode != SDE_FETCH_LINEAR) {
				DRM_ERROR("Decimation requires linear fetch\n");
				ret = -EINVAL;
			}
		}
	}

	if (!ret) {
		if (sde_plane_enabled(state) &&
			sde_plane_enabled(old_state)) {
			bool full_modeset = false;

			if (state->fb->pixel_format !=
				old_state->fb->pixel_format) {
				DBG("%s: format change!", psde->pipe_name);
				full_modeset = true;
			}
			if (state->src_w != old_state->src_w ||
				state->src_h != old_state->src_h) {
				DBG("%s: src_w change!", psde->pipe_name);
				full_modeset = true;
			}
			if (to_sde_plane_state(old_state)->pending) {
				DBG("%s: still pending!", psde->pipe_name);
				full_modeset = true;
			}
			if (full_modeset)
				to_sde_plane_state(state)->mode_changed = true;

		} else {
			to_sde_plane_state(state)->mode_changed = true;
		}
	}

exit:
	return ret;
}

void sde_plane_complete_flip(struct drm_plane *plane)
{
	if (plane && plane->state)
		to_sde_plane_state(plane->state)->pending = false;
}

static void sde_plane_atomic_update(struct drm_plane *plane,
				struct drm_plane_state *old_state)
{
	struct sde_plane *sde_plane;
	struct drm_plane_state *state;
	struct sde_plane_state *pstate;

	if (!plane || !plane->state) {
		DRM_ERROR("Invalid plane/state\n");
		return;
	}

	sde_plane = to_sde_plane(plane);
	state = plane->state;
	pstate = to_sde_plane_state(state);

	DBG("%s: update", sde_plane->pipe_name);

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

static inline struct drm_property **_sde_plane_get_property_entry(
		struct drm_device *dev, enum msm_mdp_plane_property property)
{
	struct msm_drm_private *priv;

	if (!dev  || !dev->dev_private || (property >= PLANE_PROP_COUNT))
		return NULL;

	priv = dev->dev_private;

	return &(priv->plane_property[property]);
}

static void _sde_plane_install_range_property(struct drm_plane *plane,
		struct drm_device *dev, const char *name,
		uint64_t min, uint64_t max, uint64_t init,
		enum msm_mdp_plane_property property)
{
	struct drm_property **prop;

	prop = _sde_plane_get_property_entry(dev, property);
	if (plane && name && prop) {
		/* only create the property once */
		if (*prop == 0) {
			*prop = drm_property_create_range(dev,
					0 /* flags */, name, min, max);
			if (*prop == 0)
				DRM_ERROR("Create %s property failed\n", name);
		}

		/* save init value for later */
		to_sde_plane(plane)->property_defaults[property] = init;

		/* always attach property, if created */
		if (*prop)
			drm_object_attach_property(&plane->base, *prop, init);
	}
}

static void _sde_plane_install_rotation_property(struct drm_plane *plane,
		struct drm_device *dev, enum msm_mdp_plane_property property)
{
	struct sde_plane *psde;
	struct drm_property **prop;

	prop = _sde_plane_get_property_entry(dev, property);
	if (plane && prop) {
		/* only create the property once */
		if (*prop == 0) {
			*prop = drm_mode_create_rotation_property(dev,
					BIT(DRM_REFLECT_X) |
					BIT(DRM_REFLECT_Y));
			if (*prop == 0)
				DRM_ERROR("Create rotation property failed\n");
		}

		/* save init value for later */
		psde = to_sde_plane(plane);
		psde->property_defaults[property] = 0;

		/* always attach property, if created */
		if (*prop)
			drm_object_attach_property(&plane->base, *prop,
					psde->property_defaults[property]);
	}
}

static void _sde_plane_install_enum_property(struct drm_plane *plane,
		struct drm_device *dev, const char *name, int is_bitmask,
		const struct drm_prop_enum_list *values, int num_values,
		enum msm_mdp_plane_property property)
{
	struct sde_plane *psde;
	struct drm_property **prop;

	prop = _sde_plane_get_property_entry(dev, property);
	if (plane && name && prop && values && num_values) {
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

		/* save init value for later */
		psde = to_sde_plane(plane);
		psde->property_defaults[property] = 0;

		/* always attach property, if created */
		if (*prop)
			drm_object_attach_property(&plane->base, *prop,
					psde->property_defaults[property]);
	}
}

static void _sde_plane_install_blob_property(struct drm_plane *plane,
		struct drm_device *dev, const char *name,
		enum msm_mdp_plane_property property)
{
	struct sde_plane *psde;
	struct drm_property **prop;

	prop = _sde_plane_get_property_entry(dev, property);
	if (plane && name && prop && (property < PLANE_PROP_BLOBCOUNT)) {
		/* only create the property once */
		if (*prop == 0) {
			/* use 'create' for blob property place holder */
			*prop = drm_property_create(dev,
					DRM_MODE_PROP_BLOB, name, 0);
			if (*prop == 0)
				DRM_ERROR("Create %s property failed\n", name);
		}

		/* save init value for later */
		psde = to_sde_plane(plane);
		psde->property_defaults[property] = 0;

		/* always attach property, if created */
		if (*prop)
			drm_object_attach_property(&plane->base, *prop,
					psde->property_defaults[property]);
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
		prop_array = _sde_plane_get_property_entry(plane->dev, 0);
		if (!prop_array)
			/* should never hit this */
			DRM_ERROR("Invalid property array\n");

		/* linear search is okay */
		for (idx = 0; idx < PLANE_PROP_COUNT; ++idx) {
			if (prop_array[idx] == property)
				break;
		}

		if (idx == PLANE_PROP_COUNT)
			DRM_ERROR("Invalid property pointer\n");
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

	DBG("");

	if (!psde || !psde->pipe_hw || !psde->pipe_sblk || !catalog) {
		DRM_ERROR("Catalog or h/w driver definition error\n");
		return;
	}

	/* range properties */
	_sde_plane_install_range_property(plane, dev, "zpos", 0, 255,
			plane->type == DRM_PLANE_TYPE_PRIMARY ?
				STAGE_BASE : STAGE0 + drm_plane_index(plane),
			PLANE_PROP_ZPOS);

	_sde_plane_install_range_property(plane, dev, "alpha", 0, 255, 255,
			PLANE_PROP_ALPHA);

	if (psde->pipe_hw->ops.setup_solidfill)
		_sde_plane_install_range_property(plane, dev, "color_fill",
				0, 0xFFFFFFFF, 0,
				PLANE_PROP_COLOR_FILL);

	_sde_plane_install_range_property(plane, dev, "sync_fence", 0, ~0, ~0,
			PLANE_PROP_SYNC_FENCE);

	_sde_plane_install_range_property(plane, dev, "sync_fence_timeout",
			0, ~0, 10000,
			PLANE_PROP_SYNC_FENCE_TIMEOUT);

	/* standard properties */
	_sde_plane_install_rotation_property(plane, dev, PLANE_PROP_ROTATION);

	/* enum/bitmask properties */
	_sde_plane_install_enum_property(plane, dev, "blend_op", 0,
			e_blend_op, ARRAY_SIZE(e_blend_op),
			PLANE_PROP_BLEND_OP);
	_sde_plane_install_enum_property(plane, dev, "src_config", 1,
			e_src_config, ARRAY_SIZE(e_src_config),
			PLANE_PROP_SRC_CONFIG);

	/* blob properties */
	if (psde->features & SDE_SSPP_SCALER)
		_sde_plane_install_blob_property(plane, dev, "scaler",
			PLANE_PROP_SCALER);
	if (psde->features & BIT(SDE_SSPP_CSC))
		_sde_plane_install_blob_property(plane, dev, "csc",
			PLANE_PROP_CSC);
}

static int sde_plane_atomic_set_property(struct drm_plane *plane,
		struct drm_plane_state *state, struct drm_property *property,
		uint64_t val)
{
	struct sde_plane *psde;
	struct sde_plane_state *pstate;
	struct drm_property_blob *blob, **pr_blob;
	int idx, ret = -EINVAL;

	idx = _sde_plane_get_property_index(plane, property);
	if (!state) {
		DRM_ERROR("Invalid state\n");
	} else if (idx >= PLANE_PROP_COUNT) {
		DRM_ERROR("Invalid property\n");
	} else {
		psde = to_sde_plane(plane);
		pstate = to_sde_plane_state(state);

		DBG("%s: pipe %d, prop %s, val %d", psde->pipe_name,
				sde_plane_pipe(plane),
				property->name, (int)val);

		/* extra handling for incoming properties */
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
		} else if (idx == PLANE_PROP_SYNC_FENCE) {
			_sde_plane_update_sync_fence(plane, pstate, val);
		}
		pstate->property_values[idx] = val;
		ret = 0;
	}

	return ret;
}

static int sde_plane_set_property(struct drm_plane *plane,
		struct drm_property *property, uint64_t val)
{
	DBG("");

	if (!plane)
		return -EINVAL;

	return sde_plane_atomic_set_property(plane,
			plane->state, property, val);
}

static int sde_plane_atomic_get_property(struct drm_plane *plane,
		const struct drm_plane_state *state,
		struct drm_property *property, uint64_t *val)
{
	struct sde_plane_state *pstate;
	int idx, ret = -EINVAL;

	idx = _sde_plane_get_property_index(plane, property);
	if (!state) {
		DRM_ERROR("Invalid state\n");
	} else if (!val) {
		DRM_ERROR("Value pointer is NULL\n");
	} else if (idx < PLANE_PROP_COUNT) {
		pstate = to_sde_plane_state(state);

		*val = pstate->property_values[idx];
		DBG("%d 0x%llX", idx, *val);
		ret = 0;
	}

	return ret;
}

static struct sde_plane_state *sde_plane_alloc_state(struct drm_plane *plane)
{
	struct sde_plane *psde;
	struct sde_plane_state *pstate;

	if (!plane)
		return NULL;

	psde = to_sde_plane(plane);
	pstate = NULL;

	mutex_lock(&psde->lock);
	if (psde->state_cache_size)
		pstate = psde->state_cache[--(psde->state_cache_size)];
	mutex_unlock(&psde->lock);

	if (!pstate)
		pstate = kmalloc(sizeof(struct sde_plane_state), GFP_KERNEL);

	return pstate;
}

static void sde_plane_free_state(struct drm_plane *plane,
		struct sde_plane_state *pstate)
{
	struct sde_plane *psde;

	if (!plane || !pstate)
		return;

	psde = to_sde_plane(plane);

	mutex_lock(&psde->lock);
	if (psde->state_cache_size < SDE_STATE_CACHE_SIZE) {
		psde->state_cache[(psde->state_cache_size)++] = pstate;
		mutex_unlock(&psde->lock);
	} else {
		mutex_unlock(&psde->lock);
		kfree(pstate);
	}
}

static void sde_plane_destroy(struct drm_plane *plane)
{
	struct sde_plane *psde;

	DBG("");

	if (plane) {
		psde = to_sde_plane(plane);

		debugfs_remove_recursive(psde->debugfs_root);

		mutex_destroy(&psde->lock);

		drm_plane_helper_disable(plane);

		/* this will destroy the states as well */
		drm_plane_cleanup(plane);

		if (psde->pipe_hw)
			sde_hw_sspp_destroy(psde->pipe_hw);

		/* free state cache */
		while (psde->state_cache_size > 0)
			kfree(psde->state_cache[--(psde->state_cache_size)]);

		kfree(psde);
	}
}

static void sde_plane_destroy_state(struct drm_plane *plane,
		struct drm_plane_state *state)
{
	struct sde_plane_state *pstate;
	int i;

	if (!plane || !state) {
		DRM_ERROR("Invalid plane/state\n");
		return;
	}

	pstate = to_sde_plane_state(state);

	DBG("");

	/* remove ref count for frame buffers */
	if (state->fb)
		drm_framebuffer_unreference(state->fb);

	/* remove ref count for fence */
	if (pstate->sync_fence)
		sde_sync_put(pstate->sync_fence);

	/* remove ref count for blobs */
	for (i = 0; i < PLANE_PROP_BLOBCOUNT; ++i)
		if (pstate->property_blobs[i])
			drm_property_unreference_blob(
					pstate->property_blobs[i]);
	sde_plane_free_state(plane, pstate);
}

static struct drm_plane_state *
sde_plane_duplicate_state(struct drm_plane *plane)
{
	struct sde_plane_state *pstate;
	struct sde_plane_state *old_state;
	int i;

	if (!plane || !plane->state)
		return NULL;

	old_state = to_sde_plane_state(plane->state);
	pstate = sde_plane_alloc_state(plane);

	DBG("");

	if (!pstate)
		return NULL;

	memcpy(pstate, old_state, sizeof(*pstate));

	/* add ref count for frame buffer */
	if (pstate->base.fb)
		drm_framebuffer_reference(pstate->base.fb);

	/* add ref count for fence */
	if (pstate->sync_fence) {
		pstate->sync_fence = 0;
		_sde_plane_update_sync_fence(plane, pstate, pstate->
				property_values[PLANE_PROP_SYNC_FENCE]);
	}

	/* add ref count for blobs */
	for (i = 0; i < PLANE_PROP_BLOBCOUNT; ++i)
		if (pstate->property_blobs[i])
			drm_property_reference_blob(
					pstate->property_blobs[i]);

	pstate->mode_changed = false;
	pstate->pending = false;

	return &pstate->base;
}

static void sde_plane_reset(struct drm_plane *plane)
{
	struct sde_plane *psde;
	struct sde_plane_state *pstate;
	int i;

	if (!plane) {
		DRM_ERROR("Invalid plane\n");
		return;
	}

	psde = to_sde_plane(plane);
	DBG("%s", psde->pipe_name);

	/* remove previous state, if present */
	if (plane->state)
		sde_plane_destroy_state(plane, plane->state);
	plane->state = 0;

	pstate = sde_plane_alloc_state(plane);
	if (!pstate) {
		DRM_ERROR("Failed to (re)allocate plane state\n");
		return;
	}

	memset(pstate, 0, sizeof(*pstate));

	/* assign default property values */
	for (i = 0; i < PLANE_PROP_COUNT; ++i)
		pstate->property_values[i] = psde->property_defaults[i];

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
	psde->nformats = sde_populate_formats(psde->formats,
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

	mutex_init(&psde->lock);

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
