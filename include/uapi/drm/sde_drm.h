#ifndef _SDE_DRM_H_
#define _SDE_DRM_H_

/* Total number of supported color planes */
#define SDE_MAX_PLANES  4

/* Total number of parameterized detail enhancer mapping curves */
#define SDE_MAX_DE_CURVES 3
/**
 * Blend operations for "blend_op" property
 *
 * @SDE_DRM_BLEND_OP_NOT_DEFINED:   No blend operation defined for the layer.
 * @SDE_DRM_BLEND_OP_OPAQUE:        Apply a constant blend operation. The layer
 *                                  would appear opaque in case fg plane alpha
 *                                  is 0xff.
 * @SDE_DRM_BLEND_OP_PREMULTIPLIED: Apply source over blend rule. Layer already
 *                                  has alpha pre-multiplication done. If the fg
 *                                  plane alpha is less than 0xff, apply
 *                                  modulation as well. This operation is
 *                                  intended on layers having alpha channel.
 * @SDE_DRM_BLEND_OP_COVERAGE:      Apply source over blend rule. Layer is not
 *                                  alpha pre-multiplied. Apply
 *                                  pre-multiplication. If fg plane alpha is
 *                                  less than 0xff, apply modulation as well.
 * @SDE_DRM_BLEND_OP_MAX:           Used to track maximum blend operation
 *                                  possible by mdp.
 */
#define SDE_DRM_BLEND_OP_NOT_DEFINED    0
#define SDE_DRM_BLEND_OP_OPAQUE         1
#define SDE_DRM_BLEND_OP_PREMULTIPLIED  2
#define SDE_DRM_BLEND_OP_COVERAGE       3
#define SDE_DRM_BLEND_OP_MAX            4

/**
 * Bit masks for "src_config" property
 * construct bitmask via (1UL << SDE_DRM_<flag>)
 */
#define SDE_DRM_DEINTERLACE         0   /* Specifies interlaced input */

/* DRM bitmasks are restricted to 0..63 */
#define SDE_DRM_BITMASK_COUNT       64

/**
 * struct sde_drm_pix_ext_v1 - version 1 of pixel ext structure
 * @num_pxls_start: Number of start pixels
 * @num_pxls_end:   Number of end pixels
 * @ftch_start:     Number of overfetch start pixels
 * @ftch_end:       Number of overfetch end pixels
 * @rpt_start:      Number of repeat start pixels
 * @rpt_end:        Number of repeat end pixels
 * @roi:            Input ROI settings
 */
struct sde_drm_pix_ext_v1 {
	/*
	 * Number of pixels ext in left, right, top and bottom direction
	 * for all color components. This pixel value for each color
	 * component should be sum of fetch + repeat pixels.
	 */
	int32_t num_pxls_start[SDE_MAX_PLANES];
	int32_t num_pxls_end[SDE_MAX_PLANES];

	/*
	 * Number of pixels needs to be overfetched in left, right, top
	 * and bottom directions from source image for scaling.
	 */
	int32_t ftch_start[SDE_MAX_PLANES];
	int32_t ftch_end[SDE_MAX_PLANES];

	/*
	 * Number of pixels needs to be repeated in left, right, top and
	 * bottom directions for scaling.
	 */
	int32_t rpt_start[SDE_MAX_PLANES];
	int32_t rpt_end[SDE_MAX_PLANES];

	uint32_t roi[SDE_MAX_PLANES];
};

/**
 * struct sde_drm_scaler_v1 - version 1 of struct sde_drm_scaler
 * @lr:            Pixel extension settings for left/right
 * @tb:            Pixel extension settings for top/botton
 * @init_phase_x:  Initial scaler phase values for x
 * @phase_step_x:  Phase step values for x
 * @init_phase_y:  Initial scaler phase values for y
 * @phase_step_y:  Phase step values for y
 * @horz_filter:   Horizontal filter array
 * @vert_filter:   Vertical filter array
 */
struct sde_drm_scaler_v1 {
	/*
	 * Pix ext settings
	 */
	struct sde_drm_pix_ext_v1 lr;
	struct sde_drm_pix_ext_v1 tb;

	/*
	 * Phase settings
	 */
	int32_t init_phase_x[SDE_MAX_PLANES];
	int32_t phase_step_x[SDE_MAX_PLANES];
	int32_t init_phase_y[SDE_MAX_PLANES];
	int32_t phase_step_y[SDE_MAX_PLANES];

	/*
	 * Filter type to be used for scaling in horizontal and vertical
	 * directions
	 */
	uint32_t horz_filter[SDE_MAX_PLANES];
	uint32_t vert_filter[SDE_MAX_PLANES];
};

/**
 * struct sde_drm_de_v1 - version 1 of detail enhancer structure
 * @enable:         Enables/disables detail enhancer
 * @sharpen_level1: Sharpening strength for noise
 * @sharpen_level2: Sharpening strength for context
 * @clip:           Clip coefficient
 * @limit:          Detail enhancer limit factor
 * @thr_quiet:      Quite zone threshold
 * @thr_dieout:     Die-out zone threshold
 * @thr_low:        Linear zone left threshold
 * @thr_high:       Linear zone right threshold
 * @prec_shift:     Detail enhancer precision
 * @adjust_a:       Mapping curves A coefficients
 * @adjust_b:       Mapping curves B coefficients
 * @adjust_c:       Mapping curves C coefficients
 */
struct sde_drm_de_v1 {
	uint32_t enable;
	int16_t sharpen_level1;
	int16_t sharpen_level2;
	uint16_t clip;
	uint16_t limit;
	uint16_t thr_quiet;
	uint16_t thr_dieout;
	uint16_t thr_low;
	uint16_t thr_high;
	uint16_t prec_shift;
	int16_t adjust_a[SDE_MAX_DE_CURVES];
	int16_t adjust_b[SDE_MAX_DE_CURVES];
	int16_t adjust_c[SDE_MAX_DE_CURVES];
};

/**
 * struct sde_drm_scaler_v2 - version 2 of struct sde_drm_scaler
 * @enable:        Mask of SDE_DRM_SCALER_ bits
 * @lr:            Pixel extension settings for left/right
 * @tb:            Pixel extension settings for top/botton
 * @horz_decimate: Horizontal decimation factor
 * @vert_decimate: Vertical decimation factor
 * @init_phase_x:  Initial scaler phase values for x
 * @phase_step_x:  Phase step values for x
 * @init_phase_y:  Initial scaler phase values for y
 * @phase_step_y:  Phase step values for y
 * @horz_filter:   Horizontal filter array
 * @vert_filter:   Vertical filter array
 */
struct sde_drm_scaler_v2 {
	/*
	 * General definitions
	 */
	uint32_t enable;
	uint32_t dir_en;

	/*
	 * Pix ext settings
	 */
	struct sde_drm_pix_ext_v1 lr;
	struct sde_drm_pix_ext_v1 tb;

	/*
	 * Decimation settings
	 */
	uint32_t horz_decimate;
	uint32_t vert_decimate;

	/*
	 * Phase settings
	 */
	int32_t init_phase_x[SDE_MAX_PLANES];
	int32_t phase_step_x[SDE_MAX_PLANES];
	int32_t init_phase_y[SDE_MAX_PLANES];
	int32_t phase_step_y[SDE_MAX_PLANES];

	/* alpha plane can only be scaled using bilinear or pixel
	 * repeat/drop, specify these for Y and UV planes only
	 */
	uint32_t preload_x[SDE_MAX_PLANES];
	uint32_t preload_y[SDE_MAX_PLANES];
	uint32_t src_width[SDE_MAX_PLANES];
	uint32_t src_height[SDE_MAX_PLANES];

	uint32_t dst_width;
	uint32_t dst_height;

	uint32_t y_rgb_filter_cfg;
	uint32_t uv_filter_cfg;
	uint32_t alpha_filter_cfg;
	uint32_t blend_cfg;

	uint32_t lut_flag;
	uint32_t dir_lut_idx;

	/* for Y(RGB) and UV planes*/
	uint32_t y_rgb_cir_lut_idx;
	uint32_t uv_cir_lut_idx;
	uint32_t y_rgb_sep_lut_idx;
	uint32_t uv_sep_lut_idx;

	/*
	 * Detail enhancer settings
	 */
	struct sde_drm_de_v1 de;
};


/*
 * Define constants for struct sde_drm_csc
 */
#define SDE_CSC_MATRIX_COEFF_SIZE   9
#define SDE_CSC_CLAMP_SIZE          6
#define SDE_CSC_BIAS_SIZE           3

/**
 * struct sde_drm_csc_v1 - version 1 of struct sde_drm_csc
 * @ctm_coeff:          Matrix coefficients, in S31.32 format
 * @pre_bias:           Pre-bias array values
 * @post_bias:          Post-bias array values
 * @pre_clamp:          Pre-clamp array values
 * @post_clamp:         Post-clamp array values
 */
struct sde_drm_csc_v1 {
	int64_t ctm_coeff[SDE_CSC_MATRIX_COEFF_SIZE];
	uint32_t pre_bias[SDE_CSC_BIAS_SIZE];
	uint32_t post_bias[SDE_CSC_BIAS_SIZE];
	uint32_t pre_clamp[SDE_CSC_CLAMP_SIZE];
	uint32_t post_clamp[SDE_CSC_CLAMP_SIZE];
};

/* Writeback Config version definition */
#define SDE_DRM_WB_CFG		0x1

/* SDE_DRM_WB_CONFIG_FLAGS - Writeback configuration flags */
#define SDE_DRM_WB_CFG_FLAGS_CONNECTED	(1<<0)

/**
 * struct sde_drm_wb_cfg - Writeback configuration structure
 * @flags:		see DRM_MSM_WB_CONFIG_FLAGS
 * @connector_id:	writeback connector identifier
 * @count_modes:	Count of modes in modes_ptr
 * @modes:		Pointer to struct drm_mode_modeinfo
 */
struct sde_drm_wb_cfg {
	uint32_t flags;
	uint32_t connector_id;
	uint32_t count_modes;
	uint64_t modes;
};

#endif /* _SDE_DRM_H_ */
