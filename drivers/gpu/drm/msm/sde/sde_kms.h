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

#ifndef __SDE_KMS_H__
#define __SDE_KMS_H__

#include "msm_drv.h"
#include "msm_kms.h"
#include "msm_mmu.h"
#include "mdp/mdp_kms.h"
#include "sde_hw_catalog.h"
#include "sde_hw_ctl.h"
#include "sde_hw_lm.h"
#include "sde_hw_interrupts.h"
#include "sde_connector.h"

/**
 * SDE_DEBUG - macro for kms/plane/crtc/encoder/connector logs
 * @fmt: Pointer to format string
 */
#define SDE_DEBUG(fmt, ...)                                                \
	do {                                                               \
		if (unlikely(drm_debug & DRM_UT_KMS))                      \
			drm_ut_debug_printk(__func__, fmt, ##__VA_ARGS__); \
		else                                                       \
			pr_debug(fmt, ##__VA_ARGS__);                      \
	} while (0)

/**
 * SDE_DEBUG_DRIVER - macro for hardware driver logging
 * @fmt: Pointer to format string
 */
#define SDE_DEBUG_DRIVER(fmt, ...)                                         \
	do {                                                               \
		if (unlikely(drm_debug & DRM_UT_DRIVER))                   \
			drm_ut_debug_printk(__func__, fmt, ##__VA_ARGS__); \
		else                                                       \
			pr_debug(fmt, ##__VA_ARGS__);                      \
	} while (0)

#define SDE_ERROR(fmt, ...) pr_err(fmt, ##__VA_ARGS__)

/*
 * struct sde_irq_callback - IRQ callback handlers
 * @func: intr handler
 * @arg: argument for the handler
 */
struct sde_irq_callback {
	void (*func)(void *arg, int irq_idx);
	void *arg;
};

/**
 * struct sde_irq: IRQ structure contains callback registration info
 * @total_irq:    total number of irq_idx obtained from HW interrupts mapping
 * @irq_cb_tbl:   array of IRQ callbacks setting
 * @cb_lock:      callback lock
 */
struct sde_irq {
	u32 total_irqs;
	struct sde_irq_callback *irq_cb_tbl;
	spinlock_t cb_lock;
};

/**
 *  struct sde_hw_res_map : Default resource table identifying default
 *             hw resource map. Primarily used for forcing DSI to use CTL_0/1
 *             and PingPong 0/1, if the field is set to SDE_NONE means any HW
 *             instance for that type is allowed as long as it is unused.
 */
struct sde_hw_res_map {
	enum sde_intf intf;
	enum sde_lm lm;
	enum sde_pingpong pp;
	enum sde_ctl ctl;
};

/* struct sde_hw_resource_manager : Resource manager maintains the current
 *                                  default platform config and manages shared
 *                                  hw resources ex:ctl_path hw driver context
 *                                  is needed by CRTCs/PLANEs/ENCODERs
 * @ctl        : table of control path hw driver contexts allocated
 * @mixer      : list of mixer hw drivers contexts allocated
 * @intr       : pointer to hw interrupt context
 * @res_table  : pointer to default hw_res table for this platform
 * @feature_map :BIT map for default enabled features ex:specifies if PP_SPLIT
 *               is enabled/disabled by default for this platform
 */
struct sde_hw_resource_manager {
	struct sde_hw_ctl *ctl[CTL_MAX];
	struct sde_hw_mixer *mixer[LM_MAX];
	struct sde_hw_intr *intr;
	const struct sde_hw_res_map *res_table;
	bool feature_map;
};

struct sde_kms {
	struct msm_kms base;
	struct drm_device *dev;
	int rev;
	struct sde_mdss_cfg *catalog;

	struct msm_mmu *mmu[MSM_SMMU_DOMAIN_MAX];
	int mmu_id[MSM_SMMU_DOMAIN_MAX];

	/* directory entry for debugfs */
	void *debugfs_root;

	/* io/register spaces: */
	void __iomem *mmio, *vbif[VBIF_MAX];

	struct regulator *vdd;
	struct regulator *mmagic;
	struct regulator *venus;

	struct clk *axi_clk;
	struct clk *ahb_clk;
	struct clk *src_clk;
	struct clk *core_clk;
	struct clk *lut_clk;
	struct clk *mmagic_clk;
	struct clk *iommu_axi_clk[MSM_SMMU_DOMAIN_MAX];
	struct clk *iommu_ahb_clk[MSM_SMMU_DOMAIN_MAX];
	struct clk *vsync_clk;

	struct {
		unsigned long enabled_mask;
		struct irq_domain *domain;
	} irqcontroller;

	struct sde_hw_intr *hw_intr;
	struct sde_irq irq_obj;
	struct sde_hw_resource_manager hw_res;
};

struct vsync_info {
	u32 frame_count;
	u32 line_count;
};

#define to_sde_kms(x) container_of(x, struct sde_kms, base)

struct sde_plane_state {
	struct drm_plane_state base;

	/* aligned with property */
	uint64_t property_values[PLANE_PROP_COUNT];

	/* blob properties */
	struct drm_property_blob *property_blobs[PLANE_PROP_BLOBCOUNT];

	/* dereferenced input fence pointer */
	void *input_fence;

	/* assigned by crtc blender */
	enum sde_stage stage;

	/* some additional transactional status to help us know in the
	 * apply path whether we need to update SMP allocation, and
	 * whether current update is still pending:
	 */
	bool mode_changed : 1;
	bool pending : 1;
};

#define to_sde_plane_state(x) \
	container_of(x, struct sde_plane_state, base)

/**
 * sde_plane_get_property - Query integer value of plane property
 *
 * @S: Pointer to plane state
 * @X: Property index, from enum msm_mdp_plane_property
 *
 * Return: Integer value of requested property
 */
#define sde_plane_get_property(S, X) \
	((S) && ((X) < PLANE_PROP_COUNT) ? ((S)->property_values[(X)]) : 0)

/**
 * sde_plane_get_property32 - Query 32-bit representation of plane property
 *
 * @S: Pointer to plane state
 * @X: Property index, from enum msm_mdp_plane_property
 *
 * Return: 32-bit value of requested property
 */
#define sde_plane_get_property32(S, X) \
	((S) && ((X) < PLANE_PROP_COUNT) ? \
	 (uint32_t)((S)->property_values[(X)]) : 0)

int sde_disable(struct sde_kms *sde_kms);
int sde_enable(struct sde_kms *sde_kms);

/**
 * Debugfs functions - extra helper functions for debugfs support
 *
 * Main debugfs documentation is located at,
 *
 * Documentation/filesystems/debugfs.txt
 *
 * @sde_debugfs_setup_regset32: Initialize data for sde_debugfs_create_regset32
 * @sde_debugfs_create_regset32: Create 32-bit register dump file
 * @sde_debugfs_get_root: Get root dentry for SDE_KMS's debugfs node
 */

/**
 * Companion structure for sde_debugfs_create_regset32. Do not initialize the
 * members of this structure explicitly; use sde_debugfs_setup_regset32 instead.
 */
struct sde_debugfs_regset32 {
	uint32_t offset;
	uint32_t blk_len;
	void __iomem *base;
};

/**
 * sde_debugfs_setup_regset32 - Initialize register block definition for debugfs
 * This function is meant to initialize sde_debugfs_regset32 structures for use
 * with sde_debugfs_create_regset32.
 * @regset: opaque register definition structure
 * @offset: sub-block offset
 * @length: sub-block length, in bytes
 * @base: base IOMEM address
 */
void sde_debugfs_setup_regset32(struct sde_debugfs_regset32 *regset,
		uint32_t offset, uint32_t length, void __iomem *base);

/**
 * sde_debugfs_create_regset32 - Create register read back file for debugfs
 *
 * This function is almost identical to the standard debugfs_create_regset32()
 * function, with the main difference being that a list of register
 * names/offsets do not need to be provided. The 'read' function simply outputs
 * sequential register values over a specified range.
 *
 * Similar to the related debugfs_create_regset32 API, the structure pointed to
 * by regset needs to persist for the lifetime of the created file. The calling
 * code is responsible for initialization/management of this structure.
 *
 * The structure pointed to by regset is meant to be opaque. Please use
 * sde_debugfs_setup_regset32 to initialize it.
 *
 * @name:   File name within debugfs
 * @mode:   File mode within debugfs
 * @parent: Parent directory entry within debugfs, can be NULL
 * @regset: Pointer to persistent register block definition
 *
 * Return: dentry pointer for newly created file, use either debugfs_remove()
 *         or debugfs_remove_recursive() (on a parent directory) to remove the
 *         file
 */
void *sde_debugfs_create_regset32(const char *name, umode_t mode,
		void *parent, struct sde_debugfs_regset32 *regset);

/**
 * sde_debugfs_get_root - Return root directory entry for SDE's debugfs
 *
 * The return value should be passed as the 'parent' argument to subsequent
 * debugfs create calls.
 *
 * @sde_kms: Pointer to SDE's KMS structure
 *
 * Return: dentry pointer for SDE's debugfs location
 */
void *sde_debugfs_get_root(struct sde_kms *sde_kms);

/**
 * SDE info management functions
 * These functions/definitions allow for building up a 'sde_info' structure
 * containing one or more "key=value\n" entries.
 */
#define SDE_KMS_INFO_MAX_SIZE	4096

/**
 * struct sde_kms_info - connector information structure container
 * @data: Array of information character data
 * @len: Current length of information data
 * @staged_len: Temporary data buffer length, commit to
 *              len using sde_kms_info_stop
 * @start: Whether or not a partial data entry was just started
 */
struct sde_kms_info {
	char data[SDE_KMS_INFO_MAX_SIZE];
	uint32_t len;
	uint32_t staged_len;
	bool start;
};

/**
 * SDE_KMS_INFO_DATA - Macro for accessing sde_kms_info data bytes
 * @S: Pointer to sde_kms_info structure
 * Returns: Pointer to byte data
 */
#define SDE_KMS_INFO_DATA(S)    ((S) ? ((struct sde_kms_info *)(S))->data : 0)

/**
 * SDE_KMS_INFO_DATALEN - Macro for accessing sde_kms_info data length
 * @S: Pointer to sde_kms_info structure
 * Returns: Size of available byte data
 */
#define SDE_KMS_INFO_DATALEN(S) ((S) ? ((struct sde_kms_info *)(S))->len : 0)

/**
 * sde_kms_info_reset - reset sde_kms_info structure
 * @info: Pointer to sde_kms_info structure
 */
void sde_kms_info_reset(struct sde_kms_info *info);

/**
 * sde_kms_info_add_keyint - add integer value to 'sde_kms_info'
 * @info: Pointer to sde_kms_info structure
 * @key: Pointer to key string
 * @value: Signed 32-bit integer value
 */
void sde_kms_info_add_keyint(struct sde_kms_info *info,
		const char *key,
		int32_t value);

/**
 * sde_kms_info_add_keystr - add string value to 'sde_kms_info'
 * @info: Pointer to sde_kms_info structure
 * @key: Pointer to key string
 * @value: Pointer to string value
 */
void sde_kms_info_add_keystr(struct sde_kms_info *info,
		const char *key,
		const char *value);

/**
 * sde_kms_info_start - begin adding key to 'sde_kms_info'
 * Usage:
 *      sde_kms_info_start(key)
 *      sde_kms_info_append(val_1)
 *      ...
 *      sde_kms_info_append(val_n)
 *      sde_kms_info_stop
 * @info: Pointer to sde_kms_info structure
 * @key: Pointer to key string
 */
void sde_kms_info_start(struct sde_kms_info *info,
		const char *key);

/**
 * sde_kms_info_append - append value string to 'sde_kms_info'
 * Usage:
 *      sde_kms_info_start(key)
 *      sde_kms_info_append(val_1)
 *      ...
 *      sde_kms_info_append(val_n)
 *      sde_kms_info_stop
 * @info: Pointer to sde_kms_info structure
 * @str: Pointer to partial value string
 */
void sde_kms_info_append(struct sde_kms_info *info,
		const char *str);

/**
 * sde_kms_info_append_format - append format code string to 'sde_kms_info'
 * Usage:
 *      sde_kms_info_start(key)
 *      sde_kms_info_append_format(fourcc, modifier)
 *      ...
 *      sde_kms_info_stop
 * @info: Pointer to sde_kms_info structure
 * @pixel_format: FOURCC format code
 * @modifier: 64-bit drm format modifier
 */
void sde_kms_info_append_format(struct sde_kms_info *info,
		uint32_t pixel_format,
		uint64_t modifier);

/**
 * sde_kms_info_stop - finish adding key to 'sde_kms_info'
 * Usage:
 *      sde_kms_info_start(key)
 *      sde_kms_info_append(val_1)
 *      ...
 *      sde_kms_info_append(val_n)
 *      sde_kms_info_stop
 * @info: Pointer to sde_kms_info structure
 */
void sde_kms_info_stop(struct sde_kms_info *info);

/**
 * HW resource manager functions
 * @sde_rm_acquire_ctl_path : Allocates control path
 * @sde_rm_get_ctl_path     : returns control path driver context for already
 *                           acquired ctl path
 * @sde_rm_release_ctl_path : Frees control path driver context
 * @sde_rm_acquire_mixer   : Allocates mixer hw driver context
 * @sde_rm_get_mixer       : returns mixer context for already
 *                           acquired mixer
 * @sde_rm_release_mixer   : Frees mixer hw driver context
 * @sde_rm_acquire_intr    : Allocate hw intr context
 * @sde_rm_get_intr        : Returns already acquired intr context
 * @sde_rm_get_hw_res_map  : Returns map for the passed INTF
 */
struct sde_hw_ctl *sde_rm_acquire_ctl_path(struct sde_kms *sde_kms,
		enum sde_ctl idx);
struct sde_hw_ctl *sde_rm_get_ctl_path(struct sde_kms *sde_kms,
		enum sde_ctl idx);
void sde_rm_release_ctl_path(struct sde_kms *sde_kms,
		enum sde_ctl idx);
struct sde_hw_mixer *sde_rm_acquire_mixer(struct sde_kms *sde_kms,
		enum sde_lm idx);
struct sde_hw_mixer *sde_rm_get_mixer(struct sde_kms *sde_kms,
		enum sde_lm idx);
void sde_rm_release_mixer(struct sde_kms *sde_kms,
		enum sde_lm idx);
struct sde_hw_intr *sde_rm_acquire_intr(struct sde_kms *sde_kms);
struct sde_hw_intr *sde_rm_get_intr(struct sde_kms *sde_kms);

const struct sde_hw_res_map *sde_rm_get_res_map(struct sde_kms *sde_kms,
		enum sde_intf idx);

/**
 * IRQ functions
 */
int sde_irq_domain_init(struct sde_kms *sde_kms);
int sde_irq_domain_fini(struct sde_kms *sde_kms);
void sde_irq_preinstall(struct msm_kms *kms);
int sde_irq_postinstall(struct msm_kms *kms);
void sde_irq_uninstall(struct msm_kms *kms);
irqreturn_t sde_irq(struct msm_kms *kms);

/**
 * sde_set_irqmask - IRQ helper function for writing IRQ mask
 *                   to SDE HW interrupt register.
 * @sde_kms:		SDE handle
 * @reg_off:		SDE HW interrupt register offset
 * @irqmask:		IRQ mask
 */
void sde_set_irqmask(
		struct sde_kms *sde_kms,
		uint32_t reg_off,
		uint32_t irqmask);

/**
 * sde_irq_idx_lookup - IRQ helper function for lookup irq_idx from HW
 *                      interrupt mapping table.
 * @sde_kms:		SDE handle
 * @intr_type:		SDE HW interrupt type for lookup
 * @instance_idx:	SDE HW block instance defined in sde_hw_mdss.h
 * @return:		irq_idx or -EINVAL when fail to lookup
 */
int sde_irq_idx_lookup(
		struct sde_kms *sde_kms,
		enum sde_intr_type intr_type,
		uint32_t instance_idx);

/**
 * sde_enable_irq - IRQ helper function for enabling one or more IRQs
 * @sde_kms:		SDE handle
 * @irq_idxs:		Array of irq index
 * @irq_count:		Number of irq_idx provided in the array
 * @return:		0 for success enabling IRQ, otherwise failure
 */
int sde_enable_irq(
		struct sde_kms *sde_kms,
		int *irq_idxs,
		uint32_t irq_count);

/**
 * sde_disable_irq - IRQ helper function for diabling one of more IRQs
 * @sde_kms:		SDE handle
 * @irq_idxs:		Array of irq index
 * @irq_count:		Number of irq_idx provided in the array
 * @return:		0 for success disabling IRQ, otherwise failure
 */
int sde_disable_irq(
		struct sde_kms *sde_kms,
		int *irq_idxs,
		uint32_t irq_count);

/**
 * sde_read_irq - IRQ helper function for reading IRQ status
 * @sde_kms:		SDE handle
 * @irq_idx:		irq index
 * @clear:		True to clear the irq after read
 * @return:		non-zero if irq detected; otherwise no irq detected
 */
u32 sde_read_irq(
		struct sde_kms *sde_kms,
		int irq_idx,
		bool clear);

/**
 * sde_register_irq_callback - For registering callback function on IRQ
 *                             interrupt
 * @sde_kms:		SDE handle
 * @irq_idx:		irq index
 * @irq_cb:		IRQ callback structure, containing callback function
 *			and argument. Passing NULL for irq_cb will unregister
 *			the callback for the given irq_idx
 * @return:		0 for success registering callback, otherwise failure
 */
int sde_register_irq_callback(
		struct sde_kms *sde_kms,
		int irq_idx,
		struct sde_irq_callback *irq_cb);

/**
 * sde_clear_all_irqs - Clearing all SDE IRQ interrupt status
 * @sde_kms:		SDE handle
 */
void sde_clear_all_irqs(struct sde_kms *sde_kms);

/**
 * sde_disable_all_irqs - Diabling all SDE IRQ interrupt
 * @sde_kms:		SDE handle
 */
void sde_disable_all_irqs(struct sde_kms *sde_kms);

/**
 * Vblank enable/disable functions
 */
int sde_enable_vblank(struct msm_kms *kms, struct drm_crtc *crtc);
void sde_disable_vblank(struct msm_kms *kms, struct drm_crtc *crtc);

/**
 * Plane functions
 */
enum sde_sspp sde_plane_pipe(struct drm_plane *plane);
void sde_plane_flush(struct drm_plane *plane);
struct drm_plane *sde_plane_init(struct drm_device *dev,
		uint32_t pipe, bool primary_plane);

/**
 * sde_plane_wait_input_fence - wait for input fence object
 * @plane:   Pointer to DRM plane object
 * @wait_ms: Wait timeout value
 * Returns: Zero on success
 */
int sde_plane_wait_input_fence(struct drm_plane *plane, uint32_t wait_ms);

/**
 * sde_plane_color_fill - Enables color fill on plane
 * @plane:  Pointer to DRM plane object
 * @color:  RGB fill color value, [23..16] Blue, [15..8] Green, [7..0] Red
 * @alpha:  8-bit fill alpha value, 255 selects 100% alpha
 *
 * Returns: 0 on success
 */
int sde_plane_color_fill(struct drm_plane *plane,
		uint32_t color, uint32_t alpha);

/**
 * CRTC functions
 */
int sde_crtc_vblank(struct drm_crtc *crtc, bool en);
void sde_crtc_wait_for_commit_done(struct drm_crtc *crtc);
void sde_crtc_cancel_pending_flip(struct drm_crtc *crtc, struct drm_file *file);
void sde_crtc_commit_kickoff(struct drm_crtc *crtc);

/**
 * sde_crtc_prepare_fence - callback to prepare for output fences
 * @crtc: Pointer to drm crtc object
 */
void sde_crtc_prepare_fence(struct drm_crtc *crtc);

struct drm_crtc *sde_crtc_init(struct drm_device *dev,
		struct drm_encoder *encoder,
		struct drm_plane *plane, int id);

/**
 * sde_crtc_complete_commit - callback signalling completion of current commit
 * @crtc: Pointer to drm crtc object
 */
void sde_crtc_complete_commit(struct drm_crtc *crtc);

/**
 * Encoder functions and data types
 */
struct sde_encoder_hw_resources {
	enum sde_intf_mode intfs[INTF_MAX];
	bool pingpongs[PINGPONG_MAX];
	bool ctls[CTL_MAX];
	bool pingpongsplit;
};

/**
 * sde_encoder_get_hw_resources - Populate table of required hardware resources
 * @encoder:	encoder pointer
 * @hw_res:	resource table to populate with encoder required resources
 */
void sde_encoder_get_hw_resources(struct drm_encoder *encoder,
		struct sde_encoder_hw_resources *hw_res);

/**
 * sde_encoder_register_vblank_callback - provide callback to encoder that
 *	will be called on the next vblank.
 * @encoder:	encoder pointer
 * @cb:		callback pointer, provide NULL to deregister and disable IRQs
 * @data:	user data provided to callback
 */
void sde_encoder_register_vblank_callback(struct drm_encoder *encoder,
		void (*cb)(void *), void *data);

/**
 * sde_encoder_schedule_kickoff - Register a callback with the encoder to
 *	trigger a double buffer flip of the ctl path (i.e. ctl flush and start)
 *	at the appropriate time.
 *	Immediately: if no previous commit is outstanding.
 *	Delayed: Save the callback, and return. Does not block. Callback will
 *	be triggered later. E.g. cmd encoder will trigger at pp_done irq
 *	irq if it outstanding.
 *	Callback registered is expected to flush _all_ ctl paths of the crtc
 * @encoder:	encoder pointer
 * @cb:		callback pointer, provide NULL to deregister
 * @data:	user data provided to callback
 */
void sde_encoder_schedule_kickoff(struct drm_encoder *encoder,
		void (*cb)(void *), void *data);

/**
 * sde_encoder_wait_nxt_committed - Wait for hardware to have flushed the
 *	current pending frames to hardware at a vblank or ctl_start
 *	Encoders will map this differently depending on irqs
 *	vid mode -> vsync_irq
 * @encoder:	encoder pointer
 *
 * Return: 0 on success, -EWOULDBLOCK if already signaled, error otherwise
 */
int sde_encoder_wait_for_commit_done(struct drm_encoder *drm_encoder);

/**
 * sde_encoders_init - query platform, create all encoders and bridges,
 *	and register them with the drm_device
 * @dev:	drm device pointer
 */
void sde_encoders_init(struct drm_device *dev);

#endif /* __sde_kms_H__ */
