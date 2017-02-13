/* Copyright (c) 2015-2017, The Linux Foundation. All rights reserved.
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
#include <drm/msm_drm_pp.h>
#include "sde_hw_mdss.h"
#include "sde_hwio.h"
#include "sde_hw_catalog.h"
#include "sde_hw_dspp.h"
#include "sde_hw_color_processing.h"

static struct sde_dspp_cfg *_dspp_offset(enum sde_dspp dspp,
		struct sde_mdss_cfg *m,
		void __iomem *addr,
		struct sde_hw_blk_reg_map *b)
{
	int i;

	for (i = 0; i < m->dspp_count; i++) {
		if (dspp == m->dspp[i].id) {
			b->base_off = addr;
			b->blk_off = m->dspp[i].base;
			b->hwversion = m->hwversion;
			b->log_mask = SDE_DBG_MASK_DSPP;
			return &m->dspp[i];
		}
	}

	return ERR_PTR(-EINVAL);
}

static void _setup_dspp_ops(struct sde_hw_dspp *c, unsigned long features)
{
	int i = 0, ret;

	for (i = 0; i < SDE_DSPP_MAX; i++) {
		if (!test_bit(i, &features))
			continue;
		switch (i) {
		case SDE_DSPP_PCC:
			if (c->cap->sblk->pcc.version ==
				(SDE_COLOR_PROCESS_VER(0x1, 0x7)))
				c->ops.setup_pcc = sde_setup_dspp_pcc_v1_7;
			break;
		case SDE_DSPP_HSIC:
			if (c->cap->sblk->hsic.version ==
				(SDE_COLOR_PROCESS_VER(0x1, 0x7)))
				c->ops.setup_hue = sde_setup_dspp_pa_hue_v1_7;
			break;
		case SDE_DSPP_VLUT:
			if (c->cap->sblk->vlut.version ==
				(SDE_COLOR_PROCESS_VER(0x1, 0x7))) {
				c->ops.setup_vlut =
				    sde_setup_dspp_pa_vlut_v1_7;
			} else if (c->cap->sblk->vlut.version ==
					(SDE_COLOR_PROCESS_VER(0x1, 0x8))) {
				ret = reg_dmav1_init_dspp_op_v4(i, c->idx);
				if (ret)
					c->ops.setup_vlut =
					reg_dmav1_setup_dspp_vlutv18;
			}
			break;
		case SDE_DSPP_GAMUT:
			if (c->cap->sblk->gamut.version ==
					SDE_COLOR_PROCESS_VER(0x4, 0)) {
				ret = reg_dmav1_init_dspp_op_v4(i, c->idx);
				if (!ret)
					c->ops.setup_gamut =
						reg_dmav1_setup_dspp_3d_gamutv4;
				else
					c->ops.setup_gamut =
						sde_setup_dspp_3d_gamutv4;
			}
			break;
		case SDE_DSPP_GC:
			if (c->cap->sblk->gc.version ==
					SDE_COLOR_PROCESS_VER(0x1, 8)) {
				ret = reg_dmav1_init_dspp_op_v4(i, c->idx);
				if (!ret)
					c->ops.setup_gc =
						reg_dmav1_setup_dspp_gcv18;
				/** programming for v18 through ahb is same
				 * as v17 hence assign v17 function
				 */
				else
					c->ops.setup_gc =
						sde_setup_dspp_gc_v1_7;
			}
			break;
		default:
			break;
		}
	}
}

struct sde_hw_dspp *sde_hw_dspp_init(enum sde_dspp idx,
			void __iomem *addr,
			struct sde_mdss_cfg *m)
{
	struct sde_hw_dspp *c;
	struct sde_dspp_cfg *cfg;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	cfg = _dspp_offset(idx, m, addr, &c->hw);
	if (IS_ERR_OR_NULL(cfg)) {
		kfree(c);
		return ERR_PTR(-EINVAL);
	}

	/* Assign ops */
	c->idx = idx;
	c->cap = cfg;
	_setup_dspp_ops(c, c->cap->features);

	return c;
}

void sde_hw_dspp_destroy(struct sde_hw_dspp *dspp)
{
	kfree(dspp);
}
