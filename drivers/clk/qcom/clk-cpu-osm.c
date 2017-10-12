/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "clk: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpu.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <dt-bindings/clock/qcom,cpucc-sdm845.h>

#include "common.h"
#include "clk-regmap.h"
#include "clk-voter.h"
#include "clk-debug.h"

#define OSM_INIT_RATE			300000000UL
#define XO_RATE				19200000UL
#define OSM_TABLE_SIZE			40
#define SINGLE_CORE			1
#define MAX_CLUSTER_CNT			3
#define MAX_MEM_ACC_VAL_PER_LEVEL	3
#define MAX_CORE_COUNT			4
#define CORE_COUNT_VAL(val)		((val & GENMASK(18, 16)) >> 16)

#define OSM_REG_SIZE			32

#define ENABLE_REG			0x0
#define FREQ_REG			0x110
#define VOLT_REG			0x114
#define CORE_DCVS_CTRL			0xbc

#define DCVS_PERF_STATE_DESIRED_REG_0_V1	0x780
#define DCVS_PERF_STATE_DESIRED_REG_0_V2	0x920
#define DCVS_PERF_STATE_DESIRED_REG(n, v2) \
	(((v2) ? DCVS_PERF_STATE_DESIRED_REG_0_V2 \
		: DCVS_PERF_STATE_DESIRED_REG_0_V1) + 4 * (n))

#define OSM_CYCLE_COUNTER_STATUS_REG_0_V1	0x7d0
#define OSM_CYCLE_COUNTER_STATUS_REG_0_V2	0x9c0
#define OSM_CYCLE_COUNTER_STATUS_REG(n, v2) \
	(((v2) ? OSM_CYCLE_COUNTER_STATUS_REG_0_V2 \
		: OSM_CYCLE_COUNTER_STATUS_REG_0_V1) + 4 * (n))

struct osm_entry {
	u16 virtual_corner;
	u16 open_loop_volt;
	long frequency;
};

struct clk_osm {
	struct clk_hw hw;
	struct osm_entry osm_table[OSM_TABLE_SIZE];
	struct dentry *debugfs;
	void __iomem *vbase;
	phys_addr_t pbase;
	spinlock_t lock;
	bool per_core_dcvs;
	u32 num_entries;
	u32 cluster_num;
	u32 core_num;
	u64 total_cycle_counter;
	u32 prev_cycle_counter;
};

static bool is_v2;

static inline struct clk_osm *to_clk_osm(struct clk_hw *_hw)
{
	return container_of(_hw, struct clk_osm, hw);
}

static inline void clk_osm_write_reg(struct clk_osm *c, u32 val, u32 offset)
{
	writel_relaxed(val, c->vbase + offset);
}

static inline int clk_osm_read_reg(struct clk_osm *c, u32 offset)
{
	return readl_relaxed(c->vbase + offset);
}

static inline int clk_osm_read_reg_no_log(struct clk_osm *c, u32 offset)
{
	return readl_relaxed_no_log(c->vbase + offset);
}

static inline int clk_osm_mb(struct clk_osm *c)
{
	return readl_relaxed_no_log(c->vbase + ENABLE_REG);
}

static long clk_osm_list_rate(struct clk_hw *hw, unsigned int n,
					unsigned long rate_max)
{
	if (n >= hw->init->num_rate_max)
		return -ENXIO;
	return hw->init->rate_max[n];
}

static inline bool is_better_rate(unsigned long req, unsigned long best,
			unsigned long new)
{
	if (IS_ERR_VALUE(new))
		return false;

	return (req <= new && new < best) || (best < req && best < new);
}

static long clk_osm_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	int i;
	unsigned long rrate = 0;

	if (!hw)
		return -EINVAL;

	/*
	 * If the rate passed in is 0, return the first frequency in the
	 * FMAX table.
	 */
	if (!rate)
		return hw->init->rate_max[0];

	for (i = 0; i < hw->init->num_rate_max; i++) {
		if (is_better_rate(rate, rrate, hw->init->rate_max[i])) {
			rrate = hw->init->rate_max[i];
			if (rate == rrate)
				break;
		}
	}

	pr_debug("%s: rate %lu, rrate %ld, Rate max %ld\n", __func__, rate,
						rrate, hw->init->rate_max[i]);

	return rrate;
}

static int clk_osm_search_table(struct osm_entry *table, int entries, long rate)
{
	int index = 0;

	for (index = 0; index < entries; index++) {
		if (rate == table[index].frequency)
			return index;
	}

	return -EINVAL;
}

const struct clk_ops clk_ops_cpu_osm = {
	.round_rate = clk_osm_round_rate,
	.list_rate = clk_osm_list_rate,
	.debug_init = clk_debug_measure_add,
};

static int l3_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	struct clk_osm *cpuclk = to_clk_osm(hw);
	int index = 0;
	unsigned long r_rate;

	if (!cpuclk)
		return -EINVAL;

	r_rate = clk_osm_round_rate(hw, rate, NULL);

	if (rate != r_rate) {
		pr_err("invalid requested rate=%ld\n", rate);
		return -EINVAL;
	}

	/* Convert rate to table index */
	index = clk_osm_search_table(cpuclk->osm_table,
				     cpuclk->num_entries, r_rate);
	if (index < 0) {
		pr_err("cannot set %s to %lu\n", clk_hw_get_name(hw), rate);
		return -EINVAL;
	}
	pr_debug("rate: %lu --> index %d\n", rate, index);

	clk_osm_write_reg(cpuclk, index, DCVS_PERF_STATE_DESIRED_REG(0, is_v2));

	/* Make sure the write goes through before proceeding */
	clk_osm_mb(cpuclk);

	return 0;
}

static unsigned long l3_clk_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct clk_osm *cpuclk = to_clk_osm(hw);
	int index = 0;

	if (!cpuclk)
		return -EINVAL;

	index = clk_osm_read_reg(cpuclk, DCVS_PERF_STATE_DESIRED_REG(0, is_v2));

	pr_debug("%s: Index %d, freq %ld\n", __func__, index,
				cpuclk->osm_table[index].frequency);

	/* Convert index to frequency */
	return cpuclk->osm_table[index].frequency;
}


static struct clk_ops clk_ops_l3_osm = {
	.round_rate = clk_osm_round_rate,
	.list_rate = clk_osm_list_rate,
	.recalc_rate = l3_clk_recalc_rate,
	.set_rate = l3_clk_set_rate,
	.debug_init = clk_debug_measure_add,
};

static struct clk_init_data osm_clks_init[] = {
	[0] = {
		.name = "l3_clk",
		.parent_names = (const char *[]){ "bi_tcxo_ao" },
		.num_parents = 1,
		.ops = &clk_ops_l3_osm,
	},
	[1] = {
		.name = "pwrcl_clk",
		.parent_names = (const char *[]){ "bi_tcxo_ao" },
		.num_parents = 1,
		.ops = &clk_ops_cpu_osm,
	},
	[2] = {
		.name = "perfcl_clk",
		.parent_names = (const char *[]){ "bi_tcxo_ao" },
		.num_parents = 1,
		.ops = &clk_ops_cpu_osm,
	},
};

static struct clk_osm l3_clk = {
	.cluster_num = 0,
	.hw.init = &osm_clks_init[0],
};

static DEFINE_CLK_VOTER(l3_cluster0_vote_clk, l3_clk, 0);
static DEFINE_CLK_VOTER(l3_cluster1_vote_clk, l3_clk, 0);

static struct clk_osm pwrcl_clk = {
	.cluster_num = 1,
	.hw.init = &osm_clks_init[1],
};

static struct clk_osm cpu0_pwrcl_clk = {
	.core_num = 0,
	.total_cycle_counter = 0,
	.prev_cycle_counter = 0,
	.hw.init = &(struct clk_init_data){
		.name = "cpu0_pwrcl_clk",
		.parent_names = (const char *[]){ "pwrcl_clk" },
		.num_parents = 1,
		.ops = &clk_dummy_ops,
	},
};

static struct clk_osm cpu1_pwrcl_clk = {
	.core_num = 1,
	.total_cycle_counter = 0,
	.prev_cycle_counter = 0,
	.hw.init = &(struct clk_init_data){
		.name = "cpu1_pwrcl_clk",
		.parent_names = (const char *[]){ "pwrcl_clk" },
		.num_parents = 1,
		.ops = &clk_dummy_ops,
	},
};

static struct clk_osm cpu2_pwrcl_clk = {
	.core_num = 2,
	.total_cycle_counter = 0,
	.prev_cycle_counter = 0,
	.hw.init = &(struct clk_init_data){
		.name = "cpu2_pwrcl_clk",
		.parent_names = (const char *[]){ "pwrcl_clk" },
		.num_parents = 1,
		.ops = &clk_dummy_ops,
	},
};

static struct clk_osm cpu3_pwrcl_clk = {
	.core_num = 3,
	.total_cycle_counter = 0,
	.prev_cycle_counter = 0,
	.hw.init = &(struct clk_init_data){
		.name = "cpu3_pwrcl_clk",
		.parent_names = (const char *[]){ "pwrcl_clk" },
		.num_parents = 1,
		.ops = &clk_dummy_ops,
	},
};

static struct clk_osm perfcl_clk = {
	.cluster_num = 2,
	.hw.init = &osm_clks_init[2],
};


static struct clk_osm cpu4_perfcl_clk = {
	.core_num = 0,
	.total_cycle_counter = 0,
	.prev_cycle_counter = 0,
	.hw.init = &(struct clk_init_data){
		.name = "cpu4_perfcl_clk",
		.parent_names = (const char *[]){ "perfcl_clk" },
		.num_parents = 1,
		.ops = &clk_dummy_ops,
	},
};

static struct clk_osm cpu5_perfcl_clk = {
	.core_num = 1,
	.total_cycle_counter = 0,
	.prev_cycle_counter = 0,
	.hw.init = &(struct clk_init_data){
		.name = "cpu5_perfcl_clk",
		.parent_names = (const char *[]){ "perfcl_clk" },
		.num_parents = 1,
		.ops = &clk_dummy_ops,
	},
};

static struct clk_osm cpu6_perfcl_clk = {
	.core_num = 2,
	.total_cycle_counter = 0,
	.prev_cycle_counter = 0,
	.hw.init = &(struct clk_init_data){
		.name = "cpu6_perfcl_clk",
		.parent_names = (const char *[]){ "perfcl_clk" },
		.num_parents = 1,
		.ops = &clk_dummy_ops,
	},
};

static struct clk_osm cpu7_perfcl_clk = {
	.core_num = 3,
	.total_cycle_counter = 0,
	.prev_cycle_counter = 0,
	.hw.init = &(struct clk_init_data){
		.name = "cpu7_perfcl_clk",
		.parent_names = (const char *[]){ "perfcl_clk" },
		.num_parents = 1,
		.ops = &clk_dummy_ops,
	},
};

static struct clk_hw *osm_qcom_clk_hws[] = {
	[L3_CLK] = &l3_clk.hw,
	[L3_CLUSTER0_VOTE_CLK] = &l3_cluster0_vote_clk.hw,
	[L3_CLUSTER1_VOTE_CLK] = &l3_cluster1_vote_clk.hw,
	[PWRCL_CLK] = &pwrcl_clk.hw,
	[CPU0_PWRCL_CLK] = &cpu0_pwrcl_clk.hw,
	[CPU1_PWRCL_CLK] = &cpu1_pwrcl_clk.hw,
	[CPU2_PWRCL_CLK] = &cpu2_pwrcl_clk.hw,
	[CPU3_PWRCL_CLK] = &cpu3_pwrcl_clk.hw,
	[PERFCL_CLK] = &perfcl_clk.hw,
	[CPU4_PERFCL_CLK] = &cpu4_perfcl_clk.hw,
	[CPU5_PERFCL_CLK] = &cpu5_perfcl_clk.hw,
	[CPU6_PERFCL_CLK] = &cpu6_perfcl_clk.hw,
	[CPU7_PERFCL_CLK] = &cpu7_perfcl_clk.hw,
};

static struct clk_osm *logical_cpu_to_clk(int cpu)
{
	struct device_node *cpu_node;
	const u32 *cell;
	u64 hwid;
	static struct clk_osm *cpu_clk_map[NR_CPUS];
	struct clk_osm *clk_cpu_map[] = {
		&cpu0_pwrcl_clk,
		&cpu1_pwrcl_clk,
		&cpu2_pwrcl_clk,
		&cpu3_pwrcl_clk,
		&cpu4_perfcl_clk,
		&cpu5_perfcl_clk,
		&cpu6_perfcl_clk,
		&cpu7_perfcl_clk,
	};

	if (!cpu_clk_map[cpu]) {
		cpu_node = of_get_cpu_node(cpu, NULL);
		if (!cpu_node)
			return NULL;

		cell = of_get_property(cpu_node, "reg", NULL);
		if (!cell) {
			pr_err("%s: missing reg property\n",
			       cpu_node->full_name);
			of_node_put(cpu_node);
			return NULL;
		}

		hwid = of_read_number(cell, of_n_addr_cells(cpu_node));
		hwid = (hwid >> 8) & 0xff;
		of_node_put(cpu_node);
		if (hwid >= ARRAY_SIZE(clk_cpu_map)) {
			pr_err("unsupported CPU number - %d (hw_id - %llu)\n",
			       cpu, hwid);
			return NULL;
		}

		cpu_clk_map[cpu] = clk_cpu_map[hwid];
	}

	return cpu_clk_map[cpu];
}

static struct clk_osm *osm_configure_policy(struct cpufreq_policy *policy)
{
	int cpu;
	struct clk_hw *parent, *c_parent;
	struct clk_osm *first;
	struct clk_osm *c, *n;

	c = logical_cpu_to_clk(policy->cpu);
	if (!c)
		return NULL;

	c_parent = clk_hw_get_parent(&c->hw);
	if (!c_parent)
		return NULL;

	/*
	 * Don't put any other CPUs into the policy if we're doing
	 * per_core_dcvs
	 */
	if (to_clk_osm(c_parent)->per_core_dcvs)
		return c;

	first = c;
	/* Find CPUs that share the same clock domain */
	for_each_possible_cpu(cpu) {
		n = logical_cpu_to_clk(cpu);
		if (!n)
			continue;

		parent = clk_hw_get_parent(&n->hw);
		if (!parent)
			return NULL;
		if (parent != c_parent)
			continue;

		cpumask_set_cpu(cpu, policy->cpus);
		if (n->core_num == 0)
			first = n;
	}

	return first;
}

static void
osm_set_index(struct clk_osm *c, unsigned int index, unsigned int num)
{
	clk_osm_write_reg(c, index, DCVS_PERF_STATE_DESIRED_REG(num, is_v2));

	/* Make sure the write goes through before proceeding */
	clk_osm_mb(c);
}

static int
osm_cpufreq_target_index(struct cpufreq_policy *policy, unsigned int index)
{
	struct clk_osm *c = policy->driver_data;

	osm_set_index(c, index, c->core_num);
	return 0;
}

static unsigned int osm_cpufreq_get(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
	struct clk_osm *c;
	u32 index;

	if (!policy)
		return 0;

	c = policy->driver_data;
	index = clk_osm_read_reg(c,
			DCVS_PERF_STATE_DESIRED_REG(c->core_num, is_v2));
	return policy->freq_table[index].frequency;
}

static int osm_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *table;
	struct clk_osm *c, *parent;
	struct clk_hw *p_hw;
	int ret;
	unsigned int i;
	unsigned int xo_kHz;

	c = osm_configure_policy(policy);
	if (!c) {
		pr_err("no clock for CPU%d\n", policy->cpu);
		return -ENODEV;
	}

	p_hw = clk_hw_get_parent(&c->hw);
	if (!p_hw) {
		pr_err("no parent clock for CPU%d\n", policy->cpu);
		return -ENODEV;
	}

	parent = to_clk_osm(p_hw);
	c->vbase = parent->vbase;

	p_hw = clk_hw_get_parent(p_hw);
	if (!p_hw) {
		pr_err("no xo clock for CPU%d\n", policy->cpu);
		return -ENODEV;
	}
	xo_kHz = clk_hw_get_rate(p_hw) / 1000;

	table = kcalloc(OSM_TABLE_SIZE + 1, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	for (i = 0; i < OSM_TABLE_SIZE; i++) {
		u32 data, src, div, lval, core_count;

		data = clk_osm_read_reg(c, FREQ_REG + i * OSM_REG_SIZE);
		src = (data & GENMASK(31, 30)) >> 30;
		div = (data & GENMASK(29, 28)) >> 28;
		lval = data & GENMASK(7, 0);
		core_count = CORE_COUNT_VAL(data);

		if (!src)
			table[i].frequency = OSM_INIT_RATE / 1000;
		else
			table[i].frequency = xo_kHz * lval;
		table[i].driver_data = table[i].frequency;

		if (core_count != MAX_CORE_COUNT)
			table[i].frequency = CPUFREQ_ENTRY_INVALID;

		/* Two of the same frequencies means end of table */
		if (i > 0 && table[i - 1].driver_data == table[i].driver_data) {
			struct cpufreq_frequency_table *prev = &table[i - 1];

			if (prev->frequency == CPUFREQ_ENTRY_INVALID) {
				prev->flags = CPUFREQ_BOOST_FREQ;
				prev->frequency = prev->driver_data;
			}

			break;
		}
	}
	table[i].frequency = CPUFREQ_TABLE_END;

	ret = cpufreq_table_validate_and_show(policy, table);
	if (ret) {
		pr_err("%s: invalid frequency table: %d\n", __func__, ret);
		goto err;
	}

	policy->driver_data = c;
	return 0;

err:
	kfree(table);
	return ret;
}

static int osm_cpufreq_cpu_exit(struct cpufreq_policy *policy)
{
	kfree(policy->freq_table);
	policy->freq_table = NULL;
	return 0;
}

static struct freq_attr *osm_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	&cpufreq_freq_attr_scaling_boost_freqs,
	NULL
};

static struct cpufreq_driver qcom_osm_cpufreq_driver = {
	.flags		= CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK |
			  CPUFREQ_HAVE_GOVERNOR_PER_POLICY,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= osm_cpufreq_target_index,
	.get		= osm_cpufreq_get,
	.init		= osm_cpufreq_cpu_init,
	.exit		= osm_cpufreq_cpu_exit,
	.name		= "osm-cpufreq",
	.attr		= osm_cpufreq_attr,
	.boost_enabled	= true,
};

static u32 find_voltage(struct clk_osm *c, unsigned long rate)
{
	struct osm_entry *table = c->osm_table;
	int entries = c->num_entries, i;

	for (i = 0; i < entries; i++) {
		if (rate == table[i].frequency) {
			/* OPP table voltages have units of mV */
			return table[i].open_loop_volt * 1000;
		}
	}

	return -EINVAL;
}

static int add_opp(struct clk_osm *c, struct device *dev)
{
	unsigned long rate = 0;
	u32 uv;
	long rc;
	int j = 0;
	unsigned long min_rate = c->hw.init->rate_max[0];
	unsigned long max_rate =
			c->hw.init->rate_max[c->hw.init->num_rate_max - 1];

	while (1) {
		rate = c->hw.init->rate_max[j++];
		uv = find_voltage(c, rate);
		if (uv <= 0) {
			pr_warn("No voltage for %lu.\n", rate);
			return -EINVAL;
		}

		rc = dev_pm_opp_add(dev, rate, uv);
		if (rc) {
			pr_warn("failed to add OPP for %lu\n", rate);
			return rc;
		}

		/*
		 * Print the OPP pair for the lowest and highest frequency for
		 * each device that we're populating. This is important since
		 * this information will be used by thermal mitigation and the
		 * scheduler.
		 */
		if (rate == min_rate)
			pr_info("Set OPP pair (%lu Hz, %d uv) on %s\n",
				rate, uv, dev_name(dev));

		if (rate == max_rate && max_rate != min_rate) {
			pr_info("Set OPP pair (%lu Hz, %d uv) on %s\n",
				rate, uv, dev_name(dev));
			break;
		}

		if (min_rate == max_rate)
			break;
	}
	return 0;
}

static void populate_opp_table(struct platform_device *pdev)
{
	int cpu;
	struct device *cpu_dev;
	struct clk_osm *c, *parent;
	struct clk_hw *hw_parent;
	struct device_node *l3_node_0, *l3_node_4;
	struct platform_device *l3_dev_0, *l3_dev_4;

	for_each_possible_cpu(cpu) {
		c = logical_cpu_to_clk(cpu);
		if (!c) {
			pr_err("no clock device for CPU=%d\n", cpu);
			return;
		}

		hw_parent = clk_hw_get_parent(&c->hw);
		parent = to_clk_osm(hw_parent);
		cpu_dev = get_cpu_device(cpu);
		if (cpu_dev)
			if (add_opp(parent, cpu_dev))
				pr_err("Failed to add OPP levels for %s\n",
					dev_name(cpu_dev));
	}

	l3_node_0 = of_parse_phandle(pdev->dev.of_node, "l3-dev0", 0);
	if (!l3_node_0) {
		pr_err("can't find the L3 cluster 0 dt node\n");
		return;
	}

	l3_dev_0 = of_find_device_by_node(l3_node_0);
	if (!l3_dev_0) {
		pr_err("can't find the L3 cluster 0 dt device\n");
		return;
	}

	if (add_opp(&l3_clk, &l3_dev_0->dev))
		pr_err("Failed to add OPP levels for L3 cluster 0\n");

	l3_node_4 = of_parse_phandle(pdev->dev.of_node, "l3-dev4", 0);
	if (!l3_node_4) {
		pr_err("can't find the L3 cluster 1 dt node\n");
		return;
	}

	l3_dev_4 = of_find_device_by_node(l3_node_4);
	if (!l3_dev_4) {
		pr_err("can't find the L3 cluster 1 dt device\n");
		return;
	}

	if (add_opp(&l3_clk, &l3_dev_4->dev))
		pr_err("Failed to add OPP levels for L3 cluster 1\n");
}

static u64 clk_osm_get_cpu_cycle_counter(int cpu)
{
	u32 val;
	int core_num;
	unsigned long flags;
	u64 cycle_counter_ret;
	struct clk_osm *parent, *c = logical_cpu_to_clk(cpu);

	if (IS_ERR_OR_NULL(c)) {
		pr_err("no clock device for CPU=%d\n", cpu);
		return 0;
	}

	parent = to_clk_osm(clk_hw_get_parent(&c->hw));

	spin_lock_irqsave(&parent->lock, flags);
	/*
	 * Use core 0's copy as proxy for the whole cluster when per
	 * core DCVS is disabled.
	 */
	core_num = parent->per_core_dcvs ? c->core_num : 0;
	val = clk_osm_read_reg_no_log(parent,
			OSM_CYCLE_COUNTER_STATUS_REG(core_num, is_v2));

	if (val < c->prev_cycle_counter) {
		/* Handle counter overflow */
		c->total_cycle_counter += UINT_MAX -
			c->prev_cycle_counter + val;
		c->prev_cycle_counter = val;
	} else {
		c->total_cycle_counter += val - c->prev_cycle_counter;
		c->prev_cycle_counter = val;
	}
	cycle_counter_ret = c->total_cycle_counter;
	spin_unlock_irqrestore(&parent->lock, flags);

	return cycle_counter_ret;
}

static int clk_osm_read_lut(struct platform_device *pdev, struct clk_osm *c)
{
	u32 data, src, lval, i, j = OSM_TABLE_SIZE;

	for (i = 0; i < OSM_TABLE_SIZE; i++) {
		data = clk_osm_read_reg(c, FREQ_REG + i * OSM_REG_SIZE);
		src = ((data & GENMASK(31, 30)) >> 30);
		lval = (data & GENMASK(7, 0));

		if (!src)
			c->osm_table[i].frequency = OSM_INIT_RATE;
		else
			c->osm_table[i].frequency = XO_RATE * lval;

		data = clk_osm_read_reg(c, VOLT_REG + i * OSM_REG_SIZE);
		c->osm_table[i].virtual_corner =
					((data & GENMASK(21, 16)) >> 16);
		c->osm_table[i].open_loop_volt = (data & GENMASK(11, 0));

		pr_debug("index=%d freq=%ld virtual_corner=%d open_loop_voltage=%u\n",
			 i, c->osm_table[i].frequency,
			 c->osm_table[i].virtual_corner,
			 c->osm_table[i].open_loop_volt);

		if (i > 0 && j == OSM_TABLE_SIZE && c->osm_table[i].frequency ==
					c->osm_table[i - 1].frequency)
			j = i;
	}

	osm_clks_init[c->cluster_num].rate_max = devm_kcalloc(&pdev->dev,
						 j, sizeof(unsigned long),
						       GFP_KERNEL);
	if (!osm_clks_init[c->cluster_num].rate_max)
		return -ENOMEM;

	for (i = 0; i < j; i++)
		osm_clks_init[c->cluster_num].rate_max[i] =
					c->osm_table[i].frequency;

	c->num_entries = osm_clks_init[c->cluster_num].num_rate_max = j;
	return 0;
}

static int clk_osm_resources_init(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"osm_l3_base");
	if (!res) {
		dev_err(&pdev->dev,
			"Unable to get platform resource for osm_l3_base");
		return -ENOMEM;
	}

	l3_clk.pbase = (unsigned long)res->start;
	l3_clk.vbase = devm_ioremap(&pdev->dev, res->start, resource_size(res));

	if (!l3_clk.vbase) {
		dev_err(&pdev->dev, "Unable to map osm_l3_base base\n");
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"osm_pwrcl_base");
	if (!res) {
		dev_err(&pdev->dev,
			"Unable to get platform resource for osm_pwrcl_base");
		return -ENOMEM;
	}

	pwrcl_clk.pbase = (unsigned long)res->start;
	pwrcl_clk.vbase = devm_ioremap(&pdev->dev, res->start,
						  resource_size(res));
	if (!pwrcl_clk.vbase) {
		dev_err(&pdev->dev, "Unable to map osm_pwrcl_base base\n");
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"osm_perfcl_base");
	if (!res) {
		dev_err(&pdev->dev,
			"Unable to get platform resource for osm_perfcl_base");
		return -ENOMEM;
	}

	perfcl_clk.pbase = (unsigned long)res->start;
	perfcl_clk.vbase = devm_ioremap(&pdev->dev, res->start,
						  resource_size(res));

	if (!perfcl_clk.vbase) {
		dev_err(&pdev->dev, "Unable to map osm_perfcl_base base\n");
		return -ENOMEM;
	}

	return 0;
}

static int clk_cpu_osm_driver_probe(struct platform_device *pdev)
{
	int rc = 0, i;
	u32 val;
	int num_clks = ARRAY_SIZE(osm_qcom_clk_hws);
	struct clk *ext_xo_clk, *clk;
	struct device *dev = &pdev->dev;
	struct clk_onecell_data *clk_data;
	struct cpu_cycle_counter_cb cb = {
		.get_cpu_cycle_counter = clk_osm_get_cpu_cycle_counter,
	};

	/*
	 * Require the RPM-XO clock to be registered before OSM.
	 * The cpuss_gpll0_clk_src is listed to be configured by BL.
	 */
	ext_xo_clk = devm_clk_get(dev, "xo_ao");
	if (IS_ERR(ext_xo_clk)) {
		if (PTR_ERR(ext_xo_clk) != -EPROBE_DEFER)
			dev_err(dev, "Unable to get xo clock\n");
		return PTR_ERR(ext_xo_clk);
	}

	is_v2 = of_device_is_compatible(pdev->dev.of_node,
					"qcom,clk-cpu-osm-v2");

	clk_data = devm_kzalloc(&pdev->dev, sizeof(struct clk_onecell_data),
								GFP_KERNEL);
	if (!clk_data)
		goto exit;

	clk_data->clks = devm_kzalloc(&pdev->dev, (num_clks *
					sizeof(struct clk *)), GFP_KERNEL);
	if (!clk_data->clks)
		goto clk_err;

	clk_data->clk_num = num_clks;

	rc = clk_osm_resources_init(pdev);
	if (rc) {
		if (rc != -EPROBE_DEFER)
			dev_err(&pdev->dev, "OSM resources init failed, rc=%d\n",
				rc);
		return rc;
	}

	/* Check if per-core DCVS is enabled/not */
	val = clk_osm_read_reg(&pwrcl_clk, CORE_DCVS_CTRL);
	if (val && BIT(0))
		pwrcl_clk.per_core_dcvs = true;

	val = clk_osm_read_reg(&perfcl_clk, CORE_DCVS_CTRL);
	if (val && BIT(0))
		perfcl_clk.per_core_dcvs = true;

	rc = clk_osm_read_lut(pdev, &l3_clk);
	if (rc) {
		dev_err(&pdev->dev, "Unable to read OSM LUT for L3, rc=%d\n",
			rc);
		return rc;
	}

	rc = clk_osm_read_lut(pdev, &pwrcl_clk);
	if (rc) {
		dev_err(&pdev->dev, "Unable to read OSM LUT for power cluster, rc=%d\n",
			rc);
		return rc;
	}

	rc = clk_osm_read_lut(pdev, &perfcl_clk);
	if (rc) {
		dev_err(&pdev->dev, "Unable to read OSM LUT for perf cluster, rc=%d\n",
			rc);
		return rc;
	}

	spin_lock_init(&l3_clk.lock);
	spin_lock_init(&pwrcl_clk.lock);
	spin_lock_init(&perfcl_clk.lock);

	/* Register OSM l3, pwr and perf clocks with Clock Framework */
	for (i = 0; i < num_clks; i++) {
		clk = devm_clk_register(&pdev->dev, osm_qcom_clk_hws[i]);
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "Unable to register CPU clock at index %d\n",
				i);
			return PTR_ERR(clk);
		}
		clk_data->clks[i] = clk;
	}

	rc = of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get,
								clk_data);
	if (rc) {
		dev_err(&pdev->dev, "Unable to register CPU clocks\n");
			goto provider_err;
	}

	get_online_cpus();

	WARN(clk_prepare_enable(l3_cluster0_vote_clk.hw.clk),
			"clk: Failed to enable cluster0 clock for L3\n");
	WARN(clk_prepare_enable(l3_cluster1_vote_clk.hw.clk),
			"clk: Failed to enable cluster1 clock for L3\n");

	populate_opp_table(pdev);

	of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	register_cpu_cycle_counter_cb(&cb);
	put_online_cpus();

	rc = cpufreq_register_driver(&qcom_osm_cpufreq_driver);
	if (rc)
		goto provider_err;

	pr_info("OSM CPUFreq driver inited\n");
	return 0;

provider_err:
	if (clk_data)
		devm_kfree(&pdev->dev, clk_data->clks);
clk_err:
	devm_kfree(&pdev->dev, clk_data);
exit:
	dev_err(&pdev->dev, "OSM CPUFreq driver failed to initialize, rc=%d\n",
		rc);
	panic("Unable to Setup OSM CPUFreq");
}

static const struct of_device_id match_table[] = {
	{ .compatible = "qcom,clk-cpu-osm" },
	{ .compatible = "qcom,clk-cpu-osm-v2" },
	{}
};

static struct platform_driver clk_cpu_osm_driver = {
	.probe = clk_cpu_osm_driver_probe,
	.driver = {
		.name = "clk-cpu-osm",
		.of_match_table = match_table,
		.owner = THIS_MODULE,
	},
};

static int __init clk_cpu_osm_init(void)
{
	return platform_driver_register(&clk_cpu_osm_driver);
}
subsys_initcall(clk_cpu_osm_init);

static void __exit clk_cpu_osm_exit(void)
{
	platform_driver_unregister(&clk_cpu_osm_driver);
}
module_exit(clk_cpu_osm_exit);

MODULE_DESCRIPTION("QTI CPU clock driver for OSM");
MODULE_LICENSE("GPL v2");
