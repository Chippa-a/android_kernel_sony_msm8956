#include <linux/cgroup.h>
#include <linux/err.h>
#include <linux/percpu.h>
#include <linux/printk.h>
#include <linux/slab.h>

#include "sched.h"

unsigned int sysctl_sched_cfs_boost __read_mostly;

#ifdef CONFIG_CGROUP_SCHEDTUNE

/*
 * EAS scheduler tunables for task groups.
 */

/* SchdTune tunables for a group of tasks */
struct schedtune {
	/* SchedTune CGroup subsystem */
	struct cgroup_subsys_state css;

	/* Boost group allocated ID */
	int idx;

	/* Boost value for tasks on that SchedTune CGroup */
	int boost;

#ifdef CONFIG_SCHED_HMP
	/* Toggle ability to override sched boost enabled */
	bool sched_boost_no_override;

	/*
	 * Controls whether a cgroup is eligible for sched boost or not. This
	 * can temporariliy be disabled by the kernel based on the no_override
	 * flag above.
	 */
	bool sched_boost_enabled;

	/*
	 * This tracks the default value of sched_boost_enabled and is used
	 * restore the value following any temporary changes to that flag.
	 */
	bool sched_boost_enabled_backup;

	/*
	 * Controls whether tasks of this cgroup should be colocated with each
	 * other and tasks of other cgroups that have the same flag turned on.
	 */
	bool colocate;

	/* Controls whether further updates are allowed to the colocate flag */
	bool colocate_update_disabled;
#endif

};

static inline struct schedtune *css_st(struct cgroup_subsys_state *css)
{
	return container_of(css, struct schedtune, css);
}

static inline struct schedtune *task_schedtune(struct task_struct *tsk)
{
	return css_st(task_css(tsk, schedtune_cgrp_id));
}

static inline struct schedtune *parent_st(struct schedtune *st)
{
	return css_st(st->css.parent);
}

/*
 * SchedTune root control group
 * The root control group is used to defined a system-wide boosting tuning,
 * which is applied to all tasks in the system.
 * Task specific boost tuning could be specified by creating and
 * configuring a child control group under the root one.
 * By default, system-wide boosting is disabled, i.e. no boosting is applied
 * to tasks which are not into a child control group.
 */
static struct schedtune
root_schedtune = {
	.boost	= 0,
#ifdef CONFIG_SCHED_HMP
	.sched_boost_no_override = false,
	.sched_boost_enabled = true,
	.sched_boost_enabled_backup = true,
	.colocate = false,
	.colocate_update_disabled = false,
#endif
};

/*
 * Maximum number of boost groups to support
 * When per-task boosting is used we still allow only limited number of
 * boost groups for two main reasons:
 * 1. on a real system we usually have only few classes of workloads which
 *    make sense to boost with different values (e.g. background vs foreground
 *    tasks, interactive vs low-priority tasks)
 * 2. a limited number allows for a simpler and more memory/time efficient
 *    implementation especially for the computation of the per-CPU boost
 *    value
 */
#define BOOSTGROUPS_COUNT 5

/* Array of configured boostgroups */
static struct schedtune *allocated_group[BOOSTGROUPS_COUNT] = {
	&root_schedtune,
	NULL,
};

/* SchedTune boost groups
 * Keep track of all the boost groups which impact on CPU, for example when a
 * CPU has two RUNNABLE tasks belonging to two different boost groups and thus
 * likely with different boost values.
 * Since on each system we expect only a limited number of boost groups, here
 * we use a simple array to keep track of the metrics required to compute the
 * maximum per-CPU boosting value.
 */
struct boost_groups {
	/* Maximum boost value for all RUNNABLE tasks on a CPU */
	unsigned boost_max;
	struct {
		/* The boost for tasks on that boost group */
		unsigned boost;
		/* Count of RUNNABLE tasks on that boost group */
		unsigned tasks;
	} group[BOOSTGROUPS_COUNT];
};

/* Boost groups affecting each CPU in the system */
DEFINE_PER_CPU(struct boost_groups, cpu_boost_groups);

#ifdef CONFIG_SCHED_HMP
static inline void init_sched_boost(struct schedtune *st)
{
	st->sched_boost_no_override = false;
	st->sched_boost_enabled = true;
	st->sched_boost_enabled_backup = st->sched_boost_enabled;
	st->colocate = false;
	st->colocate_update_disabled = false;
}

bool same_schedtune(struct task_struct *tsk1, struct task_struct *tsk2)
{
	return task_schedtune(tsk1) == task_schedtune(tsk2);
}

void update_cgroup_boost_settings(void)
{
	int i;

	for (i = 0; i < BOOSTGROUPS_COUNT; i++) {
		if (!allocated_group[i])
			break;

		if (allocated_group[i]->sched_boost_no_override)
			continue;

		allocated_group[i]->sched_boost_enabled = false;
	}
}

void restore_cgroup_boost_settings(void)
{
	int i;

	for (i = 0; i < BOOSTGROUPS_COUNT; i++) {
		if (!allocated_group[i])
			break;

		allocated_group[i]->sched_boost_enabled =
			allocated_group[i]->sched_boost_enabled_backup;
	}
}

bool task_sched_boost(struct task_struct *p)
{
	struct schedtune *st = task_schedtune(p);

	return st->sched_boost_enabled;
}

static u64
sched_boost_override_read(struct cgroup_subsys_state *css,
			struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->sched_boost_no_override;
}

static int sched_boost_override_write(struct cgroup_subsys_state *css,
			struct cftype *cft, u64 override)
{
	struct schedtune *st = css_st(css);

	st->sched_boost_no_override = !!override;

	return 0;
}

static u64 sched_boost_enabled_read(struct cgroup_subsys_state *css,
			struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->sched_boost_enabled;
}

static int sched_boost_enabled_write(struct cgroup_subsys_state *css,
			struct cftype *cft, u64 enable)
{
	struct schedtune *st = css_st(css);

	st->sched_boost_enabled = !!enable;
	st->sched_boost_enabled_backup = st->sched_boost_enabled;

	return 0;
}

static u64 sched_colocate_read(struct cgroup_subsys_state *css,
			struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->colocate;
}

static int sched_colocate_write(struct cgroup_subsys_state *css,
			struct cftype *cft, u64 colocate)
{
	struct schedtune *st = css_st(css);

	if (st->colocate_update_disabled)
		return -EPERM;

	st->colocate = !!colocate;
	st->colocate_update_disabled = true;
	return 0;
}

#else /* CONFIG_SCHED_HMP */

static inline void init_sched_boost(struct schedtune *st) { }

#endif /* CONFIG_SCHED_HMP */

static u64
boost_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->boost;
}

static int
boost_write(struct cgroup_subsys_state *css, struct cftype *cft,
	    u64 boost)
{
	struct schedtune *st = css_st(css);

	if (boost < 0 || boost > 100)
		return -EINVAL;

	st->boost = boost;
	if (css == &root_schedtune.css)
		sysctl_sched_cfs_boost = boost;

	return 0;
}

static void schedtune_attach(struct cgroup_taskset *tset)
{
	struct task_struct *task;
	struct cgroup_subsys_state *css;
	struct schedtune *st;
	bool colocate;

	cgroup_taskset_first(tset, &css);
	st = css_st(css);

	colocate = st->colocate;

	cgroup_taskset_for_each(task, css, tset)
		sync_cgroup_colocation(task, colocate);
}

static struct cftype files[] = {
	{
		.name = "boost",
		.read_u64 = boost_read,
		.write_u64 = boost_write,
	},
#ifdef CONFIG_SCHED_HMP
	{
		.name = "sched_boost_no_override",
		.read_u64 = sched_boost_override_read,
		.write_u64 = sched_boost_override_write,
	},
	{
		.name = "sched_boost_enabled",
		.read_u64 = sched_boost_enabled_read,
		.write_u64 = sched_boost_enabled_write,
	},
	{
		.name = "colocate",
		.read_u64 = sched_colocate_read,
		.write_u64 = sched_colocate_write,
	},
#endif
	{ }	/* terminate */
};

static int
schedtune_boostgroup_init(struct schedtune *st)
{
	/* Keep track of allocated boost groups */
	allocated_group[st->idx] = st;

	return 0;
}

static int
schedtune_init(void)
{
	struct boost_groups *bg;
	int cpu;

	/* Initialize the per CPU boost groups */
	for_each_possible_cpu(cpu) {
		bg = &per_cpu(cpu_boost_groups, cpu);
		memset(bg, 0, sizeof(struct boost_groups));
	}

	pr_info("  schedtune configured to support %d boost groups\n",
		BOOSTGROUPS_COUNT);
	return 0;
}

static struct cgroup_subsys_state *
schedtune_css_alloc(struct cgroup_subsys_state *parent_css)
{
	struct schedtune *st;
	int idx;

	if (!parent_css) {
		schedtune_init();
		return &root_schedtune.css;
	}

	/* Allow only single level hierachies */
	if (parent_css != &root_schedtune.css) {
		pr_err("Nested SchedTune boosting groups not allowed\n");
		return ERR_PTR(-ENOMEM);
	}

	/* Allow only a limited number of boosting groups */
	for (idx = 1; idx < BOOSTGROUPS_COUNT; ++idx)
		if (!allocated_group[idx])
			break;
	if (idx == BOOSTGROUPS_COUNT) {
		pr_err("Trying to create more than %d SchedTune boosting groups\n",
		       BOOSTGROUPS_COUNT);
		return ERR_PTR(-ENOSPC);
	}

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		goto out;

	/* Initialize per CPUs boost group support */
	st->idx = idx;
	init_sched_boost(st);
	if (schedtune_boostgroup_init(st))
		goto release;

	return &st->css;

release:
	kfree(st);
out:
	return ERR_PTR(-ENOMEM);
}

static void
schedtune_boostgroup_release(struct schedtune *st)
{
	/* Keep track of allocated boost groups */
	allocated_group[st->idx] = NULL;
}

static void
schedtune_css_free(struct cgroup_subsys_state *css)
{
	struct schedtune *st = css_st(css);

	schedtune_boostgroup_release(st);
	kfree(st);
}

struct cgroup_subsys schedtune_cgrp_subsys = {
	.css_alloc	= schedtune_css_alloc,
	.css_free	= schedtune_css_free,
	.legacy_cftypes	= files,
	.early_init	= 1,
	.allow_attach	= subsys_cgroup_allow_attach,
	.attach		= schedtune_attach,
};

#endif /* CONFIG_CGROUP_SCHEDTUNE */

int
sysctl_sched_cfs_boost_handler(struct ctl_table *table, int write,
			       void __user *buffer, size_t *lenp,
			       loff_t *ppos)
{
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);

	if (ret || !write)
		return ret;

	return 0;
}

