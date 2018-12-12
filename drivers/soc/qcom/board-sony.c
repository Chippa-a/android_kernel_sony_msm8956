#include <linux/export.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/setup.h>

struct proc_info_node {
    const char *filename;
    const struct file_operations fops;
};

/*
 * HACK: These two functions sets the androidboot.mode=charger based
 * on the Sony Mobile parameters startup and warmboot.
 */
static unsigned long sony_startup;

static int __init sony_param_startup(char *str)
{
	if (kstrtoul(str, 16, &sony_startup))
		return 1;

	return 0;
}
early_param("startup", sony_param_startup);

static int __init sony_param_warmboot(char *str)
{
	unsigned long warmboot;

	if (kstrtoul(str, 16, &warmboot))
		return 1;

	if (!warmboot && (sony_startup & 0x4004))
		strlcat(boot_command_line, " androidboot.mode=charger",
			COMMAND_LINE_SIZE);

	return 0;
}
early_param("warmboot", sony_param_warmboot);

/*
 * SIM variant information
 */
char *simslot_name = NULL;

static int __init sony_param_phoneid(char *str)
{
    if (strchr(str, ',')) {
        simslot_name = "dsds";
    } else {
        simslot_name = "ss";
    }

    return 0;
}
early_param("oemandroidboot.phoneid", sony_param_phoneid);

static int simslot_id_proc_read(struct seq_file *m, void *v)
{
    if (!strncmp(simslot_name, "", strlen(simslot_name))) {
        pr_info(KERN_INFO "Invalid SIM slot name\n");
        seq_printf(m, "%s\n", "NULL");
    } else {
        seq_printf(m, "%s\n", simslot_name);
    }

    return 0;
}

static int simslot_id_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, simslot_id_proc_read, NULL);
}

static const struct proc_info_node proc_info[] = {
    {
        .filename       = "simslot_id",
        .fops.open      = simslot_id_proc_open,
        .fops.read      = seq_read,
        .fops.release   = single_release,
        .fops.llseek    = seq_lseek,
    },
};

static void create_proc_device_node(void)
{
    int i = 0;

    for (i = 0; i < ARRAY_SIZE(proc_info); i++) {
        const struct proc_info_node *f = &proc_info[i];

        if (!proc_create(f->filename, S_IRUSR | S_IRGRP | S_IROTH, NULL, &f->fops))
            goto error;
    }

    return;

error:
    while (--i >= 0) {
        const struct proc_info_node *f = &proc_info[i];
        remove_proc_entry(f->filename, NULL);
    }
}

static int __init board_sony_init(void)
{
    create_proc_device_node();

    return 0;
}
subsys_initcall(board_sony_init);
