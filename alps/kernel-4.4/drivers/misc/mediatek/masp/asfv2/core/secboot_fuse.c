#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include "sec_boot_lib.h"

static int secboot_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%x\n",sec_schip_enabled()? 0x303030 : 0x0);
	return 0;
}

static int secboot_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, secboot_proc_show, NULL);
}

static const struct file_operations secboot_proc_fops = {
	.open		= secboot_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_secboot_init(void)
{
	proc_create("secboot_fuse_reg", 0, NULL, &secboot_proc_fops);
	return 0;
}
fs_initcall(proc_secboot_init);
