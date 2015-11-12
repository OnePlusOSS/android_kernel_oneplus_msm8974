/* drivers/android/ram_console.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/persistent_ram.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include "ram_console.h"

static struct persistent_ram_zone *ram_console_zone;
static const char *bootinfo;
static size_t bootinfo_size;

static void
ram_console_write(struct console *console, const char *s, unsigned int count)
{
	struct persistent_ram_zone *prz = console->data;
	persistent_ram_write(prz, s, count);
}

static struct console ram_console = {
	.name	= "ram",
	.write	= ram_console_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME,
	.index	= -1,
};

void ram_console_enable_console(int enabled)
{
	if (enabled)
		ram_console.flags |= CON_ENABLED;
	else
		ram_console.flags &= ~CON_ENABLED;
}

static int __devinit ram_console_probe(struct platform_device *pdev)
{
	struct ram_console_platform_data *pdata = pdev->dev.platform_data;
	struct persistent_ram_zone *prz;

	prz = persistent_ram_init_ringbuffer(&pdev->dev, true);
	if (IS_ERR(prz))
		return PTR_ERR(prz);


	if (pdata) {
		bootinfo = kstrdup(pdata->bootinfo, GFP_KERNEL);
		if (bootinfo)
			bootinfo_size = strlen(bootinfo);
	}

	ram_console_zone = prz;
	ram_console.data = prz;

	register_console(&ram_console);

	return 0;
}

//add by jiachenghui for boot time optimize 2015-9-16
#ifdef VENDOR_EDIT
static int probe_ret;
struct ram_console_data{
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct platform_device *pdev;
};
static struct ram_console_data optimize_data;
static void __devinit  ram_console_probe_func(struct work_struct *w)
{
	struct platform_device *pdev = optimize_data.pdev;
	printk("boot_time: after optimize call [ram_console_probe] on cpu %d\n",smp_processor_id());
	probe_ret = ram_console_probe(pdev);
	printk("boot_time: ram_console_probe return %d\n", probe_ret);
}

static int __devinit oem_ram_console_probe(struct platform_device *pdev)
{
	int i;
	int offline_times = 0;//add by jiachenghui for only one cpu online,2015-9-28
	optimize_data.pdev = pdev;
	optimize_data.workqueue = create_workqueue("ram_console_optimize");
	INIT_DELAYED_WORK(&(optimize_data.work), ram_console_probe_func);
	printk("boot_time: before optimize [ram_console_probe]  on cpu %d\n",smp_processor_id());
	for (i = 0; i <= 3; i++)
       {
             printk("boot_time: [ram_console_probe] CPU%d is %s\n",i,cpu_is_offline(i)?"offline":"online");
	      if (cpu_is_offline(i) || i == smp_processor_id())
             {
                   offline_times++;//add by jiachenghui for only one cpu online,2015-9-28
                   continue;
             }
	      queue_delayed_work_on(i,optimize_data.workqueue,&(optimize_data.work),msecs_to_jiffies(300));
	      break;
	}
	//add by jiachenghui for only one cpu online,2015-9-28
	if(offline_times == 4)
		queue_delayed_work_on(smp_processor_id(),optimize_data.workqueue,&(optimize_data.work),msecs_to_jiffies(300));
       //add by jiachenghui for only one cpu online,2015-9-28
	return probe_ret;
}
#endif //VENDOR_EDIT
//add by jiachenghui for boot time optimize 2015-9-16


#ifdef CONFIG_VENDOR_EDIT_OP_LASTKMSG
/* add by yangrujin@bsp 2015/9/2, support last_kmsg feature */
static const struct of_device_id msm_ram_console_match[] = {
	{.compatible = "ram-console"},
	{}
};
#endif

static struct platform_driver ram_console_driver = {
	.driver		= {
		.name	= "ram_console",
#ifdef CONFIG_VENDOR_EDIT_OP_LASTKMSG
/* add by yangrujin@bsp 2015/9/2, support last_kmsg feature */
		.owner = THIS_MODULE,
		.of_match_table = msm_ram_console_match,
#endif
	},
//add by jiachenghui for boot time optimize 2015-9-16
#ifdef VENDOR_EDIT
	.probe = oem_ram_console_probe,
#else
//end add by jiachenghui for boot time optimize 2015-9-16
	.probe = ram_console_probe,
#endif //add by jiachenghui for boot time optimize 2015-9-16
};

static int __init ram_console_module_init(void)
{
	return platform_driver_register(&ram_console_driver);
}

#ifndef CONFIG_PRINTK
#define dmesg_restrict	0
#endif

static ssize_t ram_console_read_old(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;
	struct persistent_ram_zone *prz = ram_console_zone;
	size_t old_log_size = persistent_ram_old_size(prz);
	const char *old_log = persistent_ram_old(prz);
	char *str;
	int ret;

	if (dmesg_restrict && !capable(CAP_SYSLOG))
		return -EPERM;

	/* Main last_kmsg log */
	if (pos < old_log_size) {
		count = min(len, (size_t)(old_log_size - pos));
		if (copy_to_user(buf, old_log + pos, count))
			return -EFAULT;
		goto out;
	}

	/* ECC correction notice */
	pos -= old_log_size;
	count = persistent_ram_ecc_string(prz, NULL, 0);
	if (pos < count) {
		str = kmalloc(count, GFP_KERNEL);
		if (!str)
			return -ENOMEM;
		persistent_ram_ecc_string(prz, str, count + 1);
		count = min(len, (size_t)(count - pos));
		ret = copy_to_user(buf, str + pos, count);
		kfree(str);
		if (ret)
			return -EFAULT;
		goto out;
	}

	/* Boot info passed through pdata */
	pos -= count;
	if (pos < bootinfo_size) {
		count = min(len, (size_t)(bootinfo_size - pos));
		if (copy_to_user(buf, bootinfo + pos, count))
			return -EFAULT;
		goto out;
	}

	/* EOF */
	return 0;

out:
	*offset += count;
	return count;
}

static const struct file_operations ram_console_file_ops = {
	.owner = THIS_MODULE,
	.read = ram_console_read_old,
};

static int __init ram_console_late_init(void)
{
	struct proc_dir_entry *entry;
	struct persistent_ram_zone *prz = ram_console_zone;

	if (!prz)
		return 0;


	if (persistent_ram_old_size(prz) == 0)
		return 0;
	pr_err("ram_console_late_init() create proc last_kmsg \r\n");
	entry = create_proc_entry("last_kmsg", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "ram_console: failed to create proc entry\n");
		persistent_ram_free_old(prz);
		return 0;
	}

	entry->proc_fops = &ram_console_file_ops;
	entry->size = persistent_ram_old_size(prz) +
		persistent_ram_ecc_string(prz, NULL, 0) +
		bootinfo_size;

	return 0;
}

late_initcall(ram_console_late_init);
postcore_initcall(ram_console_module_init);
