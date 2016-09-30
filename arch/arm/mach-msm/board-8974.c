/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/memory.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/krait-regulator.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal.h>

#ifdef VENDOR_EDIT
//Add for ram_console device
#include <linux/persistent_ram.h>
#endif

#include <asm/mach/map.h>
#include <asm/hardware/gic.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/msm_iomap.h>
#ifdef CONFIG_ION_MSM
#include <mach/ion.h>
#endif
#include <mach/msm_memtypes.h>
#include <mach/msm_smd.h>
#include <mach/restart.h>
#include <mach/rpm-smd.h>
#include <mach/rpm-regulator-smd.h>
#include <mach/socinfo.h>
#include <mach/msm_smem.h>
#include "board-dt.h"
#include "clock.h"
#include "devices.h"
#include "spm.h"
#include "pm.h"
#include "modem_notifier.h"
#include "platsmp.h"

#ifdef VENDOR_EDIT
/*add begin for factory mode*/
#include <linux/gpio.h>
static struct kobject *systeminfo_kobj;

/*add for pcb version*/
#include <linux/pcb_version.h>
static char *saved_command_line_pcb_version = NULL;
static char *saved_command_line_rf_version = NULL;
static struct ddr_info saved_command_line_ddr_version;
static int  current_pcb_version_num = PCB_VERSION_UNKNOWN;
static int  current_rf_version_num = RF_VERSION_UNKNOWN;

/* pcb_version_num: evb=10, evt=20, dvt=30, pvt=40, unkown=99 */
int get_pcb_version(void)
{
	return current_pcb_version_num;
}

int get_rf_version(void)
{
	return current_rf_version_num;
}
/*Get ddr information, include ddr manufacture information and ddr row information*/
int get_ddr_info(struct ddr_info *ddr_information){

	strcpy(ddr_information->ddr_manufacture,saved_command_line_ddr_version.ddr_manufacture);
	strcpy(ddr_information->ddr_row_info,saved_command_line_ddr_version.ddr_row_info);
	return 0;
}
int __init board_pcb_verison_init(void)
{
	if (strstr(boot_command_line,"hw.pcb_version=10"))
		current_pcb_version_num = HW_VERSION__10;
	else if (strstr(boot_command_line,"hw.pcb_version=11"))
		current_pcb_version_num = HW_VERSION__11;
	else if (strstr(boot_command_line,"hw.pcb_version=12"))
		current_pcb_version_num = HW_VERSION__12;
	else if (strstr(boot_command_line,"hw.pcb_version=13"))
		current_pcb_version_num = HW_VERSION__13;
	else if (strstr(boot_command_line,"hw.pcb_version=20"))
		current_pcb_version_num = HW_VERSION__20;
	else if (strstr(boot_command_line,"hw.pcb_version=21"))
		current_pcb_version_num = HW_VERSION__21;
	else if (strstr(boot_command_line,"hw.pcb_version=22"))
		current_pcb_version_num = HW_VERSION__22;
	else if (strstr(boot_command_line,"hw.pcb_version=23"))
		current_pcb_version_num = HW_VERSION__23;
	else
	    current_pcb_version_num = PCB_VERSION_UNKNOWN;

	//printk("pcb_version num %d \n",current_pcb_version_num);

	return 0;
}
/*
 * Get ddr manufacture information and ddr row information
*/
int __init board_ddr_info_get(struct ddr_info *ddr_information)
{
    int i;
    char *substr = strstr(boot_command_line, "ddr_manufacture_info=");
    if(substr==NULL)
            return 0;
    substr += strlen("ddr_manufacture_info=");

    for(i=0; substr[i] != ' '; i++) {
        ddr_information->ddr_manufacture[i] = substr[i];
    }

    ddr_information->ddr_manufacture[i] = '\0';

    substr = strstr(boot_command_line, "ddr_row0_info=");
    substr += strlen("ddr_row0_info=");
    for(i=0; substr[i] != ' '; i++) {
        ddr_information->ddr_row_info[i] = substr[i];
    }
    ddr_information->ddr_row_info[i] = '\0';

    printk(KERN_INFO "%s: parse ddr manufacture is %s,row:%s\n", __func__, ddr_information->ddr_manufacture,ddr_information->ddr_row_info);
    return 0;
}
int __init board_rf_version_init(void)
{
	//saved_command_line_pcb_version = s;

	if (strstr(boot_command_line,"hw.rf_version=11"))
		current_rf_version_num = RF_VERSION__11;
	else if (strstr(boot_command_line,"hw.rf_version=12"))
		current_rf_version_num = RF_VERSION__12;
	else if (strstr(boot_command_line,"hw.rf_version=13"))
		current_rf_version_num = RF_VERSION__13;
	else if (strstr(boot_command_line,"hw.rf_version=21"))
		current_rf_version_num = RF_VERSION__21;
	else if (strstr(boot_command_line,"hw.rf_version=22"))
		current_rf_version_num = RF_VERSION__22;
	else if (strstr(boot_command_line,"hw.rf_version=23"))
		current_rf_version_num = RF_VERSION__23;
	else if (strstr(boot_command_line,"hw.rf_version=31"))
		current_rf_version_num = RF_VERSION__31;
	else if (strstr(boot_command_line,"hw.rf_version=32"))
		current_rf_version_num = RF_VERSION__32;
	else if (strstr(boot_command_line,"hw.rf_version=33"))
		current_rf_version_num = RF_VERSION__33;
	else if (strstr(boot_command_line,"hw.rf_version=44"))
		current_rf_version_num = RF_VERSION__44;
	else if (strstr(boot_command_line,"hw.rf_version=66"))
		current_rf_version_num = RF_VERSION__66;
	else if (strstr(boot_command_line,"hw.rf_version=67"))
		current_rf_version_num = RF_VERSION__67;
	else if (strstr(boot_command_line,"hw.rf_version=76"))
		current_rf_version_num = RF_VERSION__76;
	else if (strstr(boot_command_line,"hw.rf_version=77"))
		current_rf_version_num = RF_VERSION__77;
	else if (strstr(boot_command_line,"hw.rf_version=87"))
		current_rf_version_num = RF_VERSION__87;
	else if (strstr(boot_command_line,"hw.rf_version=88"))
		current_rf_version_num = RF_VERSION__88;
	else if (strstr(boot_command_line,"hw.rf_version=89"))
		current_rf_version_num = RF_VERSION__89;
	else if (strstr(boot_command_line,"hw.rf_version=98"))
		current_rf_version_num = RF_VERSION__98;
	else if (strstr(boot_command_line,"hw.rf_version=99"))
		current_rf_version_num = RF_VERSION__99;
	else
	    current_rf_version_num = RF_VERSION_UNKNOWN;

	//printk("rf_version num %d \n",current_pcb_version_num);

	return 0;
}
static ssize_t pcb_version_show(struct kobject *kobj, struct kobj_attribute *attr,
								char *buf)
{
	return sprintf(buf, "%s\n", saved_command_line_pcb_version);
}

static struct kobj_attribute pcb_version_attr = {
	.attr = {
			.name = "pcb_version",
			.mode = 0444,
		},
	.show = pcb_version_show,
};

static ssize_t rf_version_show(struct kobject *kobj, struct kobj_attribute *attr,
								char *buf)
{
	return sprintf(buf, "%s\n", saved_command_line_rf_version);
}

static struct kobj_attribute rf_version_attr = {
	.attr = {
			.name = "rf_version",
			.mode = 0444,
		},
	.show = rf_version_show,
};
static struct kobject *systeminfo_kobj;

enum{
	MSM_BOOT_MODE__NORMAL,
	MSM_BOOT_MODE__RECOVERY = 2, //the number adapt system/core/init/init.c
	MSM_BOOT_MODE__FACTORY,
	MSM_BOOT_MODE__RF,
	MSM_BOOT_MODE__WLAN,
	MSM_BOOT_MODE__MOS,
};

static int ftm_mode = MSM_BOOT_MODE__NORMAL;

int __init  board_mfg_mode_init(void)
{
    char *substr;

    substr = strstr(boot_command_line, "hw_ftm_mode=");
     if(substr==NULL)
            return 0;
    if(substr) {
        substr += strlen("hw_ftm_mode=");
        if(strncmp(substr, "factory2", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__FACTORY;
        else if(strncmp(substr, "ftmwifi", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__WLAN;
		else if(strncmp(substr, "ftmmos", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__MOS;
        else if(strncmp(substr, "ftmrf", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__RF;
        else if(strncmp(substr, "ftmrecovery", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__RECOVERY;
    }

	pr_err("board_mfg_mode_init, " "ftm_mode=%d\n", ftm_mode);

	return 0;

}
//__setup("_ftm_mode=", board_mfg_mode_init);

int get_boot_mode(void)
{
	return ftm_mode;
}

static ssize_t ftmmode_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", ftm_mode);
}

struct kobj_attribute ftmmode_attr = {
    .attr = {"ftmmode", 0644},

    .show = &ftmmode_show,
};


#ifdef VENDOR_EDIT
/* Mobile Phone Software Dept.Driver, 2014/04/12  Add for gamma correction */
int gamma_index = 0;
int __init  board_gamma_index_init(void)
{
	if (strstr(boot_command_line," gamma_index=1"))
		gamma_index = 1;
	else if (strstr(boot_command_line," gamma_index=2"))
		gamma_index = 2;
	else if (strstr(boot_command_line," gamma_index=3"))
		gamma_index = 3;
	else if (strstr(boot_command_line," gamma_index=4"))
		gamma_index = 4;
	pr_err("board_gamma_index_init, " "gamma_index = %d\n", gamma_index);
	return 0;
}
int get_gamma_index(void)
{
	return gamma_index;
}

static ssize_t gamma_index_show(struct kobject *kobj, struct kobj_attribute *attr,
								char *buf)
{
	return sprintf(buf, "%d\n", gamma_index);
}

static struct kobj_attribute gamma_index_attr = {
	.attr = {
			.name = "gamma_index",
			.mode = 0444,
		},
	.show = gamma_index_show,
};
#endif /*VENDOR_EDIT*/


/*  2013-01-04 Van add start for ftm close modem*/
#define mdm_drv_ap2mdm_pmic_pwr_en_gpio  27

static ssize_t closemodem_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	//writing '1' to close and '0' to open
	//pr_err("closemodem buf[0] = 0x%x",buf[0]);
	switch (buf[0]) {
	case 0x30:
		break;
	case 0x31:
	//	pr_err("closemodem now");
		gpio_direction_output(mdm_drv_ap2mdm_pmic_pwr_en_gpio, 0);
		mdelay(4000);
		break;
	default:
		break;
	}

	return count;
}

struct kobj_attribute closemodem_attr = {
  .attr = {"closemodem", 0644},
  //.show = &closemodem_show,
  .store = &closemodem_store
};
static struct attribute * g[] = {
	&ftmmode_attr.attr,
	&closemodem_attr.attr,
	&pcb_version_attr.attr,
	&rf_version_attr.attr,
#ifdef VENDOR_EDIT
/* Mobile Phone Software Dept.Driver, 2014/04/12  Add for gamma_correction */
	&gamma_index_attr.attr,
#endif /*VENDOR_EDIT*/
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};
#endif/*VENDOR_EDIT*/

#ifdef CONFIG_VENDOR_EDIT
/*  2013-09-03 zhanglong add for add interface start reason and boot_mode begin */
char pwron_event[16];

static int __init start_reason_init(void)
{
    int i;
    char * substr = strstr(boot_command_line, "androidboot.startupmode=");
     if(substr==NULL)
            return 0;
    substr += strlen("androidboot.startupmode=");
    for(i=0; substr[i] != ' '; i++) {
        pwron_event[i] = substr[i];
    }
    pwron_event[i] = '\0';

    printk(KERN_INFO "%s: parse poweron reason %s\n", __func__, pwron_event);

	return 1;
}
//__setup("androidboot.startupmode=", start_reason_setup);

char boot_mode[16];
static int __init boot_mode_init(void)
{
    int i;
    char *substr = strstr(boot_command_line, "androidboot.mode=");
    if(substr==NULL)
            return 0;
    substr += strlen("androidboot.mode=");
    for(i=0; substr[i] != ' '; i++) {
        boot_mode[i] = substr[i];
    }
    boot_mode[i] = '\0';

    printk(KERN_INFO "%s: parse boot_mode is %s\n", __func__, boot_mode);
    return 1;
}
//__setup("androidboot.mode=", boot_mode_setup);
/*  2013-09-03 zhanglong add for add interface start reason and boot_mode end */
#endif //CONFIG_VENDOR_EDIT

static struct memtype_reserve msm8974_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static int msm8974_paddr_to_memtype(phys_addr_t paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm8974_reserve_info __initdata = {
	.memtype_reserve_table = msm8974_reserve_table,
	.paddr_to_memtype = msm8974_paddr_to_memtype,
};

void __init msm_8974_reserve(void)
{
	reserve_info = &msm8974_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_reserve, msm8974_reserve_table);
	msm_reserve();
}

static void __init msm8974_early_memory(void)
{
	reserve_info = &msm8974_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_hole, msm8974_reserve_table);
}

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8974_add_drivers(void)
{
	msm_smem_init();
	msm_init_modem_notifier_list();
	msm_smd_init();
	msm_rpm_driver_init();
	msm_pm_sleep_status_init();
	rpm_regulator_smd_driver_init();
	msm_spm_device_init();
	krait_power_init();
	if (of_board_is_rumi())
		msm_clock_init(&msm8974_rumi_clock_init_data);
	else
		msm_clock_init(&msm8974_clock_init_data);
	tsens_tm_init_driver();
	msm_thermal_device_init();
}

static struct of_dev_auxdata msm_hsic_host_adata[] = {
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, "msm_hsic_host", NULL),
	{}
};

static struct of_dev_auxdata msm8974_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("qcom,hsusb-otg", 0xF9A55000, \
			"msm_otg", NULL),
	OF_DEV_AUXDATA("qcom,ehci-host", 0xF9A55000, \
			"msm_ehci_host", NULL),
	OF_DEV_AUXDATA("qcom,dwc-usb3-msm", 0xF9200000, \
			"msm_dwc3", NULL),
	OF_DEV_AUXDATA("qcom,usb-bam-msm", 0xF9304000, \
			"usb_bam", NULL),
	OF_DEV_AUXDATA("qcom,spi-qup-v2", 0xF9924000, \
			"spi_qsd.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9824000, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98A4000, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9864000, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98E4000, \
			"msm_sdcc.4", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9824900, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98A4900, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9864900, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98E4900, \
			"msm_sdcc.4", NULL),
	OF_DEV_AUXDATA("qcom,msm-rng", 0xF9BFF000, \
			"msm_rng", NULL),
	OF_DEV_AUXDATA("qcom,qseecom", 0xFE806000, \
			"qseecom", NULL),
	OF_DEV_AUXDATA("qcom,mdss_mdp", 0xFD900000, "mdp.0", NULL),
	OF_DEV_AUXDATA("qcom,msm-tsens", 0xFC4A8000, \
			"msm-tsens", NULL),
	OF_DEV_AUXDATA("qcom,qcedev", 0xFD440000, \
			"qcedev.0", NULL),

	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, \
			"msm_hsic_host", NULL),
	OF_DEV_AUXDATA("qcom,hsic-smsc-hub", 0, "msm_smsc_hub",
			msm_hsic_host_adata),
	{}
};

static void __init msm8974_map_io(void)
{
	msm_map_8974_io();
}

void __init msm8974_init(void)
{
	struct of_dev_auxdata *adata = msm8974_auxdata_lookup;
    int rc = 0;

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);
#ifdef CONFIG_VENDOR_EDIT
	/*add for factory mode*/
	board_mfg_mode_init();
/*add for add interface start reason and boot_mode begin */
    start_reason_init();
    boot_mode_init();
#endif //CONFIG_VENDOR_EDIT

	msm_8974_init_gpiomux();
	regulator_has_full_constraints();
	board_dt_populate(adata);
	msm8974_add_drivers();

/* add begin for version */
#ifdef VENDOR_EDIT
	board_pcb_verison_init();
	board_rf_version_init();
    board_ddr_info_get(&saved_command_line_ddr_version);
#endif

#ifdef VENDOR_EDIT
/* Mobile Phone Software Dept.Driver, 2014/04/12  Add for gamma correction */
	board_gamma_index_init();
#endif /*VENDOR_EDIT*/


#ifdef CONFIG_VENDOR_EDIT
/*add begin for factory mode*/
	systeminfo_kobj = kobject_create_and_add("systeminfo", NULL);
	printk("songxh create systeminto node suscess!\n");
	if (systeminfo_kobj)
		rc = sysfs_create_group(systeminfo_kobj, &attr_group);
#endif //CONFIG_VENDOR_EDIT
}

#ifdef VENDOR_EDIT
// Add for ram_console device
static struct persistent_ram_descriptor msm_prd[] __initdata = {
	{
		.name = "ram_console",
		.size = SZ_1M,
	},
};

static struct persistent_ram msm_pr __initdata = {
	.descs = msm_prd,
	.num_descs = ARRAY_SIZE(msm_prd),
	//solve the problem samsung 6GB dai DDR can't boot up
	.start = /*0xE0200000,//*/PLAT_PHYS_OFFSET + SZ_1G + SZ_256M,
	.size = SZ_1M,
};
#endif  /* VENDOR_EDIT */

void __init msm8974_init_very_early(void)
{
	msm8974_early_memory();
	#ifdef VENDOR_EDIT
//Zhilong.Zhang@OnlineRd.Driver, 2013/12/03, Add for ram_console device
	persistent_ram_early_init(&msm_pr);
    #endif  /* VENDOR_EDIT */
}

static const char *msm8974_dt_match[] __initconst = {
	"qcom,msm8974",
	"qcom,apq8074",
	NULL
};

DT_MACHINE_START(MSM8974_DT, "Qualcomm MSM 8974 (Flattened Device Tree)")
	.map_io = msm8974_map_io,
	.init_irq = msm_dt_init_irq,
	.init_machine = msm8974_init,
	.handle_irq = gic_handle_irq,
	.timer = &msm_dt_timer,
	.dt_compat = msm8974_dt_match,
	.reserve = msm_8974_reserve,
	.init_very_early = msm8974_init_very_early,
	.restart = msm_restart,
	.smp = &msm8974_smp_ops,
MACHINE_END
