/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>

#include "mdss_dsi.h"
#ifdef VENDOR_EDIT
#include <linux/wait.h>
#include <linux/project_info.h>
/* 2013-10-24  Add begin for panel info */
#include <mach/device_info.h>
/* 2013-10-24 Add end */
/* 2013-12-09 Add begin for disable continous display for ftm, rf, wlan mode */
#include <linux/boot_mode.h>
/* 2013-12-09 Add end */
/* 2014-02-11 add begin*/
#include <linux/pcb_version.h>
/*  2014-02-11add end */
#endif
#ifdef VENDOR_EDIT
/* Mobile Phone Software Dept.Driver, 2014/02/24  Add for ESD test */
#include <linux/switch.h>
#endif

#ifdef VENDOR_EDIT
extern  int lm3630_bank_a_update_status(u32 bl_level);
extern int push_component_info(enum COMPONENT_TYPE type, char *version, char * manufacture);
#endif

#define DT_CMD_HDR 6

#define MIN_REFRESH_RATE 30

DEFINE_LED_TRIGGER(bl_led_trigger);

void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: Error: lpg_chan=%d pwm request failed",
				__func__, ctrl->pwm_lpg_chan);
	}
}
#ifdef VENDOR_EDIT
#define WAIT_INIT_TIMEOUT 125 //Samsung s6e3fa3 panel init delay.
int mdss_dsi_start_timer(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (ctrl->wait_timeout){
        del_timer(&(ctrl->delay_timer));
        ctrl->delay_timer.expires = jiffies + msecs_to_jiffies(WAIT_INIT_TIMEOUT);
        atomic_inc(&ctrl->delay_pending);
        add_timer(&(ctrl->delay_timer));
	}

	return 0;
}

int mdss_dsi_stop_timer(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (ctrl->wait_timeout){
		del_timer(&(ctrl->delay_timer));
	}
    atomic_set(&ctrl->delay_pending, 0);

    return 0;
}

void mdss_dsi_timer_cb(unsigned long data)
{
	struct mdss_dsi_ctrl_pdata *ctrl = (struct mdss_dsi_ctrl_pdata *)data;

	if (!ctrl) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}
	atomic_set(&ctrl->delay_pending, 0);
	wake_up_all(&ctrl->delay_wait_q);
}

int mdss_dsi_wait_timeout(struct mdss_dsi_ctrl_pdata *pdata)
{
    int ret=0;

	struct mdss_dsi_ctrl_pdata *ctrl = pdata;

	if (!ctrl) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
    ret = wait_event_timeout(ctrl->delay_wait_q,
		(!atomic_read(&ctrl->delay_pending)),
		msecs_to_jiffies(WAIT_INIT_TIMEOUT));
	if (!ret) {
		pr_err("wait for timeout ret= %d pending=%d\n",
				ret, atomic_read(&ctrl->delay_pending));
	}
    atomic_set(&ctrl->delay_pending, 0);

	return ret;
}
#endif

#ifdef VENDOR_EDIT
#define ESD_TE_CHECK_ON
struct mdss_dsi_ctrl_pdata *panel_data;
bool is_samsung_s6e3fa3_panel = 0;
static bool first_run_init=1;
static bool cont_splash_flag;

#ifdef ESD_TE_CHECK_ON
#define LCD_TE_GPIO  28
unsigned long flags;
static bool first_run_reset = 1;
static int irq;
static int te_state = 0;
static struct switch_dev display_switch;
static struct delayed_work techeck_work;
static struct completion te_comp;

DEFINE_SPINLOCK(te_count_lock);
DEFINE_SPINLOCK(te_state_lock);

static irqreturn_t TE_irq_thread_fn(int irq, void *dev_id)
{
	complete(&te_comp);
	return IRQ_HANDLED;
}
static int operate_display_switch(void)
{
    int ret = 0;

    pr_err("%s : state=%d.\n", __func__, te_state);
    spin_lock_irqsave(&te_state_lock, flags);
    if(te_state)
        te_state = 0;
    else
        te_state = 1;
    spin_unlock_irqrestore(&te_state_lock, flags);

    switch_set_state(&display_switch, te_state);

    return ret;
}
static void techeck_work_func( struct work_struct *work )
{
	int ret = 0;

	INIT_COMPLETION(te_comp);
	enable_irq(irq);
    ret = wait_for_completion_killable_timeout(&te_comp,
						msecs_to_jiffies(100));
	if(ret == 0){
		disable_irq(irq);
		operate_display_switch();
		return;
	}
	disable_irq(irq);
	schedule_delayed_work(&techeck_work, msecs_to_jiffies(2000));
}


static ssize_t attr_mdss_dispswitch(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    printk("ESD function test--------\n");
    operate_display_switch();
    return 0;
}

static struct class * mdss_lcd;
static struct device * dev_lcd;
static struct device_attribute mdss_lcd_attrs[] = {
	__ATTR(dispswitch, S_IRUGO|S_IWUSR, attr_mdss_dispswitch, NULL),
	__ATTR_NULL,
	};
#endif

struct dsi_panel_cmds cabc_off_sequence;
struct dsi_panel_cmds cabc_user_interface_image_sequence;
struct dsi_panel_cmds cabc_still_image_sequence;
struct dsi_panel_cmds cabc_video_image_sequence;

struct dsi_panel_cmds gamma1;
struct dsi_panel_cmds gamma2;
struct dsi_panel_cmds gamma3;
struct dsi_panel_cmds gamma4;
extern int gamma_index ;


static bool flag_lcd_off = false;
static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds);


extern int set_backlight_pwm(int state);
enum
{
    CABC_CLOSE = 0,
    CABC_LOW_MODE,
    CABC_MIDDLE_MODE,
    CABC_HIGH_MODE,

};

int cabc_mode = CABC_HIGH_MODE; //default mode level 3 in dtsi file
static DEFINE_MUTEX(cabc_mutex);



static char dcs_cmd_oem_0[2]  = {0xb0, 0x04};
static char dcs_cmd_oem_1[20] = {0xc8, 0x01, 0x0A, 0xFD,
								   0x03, 0x01, 0xE8, 0x00,
								   0x00, 0x03, 0xFC, 0xF5,
								   0xA1, 0x00, 0x00, 0x01,
								   0xFD, 0x06, 0xFC, 0x00,};
static char dcs_cmd_oem_2[2]  = {0xd6, 0x01};
static char dcs_cmd_oem_3[2]  = {0xb0, 0x03};

static struct dsi_cmd_desc user_defined_oem_gamma[] = {
	{{DTYPE_GEN_WRITE2, 1, 0, 1, 0, sizeof(dcs_cmd_oem_0)},dcs_cmd_oem_0},
	{{DTYPE_GEN_LWRITE, 1, 0, 1, 0, sizeof(dcs_cmd_oem_1)},dcs_cmd_oem_1},
	{{DTYPE_GEN_WRITE2, 1, 0, 1, 0, sizeof(dcs_cmd_oem_2)},dcs_cmd_oem_2},
	{{DTYPE_GEN_WRITE2, 1, 0, 1, 0, sizeof(dcs_cmd_oem_3)},dcs_cmd_oem_3},
};

void send_user_defined_gamma(char * buf)
{
	int i=0,len,limt_len,temp;
	char temp_buf[100];
	char * p1,*p2,*user_gamma=NULL;
	struct dcs_cmd_req cmdreq;

	if (is_samsung_s6e3fa3_panel){
        return;
    }
	mutex_lock(&cabc_mutex);
	if(flag_lcd_off == true)
    {
        printk(KERN_INFO "lcd is off,don't allow to set user gamma !\n");
        mutex_unlock(&cabc_mutex);
        return;
    }
	if((get_pcb_version() < 20)||(get_pcb_version() >=30))
	{/*add*/
		user_gamma = dcs_cmd_oem_1;
		limt_len = sizeof(dcs_cmd_oem_1);
	}
	if(user_gamma == NULL)

	{
	mutex_unlock(&cabc_mutex); return;
	}
	p1=buf;
	p2=temp_buf;
	pr_err("%s \n",p1);
	while(*p1!='\0'){
		if(*p1==' ') {p1++;continue;}
		*p2 = *p1;
		p2++;
		p1++;
	}
	*p2 ='\0';
	p2=temp_buf;
	len =strlen(p2);
	pr_err("len = %d \n",len);
	if( len/2 >limt_len){
			 mutex_unlock(&cabc_mutex);
			 pr_err("invalid gamma intput \n");
			 return;
	}
	for(i=0;i<len;i++)
	{
		if(*p2>='0' && *p2 <='9')
			temp =*p2-'0';
		else if(*p2>='a'&& *p2<='f')
			temp =*p2-'a'+10;
		else if(*p2>='A'&& *p2<='F')
			temp =*p2-'A'+10;
		if(i%2==0)
			user_gamma[i/2] = temp*16;
		else
			user_gamma[i/2]+=temp;
		p2++;
	}
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = user_defined_oem_gamma;
	cmdreq.cmds_cnt = 4;
	cmdreq.flags = CMD_REQ_COMMIT;
	mdss_dsi_cmdlist_put(panel_data, &cmdreq);
	mutex_unlock(&cabc_mutex);
	return;
}




void set_gamma(int index)
{
	printk("%s : %d \n",__func__,index);

	if (is_samsung_s6e3fa3_panel){
        return;
    }
    mutex_lock(&cabc_mutex);

	if(flag_lcd_off == true)
    {
        printk(KERN_INFO "lcd is off,don't allow to set gamma\n");
        mutex_unlock(&cabc_mutex);
        return;
    }
  //  mdss_dsi_clk_ctrl(panel_data, 1);
	if(index <= 0 || index >4){
		mutex_unlock(&cabc_mutex);
        return;
	}
	switch(index)
    {
		case 1:
			 mdss_dsi_panel_cmds_send(panel_data, &gamma1);
			 break;
		case 2:
			 mdss_dsi_panel_cmds_send(panel_data, &gamma2);
			 break;
		case 3:

			 mdss_dsi_panel_cmds_send(panel_data, &gamma3);
			 break;
		case 4:
			 mdss_dsi_panel_cmds_send(panel_data, &gamma4);
			 break;
	}
//	mdss_dsi_clk_ctrl(panel_data, 0);
    mutex_unlock(&cabc_mutex);

}

void set_resume_gamma(int index)
{
	pr_debug("%s : %d \n",__func__,index);
	if (is_samsung_s6e3fa3_panel){
        return;
    }
   if(index <= 1 || index >4){
        return;
	}
    switch(index)
    {
		case 1:
			 mdss_dsi_panel_cmds_send(panel_data, &gamma1);
			 break;
		case 2:
			 mdss_dsi_panel_cmds_send(panel_data, &gamma2);
			 break;
		case 3:
			 mdss_dsi_panel_cmds_send(panel_data, &gamma3);
			 break;
		case 4:
			 mdss_dsi_panel_cmds_send(panel_data, &gamma4);
			 break;
		default:
			pr_err("%s : invalid gamma index %d  \n",__func__,index);
			break;
	}
}

int set_cabc(int level)
{
    int ret = 0;

	if (is_samsung_s6e3fa3_panel){
        return 0;
    }

	printk("%s : %d \n",__func__,level);
    mutex_lock(&cabc_mutex);
	if(flag_lcd_off == true)
    {
        printk(KERN_INFO "lcd is off,don't allow to set cabc\n");
        cabc_mode = level;
        mutex_unlock(&cabc_mutex);
        return 0;
    }

  //  mdss_dsi_clk_ctrl(panel_data, 1);
    switch(level)
    {
        case 0:
            set_backlight_pwm(0);
			 mdss_dsi_panel_cmds_send(panel_data, &cabc_off_sequence);
            cabc_mode = CABC_CLOSE;
            break;
        case 1:
            mdss_dsi_panel_cmds_send(panel_data, &cabc_user_interface_image_sequence);
            cabc_mode = CABC_LOW_MODE;
			set_backlight_pwm(1);
            break;
        case 2:
            mdss_dsi_panel_cmds_send(panel_data, &cabc_still_image_sequence);
            cabc_mode = CABC_MIDDLE_MODE;
			set_backlight_pwm(1);
            break;
        case 3:
            mdss_dsi_panel_cmds_send(panel_data, &cabc_video_image_sequence);
            cabc_mode = CABC_HIGH_MODE;
			set_backlight_pwm(1);
            break;
        default:
            pr_err("%s Leavel %d is not supported!\n",__func__,level);
            ret = -1;
            break;
    }
  //  mdss_dsi_clk_ctrl(panel_data, 0);
    mutex_unlock(&cabc_mutex);
    return ret;

}

static int set_cabc_resume_mode(int mode)
{
    int ret;

	if (is_samsung_s6e3fa3_panel){
        return 0;
    }
	printk("%s : %d  \n",__func__,mode);
    switch(mode)
    {
        case 0:
            set_backlight_pwm(0);
			mdss_dsi_panel_cmds_send(panel_data, &cabc_off_sequence);
            break;
        case 1:
            mdss_dsi_panel_cmds_send(panel_data, &cabc_user_interface_image_sequence);
			set_backlight_pwm(1);
            break;
        case 2:
            mdss_dsi_panel_cmds_send(panel_data, &cabc_still_image_sequence);
			set_backlight_pwm(1);
            break;
        case 3:
           mdss_dsi_panel_cmds_send(panel_data, &cabc_video_image_sequence);
		   set_backlight_pwm(1);
            break;
        default:
            pr_err("%s  %d is not supported!\n",__func__,mode);
            ret = -1;
            break;
    }
    return ret;
}

enum
{
    ACL_LEVEL_0 = 0,
    ACL_LEVEL_1,
    ACL_LEVEL_2,
    ACL_LEVEL_3,

};
int acl_mode = ACL_LEVEL_1; //default mode level 0

static char set_acl[2] = {0x55, 0x0};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc set_acl_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(set_acl)},
	set_acl
};
void set_acl_mode(int level)
{
	struct dcs_cmd_req cmdreq;

	if (!is_samsung_s6e3fa3_panel){
        return;
    }
	pr_err("%s: level=%d\n", __func__, level);
	if(level < 0 || level > 3){
		pr_err("%s: invalid input %d! \n",__func__,level);
		return;
	}
	mutex_lock(&cabc_mutex);
	acl_mode = level;
	if(flag_lcd_off == true)
    {
        printk(KERN_INFO "lcd is off,don't allow to set acl mode !\n");
        mutex_unlock(&cabc_mutex);
        return;
    }
	set_acl[1] = (unsigned char)level;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &set_acl_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(panel_data, &cmdreq);
	mutex_unlock(&cabc_mutex);
}

static int send_samsung_fit_cmd(struct dsi_cmd_desc * cmd , int count)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = cmd;
	cmdreq.cmds_cnt = count;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	return mdss_dsi_cmdlist_put(panel_data, &cmdreq);
}

static int set_acl_resume_mode(int level)
{
	if (!is_samsung_s6e3fa3_panel){
        return 0;
    }
	pr_err("%s: level=%d\n", __func__, level);
	set_acl[1] = (unsigned char)level;
	return send_samsung_fit_cmd(&set_acl_cmd,1);
}

int hbm_mode=0;
static char set_hbm[2] = {0x53, 0x20};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc set_hbm_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(set_hbm)},
	set_hbm
};
void set_hbm_mode(int level)
{
	struct dcs_cmd_req cmdreq;

	if (!is_samsung_s6e3fa3_panel){
        return;
    }
	pr_debug("%s: level=%d\n", __func__, level);
	if(level < 0 || level > 1){
		pr_err("%s: invalid input %d! \n",__func__,level);
		return;
	}
	mutex_lock(&cabc_mutex);
	if(flag_lcd_off == true)
    {
        printk(KERN_INFO "lcd is off,don't allow to set hbm !\n");
        mutex_unlock(&cabc_mutex);
        return;
    }

    hbm_mode = level;
	if(level == 0)
		set_hbm[1] = 0x20;
	else
		set_hbm[1] = 0xe0;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &set_hbm_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(panel_data, &cmdreq);
	mutex_unlock(&cabc_mutex);
}
#endif

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;
	u32 period_ns;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	if (level == 0) {
		if (ctrl->pwm_enabled)
			pwm_disable(ctrl->pwm_bl);
		ctrl->pwm_enabled = 0;
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	if (ctrl->pwm_enabled) {
		pwm_disable(ctrl->pwm_bl);
		ctrl->pwm_enabled = 0;
	}

	if (ctrl->pwm_period >= USEC_PER_SEC) {
		ret = pwm_config_us(ctrl->pwm_bl, duty, ctrl->pwm_period);
		if (ret) {
			pr_err("%s: pwm_config_us() failed err=%d.\n",
					__func__, ret);
			return;
		}
	} else {
		period_ns = ctrl->pwm_period * NSEC_PER_USEC;
		ret = pwm_config(ctrl->pwm_bl,
				level * period_ns / ctrl->bklt_max,
				period_ns);
		if (ret) {
			pr_err("%s: pwm_config() failed err=%d.\n",
					__func__, ret);
			return;
		}
	}

	ret = pwm_enable(ctrl->pwm_bl);
	if (ret)
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);
	ctrl->pwm_enabled = 1;
}

static char dcs_cmd[2] = {0x54, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

u32 mdss_dsi_panel_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl, char cmd0,
		char cmd1, void (*fxn)(int), char *rbuf, int len)
{
	struct dcs_cmd_req cmdreq;

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = len;
	cmdreq.rbuf = rbuf;
	cmdreq.cb = fxn; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	/*
	 * blocked here, until call back called
	 */

	return 0;
}

static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static char led_pwm1[2] = {0x51, 0x0};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc backlight_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(led_pwm1)},
	led_pwm1
};

static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: level=%d\n", __func__, level);

	led_pwm1[1] = (unsigned char)level;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		rc = gpio_request(ctrl_pdata->disp_en_gpio,
						"disp_enable");
		if (rc) {
			pr_err("request disp_en gpio failed, rc=%d\n",
				       rc);
			goto disp_en_gpio_err;
		}
	}
	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}
	if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
		rc = gpio_request(ctrl_pdata->mode_gpio, "panel_mode");
		if (rc) {
			pr_err("request panel mode gpio failed,rc=%d\n",
								rc);
			goto mode_gpio_err;
		}
	}
	return rc;

mode_gpio_err:
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:
	if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
		gpio_free(ctrl_pdata->disp_en_gpio);
disp_en_gpio_err:
	return rc;
}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
	#ifndef VENDOR_EDIT
		if (!pinfo->panel_power_on)
	#else
        if (!pinfo->cont_splash_enabled)
	#endif
	    {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
				gpio_set_value((ctrl_pdata->disp_en_gpio), 1);

			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep(pinfo->rst_seq[i] * 1000);
			}
		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			gpio_free(ctrl_pdata->disp_en_gpio);
		}
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		gpio_free(ctrl_pdata->rst_gpio);
		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}
	return rc;
}

#ifdef VENDOR_EDIT
int mdss_dsi_panel_vci_en(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

    if (!gpio_is_valid(ctrl_pdata->vci_en_gpio)) {
		pr_debug("%s:%d, vci_en_gpio line not configured\n",
			   __func__, __LINE__);
		return rc;
	}
	pr_debug("%s: vci_en_gpio enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
	    rc = gpio_request(ctrl_pdata->vci_en_gpio,
					"vci_enable");
		if (rc) {
			pr_err("request vci_enable gpio failed, rc=%d\n",
				       rc);
			return rc;
		}
		if (!pinfo->panel_power_on)
		{
            //power on vci
            if (gpio_is_valid(ctrl_pdata->vci_en_gpio)){
                gpio_set_value((ctrl_pdata->vci_en_gpio), 1);
            }
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->vci_en_gpio)) {
			gpio_set_value((ctrl_pdata->vci_en_gpio), 0);
			gpio_free(ctrl_pdata->vci_en_gpio);
		}
	}
	return rc;
}
#endif

static char caset[] = {0x2a, 0x00, 0x00, 0x03, 0x00};	/* DTYPE_DCS_LWRITE */
static char paset[] = {0x2b, 0x00, 0x00, 0x05, 0x00};	/* DTYPE_DCS_LWRITE */

static struct dsi_cmd_desc partial_update_enable_cmd[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(caset)}, caset},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(paset)}, paset},
};

static int mdss_dsi_panel_partial_update(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct dcs_cmd_req cmdreq;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	caset[1] = (((pdata->panel_info.roi_x) & 0xFF00) >> 8);
	caset[2] = (((pdata->panel_info.roi_x) & 0xFF));
	caset[3] = (((pdata->panel_info.roi_x - 1 + pdata->panel_info.roi_w)
								& 0xFF00) >> 8);
	caset[4] = (((pdata->panel_info.roi_x - 1 + pdata->panel_info.roi_w)
								& 0xFF));
	partial_update_enable_cmd[0].payload = caset;

	paset[1] = (((pdata->panel_info.roi_y) & 0xFF00) >> 8);
	paset[2] = (((pdata->panel_info.roi_y) & 0xFF));
	paset[3] = (((pdata->panel_info.roi_y - 1 + pdata->panel_info.roi_h)
								& 0xFF00) >> 8);
	paset[4] = (((pdata->panel_info.roi_y - 1 + pdata->panel_info.roi_h)
								& 0xFF));
	partial_update_enable_cmd[1].payload = paset;

	pr_debug("%s: enabling partial update\n", __func__);
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = partial_update_enable_cmd;
	cmdreq.cmds_cnt = 2;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return rc;
}

static void mdss_dsi_panel_switch_mode(struct mdss_panel_data *pdata,
							int mode)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mipi_panel_info *mipi;
	struct dsi_panel_cmds *pcmds;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	mipi  = &pdata->panel_info.mipi;

	if (!mipi->dynamic_switch_enabled)
		return;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (mode == DSI_CMD_MODE)
		pcmds = &ctrl_pdata->video2cmd;
	else
		pcmds = &ctrl_pdata->cmd2video;

	mdss_dsi_panel_cmds_send(ctrl_pdata, pcmds);

	return;
}

static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
#ifdef VENDOR_EDIT
    if(is_samsung_s6e3fa3_panel){
    }else{
		lm3630_bank_a_update_status(bl_level);
		return;
	}
#endif
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	/*
	 * Some backlight controllers specify a minimum duty cycle
	 * for the backlight brightness. If the brightness is less
	 * than it, the controller can malfunction.
	 */

	if ((bl_level < pdata->panel_info.bl_min) && (bl_level != 0))
		bl_level = pdata->panel_info.bl_min;

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
		if (mdss_dsi_is_master_ctrl(ctrl_pdata)) {
			struct mdss_dsi_ctrl_pdata *sctrl =
				mdss_dsi_get_slave_ctrl();
			if (!sctrl) {
				pr_err("%s: Invalid slave ctrl data\n",
					__func__);
				return;
			}
			mdss_dsi_panel_bklt_dcs(sctrl, bl_level);
		}
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
}

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

#ifdef VENDOR_EDIT
	if (ctrl->on_cmds.cmd_cnt){
        mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);
        set_resume_gamma(gamma_index);
        if (is_samsung_s6e3fa3_panel){
            mdss_dsi_start_timer(pdata);
        }
    }
#else
	if (ctrl->on_cmds.cmd_cnt)
	    mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);
#endif
#ifdef VENDOR_EDIT
/* Mobile Phone Software Dept.Driver, 2014/02/17  Add for set cabc */
    if (!is_samsung_s6e3fa3_panel){
	    set_backlight_pwm(1);
	}
	if(cabc_mode != CABC_HIGH_MODE){
		set_cabc_resume_mode(cabc_mode);
	}
	if(acl_mode != ACL_LEVEL_1){
		set_acl_resume_mode(acl_mode);
	}
	mutex_lock(&cabc_mutex);
	flag_lcd_off = false;
	mutex_unlock(&cabc_mutex);
#endif /*VENDOR_EDIT*/

#ifdef VENDOR_EDIT
/* Mobile Phone Software Dept.Driver, 2014/02/25  Add for ESD test */
#ifdef ESD_TE_CHECK_ON
	if(get_boot_mode() != MSM_BOOT_MODE__FACTORY){
		if(first_run_reset==1 && !cont_splash_flag){
			first_run_reset=0;
		}
		else{
			schedule_delayed_work(&techeck_work, msecs_to_jiffies(5000));
		}
	}
#endif
#endif /*VENDOR_EDIT*/
	pr_debug("%s:-\n", __func__);
	return 0;
}

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
#ifdef VENDOR_EDIT
	mutex_lock(&cabc_mutex);
	flag_lcd_off = true;
	mutex_unlock(&cabc_mutex);
#endif
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);
#ifdef VENDOR_EDIT
/* Mobile Phone Software Dept.Driver, 2014/02/25  Add for ESD test */
#ifdef ESD_TE_CHECK_ON
	if(get_boot_mode() != MSM_BOOT_MODE__FACTORY){
		cancel_delayed_work_sync(&techeck_work);
	}
#endif
#endif /*VENDOR_EDIT*/
	mipi  = &pdata->panel_info.mipi;
#ifdef VENDOR_EDIT
	if (ctrl->off_cmds.cmd_cnt){
	    if (is_samsung_s6e3fa3_panel){
            mdss_dsi_stop_timer(pdata);
        }
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);
	}
#else
	if (ctrl->off_cmds.cmd_cnt)
        mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);
#endif
	pr_debug("%s:-\n", __func__);
	return 0;
}

static void mdss_dsi_parse_lane_swap(struct device_node *np, char *dlane_swap)
{
	const char *data;

	*dlane_swap = DSI_LANE_MAP_0123;
	data = of_get_property(np, "qcom,mdss-dsi-lane-map", NULL);
	if (data) {
		if (!strcmp(data, "lane_map_3012"))
			*dlane_swap = DSI_LANE_MAP_3012;
		else if (!strcmp(data, "lane_map_2301"))
			*dlane_swap = DSI_LANE_MAP_2301;
		else if (!strcmp(data, "lane_map_1230"))
			*dlane_swap = DSI_LANE_MAP_1230;
		else if (!strcmp(data, "lane_map_0321"))
			*dlane_swap = DSI_LANE_MAP_0321;
		else if (!strcmp(data, "lane_map_1032"))
			*dlane_swap = DSI_LANE_MAP_1032;
		else if (!strcmp(data, "lane_map_2103"))
			*dlane_swap = DSI_LANE_MAP_2103;
		else if (!strcmp(data, "lane_map_3210"))
			*dlane_swap = DSI_LANE_MAP_3210;
	}
}

static void mdss_dsi_parse_trigger(struct device_node *np, char *trigger,
		char *trigger_key)
{
	const char *data;

	*trigger = DSI_CMD_TRIGGER_SW;
	data = of_get_property(np, trigger_key, NULL);
	if (data) {
		if (!strcmp(data, "none"))
			*trigger = DSI_CMD_TRIGGER_NONE;
		else if (!strcmp(data, "trigger_te"))
			*trigger = DSI_CMD_TRIGGER_TE;
		else if (!strcmp(data, "trigger_sw_seof"))
			*trigger = DSI_CMD_TRIGGER_SW_SEOF;
		else if (!strcmp(data, "trigger_sw_te"))
			*trigger = DSI_CMD_TRIGGER_SW_TE;
	}
}


static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len > sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	/*Set default link state to LP Mode*/
	pcmds->link_state = DSI_LP_MODE;

	if (link_key) {
		data = of_get_property(np, link_key, NULL);
		if (data && !strcmp(data, "dsi_hs_mode"))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	}

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}


int mdss_panel_get_dst_fmt(u32 bpp, char mipi_mode, u32 pixel_packing,
				char *dst_format)
{
	int rc = 0;
	switch (bpp) {
	case 3:
		*dst_format = DSI_CMD_DST_FORMAT_RGB111;
		break;
	case 8:
		*dst_format = DSI_CMD_DST_FORMAT_RGB332;
		break;
	case 12:
		*dst_format = DSI_CMD_DST_FORMAT_RGB444;
		break;
	case 16:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB565;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		}
		break;
	case 18:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB666;
			break;
		default:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		}
		break;
	case 24:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB888;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}


static int mdss_dsi_parse_fbc_params(struct device_node *np,
				struct mdss_panel_info *panel_info)
{
	int rc, fbc_enabled = 0;
	u32 tmp;

	fbc_enabled = of_property_read_bool(np,	"qcom,mdss-dsi-fbc-enable");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		panel_info->fbc.enabled = 1;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bpp", &tmp);
		panel_info->fbc.target_bpp =	(!rc ? tmp : panel_info->bpp);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-packing",
				&tmp);
		panel_info->fbc.comp_mode = (!rc ? tmp : 0);
		panel_info->fbc.qerr_enable = of_property_read_bool(np,
			"qcom,mdss-dsi-fbc-quant-error");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bias", &tmp);
		panel_info->fbc.cd_bias = (!rc ? tmp : 0);
		panel_info->fbc.pat_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-pat-mode");
		panel_info->fbc.vlc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-vlc-mode");
		panel_info->fbc.bflc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-bflc-mode");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-h-line-budget",
				&tmp);
		panel_info->fbc.line_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-budget-ctrl",
				&tmp);
		panel_info->fbc.block_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-block-budget",
				&tmp);
		panel_info->fbc.block_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossless-threshold", &tmp);
		panel_info->fbc.lossless_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-threshold", &tmp);
		panel_info->fbc.lossy_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-rgb-threshold",
				&tmp);
		panel_info->fbc.lossy_rgb_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-mode-idx", &tmp);
		panel_info->fbc.lossy_mode_idx = (!rc ? tmp : 0);
	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		panel_info->fbc.enabled = 0;
		panel_info->fbc.target_bpp =
			panel_info->bpp;
	}
	return 0;
}

static void mdss_panel_parse_te_params(struct device_node *np,
				       struct mdss_panel_info *panel_info)
{

	u32 tmp;
	int rc = 0;
	/*
	 * TE default: dsi byte clock calculated base on 70 fps;
	 * around 14 ms to complete a kickoff cycle if te disabled;
	 * vclk_line base on 60 fps; write is faster than read;
	 * init == start == rdptr;
	 */
	panel_info->te.tear_check_en =
		!of_property_read_bool(np, "qcom,mdss-tear-check-disable");
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-cfg-height", &tmp);
	panel_info->te.sync_cfg_height = (!rc ? tmp : 0xfff0);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-init-val", &tmp);
	panel_info->te.vsync_init_val = (!rc ? tmp : panel_info->yres);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-start", &tmp);
	panel_info->te.sync_threshold_start = (!rc ? tmp : 4);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-continue", &tmp);
	panel_info->te.sync_threshold_continue = (!rc ? tmp : 4);
	rc = of_property_read_u32(np, "qcom,mdss-tear-check-start-pos", &tmp);
	panel_info->te.start_pos = (!rc ? tmp : panel_info->yres);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-rd-ptr-trigger-intr", &tmp);
	panel_info->te.rd_ptr_irq = (!rc ? tmp : panel_info->yres + 1);
	rc = of_property_read_u32(np, "qcom,mdss-tear-check-frame-rate", &tmp);
	panel_info->te.refx100 = (!rc ? tmp : 6000);
}


static int mdss_dsi_parse_reset_seq(struct device_node *np,
		u32 rst_seq[MDSS_DSI_RST_SEQ_LEN], u32 *rst_len,
		const char *name)
{
	int num = 0, i;
	int rc;
	struct property *data;
	u32 tmp[MDSS_DSI_RST_SEQ_LEN];
	*rst_len = 0;
	data = of_find_property(np, name, &num);
	num /= sizeof(u32);
	if (!data || !num || num > MDSS_DSI_RST_SEQ_LEN || num % 2) {
		pr_debug("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, num);
	} else {
		rc = of_property_read_u32_array(np, name, tmp, num);
		if (rc)
			pr_debug("%s:%d, error reading %s, rc = %d\n",
				__func__, __LINE__, name, rc);
		else {
			for (i = 0; i < num; ++i)
				rst_seq[i] = tmp[i];
			*rst_len = num;
		}
	}
	return 0;
}

static void mdss_dsi_parse_roi_alignment(struct device_node *np,
		struct mdss_panel_info *pinfo)
{
	int len = 0;
	u32 value[6];
	struct property *data;
	data = of_find_property(np, "qcom,panel-roi-alignment", &len);
	len /= sizeof(u32);
	if (!data || (len != 6)) {
		pr_err("%s: Panel roi alignment not found", __func__);
	} else {
		int rc = of_property_read_u32_array(np,
				"qcom,panel-roi-alignment", value, len);
		if (rc)
			pr_err("%s: Error reading panel roi alignment values",
					__func__);
		else {
			pinfo->xstart_pix_align = value[0];
			pinfo->width_pix_align = value[1];
			pinfo->ystart_pix_align = value[2];
			pinfo->height_pix_align = value[3];
			pinfo->min_width = value[4];
			pinfo->min_height = value[5];
		}

		pr_info("%s: ROI alignment: [%d, %d, %d, %d, %d, %d]",
				__func__, pinfo->xstart_pix_align,
				pinfo->width_pix_align, pinfo->ystart_pix_align,
				pinfo->height_pix_align, pinfo->min_width,
				pinfo->min_height);
	}
}

static int mdss_dsi_parse_panel_features(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct mdss_panel_info *pinfo;

	if (!np || !ctrl) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

	pinfo = &ctrl->panel_data.panel_info;

	pinfo->cont_splash_enabled = of_property_read_bool(np,
		"qcom,cont-splash-enabled");

	pinfo->partial_update_enabled = of_property_read_bool(np,
		"qcom,partial-update-enabled");
	pr_info("%s:%d Partial update %s\n", __func__, __LINE__,
		(pinfo->partial_update_enabled ? "enabled" : "disabled"));
	if (pinfo->partial_update_enabled)
		ctrl->partial_update_fnc = mdss_dsi_panel_partial_update;

	pinfo->ulps_feature_enabled = of_property_read_bool(np,
		"qcom,ulps-enabled");
	pr_info("%s: ulps feature %s", __func__,
		(pinfo->ulps_feature_enabled ? "enabled" : "disabled"));
	pinfo->esd_check_enabled = of_property_read_bool(np,
		"qcom,esd-check-enabled");

	pinfo->mipi.dynamic_switch_enabled = of_property_read_bool(np,
		"qcom,dynamic-mode-switch-enabled");

	if (pinfo->mipi.dynamic_switch_enabled) {
		mdss_dsi_parse_dcs_cmds(np, &ctrl->video2cmd,
			"qcom,video-to-cmd-mode-switch-commands", NULL);

		mdss_dsi_parse_dcs_cmds(np, &ctrl->cmd2video,
			"qcom,cmd-to-video-mode-switch-commands", NULL);

		if (!ctrl->video2cmd.cmd_cnt || !ctrl->cmd2video.cmd_cnt) {
			pr_warn("No commands specified for dynamic switch\n");
			pinfo->mipi.dynamic_switch_enabled = 0;
		}
	}

	pr_info("%s: dynamic switch feature enabled: %d", __func__,
		pinfo->mipi.dynamic_switch_enabled);

	return 0;
}

static int mdss_dsi_set_refresh_rate_range(struct device_node *pan_node,
		struct mdss_panel_info *pinfo)
{
	int rc = 0;
	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-min-refresh-rate",
			&pinfo->min_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read min refresh rate\n",
				__func__, __LINE__);

		/*
		 * Since min refresh rate is not specified when dynamic
		 * fps is enabled, using minimum as 30
		 */
		pinfo->min_fps = MIN_REFRESH_RATE;
		rc = 0;
	}

	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-max-refresh-rate",
			&pinfo->max_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read max refresh rate\n",
				__func__, __LINE__);

		/*
		 * Since max refresh rate was not specified when dynamic
		 * fps is enabled, using the default panel refresh rate
		 * as max refresh rate supported.
		 */
		pinfo->max_fps = pinfo->mipi.frame_rate;
		rc = 0;
	}

	pr_info("dyn_fps: min = %d, max = %d\n",
			pinfo->min_fps, pinfo->max_fps);
	return rc;
}

static void mdss_dsi_parse_dfps_config(struct device_node *pan_node,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	const char *data;
	bool dynamic_fps;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	dynamic_fps = of_property_read_bool(pan_node,
			"qcom,mdss-dsi-pan-enable-dynamic-fps");

	if (!dynamic_fps)
		return;

	pinfo->dynamic_fps = true;
	data = of_get_property(pan_node, "qcom,mdss-dsi-pan-fps-update", NULL);
	if (data) {
		if (!strcmp(data, "dfps_suspend_resume_mode")) {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("dfps mode: suspend/resume\n");
		} else if (!strcmp(data, "dfps_immediate_clk_mode")) {
			pinfo->dfps_update = DFPS_IMMEDIATE_CLK_UPDATE_MODE;
			pr_debug("dfps mode: Immediate clk\n");
		} else if (!strcmp(data, "dfps_immediate_porch_mode")) {
			pinfo->dfps_update = DFPS_IMMEDIATE_PORCH_UPDATE_MODE;
			pr_debug("dfps mode: Immediate porch\n");
		} else {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("default dfps mode: suspend/resume\n");
		}
		mdss_dsi_set_refresh_rate_range(pan_node, pinfo);
	} else {
		pinfo->dynamic_fps = false;
		pr_debug("dfps update mode not configured: disable\n");
	}
	pinfo->new_fps = pinfo->mipi.frame_rate;

	return;
}

static int mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32 tmp;
	int rc, i, len;
	const char *data;
	static const char *pdest;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-width", &tmp);
	if (rc) {
		pr_err("%s:%d, panel width not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->xres = (!rc ? tmp : 640);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-height", &tmp);
	if (rc) {
		pr_err("%s:%d, panel height not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->yres = (!rc ? tmp : 480);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-width-dimension", &tmp);
	pinfo->physical_width = (!rc ? tmp : 0);
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-height-dimension", &tmp);
	pinfo->physical_height = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-left-border", &tmp);
	pinfo->lcdc.xres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-right-border", &tmp);
	if (!rc)
		pinfo->lcdc.xres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-top-border", &tmp);
	pinfo->lcdc.yres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-bottom-border", &tmp);
	if (!rc)
		pinfo->lcdc.yres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, bpp not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pinfo->bpp = (!rc ? tmp : 24);
	pinfo->mipi.mode = DSI_VIDEO_MODE;
	data = of_get_property(np, "qcom,mdss-dsi-panel-type", NULL);
	if (data && !strncmp(data, "dsi_cmd_mode", 12))
		pinfo->mipi.mode = DSI_CMD_MODE;
	tmp = 0;
	data = of_get_property(np, "qcom,mdss-dsi-pixel-packing", NULL);
	if (data && !strcmp(data, "loose"))
		pinfo->mipi.pixel_packing = 1;
	else
		pinfo->mipi.pixel_packing = 0;
	rc = mdss_panel_get_dst_fmt(pinfo->bpp,
		pinfo->mipi.mode, pinfo->mipi.pixel_packing,
		&(pinfo->mipi.dst_format));
	if (rc) {
		pr_debug("%s: problem determining dst format. Set Default\n",
			__func__);
		pinfo->mipi.dst_format =
			DSI_VIDEO_DST_FORMAT_RGB888;
	}
	pdest = of_get_property(np,
		"qcom,mdss-dsi-panel-destination", NULL);

	if (pdest) {
		if (strlen(pdest) != 9) {
			pr_err("%s: Unknown pdest specified\n", __func__);
			return -EINVAL;
		}
		if (!strcmp(pdest, "display_1"))
			pinfo->pdest = DISPLAY_1;
		else if (!strcmp(pdest, "display_2"))
			pinfo->pdest = DISPLAY_2;
		else {
			pr_debug("%s: incorrect pdest. Set Default\n",
				__func__);
			pinfo->pdest = DISPLAY_1;
		}
	} else {
		pr_debug("%s: pdest not specified. Set Default\n",
				__func__);
		pinfo->pdest = DISPLAY_1;
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch", &tmp);
	pinfo->lcdc.h_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch", &tmp);
	pinfo->lcdc.h_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &tmp);
	pinfo->lcdc.h_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-sync-skew", &tmp);
	pinfo->lcdc.hsync_skew = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch", &tmp);
	pinfo->lcdc.v_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch", &tmp);
	pinfo->lcdc.v_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &tmp);
	pinfo->lcdc.v_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-underflow-color", &tmp);
	pinfo->lcdc.underflow_clr = (!rc ? tmp : 0xff);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-border-color", &tmp);
	pinfo->lcdc.border_clr = (!rc ? tmp : 0);
	pinfo->bklt_ctrl = UNKNOWN_CTRL;
	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strncmp(data, "bl_ctrl_wled", 12)) {
			led_trigger_register_simple("bkl-trigger",
				&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
				__func__);
			ctrl_pdata->bklt_ctrl = BL_WLED;
		} else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
			ctrl_pdata->bklt_ctrl = BL_PWM;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_period = tmp;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-bank-select", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, dsi lpg channel\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_lpg_chan = tmp;
			tmp = of_get_named_gpio(np,
				"qcom,mdss-dsi-pwm-gpio", 0);
			ctrl_pdata->pwm_pmic_gpio = tmp;
		} else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
			ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
		}
	}
	rc = of_property_read_u32(np, "qcom,mdss-brightness-max-level", &tmp);
	pinfo->brightness_max = (!rc ? tmp : MDSS_MAX_BL_BRIGHTNESS);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level", &tmp);
	pinfo->bl_min = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level", &tmp);
	pinfo->bl_max = (!rc ? tmp : 255);
	ctrl_pdata->bklt_max = pinfo->bl_max;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-interleave-mode", &tmp);
	pinfo->mipi.interleave_mode = (!rc ? tmp : 0);

	pinfo->mipi.vsync_enable = of_property_read_bool(np,
		"qcom,mdss-dsi-te-check-enable");
	pinfo->mipi.hw_vsync_mode = of_property_read_bool(np,
		"qcom,mdss-dsi-te-using-te-pin");

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-h-sync-pulse", &tmp);
	pinfo->mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	pinfo->mipi.hfp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hfp-power-mode");
	pinfo->mipi.hsa_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hsa-power-mode");
	pinfo->mipi.hbp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hbp-power-mode");
	pinfo->mipi.last_line_interleave_en = of_property_read_bool(np,
		"qcom,mdss-dsi-last-line-interleave");
	pinfo->mipi.bllp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-bllp-power-mode");
	pinfo->mipi.eof_bllp_power_stop = of_property_read_bool(
		np, "qcom,mdss-dsi-bllp-eof-power-mode");
	pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	data = of_get_property(np, "qcom,mdss-dsi-traffic-mode", NULL);
	if (data) {
		if (!strcmp(data, "non_burst_sync_event"))
			pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
		else if (!strcmp(data, "burst_mode"))
			pinfo->mipi.traffic_mode = DSI_BURST_MODE;
	}
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-dcs-command", &tmp);
	pinfo->mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-continue", &tmp);
	pinfo->mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-start", &tmp);
	pinfo->mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-pin-select", &tmp);
	pinfo->mipi.te_sel =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-virtual-channel-id", &tmp);
	pinfo->mipi.vc = (!rc ? tmp : 0);
	pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	data = of_get_property(np, "qcom,mdss-dsi-color-order", NULL);
	if (data) {
		if (!strcmp(data, "rgb_swap_rbg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RBG;
		else if (!strcmp(data, "rgb_swap_bgr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BGR;
		else if (!strcmp(data, "rgb_swap_brg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BRG;
		else if (!strcmp(data, "rgb_swap_grb"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GRB;
		else if (!strcmp(data, "rgb_swap_gbr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GBR;
	}
	pinfo->mipi.data_lane0 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-0-state");
	pinfo->mipi.data_lane1 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-1-state");
	pinfo->mipi.data_lane2 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-2-state");
	pinfo->mipi.data_lane3 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-3-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-pre", &tmp);
	pinfo->mipi.t_clk_pre = (!rc ? tmp : 0x24);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-post", &tmp);
	pinfo->mipi.t_clk_post = (!rc ? tmp : 0x03);

	pinfo->mipi.rx_eot_ignore = of_property_read_bool(np,
		"qcom,mdss-dsi-rx-eot-ignore");
	pinfo->mipi.tx_eot_append = of_property_read_bool(np,
		"qcom,mdss-dsi-tx-eot-append");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-stream", &tmp);
	pinfo->mipi.stream = (!rc ? tmp : 0);

	data = of_get_property(np, "qcom,mdss-dsi-panel-mode-gpio-state", NULL);
	if (data) {
		if (!strcmp(data, "high"))
			pinfo->mode_gpio_state = MODE_GPIO_HIGH;
		else if (!strcmp(data, "low"))
			pinfo->mode_gpio_state = MODE_GPIO_LOW;
	} else {
		pinfo->mode_gpio_state = MODE_GPIO_NOT_VALID;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-framerate", &tmp);
	pinfo->mipi.frame_rate = (!rc ? tmp : 60);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-clockrate", &tmp);
	pinfo->clk_rate = (!rc ? tmp : 0);
	data = of_get_property(np, "qcom,mdss-dsi-panel-timings", &len);
	if ((!data) || (len != 12)) {
		pr_err("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		pinfo->mipi.dsi_phy_db.timing[i] = data[i];

	pinfo->mipi.lp11_init = of_property_read_bool(np,
					"qcom,mdss-dsi-lp11-init");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-init-delay-us", &tmp);
	pinfo->mipi.init_delay = (!rc ? tmp : 0);

	mdss_dsi_parse_fbc_params(np, pinfo);
	mdss_dsi_parse_roi_alignment(np, pinfo);

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.mdp_trigger),
		"qcom,mdss-dsi-mdp-trigger");

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.dma_trigger),
		"qcom,mdss-dsi-dma-trigger");

	mdss_dsi_parse_lane_swap(np, &(pinfo->mipi.dlane_swap));

	mdss_dsi_parse_reset_seq(np, pinfo->rst_seq, &(pinfo->rst_seq_len),
		"qcom,mdss-dsi-reset-sequence");
	mdss_panel_parse_te_params(np, pinfo);

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_cmds,
		"qcom,mdss-dsi-on-command", "qcom,mdss-dsi-on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->off_cmds,
		"qcom,mdss-dsi-off-command", "qcom,mdss-dsi-off-command-state");


#ifdef VENDOR_EDIT
/* Mobile Phone Software Dept.Driver, 2014/02/17  Add for set cabc */
    if (is_samsung_s6e3fa3_panel){
    }else{
	mdss_dsi_parse_dcs_cmds(np, &cabc_off_sequence,
		"qcom,mdss-dsi-cabc-off-command", "qcom,mdss-dsi-off-command-state");
	mdss_dsi_parse_dcs_cmds(np, &cabc_user_interface_image_sequence,
		"qcom,mdss-dsi-cabc-ui-command", "qcom,mdss-dsi-off-command-state");
	mdss_dsi_parse_dcs_cmds(np, &cabc_still_image_sequence,
		"qcom,mdss-dsi-cabc-still-image-command", "qcom,mdss-dsi-off-command-state");
	mdss_dsi_parse_dcs_cmds(np, &cabc_video_image_sequence,
		"qcom,mdss-dsi-cabc-video-command", "qcom,mdss-dsi-off-command-state");

        mdss_dsi_parse_dcs_cmds(np, &gamma1,
            "qcom,mdss-dsi-gamma1", "qcom,mdss-dsi-off-command-state");
        mdss_dsi_parse_dcs_cmds(np, &gamma2,
            "qcom,mdss-dsi-gamma2", "qcom,mdss-dsi-off-command-state");
        mdss_dsi_parse_dcs_cmds(np, &gamma3,
            "qcom,mdss-dsi-gamma3", "qcom,mdss-dsi-off-command-state");
        mdss_dsi_parse_dcs_cmds(np, &gamma4,
            "qcom,mdss-dsi-gamma4", "qcom,mdss-dsi-off-command-state");
    }
#endif /*VENDOR_EDIT*/
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->status_cmds,
			"qcom,mdss-dsi-panel-status-command",
				"qcom,mdss-dsi-panel-status-command-state");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-status-value", &tmp);
	ctrl_pdata->status_value = (!rc ? tmp : 0);


	ctrl_pdata->status_mode = ESD_MAX;
	rc = of_property_read_string(np,
				"qcom,mdss-dsi-panel-status-check-mode", &data);
	if (!rc) {
		if (!strcmp(data, "bta_check"))
			ctrl_pdata->status_mode = ESD_BTA;
		else if (!strcmp(data, "reg_read"))
			ctrl_pdata->status_mode = ESD_REG;
	}

	rc = mdss_dsi_parse_panel_features(np, ctrl_pdata);
	if (rc) {
		pr_err("%s: failed to parse panel features\n", __func__);
		goto error;
	}

	mdss_dsi_parse_dfps_config(np, ctrl_pdata);

	return 0;

error:
	return -EINVAL;
}

int mdss_dsi_panel_init(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	bool cmd_cfg_cont_splash)
{
	int rc = 0;
	static const char *panel_name;
	struct mdss_panel_info *pinfo;

#ifdef VENDOR_EDIT
	bool cont_splash_enabled;
	/* 2013-10-24 Add begin for panel info */
	static const char *panel_manufacture;
	static const char *panel_version;
	/* 2013-10-24 Add end */
#endif
	if (!node || !ctrl_pdata) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

#ifdef VENDOR_EDIT
	/* Mobile Phone Software Dept.Driver, 2014/02/17  Add*/
    if (first_run_init == 1){
		panel_data = ctrl_pdata;
	}
#endif /*VENDOR_EDIT*/
	pinfo = &ctrl_pdata->panel_data.panel_info;

	pr_debug("%s:%d\n", __func__, __LINE__);
	panel_name = of_get_property(node, "qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name)
		pr_info("%s:%d, Panel name not specified\n",
						__func__, __LINE__);
	else
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);

#ifdef VENDOR_EDIT
	/* 2013-10-24 Add begin for panel info */
		/*it just need to do one time*/
	if (first_run_init == 1){
		panel_manufacture = of_get_property(node, "qcom,mdss-dsi-panel-manufacture", NULL);
		if (!panel_manufacture)
			pr_info("%s:%d, panel manufacture not specified\n", __func__, __LINE__);
		else
			pr_info("%s: Panel Manufacture = %s\n", __func__, panel_manufacture);
		panel_version = of_get_property(node, "qcom,mdss-dsi-panel-version", NULL);
		if (!panel_version)
			pr_info("%s:%d, panel version not specified\n", __func__, __LINE__);
		else
			pr_info("%s: Panel Version = %s\n", __func__, panel_version);
		register_device_proc("lcd", (char *)panel_version, (char *)panel_manufacture);
		push_component_info(LCD, (char *)panel_version, (char *)panel_manufacture);
	    if(strstr(panel_version, "S6E3FA3")){
		    register_device_proc("backlight", (char *)panel_version, (char *)panel_manufacture);
		    push_component_info(BACKLIGHT, (char *)panel_version, (char *)panel_manufacture);
		    is_samsung_s6e3fa3_panel = 1;
	    }
	}
	/* 2013-10-24 Add end */
#endif
#ifdef VENDOR_EDIT
	/* Mobile Phone Software Dept.Driver, 2014/02/22  Add for ESD test*/
		if (first_run_init==1 && get_boot_mode() != MSM_BOOT_MODE__FACTORY){
			first_run_init=0;
    #ifdef ESD_TE_CHECK_ON
            init_completion(&te_comp);
			irq = gpio_to_irq(LCD_TE_GPIO); //gpio 28 has configed in mdss_dsi.c
			rc = request_threaded_irq(irq, NULL, TE_irq_thread_fn,
				IRQF_TRIGGER_RISING, "LCD_TE",NULL);
			if (rc < 0) {
				pr_err("Unable to register IRQ handler\n");
				return -ENODEV;
			}
			disable_irq(irq);
			INIT_DELAYED_WORK(&techeck_work, techeck_work_func );
			schedule_delayed_work(&techeck_work, msecs_to_jiffies(20000));

			display_switch.name = "dispswitch";
			rc = switch_dev_register(&display_switch);
			if (rc)
			{
				pr_err("Unable to register display switch device\n");
				return rc;
			}
			/*dir: /sys/class/mdss_lcd/lcd_control*/
			mdss_lcd = class_create(THIS_MODULE,"mdss_lcd");
			mdss_lcd->dev_attrs = mdss_lcd_attrs;
			device_create(mdss_lcd,dev_lcd,0,NULL,"lcd_control");
	#endif
        }
#endif /*VENDOR_EDIT*/
	rc = mdss_panel_parse_dt(node, ctrl_pdata);
	if (rc) {
		pr_err("%s:%d panel dt parse failed\n", __func__, __LINE__);
		return rc;
	}

	if (!cmd_cfg_cont_splash)
		pinfo->cont_splash_enabled = false;
	/* 2013-12-09 Add begin for disable continous display for ftm, rf, wlan mode */
#ifdef VENDOR_EDIT
	if (cmd_cfg_cont_splash)
		cont_splash_enabled = of_property_read_bool(node,
				"qcom,cont-splash-enabled");
	else
		cont_splash_enabled = false;

		if ((MSM_BOOT_MODE__FACTORY == get_boot_mode()) ||
			(MSM_BOOT_MODE__RF == get_boot_mode()) ||
			(MSM_BOOT_MODE__WLAN == get_boot_mode()) ||
			(MSM_BOOT_MODE__MOS == get_boot_mode())) {
			cont_splash_enabled = false;
		}
#endif
	/* 2013-12-09 Add end */
#ifdef VENDOR_EDIT
	/* Mobile Phone Software Dept.Driver, 2014/02/25  Add for ESD test */
		cont_splash_flag = cont_splash_enabled;
#endif /*VENDOR_EDIT*/
	pr_info("%s: Continuous splash %s", __func__,
		pinfo->cont_splash_enabled ? "enabled" : "disabled");

	pinfo->dynamic_switch_pending = false;
	pinfo->is_lpm_mode = false;

	ctrl_pdata->on = mdss_dsi_panel_on;
	ctrl_pdata->off = mdss_dsi_panel_off;
	ctrl_pdata->panel_data.set_backlight = mdss_dsi_panel_bl_ctrl;
	ctrl_pdata->switch_mode = mdss_dsi_panel_switch_mode;

#ifdef VENDOR_EDIT
	ctrl_pdata->delay_timer.function = mdss_dsi_timer_cb;
	ctrl_pdata->delay_timer.data = (unsigned long)ctrl_pdata;
	ctrl_pdata->wait_timeout= mdss_dsi_wait_timeout;
	atomic_set(&ctrl_pdata->delay_pending, 0);
	init_timer(&ctrl_pdata->delay_timer);
	init_waitqueue_head(&ctrl_pdata->delay_wait_q);
#endif

	return 0;
}
