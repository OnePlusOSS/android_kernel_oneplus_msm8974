/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "msm_sensor_driver.h"
#include <linux/boot_mode.h>

#define SENSOR_FLASH_SUPPORTED 1
#define SENSOR_FLASH_NOT_SUPPORTED 0

static struct sensor_init_cfg_data *cfg;

static struct msm_sensor_power_setting power_setting[] = {
    {
	  .seq_type = SENSOR_GPIO,
	  .seq_val = SENSOR_GPIO_VDIG,
	  .config_val = GPIO_OUT_LOW,
	  .delay = 1,
	},

    {
	  .seq_type = SENSOR_GPIO,
	  .seq_val = SENSOR_GPIO_RESET,
	  .config_val = GPIO_OUT_LOW,
	  .delay = 1,
	},

	{
	  .seq_type = SENSOR_CLK,
	  .seq_val = SENSOR_CAM_MCLK,
	  .config_val = 24000000,
	  .delay = 10,
	},

    {
	  .seq_type = SENSOR_VREG,
	  .seq_val = CAM_VANA,
	  .config_val = 0,
	  .delay = 5,
	},

	{
	  .seq_type = SENSOR_VREG,
	  .seq_val = CAM_VIO,
	  .config_val = 0,
	  .delay = 1,
	},

    {
	  .seq_type = SENSOR_GPIO,
	  .seq_val = SENSOR_GPIO_VDIG,
	  .config_val = GPIO_OUT_HIGH,
	  .delay = 1,
	},

    {
	  .seq_type = SENSOR_GPIO,
	  .seq_val = SENSOR_GPIO_RESET,
	  .config_val = GPIO_OUT_HIGH,
	  .delay = 10,
	},

	{
	  .seq_type = SENSOR_I2C_MUX,
	  .seq_val = 0,
	  .config_val = 0,
	  .delay = 5,
	},
};
static struct msm_sensor_power_setting power_down_setting[] = {
    {
	  .seq_type = SENSOR_I2C_MUX,
	  .seq_val = 0,
	  .config_val = 0,
	  .delay = 0,
	},

	{
	  .seq_type = SENSOR_CLK,
	  .seq_val = SENSOR_CAM_MCLK,
	  .config_val = 24000000,
	  .delay = 5,
	},

	{
	  .seq_type = SENSOR_GPIO,
	  .seq_val = SENSOR_GPIO_RESET,
	  .config_val = GPIO_OUT_LOW,
	  .delay = 5,
	},

	{
	  .seq_type = SENSOR_GPIO,
	  .seq_val = SENSOR_GPIO_VDIG,
	  .config_val = GPIO_OUT_LOW,
	  .delay = 1,
	},

	{
	  .seq_type = SENSOR_VREG,
	  .seq_val = CAM_VIO,
	  .config_val = 0,
	  .delay = 0,
	},

    {
	  .seq_type = SENSOR_VREG,
	  .seq_val = CAM_VANA,
	  .config_val = 0,
	  .delay = 0,
	},
};
static struct msm_camera_sensor_slave_info sensor_slave_info = {
  /* sensor name */
  .sensor_name = "s5k3m2",
  /* Camera slot where this camera is mounted */
  .camera_id = CAMERA_0,
  /* sensor slave address */
  .slave_addr = 0x5a,
  /* sensor i2c frequency*/
  .i2c_freq_mode = I2C_FAST_MODE,
  /* sensor address type */
  .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
  /* sensor id info*/
  .sensor_id_info = {
    /* sensor id register address */
    .sensor_id_reg_addr = 0x0000,
    /* sensor id */
    .sensor_id = 0x30d2,
  },
  /* power up / down setting */
  .power_setting_array = {
    .power_setting = power_setting,
    .size = ARRAY_SIZE(power_setting),
    .power_down_setting = power_down_setting,
    .size_down = ARRAY_SIZE(power_down_setting),
  },
  .is_flash_supported = SENSOR_FLASH_SUPPORTED,
};

static int __init s5k3m2_init_module(void)
{
	int32_t rc = 0;
	if(get_boot_mode() == MSM_BOOT_MODE__FACTORY) {
    	cfg = kzalloc(sizeof(struct sensor_init_cfg_data), GFP_KERNEL);
    	if (!cfg) {
    		pr_err("failed: no memory cfg %p", NULL);
    		return -ENOMEM;
    	}
    	cfg->cfg.setting = &sensor_slave_info;
    	rc = msm_sensor_driver_probe(cfg->cfg.setting,
    			&cfg->probed_info,
    			cfg->entity_name);
        if (rc < 0) {
            pr_err("failed: msm_sensor_driver_probe rc %d", rc);
            kfree(cfg);
        }
    }
	return rc;
}

module_init(s5k3m2_init_module);
MODULE_DESCRIPTION("s5k3m2");
MODULE_LICENSE("GPL v2");
