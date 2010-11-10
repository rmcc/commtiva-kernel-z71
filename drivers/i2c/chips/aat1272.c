/*
 *     aat1272.c - AAT1272 Flash Driver
 *
 *     Copyright (C) 2009 Charles YS Huang <charlesyshuang@fihtdc.com>
 *     Copyright (C) 2008 FIH CO., Inc.
 *
 *     This program is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/i2c/aat1272.h>
#include <mach/gpio.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <mach/msm_rpcrouter.h>
#include <mach/mpp.h>
#include <linux/completion.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/hrtimer.h>
#ifndef CONFIG_ARCH_MSM_FLASHLIGHT
#include <linux/leds.h>
#endif

static unsigned mpp_19= 18;
static unsigned mpp_22= 21;

struct aat1272_driver_data {
	struct mutex aat1272_lock;
	struct i2c_client *aat1272_i2c_client;
	unsigned int HWID;
};

static struct timed_output_dev flashled = {
	.name = "flashled",
};

struct hrtimer flashled_timer;
static pid_t thread_id;
DECLARE_COMPLETION(flashled_comp);

static int aat1272_remove(struct i2c_client *client);
static int aat1272_suspend(struct i2c_client *client, pm_message_t mesg);
static int aat1272_resume(struct i2c_client *client);
static int aat1272_probe(struct i2c_client *client, const struct i2c_device_id *id);


static struct aat1272_driver_data aat1272_drvdata;

enum {
  GPIO_LOW = 0,
  GPIO_HIGH
};

static int aat1272_read( struct i2c_client *client, u8 *rxdata, int length )
{
	struct i2c_msg msgs[] =
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxdata,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxdata,
		},
	};

	if (i2c_transfer( client->adapter, msgs, 2) < 0 )
	{
		return -EIO;
	}

	return 0;
}

static int aat1272_write( struct i2c_client *client, u8 *txdata, int length )
{
	struct i2c_msg msg[] =
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	if ( i2c_transfer( client->adapter, msg, 1 ) < 0 )
	{
		return -EIO;
	}

	return 0;
}

static int aat1272_led_init(void)
{
	int ret = 0;

	return ret;
}

static struct miscdevice aat1272_device = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "aat1272",
};

/******************Driver Functions*******************/
static enum hrtimer_restart flashled_timer_func(struct hrtimer *timer)
{
	//static struct mpp *mpp_19;

	//mpp_19 = mpp_get(NULL, "mpp19");
	mutex_lock(&aat1272_drvdata.aat1272_lock);
	mpp_config_digital_out(mpp_19,MPP_CFG(MPP_DLOGIC_LVL_MSMP,
		MPP_DLOGIC_OUT_CTRL_HIGH));
	mutex_unlock(&aat1272_drvdata.aat1272_lock);

	complete(&flashled_comp);
	return HRTIMER_NORESTART;
}

static void flashled_enable(struct timed_output_dev *dev, int value)
{
	hrtimer_cancel(&flashled_timer);

	mutex_lock(&aat1272_drvdata.aat1272_lock);
	value = (value > 15000 ? 15000 : value);

	hrtimer_start(&flashled_timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	mutex_unlock(&aat1272_drvdata.aat1272_lock);

	if (hrtimer_active(&flashled_timer))
		printk(KERN_INFO "%s: TIMER running\n", __func__);

}

static int flashled_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&flashled_timer)) {
		ktime_t r = hrtimer_get_remaining(&flashled_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static int flashled_off_thread(void *arg)
{
	int ret = 0;
	//static struct mpp *mpp_19;


	daemonize("flashled_off_thread");

	while (1) {
		wait_for_completion(&flashled_comp);
		/* wait for flash on and turn off mpp */
		msleep(1000);
		//mpp_19 = mpp_get(NULL, "mpp19");
		mutex_lock(&aat1272_drvdata.aat1272_lock);
		ret=mpp_config_digital_out(mpp_19,MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			MPP_DLOGIC_OUT_CTRL_LOW));
		mutex_unlock(&aat1272_drvdata.aat1272_lock);
	}
	
    return 0;
}

static ssize_t flash_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned enable;
	
	sscanf(buf, "%3d\n", &enable);
/* FIH, Charles Huang, 2009/08/20 { */
/* [FXX_CR], Keep test code */
#ifdef CONFIG_FIH_FXX
	flashled_enable(&flashled,enable);
#else
	while(1)
	{
		//static struct mpp *mpp_19;
		//mpp_19 = mpp_get(NULL, "mpp19");
		mpp_config_digital_out(mpp_19,MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			MPP_DLOGIC_OUT_CTRL_HIGH));
		msleep(enable);
		mpp_config_digital_out(mpp_19,MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			MPP_DLOGIC_OUT_CTRL_LOW));
	}
#endif
/* } FIH, Charles Huang, 2009/08/20 */
	return count;
}

void brightness_onoff(int on)
{
	uint8_t write_cmd[2];
	int ret = 0;
	
	//static struct mpp *mpp_22;

	//mpp_22 = mpp_get(NULL, "mpp22");
	if (on==1){
		ret=mpp_config_digital_out(mpp_22,MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			MPP_DLOGIC_OUT_CTRL_HIGH));

		/* Set level */
		write_cmd[0] = 0x00;
		write_cmd[1] = 0x30;
		ret = aat1272_write(aat1272_drvdata.aat1272_i2c_client, write_cmd, sizeof(write_cmd));

		/* Enable */
		write_cmd[0] = 0x01;
		write_cmd[1] = 0x33;
		ret = aat1272_write(aat1272_drvdata.aat1272_i2c_client, write_cmd, sizeof(write_cmd));
	}else{
		/* Set level */
		write_cmd[0] = 0x00;
		write_cmd[1] = 0x00;
		ret = aat1272_write(aat1272_drvdata.aat1272_i2c_client, write_cmd, sizeof(write_cmd));

		/* Disable */
		write_cmd[0] = 0x01;
		write_cmd[1] = 0x00;
		ret = aat1272_write(aat1272_drvdata.aat1272_i2c_client, write_cmd, sizeof(write_cmd));
		ret=mpp_config_digital_out(mpp_22,MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			MPP_DLOGIC_OUT_CTRL_LOW));
	
	}
}

EXPORT_SYMBOL(brightness_onoff);

void flash_settime(int time)
{
	uint8_t write_cmd[2];
	int ret = 0;
	
	//static struct mpp *mpp_22;
	int level;

	//mpp_22 = mpp_get(NULL, "mpp22");
	if (time>0){
		ret=mpp_config_digital_out(mpp_22,MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			MPP_DLOGIC_OUT_CTRL_HIGH));
		level = 16 - time;
		/* Set level */
		write_cmd[0] = 0x00;
		write_cmd[1] = 0x30 + level;
		ret = aat1272_write(aat1272_drvdata.aat1272_i2c_client, write_cmd, sizeof(write_cmd));

	}else{
		ret=mpp_config_digital_out(mpp_22,MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			MPP_DLOGIC_OUT_CTRL_LOW));
	}
}

EXPORT_SYMBOL(flash_settime);

static ssize_t brightness_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned enable;
	
	sscanf(buf, "%3d\n", &enable);

	if (enable==1){
		brightness_onoff(1);
	}else{
		brightness_onoff(0);
	}
	return count;
}

static ssize_t flash_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t read_cmd_R00;
	uint8_t read_cmd_R01;
	int ret = 0;

	read_cmd_R00 = 0x00;
	read_cmd_R01 = 0x01;

	ret = aat1272_read(aat1272_drvdata.aat1272_i2c_client, &read_cmd_R00, 1);
	ret = aat1272_read(aat1272_drvdata.aat1272_i2c_client, &read_cmd_R01, 1);

	return (sprintf(buf,"R00=0x%x, R01=0x%x\n",read_cmd_R00, read_cmd_R01));
}

static ssize_t flash_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint8_t write_cmd_R00[2];
	uint8_t write_cmd_R01[2];
	int param1,param2;
	int ret = 0;

	sscanf(buf,"%d %d",&param1,&param2);

	write_cmd_R00[0] = 0x00;
	write_cmd_R00[1] = (uint8_t)param1;
	write_cmd_R01[0] = 0x01;
	write_cmd_R01[1] = (uint8_t)param2;


	ret = aat1272_write(aat1272_drvdata.aat1272_i2c_client, write_cmd_R00, sizeof(write_cmd_R00));
	ret = aat1272_write(aat1272_drvdata.aat1272_i2c_client, write_cmd_R01, sizeof(write_cmd_R01));

	return count;
}

DEVICE_ATTR(flashenable, 0666, NULL, flash_enable);
DEVICE_ATTR(brightness, 0666, NULL, brightness_enable);
DEVICE_ATTR(flashreg, 0666, flash_read, flash_write);


static int create_attributes(struct i2c_client *client)
{
	int rc;
	
	rc = device_create_file(&client->dev, &dev_attr_flashenable);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create flashdriver attribute \"flashenable\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}
	rc = device_create_file(&client->dev, &dev_attr_brightness);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create flashdriver attribute \"brightness\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}
	rc = device_create_file(&client->dev, &dev_attr_flashreg);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create flashdriver attribute \"flashreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}

	return rc;
}

static int remove_attributes(struct i2c_client *client)
{
	device_remove_file(&client->dev, &dev_attr_flashenable);
	device_remove_file(&client->dev, &dev_attr_brightness);
	device_remove_file(&client->dev, &dev_attr_flashreg);
	return 0;
}

static int aat1272_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
  	int ret = 0;

	aat1272_drvdata.aat1272_i2c_client	= client;
	aat1272_drvdata.HWID			= FIH_READ_HWID_FROM_SMEM();
	mutex_init(&aat1272_drvdata.aat1272_lock);

	ret = create_attributes(client);
	if (ret < 0) {
		dev_err(&client->dev, "%s: create attributes failed!! <%d>", __func__, ret);

		return ret;
	}

	hrtimer_init(&flashled_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	flashled_timer.function = flashled_timer_func;
	flashled.enable=flashled_enable;
	flashled.get_time=flashled_get_time;
	thread_id = kernel_thread(flashled_off_thread, NULL, CLONE_FS | CLONE_FILES);
	
	ret = aat1272_led_init();
	if(ret < 0) {
		dev_err(&client->dev, "aat1272 LED init failed\n");
	}

	/* Register a misc device */
	ret = misc_register(&aat1272_device);
	if(ret < 0) {
		dev_err(&client->dev, "aat1272 register failed\n");
	}
	
	return ret;
}

static int aat1272_remove(struct i2c_client *client)
{
	int ret = 0;
	
	mutex_destroy(&aat1272_drvdata.aat1272_lock);
	remove_attributes(client);
	misc_deregister(&aat1272_device);

	return ret;
}

#ifdef CONFIG_PM
static int aat1272_suspend(struct i2c_client *nLeds, pm_message_t mesg)
{
	return 0;
}

static int aat1272_resume(struct i2c_client *nLeds)
{
	return 0;
}
#else
# define aat1272_suspend NULL
# define aat1272_resume  NULL
#endif

static const struct i2c_device_id aat1272_idtable[] = {
       { "aat1272", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, aat1272_idtable);

static struct i2c_driver aat1272_driver = {
	.probe		= aat1272_probe,
	.remove		= aat1272_remove,
	.suspend  	= aat1272_suspend,
	.resume   	= aat1272_resume,
	.id_table	= aat1272_idtable,
	.driver = {
		.name = "aat1272",
		.owner = THIS_MODULE,
	},
};

#ifndef CONFIG_ARCH_MSM_FLASHLIGHT
struct led_classdev fl_lcdev;

static void fl_lcdev_brightness_set(struct led_classdev *led_cdev,
                        int brightness)
{

	uint8_t write_cmd_R00[2];
	uint8_t write_cmd_R01[2];
	uint8_t param1;
	uint8_t param2;

	if (brightness > 0 && brightness <=128) {
		param2 = 51;
		if (brightness == 1) {
			param1 = 239; // Lowest setting
		} else if (brightness == 3) {
			param1 = 0; // Highest setting
		} else {
			param1 = 120; // Halfway
		}
		brightness_onoff(1);
	} else {
		param1 = 0; param2 = 0;
	}
	write_cmd_R00[0] = 0x00;
	write_cmd_R00[1] = (uint8_t)param1;
	write_cmd_R01[0] = 0x01;
	write_cmd_R01[1] = (uint8_t)param2;

	if (!aat1272_write(aat1272_drvdata.aat1272_i2c_client, write_cmd_R00, sizeof(write_cmd_R00))) {
		aat1272_write(aat1272_drvdata.aat1272_i2c_client, write_cmd_R01, sizeof(write_cmd_R01));
	}

	if (!param2)
		brightness_onoff(0);

	return;
}
static int flashlight_probe(struct platform_device *pdev)
{
	fl_lcdev.name = pdev->name;
	fl_lcdev.brightness_set = fl_lcdev_brightness_set;
	fl_lcdev.brightness = 0;
	return led_classdev_register(&pdev->dev, &fl_lcdev);
}

static int flashlight_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&fl_lcdev);
	return 0;
}

static struct platform_driver flashlight_driver = {
    .probe      = flashlight_probe,
    .remove     = flashlight_remove,
    .driver     = {
        .name       = "flashlight",
        .owner      = THIS_MODULE,
    },
};

static int __init flashlight_init(void)
{
    return platform_driver_register(&flashlight_driver);
}

static void __exit flashlight_exit(void)
{
    platform_driver_unregister(&flashlight_driver);
}
#endif

static int __init aat1272_init(void)
{
	int ret = 0;


	ret = i2c_add_driver(&aat1272_driver);
	if (ret) {
		printk(KERN_ERR "%s: Driver registration failed, module not inserted.\n", __func__);
		goto driver_del;
	}

#ifndef CONFIG_ARCH_MSM_FLASHLIGHT
	ret = flashlight_init();
#endif
	return ret;

driver_del:
	i2c_del_driver(&aat1272_driver);
	
	return -1;
}

static void __exit aat1272_exit(void)
{
#ifndef CONFIG_ARCH_MSM_FLASHLIGHT
	flashlight_exit();
#endif
	i2c_del_driver(&aat1272_driver);
}

module_init(aat1272_init);
module_exit(aat1272_exit);

MODULE_AUTHOR( "Charles YS Huang <charlesyshuang@fihtdc.com>" );
MODULE_DESCRIPTION( "AAT1272 driver" );
MODULE_LICENSE( "GPL" );

