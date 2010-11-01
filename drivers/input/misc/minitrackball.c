/* drivers/input/misc/minitrackball.c
 *
 * Input driver for Panasonic GPIO-based "Jog Ball" (aka mini-trackball)
 * device (part # EVQWJN)
 *
 * Copyright (c) 2009 FIH
 * 
 * All source code in this file is licensed under the following license
 * except where indicated.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <asm/gpio.h>

#include <linux/jiffies.h>
#include <linux/timer.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>

#include <asm/io.h>
#include <asm/system.h>

#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>


#define ENABLE_FAKE_DEVICE		1
#if ENABLE_FAKE_DEVICE
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#endif

#define PORTING 0

#if !PORTING
extern void tca6507_Jogball(bool jogball);
extern void tca6507_Jogball_enable(bool enable);
#else
#define tca6507_Jogball(X)	printk(KERN_INFO "tca6507_Jogball(%d)\r\n", X)
#define tca6507_Jogball_enable(X)	printk(KERN_INFO "tca6507_Jogball_enable(%d)\r\n", X)
#endif

#define DEFAULT_DELTA_X 2
#define DEFAULT_DELTA_Y 2
#define SELECT_INPUT_BTN BTN_LEFT	/* This is chosen to match with
					 * the mousedev driver */

struct jog_ball_platform_data {
	unsigned gpio_plus_x;
	unsigned gpio_neg_x;
	unsigned gpio_plus_y;
	unsigned gpio_neg_y;
	unsigned gpio_select;
	int jogball_opened;
	int hw_pr2;
	int jogball_exist;
	int hwid;
	int oriHwid;
	struct	work_struct	registerJogballWork;
	struct	delayed_work disableJogballIRQWork;
	struct early_suspend early_suspend;
	int enableIrq;
	int pwState;
	int pwCtrl;
	int suspendIrq;
}; 

static struct jog_ball_platform_data myJogBallData = {
	.gpio_plus_x = 91,
	.gpio_neg_x = 88,
	.gpio_plus_y = 90,  
	.gpio_neg_y = 93,
	.gpio_select = 92,  // default for EVB
	.jogball_opened = 0,
	.hw_pr2 = 0,
	.jogball_exist = 0,
	.hwid = 0,
	.oriHwid = 0,
	.enableIrq = 0,
	.pwState = 0,
	.pwCtrl = 0,
	.suspendIrq = 0,
};

int input_jogball_exist(void) 
{
	return myJogBallData.jogball_exist;
}
EXPORT_SYMBOL(input_jogball_exist);

#define PLUS_X_GPIO  (myJogBallData.gpio_plus_x)
#define PLUS_X_IRQ   MSM_GPIO_TO_INT(PLUS_X_GPIO)
#define NEG_X_GPIO   (myJogBallData.gpio_neg_x)
#define NEG_X_IRQ    MSM_GPIO_TO_INT(NEG_X_GPIO)
#define PLUS_Y_GPIO  (myJogBallData.gpio_plus_y)
#define PLUS_Y_IRQ   MSM_GPIO_TO_INT(PLUS_Y_GPIO)
#define NEG_Y_GPIO   (myJogBallData.gpio_neg_y)
#define NEG_Y_IRQ    MSM_GPIO_TO_INT(NEG_Y_GPIO)
#define SELECT_GPIO  (myJogBallData.gpio_select)
#define SELECT_IRQ   MSM_GPIO_TO_INT(SELECT_GPIO)

#define DEBUG		1

static int delta_x = DEFAULT_DELTA_X;
static int delta_y = DEFAULT_DELTA_Y;
module_param(delta_x, int, S_IRUGO);
module_param(delta_y, int, S_IRUGO);

static int lockjogball = -1; // 1: lock screen, 0: unlock screen
module_param(lockjogball, int, S_IRUGO|S_IWUSR);

static struct input_dev *mtb_dev;
static unsigned long jogballLoadTime = 0;
static unsigned long jogballResumeTime = 0;


#define CHECK_JOGBALL_DEVICE()		\
			do {	\
				if(!myJogBallData.jogball_exist) {	\
					if(time_after(jiffies, jogballLoadTime+5*HZ)) {	\
						myJogBallData.jogball_exist = 1;\
						schedule_work(&myJogBallData.registerJogballWork);	\
					}	\
					return IRQ_HANDLED;	\
				}	\
			} while(0)

#define SEND_JOGBALL_EVENT(EVENT, DELTA)	\
			do {		\
				if (time_after(jiffies, jogballResumeTime+HZ)) { \
					if(myJogBallData.jogball_opened && lockjogball==0) {	\
					input_report_rel(mtb_dev, EVENT, DELTA);		\
					input_sync(mtb_dev);		\
				}	\
				} \
			} while(0)



static irqreturn_t plus_x_isr(int irq, void *dummy)
{
	CHECK_JOGBALL_DEVICE();

	SEND_JOGBALL_EVENT(REL_X, delta_x);
	
	return IRQ_HANDLED;
}

static irqreturn_t neg_x_isr(int irq, void *dummy)
{
	CHECK_JOGBALL_DEVICE();

	SEND_JOGBALL_EVENT(REL_X, -delta_x);

	return IRQ_HANDLED;
}

static irqreturn_t plus_y_isr(int irq, void *dummy)
{
	CHECK_JOGBALL_DEVICE();

	SEND_JOGBALL_EVENT(REL_Y, delta_y);
	
	return IRQ_HANDLED;
}

static irqreturn_t neg_y_isr(int irq, void *dummy)
{
	CHECK_JOGBALL_DEVICE();

	SEND_JOGBALL_EVENT(REL_Y, -delta_y);

	return IRQ_HANDLED;
}

static int mtb_open(struct input_dev *dev)
{
	int err = 0;

	//lockjogball = -1; // initial state: lock jogball events


	if(myJogBallData.jogball_opened == 1) {
		return 0;
	}

	myJogBallData.jogball_opened = 1;

	if(myJogBallData.enableIrq == 0) {
		//  power on jogball
		if(myJogBallData.pwCtrl) {
			tca6507_Jogball_enable(true);
			myJogBallData.pwState = 1;
		}

		err = request_irq(PLUS_X_IRQ,
				  &plus_x_isr,
				  IRQF_TRIGGER_FALLING, "MTB Plus X", NULL);
		if (err)
			goto cleanup5;

		err = request_irq(NEG_X_IRQ,
				  &neg_x_isr, IRQF_TRIGGER_FALLING, "MTB Neg X", NULL);
		if (err)
			goto cleanup4;

		err = request_irq(PLUS_Y_IRQ,
				  &plus_y_isr,
				  IRQF_TRIGGER_FALLING, "MTB Plus Y", NULL);
		if (err)
			goto cleanup3;

		err = request_irq(NEG_Y_IRQ,
				  &neg_y_isr, IRQF_TRIGGER_FALLING, "MTB Neg Y", NULL);
		if (err)
			goto cleanup2;

		myJogBallData.enableIrq = 1;
	}

	return 0;


cleanup2:
	free_irq(PLUS_Y_IRQ, NULL);
cleanup3:
	free_irq(NEG_X_IRQ, NULL);
cleanup4:
	free_irq(PLUS_X_IRQ, NULL);
cleanup5:
	
	return err;
}

static void mtb_close(struct input_dev *dev)
{

	if(myJogBallData.jogball_opened == 0) {
		return;
	}

	free_irq(PLUS_X_IRQ, NULL);
	free_irq(NEG_X_IRQ, NULL);
	free_irq(PLUS_Y_IRQ, NULL);
	free_irq(NEG_Y_IRQ, NULL);

	//  power off jogball
	if(myJogBallData.pwCtrl) {
		tca6507_Jogball_enable(false);
		myJogBallData.pwState = 0;
	}

	myJogBallData.jogball_opened = 0;
	myJogBallData.enableIrq = 0;
}

static void register_jogball_device(struct work_struct *work)
{
	mtb_dev = input_allocate_device();
	if (mtb_dev)
	{
		int err;

		mtb_dev->name = "Mini Trackball Device";
		mtb_dev->phys = "/dev/mtb";
		mtb_dev->open = mtb_open;
		mtb_dev->close = mtb_close;
		mtb_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_REL);
		set_bit(SELECT_INPUT_BTN, mtb_dev->keybit);
		mtb_dev->relbit[0] = BIT(REL_X) | BIT(REL_Y);
	
		err = input_register_device(mtb_dev);
		if (err) {
			input_free_device(mtb_dev);
		}		
	}
}

static void disable_jogball_irq(struct work_struct *work)
{
	if(!myJogBallData.jogball_exist) 
	{		
		free_irq(PLUS_X_IRQ, NULL);
		free_irq(NEG_X_IRQ, NULL);
		free_irq(PLUS_Y_IRQ, NULL);
		free_irq(NEG_Y_IRQ, NULL);

		myJogBallData.enableIrq = 0;
		myJogBallData.jogball_exist = -1;
		
		//  power off jogball
		if(myJogBallData.pwCtrl) {
			tca6507_Jogball(false);
			tca6507_Jogball_enable(false);
			myJogBallData.pwState = 0;
		}
	}
}

static ssize_t
show_jogball_state(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d-%d-%d-%d-%d-%d-%d-%d\r\n", 
					myJogBallData.jogball_exist,
					myJogBallData.jogball_opened,
					myJogBallData.hw_pr2,
					myJogBallData.hwid,
					myJogBallData.pwCtrl,
					myJogBallData.pwState,
					myJogBallData.oriHwid,
					myJogBallData.suspendIrq);
}

static DEVICE_ATTR(jogball_state, S_IRUGO, show_jogball_state, NULL);

static int gpioInitState = 0;
static ssize_t
show_jogball_gpio(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d,+Y(%d),-Y(%d),-X(%d),+X(%d)\r\n", 
					gpioInitState,
					gpio_get_value(PLUS_Y_GPIO),
					gpio_get_value(NEG_Y_GPIO),
					gpio_get_value(NEG_X_GPIO),
					gpio_get_value(PLUS_X_GPIO));
}

static DEVICE_ATTR(jogball_gpio, S_IRUGO, show_jogball_gpio, NULL);

static struct attribute *dev_attrs[] = {
        &dev_attr_jogball_state.attr,
        &dev_attr_jogball_gpio.attr,	
        NULL,
};

static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
void mtb_early_suspend(struct early_suspend *h)
{
	if(myJogBallData.jogball_opened || !myJogBallData.jogball_exist) {

		if(myJogBallData.suspendIrq == 0) {
			disable_irq(PLUS_X_IRQ);
			disable_irq(NEG_X_IRQ);
			disable_irq(PLUS_Y_IRQ);
			disable_irq(NEG_Y_IRQ);
			myJogBallData.suspendIrq = 1;
		}

		gpio_tlmm_config(GPIO_CFG(PLUS_X_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_PULL_DOWN,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(NEG_X_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_PULL_DOWN,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(PLUS_Y_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_PULL_DOWN,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(NEG_Y_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_PULL_DOWN,
					  GPIO_2MA),
				 GPIO_ENABLE);

		// power off jogball
		if(myJogBallData.pwCtrl) {
			tca6507_Jogball_enable(false);
			myJogBallData.pwState = 0;
		}
	}
}

void mtb_late_resume(struct early_suspend *h)
{
	if(myJogBallData.jogball_opened || !myJogBallData.jogball_exist) {

		gpio_tlmm_config(GPIO_CFG(PLUS_X_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(NEG_X_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(PLUS_Y_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(NEG_Y_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);

		// power on jogball
		if(myJogBallData.pwCtrl) {
			tca6507_Jogball_enable(true);
			myJogBallData.pwState = 1;
		}
		
		jogballResumeTime = jiffies;

		if(myJogBallData.suspendIrq == 1) {
			myJogBallData.suspendIrq = 0;
			enable_irq(PLUS_X_IRQ);
			enable_irq(NEG_X_IRQ);
			enable_irq(PLUS_Y_IRQ);
			enable_irq(NEG_Y_IRQ);
		}
	}
}
#endif

static int __init mtb_platform_probe(struct platform_device *pdev)
{
	int err = 0;

	myJogBallData.hwid = FIH_READ_HWID_FROM_SMEM();
	myJogBallData.oriHwid = FIH_READ_ORIG_HWID_FROM_SMEM();

	jogballLoadTime = jiffies;
	jogballResumeTime = jiffies;

	INIT_WORK(&myJogBallData.registerJogballWork, register_jogball_device);
	INIT_DELAYED_WORK(&myJogBallData.disableJogballIRQWork, disable_jogball_irq);

#ifdef CONFIG_HAS_EARLYSUSPEND
	myJogBallData.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	myJogBallData.early_suspend.suspend = mtb_early_suspend;
	myJogBallData.early_suspend.resume = mtb_late_resume;
	register_early_suspend(&myJogBallData.early_suspend);
#endif

	if(sysfs_create_group(&pdev->dev.kobj, &dev_attr_grp)) {
		printk(KERN_ERR "Create attributes in sysfs fail\r\n");
	}

	if((myJogBallData.hwid >= CMCS_RTP_PR2 && myJogBallData.hwid <= CMCS_RTP_MP3) ||
				(myJogBallData.hwid >= CMCS_7627_PR2 && myJogBallData.hwid <= CMCS_7627_PR5) ||
				(myJogBallData.hwid >= CMCS_F913_PR2 && myJogBallData.hwid <= CMCS_F913_MP1) ||
				(myJogBallData.hwid >= CMCS_CTP_PR2 && myJogBallData.hwid <= CMCS_CTP_MP3))
	{
		myJogBallData.hw_pr2 = 1;
	}

	if((myJogBallData.hwid >= CMCS_RTP_PR2 && myJogBallData.hwid <= CMCS_RTP_MP3) ||
		(myJogBallData.hwid >= CMCS_CTP_PR2 && myJogBallData.hwid <= CMCS_CTP_MP3))
	{
		myJogBallData.pwCtrl = 1;

		tca6507_Jogball_enable(false);
		myJogBallData.pwState = 0;
	}
		
	if(gpio_get_value(PLUS_Y_GPIO))
		gpioInitState |= 8;
	if(gpio_get_value(NEG_Y_GPIO))
		gpioInitState |= 4;
	if(gpio_get_value(NEG_X_GPIO))
		gpioInitState |= 2;
	if(gpio_get_value(PLUS_X_GPIO))
		gpioInitState |= 1;


	if(myJogBallData.oriHwid >= CMCS_125_FST_PR1 &&  myJogBallData.oriHwid <= CMCS_128_FST_MP1)
	{
		myJogBallData.jogball_exist = -1;
	}
	else if(myJogBallData.oriHwid >= CMCS_CTP_F917_PR1 &&  myJogBallData.oriHwid <= CMCS_CTP_F917_MP3)
	{
		myJogBallData.jogball_exist = -1;
	}
	else if(myJogBallData.hwid >= CMCS_RTP_PR2 && myJogBallData.hwid <= CMCS_RTP_MP3) // F902_pr2, F910_pr2
	{
		if(!gpio_get_value(NEG_Y_GPIO)) 
		{
			tca6507_Jogball(true);
			
			myJogBallData.jogball_exist = 1;
			schedule_work(&myJogBallData.registerJogballWork);
		}
		else 
		{
			myJogBallData.jogball_exist = -1;
		}
	}
	else if(myJogBallData.hwid >= CMCS_F913_PR1 && 
				myJogBallData.hwid < CMCS_HW_VER_MAX) 
	{
		myJogBallData.jogball_exist = -1;
	}
	else if(myJogBallData.hwid >= CMCS_7627_EVB1 &&
				myJogBallData.hwid <= CMCS_7627_PR5) 
	{
		myJogBallData.jogball_exist = 1;
		schedule_work(&myJogBallData.registerJogballWork);
	}
	else if(myJogBallData.hwid >= CMCS_CTP_PR2 && myJogBallData.hwid <= CMCS_CTP_MP3) // F903_pr2, F911_pr2
	{	
		if(myJogBallData.hwid >= CMCS_CTP_PR3) 
		{
			if(!gpio_get_value(NEG_Y_GPIO) && !gpio_get_value(PLUS_X_GPIO)) 
			{
				tca6507_Jogball(true);
				
				myJogBallData.jogball_exist = 1;
				schedule_work(&myJogBallData.registerJogballWork);
			}
			else 
			{
				myJogBallData.jogball_exist = -1;
			}
		}
		else
		{
			if(!gpio_get_value(NEG_Y_GPIO)) 
			{
				tca6507_Jogball(true);

				myJogBallData.jogball_exist = 1;
				schedule_work(&myJogBallData.registerJogballWork);
			}
			else 
			{
				myJogBallData.jogball_exist = -1;
			}
		}
	}
	else if(myJogBallData.hwid == CMCS_HW_VER_EVB1 ||
			myJogBallData.hwid == CMCS_RTP_PR1 ||
			myJogBallData.hwid == CMCS_CTP_PR1)
	{ // EVB, F902/F903/F910/F911 PR1
		gpio_tlmm_config(GPIO_CFG(PLUS_X_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(NEG_X_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(PLUS_Y_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(NEG_Y_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);

		if(!gpio_get_value(PLUS_X_GPIO) || !gpio_get_value(NEG_X_GPIO) ||
				!gpio_get_value(PLUS_Y_GPIO) || !gpio_get_value(NEG_Y_GPIO)) 
		{
			myJogBallData.jogball_exist = 1;
			schedule_work(&myJogBallData.registerJogballWork);		    
		}
		else 
		{
			myJogBallData.jogball_exist = -1; 
		}
	}
	else {
		myJogBallData.jogball_exist = -1; 
	}

	if(myJogBallData.jogball_exist == 1)
	{
		gpio_tlmm_config(GPIO_CFG(PLUS_X_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(NEG_X_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(PLUS_Y_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(NEG_Y_GPIO,
					  0, /* function */
					  GPIO_INPUT,
					  GPIO_NO_PULL,
					  GPIO_2MA),
				 GPIO_ENABLE);
	}
	else if(myJogBallData.jogball_exist == 0)
	{
		// apply power to jogball
		if(myJogBallData.pwCtrl) {
			tca6507_Jogball(true);
			tca6507_Jogball_enable(true);
			myJogBallData.pwState = 1;
		}

		//schedule_delayed_work(&myJogBallData.disableJogballIRQWork, 5*60*HZ);
		
		err = request_irq(PLUS_X_IRQ,
				  &plus_x_isr,
				  IRQF_TRIGGER_FALLING, "MTB Plus X", NULL);
		if (err)
			goto cleanup5;

		err = request_irq(NEG_X_IRQ,
				  &neg_x_isr, IRQF_TRIGGER_FALLING, "MTB Neg X", NULL);
		if (err)
			goto cleanup4;

		err = request_irq(PLUS_Y_IRQ,
				  &plus_y_isr,
				  IRQF_TRIGGER_FALLING, "MTB Plus Y", NULL);
		if (err)
			goto cleanup3;

		err = request_irq(NEG_Y_IRQ,
				  &neg_y_isr, IRQF_TRIGGER_FALLING, "MTB Neg Y", NULL);
		if (err)
			goto cleanup2;

		myJogBallData.enableIrq = 1;
	}

	return 0;

cleanup2:
	free_irq(PLUS_Y_IRQ, NULL);
cleanup3:
	free_irq(NEG_X_IRQ, NULL);
cleanup4:
	free_irq(PLUS_X_IRQ, NULL);
cleanup5:

	return err;
}

static int mtb_platform_remove(struct platform_device *pdev)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&myJogBallData.early_suspend);
#endif

	return 0;
}

/* implement suspend/resume for jogball */
/* Use early suspend instead of device suspend */
#ifdef CONFIG_PM_
static int mtb_platform_suspend(struct platform_device *pdev, pm_message_t state)
{
	if(myJogBallData.jogball_opened) {

		disable_irq(PLUS_X_IRQ);
		disable_irq(NEG_X_IRQ);
		disable_irq(PLUS_Y_IRQ);
		disable_irq(NEG_Y_IRQ);

		// power off jogball
		if(myJogBallData.pwCtrl) {
			tca6507_Jogball_enable(false);
			myJogBallData.pwState = 0;
		}
	}
	
	return 0;
}

static int mtb_platform_resume(struct platform_device *pdev)
{
	if(myJogBallData.jogball_opened) {

		// power on jogball
		if(myJogBallData.pwCtrl) {
			tca6507_Jogball_enable(true);
			myJogBallData.pwState = 1;
		}

		enable_irq(PLUS_X_IRQ);
		enable_irq(NEG_X_IRQ);
		enable_irq(PLUS_Y_IRQ);
		enable_irq(NEG_Y_IRQ);
	}

	return 0;
}
#else
#define mtb_platform_suspend NULL
#define mtb_platform_resume NULL
#endif


static struct platform_driver mtb_platform_driver = {
	.probe = mtb_platform_probe,
	.remove = mtb_platform_remove,
	/* implement suspend/resume for jogball */
	.suspend  = mtb_platform_suspend,
	.resume   = mtb_platform_resume,
	.driver = {
		.name = "mtb",
	},
};


#if ENABLE_FAKE_DEVICE
static int fake_open(struct inode *inode, struct file *file)
{
	//printk(KERN_INFO "fake_open()\r\n");
	
	return 0;
}

static int fake_release(struct inode *inode, struct file *file)
{
	//printk(KERN_INFO "fake_release()\r\n");

	return 0;
}

static ssize_t fake_read(struct file *fp, char __user *buf,
                            size_t count, loff_t *pos)
{
	int ret = 0;

	return ret;
}
                     
static ssize_t fake_write(struct file *fp, const char __user *buf,
                             size_t count, loff_t *pos)
{
	int ret = 0;
	char tmp[512+1];

	if(count > 512)
		count = 512;

	if(copy_from_user(tmp, buf, count))
	{
		ret = -EFAULT;
	}
	else
	{
		tmp[count] = 0;
		printk(KERN_INFO "\r\nTiger: %s\r\n", tmp);
		ret = count;
	}
	
	return ret;
}          


static struct file_operations fake_fops = {
	.owner = THIS_MODULE,
	.open = fake_open,
	.release = fake_release,
	.ioctl = NULL,
	.read =    fake_read,
	.write =   fake_write,	
};

static struct miscdevice fake_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "fakeDev",
	.fops = &fake_fops,
};
#endif

static __init int mtb_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&mtb_platform_driver);
	if(ret)	{
		printk(KERN_ERR "mtb_init fail\r\n");
	}

#if ENABLE_FAKE_DEVICE
	if (misc_register(&fake_device)) {
		printk(KERN_ERR "fake_device register failed\n");
	}
#endif
	
	return ret;
}

/**
*  @brief Exits mtb driver.
*
*  Deregisters input device.
*/
static void __exit mtb_exit(void)
{
	platform_driver_unregister(&mtb_platform_driver);
}

late_initcall(mtb_init);
module_exit(mtb_exit);

MODULE_DESCRIPTION("Mini-trackball driver");
MODULE_VERSION("0.2");
MODULE_LICENSE("GPL v2");
