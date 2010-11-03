/*
 * Driver for CM3602A30P Proximity and Light Sensor chip whick works on MSM7225/MSM7227.
 * 
 * Copyright 2008 Hanson Lin and James Tsai, FIH/CMCS
 *      
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *          CM3602_PR1_PS_GPIO_OUT
 * Version 1.0 2009/05/18
 * -basic support:
 *                     
 */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>

/* FIH, Henry Juang, 2009/11/20 ++*/
/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
#include <linux/elan_i2c.h>
/* FIH, Henry Juang, 2009/11/20 --*/

#include <asm/uaccess.h> 
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include "LPSCM3602.h"
#include <asm/signal.h>
#include <linux/interrupt.h>

#include <mach/gpio.h>
#include <linux/interrupt.h>

//FIH, HenryJuang 2009/11/11 ++
/* Enable Proximaty wake source.*/
#include <linux/input.h>

#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>

// FIH;NicoleWeng;2010/6/01 { 
//#define DISABLE_PROXIMITY //if not support PS, open this define (ex:FM6)
#define FAST_DATA_PATH // if hal and framework has changed to support this, open this define (Tiger implement)
// } FIH;NicoleWeng;2010/6/01 

int g_activate;
/* cm3602_ioctl: used to cotrol cm3602_ *PS_On  : it will set the ping as output and pull its status low
 *CM3602_PS_On  : it will set the ping as output and pull its status high
 *CM3602_PS_Off : it will set the ping as output and pull its status low
 *CM3602_ALS_On : it will set the ping as output and pull its status high
 *CM3602_ALS_Off: it will set the ping as output and pull its status low
 *return: 0 or EINVEL
 * */
int HWID = 0;
#ifndef CMCS_FM6_PR1
int CMCS_FM6_PR1=0x100;
#endif
int cm3602_pid =0;
struct task_struct * cm3602_task_struct = NULL;
int g_CM3602_PS_GPIO_VALUE;
int g_CM3602_EN_GPIO_VALUE;
extern struct pid *find_pid(int nr);
extern int proc_comm_read_adie_adc(unsigned *cmd_parameter);

static int isFQC_Testing=0;

static char enablePS = 0;
static char enableALS = 0;

/*  FIH_ADQ, Hanson Lin { */
/*  CM3602_ALSPS ADC Read Function */
adie_adc_config cm3602_adie_adc_config = {
	.muxsel = ADIE_AD_AIN1,
	.res = ADIE_ADC_RES_12_BITS,
};
int msm_cm3602_read_adc(void)
{
	unsigned smem_response;
	/*cmd_parameter setting */
	unsigned cmd_parameter[2];
	cmd_parameter[0] = cm3602_adie_adc_config.muxsel;
	cmd_parameter[1] = cm3602_adie_adc_config.res;

	smem_response = proc_comm_read_adie_adc(cmd_parameter);
	return smem_response;
}
EXPORT_SYMBOL(msm_cm3602_read_adc);
/* } FIH_ADQ, Hanson Lin */


#ifndef DISABLE_PROXIMITY 
static int cm3602_irq=-1;
static int flag_cm3602_irq=0;
static int isCM3602Suspend=0;
#ifdef FAST_DATA_PATH

#include <linux/moduleparam.h>

static int set_fih_in_call(const char *val, struct kernel_param *kp)
{
	char *endp;
	int  l;
	int  rv = 0;

	if (!val)
		return -EINVAL;
	l = simple_strtoul(val, &endp, 0);
	if (endp == val)
		return -EINVAL;

	*((int *)kp->arg) = l;

	if(l == 0) {
		notify_from_proximity(0);
	}

	return rv;
}

static int fih_in_call = 0; 
module_param_call(fih_in_call, set_fih_in_call, param_get_int, &fih_in_call, 0644);
static char fih_proximity_level = -1;

#else
// FIH, Henry Juang, 2009/11/20 ++
// [FXX_CR], Add for proximity driver to turn on/off BL and TP. 
#include <linux/workqueue.h>
extern int Q7x27_kybd_proximity_irqsetup(void);
extern int Proximity_Flag_Set(int flag);
struct workqueue_struct *proximity_wq;
struct work_struct proximity_work;

static int touch_last_flag=1;

static void proximaty_cb(struct work_struct *w){
	int level;
	
	if (HWID == CMCS_HW_VER_EVB1)
	{
		gpio_direction_input(CM3602_EVB1_PS_GPIO_OUT);
		level = gpio_get_value(CM3602_EVB1_PS_GPIO_OUT);
	}
	else
	{
		gpio_direction_input(CM3602_PR1_PS_GPIO_OUT);
		level = gpio_get_value(CM3602_PR1_PS_GPIO_OUT);
	}	
	
//#if 1
	if(level==0 && touch_last_flag==1){
	//Call Stanley
		if(!Proximity_Flag_Set(1)){
			printk(KERN_ERR "*******CM3602: ALS notifies BL failed.\n");
		}
		if(!notify_from_proximity(1)){
			printk(KERN_ERR "*******CM3602: ALS notifies TP failed.\n");
		}
	}

	if(level==1 && touch_last_flag==0){
	//Call Stanley
		if(!Proximity_Flag_Set(0)){
			printk(KERN_ERR "*******CM3602: ALS notifies BL failed.\n");
		}
		if(!notify_from_proximity(0)){
			printk(KERN_ERR "*******CM3602: ALS notifies Touch panel failed.\n");
		}
//		Q7x27_kybd_hookswitch_irqsetup(0);
		Q7x27_kybd_proximity_irqsetup();
	}
	touch_last_flag=level;
//#endif

	printk("proximaty_cb finish.\n");

}
// FIH, Henry Juang, 2009/11/20 --
#endif
#endif


static int ALSPS_panic_handler(struct notifier_block *this,
			       unsigned long event, void *unused)
{
	int level;
	if (HWID == CMCS_HW_VER_EVB1)
	{
		gpio_direction_input(CM3602_EVB1_PS_GPIO_OUT);
		level = gpio_get_value(CM3602_EVB1_PS_GPIO_OUT);
	}
	else
	{
		gpio_direction_input(CM3602_PR1_PS_GPIO_OUT);
		level = gpio_get_value(CM3602_PR1_PS_GPIO_OUT);
	}
	return 0;
}




static int cm3602_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg){
	int value;
	//int rc;	


	switch (cmd) {
		case CM3602_PS_OFF: //For CM3602 proximity sensor switch off.
#ifndef DISABLE_PROXIMITY 
			/* FIH;Tiger;2010/4/10 { */
		#ifdef FAST_DATA_PATH
			fih_proximity_level = -1;	
			/* fast data path */		
			cm3602_pid = 0;			
		#endif
			/* } FIH;Tiger;2010/4/12 */
			enablePS = 0;			
			if(!enableALS) gpio_direction_output(CM3602_EN_GPIO,1);
			gpio_direction_output(CM3602_PS_GPIO,1);
			
			if(flag_cm3602_irq==1)
			{				
				disable_irq(cm3602_irq);
				flag_cm3602_irq=0;
			}
		#ifndef FAST_DATA_PATH
			if(!Proximity_Flag_Set(0)){
				printk(KERN_ERR "*******CM3602: ALS notifies BL failed.\n");
			}
			if(!notify_from_proximity(0)){
				printk(KERN_ERR "*******CM3602: ALS notifies Touch panel failed.\n");
			}
		#endif
#endif
			return 0;			
		case CM3602_PS_ON: //For CM3602 proximity sensor switch on.
#ifndef DISABLE_PROXIMITY 
			if ((HWID >= CMCS_RTP_PR2) && (HWID <= CMCS_RTP_MP3)){
				return 0;			
			}
			
			/* FIH;Tiger;2010/4/12 { */
			/* fast data path */
		#ifdef FAST_DATA_PATH
			cm3602_task_struct = current;
			cm3602_pid = current->pid;			
		#endif
			/* } FIH;Tiger;2010/4/12 */
			enablePS = 1;
			gpio_direction_output(CM3602_EN_GPIO,0);
			gpio_direction_output(CM3602_PS_GPIO,0);
			
			if(flag_cm3602_irq==0)
			{
				enable_irq(cm3602_irq);	
				flag_cm3602_irq=1;
			}
#endif
			return 0;		
		case CM3602_ALS_ON: //For CM3602 light sensor switch on.		
			enableALS = 1;	
			gpio_direction_output(CM3602_EN_GPIO,0);
			return 0;			
		case CM3602_ALS_OFF: //For CM3602 light sensor switch off.		
			enableALS = 0;						
			if(enablePS) return 0;		
			gpio_direction_output(CM3602_EN_GPIO,1);
#ifndef DISABLE_PROXIMITY 
			gpio_direction_output(CM3602_PS_GPIO,1);
#endif
			return 0;
			break;
		case CM3602_ALS_READ:   //Read Light Sensor Value after ADC
			/* TODO */
			value = msm_cm3602_read_adc();	
			//value = micco_read_ACD_IN5();
			return value;
			
			/* Send Signal to process */
		case CM3602_PS_ON_SIGNAL:			
#ifndef DISABLE_PROXIMITY 
			{
				// fin pid struct
				struct task_struct * tsk;
				gpio_direction_output(CM3602_PS_GPIO,0);

				cm3602_pid = ( int )arg;
				
				for_each_process( tsk )
				if( tsk->pid == cm3602_pid && cm3602_pid != 0 )
				{
					cm3602_task_struct = tsk;
				}
			}
#endif
			break;

		case CM3602_PS_READ:
#ifndef DISABLE_PROXIMITY 
			gpio_direction_output(CM3602_PS_GPIO,0);

			if (HWID == CMCS_HW_VER_EVB1)
				value = gpio_get_value(CM3602_EVB1_PS_GPIO_OUT);
			else
				value = gpio_get_value(CM3602_PR1_PS_GPIO_OUT);
			return value;
#endif
			break;
		case CM3602_ACTIVATE:
			g_activate = arg;
			return 1;
		case CM3602_Proximity_Status:
#ifndef DISABLE_PROXIMITY 
			if ((HWID >= CMCS_RTP_PR2) && ((HWID <= CMCS_RTP_MP3)))
				return 0;
			else
				return 1;
#endif
			break;
/* FIH, Henry Juang, 2009/11/20 ++*/
/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
		case CM3602_FQC_Testing:
			isFQC_Testing=arg;
			return 0;
/* FIH, Henry Juang, 2009/11/20 --*/
		case CM3602_OFF: //For CM3602 whole switch off.			
			enableALS = 0;
			enablePS = 0;	
			gpio_direction_output(CM3602_EN_GPIO,1);
#ifndef DISABLE_PROXIMITY 
			gpio_direction_output(CM3602_PS_GPIO,1);
			#endif
return 0;		
		default:
			return -EINVAL;
	}

	gpio_free(CM3602_EN_GPIO);
	gpio_free(CM3602_PS_GPIO);
	return 0;
}

#ifndef DISABLE_PROXIMITY 
//FIH, HenryJuang 2009/11/11 ++
/* Enable Proximaty wake source.*/
static irqreturn_t cm3602_isr( int irq, void * dev_id)
{
#ifdef FAST_DATA_PATH
/* FIH;Tiger;2010/4/12 { */
/* implement fast path for ALS/PS */	
	
	int level;
	
	if (HWID == CMCS_HW_VER_EVB1)
	{
		gpio_direction_input(CM3602_EVB1_PS_GPIO_OUT);
		level = gpio_get_value(CM3602_EVB1_PS_GPIO_OUT);
	}
	else
	{
		gpio_direction_input(CM3602_PR1_PS_GPIO_OUT);
		level = gpio_get_value(CM3602_PR1_PS_GPIO_OUT);
	}
	
	fih_proximity_level = level;		
	if(fih_in_call) {
		if(fih_proximity_level == 1) {
			notify_from_proximity(0);
		}
		else if(fih_proximity_level == 0) {
			notify_from_proximity(1);
		}
	}
	
	
	if(cm3602_pid) {
		struct siginfo info;
		info.si_signo = SIGUSR1;
		info.si_code = (int)fih_proximity_level;

		send_sig_info(SIGUSR1, &info, cm3602_task_struct);
	}
	/* } FIH;Tiger;2010/4/12 */
#else		
	
	/* FIH, Henry Juang, 2009/11/20 ++*/
	/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
	if(!isFQC_Testing){
		queue_work(proximity_wq, &proximity_work);
	}
#endif
/* FIH, Henry Juang, 2009/11/20 --*/
	return IRQ_HANDLED;
}
//FIH, HenryJuang 2009/11/11 --
#endif

 /*cm3602_read_ps: read the status of ps
 *0: port status is low 
 *1: port status is high
 * return: 0 or EFAULT
 * */ 

static ssize_t cm3602_read_ps(struct file *file, char *buf, size_t count, loff_t *ofs) 
{	
	int level=1;
	char *st;
#ifndef DISABLE_PROXIMITY 
#ifdef FAST_DATA_PATH
	if(fih_proximity_level == (char)-1) {
		if (HWID == CMCS_HW_VER_EVB1)
		{
			gpio_direction_input(CM3602_EVB1_PS_GPIO_OUT);
			level = gpio_get_value(CM3602_EVB1_PS_GPIO_OUT);
		}
		else
		{
			gpio_direction_input(CM3602_PR1_PS_GPIO_OUT);
			level = gpio_get_value(CM3602_PR1_PS_GPIO_OUT);
		}

		fih_proximity_level = level;
	}
	else {
		level = fih_proximity_level;
	}
#else	
	if (HWID == CMCS_HW_VER_EVB1)
	{
		gpio_direction_input(CM3602_EVB1_PS_GPIO_OUT);
		level = gpio_get_value(CM3602_EVB1_PS_GPIO_OUT);
	}
	else
	{
		gpio_direction_input(CM3602_PR1_PS_GPIO_OUT);
		level = gpio_get_value(CM3602_PR1_PS_GPIO_OUT);
	}
#endif
#endif
	st=kmalloc(sizeof(char)*2,GFP_KERNEL);

	if(enableALS) {
		st[0] = msm_cm3602_read_adc();
		if (HWID >= CMCS_CTP_PR1 && HWID!=CMCS_FM6_PR1)
			st[0] = st[0] * 23/100;		
	}
	else {
		st[0] = (char)-1;		
	}
	
	if(enablePS) {
		st[1] = level;
	}
	else {
		st[1] = 1;
	}	
	
	if(copy_to_user(buf, st,sizeof(char)*2)){
		kfree(st);
		return -EFAULT;
	}
	kfree(st);
	return 0;
}

//seq_fs -- for proc interface

/*cm3602_seq_show_ps: show cm3602 ps status: High/Low
 *get_cm3602_ps_status():
 *	5 means gpio group number, 8 means the bit number in the correspoding group
 */
static int cm3602_seq_show_ps(struct seq_file *f, void *v)
{
	if (HWID == CMCS_HW_VER_EVB1)
		gpio_direction_input(CM3602_EVB1_PS_GPIO_OUT);
	else
		gpio_direction_input(CM3602_PR1_PS_GPIO_OUT);
	seq_printf(f,"=== CM3602 ALS AND PS Status ===\n");
	seq_printf(f,"PS function: %s (GPIO:%d)\n", get_cm3602_ps_status(CM3602_PS_GPIO), CM3602_PS_GPIO);
	seq_printf(f,"=== CM3602 PS Out ===\n");
	if (HWID == CMCS_HW_VER_EVB1)
		seq_printf(f,"PS level: %s (GPIO:%d)\n", get_cm3602_ps_out(CM3602_EVB1_PS_GPIO_OUT), CM3602_EVB1_PS_GPIO_OUT);
	else
		seq_printf(f,"PS level: %s (GPIO:%d)\n", get_cm3602_ps_out(CM3602_PR1_PS_GPIO_OUT), CM3602_PR1_PS_GPIO_OUT);
	/* TODO: return light sensor value */
	seq_printf(f,"ALS level: %x\n", msm_cm3602_read_adc() );

	return 0;
}

static int cm3602_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int cm3602_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, &cm3602_seq_show_ps, NULL);
}

static ssize_t cm3602_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char msg[ 16];

	if( len > 16)
		len = 16;

	if( copy_from_user( msg, buff, len))
		return -EFAULT;

	if( strncmp( "poweron", msg, 7) == 0)
	{
		/* Proximity power on */
		gpio_direction_output(CM3602_EN_GPIO, 0);
		gpio_direction_output(CM3602_PS_GPIO, 0);
	}
	else if( strncmp( "poweroff", msg, 8) == 0)
	{
		/* Proximity power off */
		gpio_direction_output(CM3602_PS_GPIO, 1);
	}

	return len;
}

static int __init sensor_probe(struct platform_device *pdev)
{
	int ret;	
#ifndef DISABLE_PROXIMITY
#ifndef FAST_DATA_PATH
//FIH, HenryJuang 2009/11/11 ++
/* Enable Proximaty wake source.*/
	proximity_wq = create_singlethread_workqueue("proximaty_work");
	INIT_WORK(&proximity_work, proximaty_cb);
//FIH, HenryJuang 2009/11/11 --
#endif
#endif
	ret = misc_register(&cm3602_alsps_dev);
	if (ret){
		printk(KERN_WARNING "CM3602 ALSPS Unable to register misc device.\n");
		return ret;
	}
	
	return 0;
}
static int sensor_remove(struct platform_device *pdev)
{
	return 0;
}
static int sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifndef DISABLE_PROXIMITY
//FIH, HenryJuang 2009/11/11 ++
/* Enable Proximaty wake source.*/
	if (!((HWID >= CMCS_RTP_PR2) && ((HWID <= CMCS_RTP_MP3)))){
	if (HWID == CMCS_HW_VER_EVB1)
		enable_irq_wake(MSM_GPIO_TO_INT(CM3602_EVB1_PS_GPIO_OUT));
	else
		enable_irq_wake(MSM_GPIO_TO_INT(CM3602_PR1_PS_GPIO_OUT));	
	isCM3602Suspend=1;
	}
//FIH, HenryJuang 2009/11/11 --
#endif
	return 0;
}
static int sensor_resume(struct platform_device *pdev)
{
#ifndef DISABLE_PROXIMITY
//FIH, HenryJuang 2009/11/11 ++
/* Enable Proximaty wake source.*/
	if (!((HWID >= CMCS_RTP_PR2) && ((HWID <= CMCS_RTP_MP3)))){
	if (HWID == CMCS_HW_VER_EVB1)
		disable_irq_wake(MSM_GPIO_TO_INT(CM3602_EVB1_PS_GPIO_OUT));
	else
		disable_irq_wake(MSM_GPIO_TO_INT(CM3602_PR1_PS_GPIO_OUT));		
	isCM3602Suspend=0;		
	}	
//FIH, HenryJuang 2009/11/11 --
#endif
	return 0;
}


/* cm3602_init: init cm3602 driver
 * Its major is the same as misc and the minor is randomly assigned.
 * */
static int __init cm3602_init(void)
{
	int ret, rc;
	struct proc_dir_entry *entry; 	

	//use misc major number plus random minor number, and init device
	/*
	ret = misc_register(&cm3602_alsps_dev);
	if (ret){
		printk(KERN_WARNING "CM3602 ALSPS Unable to register misc device.\n");
		return ret;
	}
	*/
	ret = platform_driver_register(&ALSPS_driver);
	entry = create_proc_entry("driver/cm3602_alsps", 0, NULL);
	if (entry)
		entry->proc_fops = &cm3602_proc_ops;

	rc = gpio_request(CM3602_EN_GPIO, "alsps_enable");
	if (rc){
		return -EIO;
	}

	HWID = FIH_READ_HWID_FROM_SMEM();
	if (!((HWID >= CMCS_RTP_PR2) && ((HWID <= CMCS_RTP_MP3)))){
		
	rc = gpio_tlmm_config(GPIO_CFG(CM3602_EN_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		return -EIO;
	}	
	gpio_direction_output(CM3602_EN_GPIO, 1);
	gpio_free(CM3602_EN_GPIO);
#ifndef DISABLE_PROXIMITY
//FIH, HenryJuang 2009/11/11 ++
/* Enable Proximaty wake source.*/
	if (HWID == CMCS_HW_VER_EVB1)
		cm3602_irq = MSM_GPIO_TO_INT(CM3602_EVB1_PS_GPIO_OUT);
	else
		cm3602_irq = MSM_GPIO_TO_INT(CM3602_PR1_PS_GPIO_OUT);
/* FIH, Henry Juang, 2009/11/20 ++*/
/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
	ret = request_irq( cm3602_irq, &cm3602_isr, IRQF_TRIGGER_RISING |IRQF_TRIGGER_FALLING , "CM3602", NULL );
/* FIH, Henry Juang, 2009/11/20 --*/
	
	disable_irq(cm3602_irq);
	#endif
	}
	#ifndef DISABLE_PROXIMITY
	flag_cm3602_irq=0;
	isCM3602Suspend=0;
//FIH, HenryJuang 2009/11/11 --
	
	rc = gpio_tlmm_config(GPIO_CFG(CM3602_PS_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		return -EIO;
	}
	
	if (HWID == CMCS_HW_VER_EVB1)
	{
		rc = gpio_tlmm_config(GPIO_CFG(CM3602_EVB1_PS_GPIO_OUT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN/*GPIO_CFG_NO_PULL*/, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			return -EIO;
		}
	}
	else
	{
		rc = gpio_tlmm_config(GPIO_CFG(CM3602_PR1_PS_GPIO_OUT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN/*GPIO_CFG_NO_PULL*/, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			return -EIO;
		}
	}

	rc = gpio_request(CM3602_PS_GPIO, "alsps_prox");
	if (rc){
		return -EIO;
	}
	gpio_direction_output(CM3602_PS_GPIO, 0);

	rc = gpio_request(CM3602_EN_GPIO, "alsps_ena");
	if (rc){
		return -EIO;
	}

//FIH, HenryJuang 2009/11/11 ++
/* Enable Proximaty wake source.*/
	
	gpio_direction_output(CM3602_PS_GPIO, 1);

	if (HWID == CMCS_HW_VER_EVB1)
		gpio_request(CM3602_EVB1_PS_GPIO_OUT,"alsps_ps");
	else
		gpio_request(CM3602_PR1_PS_GPIO_OUT,"alsps_ps");
//FIH, HenryJuang 2009/11/11 --
/* Enable Proximaty wake source.*/
	if (HWID == CMCS_HW_VER_EVB1)
		gpio_direction_input(CM3602_EVB1_PS_GPIO_OUT);
	else
		gpio_direction_input(CM3602_PR1_PS_GPIO_OUT);
#endif
	atomic_notifier_chain_register(&panic_notifier_list, &trace_panic_notifier);
	return 0;
}

static void  __exit cm3602_exit(void)
{	
	//free_irq( cm3602_irq, NULL );
	remove_proc_entry("driver/cm3602_alsps", NULL /* parent dir */);
	misc_deregister(&cm3602_alsps_dev);
}

module_init(cm3602_init);
module_exit(cm3602_exit);

MODULE_DESCRIPTION("CM3602 proximity and light sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("WillChen,HansonLin,JamesTsai");
MODULE_VERSION("Version 1.0");
MODULE_ALIAS("platform:msm7227");

