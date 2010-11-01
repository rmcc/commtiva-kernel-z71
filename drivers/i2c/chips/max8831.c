/*
 *     max8831.c - MAX8831 LED EXPANDER for BACKLIGHT and LEDs CONTROLLER
 *
 *     Copyright (C) 2009 Neo CH Chen <NeoCHChen@fihtdc.com>
 *     Copyright (C) 2009 Chi Mei Communication Systems Inc.
 *
 *     This program is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; version 2 of the License.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h> 
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/major.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/ioctl.h>
// +++FIH_ADQ +++
#include <linux/i2c/max8831.h>
#include <mach/gpio.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>
// ---FIH_ADQ ---
//++++++++++++++++++++++++FIH_FXX_Neo
#include "../../../kernel/power/power.h"
//-----------------------FIH_FXX_Neo

/*FIH_FTM, PinyCHWu, 2009/06/15 {*/
/*Add misc device*/
#ifdef CONFIG_FIH_FXX
#include <linux/miscdevice.h>
#endif
/*} FIH_FTM, PinyCHWu, 2009/06/15*/

//I2C 
//#define I2C_SLAVE_ADDR	0x4d
#define I2C_SLAVE_ADDR	(0x9a >> 1)

//Neo: workaround for adjusting LCD brightness
#define ISNUM(x) (((x) >= '0') && ((x) <= '9'))

//chandler_i2c: workaround for init version
int uiHWID=0; // Teng Rui MAX8831
//#define CMCS_HW_VER_EVT2 127

// +++FIH_ADQ +++
#define i2cmax8831_name "max8831"
// ---FIH_ADQ ---

/* FIH, Henry Juang, 2009/11/20 ++*/
/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
int gProximity_flag_On = 0;
int gBrightness_level = 0;
/* FIH, Henry Juang, 2009/11/20 --*/
// FXX_CR, Neo Chen, 2009.06.12, Add for ensuring backlight on after panel on
extern int msm_fb_check_panel_on(int fb_no);

static DEFINE_MUTEX(g_mutex);


struct i2c_client *bl_max8831_i2c = NULL;
static struct proc_dir_entry *bl_max8831_proc_file = NULL;

/****************Driver general function**************/
int check_max8831_exist(void)
{
  	if(!bl_max8831_i2c)
	{
	  	printk(KERN_ERR "Error!! Driver max8831 not exist!! without bl_max8831_i2c\n");
	  	return -1;
	}
	return 0;
}
EXPORT_SYMBOL(check_max8831_exist);

/*************I2C functions*******************/
static int i2c_rx( u8 * rxdata, int length )
{
	struct i2c_msg msgs[] =
	{
		{
			.addr = I2C_SLAVE_ADDR,
			.flags = 0,
			.len = 1,
			.buf = rxdata,
		},
		{
			.addr = I2C_SLAVE_ADDR,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxdata,
		},
	};

	int ret;
	if( ( ret = i2c_transfer( bl_max8831_i2c->adapter, msgs, 2 ) ) < 0 )
	{
		printk( KERN_ERR "[Max8831]i2c rx failed %d\n", ret );
		return -EIO;
	}

	return 0;
}

static int i2c_tx( u8 * txdata, int length )
{
	struct i2c_msg msg[] =
	{
		{
			.addr = I2C_SLAVE_ADDR,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	if( i2c_transfer( bl_max8831_i2c->adapter, msg, 1 ) < 0 )
	{
	    if(length==1)
		    printk( KERN_ERR "[Max8831]i2c tx failed, addr(%x), len(%d), data0(0x%x)\n",msg[0].addr, msg[0].len, msg[0].buf[0]);
	   else
    		    printk( KERN_ERR "[Max8831]i2c tx failed, addr(%x), len(%d), data0(0x%x),data1(0x%x)\n",msg[0].addr, msg[0].len, msg[0].buf[0], msg[0].buf[1]);

	   return -EIO;
	}

	return 0;
}


/*****************backlight & led general control function*************/
static int pwm_level_check(int port, int data)
{
	int gataBuf;
	
	gataBuf = data;
  	if(data <= 0)
	  	data = PWM_STATIC_LOW;
	else if(data > 100)
	  	data = PWM_STATIC_HIGH;
	else
	  	data = PWM_LEVEL(data);
	
	return data;
}


int max8831_port_set_level(int port, int level)
{
 	int ret = 0;
	struct max8831_i2c_data value;

	mutex_lock(&g_mutex);	
	
	level = pwm_level_check(port, level);

	//set lcd bl port on
	value.reg = port;
	value.data = level;
	ret = i2c_tx((u8*)&value, 2);

	mutex_unlock(&g_mutex);

	if(ret)
	{
	  	printk(KERN_ERR "max8831_port_set_level(): set i2c tx on failed\n");
		return ret;
	}
	
	//return level; //This might for record it.
	return ret;
}
EXPORT_SYMBOL(max8831_port_set_level);

static void max8831_enable(int port, int data)
{
 	int ret = 0;
	struct max8831_i2c_data value;
	
/*FXX_CR, NeoCHChen, 2009/10/30 {*/
/*Add to set backlight ramp control for chip max8831*/
#ifdef CONFIG_FIH_FXX
 	//set lcd bl ramp control
  	value.reg = PORT_P3_REG;
  	value.data = 0x12; // ramp down/up for 256ms
  	ret = i2c_tx((u8*)&value, 2);
	printk(KERN_INFO "MAX8831 enable ramp down/up for 256ms\n");
  	if(ret)
		printk(KERN_INFO "MAX8831 enable ramp down/up failed\n");
#endif
/*} FIH_FTM, PinyCHWu, 2009/06/15*/

	//set lcd bl port on
	value.reg = port;
	value.data = data;
	ret = i2c_tx((u8*)&value, 2);
	if(ret)
		printk(KERN_INFO "Enable MAX8831 failed\n");
}


/*****************LCD part***************/
int lcd_bl_set_intensity(int level)
{
  	int ret = 0;
	int checkPanelON = 0; // FXX_CR, Neo Chen, 2009.06.12, Add for ensuring backlight on after panel on
	int checkTimes = 0; //FXX_CR, Neo Chen, 2009.08.11, Prevent to block in while loop when check panel on
	struct max8831_i2c_data value;
	suspend_state_t SuspendState = PM_SUSPEND_ON;//0

	if(check_max8831_exist())
	  	return -1;

/* FIH, Henry Juang, 2009/11/20 ++*/
/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
	gBrightness_level = level;

	//proxi
	if(gProximity_flag_On)
	{
		if(level !=0)
			return 0;
	}
/* FIH, Henry Juang, 2009/11/20 --*/

/* FIH_ADQ , added by guorui */	
	if (uiHWID >= CMCS_HW_VER_EVB1)
	{
		value.reg = 0x00;
		if(i2c_rx((u8*)&value, 2))
			printk(KERN_ERR "[Max8831]lcd_bl_set_intensity(): i2c_rx failed!!\n");
		if(level == 0 && (value.reg & 0x01)) 
		{
			value.data = (value.reg & 0xFE);
			value.reg = 0x00;
			if(i2c_tx((u8*)&value, 2))
				printk(KERN_ERR "[Max8831]lcd_bl_set_intensity(): i2c_tx to close BL failed!!\n");

			return ret;
		}else if( level > 0 && (value.reg & 0x01) == 0){
			// FXX_CR, Neo Chen, 2009.08.13, if in suspend state, not to open backlight
			SuspendState = get_suspend_state();
			if(SuspendState == PM_SUSPEND_MEM) //3
				return -1;
			
			// FXX_CR, Neo Chen, 2009.06.12, Add for ensuring backlight on after panel on +++
			while(!checkPanelON)
			{
				checkPanelON = msm_fb_check_panel_on(0);
				if(!checkPanelON)
				{
					msleep(10);
					checkTimes++;//FXX_CR, Neo Chen, 2009.08.11, Prevent to block in while loop when check panel on
				}
				//FXX_CR, Neo Chen, 2009.12.31 Let backlight not to open until LCM is on
				//if(checkTimes>=20) return -1;
			}
			// FXX_CR, Neo Chen, 2009.06.12 ---
			
			value.data = value.reg | 0x01;
			value.reg = 0x00;
			if(i2c_tx((u8*)&value, 2))
				printk(KERN_ERR "[Max8831]lcd_bl_set_intensity(): i2c_tx to open BL failed!!\n");
			
		}
		
		ret = max8831_port_set_level(PORT_P11_REG, level);
/* FIH_ADQ  */
	}
	
	if(ret < 0)
	  	return -1;

	return 0;
}
EXPORT_SYMBOL(lcd_bl_set_intensity);

/* FIH, Henry Juang, 2009/11/20 ++*/
/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
int Proximity_Flag_Set(int flag)
{
	if(flag){
		gProximity_flag_On = flag;
		max8831_port_set_level(PORT_P11_REG, 0);
	}
	else{
		gProximity_flag_On = flag;
		if(gBrightness_level > 0) lcd_bl_set_intensity(gBrightness_level);
	}
	
	//return gProximity_flag_On;
	return 1;
}
EXPORT_SYMBOL(Proximity_Flag_Set);
/* FIH, Henry Juang, 2009/11/20 --*/

/***************max8831 backlight init***************/
static int max8831_bl_init(void)
{
	int ret = 0;

	uiHWID = FIH_READ_HWID_FROM_SMEM();     

	printk(KERN_INFO "max8831_bl_init(): uiHWID: %d\n", uiHWID);

	if (uiHWID >= CMCS_HW_VER_EVB1){
		max8831_enable(PORT_P0_REG,1);//chandler
		ret = max8831_port_set_level(PORT_P11_REG, 101);
		if(ret<0){
		  	printk(KERN_ERR "max8831_bl_init(): set level failed\n");
			return ret;
		}	
	}
	
	return ret;  //Debbie add
}

/******************Proc operation********************/
static int max8831_seq_open(struct inode *inode, struct file *file)
{
  	return single_open(file, NULL, NULL);
}

static ssize_t max8831_seq_write(struct file *file, const char *buff,
								size_t len, loff_t *off)
{
	char str[64];
	int param = -1;
	int param2 = -1;
	int param3 = -1;
	char cmd[32];

	if(copy_from_user(str, buff, sizeof(str)))
	{
		printk(KERN_ERR "max8831_seq_write(): copy_from_user failed!\n");
		return -EFAULT;
	}

//	if(sscanf(str, "%s %d", cmd, &param) == -1)
	{
	  	if(sscanf(str, "%s %d %d %d", cmd, &param, &param2, &param3) == -1)
		{
		  	printk("parameter format: <type> <value>\n");
	 		return -EINVAL;
		}
	}

	//Neo: workaround for adjusting LCD brightness +++
	if (ISNUM(cmd[0]))
	{
		if(ISNUM(cmd[1])&&ISNUM(cmd[2])) param = 100;
		else if(ISNUM(cmd[1])) param = (cmd[0]-48)*10+(cmd[1]-48);
		else param = cmd[0]-48;
		
		//printk("[Neo] digit param=%d\n",param);
		cmd[0] = 'l';cmd[1] = 'c';cmd[2] = 'd';
	}
	//Neo: workaround for adjusting LCD brightness ---

	if(!strnicmp(cmd, "lcd", 3))
	  	cmd[0] = 'c';
	else
	  	cmd[0] = '?';

	switch(cmd[0])
	{
	  	case 'c':
		  	lcd_bl_set_intensity(param);
		  	break;

		default:
			printk(KERN_NOTICE "type parameter error\n");
			break;
	}

	return len;
}

static struct file_operations max8831_seq_fops = 
{
  	.owner 		= THIS_MODULE,
	.open  		= max8831_seq_open,
	.write 		= max8831_seq_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int max8831_create_proc(void)
{
  	bl_max8831_proc_file = create_proc_entry("driver/max8831", 0666, NULL);
	
	if(!bl_max8831_proc_file){
	  	printk(KERN_ERR "create proc file for MAX8831 failed\n");
		return -ENOMEM;
	}

	printk(KERN_INFO "MAX8831 proc ok\n");
	bl_max8831_proc_file->proc_fops = &max8831_seq_fops;
	return 0;
}

/*FIH_FTM, PinyCHWu, 2009/06/15 {*/
/*Add full device control functions*/
#ifdef CONFIG_FIH_FXX
/*******************File control function******************/
// devfs
static int max7302_dev_open( struct inode * inode, struct file * file )
{
	printk( KERN_INFO "MAX7302 open\n" );

	if( ( file->f_flags & O_ACCMODE ) == O_WRONLY )
	{
		printk( KERN_INFO "MAX7302's device node is readonly\n" );
		return -1;
	}
	else
		return 0;
}

static ssize_t max7302_dev_write(struct file *filp, const char __user *buff, size_t count, loff_t *f_pos)
{
	char str[64];
	int param = -1;
	//char cmd[32]; Neo removed

 	int ret = 0;
	struct max8831_i2c_data value;
	printk(KERN_INFO "MAX7302 write\n");

	if(copy_from_user(str, buff, sizeof(str)))
	  	return -EFAULT;

  	if(sscanf(str, "%d", &param) == -1)
	{
	  	printk("parameter format: <value>\n");
 		return -EINVAL;
	}
	
	value.reg = 0x00;
	value.data = 0x00;
	ret = i2c_rx((u8*)&value, 2);
	printk(KERN_NOTICE "Read MAX7302 0x00 =%d %d\n", value.data , ret);
	
  	lcd_bl_set_intensity(param);

	return count;
}

static int max7302_dev_ioctl( struct inode * inode, struct file * filp, unsigned int cmd, unsigned long arg )
{
	int read_p = 0, write_p = 0;

	printk( KERN_INFO "MAX7302 ioctl, cmd = %d\n", cmd );

	switch (cmd)
	{
		case MAX7302_S_LCD:
			if(copy_from_user(&write_p, (int __user*)arg, sizeof(int)))
			{
			  	printk(KERN_ERR "Get user-space data error\n");
				return -1;
			}
			read_p = lcd_bl_set_intensity(write_p);
			if(read_p)
			  	printk(KERN_NOTICE "Set LCD backlight failed\n");
			else
				printk(KERN_INFO "Set LCD backlight value = 0x%2x\n", write_p);
			break;

		default:
			printk(KERN_NOTICE "IO-Control: wrong command\n");
			return -1;
	}
	return 0;
}

static int max7302_dev_release( struct inode * inode, struct file * filp )
{
	printk(KERN_INFO "MAX7302 release\n");
	return 0;
}

static const struct file_operations max7302_dev_fops = {
	.open = max7302_dev_open,
	.write = max7302_dev_write,
	.ioctl = max7302_dev_ioctl,
	.release = max7302_dev_release,
};

static struct miscdevice max7302_dev = {
   	.minor = MISC_DYNAMIC_MINOR,
  	.name = "max7302",
  	.fops = &max7302_dev_fops,
};
#endif
/*} FIH_FTM, PinyCHWu, 2009/06/15*/

/******************Driver Functions*******************/
//Debbie modify
//static int max7302_probe(struct i2c_client *client)
static int max8831_probe(struct i2c_client *client,const struct i2c_device_id *device_id)
{
  	int ret = 0;

	bl_max8831_i2c = client;
	printk( KERN_INFO "Driver probe: %s\n", __func__ );
	
	ret = max8831_bl_init();
	if(ret)
		printk(KERN_INFO "Max8831 backlight init failed\n");

	return ret;
}

static int max8831_remove(struct i2c_client *client)
{
	int ret = 0;
	
	mutex_destroy(&g_mutex);
	
	return ret;
}

 static void max8831_shutdown(struct i2c_client *client)
{
	//close backlight
	lcd_bl_set_intensity(0);
}

// +++ FIH_ADQ +++
static const struct i2c_device_id i2cmax8831_idtable[] = {
       { i2cmax8831_name, 0 },
       { }
};
// --- FIH_ADQ ---

static struct i2c_driver max8831_driver = {
	.driver = {
		.name	= "max8831",
	},
	.probe		= max8831_probe,
	.remove		= max8831_remove,
	.shutdown     = max8831_shutdown,
	.id_table = i2cmax8831_idtable,
};
static int __init max8831_init(void)
{
	int ret = 0;

	printk( KERN_INFO "Driver init: %s\n", __func__ );

/*FIH_FTM, PinyCHWu, 2009/06/15 {*/
/*Use miscdev*/
#ifdef CONFIG_FIH_FXX
 	//register and allocate device, it would create an device node automatically.
  	//use misc major number plus random minor number, and init device
  	ret = misc_register(&max7302_dev);
    	if (ret){
  		printk(KERN_ERR "%s: Register misc device failed.\n", __func__);
  		goto register_del;
    	}
#endif
/*}FIH_FTM, PinyCHWu, 2009/06/15*/

	// i2c
	ret = i2c_add_driver(&max8831_driver);
	if (ret) {
		printk(KERN_ERR "%s: Driver registration failed, module not inserted.\n", __func__);
		goto driver_del;
	}

	// create proc
	ret = max8831_create_proc();
	if(ret) {
	  	printk(KERN_ERR "%s: create proc file failed\n", __func__);
		goto driver_del;
	}

	//all successfully.
	return ret;

driver_del:
	i2c_del_driver(&max8831_driver);

/*FIH_FTM, PinyCHWu, 2009/06/15 {*/
/*new label name for remove misc device*/
#ifdef CONFIG_FIH_FXX
register_del:
	misc_deregister(&max7302_dev);
#endif
/*} FIH_FTM, PinyCHWu, 2009/06/15*/

	return -1;
}

static void __exit max8831_exit(void)
{
	printk( KERN_INFO "Driver exit: %s\n", __func__ );

/*FIH_FTM, PinyCHWu, 2009/06/15 {*/
/*new label name for remove misc device*/
#ifdef CONFIG_FIH_FXX
	misc_deregister(&max7302_dev);
#endif
/*} FIH_FTM, PinyCHWu, 2009/06/15*/

	i2c_del_driver(&max8831_driver);
}

module_init(max8831_init);
module_exit(max8831_exit);

MODULE_AUTHOR( "Neo CH Chen <NeoCHChen@fihtdc.com>" );
MODULE_DESCRIPTION( "MAX8831 driver" );
MODULE_LICENSE( "GPL" );

