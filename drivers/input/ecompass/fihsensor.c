/* fihsensor.c - YAMAHA MS-3C compass driver
 * 
 * Copyright (C) 2007-2008 FIH Corporation.
 * Author: Tiger JT Lee
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/akm8976.h>
#include <linux/module.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>

#define DEBUG 0
/* FIH; Tiger; 2009/6/10 { */
/* implement suspend/resume */
#define FEATURE_SLEEP	1
/* } FIH; Tiger; 2009/6/10 */

#define PORTING	1

static struct i2c_client *ms3c_i2c_client = NULL;
static struct i2c_client *smb380_i2c_client = NULL;

static int profile = 1;
module_param(profile, int, S_IRUGO);

static int gsensorCalibStart = 0;
module_param(gsensorCalibStart, int, S_IRUGO|S_IWUSR);

struct fih_sensor_context {
	struct i2c_client *activeSlave;
/* FIH; Tiger; 2009/6/10 { */
/* implement suspend/resume */
#if FEATURE_SLEEP
	unsigned char bIsIdle;
#endif
/* } FIH; Tiger; 2009/6/10 */
	int depth;
}; 
typedef struct fih_sensor_context fih_sensor_context_s;

#ifndef SMB380_I2C_ADDR
#define SMB380_I2C_ADDR		0x38
#endif
#ifndef MS3CDRV_I2C_SLAVE_ADDRESS
#define MS3CDRV_I2C_SLAVE_ADDRESS 0x2e
#endif

#define IOCTL_SET_ACTIVE_SLAVE	0x0706
#define IOCTL_GET_SENSOR_CONTROL_INFO	0x0707
#define IOCTL_SET_CHARGER_STATE	0x0708
#define IOCTL_GET_MAGNET_GUARD	0x0709
#define IOCTL_SET_MAGNET_GUARD	0x0710

static fih_sensor_context_s fihsensor_ctx = {
	.activeSlave = NULL,
/* FIH; Tiger; 2009/6/10 { */
/* implement suspend/resume */
#if FEATURE_SLEEP
	.bIsIdle = false,
#endif
/* } FIH; Tiger; 2009/6/10 */
	.depth = 0,
};


/* FIH; Tiger; 2009/6/10 { */
/* implement suspend/resume */
#if FEATURE_SLEEP
#define SMB380_MODE_NORMAL	0
#define SMB380_MODE_SLEEP	2

#define SMB380_CONF2_REG	0x15
#define SMB380_CTRL_REG		0x0a

#define WAKE_UP_REG		SMB380_CONF2_REG
#define WAKE_UP_POS		0
#define WAKE_UP_LEN		1
#define WAKE_UP_MSK		0x01

#define SLEEP_REG		SMB380_CTRL_REG
#define SLEEP_POS		0
#define SLEEP_LEN		1
#define SLEEP_MSK		0x01

#define CHR_EN 33

#define SMB380_SET_BITSLICE(regvar, bitname, val) \
			(regvar & ~bitname##_MSK) | ((val<<bitname##_POS)&bitname##_MSK)

static int compass_read_reg(struct i2c_client *clnt, unsigned char reg, unsigned char *data, unsigned char count)
{
	unsigned char tmp[10];
	
	if(10 < count)
		return -1;
	
	tmp[0] = reg;
	if(1 != i2c_master_send(clnt, tmp, 1))
	{
		return -EIO;
	}

	if(count != i2c_master_recv(clnt, tmp, count))
	{
		return -EIO;
	}

	strncpy(data, tmp, count);

	return 0;
}

static int compass_write_reg(struct i2c_client *clnt, unsigned char reg, unsigned char *data, unsigned char count)
{
	unsigned char tmp[2];

	while(count)
	{
		tmp[0] = reg++;
		tmp[1] = *(data++);

		if(2 != i2c_master_send(clnt, tmp, 2))
		{
			return -EIO;
		}

		count--;
	}

	return 0;
}

static int enter_mode(unsigned char mode)
{
	unsigned char data1, data2;
	int ret;

	if(mode==SMB380_MODE_NORMAL || mode==SMB380_MODE_SLEEP) 
	{
		if((ret=compass_read_reg(smb380_i2c_client, WAKE_UP_REG, &data1, 1)))
			return ret;

		data1 = SMB380_SET_BITSLICE(data1, WAKE_UP, mode);

		if((ret=compass_read_reg(smb380_i2c_client, SLEEP_REG, &data2, 1)))
			return ret;

		data2 = SMB380_SET_BITSLICE(data2, SLEEP, (mode>>1));

		if((ret=compass_write_reg(smb380_i2c_client, WAKE_UP_REG, &data1, 1)))
			return ret;
		if((ret=compass_write_reg(smb380_i2c_client, SLEEP_REG, &data2, 1)))
			return ret;
	}

	return 0;
}

static int start_suspend(void)
{

	return enter_mode(SMB380_MODE_SLEEP);
}

static int start_resume(void)
{

	return enter_mode(SMB380_MODE_NORMAL);
}
#endif	
/* } FIH; Tiger; 2009/6/10 */



static int compass_open(struct inode *inode, struct file *file)
{

	fihsensor_ctx.depth++;

	if(fihsensor_ctx.depth > 1)
	{
		return 0; 
	}

	//fihsensor_ctx.activeSlave = ms3c_i2c_client;
	fihsensor_ctx.activeSlave = smb380_i2c_client;

/* FIH; Tiger; 2009/6/10 { */
/* implement suspend/resume */
#if FEATURE_SLEEP
	//if(fihsensor_ctx.bIsIdle == true)
	{
		if(start_resume()) {
			fihsensor_ctx.depth--;
			return -EIO;
		}
		else {
			fihsensor_ctx.bIsIdle = false;
		}
	}
#endif
/* } FIH; Tiger; 2009/6/10 */
	
/* FIH; Tiger; 2009/11/30 { */
/* restrict others to access compass */
	file->private_data = (void*)0;
/* } FIH; Tiger; 2009/11/30 */
	
	return 0;
}

static int compass_release(struct inode *inode, struct file *file)
{

	fihsensor_ctx.depth--;

	if(fihsensor_ctx.depth > 0)
	{
		return 0;
	}

	gsensorCalibStart = 0;
	
	fihsensor_ctx.activeSlave = NULL;

/* FIH; Tiger; 2009/6/10 { */
/* implement suspend/resume */
#if FEATURE_SLEEP
	//if(fihsensor_ctx.bIsIdle == false)
	{
		if(start_suspend()) {
			printk(KERN_ERR "fihsensor: start_suspend() fail\r\n");
		}
		else {
			fihsensor_ctx.bIsIdle = true;
		}
	}
#endif
/* } FIH; Tiger; 2009/6/10 */

	return 0;
}


//extern int USB_Connect;
#if !PORTING
extern int check_USB_type;
#endif
unsigned fihsensor_battery_voltage = 0;
unsigned fihsensor_battery_level = 0;
int fihsensor_magnet_guard1 = -1;

#define CHARGER_INFO_MASK		0x000000FF
#define CHARGER_INFO_1A		0x00000001
#define CHARGER_INFO_500mA	0x00000002
#define CHARGER_REACH_CV		0x00000010

#define MAGNET_CONTROL_INFO_MASK	0x0000FF00
#define MAGNET_MANUAL_CALIBRATION	0x00000100
#define MAGNET_CHECK_DISTORTION	0x00000200
#define MAGNET_START_CALIBRATION	0x00000400
#define GSENSOR_START_CALIBRATION	0x00000800

#define BATTERY_VOLTAGE_MASK		0x1FFF0000
#define BATTERY_VOLTAGE_SHIFT		16
#define BATTERY_LEVEL_MASK		0x00FF0000
#define BATTERY_LEVEL_SHIFT		16

static int bMagnetCheckDistort = 0;
static int bMagnetManualCalibration = 0;
static int bMagnetStartCalibration = 0;   

module_param(bMagnetCheckDistort, int, S_IRUGO|S_IWUSR);
module_param(bMagnetManualCalibration, int, S_IRUGO|S_IWUSR);
module_param(bMagnetStartCalibration, int, S_IRUGO|S_IWUSR);

static int
compass_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
#if !PORTING
	static int prev_charger_state = -1;
#endif


	switch (cmd) {
	case IOCTL_SET_ACTIVE_SLAVE:
		
		switch (arg) {
		case SMB380_I2C_ADDR:
			fihsensor_ctx.activeSlave = smb380_i2c_client;

			/* FIH; Tiger; 2009/11/30 { */
			/* restrict others to access compass */
			file->private_data = (void*)current;
			/* } FIH; Tiger; 2009/11/30 */
			break;
			
		case MS3CDRV_I2C_SLAVE_ADDRESS:
			fihsensor_ctx.activeSlave = ms3c_i2c_client;

			/* FIH; Tiger; 2009/11/30 { */
			/* restrict others to access compass */
			file->private_data = (void*)current;
			/* } FIH; Tiger; 2009/11/30 */
			break;
		
		default:
			break;
		}
		
		break;


	case IOCTL_GET_SENSOR_CONTROL_INFO:
		{
			int controlInfo = 0;
			//printk(KERN_INFO "Compass: check_USB_type=%d\r\n", check_USB_type);
			//printk(KERN_INFO "Compass: USB_Connect=%d\r\n", USB_Connect);

			if(bMagnetManualCalibration) {
				controlInfo |= MAGNET_MANUAL_CALIBRATION;
			}

			if(bMagnetStartCalibration) {
				controlInfo |= MAGNET_START_CALIBRATION;
				bMagnetStartCalibration = 0;
			}

			if(gsensorCalibStart > 0) {
				controlInfo |= GSENSOR_START_CALIBRATION;
				gsensorCalibStart = 2;
			}

			if(bMagnetCheckDistort) {
				controlInfo |= MAGNET_CHECK_DISTORTION;
			}

			if(fihsensor_battery_voltage >= 4070 /* 92.8% */) { 
				controlInfo |= CHARGER_REACH_CV;
			}
#if !PORTING
			if(check_USB_type == 1) { 
				/* USB charger */
				controlInfo |= CHARGER_INFO_500mA;
				//controlInfo |= ((fihsensor_battery_voltage<<BATTERY_VOLTAGE_SHIFT)&BATTERY_VOLTAGE_MASK);
				controlInfo |= ((fihsensor_battery_level<<BATTERY_VOLTAGE_SHIFT)&BATTERY_LEVEL_MASK);
			}
			else if(check_USB_type == 2) {
				/* Wall charger */
				controlInfo |= CHARGER_INFO_1A;
				//controlInfo |= ((fihsensor_battery_voltage<<BATTERY_VOLTAGE_SHIFT)&BATTERY_VOLTAGE_MASK);
				controlInfo |= ((fihsensor_battery_level<<BATTERY_VOLTAGE_SHIFT)&BATTERY_LEVEL_MASK);
			}
#endif
			return controlInfo;
		}
		break;

	case IOCTL_SET_CHARGER_STATE:
#if !PORTING
		if(arg == 1) {
			/* restore charging state */
			if(check_USB_type != 0) {
				if(fihsensor_magnet_guard1 != -1)  {
					gpio_set_value(CHR_EN, prev_charger_state);	
					fihsensor_magnet_guard1 = prev_charger_state;
				}
			}
		}
		else if(arg == 0) {
			/* disable charging state */
			if(check_USB_type != 0) {
				prev_charger_state = gpio_get_value(CHR_EN);
				if(prev_charger_state == 0) {
					gpio_set_value(CHR_EN, 1);
				}
				fihsensor_magnet_guard1 = 1;
			}
			else {
				fihsensor_magnet_guard1 = -1;
			}
		}
#endif
		break;

	case IOCTL_GET_MAGNET_GUARD:
		if(fihsensor_magnet_guard1 == gpio_get_value(CHR_EN)) {
			return fihsensor_magnet_guard1;
		}
		else {
			return 2;
		}
		break;

	case IOCTL_SET_MAGNET_GUARD:
		fihsensor_magnet_guard1 = gpio_get_value(CHR_EN);
		break;		
		
	default:
		break;
	}

	return 0;
}

static ssize_t compass_read(struct file *fp, char __user *buf,
                            size_t count, loff_t *pos)
{
	int ret = 0;
	char tmp[64];


/* FIH; Tiger; 2009/11/30 { */
/* restrict others to access compass */
	if(fp->private_data != (void*)current) {
		return -EPERM;
	}
/* } FIH; Tiger; 2009/11/30 */


/* FIH; Tiger; 2009/6/10 { */
/* implement suspend/resume */
#if FEATURE_SLEEP
	if(fihsensor_ctx.bIsIdle == true && fihsensor_ctx.activeSlave == smb380_i2c_client)
	{
		return -EIO;
	}
#endif
/* } FIH; Tiger; 2009/6/10 */	
		
	if(fihsensor_ctx.activeSlave == NULL) 
	{
		ret = -ENODEV;	
	}
	else 
	{
		ret = i2c_master_recv(fihsensor_ctx.activeSlave, tmp, count);
		if(copy_to_user(buf, tmp, ret))
		{
			ret = -EFAULT;
		}
	}	
	
	return ret;
}
                     
static ssize_t compass_write(struct file *fp, const char __user *buf,
                             size_t count, loff_t *pos)
{
	int ret = 0;
	char tmp[64];

	
/* FIH; Tiger; 2009/11/30 { */
/* restrict others to access compass */
	if(fp->private_data != (void*)current) {
		return -EPERM;
	}
/* } FIH; Tiger; 2009/11/30 */	
	
/* FIH; Tiger; 2009/6/10 { */
/* implement suspend/resume */
#if FEATURE_SLEEP
	if(fihsensor_ctx.bIsIdle == true && fihsensor_ctx.activeSlave == smb380_i2c_client)
	{
		return -EIO;
	}
#endif
/* } FIH; Tiger; 2009/6/10 */

	if(fihsensor_ctx.activeSlave == NULL) 
	{
		ret = -ENODEV;	
	}
	else 
	{
		if(copy_from_user(tmp, buf, count))
		{
			ret = -EFAULT;
		}
		else
		{
			ret = i2c_master_send(fihsensor_ctx.activeSlave, tmp, count);
		}

	}
	
	return ret;
}                                                  
                            
static struct file_operations compass_fops = {
	.owner = THIS_MODULE,
	.open = compass_open,
	.release = compass_release,
	.ioctl = compass_ioctl,
	.read =    compass_read,
	.write =   compass_write,	
};

static struct miscdevice compass_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "compass",
	.fops = &compass_fops,
};

static int ms3c_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;

#if 0
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
#endif

	ms3c_i2c_client = client;

	err = misc_register(&compass_device);
	if (err) {
		goto exit_misc_device_register_failed;
	}

	return 0;

exit_misc_device_register_failed:
//exit_check_functionality_failed:

	return err;
}

static int ms3c_remove(struct i2c_client *client)
{
	//i2c_detach_client(client);
	return 0;
}

static int ms3c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int ms3c_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ms3c_id[] = {
	{ "MS3C", 0 },
	{ }
};

static struct i2c_driver ms3c_driver = {
	.probe = ms3c_probe,
	.remove = ms3c_remove,
	.suspend	= ms3c_suspend,
	.resume		= ms3c_resume,
	.id_table = ms3c_id,
	.driver = {
		   .name = "MS3C",
		   },
};

static int smb380_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;
#if 0
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
#endif

	smb380_i2c_client = client;

/* FIH; Tiger; 2009/6/10 { */
/* implement suspend/resume */
#if FEATURE_SLEEP
	if(start_suspend()) {
		printk(KERN_ERR "fihsensor: start_suspend() fail\r\n");
	}
	else {
		fihsensor_ctx.bIsIdle = true;
	}
#endif
/* } FIH; Tiger; 2009/6/10 */

	return 0;

//exit_check_functionality_failed:
	return err;
}

static int smb380_remove(struct i2c_client *client)
{
	//i2c_detach_client(client);
	return 0;
}

static int smb380_suspend(struct i2c_client *client, pm_message_t mesg)
{
/* FIH; Tiger; 2009/6/10 { */
/* implement suspend/resume */
#if FEATURE_SLEEP

	/* always execute it to recover state error */
	//if(fihsensor_ctx.bIsIdle == false)
	{
		if(start_suspend()) {
			printk(KERN_ERR "fihsensor: start_suspend() fail\r\n");
		}
		else {
			fihsensor_ctx.bIsIdle = true;
		}
	}
#endif
/* } FIH; Tiger; 2009/6/10 */

	return 0;
}

static int smb380_resume(struct i2c_client *client)
{
/* FIH; Tiger; 2009/6/10 { */
/* implement suspend/resume */
#if FEATURE_SLEEP

	/* always execute it to recover state error */
	if(fihsensor_ctx.activeSlave)
	//if(fihsensor_ctx.activeSlave && fihsensor_ctx.bIsIdle==true)
	{
		if(start_resume()) {
			printk(KERN_ERR "fihsensor: start_resume() fail\r\n");
		}
		else {
			fihsensor_ctx.bIsIdle = false;
		}	
	}
#endif
/* } FIH; Tiger; 2009/6/10 */

	return 0;
}

static const struct i2c_device_id smb380_id[] = {
	{ "SMB380", 0 },
	{ }
};

static struct i2c_driver smb380_driver = {
	.probe = smb380_probe,
	.remove = smb380_remove,
	.suspend	= smb380_suspend,
	.resume		= smb380_resume,
	.id_table = smb380_id,
	.driver = {
		   .name = "SMB380",
		   },
};

static int __init fihsensor_init(void)
{
	int ret;

	int hwid = FIH_READ_HWID_FROM_SMEM();

	if(hwid >= CMCS_HW_VER_EVB1 && hwid <= CMCS_RTP_MP3) {
		// f902/f910
		profile = 1;
	}
	else if(hwid >= CMCS_CTP_PR1 && hwid <= CMCS_CTP_MP3) {
		// f903/f911
		profile = 2;
	}
	else if(hwid >= CMCS_7627_EVB1 && hwid <= CMCS_F913_MP1) {
		// f913/f905
		profile = 3;
	}	
	
	if((ret=i2c_add_driver(&ms3c_driver)))
	{
		printk(KERN_ERR "YAMAHA MS-3C compass driver init fail\r\n");	
	}
	else if((ret=i2c_add_driver(&smb380_driver)))
	{
		i2c_del_driver(&ms3c_driver);
	}
	
	return ret;
}

static void __exit fihsensor_exit(void)
{
	i2c_del_driver(&ms3c_driver);
	i2c_del_driver(&smb380_driver);
}

module_init(fihsensor_init);
module_exit(fihsensor_exit);

MODULE_AUTHOR("Tiger JT Lee <TigerJTLee@fihtdc.com>");
MODULE_DESCRIPTION("YAMAHA MS-3C compass driver");
MODULE_LICENSE("GPL");
