/*
 *     ov3642.c - Camera Sensor Driver
 *
 *     Copyright (C) 2009 Charles YS Huang <charlesyshuang@fihtdc.com>
 *     Copyright (C) 2008 FIH CO., Inc.
 *
 *     This program is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; version 2 of the License.
 */


#include <linux/version.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include "ov3642.h"

#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>
/* FIH, Charles Huang, 2010/03/01 { */
/* [FXX_CR], Add power onoff vreg */
#ifdef CONFIG_FIH_FXX
#include <mach/vreg.h>
#endif
/* } FIH, Charles Huang, 2010/03/01 */
/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
#include <mach/mpp.h>
#include <linux/completion.h>
#include <linux/hrtimer.h>
int ov3642_m_ledmod=0;
static pid_t ov3642_thread_id;
struct hrtimer ov3642_flashled_timer;
int ov3642_ledtime=16;
#define OV3642_FLASHLED_DELAY 190
DECLARE_COMPLETION(ov3642_flashled_comp);
#endif
/* } FIH, Charles Huang, 2009/09/01 */

DEFINE_MUTEX(ov3642_mut);

/* Micron OV3642 Registers and their values */
/* Sensor Core Registers */
#define  REG_OV3642_MODEL_ID_HI 0x302A
#define  REG_OV3642_MODEL_ID_LO 0x302B
#define  OV3642_MODEL_ID     0x3644
#define  OV3642_I2C_READ_SLAVE_ID     0x78 >> 1  
#define  OV3642_I2C_WRITE_SLAVE_ID     0x79 >> 1  
/* FIH, Charles Huang, 2009/07/29 { */
/* [FXX_CR], Calculate AEC */
#ifdef CONFIG_FIH_FXX
#define OV3642_CAPTURE_FRAMERATE 7.5
#define OV3642_PREVIEW_FRAMERATE 27
int ov3642_m_60Hz=0;
#endif
/* } FIH, Charles Huang, 2009/07/29 */

struct ov3642_work {
	struct work_struct work;
};

static struct  ov3642_work *ov3642_sensorw;
static struct  i2c_client *ov3642_client;

struct ov3642_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};

extern void brightness_onoff(int on);
extern void flash_settime(int time);
static struct ov3642_ctrl *ov3642_ctrl;

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#define OV3642_USE_VFS
#ifdef OV3642_USE_VFS
#define OV3642_INITREG "initreg"
#define OV3642_OEMREG "oemreg"
#define OV3642_PREVIEWREG "previewreg"
#define OV3642_SNAPREG "snapreg"
#define OV3642_SNAPAEREG "snapaereg"
#define OV3642_FLASHAEREG "flashaereg"
#define OV3642_IQREG "iqreg"
#define OV3642_LENSREG "lensreg"
#define OV3642_WRITEREG "writereg"
#define OV3642_GETREG "getreg"
#define OV3642_MCLK "mclk"
#define OV3642_MULTIPLE "multiple"
#define OV3642_FLASHTIME "flashtime"

/* MAX buf is ???? */
#define OV3642_MAX_VFS_INIT_INDEX 330
int ov3642_use_vfs_init_setting=0;
struct ov3642_i2c_reg_conf ov3642_vfs_init_settings_tbl[OV3642_MAX_VFS_INIT_INDEX];
uint16_t ov3642_vfs_init_settings_tbl_size= ARRAY_SIZE(ov3642_vfs_init_settings_tbl);

#define OV3642_MAX_VFS_OEM_INDEX 330
int ov3642_use_vfs_oem_setting=0;
struct ov3642_i2c_reg_conf ov3642_vfs_oem_settings_tbl[OV3642_MAX_VFS_OEM_INDEX];
uint16_t ov3642_vfs_oem_settings_tbl_size= ARRAY_SIZE(ov3642_vfs_oem_settings_tbl);

#define OV3642_MAX_VFS_PREVIEW_INDEX 330
int ov3642_use_vfs_preview_setting=0;
struct ov3642_i2c_reg_conf ov3642_vfs_preview_settings_tbl[OV3642_MAX_VFS_PREVIEW_INDEX];
uint16_t ov3642_vfs_preview_settings_tbl_size= ARRAY_SIZE(ov3642_vfs_preview_settings_tbl);

#define OV3642_MAX_VFS_SNAP_INDEX 330
int ov3642_use_vfs_snap_setting=0;
struct ov3642_i2c_reg_conf ov3642_vfs_snap_settings_tbl[OV3642_MAX_VFS_SNAP_INDEX];
uint16_t ov3642_vfs_snap_settings_tbl_size= ARRAY_SIZE(ov3642_vfs_snap_settings_tbl);

#define OV3642_MAX_VFS_SNAPAE_INDEX 330
int ov3642_use_vfs_snapae_setting=0;
struct ov3642_i2c_reg_conf ov3642_vfs_snapae_settings_tbl[OV3642_MAX_VFS_SNAPAE_INDEX];
uint16_t ov3642_vfs_snapae_settings_tbl_size= ARRAY_SIZE(ov3642_vfs_snapae_settings_tbl);

#define OV3642_MAX_VFS_FLASHAE_INDEX 330
int ov3642_use_vfs_flashae_setting=0;
struct ov3642_i2c_reg_conf ov3642_vfs_flashae_settings_tbl[OV3642_MAX_VFS_FLASHAE_INDEX];
uint16_t ov3642_vfs_flashae_settings_tbl_size= ARRAY_SIZE(ov3642_vfs_flashae_settings_tbl);

#define OV3642_MAX_VFS_IQ_INDEX 330
int ov3642_use_vfs_iq_setting=0;
struct ov3642_i2c_reg_conf ov3642_vfs_iq_settings_tbl[OV3642_MAX_VFS_IQ_INDEX];
uint16_t ov3642_vfs_iq_settings_tbl_size= ARRAY_SIZE(ov3642_vfs_iq_settings_tbl);

#define OV3642_MAX_VFS_LENS_INDEX 330
int ov3642_use_vfs_lens_setting=0;
struct ov3642_i2c_reg_conf ov3642_vfs_lens_settings_tbl[OV3642_MAX_VFS_LENS_INDEX];
uint16_t ov3642_vfs_lens_settings_tbl_size= ARRAY_SIZE(ov3642_vfs_lens_settings_tbl);

#define OV3642_MAX_VFS_WRITEREG_INDEX 330
int ov3642_use_vfs_writereg_setting=0;
struct ov3642_i2c_reg_conf ov3642_vfs_writereg_settings_tbl[OV3642_MAX_VFS_IQ_INDEX];
uint16_t ov3642_vfs_writereg_settings_tbl_size= ARRAY_SIZE(ov3642_vfs_writereg_settings_tbl);

#define OV3642_MAX_VFS_GETREG_INDEX 330
int ov3642_use_vfs_getreg_setting=0;
struct ov3642_i2c_reg_conf ov3642_vfs_getreg_settings_tbl[OV3642_MAX_VFS_GETREG_INDEX];
uint16_t ov3642_vfs_getreg_settings_tbl_size= ARRAY_SIZE(ov3642_vfs_getreg_settings_tbl);

int ov3642_use_vfs_mclk_setting=0;
int ov3642_use_vfs_multiple_setting=0;
int ov3642_use_vfs_flashtime_setting=0;
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */

static DECLARE_WAIT_QUEUE_HEAD(ov3642_wait_queue);
DECLARE_MUTEX(ov3642_sem);

/*=============================================================*/
static int ov3642_reset(const struct msm_camera_sensor_info *dev)
{
	int rc = 0;
	int HWID=FIH_READ_HWID_FROM_SMEM();

       gpio_tlmm_config(GPIO_CFG(0, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
       	gpio_tlmm_config(GPIO_CFG(17, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
      	gpio_tlmm_config(GPIO_CFG(121, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);

	if (HWID>=CMCS_7627_EVB1)
	{
		/* Switch enable */
		gpio_direction_output(121, 0);	
		msleep(10);
	}

	if (HWID==CMCS_HW_VER_EVB1 || HWID>=CMCS_7627_EVB1)
	{
		/* Clk switch */
		gpio_direction_output(17, 0);	
		msleep(10);
	}

	rc = gpio_request(dev->sensor_pwd, "ov3642");
	if (!rc) {
		rc = gpio_direction_output(dev->sensor_pwd, 0);
	}
	gpio_free(dev->sensor_pwd);
	
	rc = gpio_request(dev->sensor_reset, "ov3642");
	if (!rc) {
		rc = gpio_direction_output(dev->sensor_reset, 0);
		msleep(20);
		rc = gpio_direction_output(dev->sensor_reset, 1);
	}
	gpio_free(dev->sensor_reset);
	
       CDBG("[OV3642 3M] gpio_get_value(0) = %d\n", gpio_get_value(0));
       CDBG("[OV3642 3M] gpio_get_value(17) = %d\n", gpio_get_value(17));
       CDBG("[OV3642 3M] gpio_get_value(31) = %d\n", gpio_get_value(31));
       CDBG("[OV3642 3M] gpio_get_value(121) = %d\n", gpio_get_value(121));

	return rc;
}

static int32_t ov3642_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	if (i2c_transfer(ov3642_client->adapter, msg, 1) < 0) {
		CAM_USER_G0(KERN_ERR "ov3642_i2c_txdata failed, try again!\n");
		if (i2c_transfer(ov3642_client->adapter, msg, 1) < 0) {
			CAM_USER_G0(KERN_ERR "ov3642_i2c_txdata failed\n");
			CDBG("ov3642_i2c_txdata failed\n");
			return -EIO;
		}
	}

	return 0;
}

static int32_t ov3642_i2c_write(unsigned short saddr,
	unsigned short waddr, unsigned short wdata, enum ov3642_width width)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	switch (width) {
	case WORD_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (wdata & 0xFF00)>>8;
		buf[3] = (wdata & 0x00FF);

		rc = ov3642_i2c_txdata(saddr, buf, 4);
	}
		break;

	case BYTE_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (uint8_t) wdata;
		rc = ov3642_i2c_txdata(saddr, buf, 3);
	}
		break;

	default:
		break;
	}

	if (rc < 0)
		printk(
		KERN_ERR "ov3642 i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int32_t ov3642_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr   = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(ov3642_client->adapter, msgs, 2) < 0) {
		CAM_USER_G0(KERN_ERR "ov3642_i2c_rxdata failed, try again!\n");
		if (i2c_transfer(ov3642_client->adapter, msgs, 2) < 0) {
			CAM_USER_G0(KERN_ERR "ov3642_i2c_rxdata failed!\n");
			CDBG("ov3642_i2c_rxdata failed!\n");
			return -EIO;
		}
	}

	return 0;
}

static int32_t ov3642_i2c_read(unsigned short   saddr,
	unsigned short raddr, unsigned short *rdata, enum ov3642_width width)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	switch (width) {
	case WORD_LEN: {
		buf[0] = (raddr & 0xFF00)>>8;
		buf[1] = (raddr & 0x00FF);

		rc = ov3642_i2c_rxdata(saddr, buf, 2);
		if (rc < 0)
			return rc;

		*rdata = buf[0] << 8 | buf[1];
	}
		break;
	case BYTE_LEN: {
		buf[0] = (raddr & 0xFF00)>>8;
		buf[1] = (raddr & 0x00FF);

		rc = ov3642_i2c_rxdata(saddr, buf, 1);
		if (rc < 0)
			return rc;

		*rdata = buf[0];
	}
		break;

	default:
		break;
	}

	if (rc < 0)
		printk(KERN_ERR "ov3642_i2c_read failed!\n");

	return rc;
}

void ov3642_set_value_by_bitmask(uint16_t bitset, uint16_t mask, uint16_t  *new_value)
{
	uint16_t org;

	org= *new_value;
	*new_value = (org&~mask) | (bitset & mask);
}

static int32_t ov3642_i2c_write_table(
	struct ov3642_i2c_reg_conf const *reg_conf_tbl,
	int num_of_items_in_table)
{
	int i;
	int32_t rc = 0;

	for (i = 0; i < num_of_items_in_table; i++) {
		/* illegal addr */
		if (reg_conf_tbl->waddr== 0xFFFF)
			break;
		if (reg_conf_tbl->mask == 0xFFFF)
		{
			rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
				reg_conf_tbl->waddr, reg_conf_tbl->wdata,
				reg_conf_tbl->width);
		}else{
			uint16_t reg_value = 0;
			rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
				reg_conf_tbl->waddr, &reg_value, BYTE_LEN);
			ov3642_set_value_by_bitmask(reg_conf_tbl->wdata,reg_conf_tbl->mask,&reg_value);
			rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
					reg_conf_tbl->waddr, reg_value, BYTE_LEN);
		}
		
		if (rc < 0)
			break;
		if (reg_conf_tbl->mdelay_time != 0)
			msleep(reg_conf_tbl->mdelay_time);
		reg_conf_tbl++;
	}

	return rc;
}

#if 0
static int32_t ov3642_set_lens_roll_off(void)
{
	int32_t rc = 0;
	rc = ov3642_i2c_write_table(&ov3642_regs.rftbl[0],
		ov3642_regs.rftbl_size);
	return rc;
}
#endif

static long ov3642_reg_init(void)
{
#if 0
	int32_t i;
#endif
	long rc;
	uint16_t lens_id = 0;
#if 0
	/* PLL Setup Start */
	rc = ov3642_i2c_write_table(&mt9d112_regs.plltbl[0],
		ov3642_regs.plltbl_size);

	if (rc < 0)
		return rc;
	/* PLL Setup End   */
#endif

	/* Configure sensor for Preview mode and Snapshot mode */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
	if (ov3642_use_vfs_init_setting)
		rc = ov3642_i2c_write_table(&ov3642_vfs_init_settings_tbl[0],
			ov3642_vfs_init_settings_tbl_size);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
		rc = ov3642_i2c_write_table(&ov3642_regs.init_settings_tbl[0],
			ov3642_regs.init_settings_tbl_size);

	if (rc < 0)
		return rc;

	/* Configure sensor for image quality settings */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
	if (ov3642_use_vfs_iq_setting)
		rc = ov3642_i2c_write_table(&ov3642_vfs_iq_settings_tbl[0],
			ov3642_vfs_iq_settings_tbl_size);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
	rc = ov3642_i2c_write_table(&ov3642_regs.iq_settings_tbl[0],
		ov3642_regs.iq_settings_tbl_size);

	if (rc < 0)
		return rc;

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
	if (ov3642_use_vfs_lens_setting)
		rc = ov3642_i2c_write_table(&ov3642_vfs_lens_settings_tbl[0],
			ov3642_vfs_lens_settings_tbl_size);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
	{
		rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
			0x3D06, &lens_id, BYTE_LEN);

		if (rc < 0)
			return rc;

		if (lens_id==0x47)
			rc = ov3642_i2c_write_table(&ov3642_regs.gseolens_settings_tbl[0],
				ov3642_regs.gseolens_settings_tbl_size);
		else if (lens_id==0x46)
			rc = ov3642_i2c_write_table(&ov3642_regs.grdlens_settings_tbl[0],
				ov3642_regs.grdlens_settings_tbl_size);
		else
			rc = ov3642_i2c_write_table(&ov3642_regs.gseolens_settings_tbl[0],
				ov3642_regs.gseolens_settings_tbl_size);
	}
	if (rc < 0)
		return rc;

	/* Configure sensor for customer settings */
#if 0
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
	if (ov3642_use_vfs_oem_setting)
		rc = ov3642_i2c_write_table(&vfs_oem_settings_tbl[0],
			vfs_oem_settings_tbl_size);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
	rc = ov3642_i2c_write_table(&ov3642_regs.oem_settings_tbl[0],
		ov3642_regs.oem_settings_tbl_size);

	if (rc < 0)
		return rc;
#endif
#if 0
	/* Configure for Noise Reduction, Saturation and Aperture Correction */
	array_length = ov3642_regs.noise_reduction_reg_settings_;

	for (i = 0; i < array_length; i++) {

		rc = ov3642_i2c_write(ov3642_client->addr,
		ov3642_regs.noise_reduction_reg_settings[i].register_address,
		ov3642_regs.noise_reduction_reg_settings[i].register_value,
			WORD_LEN);

		if (rc < 0)
			return rc;
	}

	/* Set Color Kill Saturation point to optimum value */
	rc =
	ov3642_i2c_write(ov3642_client->addr,
	0x35A4,
	0x0593,
	WORD_LEN);
	if (rc < 0)
		return rc;

	rc = ov3642_i2c_write_table(&ov3642_regs.stbl[0],
		ov3642_regs.stbl_size);
	if (rc < 0)
		return rc;

	rc = ov3642_set_lens_roll_off();
	if (rc < 0)
		return rc;
#endif

	return 0;
}

static long ov3642_set_sensor_mode(int mode)
{
	long rc = 0;
/* FIH, Charles Huang, 2009/07/29 { */
/* [FXX_CR], Calculate AEC */
#ifdef CONFIG_FIH_FXX
	unsigned short   R0x350b,R0x5690,R0x5690preview,R0x350bpreview;

	unsigned short m_iWrite0x350b;
	unsigned short m_iWrite0x3502;
	unsigned short m_iWrite0x3501;
	unsigned short m_iWrite0x3500;

	unsigned short m_iFlashWrite0x350b=0;
	unsigned short m_iFlashWrite0x3502=0;
	unsigned short m_iFlashWrite0x3501=0;
	unsigned short m_iFlashWrite0x3500=0;
#endif
/* } FIH, Charles Huang, 2009/07/29 */

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Configure sensor for Preview mode */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
		if (ov3642_use_vfs_preview_setting)
			rc = ov3642_i2c_write_table(&ov3642_vfs_preview_settings_tbl[0],
				ov3642_vfs_preview_settings_tbl_size);
		else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
			rc = ov3642_i2c_write_table(&ov3642_regs.preview_settings_tbl[0],
				ov3642_regs.preview_settings_tbl_size);

		if (rc < 0)
			return rc;

	/* Configure sensor for customer settings */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
		if (ov3642_use_vfs_oem_setting)
			rc = ov3642_i2c_write_table(&ov3642_vfs_oem_settings_tbl[0],
				ov3642_vfs_oem_settings_tbl_size);
		else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
			rc = ov3642_i2c_write_table(&ov3642_regs.oem_settings_tbl[0],
				ov3642_regs.oem_settings_tbl_size);

		if (rc < 0)
			return rc;

/* FIH, Charles Huang, 2009/07/28 { */
/* [FXX_CR], double AEC */
#ifdef CONFIG_FIH_FXX
		/* Turn on 3A */
		rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
				0x3503, 0x00, BYTE_LEN);

		if (rc < 0)
			return rc;
#endif
/* } FIH, Charles Huang, 2009/07/28 */

		msleep(5);
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* read the registers value to the BYTE 0x350b  parameters. */
		rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
			0x350b, &R0x350bpreview, BYTE_LEN);

		if (rc < 0)
			return rc;

		/* read the registers value to the BYTE 0x5690  parameters. */
		rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
			0x5690, &R0x5690preview, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		CAM_USER_G0(KERN_INFO "ov3642 R0x350bpreview = 0x%x\n", R0x350bpreview);
		CAM_USER_G0(KERN_INFO "ov3642 R0x5690preview = 0x%x\n", R0x5690preview);

		/*stop AEC/AGC here :write_i2c(0x3503 ,0x07) */
		/* Turn off 3A */
		rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
				0x3503, 0x07, BYTE_LEN);
		/* MUST delay 30ms according to datasheet or sensor will crash */
		msleep(30);
		
/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
		CAM_USER_G0(KERN_INFO "snapshot ov3642 ov3642_m_ledmod = %d\n", ov3642_m_ledmod);
		if ((ov3642_m_ledmod==2) ||((ov3642_m_ledmod==1)&&(R0x5690preview <= 0x1C)))
		{
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
			if (ov3642_use_vfs_flashae_setting!=0 && ov3642_vfs_flashae_settings_tbl[0].wdata != 0xFFFF)
			{
				m_iFlashWrite0x350b=ov3642_vfs_flashae_settings_tbl[3].wdata;
				m_iFlashWrite0x3502=ov3642_vfs_flashae_settings_tbl[2].wdata;
				m_iFlashWrite0x3501=ov3642_vfs_flashae_settings_tbl[1].wdata;
				m_iFlashWrite0x3500=ov3642_vfs_flashae_settings_tbl[0].wdata;
			}else
			{
				m_iFlashWrite0x350b=0x19;
				m_iFlashWrite0x3502=0x90;
				m_iFlashWrite0x3501=0x30;
				m_iFlashWrite0x3500=0x0;
			}
				
			CAM_USER_G0(KERN_INFO "ov3642 m_iFlashWrite0x350b = 0x%x\n", m_iFlashWrite0x350b);
			CAM_USER_G0(KERN_INFO "ov3642 m_iFlashWrite0x3502 = 0x%x\n", m_iFlashWrite0x3502);
			CAM_USER_G0(KERN_INFO "ov3642 m_iFlashWrite0x3501 = 0x%x\n", m_iFlashWrite0x3501);
			CAM_USER_G0(KERN_INFO "ov3642 m_iFlashWrite0x3500 = 0x%x\n", m_iFlashWrite0x3500);

			rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
					0x350b, m_iFlashWrite0x350b, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
					0x3502, m_iFlashWrite0x3502, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
					0x3501, m_iFlashWrite0x3501, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
					0x3500, m_iFlashWrite0x3500, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
					0x350b, &m_iFlashWrite0x350b, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
					0x3502, &m_iFlashWrite0x3502, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
					0x3501, &m_iFlashWrite0x3501, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
					0x3500, &m_iFlashWrite0x3500, BYTE_LEN);

			if (rc < 0)
				return rc;

			CAM_USER_G0(KERN_INFO "ov3642 m_iFlashRead0x350b = 0x%x\n", m_iFlashWrite0x350b);
			CAM_USER_G0(KERN_INFO "ov3642 m_iFlashRead0x3502 = 0x%x\n", m_iFlashWrite0x3502);
			CAM_USER_G0(KERN_INFO "ov3642 m_iFlashRead0x3501 = 0x%x\n", m_iFlashWrite0x3501);
			CAM_USER_G0(KERN_INFO "ov3642 m_iFlashRead0x3500 = 0x%x\n", m_iFlashWrite0x3500);

			msleep(100);
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */

#if 1 /* Use flash or torch mode */
			flash_settime(16);
			ov3642_ledtime=1;
			hrtimer_cancel(&ov3642_flashled_timer);
			hrtimer_start(&ov3642_flashled_timer,
				ktime_set(0, 0),
				HRTIMER_MODE_REL);
			if (hrtimer_active(&ov3642_flashled_timer))
			CAM_USER_G0(KERN_INFO "%s: TIMER running\n", __func__);
#else
			brightness_onoff(1);
#endif
		}
#endif
/* } FIH, Charles Huang, 2009/09/01 */
		
		/* delay for getting stable R0x5690 */
		msleep(145);

		/* read the registers value to the BYTE 0x5690  parameters. */
		rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
			0x350b, &R0x350b, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		CAM_USER_G0(KERN_INFO "flash: ov642 R0x350b = 0x%x\n", R0x350b);

		rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
			0x5690, &R0x5690, BYTE_LEN);

		if (rc < 0)
			return rc;

		CAM_USER_G0(KERN_INFO "flash: ov3642 R0x5690 = 0x%x\n", R0x5690);

		/*change resolution from VGA to QXSGA here*/
		/* Configure sensor for Snapshot mode */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
		if (ov3642_use_vfs_snap_setting)
			rc = ov3642_i2c_write_table(&ov3642_vfs_snap_settings_tbl[0],
				ov3642_vfs_snap_settings_tbl_size);
		else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
			rc = ov3642_i2c_write_table(&ov3642_regs.snapshot_settings_tbl[0],
				ov3642_regs.snapshot_settings_tbl_size);

		if (rc < 0)
			return rc;

		msleep(5);

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
		/* flash auto or on and  R0x5690preview <= 0x67*/
		if ((ov3642_m_ledmod==2)
			|| ((ov3642_m_ledmod==1)&&(R0x5690preview <= 0x1C)))
		{
			/* write the gain and exposure to 0x350* registers by fih flash light table when using flash light */
			m_iWrite0x3500=0x0;
			if (R0x5690 < 0x1E)
			{
				m_iWrite0x350b=0x5;
				m_iWrite0x3501=0x60;
				m_iWrite0x3502=0x90;
			}else if (R0x5690 >= 0x1E && R0x5690 < 0x2A)
			{
				m_iWrite0x350b=0x00;
				m_iWrite0x3501=0x60;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x2A && R0x5690 < 0x3A)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x45;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x3A && R0x5690 < 0x54)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x30;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x54 && R0x5690 < 0x8C)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x18;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x8C && R0x5690 < 0xD5)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x10;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0xD5 && R0x5690 <= 0xFF)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x4;
				m_iWrite0x3502=0x0;
			}
			CAM_USER_G0(KERN_INFO "ov3642 m_iWrite0x350b = 0x%x\n", m_iWrite0x350b);
			CAM_USER_G0(KERN_INFO "ov3642 m_iWrite0x3502 = 0x%x\n", m_iWrite0x3502);
			CAM_USER_G0(KERN_INFO "ov3642 m_iWrite0x3501 = 0x%x\n", m_iWrite0x3501);
			CAM_USER_G0(KERN_INFO "ov3642 m_iWrite0x3500 = 0x%x\n", m_iWrite0x3500);
		
			rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
					0x350b, m_iWrite0x350b, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
					0x3502, m_iWrite0x3502, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
					0x3501, m_iWrite0x3501, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
					0x3500, m_iWrite0x3500, BYTE_LEN);

			if (rc < 0)
				return rc;


			rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
					0x350b, &m_iWrite0x350b, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
					0x3502, &m_iWrite0x3502, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
					0x3501, &m_iWrite0x3501, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
					0x3500, &m_iWrite0x3500, BYTE_LEN);

			if (rc < 0)
				return rc;

			CAM_USER_G0(KERN_INFO "ov3642 m_iRead0x350b = 0x%x\n", m_iWrite0x350b);
			CAM_USER_G0(KERN_INFO "ov3642 m_iRead0x3502 = 0x%x\n", m_iWrite0x3502);
			CAM_USER_G0(KERN_INFO "ov3642 m_iRead0x3501 = 0x%x\n", m_iWrite0x3501);
			CAM_USER_G0(KERN_INFO "ov3642 m_iRead0x3500 = 0x%x\n", m_iWrite0x3500);

		}
#endif
/* } FIH, Charles Huang, 2009/09/01 */



/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
	if (ov3642_use_vfs_snapae_setting!=0 && ov3642_vfs_snapae_settings_tbl[0].wdata != 0xFFFF)
	{
		m_iWrite0x350b=ov3642_vfs_snapae_settings_tbl[3].wdata;
		m_iWrite0x3502=ov3642_vfs_snapae_settings_tbl[2].wdata;
		m_iWrite0x3501=ov3642_vfs_snapae_settings_tbl[1].wdata;
		m_iWrite0x3500=ov3642_vfs_snapae_settings_tbl[0].wdata;

		CAM_USER_G0(KERN_INFO "ov3642 m_iWrite0x350b = 0x%x\n", m_iWrite0x350b);
		CAM_USER_G0(KERN_INFO "ov3642 m_iWrite0x3502 = 0x%x\n", m_iWrite0x3502);
		CAM_USER_G0(KERN_INFO "ov3642 m_iWrite0x3501 = 0x%x\n", m_iWrite0x3501);
		CAM_USER_G0(KERN_INFO "ov3642 m_iWrite0x3500 = 0x%x\n", m_iWrite0x3500);
		
		rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
				0x350b, m_iWrite0x350b, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
				0x3502, m_iWrite0x3502, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
				0x3501, m_iWrite0x3501, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov3642_i2c_write(OV3642_I2C_WRITE_SLAVE_ID,
				0x3500, m_iWrite0x3500, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;


		rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
				0x350b, &m_iWrite0x350b, BYTE_LEN);

		if (rc < 0)
			return rc;

		rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
				0x3502, &m_iWrite0x3502, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
				0x3501, &m_iWrite0x3501, BYTE_LEN);

		if (rc < 0)
			return rc;

		rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
				0x3500, &m_iWrite0x3500, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		CAM_USER_G0(KERN_INFO "ov3642 m_iRead0x350b = 0x%x\n", m_iWrite0x350b);
		CAM_USER_G0(KERN_INFO "ov3642 m_iRead0x3502 = 0x%x\n", m_iWrite0x3502);
		CAM_USER_G0(KERN_INFO "ov3642 m_iRead0x3501 = 0x%x\n", m_iWrite0x3501);
		CAM_USER_G0(KERN_INFO "ov3642 m_iRead0x3500 = 0x%x\n", m_iWrite0x3500);
	}
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
		/* Get third frame */
		/* 2frame * (1s/7.5frame) *1000ms */
		msleep(135);
		/*msleep(2*100*1000/Capture_Framerate);*/

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
		CAM_USER_G0(KERN_INFO "snapshot ov3642 ov3642_m_ledmod = %d\n", ov3642_m_ledmod);
		if ((ov3642_m_ledmod==2) ||((ov3642_m_ledmod==1)&&(R0x5690preview <= 0x1C)))
		{
			hrtimer_cancel(&ov3642_flashled_timer);
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
		if (ov3642_use_vfs_flashtime_setting!=0)
			hrtimer_start(&ov3642_flashled_timer,
				ktime_set(ov3642_use_vfs_flashtime_setting / 1000, (ov3642_use_vfs_flashtime_setting % 1000) * 1000000),
				HRTIMER_MODE_REL);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
		{
			flash_settime(16);
			ov3642_ledtime=1;
			hrtimer_start(&ov3642_flashled_timer,
				ktime_set(OV3642_FLASHLED_DELAY / 1000, (OV3642_FLASHLED_DELAY % 1000) * 1000000),
				HRTIMER_MODE_REL);

			if (hrtimer_active(&ov3642_flashled_timer))
				CAM_USER_G0(KERN_INFO "%s: TIMER running\n", __func__);
		}

		}
#endif
/* } FIH, Charles Huang, 2009/09/01 */

		break;

	default:
		return -EFAULT;
	}

	return 0;
}

static long ov3642_set_effect(int mode, int effect)
{
	long rc = 0;

	CDBG("ov3642_set_effect, mode = %d, effect = %d\n",
		mode, effect);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		break;

	default:
		break;
	}

	switch (effect) {
	case CAMERA_EFFECT_OFF: {//Normal
		struct ov3642_i2c_reg_conf const ov3642_effect_off_tbl[] = {
			{0x5001, 0x004F, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0000, BYTE_LEN, 0, 0x0058},
		};

		rc = ov3642_i2c_write_table(&ov3642_effect_off_tbl[0],
				ARRAY_SIZE(ov3642_effect_off_tbl));

		if (rc < 0)
			return rc;
	}
			break;

	case CAMERA_EFFECT_MONO: {//B&W
		struct ov3642_i2c_reg_conf const ov3642_effect_mono_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x5585, 0x0080, BYTE_LEN, 0, 0xFFFF},
			{0x5586, 0x0080, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_effect_mono_tbl[0],
				ARRAY_SIZE(ov3642_effect_mono_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_NEGATIVE: {//Negative
		struct ov3642_i2c_reg_conf const ov3642_effect_negative_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0040, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_effect_negative_tbl[0],
				ARRAY_SIZE(ov3642_effect_negative_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_SEPIA: {
		struct ov3642_i2c_reg_conf const ov3642_effect_sepia_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x5585, 0x0040, BYTE_LEN, 0, 0xFFFF},
			{0x5586, 0x00A0, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_effect_sepia_tbl[0],
				ARRAY_SIZE(ov3642_effect_sepia_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_BLUISH: {//Bluish
		struct ov3642_i2c_reg_conf const ov3642_effect_bluish_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x5585, 0x00A0, BYTE_LEN, 0, 0xFFFF},
			{0x5586, 0x0040, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_effect_bluish_tbl[0],
				ARRAY_SIZE(ov3642_effect_bluish_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_REDDISH: {//Reddish
		struct ov3642_i2c_reg_conf const ov3642_effect_reddish_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x5585, 0x0080, BYTE_LEN, 0, 0xFFFF},
			{0x5586, 0x00C0, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_effect_reddish_tbl[0],
				ARRAY_SIZE(ov3642_effect_reddish_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_GREENISH: {//Greenish
		struct ov3642_i2c_reg_conf const ov3642_effect_greenish_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x5585, 0x0060, BYTE_LEN, 0, 0xFFFF},
			{0x5586, 0x0060, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_effect_greenish_tbl[0],
				ARRAY_SIZE(ov3642_effect_greenish_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	default: {
		if (rc < 0)
			return rc;

		/*return -EFAULT;*/
		/* -EFAULT makes app fail */
		return 0;
	}
	}

	return rc;
}

static long ov3642_set_wb(int mode, int wb)
{
	long rc = 0;

	CDBG("ov3642_set_wb, mode = %d, wb = %d\n",
		mode, wb);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		break;

	default:
		break;
	}

	switch (wb) {
	case CAMERA_WB_AUTO: {
		struct ov3642_i2c_reg_conf const ov3642_wb_auto_tbl[] = {
			{0x3406, 0x0000, BYTE_LEN, 0, 0x0001},
		};

		rc = ov3642_i2c_write_table(&ov3642_wb_auto_tbl[0],
				ARRAY_SIZE(ov3642_wb_auto_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_WB_INCANDESCENT: {//TUNGSTEN
		struct ov3642_i2c_reg_conf const ov3642_wb_incandescent_tbl[] = {
			{0x3406, 0x0001, BYTE_LEN, 0, 0x0001},
			{0x3400, 0x0005, BYTE_LEN, 0, 0xFFFF},
			{0x3401, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x3402, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x3403, 0x00A0, BYTE_LEN, 0, 0xFFFF},
			{0x3404, 0x0006, BYTE_LEN, 0, 0xFFFF},
			{0x3405, 0x00D9, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_wb_incandescent_tbl[0],
				ARRAY_SIZE(ov3642_wb_incandescent_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_WB_FLUORESCENT: {//Office
		struct ov3642_i2c_reg_conf const ov3642_wb_fluorescent_tbl[] = {
			{0x3406, 0x0001, BYTE_LEN, 0, 0x0001},
			{0x3400, 0x0006, BYTE_LEN, 0, 0xFFFF},
			{0x3401, 0x00C0, BYTE_LEN, 0, 0xFFFF},
			{0x3402, 0x0005, BYTE_LEN, 0, 0xFFFF},
			{0x3403, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x3404, 0x0007, BYTE_LEN, 0, 0xFFFF},
			{0x3405, 0x001E, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_wb_fluorescent_tbl[0],
				ARRAY_SIZE(ov3642_wb_fluorescent_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_WB_DAYLIGHT: {//Sunny
		struct ov3642_i2c_reg_conf const ov3642_wb_daylight_tbl[] = {
			{0x3406, 0x0001, BYTE_LEN, 0, 0x0001},
			{0x3400, 0x0007, BYTE_LEN, 0, 0xFFFF},
			{0x3401, 0x008B, BYTE_LEN, 0, 0xFFFF},
			{0x3402, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x3403, 0x00C8, BYTE_LEN, 0, 0xFFFF},
			{0x3404, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x3405, 0x00DE, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_wb_daylight_tbl[0],
				ARRAY_SIZE(ov3642_wb_daylight_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_WB_CLOUDY_DAYLIGHT: {//Cloudy
		struct ov3642_i2c_reg_conf const ov3642_wb_cloudydaylight_tbl[] = {
			{0x3406, 0x0001, BYTE_LEN, 0, 0x0001},
			{0x3400, 0x0006, BYTE_LEN, 0, 0xFFFF},
			{0x3401, 0x007D, BYTE_LEN, 0, 0xFFFF},
			{0x3402, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x3403, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x3404, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x3405, 0x001A, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_wb_cloudydaylight_tbl[0],
				ARRAY_SIZE(ov3642_wb_cloudydaylight_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	default: {
		if (rc < 0)
			return rc;

		/*return -EFAULT;*/
		/* -EFAULT makes app fail */
		return 0;
	}
	}

	return rc;
}


static long ov3642_set_brightness(int mode, int brightness)
{
	long rc = 0;

	CDBG("ov3642_set_brightness, mode = %d, brightness = %d\n",
		mode, brightness);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		break;

	default:
		break;
	}

	switch (brightness) {
	case CAMERA_BRIGHTNESS_0: {
		struct ov3642_i2c_reg_conf const ov3642_brightness_0_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5589, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0008, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_brightness_0_tbl[0],
				ARRAY_SIZE(ov3642_brightness_0_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_BRIGHTNESS_1: {
		struct ov3642_i2c_reg_conf const ov3642_brightness_1_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5589, 0x0010, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0008, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_brightness_1_tbl[0],
				ARRAY_SIZE(ov3642_brightness_1_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_BRIGHTNESS_2: {
		struct ov3642_i2c_reg_conf const ov3642_brightness_2_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5589, 0x0008, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0008, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_brightness_2_tbl[0],
				ARRAY_SIZE(ov3642_brightness_2_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_BRIGHTNESS_3: {/* Default */
		struct ov3642_i2c_reg_conf const ov3642_brightness_3_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5589, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0000, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_brightness_3_tbl[0],
				ARRAY_SIZE(ov3642_brightness_3_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_BRIGHTNESS_4: {
		struct ov3642_i2c_reg_conf const ov3642_brightness_4_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5589, 0x0008, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0000, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_brightness_4_tbl[0],
				ARRAY_SIZE(ov3642_brightness_4_tbl));

		if (rc < 0)
			return rc;
	} 
		break;

	case CAMERA_BRIGHTNESS_5: {
		struct ov3642_i2c_reg_conf const ov3642_brightness_5_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5589, 0x0010, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0000, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_brightness_5_tbl[0],
				ARRAY_SIZE(ov3642_brightness_5_tbl));

		if (rc < 0)
			return rc;
	} 
		break;

	case CAMERA_BRIGHTNESS_6: {
		struct ov3642_i2c_reg_conf const ov3642_brightness_6_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5589, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0000, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_brightness_6_tbl[0],
				ARRAY_SIZE(ov3642_brightness_6_tbl));

		if (rc < 0)
			return rc;
	} 
		break;

	case CAMERA_BRIGHTNESS_7: 
	case CAMERA_BRIGHTNESS_8:
	case CAMERA_BRIGHTNESS_9:
	case CAMERA_BRIGHTNESS_10: {
		struct ov3642_i2c_reg_conf const ov3642_brightness_10_tbl[] = {
		};

		rc = ov3642_i2c_write_table(&ov3642_brightness_10_tbl[0],
				ARRAY_SIZE(ov3642_brightness_10_tbl));

		if (rc < 0)
			return rc;
	} 
		break;

	default: {
		if (rc < 0)
			return rc;

		/*return -EFAULT;*/
		/* -EFAULT makes app fail */
		return 0;
	}
	}

	return rc;
}

/* FIH, Charles Huang, 2009/11/05 { */
/* [FXX_CR], contrast function  */
#ifdef CONFIG_FIH_FXX
static long ov3642_set_contrast(int mode, int contrast)
{
	long rc = 0;

	CDBG("ov3642_set_contrast, mode = %d, contrast = %d\n",
		mode, contrast);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		break;

	default:
		break;
	}

	switch (contrast) {
	case CAMERA_CONTRAST_MINUS_2: {
		struct ov3642_i2c_reg_conf const ov3642_contrast_minus_2_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5587, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x5588, 0x0018, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_contrast_minus_2_tbl[0],
				ARRAY_SIZE(ov3642_contrast_minus_2_tbl));

		if (rc < 0)
			return rc;

	}
			break;

	case CAMERA_CONTRAST_MINUS_1: {
		struct ov3642_i2c_reg_conf const ov3642_contrast_minus_1_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5587, 0x001C, BYTE_LEN, 0, 0xFFFF},
			{0x5588, 0x001C, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_contrast_minus_1_tbl[0],
				ARRAY_SIZE(ov3642_contrast_minus_1_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_CONTRAST_ZERO: {
		struct ov3642_i2c_reg_conf const ov3642_contrast_zero_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5587, 0x0020, BYTE_LEN, 0, 0xFFFF},
			{0x5588, 0x0020, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_contrast_zero_tbl[0],
				ARRAY_SIZE(ov3642_contrast_zero_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_CONTRAST_POSITIVE_1: {
		struct ov3642_i2c_reg_conf const ov3642_contrast_positive_1_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5587, 0x0024, BYTE_LEN, 0, 0xFFFF},
			{0x5588, 0x0024, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_contrast_positive_1_tbl[0],
				ARRAY_SIZE(ov3642_contrast_positive_1_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_CONTRAST_POSITIVE_2: {
		struct ov3642_i2c_reg_conf const ov3642_contrast_positive_2_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x00CF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5587, 0x0028, BYTE_LEN, 0, 0xFFFF},
			{0x5588, 0x0028, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_contrast_positive_2_tbl[0],
				ARRAY_SIZE(ov3642_contrast_positive_2_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	default: {
		if (rc < 0)
			return rc;

		return -EFAULT;
	}
	}

	return rc;
}
#endif
/* } FIH, Charles Huang, 2009/11/05 */

static long ov3642_set_antibanding(int mode, int antibanding)
{
	long rc = 0;

	CDBG("ov3642_set_antibanding, mode = %d, antibanding = %d\n",
		mode, antibanding);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		break;

	default:
		break;
	}

	switch (antibanding) {
	case CAMERA_ANTIBANDING_OFF: {
		struct ov3642_i2c_reg_conf const ov3642_antibanding_off_tbl[] = {
			{0x3A00, 0x0000, BYTE_LEN, 0, 0x0020},
		};

		rc = ov3642_i2c_write_table(&ov3642_antibanding_off_tbl[0],
				ARRAY_SIZE(ov3642_antibanding_off_tbl));

		if (rc < 0)
			return rc;

		ov3642_m_60Hz=0;
	}
			break;

	case CAMERA_ANTIBANDING_60HZ: {
		struct ov3642_i2c_reg_conf const ov3642_antibanding_60hz_tbl[] = {
			{0x3A00, 0x0020, BYTE_LEN, 0, 0x0020},
			{0x3C01, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x3C00, 0x0000, BYTE_LEN, 0, 0x0004},
		};

		rc = ov3642_i2c_write_table(&ov3642_antibanding_60hz_tbl[0],
				ARRAY_SIZE(ov3642_antibanding_60hz_tbl));

		if (rc < 0)
			return rc;

		ov3642_m_60Hz=1;
	}
		break;

	case CAMERA_ANTIBANDING_50HZ: {
		struct ov3642_i2c_reg_conf const ov3642_antibanding_60hz_tbl[] = {
			{0x3A00, 0x0020, BYTE_LEN, 0, 0x0020},
			{0x3C01, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x3C00, 0x0004, BYTE_LEN, 0, 0x0004},
		};

		rc = ov3642_i2c_write_table(&ov3642_antibanding_60hz_tbl[0],
				ARRAY_SIZE(ov3642_antibanding_60hz_tbl));

		if (rc < 0)
			return rc;

		ov3642_m_60Hz=0;
	}
		break;

	case CAMERA_ANTIBANDING_AUTO: {
		struct ov3642_i2c_reg_conf const ov3642_antibanding_60hz_tbl[] = {
			{0x3A00, 0x0020, BYTE_LEN, 0, 0x0020},
			{0x3C01, 0x0000, BYTE_LEN, 0, 0x0080},
			{0x3C00, 0x0004, BYTE_LEN, 0, 0x0004},
		};

		rc = ov3642_i2c_write_table(&ov3642_antibanding_60hz_tbl[0],
				ARRAY_SIZE(ov3642_antibanding_60hz_tbl));

		if (rc < 0)
			return rc;

		ov3642_m_60Hz=0;
	}
		break;

	default: {
		if (rc < 0)
			return rc;

		ov3642_m_60Hz=0;
		return -EFAULT;
	}
	}

	return rc;
}

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
static long ov3642_set_ledmod(int mode, int ledmod)
{
	long rc = 0;

	CDBG("ov3642_set_ledmod, mode = %d, ledmod = %d\n",
		mode, ledmod);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		break;

	default:
		break;
	}

	switch (ledmod) {
	case CAMERA_LED_MODE_OFF: {
		ov3642_m_ledmod=0;
	}
			break;

	case CAMERA_LED_MODE_AUTO: {
		ov3642_m_ledmod=1;
	}
		break;

	case CAMERA_LED_MODE_ON: {
		ov3642_m_ledmod=2;
	}
		break;

	default: {
		if (rc < 0)
			return rc;

		ov3642_m_ledmod=0;
		return -EFAULT;
	}
	}

	return rc;
}
#endif
/* } FIH, Charles Huang, 2009/09/01 */

/* FIH, Charles Huang, 2009/11/05 { */
/* [FXX_CR], saturation mode function  */
#ifdef CONFIG_FIH_FXX
static long ov3642_set_saturation(int mode, int saturation)
{
	long rc = 0;

	CDBG("ov3642_set_saturation, mode = %d, saturation = %d\n",
		mode, saturation);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		break;

	default:
		break;
	}

	switch (saturation) {
	case CAMERA_SATURATION_MINUS_2: {
		struct ov3642_i2c_reg_conf const ov3642_saturation_mnius_2_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x0080},
			{0x5583, 0x0040, BYTE_LEN, 0, 0xFFFF},
			{0x5584, 0x0040, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0002, BYTE_LEN, 0, 0x0002},
		};

		rc = ov3642_i2c_write_table(&ov3642_saturation_mnius_2_tbl[0],
				ARRAY_SIZE(ov3642_saturation_mnius_2_tbl));

		if (rc < 0)
			return rc;

	}
			break;

	case CAMERA_SATURATION_MINUS_1: {
		struct ov3642_i2c_reg_conf const ov3642_saturation_mnius_1_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x0080},
			{0x5583, 0x0050, BYTE_LEN, 0, 0xFFFF},
			{0x5584, 0x0050, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0002, BYTE_LEN, 0, 0x0002},
		};

		rc = ov3642_i2c_write_table(&ov3642_saturation_mnius_1_tbl[0],
				ARRAY_SIZE(ov3642_saturation_mnius_1_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SATURATION_ZERO: {
		struct ov3642_i2c_reg_conf const ov3642_saturation_zero_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x0080},
			{0x5583, 0x0060, BYTE_LEN, 0, 0xFFFF},
			{0x5584, 0x0060, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0002, BYTE_LEN, 0, 0x0002},
		};

		rc = ov3642_i2c_write_table(&ov3642_saturation_zero_tbl[0],
				ARRAY_SIZE(ov3642_saturation_zero_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SATURATION_POSITIVE_1: {
		struct ov3642_i2c_reg_conf const ov3642_saturation_positive_1_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x0080},
			{0x5583, 0x0070, BYTE_LEN, 0, 0xFFFF},
			{0x5584, 0x0070, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0002, BYTE_LEN, 0, 0x0002},
		};

		rc = ov3642_i2c_write_table(&ov3642_saturation_positive_1_tbl[0],
				ARRAY_SIZE(ov3642_saturation_positive_1_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SATURATION_POSITIVE_2: {
		struct ov3642_i2c_reg_conf const ov3642_saturation_positive_2_tbl[] = {
			{0x5001, 0x00CF, BYTE_LEN, 0, 0x0080},
			{0x5583, 0x0080, BYTE_LEN, 0, 0xFFFF},
			{0x5584, 0x0080, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0002, BYTE_LEN, 0, 0x0002},
		};

		rc = ov3642_i2c_write_table(&ov3642_saturation_positive_2_tbl[0],
				ARRAY_SIZE(ov3642_saturation_positive_2_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	default: {
		if (rc < 0)
			return rc;

		return -EFAULT;
	}
	}

	return rc;
}
#endif
/* } FIH, Charles Huang, 2009/11/05 */

/* FIH, Charles Huang, 2009/11/05 { */
/* [FXX_CR], sharpness mode function  */
#ifdef CONFIG_FIH_FXX
static long ov3642_set_sharpness(int mode, int sharpness)
{
	long rc = 0;

	CDBG("ov3642_set_sharpness, mode = %d, sharpness = %d\n",
		mode, sharpness);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		break;

	default:
		break;
	}

	switch (sharpness) {
	case CAMERA_SHARPNESS_ZERO: {
		struct ov3642_i2c_reg_conf const ov3642_sharpness_zero_tbl[] = {
			{0x530A, 0x0000, BYTE_LEN, 0, 0x0008},
			{0x530C, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x530D, 0x000C, BYTE_LEN, 0, 0xFFFF},
			{0x5312, 0x0040, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_sharpness_zero_tbl[0],
				ARRAY_SIZE(ov3642_sharpness_zero_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SHARPNESS_POSITIVE_1: {
		struct ov3642_i2c_reg_conf const ov3642_sharpness_positive_1_tbl[] = {
			{0x530A, 0x0000, BYTE_LEN, 0, 0x0008},
			{0x530C, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x530D, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x5312, 0x0020, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_sharpness_positive_1_tbl[0],
				ARRAY_SIZE(ov3642_sharpness_positive_1_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SHARPNESS_POSITIVE_2: {
		struct ov3642_i2c_reg_conf const ov3642_sharpness_positive_2_tbl[] = {
			{0x530A, 0x0000, BYTE_LEN, 0, 0x0008},
			{0x530C, 0x0008, BYTE_LEN, 0, 0xFFFF},
			{0x530D, 0x0030, BYTE_LEN, 0, 0xFFFF},
			{0x5312, 0x0010, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_sharpness_positive_2_tbl[0],
				ARRAY_SIZE(ov3642_sharpness_positive_2_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	default: {
		if (rc < 0)
			return rc;

		return -EFAULT;
	}
	}

	return rc;
}
#endif
/* } FIH, Charles Huang, 2009/11/05 */

/* FIH, Charles Huang, 2009/11/05 { */
/* [FXX_CR], metering mode function  */
#ifdef CONFIG_FIH_FXX
static long ov3642_set_meteringmod(int mode, int meteringmod)
{
	long rc = 0;

	CDBG("ov3642_set_meteringmod, mode = %d, meteringmod = %d\n",
		mode, meteringmod);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		break;

	default:
		break;
	}

	switch (meteringmod) {
	case CAMERA_AVERAGE_METERING: {
		struct ov3642_i2c_reg_conf const ov3642_meteringmod_average_tbl[] = {
			{0x5688, 0x00FF, BYTE_LEN, 0, 0xFFFF},
			{0x5689, 0x00FF, BYTE_LEN, 0, 0xFFFF},
			{0x568A, 0x00FF, BYTE_LEN, 0, 0xFFFF},
			{0x568B, 0x00FF, BYTE_LEN, 0, 0xFFFF},
			{0x568C, 0x00FF, BYTE_LEN, 0, 0xFFFF},
			{0x568D, 0x00FF, BYTE_LEN, 0, 0xFFFF},
			{0x568E, 0x00FF, BYTE_LEN, 0, 0xFFFF},
			{0x568F, 0x00FF, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_meteringmod_average_tbl[0],
				ARRAY_SIZE(ov3642_meteringmod_average_tbl));

		if (rc < 0)
			return rc;

	}
			break;

	case CAMERA_CENTER_METERING: {
		struct ov3642_i2c_reg_conf const ov3642_meteringmod_center_tbl[] = {
			{0x5688, 0x0062, BYTE_LEN, 0, 0xFFFF},
			{0x5689, 0x0026, BYTE_LEN, 0, 0xFFFF},
			{0x568A, 0x00E6, BYTE_LEN, 0, 0xFFFF},
			{0x568B, 0x006E, BYTE_LEN, 0, 0xFFFF},
			{0x568C, 0x00EA, BYTE_LEN, 0, 0xFFFF},
			{0x568D, 0x00AE, BYTE_LEN, 0, 0xFFFF},
			{0x568E, 0x00A6, BYTE_LEN, 0, 0xFFFF},
			{0x568F, 0x006A, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_meteringmod_center_tbl[0],
				ARRAY_SIZE(ov3642_meteringmod_center_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SPOT_METERING: {
		struct ov3642_i2c_reg_conf const ov3642_meteringmod_spot_tbl[] = {
			{0x5688, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5689, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x568A, 0x0010, BYTE_LEN, 0, 0xFFFF},
			{0x568B, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x568C, 0x0010, BYTE_LEN, 0, 0xFFFF},
			{0x568D, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x568E, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x568F, 0x0000, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov3642_i2c_write_table(&ov3642_meteringmod_spot_tbl[0],
				ARRAY_SIZE(ov3642_meteringmod_spot_tbl));

		if (rc < 0)
			return rc;

	}
		break;
	default: {
		if (rc < 0)
			return rc;

		return -EFAULT;
	}
	}

	return rc;
}
#endif
/* } FIH, Charles Huang, 2009/11/05 */

/* FIH, Charles Huang, 2009/11/05 { */
/* [FXX_CR], scene mode function  */
#ifdef CONFIG_FIH_FXX
static long ov3642_set_scenemod(int mode, int scenemod)
{
	long rc = 0;

	CDBG("ov3642_set_scenemod, mode = %d, scenemod = %d\n",
		mode, scenemod);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		break;

	default:
		break;
	}

	switch (scenemod) {
	case CAMERA_SCENE_MODE_AUTO: {
		struct ov3642_i2c_reg_conf const ov3642_scenemod_auto_tbl[] = {
		};

		rc = ov3642_i2c_write_table(&ov3642_scenemod_auto_tbl[0],
				ARRAY_SIZE(ov3642_scenemod_auto_tbl));

		if (rc < 0)
			return rc;

	}
			break;

	case CAMERA_SCENE_MODE_LANDSCAPE: {
		struct ov3642_i2c_reg_conf const ov3642_scenemod_landscape_tbl[] = {
		};

		rc = ov3642_i2c_write_table(&ov3642_scenemod_landscape_tbl[0],
				ARRAY_SIZE(ov3642_scenemod_landscape_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SCENE_MODE_PORTRAIT: {
		struct ov3642_i2c_reg_conf const ov3642_scenemod_portrait_tbl[] = {
		};

		rc = ov3642_i2c_write_table(&ov3642_scenemod_portrait_tbl[0],
				ARRAY_SIZE(ov3642_scenemod_portrait_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SCENE_MODE_NIGHT: {
		struct ov3642_i2c_reg_conf const ov3642_scenemod_night_tbl[] = {
		};

		rc = ov3642_i2c_write_table(&ov3642_scenemod_night_tbl[0],
				ARRAY_SIZE(ov3642_scenemod_night_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SCENE_MODE_NIGHT_PORTRAIT: {
		struct ov3642_i2c_reg_conf const ov3642_scenemod_night_portrait_tbl[] = {
		};

		rc = ov3642_i2c_write_table(&ov3642_scenemod_night_portrait_tbl[0],
				ARRAY_SIZE(ov3642_scenemod_night_portrait_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SCENE_MODE_SUNSET: {
		struct ov3642_i2c_reg_conf const ov3642_scenemod_sunset_tbl[] = {
		};

		rc = ov3642_i2c_write_table(&ov3642_scenemod_sunset_tbl[0],
				ARRAY_SIZE(ov3642_scenemod_sunset_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	default: {
		if (rc < 0)
			return rc;

		return -EFAULT;
	}
	}

	return rc;
}
#endif
/* } FIH, Charles Huang, 2009/11/05 */

static int ov3642_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	uint16_t model_id = 0;
	int rc = 0;
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifndef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
	struct file *ov3642_Fs = NULL;
	struct inode		*inode = NULL;
	int			length;
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */

	/* OV suggested Power up block End */
	/* Read the Model ID of the sensor */
	/* Read REG_OV3642_MODEL_ID_HI & REG_OV3642_MODEL_ID_LO */
	rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
		REG_OV3642_MODEL_ID_HI, &model_id, WORD_LEN);
	if (rc < 0)
		goto init_probe_fail;

	CDBG("ov3642 model_id = 0x%x\n", model_id);

	/* Check if it matches it with the value in Datasheet */
	if (model_id != OV3642_MODEL_ID) {
		rc = -EFAULT;
		goto init_probe_fail;
	}

	/* Get version */
	rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
	0x302A, &model_id, BYTE_LEN);
	CDBG("ov3642 version reg 0x302A = 0x%x\n", model_id);

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifndef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
	if(ov3642_Fs==NULL)
	{
		ov3642_Fs = filp_open("/etc/ov3642.cfg", O_RDONLY, 0);

		if(IS_ERR(ov3642_Fs))
		{
			ov3642_Fs = NULL;
			printk(KERN_ERR "Can't open /etc/ov3642.cfg\n");
		}


		if (!ov3642_Fs->f_op)
		{
			ov3642_Fs = NULL;
			printk("%s: File Operation Method Error\n", __FUNCTION__);
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
		inode = ov3642_Fs->f_path.dentry->d_inode;
#else
		inode = ov3642_Fs->f_dentry->d_inode;
#endif

		if (!inode)
		{
			printk("%s: Get inode from filp failed\n", __FUNCTION__);
		}
		/* file's size */
		length = i_size_read(inode->i_mapping->host);

		printk(KERN_INFO "Open /etc/ov3642.cfg\n");
		filp_close(ov3642_Fs, NULL);   
	}
#endif /* OV3642_USE_VFS */
#endif
/* } FIH, Charles Huang, 2009/06/24 */

	rc = ov3642_reg_init();
	if (rc < 0)
		goto init_probe_fail;

	return rc;

init_probe_fail:
	return rc;
}

int ov3642_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
/* FIH, Charles Huang, 2010/03/01 { */
/* [FXX_CR], Add power onoff vreg */
#ifdef CONFIG_FIH_FXX
	/* VDD 3V F0X:rftx FA3:wlan*/
	struct vreg *vreg_vcm;
	/* DCORE 1.5V F0X:gp2 FA3:msme2*/
	struct vreg *vreg_dcore;
	/* ACORE 2.8V F0X:gp3 FA3:gp1*/
	struct vreg *vreg_acore;
	/* io 2.6V F0X:msmp FA3:gp3*/
	struct vreg *vreg_io;
#endif
/* } FIH, Charles Huang, 2010/03/01 */

	ov3642_ctrl = kzalloc(sizeof(struct ov3642_ctrl), GFP_KERNEL);
	if (!ov3642_ctrl) {
		CDBG("ov3642_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		ov3642_ctrl->sensordata = data;

/* FIH, Charles Huang, 2010/03/01 { */
/* [FXX_CR], Add power onoff vreg */
#ifdef CONFIG_FIH_FXX
	/* turn on power */
	if (FIH_READ_HWID_FROM_SMEM() < CMCS_7627_EVB1)
	{
		vreg_vcm = vreg_get(NULL, "rftx");
		vreg_set_level(vreg_vcm, 3000);
		vreg_enable(vreg_vcm);

		vreg_dcore = vreg_get(NULL, "gp2");
		vreg_set_level(vreg_dcore, 1500);
		vreg_enable(vreg_dcore);

		vreg_acore = vreg_get(NULL, "gp3");
		vreg_set_level(vreg_acore, 2800);
		vreg_enable(vreg_acore);
	}else
	{
		/* vcm use the same vreg as touch, we don't turn off by sensor driver */
		//vreg_vcm = vreg_get(NULL, "wlan");
		//vreg_set_level(vreg_vcm, 3000);
		//vreg_enable(vreg_vcm);

		vreg_dcore = vreg_get(NULL, "msme2");
		vreg_set_level(vreg_dcore, 1500);
		vreg_enable(vreg_dcore);

		vreg_acore = vreg_get(NULL, "gp1");
		vreg_set_level(vreg_acore, 2800);
		vreg_enable(vreg_acore);

		vreg_io = vreg_get(NULL, "gp3");
		vreg_set_level(vreg_io, 2600);
		vreg_enable(vreg_io);
	}
#endif
/* } FIH, Charles Huang, 2010/03/01 */

	rc = ov3642_reset(data);
	if (rc < 0) {
		CDBG("reset failed!\n");
		goto init_fail;
	}

	/* EVB CAMIF cannont handle in 24MHz */
	/* EVB use 12.8MHz */
	/* R4215 couldn't use too high clk */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
	if (ov3642_use_vfs_mclk_setting!=0)
		msm_camio_clk_rate_set(ov3642_use_vfs_mclk_setting);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
	if (FIH_READ_HWID_FROM_SMEM()==CMCS_HW_VER_EVB1)
		msm_camio_clk_rate_set(12288000);
	else
		msm_camio_clk_rate_set(24000000);
	msleep(25);

	msm_camio_camif_pad_reg_reset();

	rc = ov3642_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("ov3642_sensor_init failed!\n");
		goto init_fail;
	}

init_done:
	return rc;

init_fail:
	kfree(ov3642_ctrl);
	return rc;
}

static int ov3642_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov3642_wait_queue);
	return 0;
}

int ov3642_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(
				&cfg_data,
				(void *)argp,
				sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&ov3642_sem); */
	CAM_USER_G1("ov3642_sensor_config, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

	CDBG("ov3642_sensor_config, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = ov3642_set_sensor_mode(
					cfg_data.mode);
		break;

	case CFG_SET_EFFECT:
		rc = ov3642_set_effect(
					cfg_data.mode,
					cfg_data.cfg.effect);
		break;

	case CFG_START:
		rc = -EFAULT;
		break;
		
	case CFG_PWR_UP:
		rc = -EFAULT;
		break;

	case CFG_PWR_DOWN:
		rc = -EFAULT;
		break;

	case CFG_WRITE_EXPOSURE_GAIN:
		rc = -EFAULT;
		break;

	case CFG_MOVE_FOCUS:
		rc = -EFAULT;
		break;

	case CFG_REGISTER_TO_REAL_GAIN:
		rc = -EFAULT;
		break;

	case CFG_REAL_TO_REGISTER_GAIN:
		rc = -EFAULT;
		break;

	case CFG_SET_FPS:
		rc = -EFAULT;
		break;

	case CFG_SET_PICT_FPS:
		rc = -EFAULT;
		break;

	case CFG_SET_BRIGHTNESS:
		rc = ov3642_set_brightness(
					cfg_data.mode,
					cfg_data.cfg.brightness);
		break;

	case CFG_SET_CONTRAST:
		rc = ov3642_set_contrast(
					cfg_data.mode,
					cfg_data.cfg.contrast);
		break;

	case CFG_SET_EXPOSURE_MODE:
		rc = -EFAULT;
		break;

	case CFG_SET_WB:
		rc = ov3642_set_wb(
					cfg_data.mode,
					cfg_data.cfg.wb);
		break;

	case CFG_SET_ANTIBANDING:
		rc = ov3642_set_antibanding(
					cfg_data.mode,
					cfg_data.cfg.antibanding);
		break;

	case CFG_SET_EXP_GAIN:
		rc = -EFAULT;
		break;

	case CFG_SET_PICT_EXP_GAIN:
		rc = -EFAULT;
		break;

	case CFG_SET_LENS_SHADING:
		rc = -EFAULT;
		break;

	case CFG_GET_PICT_FPS:
		rc = -EFAULT;
		break;

	case CFG_GET_PREV_L_PF:
		rc = -EFAULT;
		break;

	case CFG_GET_PREV_P_PL:
		rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		rc = -EFAULT;
		break;

	case CFG_GET_PICT_P_PL:
		rc = -EFAULT;
		break;

	case CFG_GET_AF_MAX_STEPS:
		rc = -EFAULT;
		break;

	case CFG_GET_PICT_MAX_EXP_LC:
		rc = -EFAULT;
		break;

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
	case CFG_SET_LEDMOD:
		rc = ov3642_set_ledmod(
					cfg_data.mode,
					cfg_data.cfg.ledmod);
		break;
#endif
/* } FIH, Charles Huang, 2009/09/01 */

/* FIH, Charles Huang, 2009/11/05 { */
/* [FXX_CR], saturation function  */
#ifdef CONFIG_FIH_FXX
	case CFG_SET_SATURATION:
		rc = ov3642_set_saturation(
					cfg_data.mode,
					cfg_data.cfg.saturation);
		break;
#endif
/* } FIH, Charles Huang, 2009/11/05 */

/* FIH, Charles Huang, 2009/11/05 { */
/* [FXX_CR], sharpness function  */
#ifdef CONFIG_FIH_FXX
	case CFG_SET_SHARPNESS:
		rc = ov3642_set_sharpness(
					cfg_data.mode,
					cfg_data.cfg.sharpness);
		break;
#endif
/* } FIH, Charles Huang, 2009/11/05 */

/* FIH, Charles Huang, 2009/11/05 { */
/* [FXX_CR], metering mode function  */
#ifdef CONFIG_FIH_FXX
	case CFG_SET_METERINGMOD:
		rc = ov3642_set_meteringmod(
					cfg_data.mode,
					cfg_data.cfg.meteringmod);
		break;
#endif
/* } FIH, Charles Huang, 2009/11/05 */

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], scene mode function  */
#ifdef CONFIG_FIH_FXX
	case CFG_SET_SCENEMOD:
		rc = ov3642_set_scenemod(
					cfg_data.mode,
					cfg_data.cfg.scenemod);
		break;
#endif
/* } FIH, Charles Huang, 2009/11/05 */
	default:
		rc = -EINVAL;
		break;
	}

	/* up(&ov3642_sem); */

	return rc;
}

int ov3642_sensor_release(void)
{
	int rc = 0;
/* FIH, Charles Huang, 2010/03/01 { */
/* [FXX_CR], Add power onoff vreg */
#ifdef CONFIG_FIH_FXX
	/* VDD 3V F0X:rftx FA3:wlan*/
	struct vreg *vreg_vcm;
	/* DCORE 1.5V F0X:gp2 FA3:msme2*/
	struct vreg *vreg_dcore;
	/* ACORE 2.8V F0X:gp3 FA3:gp1*/
	struct vreg *vreg_acore;
	/* io 2.6V F0X:msmp FA3:gp3*/
	struct vreg *vreg_io;
#endif
/* } FIH, Charles Huang, 2010/03/01 */
	const struct msm_camera_sensor_info *dev;
	/* down(&ov3642_sem); */
	int HWID=FIH_READ_HWID_FROM_SMEM();

	mutex_lock(&ov3642_mut);

	dev = ov3642_ctrl->sensordata;
	rc = gpio_request(dev->sensor_reset, "ov3642");
	if (!rc) {
		rc = gpio_direction_output(dev->sensor_reset, 0);
	}
	gpio_free(dev->sensor_reset);

	rc = gpio_request(dev->sensor_pwd, "ov3642");
	if (!rc) {
		rc = gpio_direction_output(dev->sensor_pwd, 1);
	}
	gpio_free(dev->sensor_pwd);

	if (HWID>=CMCS_7627_EVB1)
	{
		/* Switch disable */
		gpio_tlmm_config(GPIO_CFG(121, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		gpio_direction_output(121, 1);	
		msleep(10);
	}

       CDBG("[OV3642 3M]  gpio_get_value(0) = %d\n", gpio_get_value(0));
       CDBG("[OV3642 3M]  gpio_get_value(17) = %d\n", gpio_get_value(17));
       CDBG("[OV3642 3M]  gpio_get_value(31) = %d\n", gpio_get_value(31));
       CDBG("[OV3642 3M]  gpio_get_value(121) = %d\n", gpio_get_value(121));

       gpio_tlmm_config(GPIO_CFG(0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
       gpio_tlmm_config(GPIO_CFG(17, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);

/* FIH, Charles Huang, 2010/03/01 { */
/* [FXX_CR], Add power onoff vreg */
#ifdef CONFIG_FIH_FXX
	/* turn on power */
	if (FIH_READ_HWID_FROM_SMEM() < CMCS_7627_EVB1)
	{
		vreg_vcm = vreg_get(NULL, "rftx");
		vreg_disable(vreg_vcm);

		vreg_dcore = vreg_get(NULL, "gp2");
		vreg_disable(vreg_dcore);

		vreg_acore = vreg_get(NULL, "gp3");
		vreg_disable(vreg_acore);
	}else
	{
		/* vcm use the same vreg as touch, we don't turn off by sensor driver */
		//vreg_vcm = vreg_get(NULL, "wlan");
		//vreg_disable(vreg_vcm);

		vreg_dcore = vreg_get(NULL, "msme2");
		vreg_disable(vreg_dcore);

		vreg_acore = vreg_get(NULL, "gp1");
		vreg_disable(vreg_acore);

		vreg_io = vreg_get(NULL, "gp3");
		vreg_disable(vreg_io);
	}
#endif
/* } FIH, Charles Huang, 2010/03/01 */

	kfree(ov3642_ctrl);
	ov3642_ctrl = NULL;
	/* up(&ov3642_sem); */
	mutex_unlock(&ov3642_mut);
	return rc;
}

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
void ov3642_get_param(const char *buf, size_t count, struct ov3642_i2c_reg_conf *tbl, 
	unsigned short tbl_size, int *use_setting, int param_num)
{
	unsigned short waddr;
	unsigned short wdata;
	enum ov3642_width width;
	unsigned short mdelay_time;
	unsigned short mask;
	char param1[10],param2[10],param3[10],param4[10];
	int read_count;
	const char *pstr;
	int vfs_index=0;
	pstr=buf;

	CDBG("count=%d\n",count);
	do
	{
		if (param_num ==3)
		read_count=sscanf(pstr,"%4s,%2s,%2s",param1,param2,param3);
		else
			read_count=sscanf(pstr,"%4s,%2s,%s,%2s",param1,param2,param3,param4);

      		//CDBG("pstr=%s\n",pstr);
      		//CDBG("read_count=%d,count=%d\n",read_count,count);
		if (read_count ==3)
		{
			waddr=simple_strtoul(param1,NULL,16);
			wdata=simple_strtoul(param2,NULL,16);
			width=1;
			mdelay_time=0;
			mask=simple_strtoul(param3,NULL,16);
				
			tbl[vfs_index].waddr= waddr;
			tbl[vfs_index].wdata= wdata;
			tbl[vfs_index].width= width;
			tbl[vfs_index].mdelay_time= mdelay_time;
			tbl[vfs_index].mask= mask;
			vfs_index++;

			if (vfs_index == tbl_size)
			{
				CDBG("Just match MAX_VFS_INDEX\n");
				*use_setting=1;
			}else if (vfs_index > tbl_size)
			{
				CDBG("Out of range MAX_VFS_INDEX\n");
				*use_setting=0;
				break;
			}
			
       		//CDBG("param1=%s,param2=%s,param3=%s\n",param1,param2,param3);
       		//CDBG("waddr=0x%04X,wdata=0x%04X,width=%d,mdelay_time=%d,mask=0x%04X\n",waddr,wdata,width,mdelay_time,mask);
		}else if (read_count==4)
		{
			waddr=simple_strtoul(param1,NULL,16);
			wdata=simple_strtoul(param2,NULL,16);
			width=1;
			mdelay_time=simple_strtoul(param3,NULL,16);;
			mask=simple_strtoul(param4,NULL,16);
				
			tbl[vfs_index].waddr= waddr;
			tbl[vfs_index].wdata= wdata;
			tbl[vfs_index].width= width;
			tbl[vfs_index].mdelay_time= mdelay_time;
			tbl[vfs_index].mask= mask;
			vfs_index++;

			if (vfs_index == tbl_size)
			{
				CDBG("Just match MAX_VFS_INDEX\n");
				*use_setting=1;
			}else if (vfs_index > tbl_size)
			{
				CDBG("Out of range MAX_VFS_INDEX\n");
				*use_setting=0;
				break;
			}
			
       		//CDBG("param1=%s,param2=%s,param3=%s\n",param1,param2,param3);
       		//CDBG("waddr=0x%04X,wdata=0x%04X,width=%d,mdelay_time=%d,mask=0x%04X\n",waddr,wdata,width,mdelay_time,mask);
		}else{
			tbl[vfs_index].waddr= 0xFFFF;
			tbl[vfs_index].wdata= 0xFFFF;
			tbl[vfs_index].width= 1;
			tbl[vfs_index].mdelay_time= 0xFFFF;
			tbl[vfs_index].mask= 0xFFFF;
			*use_setting=1;
			break;
		}
		/* get next line */
		pstr=strchr(pstr, '\n');
		if (pstr==NULL)
			break;
		pstr++;
	}while(read_count!=0);


}

static ssize_t ov3642_write_initreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov3642_get_param(buf, count, &ov3642_vfs_init_settings_tbl[0], ov3642_vfs_init_settings_tbl_size, &ov3642_use_vfs_init_setting, 3);
	return count;
}

static ssize_t ov3642_write_oemtreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov3642_get_param(buf, count, &ov3642_vfs_oem_settings_tbl[0], ov3642_vfs_oem_settings_tbl_size, &ov3642_use_vfs_oem_setting, 3);
	return count;
}

static ssize_t ov3642_write_previewreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov3642_get_param(buf, count, &ov3642_vfs_preview_settings_tbl[0], ov3642_vfs_preview_settings_tbl_size, &ov3642_use_vfs_preview_setting, 3);
	return count;
}

static ssize_t ov3642_write_snapreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov3642_get_param(buf, count, &ov3642_vfs_snap_settings_tbl[0], ov3642_vfs_snap_settings_tbl_size, &ov3642_use_vfs_snap_setting, 3);
	return count;
}

static ssize_t ov3642_write_snapaereg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov3642_get_param(buf, count, &ov3642_vfs_snapae_settings_tbl[0], ov3642_vfs_snapae_settings_tbl_size, &ov3642_use_vfs_snapae_setting, 3);
	return count;
}

static ssize_t ov3642_write_flashaereg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov3642_get_param(buf, count, &ov3642_vfs_flashae_settings_tbl[0], ov3642_vfs_flashae_settings_tbl_size, &ov3642_use_vfs_flashae_setting, 3);
	return count;
}

static ssize_t ov3642_write_iqreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov3642_get_param(buf, count, &ov3642_vfs_iq_settings_tbl[0], ov3642_vfs_iq_settings_tbl_size, &ov3642_use_vfs_iq_setting, 3);
	return count;
}

static ssize_t ov3642_write_lensreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov3642_get_param(buf, count, &ov3642_vfs_lens_settings_tbl[0], ov3642_vfs_lens_settings_tbl_size, &ov3642_use_vfs_lens_setting, 3);
	return count;
}

static ssize_t ov3642_write_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	long rc = 0;

	ov3642_get_param(buf, count, &ov3642_vfs_writereg_settings_tbl[0], ov3642_vfs_writereg_settings_tbl_size, &ov3642_use_vfs_writereg_setting, 3);
	if (ov3642_use_vfs_writereg_setting)
	{
		rc = ov3642_i2c_write_table(&ov3642_vfs_writereg_settings_tbl[0],
			ov3642_vfs_writereg_settings_tbl_size);
		ov3642_use_vfs_writereg_setting =0;
	}
	return count;
}

static ssize_t ov3642_setrange(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov3642_get_param(buf, count, &ov3642_vfs_getreg_settings_tbl[0], ov3642_vfs_getreg_settings_tbl_size, &ov3642_use_vfs_getreg_setting, 3);
	return count;
}

static ssize_t ov3642_getrange(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i,rc;
	char *str = buf;

	if (ov3642_use_vfs_getreg_setting)
	{
		for (i=0;i<=ov3642_vfs_getreg_settings_tbl_size;i++)
		{
			if (ov3642_vfs_getreg_settings_tbl[i].waddr==0xFFFF)
				break;

			rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
				ov3642_vfs_getreg_settings_tbl[i].waddr, &(ov3642_vfs_getreg_settings_tbl[i].wdata), BYTE_LEN);
			CDBG("ov3642 reg 0x%4X = 0x%2X\n", ov3642_vfs_getreg_settings_tbl[i].waddr, ov3642_vfs_getreg_settings_tbl[i].wdata);

			str += sprintf(str, "%04X,%2X,%2X\n", ov3642_vfs_getreg_settings_tbl[i].waddr, 
				ov3642_vfs_getreg_settings_tbl[i].wdata, 
				ov3642_vfs_getreg_settings_tbl[i].mask);

			if (rc <0)
				break;
		}
	}
	return (str - buf);
}

static ssize_t ov3642_setmclk(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf,"%d",&ov3642_use_vfs_mclk_setting);
	return count;
}

static ssize_t ov3642_getmclk(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (sprintf(buf,"%d\n",ov3642_use_vfs_mclk_setting));
}

static ssize_t ov3642_setmultiple(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf,"%d",&ov3642_use_vfs_multiple_setting);
	return count;
}

static ssize_t ov3642_getmultiple(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (sprintf(buf,"%d\n",ov3642_use_vfs_multiple_setting));
}

static ssize_t ov3642_setflashtime(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf,"%d",&ov3642_use_vfs_flashtime_setting);
	return count;
}

static ssize_t ov3642_getflashtime(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (sprintf(buf,"%d\n",ov3642_use_vfs_flashtime_setting));
}

DEVICE_ATTR(initreg_3642, 0666, NULL, ov3642_write_initreg);
DEVICE_ATTR(oemreg_3642, 0666, NULL, ov3642_write_oemtreg);
DEVICE_ATTR(previewreg_3642, 0666, NULL, ov3642_write_previewreg);
DEVICE_ATTR(snapreg_3642, 0666, NULL, ov3642_write_snapreg);
DEVICE_ATTR(snapregae_3642, 0666, NULL, ov3642_write_snapaereg);
DEVICE_ATTR(flashregae_3642, 0666, NULL, ov3642_write_flashaereg);
DEVICE_ATTR(iqreg_3642, 0666, NULL, ov3642_write_iqreg);
DEVICE_ATTR(lensreg_3642, 0666, NULL, ov3642_write_lensreg);
DEVICE_ATTR(writereg_3642, 0666, NULL, ov3642_write_reg);
DEVICE_ATTR(getreg_3642, 0666, ov3642_getrange, ov3642_setrange);
DEVICE_ATTR(mclk_3642, 0666, ov3642_getmclk, ov3642_setmclk);
DEVICE_ATTR(multiple_3642, 0666, ov3642_getmultiple, ov3642_setmultiple);
DEVICE_ATTR(flashtime_3642, 0666, ov3642_getflashtime, ov3642_setflashtime);

#if 0
static struct kobject *android_ov3642 = NULL;
static int ov3642_sysfs_init(void)
{
	int ret ;
	CAM_USER_G0(KERN_INFO "ov3642 :kobject creat and add\n");
	android_ov3642 = kobject_create_and_add("android_camera", NULL);
	if (android_ov3642 == NULL) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	CAM_USER_G0(KERN_INFO "ov3642:sysfs_create_file\n");

	ret = sysfs_create_file(android_ov3642, &dev_attr_initreg_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	ret = sysfs_create_file(android_ov3642, &dev_attr_oemreg_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	ret = sysfs_create_file(android_ov3642, &dev_attr_previewreg_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	ret = sysfs_create_file(android_ov3642, &dev_attr_snapreg_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	ret = sysfs_create_file(android_ov3642, &dev_attr_snapregae_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	ret = sysfs_create_file(android_ov3642, &dev_attr_iqreg_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	ret = sysfs_create_file(android_ov3642, &dev_attr_lensreg_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	ret = sysfs_create_file(android_ov3642, &dev_attr_writereg_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	ret = sysfs_create_file(android_ov3642, &dev_attr_getreg_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	ret = sysfs_create_file(android_ov3642, &dev_attr_mclk_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	ret = sysfs_create_file(android_ov3642, &dev_attr_multiple_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	ret = sysfs_create_file(android_ov3642, &dev_attr_flashtime_3642.attr);
	if (ret) {
		CAM_USER_G0(KERN_ERR "ov3642_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov3642);
	}

	return 0 ;
}

static int ov3642_sysfs_remove(void)
{
	sysfs_remove_file (android_ov3642, &dev_attr_initreg_3642.attr);
	sysfs_remove_file(android_ov3642, &dev_attr_oemreg_3642.attr);
	sysfs_remove_file(android_ov3642, &dev_attr_previewreg_3642.attr);
	sysfs_remove_file(android_ov3642, &dev_attr_snapreg_3642.attr);
	sysfs_remove_file(android_ov3642, &dev_attr_snapregae_3642.attr);
	sysfs_remove_file(android_ov3642, &dev_attr_iqreg_3642.attr);
	sysfs_remove_file(android_ov3642, &dev_attr_lensreg_3642.attr);
	sysfs_remove_file(android_ov3642, &dev_attr_writereg_3642.attr);
	sysfs_remove_file(android_ov3642, &dev_attr_getreg_3642.attr);
	sysfs_remove_file(android_ov3642, &dev_attr_mclk_3642.attr);
	sysfs_remove_file(android_ov3642, &dev_attr_multiple_3642.attr);
	sysfs_remove_file(android_ov3642, &dev_attr_flashtime_3642.attr);

	return 0 ;
}

#endif

static int create_attributes(struct i2c_client *client)
{
	int rc;

	dev_attr_initreg_3642.attr.name = OV3642_INITREG;
	dev_attr_oemreg_3642.attr.name = OV3642_OEMREG;
	dev_attr_previewreg_3642.attr.name = OV3642_PREVIEWREG;
	dev_attr_snapreg_3642.attr.name = OV3642_SNAPREG;
	dev_attr_snapregae_3642.attr.name = OV3642_SNAPAEREG;
	dev_attr_flashregae_3642.attr.name = OV3642_FLASHAEREG;
	dev_attr_iqreg_3642.attr.name = OV3642_IQREG;
	dev_attr_lensreg_3642.attr.name = OV3642_LENSREG;
	dev_attr_writereg_3642.attr.name = OV3642_WRITEREG;
	dev_attr_getreg_3642.attr.name = OV3642_GETREG;
	dev_attr_mclk_3642.attr.name = OV3642_MCLK;
	dev_attr_multiple_3642.attr.name = OV3642_MULTIPLE;
	dev_attr_flashtime_3642.attr.name = OV3642_FLASHTIME;

#if 0
	rc = ov3642_sysfs_init();
#else
	rc = device_create_file(&client->dev, &dev_attr_initreg_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"initreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}

	rc = device_create_file(&client->dev, &dev_attr_oemreg_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"oemreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}

	rc = device_create_file(&client->dev, &dev_attr_previewreg_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"previewreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}

	rc = device_create_file(&client->dev, &dev_attr_snapreg_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"snapreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_snapregae_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"snapregae\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_flashregae_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"flashregae\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_iqreg_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"iqreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_lensreg_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"lensreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_writereg_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"writereg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_getreg_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"getreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_mclk_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"mclk\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_multiple_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov3642 attribute \"multiple\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_flashtime_3642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create o33642 attribute \"flashtime\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	
#endif

	return rc;
}

static int remove_attributes(struct i2c_client *client)
{
#if 0
	ov3642_sysfs_remove();
	kobject_del(android_ov3642);
#else
	device_remove_file(&client->dev, &dev_attr_initreg_3642);
	device_remove_file(&client->dev, &dev_attr_oemreg_3642);
	device_remove_file(&client->dev, &dev_attr_previewreg_3642);
	device_remove_file(&client->dev, &dev_attr_snapreg_3642);
	device_remove_file(&client->dev, &dev_attr_snapregae_3642);
	device_remove_file(&client->dev, &dev_attr_flashregae_3642);
	device_remove_file(&client->dev, &dev_attr_iqreg_3642);
	device_remove_file(&client->dev, &dev_attr_lensreg_3642);
	device_remove_file(&client->dev, &dev_attr_writereg_3642);
	device_remove_file(&client->dev, &dev_attr_getreg_3642);
	device_remove_file(&client->dev, &dev_attr_mclk_3642);
	device_remove_file(&client->dev, &dev_attr_multiple_3642);
	device_remove_file(&client->dev, &dev_attr_flashtime_3642);
#endif

	return 0;
}
#endif /* OV3642_USE_VFS */
#endif
/* } FIH, Charles Huang, 2009/06/24 */

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
static enum hrtimer_restart ov3642_flashled_timer_func(struct hrtimer *timer)
{
	//static struct mpp *mpp_19;

	//mpp_19 = mpp_get(NULL, "mpp19");
	CAM_USER_G0(KERN_INFO "ov3642_flashled_timer_func ov3642_m_ledmod = %d\n", ov3642_m_ledmod);

	mpp_config_digital_out(18,MPP_CFG(MPP_DLOGIC_LVL_MSMP,
		MPP_DLOGIC_OUT_CTRL_HIGH));

	complete(&ov3642_flashled_comp);
	return HRTIMER_NORESTART;
}
#endif
/* } FIH, Charles Huang, 2009/09/01 */

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
static int ov3642_flashled_off_thread(void *arg)
{
	int ret = 0;
	//static struct mpp *mpp_19;

	CAM_USER_G0(KERN_INFO "%s: ov3642_flashled_off_thread running\n", __func__);

	daemonize("ov3642_flashled_off_thread");

	while (1) {
		wait_for_completion(&ov3642_flashled_comp);
		CAM_USER_G0(KERN_INFO "%s: Got complete signal\n", __func__);
		/* wait for flash on and turn off mpp */
		//msleep(400);
		msleep(380/ov3642_ledtime);
		//mpp_19 = mpp_get(NULL, "mpp19");
		ret=mpp_config_digital_out(18,MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			MPP_DLOGIC_OUT_CTRL_LOW));
		flash_settime(0);
		//brightness_onoff(0);
	}
	
    return 0;
}
#endif
/* } FIH, Charles Huang, 2009/09/01 */

static int ov3642_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	ov3642_sensorw =
		kzalloc(sizeof(struct ov3642_work), GFP_KERNEL);

	if (!ov3642_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov3642_sensorw);
	ov3642_init_client(client);
	ov3642_client = client;
	
	CDBG("ov3642_probe successed!\n");

	return 0;

probe_failure:
	kfree(ov3642_sensorw);
	ov3642_sensorw = NULL;
	CDBG("ov3642_probe failed!\n");
	return rc;
}

static int __exit ov3642_i2c_remove(struct i2c_client *client)
{
	struct ov3642_work *sensorw = i2c_get_clientdata(client);

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
	remove_attributes(client);
#endif /* OV3642_USE_VFS */
#endif
/* } FIH, Charles Huang, 2009/06/24 */

	free_irq(client->irq, sensorw);
	ov3642_client = NULL;
	ov3642_sensorw = NULL;
	kfree(sensorw);
	return 0;
}

#ifdef CONFIG_PM
static int ov3642_suspend(struct i2c_client *client, pm_message_t mesg)
{
/* FIH, Charles Huang, 2009/06/25 { */
/* [FXX_CR], suspend/resume for pm */
#ifdef CONFIG_FIH_FXX
	/* sensor_pwd pin gpio31 */
	CAM_USER_G0(KERN_INFO "ov3642_suspend!\n");
	if (ov3642_ctrl)
	{
		CAM_USER_G0(KERN_INFO "ov3642 power down high!\n");
		gpio_direction_output(31,1);
	}
#endif
/* } FIH, Charles Huang, 2009/06/25 */
	return 0;
}

static int ov3642_resume(struct i2c_client *client)
{
/* FIH, Charles Huang, 2009/06/25 { */
/* [FXX_CR], suspend/resume for pm */
#ifdef CONFIG_FIH_FXX
	/* sensor_pwd pin gpio31 */
	/* Handle by sensor initialization */
	/* workable setting for waste power while resuming */
	CAM_USER_G0(KERN_INFO "ov3642_resume!\n");
	if (ov3642_ctrl)
	{
		CAM_USER_G0(KERN_INFO "ov3642 power down low!\n");
		gpio_direction_output(31,0);
		CAM_USER_G0("[OV3642 3M]  gpio_get_value(31) = %d\n", gpio_get_value(31));
	}
#endif
/* } FIH, Charles Huang, 2009/06/25 */
	return 0;
}
#else
# define ov3642_suspend NULL
# define ov3642_resume  NULL
#endif

static const struct i2c_device_id ov3642_i2c_id[] = {
	{ "ov3642", 0},
	{ },
};

static struct i2c_driver ov3642_i2c_driver = {
	.id_table = ov3642_i2c_id,
	.probe  = ov3642_i2c_probe,
	.remove = __exit_p(ov3642_i2c_remove),
	.suspend  	= ov3642_suspend,
	.resume   	= ov3642_resume,
	.driver = {
		.name = "ov3642",
	},
};

static int ov3642_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = i2c_add_driver(&ov3642_i2c_driver);
	uint16_t model_id = 0;
	int HWID=FIH_READ_HWID_FROM_SMEM();
/* FIH, Charles Huang, 2010/03/01 { */
/* [FXX_CR], Add power onoff vreg */
#ifdef CONFIG_FIH_FXX
	/* VDD 3V F0X:rftx FA3:wlan*/
	struct vreg *vreg_vcm;
	/* DCORE 1.5V F0X:gp2 FA3:msme2*/
	struct vreg *vreg_dcore;
	/* ACORE 2.8V F0X:gp3 FA3:gp1*/
	struct vreg *vreg_acore;
	/* io 2.6V F0X:msmp FA3:gp3*/
	struct vreg *vreg_io;
#endif
/* } FIH, Charles Huang, 2010/03/01 */

	if (rc < 0 || ov3642_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

/* FIH, Charles Huang, 2010/03/01 { */
/* [FXX_CR], Add power onoff vreg */
#ifdef CONFIG_FIH_FXX
	/* turn on power */
	if (FIH_READ_HWID_FROM_SMEM() < CMCS_7627_EVB1)
	{
		vreg_vcm = vreg_get(NULL, "rftx");
		vreg_set_level(vreg_vcm, 3000);
		vreg_enable(vreg_vcm);

		vreg_dcore = vreg_get(NULL, "gp2");
		vreg_set_level(vreg_dcore, 1500);
		vreg_enable(vreg_dcore);

		vreg_acore = vreg_get(NULL, "gp3");
		vreg_set_level(vreg_acore, 2800);
		vreg_enable(vreg_acore);
	}else
	{
		/* vcm use the same vreg as touch, we don't turn off by sensor driver */
		//vreg_vcm = vreg_get(NULL, "wlan");
		//vreg_set_level(vreg_vcm, 3000);
		//vreg_enable(vreg_vcm);

		vreg_dcore = vreg_get(NULL, "msme2");
		vreg_set_level(vreg_dcore, 1500);
		vreg_enable(vreg_dcore);

		vreg_acore = vreg_get(NULL, "gp1");
		vreg_set_level(vreg_acore, 2800);
		vreg_enable(vreg_acore);

		vreg_io = vreg_get(NULL, "gp3");
		vreg_set_level(vreg_io, 2600);
		vreg_enable(vreg_io);
	}
#endif
/* } FIH, Charles Huang, 2010/03/01 */

	rc = ov3642_reset(info);
	if (rc < 0) {
		CDBG("reset failed!\n");
		goto probe_fail;
	}

	/* EVB CAMIF cannont handle in 24MHz */
	/* EVB use 12.8MHz */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
	if (ov3642_use_vfs_mclk_setting!=0)
		msm_camio_clk_rate_set(ov3642_use_vfs_mclk_setting);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
	if (HWID==CMCS_HW_VER_EVB1)
		msm_camio_clk_rate_set(12288000);
	else
		msm_camio_clk_rate_set(24000000);
	msleep(25);

	/* OV suggested Power up block End */
	/* Read the Model ID of the sensor */
	/* Read REG_OV3642_MODEL_ID_HI & REG_OV3642_MODEL_ID_LO */
	rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
		REG_OV3642_MODEL_ID_HI, &model_id, WORD_LEN);
	if (rc < 0)
		goto probe_fail;

	CDBG("ov3642 model_id = 0x%x\n", model_id);

	/* Check if it matches it with the value in Datasheet */
	if (model_id != OV3642_MODEL_ID) {
		rc = -EFAULT;
		goto probe_fail;
	}

	/* Get version */
	rc = ov3642_i2c_read(OV3642_I2C_READ_SLAVE_ID,
	0x302A, &model_id, BYTE_LEN);
	CDBG("ov3642 version reg 0x302A = 0x%x\n", model_id);

	rc = gpio_request(info->sensor_reset, "ov3642");
	if (!rc) {
		rc = gpio_direction_output(info->sensor_reset, 0);
	}
	gpio_free(info->sensor_reset);

	rc = gpio_request(info->sensor_pwd, "ov3642");
	if (!rc) {
		rc = gpio_direction_output(info->sensor_pwd, 1);
	}
	gpio_free(info->sensor_pwd);

/* FIH, Charles Huang, 2009/06/09 { */
/* [FXX_CR], pull pwdn pin high */
#ifdef CONFIG_FIH_FXX
	if (HWID>=CMCS_7627_EVB1)
	{
		/* Switch disable */
		gpio_tlmm_config(GPIO_CFG(121, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		gpio_direction_output(121,1);
	}
#endif
/* } FIH, Charles Huang, 2009/06/09 */
       CDBG("[OV3642 3M]  gpio_get_value(0) = %d\n", gpio_get_value(0));
       CDBG("[OV3642 3M]  gpio_get_value(17) = %d\n", gpio_get_value(17));
       CDBG("[OV3642 3M]  gpio_get_value(31) = %d\n", gpio_get_value(31));
       CDBG("[OV3642 3M]  gpio_get_value(121) = %d\n", gpio_get_value(121));

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV3642_USE_VFS
	rc = create_attributes(ov3642_client);
	if (rc < 0) {
		dev_err(&ov3642_client->dev, "%s: create attributes failed!! <%d>", __func__, rc);
		goto probe_done;
	}
#endif /* OV3642_USE_VFS */
#endif
/* } FIH, Charles Huang, 2009/06/24 */

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
	hrtimer_init(&ov3642_flashled_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ov3642_flashled_timer.function = ov3642_flashled_timer_func;
	ov3642_thread_id = kernel_thread(ov3642_flashled_off_thread, NULL, CLONE_FS | CLONE_FILES);
#endif
/* } FIH, Charles Huang, 2009/09/01 */

	s->s_init = ov3642_sensor_init;
	s->s_release = ov3642_sensor_release;
	s->s_config  = ov3642_sensor_config;

probe_done:
/* FIH, Charles Huang, 2010/03/01 { */
/* [FXX_CR], Add power onoff vreg */
#ifdef CONFIG_FIH_FXX
	/* turn on power */
	if (FIH_READ_HWID_FROM_SMEM() < CMCS_7627_EVB1)
	{
		vreg_vcm = vreg_get(NULL, "rftx");
		vreg_disable(vreg_vcm);

		vreg_dcore = vreg_get(NULL, "gp2");
		vreg_disable(vreg_dcore);

		vreg_acore = vreg_get(NULL, "gp3");
		vreg_disable(vreg_acore);
	}else
	{
		/* vcm use the same vreg as touch, we don't turn off by sensor driver */
		//vreg_vcm = vreg_get(NULL, "wlan");
		//vreg_disable(vreg_vcm);

		vreg_dcore = vreg_get(NULL, "msme2");
		vreg_disable(vreg_dcore);

		vreg_acore = vreg_get(NULL, "gp1");
		vreg_disable(vreg_acore);

		vreg_io = vreg_get(NULL, "gp3");
		vreg_disable(vreg_io);
	}
#endif
/* } FIH, Charles Huang, 2010/03/01 */

	dev_info(&ov3642_client->dev, "probe_done %s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;

probe_fail:
	dev_info(&ov3642_client->dev, "probe_fail %s %s:%d\n", __FILE__, __func__, __LINE__);
	gpio_tlmm_config(GPIO_CFG(0, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
	gpio_direction_output(0,0);
	gpio_tlmm_config(GPIO_CFG(31, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
	gpio_direction_output(31,1);

	if (HWID>=CMCS_7627_EVB1)
	{
		/* Switch disable */
		gpio_tlmm_config(GPIO_CFG(121, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		gpio_direction_output(121,1);
	}
       CDBG("[OV3642 3M]  gpio_get_value(0) = %d\n", gpio_get_value(0));
       CDBG("[OV3642 3M]  gpio_get_value(17) = %d\n", gpio_get_value(17));
       CDBG("[OV3642 3M]  gpio_get_value(31) = %d\n", gpio_get_value(31));
       CDBG("[OV3642 3M]  gpio_get_value(121) = %d\n", gpio_get_value(121));

/* FIH, Charles Huang, 2010/03/01 { */
/* [FXX_CR], Add power onoff vreg */
#ifdef CONFIG_FIH_FXX
	/* turn on power */
	if (FIH_READ_HWID_FROM_SMEM() < CMCS_7627_EVB1)
	{
		vreg_vcm = vreg_get(NULL, "rftx");
		vreg_disable(vreg_vcm);

		vreg_dcore = vreg_get(NULL, "gp2");
		vreg_disable(vreg_dcore);

		vreg_acore = vreg_get(NULL, "gp3");
		vreg_disable(vreg_acore);
	}else
	{
		/* vcm use the same vreg as touch, we don't turn off by sensor driver */
		//vreg_vcm = vreg_get(NULL, "wlan");
		//vreg_disable(vreg_vcm);

		vreg_dcore = vreg_get(NULL, "msme2");
		vreg_disable(vreg_dcore);

		vreg_acore = vreg_get(NULL, "gp1");
		vreg_disable(vreg_acore);

		vreg_io = vreg_get(NULL, "gp3");
		vreg_disable(vreg_io);
	}
#endif
/* } FIH, Charles Huang, 2010/03/01 */

	return rc;
}

static int __ov3642_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, ov3642_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov3642_probe,
	.driver = {
		.name = "msm_camera_ov3642",
		.owner = THIS_MODULE,
	},
};

static int __init ov3642_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov3642_init);
