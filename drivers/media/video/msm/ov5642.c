/*
 *     ov5642.c - Camera Sensor Driver
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
#include <linux/slab.h>
#include "ov5642.h"

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
int ov5642_m_ledmod=0;
static pid_t ov5642_thread_id;
struct hrtimer ov5642_flashled_timer;
int ov5642_ledtime=16;
#define OV5642_FLASHLED_DELAY 190
DECLARE_COMPLETION(ov5642_flashled_comp);
#endif
/* } FIH, Charles Huang, 2009/09/01 */

DEFINE_MUTEX(ov5642_mut);

/* Micron OV5642 Registers and their values */
/* Sensor Core Registers */
#define  REG_OV5642_MODEL_ID_HI 0x300A
#define  REG_OV5642_MODEL_ID_LO 0x300B
#define  OV5642_MODEL_ID     0x5642
#define  OV5642_I2C_READ_SLAVE_ID     0x78 >> 1  
#define  OV5642_I2C_WRITE_SLAVE_ID     0x79 >> 1  
/* FIH, Charles Huang, 2009/07/29 { */
/* [FXX_CR], Calculate AEC */
#ifdef CONFIG_FIH_FXX
#define OV5642_CAPTURE_FRAMERATE 7.5
#define OV5642_PREVIEW_FRAMERATE 27
int ov5642_m_60Hz=0;
#endif
/* } FIH, Charles Huang, 2009/07/29 */

struct ov5642_work {
	struct work_struct work;
};

static struct  ov5642_work *ov5642_sensorw;
static struct  i2c_client *ov5642_client;

struct ov5642_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};

extern void brightness_onoff(int on);
extern void flash_settime(int time);
static struct ov5642_ctrl *ov5642_ctrl;

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#define OV5642_USE_VFS
#ifdef OV5642_USE_VFS
#define OV5642_INITREG "initreg"
#define OV5642_OEMREG "oemreg"
#define OV5642_PREVIEWREG "previewreg"
#define OV5642_SNAPREG "snapreg"
#define OV5642_SNAPAEREG "snapaereg"
#define OV5642_FLASHAEREG "flashaereg"
#define OV5642_IQREG "iqreg"
#define OV5642_LENSREG "lensreg"
#define OV5642_WRITEREG "writereg"
#define OV5642_GETREG "getreg"
#define OV5642_MCLK "mclk"
#define OV5642_MULTIPLE "multiple"
#define OV5642_FLASHTIME "flashtime"

/* MAX buf is ???? */
#define OV5642_MAX_VFS_INIT_INDEX 330
int ov5642_use_vfs_init_setting=0;
struct ov5642_i2c_reg_conf ov5642_vfs_init_settings_tbl[OV5642_MAX_VFS_INIT_INDEX];
uint16_t ov5642_vfs_init_settings_tbl_size= ARRAY_SIZE(ov5642_vfs_init_settings_tbl);

#define OV5642_MAX_VFS_OEM_INDEX 330
int ov5642_use_vfs_oem_setting=0;
struct ov5642_i2c_reg_conf ov5642_vfs_oem_settings_tbl[OV5642_MAX_VFS_OEM_INDEX];
uint16_t ov5642_vfs_oem_settings_tbl_size= ARRAY_SIZE(ov5642_vfs_oem_settings_tbl);

#define OV5642_MAX_VFS_PREVIEW_INDEX 330
int ov5642_use_vfs_preview_setting=0;
struct ov5642_i2c_reg_conf ov5642_vfs_preview_settings_tbl[OV5642_MAX_VFS_PREVIEW_INDEX];
uint16_t ov5642_vfs_preview_settings_tbl_size= ARRAY_SIZE(ov5642_vfs_preview_settings_tbl);

#define OV5642_MAX_VFS_SNAP_INDEX 330
int ov5642_use_vfs_snap_setting=0;
struct ov5642_i2c_reg_conf ov5642_vfs_snap_settings_tbl[OV5642_MAX_VFS_SNAP_INDEX];
uint16_t ov5642_vfs_snap_settings_tbl_size= ARRAY_SIZE(ov5642_vfs_snap_settings_tbl);

#define OV5642_MAX_VFS_SNAPAE_INDEX 330
int ov5642_use_vfs_snapae_setting=0;
struct ov5642_i2c_reg_conf ov5642_vfs_snapae_settings_tbl[OV5642_MAX_VFS_SNAPAE_INDEX];
uint16_t ov5642_vfs_snapae_settings_tbl_size= ARRAY_SIZE(ov5642_vfs_snapae_settings_tbl);

#define OV5642_MAX_VFS_FLASHAE_INDEX 330
int ov5642_use_vfs_flashae_setting=0;
struct ov5642_i2c_reg_conf ov5642_vfs_flashae_settings_tbl[OV5642_MAX_VFS_FLASHAE_INDEX];
uint16_t ov5642_vfs_flashae_settings_tbl_size= ARRAY_SIZE(ov5642_vfs_flashae_settings_tbl);

#define OV5642_MAX_VFS_IQ_INDEX 330
int ov5642_use_vfs_iq_setting=0;
struct ov5642_i2c_reg_conf ov5642_vfs_iq_settings_tbl[OV5642_MAX_VFS_IQ_INDEX];
uint16_t ov5642_vfs_iq_settings_tbl_size= ARRAY_SIZE(ov5642_vfs_iq_settings_tbl);

#define OV5642_MAX_VFS_LENS_INDEX 330
int ov5642_use_vfs_lens_setting=0;
struct ov5642_i2c_reg_conf ov5642_vfs_lens_settings_tbl[OV5642_MAX_VFS_LENS_INDEX];
uint16_t ov5642_vfs_lens_settings_tbl_size= ARRAY_SIZE(ov5642_vfs_lens_settings_tbl);

#define OV5642_MAX_VFS_WRITEREG_INDEX 330
int ov5642_use_vfs_writereg_setting=0;
struct ov5642_i2c_reg_conf ov5642_vfs_writereg_settings_tbl[OV5642_MAX_VFS_IQ_INDEX];
uint16_t ov5642_vfs_writereg_settings_tbl_size= ARRAY_SIZE(ov5642_vfs_writereg_settings_tbl);

#define OV5642_MAX_VFS_GETREG_INDEX 330
int ov5642_use_vfs_getreg_setting=0;
struct ov5642_i2c_reg_conf ov5642_vfs_getreg_settings_tbl[OV5642_MAX_VFS_GETREG_INDEX];
uint16_t ov5642_vfs_getreg_settings_tbl_size= ARRAY_SIZE(ov5642_vfs_getreg_settings_tbl);

int ov5642_use_vfs_mclk_setting=0;
int ov5642_use_vfs_multiple_setting=0;
int ov5642_use_vfs_flashtime_setting=0;
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */

static DECLARE_WAIT_QUEUE_HEAD(ov5642_wait_queue);
DECLARE_MUTEX(ov5642_sem);

/*=============================================================*/
static int ov5642_reset(const struct msm_camera_sensor_info *dev)
{
	int rc = 0;
	int HWID=FIH_READ_HWID_FROM_SMEM();

       gpio_tlmm_config(GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
       	gpio_tlmm_config(GPIO_CFG(17, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
      	gpio_tlmm_config(GPIO_CFG(121, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

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

	rc = gpio_request(dev->sensor_pwd, "ov5642");
	if (!rc) {
		rc = gpio_direction_output(dev->sensor_pwd, 0);
	}
	gpio_free(dev->sensor_pwd);
	
	rc = gpio_request(dev->sensor_reset, "ov5642");
	if (!rc) {
		rc = gpio_direction_output(dev->sensor_reset, 0);
		msleep(20);
		rc = gpio_direction_output(dev->sensor_reset, 1);
	}
	gpio_free(dev->sensor_reset);
	
       CDBG("[OV5642 5M] gpio_get_value(0) = %d\n", gpio_get_value(0));
       CDBG("[OV5642 5M] gpio_get_value(17) = %d\n", gpio_get_value(17));
       CDBG("[OV5642 5M] gpio_get_value(31) = %d\n", gpio_get_value(31));
       CDBG("[OV5642 5M] gpio_get_value(121) = %d\n", gpio_get_value(121));

	return rc;
}

static int32_t ov5642_i2c_txdata(unsigned short saddr,
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

	if (i2c_transfer(ov5642_client->adapter, msg, 1) < 0) {
		if (i2c_transfer(ov5642_client->adapter, msg, 1) < 0) {
			CDBG("ov5642_i2c_txdata failed\n");
			return -EIO;
		}
	}

	return 0;
}

static int32_t ov5642_i2c_write(unsigned short saddr,
	unsigned short waddr, unsigned short wdata, enum ov5642_width width)
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

		rc = ov5642_i2c_txdata(saddr, buf, 4);
	}
		break;

	case BYTE_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (uint8_t) wdata;
		rc = ov5642_i2c_txdata(saddr, buf, 3);
	}
		break;

	default:
		break;
	}

	if (rc < 0)
		printk(
		KERN_ERR "ov5642 i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int32_t ov5642_i2c_rxdata(unsigned short saddr,
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

	if (i2c_transfer(ov5642_client->adapter, msgs, 2) < 0) {
		if (i2c_transfer(ov5642_client->adapter, msgs, 2) < 0) {
			CDBG("ov5642_i2c_rxdata failed!\n");
			return -EIO;
		}
	}

	return 0;
}

static int32_t ov5642_i2c_read(unsigned short   saddr,
	unsigned short raddr, unsigned short *rdata, enum ov5642_width width)
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

		rc = ov5642_i2c_rxdata(saddr, buf, 2);
		if (rc < 0)
			return rc;

		*rdata = buf[0] << 8 | buf[1];
	}
		break;
	case BYTE_LEN: {
		buf[0] = (raddr & 0xFF00)>>8;
		buf[1] = (raddr & 0x00FF);

		rc = ov5642_i2c_rxdata(saddr, buf, 1);
		if (rc < 0)
			return rc;

		*rdata = buf[0];
	}
		break;

	default:
		break;
	}

	if (rc < 0)
		printk(KERN_ERR "ov5642_i2c_read failed!\n");

	return rc;
}

void ov5642_set_value_by_bitmask(uint16_t bitset, uint16_t mask, uint16_t  *new_value)
{
	uint16_t org;

	org= *new_value;
	*new_value = (org&~mask) | (bitset & mask);
}

static int32_t ov5642_i2c_write_table(
	struct ov5642_i2c_reg_conf const *reg_conf_tbl,
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
			rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				reg_conf_tbl->waddr, reg_conf_tbl->wdata,
				reg_conf_tbl->width);
		}else{
			uint16_t reg_value = 0;
			rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
				reg_conf_tbl->waddr, &reg_value, BYTE_LEN);
			ov5642_set_value_by_bitmask(reg_conf_tbl->wdata,reg_conf_tbl->mask,&reg_value);
			rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
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
static int32_t ov5642_set_lens_roll_off(void)
{
	int32_t rc = 0;
	rc = ov5642_i2c_write_table(&ov5642_regs.rftbl[0],
		ov5642_regs.rftbl_size);
	return rc;
}
#endif

static long ov5642_reg_init(void)
{
#if 0
	int32_t i;
#endif
	long rc;
	uint16_t lens_id = 0;
#if 0
	/* PLL Setup Start */
	rc = ov5642_i2c_write_table(&mt9d112_regs.plltbl[0],
		ov5642_regs.plltbl_size);

	if (rc < 0)
		return rc;
	/* PLL Setup End   */
#endif

	/* Configure sensor for Preview mode and Snapshot mode */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
	if (ov5642_use_vfs_init_setting)
		rc = ov5642_i2c_write_table(&ov5642_vfs_init_settings_tbl[0],
			ov5642_vfs_init_settings_tbl_size);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
		rc = ov5642_i2c_write_table(&ov5642_regs.init_settings_tbl[0],
			ov5642_regs.init_settings_tbl_size);

	if (rc < 0)
		return rc;

	/* Configure sensor for image quality settings */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
	if (ov5642_use_vfs_iq_setting)
		rc = ov5642_i2c_write_table(&ov5642_vfs_iq_settings_tbl[0],
			ov5642_vfs_iq_settings_tbl_size);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
	rc = ov5642_i2c_write_table(&ov5642_regs.iq_settings_tbl[0],
		ov5642_regs.iq_settings_tbl_size);

	if (rc < 0)
		return rc;

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
	if (ov5642_use_vfs_lens_setting)
		rc = ov5642_i2c_write_table(&ov5642_vfs_lens_settings_tbl[0],
			ov5642_vfs_lens_settings_tbl_size);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
	{
		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x3D06, &lens_id, BYTE_LEN);

		if (rc < 0)
			return rc;

		if (lens_id==0x47)
			rc = ov5642_i2c_write_table(&ov5642_regs.gseolens_settings_tbl[0],
				ov5642_regs.gseolens_settings_tbl_size);
		else if (lens_id==0x46)
			rc = ov5642_i2c_write_table(&ov5642_regs.grdlens_settings_tbl[0],
				ov5642_regs.grdlens_settings_tbl_size);
		else
			rc = ov5642_i2c_write_table(&ov5642_regs.gseolens_settings_tbl[0],
				ov5642_regs.gseolens_settings_tbl_size);
	}
	if (rc < 0)
		return rc;

	/* Configure sensor for customer settings */
#if 0
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
	if (ov5642_use_vfs_oem_setting)
		rc = ov5642_i2c_write_table(&vfs_oem_settings_tbl[0],
			vfs_oem_settings_tbl_size);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
	rc = ov5642_i2c_write_table(&ov5642_regs.oem_settings_tbl[0],
		ov5642_regs.oem_settings_tbl_size);

	if (rc < 0)
		return rc;
#endif
#if 0
	/* Configure for Noise Reduction, Saturation and Aperture Correction */
	array_length = ov5642_regs.noise_reduction_reg_settings_;

	for (i = 0; i < array_length; i++) {

		rc = ov5642_i2c_write(ov5642_client->addr,
		ov5642_regs.noise_reduction_reg_settings[i].register_address,
		ov5642_regs.noise_reduction_reg_settings[i].register_value,
			WORD_LEN);

		if (rc < 0)
			return rc;
	}

	/* Set Color Kill Saturation point to optimum value */
	rc =
	ov5642_i2c_write(ov5642_client->addr,
	0x35A4,
	0x0593,
	WORD_LEN);
	if (rc < 0)
		return rc;

	rc = ov5642_i2c_write_table(&ov5642_regs.stbl[0],
		ov5642_regs.stbl_size);
	if (rc < 0)
		return rc;

	rc = ov5642_set_lens_roll_off();
	if (rc < 0)
		return rc;
#endif

	return 0;
}

static long ov5642_set_sensor_mode(int mode)
{
	long rc = 0;
/* FIH, Charles Huang, 2009/07/29 { */
/* [FXX_CR], Calculate AEC */
#ifdef CONFIG_FIH_FXX
	unsigned short   R0x350b,R0x3502,R0x3501,R0x3500;
	unsigned short   Rcap0x350c,Rcap0x350d,Rpre0x350c,Rpre0x350d,R0x5690,R0x350bpreview,R0x5690preview;	
	int	Capture_Framerate=OV5642_CAPTURE_FRAMERATE;
	int	Lines_10ms;
	int	Capture_MaxLines;
	int	Preview_FrameRate=OV5642_PREVIEW_FRAMERATE;
	int	Preview_Maxlines;
	unsigned long	ulCapture_Exposure;
	unsigned long	ulCapture_Exposure_Gain;
	unsigned long	ulPreviewExposure;
	unsigned long	iCapture_Gain;
	unsigned long ulCapture_Exposure_end;
	unsigned long iCapture_Gain_end;

	u_int8_t Gain;
	u_int8_t ExposureLow;
	u_int8_t ExposureMid;
	u_int8_t ExposureHigh;
	u_int8_t PreviewMaxlineHigh;
	u_int8_t PreviewMaxlineLow;
	u_int8_t CaptureMaxlineHigh;
	u_int8_t CaptureMaxlineLow;

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
#ifdef OV5642_USE_VFS
		if (ov5642_use_vfs_preview_setting)
			rc = ov5642_i2c_write_table(&ov5642_vfs_preview_settings_tbl[0],
				ov5642_vfs_preview_settings_tbl_size);
		else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
			rc = ov5642_i2c_write_table(&ov5642_regs.preview_settings_tbl[0],
				ov5642_regs.preview_settings_tbl_size);

		if (rc < 0)
			return rc;

	/* Configure sensor for customer settings */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
		if (ov5642_use_vfs_oem_setting)
			rc = ov5642_i2c_write_table(&ov5642_vfs_oem_settings_tbl[0],
				ov5642_vfs_oem_settings_tbl_size);
		else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
			rc = ov5642_i2c_write_table(&ov5642_regs.oem_settings_tbl[0],
				ov5642_regs.oem_settings_tbl_size);

		if (rc < 0)
			return rc;

/* FIH, Charles Huang, 2009/07/28 { */
/* [FXX_CR], double AEC */
#ifdef CONFIG_FIH_FXX
		/* Turn on 3A */
		rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				0x3503, 0x00, BYTE_LEN);

		if (rc < 0)
			return rc;
#endif
/* } FIH, Charles Huang, 2009/07/28 */

		msleep(5);
		break;

	case SENSOR_SNAPSHOT_MODE:
/* FIH, Charles Huang, 2009/07/29 { */
/* [FXX_CR], Calculate AEC */
#ifdef CONFIG_FIH_FXX
	{

		/* read the registers value to the BYTE 0x350b  parameters. */
		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x350b, &R0x350bpreview, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		/* read the registers value to the BYTE 0x5690  parameters. */
		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x5690, &R0x5690preview, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		/*stop AEC/AGC here :write_i2c(0x3503 ,0x07) */
		/* Turn off 3A */
		rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				0x3503, 0x07, BYTE_LEN);
		/* MUST delay 30ms according to datasheet or sensor will crash */
		msleep(30);
		
/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
		if ((ov5642_m_ledmod==2) ||((ov5642_m_ledmod==1)&&(R0x5690preview <= 0x30)))
		{
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
			if (ov5642_use_vfs_flashae_setting!=0 && ov5642_vfs_flashae_settings_tbl[0].wdata != 0xFFFF)
			{
				m_iFlashWrite0x350b=ov5642_vfs_flashae_settings_tbl[3].wdata;
				m_iFlashWrite0x3502=ov5642_vfs_flashae_settings_tbl[2].wdata;
				m_iFlashWrite0x3501=ov5642_vfs_flashae_settings_tbl[1].wdata;
				m_iFlashWrite0x3500=ov5642_vfs_flashae_settings_tbl[0].wdata;
			}else
			{
				m_iFlashWrite0x350b=0x0;
				m_iFlashWrite0x3502=0x0;
				m_iFlashWrite0x3501=0x30;
				m_iFlashWrite0x3500=0x0;
			}
				
			rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
					0x350b, m_iFlashWrite0x350b, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
					0x3502, m_iFlashWrite0x3502, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
					0x3501, m_iFlashWrite0x3501, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
					0x3500, m_iFlashWrite0x3500, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
					0x350b, &m_iFlashWrite0x350b, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
					0x3502, &m_iFlashWrite0x3502, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
					0x3501, &m_iFlashWrite0x3501, BYTE_LEN);

			if (rc < 0)
				return rc;

			rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
					0x3500, &m_iFlashWrite0x3500, BYTE_LEN);

			if (rc < 0)
				return rc;

			msleep(100);
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */

#if 1 /* Use flash or torch mode */
			flash_settime(16);
			ov5642_ledtime=1;
			hrtimer_cancel(&ov5642_flashled_timer);
			hrtimer_start(&ov5642_flashled_timer,
				ktime_set(0, 0),
				HRTIMER_MODE_REL);
			if (hrtimer_active(&ov5642_flashled_timer))
				printk(KERN_INFO "%s: TIMER running\n", __func__);
#else
			brightness_onoff(1);
#endif
		}
#endif
/* } FIH, Charles Huang, 2009/09/01 */

		/* delay for getting stable R0x5690 */
		msleep(85);

		/* read the registers value to the BYTE 0x5690  parameters. */
		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x350b, &R0x350b, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x5690, &R0x5690, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		/*read the registers value to the BYTE 0x350*  parameters.*/
		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x350b, &R0x350b, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x3502, &R0x3502, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x3501, &R0x3501, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x3500, &R0x3500, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x350c, &Rpre0x350c, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x350d, &Rpre0x350d, BYTE_LEN);
		msleep(1);

		Gain=R0x350b;
		ExposureLow=R0x3502;
		ExposureMid=R0x3501;
		ExposureHigh=R0x3500;
		PreviewMaxlineHigh=Rpre0x350c;
		PreviewMaxlineLow=Rpre0x350d;

		/*change resolution from VGA to QXSGA here*/
		/* Configure sensor for Snapshot mode */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
		if (ov5642_use_vfs_snap_setting)
			rc = ov5642_i2c_write_table(&ov5642_vfs_snap_settings_tbl[0],
				ov5642_vfs_snap_settings_tbl_size);
		else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
			rc = ov5642_i2c_write_table(&ov5642_regs.snapshot_settings_tbl[0],
				ov5642_regs.snapshot_settings_tbl_size);

		if (rc < 0)
			return rc;

		msleep(5);

		/*read the registers value to the BYTE 0x350c and 0x350d  parameters.*/
		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x350c, &Rcap0x350c, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
			0x350d, &Rcap0x350d, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		CaptureMaxlineHigh=Rcap0x350c;
		CaptureMaxlineLow=Rcap0x350d;


		Preview_Maxlines = 256*PreviewMaxlineHigh + PreviewMaxlineLow;
		Capture_MaxLines = 256*CaptureMaxlineHigh + CaptureMaxlineLow;

		if(ov5642_m_60Hz== true)
		{
			Lines_10ms = 100 * Capture_Framerate * Capture_MaxLines/12000;
		}
		else
		{
			Lines_10ms = 100 * Capture_Framerate * Capture_MaxLines/10000;
		}

		ulPreviewExposure  = ((unsigned long)ExposureHigh)<<12 ;
		ulPreviewExposure += ((unsigned long)ExposureMid)<<4 ;
		ulPreviewExposure += ((unsigned long) ExposureLow) >>4;

		 if(0 == Preview_Maxlines ||0 ==Preview_FrameRate ||0== Lines_10ms)
		{
			return rc;
		}

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
	if (ov5642_use_vfs_multiple_setting!=0)
		ulCapture_Exposure = 
			ov5642_use_vfs_multiple_setting*(ulPreviewExposure*(Capture_Framerate)*(Capture_MaxLines))/(((Preview_Maxlines)*(Preview_FrameRate)));
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
		/* Correct ov wrong setting to multiple by 4 */
		/* Night mode Preview_Maxlines > 1000 */
		if (Preview_Maxlines > 1000)
			ulCapture_Exposure = 
				8*(ulPreviewExposure*(Capture_Framerate)*(Capture_MaxLines))/(((Preview_Maxlines)*(Preview_FrameRate)));
		else
			ulCapture_Exposure = 
				4*(ulPreviewExposure*(Capture_Framerate)*(Capture_MaxLines))/(((Preview_Maxlines)*(Preview_FrameRate)));

		iCapture_Gain = (Gain & 0x0f) + 16;

		if (Gain & 0x10)
		{
			iCapture_Gain = iCapture_Gain << 1;
		}
		if (Gain & 0x20)
		{
			iCapture_Gain = iCapture_Gain << 1;
		}
		if (Gain & 0x40)
		{
			iCapture_Gain = iCapture_Gain << 1;
		}
		if (Gain & 0x80)
		{
			iCapture_Gain = iCapture_Gain << 1;
		}

		ulCapture_Exposure_Gain = ulCapture_Exposure * iCapture_Gain;

		if(ulCapture_Exposure_Gain < ((unsigned long)(Capture_MaxLines)*16))
		{
			ulCapture_Exposure = ulCapture_Exposure_Gain/16;
			if (ulCapture_Exposure > Lines_10ms)
			{
				ulCapture_Exposure /= Lines_10ms;
				ulCapture_Exposure *= Lines_10ms;
			}
		}
		else
		{
			ulCapture_Exposure = Capture_MaxLines;
		}

		if(ulCapture_Exposure == 0)
		{
			ulCapture_Exposure = 1;
		}

		iCapture_Gain = (ulCapture_Exposure_Gain*2/ulCapture_Exposure + 1)/2;

		ExposureLow = (u_int8_t)(ulCapture_Exposure) <<4;
		ExposureMid = (u_int8_t)(ulCapture_Exposure >> 4) & 0xff;
		ExposureHigh = (u_int8_t)(ulCapture_Exposure >> 12) & 0xff;

		ulCapture_Exposure_end=ulCapture_Exposure;
		iCapture_Gain_end=iCapture_Gain;

		Gain = 0;
		if (iCapture_Gain > 31)
		{
			Gain |= 0x10;
			iCapture_Gain = iCapture_Gain >> 1;
		}
		if (iCapture_Gain > 31)
		{
			Gain |= 0x20;
			iCapture_Gain = iCapture_Gain >> 1;
		}
		if (iCapture_Gain > 31)
		{
			Gain |= 0x40;
			iCapture_Gain = iCapture_Gain >> 1;
		}
		if (iCapture_Gain > 31)
		{
			Gain |= 0x80;
			iCapture_Gain = iCapture_Gain >> 1;
		}
		if (iCapture_Gain > 16)
		{
			Gain |= ((iCapture_Gain -16) & 0x0f);
		}
		if(Gain==0x10)
		{
			Gain=0x11;
		}

		/* write the gain and exposure to 0x350* registers by using ov transfer form*/
		m_iWrite0x350b=Gain;
		m_iWrite0x3502=ExposureLow;
		m_iWrite0x3501=ExposureMid;
		m_iWrite0x3500=ExposureHigh;

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
		/* flash auto or on and  R0x5690preview <= 0x67*/
		if ((ov5642_m_ledmod==2)
			|| ((ov5642_m_ledmod==1)&&(R0x5690preview <= 0x30)))
		{
			/* write the gain and exposure to 0x350* registers by fih flash light table when using flash light */
			m_iWrite0x3500=0x0;
			if (R0x5690 < 0x19)
			{
				m_iWrite0x350b=0x3A;
				m_iWrite0x3501=0xD0;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x19 && R0x5690 < 0x1D)
			{
				m_iWrite0x350b=0x3A;
				m_iWrite0x3501=0x60;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x1D && R0x5690 < 0x23)
			{
				m_iWrite0x350b=0x30;
				m_iWrite0x3501=0xC0;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x23 && R0x5690 < 0x2A)
			{
				m_iWrite0x350b=0x2F;
				m_iWrite0x3501=0xD0;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x2A && R0x5690 < 0x32)
			{
				m_iWrite0x350b=0x12;
				m_iWrite0x3501=0xC0;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x32 && R0x5690 < 0x3A)
			{
				m_iWrite0x350b=0x7;
				m_iWrite0x3501=0xD0;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x3A && R0x5690 < 0x42)
			{
				m_iWrite0x350b=0x7;
				m_iWrite0x3501=0xD0;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x42 && R0x5690 < 0x4B)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x84;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x4B && R0x5690 < 0x55)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x72;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x55 && R0x5690 < 0x5F)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x62;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x5F && R0x5690 < 0x69)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x54;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x69 && R0x5690 < 0x73)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x48;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x73 && R0x5690 < 0x7D)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x3E;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x7D && R0x5690 < 0x87)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x36;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x89 && R0x5690 < 0x91)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x2E;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x91 && R0x5690 < 0x9B)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x28;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0x9B && R0x5690 < 0xA5)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x22;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0xA5 && R0x5690 < 0xAF)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x1D;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0xAF && R0x5690 < 0xB9)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x19;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0xB9 && R0x5690 < 0xC3)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x15;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0xC3 && R0x5690 < 0xCD)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x12;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0xCD && R0x5690 < 0xD7)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x10;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0xD7 && R0x5690 < 0xE1)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0xE;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0xE1 && R0x5690 < 0xEB)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0xC;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0xEB && R0x5690 < 0xF5)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0xA;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0xF5 && R0x5690 < 0xFC)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x9;
				m_iWrite0x3502=0x0;
			}else if (R0x5690 >= 0xFC && R0x5690 <= 0xFF)
			{
				m_iWrite0x350b=0x0;
				m_iWrite0x3501=0x8;
				m_iWrite0x3502=0x0;
			}
		}
#endif
/* } FIH, Charles Huang, 2009/09/01 */



/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
	if (ov5642_use_vfs_snapae_setting!=0 && ov5642_vfs_snapae_settings_tbl[0].wdata != 0xFFFF)
	{
		m_iWrite0x350b=ov5642_vfs_snapae_settings_tbl[3].wdata;
		m_iWrite0x3502=ov5642_vfs_snapae_settings_tbl[2].wdata;
		m_iWrite0x3501=ov5642_vfs_snapae_settings_tbl[1].wdata;
		m_iWrite0x3500=ov5642_vfs_snapae_settings_tbl[0].wdata;
	}
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
		
		rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				0x350b, m_iWrite0x350b, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				0x3502, m_iWrite0x3502, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				0x3501, m_iWrite0x3501, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				0x3500, m_iWrite0x3500, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;


		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
				0x350b, &m_iWrite0x350b, BYTE_LEN);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
				0x3502, &m_iWrite0x3502, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
				0x3501, &m_iWrite0x3501, BYTE_LEN);

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
				0x3500, &m_iWrite0x3500, BYTE_LEN);
		msleep(1);

		if (rc < 0)
			return rc;

		/* Get third frame */
		/* 2frame * (1s/7.5frame) *1000ms */
		msleep(135);
		/*msleep(2*100*1000/Capture_Framerate);*/

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
		if ((ov5642_m_ledmod==2) ||((ov5642_m_ledmod==1)&&(R0x5690preview <= 0x30)))
		{
			hrtimer_cancel(&ov5642_flashled_timer);
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
		if (ov5642_use_vfs_flashtime_setting!=0)
			hrtimer_start(&ov5642_flashled_timer,
				ktime_set(ov5642_use_vfs_flashtime_setting / 1000, (ov5642_use_vfs_flashtime_setting % 1000) * 1000000),
				HRTIMER_MODE_REL);
	else
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */
		{
			flash_settime(16);
			ov5642_ledtime=1;
			hrtimer_start(&ov5642_flashled_timer,
				ktime_set(OV5642_FLASHLED_DELAY / 1000, (OV5642_FLASHLED_DELAY % 1000) * 1000000),
				HRTIMER_MODE_REL);

			if (hrtimer_active(&ov5642_flashled_timer))
				printk(KERN_INFO "%s: TIMER running\n", __func__);
		}

		}
#endif
/* } FIH, Charles Huang, 2009/09/01 */

		break;
	}
#endif
/* } FIH, Charles Huang, 2009/07/29 */

	default:
		return -EFAULT;
	}

	return 0;
}

static long ov5642_set_effect(int mode, int effect)
{
	long rc = 0;

	CDBG("ov5642_set_effect, mode = %d, effect = %d\n",
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
		struct ov5642_i2c_reg_conf const ov5642_effect_off_tbl[] = {
			//{0x5001, 0x0000, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0000, BYTE_LEN, 0, 0x0058},
		};

		rc = ov5642_i2c_write_table(&ov5642_effect_off_tbl[0],
				ARRAY_SIZE(ov5642_effect_off_tbl));

		if (rc < 0)
			return rc;
	}
			break;

	case CAMERA_EFFECT_MONO: {//B&W
		struct ov5642_i2c_reg_conf const ov5642_effect_mono_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0018, BYTE_LEN, 0, 0x0058},
			{0x5585, 0x0080, BYTE_LEN, 0, 0xFFFF},
			{0x5586, 0x0080, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_effect_mono_tbl[0],
				ARRAY_SIZE(ov5642_effect_mono_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_NEGATIVE: {//Negative
		struct ov5642_i2c_reg_conf const ov5642_effect_negative_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0040, BYTE_LEN, 0, 0x0040},
		};

		rc = ov5642_i2c_write_table(&ov5642_effect_negative_tbl[0],
				ARRAY_SIZE(ov5642_effect_negative_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_SEPIA: {
		struct ov5642_i2c_reg_conf const ov5642_effect_sepia_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0018, BYTE_LEN, 0, 0x0058},
			{0x5585, 0x0040, BYTE_LEN, 0, 0xFFFF},
			{0x5586, 0x00A0, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_effect_sepia_tbl[0],
				ARRAY_SIZE(ov5642_effect_sepia_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_BLUISH: {//Bluish
		struct ov5642_i2c_reg_conf const ov5642_effect_bluish_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0018, BYTE_LEN, 0, 0x0058},
			{0x5585, 0x00A0, BYTE_LEN, 0, 0xFFFF},
			{0x5586, 0x0040, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_effect_bluish_tbl[0],
				ARRAY_SIZE(ov5642_effect_bluish_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_REDDISH: {//Reddish
		struct ov5642_i2c_reg_conf const ov5642_effect_reddish_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0018, BYTE_LEN, 0, 0x0058},
			{0x5585, 0x0080, BYTE_LEN, 0, 0xFFFF},
			{0x5586, 0x00C0, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_effect_reddish_tbl[0],
				ARRAY_SIZE(ov5642_effect_reddish_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_GREENISH: {//Greenish
		struct ov5642_i2c_reg_conf const ov5642_effect_greenish_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0018, BYTE_LEN, 0, 0x0058},
			{0x5585, 0x0060, BYTE_LEN, 0, 0xFFFF},
			{0x5586, 0x0060, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_effect_greenish_tbl[0],
				ARRAY_SIZE(ov5642_effect_greenish_tbl));

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

static long ov5642_set_wb(int mode, int wb)
{
	long rc = 0;

	CDBG("ov5642_set_wb, mode = %d, wb = %d\n",
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
		struct ov5642_i2c_reg_conf const ov5642_wb_auto_tbl[] = {
			{0x3406, 0x0000, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_wb_auto_tbl[0],
				ARRAY_SIZE(ov5642_wb_auto_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_WB_INCANDESCENT: {//TUNGSTEN
		struct ov5642_i2c_reg_conf const ov5642_wb_incandescent_tbl[] = {
			{0x3406, 0x0001, BYTE_LEN, 0, 0x0001},
			{0x3400, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x3401, 0x005E, BYTE_LEN, 0, 0xFFFF},
			{0x3402, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x3403, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x3404, 0x0008, BYTE_LEN, 0, 0xFFFF},
			{0x3405, 0x0050, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_wb_incandescent_tbl[0],
				ARRAY_SIZE(ov5642_wb_incandescent_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_WB_FLUORESCENT: {//Office
		struct ov5642_i2c_reg_conf const ov5642_wb_fluorescent_tbl[] = {
			{0x3406, 0x0001, BYTE_LEN, 0, 0x0001},
			{0x3400, 0x0005, BYTE_LEN, 0, 0xFFFF},
			{0x3401, 0x0080, BYTE_LEN, 0, 0xFFFF},
			{0x3402, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x3403, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x3404, 0x0007, BYTE_LEN, 0, 0xFFFF},
			{0x3405, 0x0080, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_wb_fluorescent_tbl[0],
				ARRAY_SIZE(ov5642_wb_fluorescent_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_WB_DAYLIGHT: {//Sunny
		struct ov5642_i2c_reg_conf const ov5642_wb_daylight_tbl[] = {
			{0x3406, 0x0001, BYTE_LEN, 0, 0x0001},
			{0x3400, 0x0006, BYTE_LEN, 0, 0xFFFF},
			{0x3401, 0x0060, BYTE_LEN, 0, 0xFFFF},
			{0x3402, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x3403, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x3404, 0x0004, BYTE_LEN, 0, 0xFFFF},//check
			{0x3405, 0x00B9, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_wb_daylight_tbl[0],
				ARRAY_SIZE(ov5642_wb_daylight_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_WB_CLOUDY_DAYLIGHT: {//Cloudy
		struct ov5642_i2c_reg_conf const ov5642_wb_cloudydaylight_tbl[] = {
			{0x3406, 0x0001, BYTE_LEN, 0, 0x0001},
			{0x3400, 0x0006, BYTE_LEN, 0, 0xFFFF},
			{0x3401, 0x009C, BYTE_LEN, 0, 0xFFFF},
			{0x3402, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x3403, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x3404, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x3405, 0x003F, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_wb_cloudydaylight_tbl[0],
				ARRAY_SIZE(ov5642_wb_cloudydaylight_tbl));

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				0x3405, 0x45, BYTE_LEN);

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


static long ov5642_set_brightness(int mode, int brightness)
{
	long rc = 0;

	CDBG("ov5642_set_brightness, mode = %d, brightness = %d\n",
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
		struct ov5642_i2c_reg_conf const ov5642_brightness_0_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5589, 0x0030, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x558A, 0x0008, BYTE_LEN, 0, 0x0008},
		};

		rc = ov5642_i2c_write_table(&ov5642_brightness_0_tbl[0],
				ARRAY_SIZE(ov5642_brightness_0_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_BRIGHTNESS_1: {
		struct ov5642_i2c_reg_conf const ov5642_brightness_1_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5589, 0x0020, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x558A, 0x0008, BYTE_LEN, 0, 0x0008},
		};

		rc = ov5642_i2c_write_table(&ov5642_brightness_1_tbl[0],
				ARRAY_SIZE(ov5642_brightness_1_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_BRIGHTNESS_2: {
		struct ov5642_i2c_reg_conf const ov5642_brightness_2_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5589, 0x0010, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x558A, 0x0008, BYTE_LEN, 0, 0x0008},
		};

		rc = ov5642_i2c_write_table(&ov5642_brightness_2_tbl[0],
				ARRAY_SIZE(ov5642_brightness_2_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_BRIGHTNESS_3: {/* Default */
		struct ov5642_i2c_reg_conf const ov5642_brightness_3_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5589, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x558A, 0x0000, BYTE_LEN, 0, 0x0008},
		};

		rc = ov5642_i2c_write_table(&ov5642_brightness_3_tbl[0],
				ARRAY_SIZE(ov5642_brightness_3_tbl));

		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_BRIGHTNESS_4: {
		struct ov5642_i2c_reg_conf const ov5642_brightness_4_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5589, 0x0010, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x558A, 0x0000, BYTE_LEN, 0, 0x0008},
		};

		rc = ov5642_i2c_write_table(&ov5642_brightness_4_tbl[0],
				ARRAY_SIZE(ov5642_brightness_4_tbl));

		if (rc < 0)
			return rc;
	} 
		break;

	case CAMERA_BRIGHTNESS_5: {
		struct ov5642_i2c_reg_conf const ov5642_brightness_5_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5589, 0x0020, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x558A, 0x0000, BYTE_LEN, 0, 0x0008},
		};

		rc = ov5642_i2c_write_table(&ov5642_brightness_5_tbl[0],
				ARRAY_SIZE(ov5642_brightness_5_tbl));

		if (rc < 0)
			return rc;
	} 
		break;

	case CAMERA_BRIGHTNESS_6: {
		struct ov5642_i2c_reg_conf const ov5642_brightness_6_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5589, 0x0030, BYTE_LEN, 0, 0xFFFF},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x558A, 0x0000, BYTE_LEN, 0, 0x0008},
		};

		rc = ov5642_i2c_write_table(&ov5642_brightness_6_tbl[0],
				ARRAY_SIZE(ov5642_brightness_6_tbl));

		if (rc < 0)
			return rc;
	} 
		break;

	case CAMERA_BRIGHTNESS_7:
	case CAMERA_BRIGHTNESS_8:
	case CAMERA_BRIGHTNESS_9:
	case CAMERA_BRIGHTNESS_10: {
		struct ov5642_i2c_reg_conf const ov5642_brightness_10_tbl[] = {
			{0x3A0F, 0x0090, BYTE_LEN, 0, 0xFFFF},
			{0x3A10, 0x0088, BYTE_LEN, 0, 0xFFFF},
			{0x3A11, 0x00A4, BYTE_LEN, 0, 0xFFFF},
			{0x3A1B, 0x0090, BYTE_LEN, 0, 0xFFFF},
			{0x3A1E, 0x0088, BYTE_LEN, 0, 0xFFFF},
			{0x3A1F, 0x0074, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_brightness_10_tbl[0],
				ARRAY_SIZE(ov5642_brightness_10_tbl));

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
static long ov5642_set_contrast(int mode, int contrast)
{
	long rc = 0;

	CDBG("ov5642_set_contrast, mode = %d, contrast = %d\n",
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
		struct ov5642_i2c_reg_conf const ov5642_contrast_minus_2_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5587, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x5588, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0000, BYTE_LEN, 0, 0x0004},
		};

		rc = ov5642_i2c_write_table(&ov5642_contrast_minus_2_tbl[0],
				ARRAY_SIZE(ov5642_contrast_minus_2_tbl));

		if (rc < 0)
			return rc;

	}
			break;

	case CAMERA_CONTRAST_MINUS_1: {
		struct ov5642_i2c_reg_conf const ov5642_contrast_minus_1_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5587, 0x001C, BYTE_LEN, 0, 0xFFFF},
			{0x5588, 0x001C, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0000, BYTE_LEN, 0, 0x0004},
		};

		rc = ov5642_i2c_write_table(&ov5642_contrast_minus_1_tbl[0],
				ARRAY_SIZE(ov5642_contrast_minus_1_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_CONTRAST_ZERO: {
		struct ov5642_i2c_reg_conf const ov5642_contrast_zero_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5587, 0x0020, BYTE_LEN, 0, 0xFFFF},
			{0x5588, 0x0020, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0000, BYTE_LEN, 0, 0x0004},
		};

		rc = ov5642_i2c_write_table(&ov5642_contrast_zero_tbl[0],
				ARRAY_SIZE(ov5642_contrast_zero_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_CONTRAST_POSITIVE_1: {
		struct ov5642_i2c_reg_conf const ov5642_contrast_positive_1_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5587, 0x0024, BYTE_LEN, 0, 0xFFFF},
			{0x5588, 0x0024, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0000, BYTE_LEN, 0, 0x0004},
		};

		rc = ov5642_i2c_write_table(&ov5642_contrast_positive_1_tbl[0],
				ARRAY_SIZE(ov5642_contrast_positive_1_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_CONTRAST_POSITIVE_2: {
		struct ov5642_i2c_reg_conf const ov5642_contrast_positive_2_tbl[] = {
			{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0004, BYTE_LEN, 0, 0x0004},
			{0x5587, 0x0028, BYTE_LEN, 0, 0xFFFF},
			{0x5588, 0x0028, BYTE_LEN, 0, 0xFFFF},
			{0x558A, 0x0000, BYTE_LEN, 0, 0x0004},
		};

		rc = ov5642_i2c_write_table(&ov5642_contrast_positive_2_tbl[0],
				ARRAY_SIZE(ov5642_contrast_positive_2_tbl));

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

static long ov5642_set_antibanding(int mode, int antibanding)
{
	long rc = 0;

	CDBG("ov5642_set_antibanding, mode = %d, antibanding = %d\n",
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
		struct ov5642_i2c_reg_conf const ov5642_antibanding_off_tbl[] = {
			{0x3013, 0x0000, BYTE_LEN, 0, 0x0020},
		};

		rc = ov5642_i2c_write_table(&ov5642_antibanding_off_tbl[0],
				ARRAY_SIZE(ov5642_antibanding_off_tbl));

		if (rc < 0)
			return rc;

		ov5642_m_60Hz=0;
	}
			break;

	case CAMERA_ANTIBANDING_60HZ: {
		struct ov5642_i2c_reg_conf const ov5642_antibanding_60hz_tbl[] = {
			{0x3C01, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x3C00, 0x0000, BYTE_LEN, 0, 0x0004},
		};

		rc = ov5642_i2c_write_table(&ov5642_antibanding_60hz_tbl[0],
				ARRAY_SIZE(ov5642_antibanding_60hz_tbl));

		if (rc < 0)
			return rc;

		ov5642_m_60Hz=1;
	}
		break;

	case CAMERA_ANTIBANDING_50HZ: {
		struct ov5642_i2c_reg_conf const ov5642_antibanding_60hz_tbl[] = {
			{0x3C01, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x3C00, 0x0004, BYTE_LEN, 0, 0x0004},
		};

		rc = ov5642_i2c_write_table(&ov5642_antibanding_60hz_tbl[0],
				ARRAY_SIZE(ov5642_antibanding_60hz_tbl));

		if (rc < 0)
			return rc;

		ov5642_m_60Hz=0;
	}
		break;

	case CAMERA_ANTIBANDING_AUTO: {
		struct ov5642_i2c_reg_conf const ov5642_antibanding_60hz_tbl[] = {
			{0x3C01, 0x0000, BYTE_LEN, 0, 0x0080},
			{0x3C00, 0x0000, BYTE_LEN, 0, 0x0004},
		};

		rc = ov5642_i2c_write_table(&ov5642_antibanding_60hz_tbl[0],
				ARRAY_SIZE(ov5642_antibanding_60hz_tbl));

		if (rc < 0)
			return rc;

		ov5642_m_60Hz=0;
	}
		break;

	default: {
		if (rc < 0)
			return rc;

		ov5642_m_60Hz=0;
		return -EFAULT;
	}
	}

	return rc;
}

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
static long ov5642_set_ledmod(int mode, int ledmod)
{
	long rc = 0;

	CDBG("ov5642_set_ledmod, mode = %d, ledmod = %d\n",
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
		ov5642_m_ledmod=0;
	}
			break;

	case CAMERA_LED_MODE_AUTO: {
		ov5642_m_ledmod=1;
	}
		break;

	case CAMERA_LED_MODE_ON: {
		ov5642_m_ledmod=2;
	}
		break;

	default: {
		if (rc < 0)
			return rc;

		ov5642_m_ledmod=0;
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
static long ov5642_set_saturation(int mode, int saturation)
{
	long rc = 0;

	CDBG("ov5642_set_saturation, mode = %d, saturation = %d\n",
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
		struct ov5642_i2c_reg_conf const ov5642_saturation_mnius_2_tbl[] = {
			//{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0002, BYTE_LEN, 0, 0x0002},
			{0x5583, 0x0020, BYTE_LEN, 0, 0xFFFF},
			{0x5584, 0x0020, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_saturation_mnius_2_tbl[0],
				ARRAY_SIZE(ov5642_saturation_mnius_2_tbl));

		if (rc < 0)
			return rc;

	}
			break;

	case CAMERA_SATURATION_MINUS_1: {
		struct ov5642_i2c_reg_conf const ov5642_saturation_mnius_1_tbl[] = {
			//{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0002, BYTE_LEN, 0, 0x0002},
			{0x5583, 0x0030, BYTE_LEN, 0, 0xFFFF},
			{0x5584, 0x0030, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_saturation_mnius_1_tbl[0],
				ARRAY_SIZE(ov5642_saturation_mnius_1_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SATURATION_ZERO: {
		struct ov5642_i2c_reg_conf const ov5642_saturation_zero_tbl[] = {
			//{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0002, BYTE_LEN, 0, 0x0002},
			{0x5583, 0x0040, BYTE_LEN, 0, 0xFFFF},
			{0x5584, 0x0040, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_saturation_zero_tbl[0],
				ARRAY_SIZE(ov5642_saturation_zero_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SATURATION_POSITIVE_1: {
		struct ov5642_i2c_reg_conf const ov5642_saturation_positive_1_tbl[] = {
			//{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0002, BYTE_LEN, 0, 0x0002},
			{0x5583, 0x004D, BYTE_LEN, 0, 0xFFFF},
			{0x5584, 0x004D, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_saturation_positive_1_tbl[0],
				ARRAY_SIZE(ov5642_saturation_positive_1_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SATURATION_POSITIVE_2: {
		struct ov5642_i2c_reg_conf const ov5642_saturation_positive_2_tbl[] = {
			//{0x5001, 0x0080, BYTE_LEN, 0, 0x0080},
			{0x5580, 0x0002, BYTE_LEN, 0, 0x0002},
			{0x5583, 0x0053, BYTE_LEN, 0, 0xFFFF},
			{0x5584, 0x0053, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_saturation_positive_2_tbl[0],
				ARRAY_SIZE(ov5642_saturation_positive_2_tbl));

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
static long ov5642_set_sharpness(int mode, int sharpness)
{
	long rc = 0;

	CDBG("ov5642_set_sharpness, mode = %d, sharpness = %d\n",
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
		struct ov5642_i2c_reg_conf const ov5642_sharpness_zero_tbl[] = {
			{0x530A, 0x0000, BYTE_LEN, 0, 0x0008},
			{0x530C, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x530D, 0x000C, BYTE_LEN, 0, 0xFFFF},
			{0x5312, 0x0040, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_sharpness_zero_tbl[0],
				ARRAY_SIZE(ov5642_sharpness_zero_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SHARPNESS_POSITIVE_1: {
		struct ov5642_i2c_reg_conf const ov5642_sharpness_positive_1_tbl[] = {
			{0x530A, 0x0000, BYTE_LEN, 0, 0x0008},
			{0x530C, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x530D, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x5312, 0x0020, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_sharpness_positive_1_tbl[0],
				ARRAY_SIZE(ov5642_sharpness_positive_1_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SHARPNESS_POSITIVE_2: {
		struct ov5642_i2c_reg_conf const ov5642_sharpness_positive_2_tbl[] = {
			{0x530A, 0x0000, BYTE_LEN, 0, 0x0008},
			{0x530C, 0x0008, BYTE_LEN, 0, 0xFFFF},
			{0x530D, 0x0030, BYTE_LEN, 0, 0xFFFF},
			{0x5312, 0x0010, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_sharpness_positive_2_tbl[0],
				ARRAY_SIZE(ov5642_sharpness_positive_2_tbl));

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
static long ov5642_set_meteringmod(int mode, int meteringmod)
{
	long rc = 0;

	CDBG("ov5642_set_meteringmod, mode = %d, meteringmod = %d\n",
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
		struct ov5642_i2c_reg_conf const ov5642_meteringmod_average_tbl[] = {
			{0x5680, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5681, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5684, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5685, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5682, 0x0005, BYTE_LEN, 0, 0xFFFF},
			{0x5683, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5686, 0x0003, BYTE_LEN, 0, 0xFFFF},
			{0x5687, 0x00BC, BYTE_LEN, 0, 0xFFFF},
			{0x5688, 0x0011, BYTE_LEN, 0, 0xFFFF},
			{0x5689, 0x0011, BYTE_LEN, 0, 0xFFFF},
			{0x568A, 0x0011, BYTE_LEN, 0, 0xFFFF},
			{0x568B, 0x0011, BYTE_LEN, 0, 0xFFFF},
			{0x568C, 0x0011, BYTE_LEN, 0, 0xFFFF},
			{0x568D, 0x0011, BYTE_LEN, 0, 0xFFFF},
			{0x568E, 0x0011, BYTE_LEN, 0, 0xFFFF},
			{0x568F, 0x0011, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_meteringmod_average_tbl[0],
				ARRAY_SIZE(ov5642_meteringmod_average_tbl));

		if (rc < 0)
			return rc;

	}
			break;

	case CAMERA_CENTER_METERING: {
		struct ov5642_i2c_reg_conf const ov5642_meteringmod_center_tbl[] = {
			{0x5680, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5681, 0x0040, BYTE_LEN, 0, 0xFFFF},
			{0x5684, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5685, 0x00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5682, 0x0003, BYTE_LEN, 0, 0xFFFF},
			{0x5683, 0x00C0, BYTE_LEN, 0, 0xFFFF},
			{0x5686, 0x0002, BYTE_LEN, 0, 0xFFFF},
			{0x5687, 0x00D0, BYTE_LEN, 0, 0xFFFF},
			{0x5688, 0x0033, BYTE_LEN, 0, 0xFFFF},
			{0x5689, 0x0033, BYTE_LEN, 0, 0xFFFF},
			{0x568A, 0x00C5, BYTE_LEN, 0, 0xFFFF},
			{0x568B, 0x005C, BYTE_LEN, 0, 0xFFFF},
			{0x568C, 0x00C5, BYTE_LEN, 0, 0xFFFF},
			{0x568D, 0x005C, BYTE_LEN, 0, 0xFFFF},
			{0x568E, 0x0083, BYTE_LEN, 0, 0xFFFF},
			{0x568F, 0x0038, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_meteringmod_center_tbl[0],
				ARRAY_SIZE(ov5642_meteringmod_center_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SPOT_METERING: {
		struct ov5642_i2c_reg_conf const ov5642_meteringmod_spot_tbl[] = {
			{0x5680, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5681, 0x0040, BYTE_LEN, 0, 0xFFFF},
			{0x5684, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5685, 0x00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5682, 0x0003, BYTE_LEN, 0, 0xFFFF},
			{0x5683, 0x00C0, BYTE_LEN, 0, 0xFFFF},
			{0x5686, 0x0002, BYTE_LEN, 0, 0xFFFF},
			{0x5687, 0x00D0, BYTE_LEN, 0, 0xFFFF},
			{0x5688, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5689, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x568A, 0x00E0, BYTE_LEN, 0, 0xFFFF},
			{0x568B, 0x000E, BYTE_LEN, 0, 0xFFFF},
			{0x568C, 0x00E0, BYTE_LEN, 0, 0xFFFF},
			{0x568D, 0x000F, BYTE_LEN, 0, 0xFFFF},
			{0x568E, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x568F, 0x0000, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_meteringmod_spot_tbl[0],
				ARRAY_SIZE(ov5642_meteringmod_spot_tbl));

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
static long ov5642_set_scenemod(int mode, int scenemod)
{
	long rc = 0;

	CDBG("ov5642_set_scenemod, mode = %d, scenemod = %d\n",
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
		struct ov5642_i2c_reg_conf const ov5642_scenemod_auto_tbl[] = {
		};

		rc = ov5642_i2c_write_table(&ov5642_scenemod_auto_tbl[0],
				ARRAY_SIZE(ov5642_scenemod_auto_tbl));

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				0x3A00, 0x78, BYTE_LEN);

		if (rc < 0)
			return rc;

	}
			break;

	case CAMERA_SCENE_MODE_LANDSCAPE: {
		struct ov5642_i2c_reg_conf const ov5642_scenemod_landscape_tbl[] = {
			/* landscape mode */
			/* landscape CCM */
			{0x5380, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5381, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5382, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5383, 0x0017, BYTE_LEN, 0, 0xFFFF},
			{0x5384, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5385, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5386, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5387, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5388, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5389, 0x0072, BYTE_LEN, 0, 0xFFFF},
			{0x538A, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538B, 0x0035, BYTE_LEN, 0, 0xFFFF},
			{0x538C, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538D, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538E, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538F, 0x0024, BYTE_LEN, 0, 0xFFFF},
			{0x5390, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5391, 0x00FE, BYTE_LEN, 0, 0xFFFF},
			{0x5392, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5393, 0x00A0, BYTE_LEN, 0, 0xFFFF},
			{0x5394, 0x0008, BYTE_LEN, 0, 0xFFFF},
			/*Advanced AWB 0209*/
			{0x3406, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5180, 0x00FF, BYTE_LEN, 0, 0xFFFF},
			{0x5181, 0x0050, BYTE_LEN, 0, 0xFFFF},
			{0x5182, 0x0011, BYTE_LEN, 0, 0xFFFF},
			{0x5183, 0x0014, BYTE_LEN, 0, 0xFFFF},
			{0x5184, 0x0025, BYTE_LEN, 0, 0xFFFF},
			{0x5185, 0x0024, BYTE_LEN, 0, 0xFFFF},
			{0x5186, 0x000F, BYTE_LEN, 0, 0xFFFF},
			{0x5187, 0x000B, BYTE_LEN, 0, 0xFFFF},
			{0x5188, 0x0008, BYTE_LEN, 0, 0xFFFF},
			{0x5189, 0x0077, BYTE_LEN, 0, 0xFFFF},
			{0x518A, 0x004F, BYTE_LEN, 0, 0xFFFF},
			{0x518B, 0x00DA, BYTE_LEN, 0, 0xFFFF},
			{0x518C, 0x00B9, BYTE_LEN, 0, 0xFFFF},
			{0x518D, 0x0023, BYTE_LEN, 0, 0xFFFF},
			{0x518E, 0x0025, BYTE_LEN, 0, 0xFFFF},
			{0x518F, 0x0054, BYTE_LEN, 0, 0xFFFF},
			{0x5190, 0x0040, BYTE_LEN, 0, 0xFFFF},
			{0x5191, 0x00F8, BYTE_LEN, 0, 0xFFFF},
			{0x5192, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x5193, 0x0070, BYTE_LEN, 0, 0xFFFF},
			{0x5194, 0x00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5195, 0x00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5196, 0x0003, BYTE_LEN, 0, 0xFFFF},
			{0x5197, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5198, 0x0006, BYTE_LEN, 0, 0xFFFF},
			{0x5199, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x519A, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x519B, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x519C, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x519D, 0x00F0, BYTE_LEN, 0, 0xFFFF},
			{0x519E, 0x0000, BYTE_LEN, 0, 0xFFFF},
			/*Gamma*/    
			{0x5480, 0x000B, BYTE_LEN, 0, 0xFFFF},
			{0x5481, 0x0019, BYTE_LEN, 0, 0xFFFF},
			{0x5482, 0x0031, BYTE_LEN, 0, 0xFFFF},
			{0x5483, 0x0054, BYTE_LEN, 0, 0xFFFF},
			{0x5484, 0x0065, BYTE_LEN, 0, 0xFFFF},
			{0x5485, 0x0071, BYTE_LEN, 0, 0xFFFF},
			{0x5486, 0x007D, BYTE_LEN, 0, 0xFFFF},
			{0x5487, 0x0087, BYTE_LEN, 0, 0xFFFF},
			{0x5488, 0x0091, BYTE_LEN, 0, 0xFFFF},
			{0x5489, 0x009A, BYTE_LEN, 0, 0xFFFF},
			{0x548A, 0x00AA, BYTE_LEN, 0, 0xFFFF},
			{0x548B, 0x00B8, BYTE_LEN, 0, 0xFFFF},
			{0x548C, 0x00CD, BYTE_LEN, 0, 0xFFFF},
			{0x548D, 0x00DD, BYTE_LEN, 0, 0xFFFF},
			{0x548E, 0x00EA, BYTE_LEN, 0, 0xFFFF},
			{0x548F, 0x001D, BYTE_LEN, 0, 0xFFFF},
			/* auto sharpness +2 */
			{0x530A, 0x0000, BYTE_LEN, 0, 0x0008},
			{0x530C, 0x0008, BYTE_LEN, 0, 0xFFFF},
			{0x530D, 0x0030, BYTE_LEN, 0, 0xFFFF},
			{0x5312, 0x0010, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_scenemod_landscape_tbl[0],
				ARRAY_SIZE(ov5642_scenemod_landscape_tbl));

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				0x3A00, 0x78, BYTE_LEN);

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SCENE_MODE_PORTRAIT: {
		struct ov5642_i2c_reg_conf const ov5642_scenemod_portrait_tbl[] = {
			/* Protarit mode */
			/* protrait_CCM */
			{0x5380, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5381, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5382, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5383, 0x0017, BYTE_LEN, 0, 0xFFFF},
			{0x5384, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5385, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5386, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5387, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5388, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5389, 0x0072, BYTE_LEN, 0, 0xFFFF},
			{0x538a, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538b, 0x0035, BYTE_LEN, 0, 0xFFFF},
			{0x538c, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538d, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538e, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538f, 0x0024, BYTE_LEN, 0, 0xFFFF},
			{0x5390, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5391, 0x008D, BYTE_LEN, 0, 0xFFFF},
			{0x5392, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5393, 0x00A0, BYTE_LEN, 0, 0xFFFF},
			{0x5394, 0x0008, BYTE_LEN, 0, 0xFFFF},
			/*Advanced AWB 0209*/
			{0x3406, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5180, 0x00FF, BYTE_LEN, 0, 0xFFFF},
			{0x5181, 0x0050, BYTE_LEN, 0, 0xFFFF},
			{0x5182, 0x0011, BYTE_LEN, 0, 0xFFFF},
			{0x5183, 0x0014, BYTE_LEN, 0, 0xFFFF},
			{0x5184, 0x0025, BYTE_LEN, 0, 0xFFFF},
			{0x5185, 0x0024, BYTE_LEN, 0, 0xFFFF},
			{0x5186, 0x000F, BYTE_LEN, 0, 0xFFFF},
			{0x5187, 0x000B, BYTE_LEN, 0, 0xFFFF},
			{0x5188, 0x0008, BYTE_LEN, 0, 0xFFFF},
			{0x5189, 0x0077, BYTE_LEN, 0, 0xFFFF},
			{0x518A, 0x004F, BYTE_LEN, 0, 0xFFFF},
			{0x518B, 0x00DA, BYTE_LEN, 0, 0xFFFF},
			{0x518C, 0x00B9, BYTE_LEN, 0, 0xFFFF},
			{0x518D, 0x0023, BYTE_LEN, 0, 0xFFFF},
			{0x518E, 0x0025, BYTE_LEN, 0, 0xFFFF},
			{0x518F, 0x0054, BYTE_LEN, 0, 0xFFFF},
			{0x5190, 0x0040, BYTE_LEN, 0, 0xFFFF},
			{0x5191, 0x00F8, BYTE_LEN, 0, 0xFFFF},
			{0x5192, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x5193, 0x0070, BYTE_LEN, 0, 0xFFFF},
			{0x5194, 0x00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5195, 0x00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5196, 0x0003, BYTE_LEN, 0, 0xFFFF},
			{0x5197, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5198, 0x0006, BYTE_LEN, 0, 0xFFFF},
			{0x5199, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x519A, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x519B, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x519C, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x519D, 0x00F0, BYTE_LEN, 0, 0xFFFF},
			{0x519E, 0x0000, BYTE_LEN, 0, 0xFFFF},
			/*Gamma*/    
			{0x5480, 0x000B, BYTE_LEN, 0, 0xFFFF},
			{0x5481, 0x0019, BYTE_LEN, 0, 0xFFFF},
			{0x5482, 0x0031, BYTE_LEN, 0, 0xFFFF},
			{0x5483, 0x0054, BYTE_LEN, 0, 0xFFFF},
			{0x5484, 0x0065, BYTE_LEN, 0, 0xFFFF},
			{0x5485, 0x0071, BYTE_LEN, 0, 0xFFFF},
			{0x5486, 0x007D, BYTE_LEN, 0, 0xFFFF},
			{0x5487, 0x0087, BYTE_LEN, 0, 0xFFFF},
			{0x5488, 0x0091, BYTE_LEN, 0, 0xFFFF},
			{0x5489, 0x009A, BYTE_LEN, 0, 0xFFFF},
			{0x548A, 0x00AA, BYTE_LEN, 0, 0xFFFF},
			{0x548B, 0x00B8, BYTE_LEN, 0, 0xFFFF},
			{0x548C, 0x00CD, BYTE_LEN, 0, 0xFFFF},
			{0x548D, 0x00DD, BYTE_LEN, 0, 0xFFFF},
			{0x548E, 0x00EA, BYTE_LEN, 0, 0xFFFF},
			{0x548F, 0x001D, BYTE_LEN, 0, 0xFFFF},
			/* auto sharpness +2 */
			{0x530A, 0x0000, BYTE_LEN, 0, 0x0008},
			{0x530C, 0x0008, BYTE_LEN, 0, 0xFFFF},
			{0x530D, 0x0030, BYTE_LEN, 0, 0xFFFF},
			{0x5312, 0x0010, BYTE_LEN, 0, 0xFFFF},
			/* Auto denoise +2 */
			{0x528A, 0X0008, BYTE_LEN, 0, 0xFFFF},
			{0x528B, 0X0010, BYTE_LEN, 0, 0xFFFF},
			{0x528C, 0X0020, BYTE_LEN, 0, 0xFFFF},
			{0x528D, 0X0030, BYTE_LEN, 0, 0xFFFF},
			{0x528E, 0X0050, BYTE_LEN, 0, 0xFFFF},
			{0x528F, 0X00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5290, 0X00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5292, 0X0000, BYTE_LEN, 0, 0xFFFF}, /* UV */
			{0x5293, 0X0010, BYTE_LEN, 0, 0xFFFF},
			{0x5294, 0X0000, BYTE_LEN, 0, 0xFFFF},
			{0x5295, 0X0020, BYTE_LEN, 0, 0xFFFF},
			{0x5296, 0X0000, BYTE_LEN, 0, 0xFFFF},
			{0x5297, 0X0040, BYTE_LEN, 0, 0xFFFF},
			{0x5298, 0X0000, BYTE_LEN, 0, 0xFFFF},
			{0x5299, 0X0060, BYTE_LEN, 0, 0xFFFF},
			{0x529A, 0X0001, BYTE_LEN, 0, 0xFFFF},
			{0x529B, 0X0080, BYTE_LEN, 0, 0xFFFF},
			{0x529C, 0X0000, BYTE_LEN, 0, 0xFFFF},
			{0x529D, 0X00A0, BYTE_LEN, 0, 0xFFFF},
			{0x529E, 0X0001, BYTE_LEN, 0, 0xFFFF},
			{0x529F, 0X00F8, BYTE_LEN, 0, 0xFFFF},
			{0x5282, 0X0000, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_scenemod_portrait_tbl[0],
				ARRAY_SIZE(ov5642_scenemod_portrait_tbl));

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				0x3A00, 0x78, BYTE_LEN);

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SCENE_MODE_NIGHT: {
		struct ov5642_i2c_reg_conf const ov5642_scenemod_night_tbl[] = {
			/* Night mode */
			{0x3A00, 0x007C, BYTE_LEN, 0, 0xFFFF},
			{0x3A02, 0x0000, BYTE_LEN, 0, 0x000F},
			{0x3A03, 0x007D, BYTE_LEN, 0, 0xFFFF},
			{0x3A04, 0x0000, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_scenemod_night_tbl[0],
				ARRAY_SIZE(ov5642_scenemod_night_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SCENE_MODE_NIGHT_PORTRAIT: {
		struct ov5642_i2c_reg_conf const ov5642_scenemod_night_portrait_tbl[] = {
			/* Protarit mode */
			/* protrait_CCM */
			{0x5380, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5381, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5382, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5383, 0x0017, BYTE_LEN, 0, 0xFFFF},
			{0x5384, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5385, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5386, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5387, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5388, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5389, 0x0072, BYTE_LEN, 0, 0xFFFF},
			{0x538a, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538b, 0x0035, BYTE_LEN, 0, 0xFFFF},
			{0x538c, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538d, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538e, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538f, 0x0024, BYTE_LEN, 0, 0xFFFF},
			{0x5390, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5391, 0x008D, BYTE_LEN, 0, 0xFFFF},
			{0x5392, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5393, 0x00A0, BYTE_LEN, 0, 0xFFFF},
			{0x5394, 0x0008, BYTE_LEN, 0, 0xFFFF},
			/*Advanced AWB 0209*/
			{0x3406, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5180, 0x00FF, BYTE_LEN, 0, 0xFFFF},
			{0x5181, 0x0050, BYTE_LEN, 0, 0xFFFF},
			{0x5182, 0x0011, BYTE_LEN, 0, 0xFFFF},
			{0x5183, 0x0014, BYTE_LEN, 0, 0xFFFF},
			{0x5184, 0x0025, BYTE_LEN, 0, 0xFFFF},
			{0x5185, 0x0024, BYTE_LEN, 0, 0xFFFF},
			{0x5186, 0x000F, BYTE_LEN, 0, 0xFFFF},
			{0x5187, 0x000B, BYTE_LEN, 0, 0xFFFF},
			{0x5188, 0x0008, BYTE_LEN, 0, 0xFFFF},
			{0x5189, 0x0077, BYTE_LEN, 0, 0xFFFF},
			{0x518A, 0x004F, BYTE_LEN, 0, 0xFFFF},
			{0x518B, 0x00DA, BYTE_LEN, 0, 0xFFFF},
			{0x518C, 0x00B9, BYTE_LEN, 0, 0xFFFF},
			{0x518D, 0x0023, BYTE_LEN, 0, 0xFFFF},
			{0x518E, 0x0025, BYTE_LEN, 0, 0xFFFF},
			{0x518F, 0x0054, BYTE_LEN, 0, 0xFFFF},
			{0x5190, 0x0040, BYTE_LEN, 0, 0xFFFF},
			{0x5191, 0x00F8, BYTE_LEN, 0, 0xFFFF},
			{0x5192, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x5193, 0x0070, BYTE_LEN, 0, 0xFFFF},
			{0x5194, 0x00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5195, 0x00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5196, 0x0003, BYTE_LEN, 0, 0xFFFF},
			{0x5197, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5198, 0x0006, BYTE_LEN, 0, 0xFFFF},
			{0x5199, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x519A, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x519B, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x519C, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x519D, 0x00F0, BYTE_LEN, 0, 0xFFFF},
			{0x519E, 0x0000, BYTE_LEN, 0, 0xFFFF},
			/*Gamma*/    
			{0x5480, 0x000B, BYTE_LEN, 0, 0xFFFF},
			{0x5481, 0x0019, BYTE_LEN, 0, 0xFFFF},
			{0x5482, 0x0031, BYTE_LEN, 0, 0xFFFF},
			{0x5483, 0x0054, BYTE_LEN, 0, 0xFFFF},
			{0x5484, 0x0065, BYTE_LEN, 0, 0xFFFF},
			{0x5485, 0x0071, BYTE_LEN, 0, 0xFFFF},
			{0x5486, 0x007D, BYTE_LEN, 0, 0xFFFF},
			{0x5487, 0x0087, BYTE_LEN, 0, 0xFFFF},
			{0x5488, 0x0091, BYTE_LEN, 0, 0xFFFF},
			{0x5489, 0x009A, BYTE_LEN, 0, 0xFFFF},
			{0x548A, 0x00AA, BYTE_LEN, 0, 0xFFFF},
			{0x548B, 0x00B8, BYTE_LEN, 0, 0xFFFF},
			{0x548C, 0x00CD, BYTE_LEN, 0, 0xFFFF},
			{0x548D, 0x00DD, BYTE_LEN, 0, 0xFFFF},
			{0x548E, 0x00EA, BYTE_LEN, 0, 0xFFFF},
			{0x548F, 0x001D, BYTE_LEN, 0, 0xFFFF},
			/* auto sharpness +2 */
			{0x530A, 0x0000, BYTE_LEN, 0, 0x0008},
			{0x530C, 0x0008, BYTE_LEN, 0, 0xFFFF},
			{0x530D, 0x0030, BYTE_LEN, 0, 0xFFFF},
			{0x5312, 0x0010, BYTE_LEN, 0, 0xFFFF},
			/* Auto denoise +2 */
			{0x528A, 0X0008, BYTE_LEN, 0, 0xFFFF},
			{0x528B, 0X0010, BYTE_LEN, 0, 0xFFFF},
			{0x528C, 0X0020, BYTE_LEN, 0, 0xFFFF},
			{0x528D, 0X0030, BYTE_LEN, 0, 0xFFFF},
			{0x528E, 0X0050, BYTE_LEN, 0, 0xFFFF},
			{0x528F, 0X00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5290, 0X00F0, BYTE_LEN, 0, 0xFFFF},
			{0x5292, 0X0000, BYTE_LEN, 0, 0xFFFF}, /* UV */
			{0x5293, 0X0010, BYTE_LEN, 0, 0xFFFF},
			{0x5294, 0X0000, BYTE_LEN, 0, 0xFFFF},
			{0x5295, 0X0020, BYTE_LEN, 0, 0xFFFF},
			{0x5296, 0X0000, BYTE_LEN, 0, 0xFFFF},
			{0x5297, 0X0040, BYTE_LEN, 0, 0xFFFF},
			{0x5298, 0X0000, BYTE_LEN, 0, 0xFFFF},
			{0x5299, 0X0060, BYTE_LEN, 0, 0xFFFF},
			{0x529A, 0X0001, BYTE_LEN, 0, 0xFFFF},
			{0x529B, 0X0080, BYTE_LEN, 0, 0xFFFF},
			{0x529C, 0X0000, BYTE_LEN, 0, 0xFFFF},
			{0x529D, 0X00A0, BYTE_LEN, 0, 0xFFFF},
			{0x529E, 0X0001, BYTE_LEN, 0, 0xFFFF},
			{0x529F, 0X00F8, BYTE_LEN, 0, 0xFFFF},
			{0x5282, 0X0000, BYTE_LEN, 0, 0xFFFF},
			/* Night mode */
			{0x3A00, 0x007C, BYTE_LEN, 0, 0xFFFF},
			{0x3A02, 0x0000, BYTE_LEN, 0, 0x000F},
			{0x3A03, 0x007D, BYTE_LEN, 0, 0xFFFF},
			{0x3A04, 0x0000, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_scenemod_night_portrait_tbl[0],
				ARRAY_SIZE(ov5642_scenemod_night_portrait_tbl));

		if (rc < 0)
			return rc;

	}
		break;

	case CAMERA_SCENE_MODE_SUNSET: {
		struct ov5642_i2c_reg_conf const ov5642_scenemod_sunset_tbl[] = {
			/* Sunset mode */
			/* Landscape mode CCM */
			{0x5380, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5381, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5382, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5383, 0x0017, BYTE_LEN, 0, 0xFFFF},
			{0x5384, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5385, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5386, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5387, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5388, 0x0001, BYTE_LEN, 0, 0xFFFF},
			{0x5389, 0x0072, BYTE_LEN, 0, 0xFFFF},
			{0x538A, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538B, 0x0035, BYTE_LEN, 0, 0xFFFF},
			{0x538C, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538D, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538E, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x538F, 0x0024, BYTE_LEN, 0, 0xFFFF},
			{0x5390, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5391, 0x00FE, BYTE_LEN, 0, 0xFFFF},
			{0x5392, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x5393, 0x00A0, BYTE_LEN, 0, 0xFFFF},
			{0x5394, 0x0008, BYTE_LEN, 0, 0xFFFF},
			/* Sunset_MWB */
			{0x3406, 0x0001, BYTE_LEN, 0, 0x0001}, /* Man En */
			{0x3400, 0x0003, BYTE_LEN, 0, 0xFFFF}, /* R Gain */
			{0x3401, 0x0000, BYTE_LEN, 0, 0xFFFF},
			{0x3402, 0x0002, BYTE_LEN, 0, 0xFFFF}, /* G Gain */
			{0x3403, 0x0079, BYTE_LEN, 0, 0xFFFF},
			{0x3404, 0x0003, BYTE_LEN, 0, 0xFFFF}, /* B Gain */
			{0x3405, 0x000F, BYTE_LEN, 0, 0xFFFF},         
			/* Contrast+1 */
			{0x530A, 0x0000, BYTE_LEN, 0, 0x0008},
			{0x530C, 0x0004, BYTE_LEN, 0, 0xFFFF},
			{0x530D, 0x0018, BYTE_LEN, 0, 0xFFFF},
			{0x5312, 0x0020, BYTE_LEN, 0, 0xFFFF},
		};

		rc = ov5642_i2c_write_table(&ov5642_scenemod_sunset_tbl[0],
				ARRAY_SIZE(ov5642_scenemod_sunset_tbl));

		if (rc < 0)
			return rc;

		rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
				0x3A00, 0x78, BYTE_LEN);

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

static int ov5642_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	uint16_t model_id = 0;
	int rc = 0;
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifndef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
	struct file *ov5642_Fs = NULL;
	struct inode		*inode = NULL;
	int			length;
#endif
#endif
/* } FIH, Charles Huang, 2009/06/24 */

	/* OV suggested Power up block End */
	/* Read the Model ID of the sensor */
	/* Read REG_OV5642_MODEL_ID_HI & REG_OV5642_MODEL_ID_LO */
	rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
		REG_OV5642_MODEL_ID_HI, &model_id, WORD_LEN);
	if (rc < 0)
		goto init_probe_fail;

	CDBG("ov5642 model_id = 0x%x\n", model_id);

	/* Check if it matches it with the value in Datasheet */
	if (model_id != OV5642_MODEL_ID) {
		rc = -EFAULT;
		goto init_probe_fail;
	}

	/* Get version */
	rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
	0x302A, &model_id, BYTE_LEN);
	CDBG("ov5642 version reg 0x302A = 0x%x\n", model_id);

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifndef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
	if(ov5642_Fs==NULL)
	{
		ov5642_Fs = filp_open("/etc/ov5642.cfg", O_RDONLY, 0);

		if(IS_ERR(ov5642_Fs))
		{
			ov5642_Fs = NULL;
			printk(KERN_ERR "Can't open /etc/ov5642.cfg\n");
		}


		if (!ov5642_Fs->f_op)
		{
			ov5642_Fs = NULL;
			printk("%s: File Operation Method Error\n", __FUNCTION__);
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
		inode = ov5642_Fs->f_path.dentry->d_inode;
#else
		inode = ov5642_Fs->f_dentry->d_inode;
#endif

		if (!inode)
		{
			printk("%s: Get inode from filp failed\n", __FUNCTION__);
		}
		/* file's size */
		length = i_size_read(inode->i_mapping->host);

		printk(KERN_INFO "Open /etc/ov5642.cfg\n");
		filp_close(ov5642_Fs, NULL);   
	}
#endif /* OV5642_USE_VFS */
#endif
/* } FIH, Charles Huang, 2009/06/24 */

	rc = ov5642_reg_init();
	if (rc < 0)
		goto init_probe_fail;

	return rc;

init_probe_fail:
	return rc;
}

int ov5642_sensor_init(const struct msm_camera_sensor_info *data)
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

	ov5642_ctrl = kzalloc(sizeof(struct ov5642_ctrl), GFP_KERNEL);
	if (!ov5642_ctrl) {
		CDBG("ov5642_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		ov5642_ctrl->sensordata = data;

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

	rc = ov5642_reset(data);
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
#ifdef OV5642_USE_VFS
	if (ov5642_use_vfs_mclk_setting!=0)
		msm_camio_clk_rate_set(ov5642_use_vfs_mclk_setting);
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

	rc = ov5642_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("ov5642_sensor_init failed!\n");
		goto init_fail;
	}

init_done:
	return rc;

init_fail:
	kfree(ov5642_ctrl);
	return rc;
}

static int ov5642_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov5642_wait_queue);
	return 0;
}

int ov5642_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(
				&cfg_data,
				(void *)argp,
				sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&ov5642_sem); */
	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = ov5642_set_sensor_mode(
					cfg_data.mode);
		break;

	case CFG_SET_EFFECT:
		rc = ov5642_set_effect(
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
		rc = ov5642_set_brightness(
					cfg_data.mode,
					cfg_data.cfg.brightness);
		break;

	case CFG_SET_CONTRAST:
		rc = ov5642_set_contrast(
					cfg_data.mode,
					cfg_data.cfg.contrast);
		break;

	case CFG_SET_EXPOSURE_MODE:
		rc = -EFAULT;
		break;

	case CFG_SET_WB:
		rc = ov5642_set_wb(
					cfg_data.mode,
					cfg_data.cfg.wb);
		break;

	case CFG_SET_ANTIBANDING:
		rc = ov5642_set_antibanding(
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
		rc = ov5642_set_ledmod(
					cfg_data.mode,
					cfg_data.cfg.ledmod);
		break;
#endif
/* } FIH, Charles Huang, 2009/09/01 */

/* FIH, Charles Huang, 2009/11/05 { */
/* [FXX_CR], saturation function  */
#ifdef CONFIG_FIH_FXX
	case CFG_SET_SATURATION:
		rc = ov5642_set_saturation(
					cfg_data.mode,
					cfg_data.cfg.saturation);
		break;
#endif
/* } FIH, Charles Huang, 2009/11/05 */

/* FIH, Charles Huang, 2009/11/05 { */
/* [FXX_CR], sharpness function  */
#ifdef CONFIG_FIH_FXX
	case CFG_SET_SHARPNESS:
		rc = ov5642_set_sharpness(
					cfg_data.mode,
					cfg_data.cfg.sharpness);
		break;
#endif
/* } FIH, Charles Huang, 2009/11/05 */

/* FIH, Charles Huang, 2009/11/05 { */
/* [FXX_CR], metering mode function  */
#ifdef CONFIG_FIH_FXX
	case CFG_SET_METERINGMOD:
		rc = ov5642_set_meteringmod(
					cfg_data.mode,
					cfg_data.cfg.meteringmod);
		break;
#endif
/* } FIH, Charles Huang, 2009/11/05 */

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], scene mode function  */
#ifdef CONFIG_FIH_FXX
	case CFG_SET_SCENEMOD:
		rc = ov5642_set_scenemod(
					cfg_data.mode,
					cfg_data.cfg.scenemod);
		break;
#endif
/* } FIH, Charles Huang, 2009/11/05 */
	default:
		rc = -EINVAL;
		break;
	}

	/* up(&ov5642_sem); */

	return rc;
}

int ov5642_sensor_release(void)
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
	/* down(&ov5642_sem); */
	int HWID=FIH_READ_HWID_FROM_SMEM();

	mutex_lock(&ov5642_mut);

	dev = ov5642_ctrl->sensordata;
	rc = gpio_request(dev->sensor_reset, "ov5642");
	if (!rc) {
		rc = gpio_direction_output(dev->sensor_reset, 0);
	}
	gpio_free(dev->sensor_reset);

	rc = gpio_request(dev->sensor_pwd, "ov5642");
	if (!rc) {
		rc = gpio_direction_output(dev->sensor_pwd, 1);
	}
	gpio_free(dev->sensor_pwd);

	if (HWID>=CMCS_7627_EVB1)
	{
		/* Switch disable */
		gpio_tlmm_config(GPIO_CFG(121, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_direction_output(121, 1);	
		msleep(10);
	}

       CDBG("[OV5642 5M]  gpio_get_value(0) = %d\n", gpio_get_value(0));
       CDBG("[OV5642 5M]  gpio_get_value(17) = %d\n", gpio_get_value(17));
       CDBG("[OV5642 5M]  gpio_get_value(31) = %d\n", gpio_get_value(31));
       CDBG("[OV5642 5M]  gpio_get_value(121) = %d\n", gpio_get_value(121));

       gpio_tlmm_config(GPIO_CFG(0, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
       gpio_tlmm_config(GPIO_CFG(17, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

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

	kfree(ov5642_ctrl);
	ov5642_ctrl = NULL;
	/* up(&ov5642_sem); */
	mutex_unlock(&ov5642_mut);
	return rc;
}

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
void ov5642_get_param(const char *buf, size_t count, struct ov5642_i2c_reg_conf *tbl, 
	unsigned short tbl_size, int *use_setting, int param_num)
{
	unsigned short waddr;
	unsigned short wdata;
	enum ov5642_width width;
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

static ssize_t ov5642_write_initreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov5642_get_param(buf, count, &ov5642_vfs_init_settings_tbl[0], ov5642_vfs_init_settings_tbl_size, &ov5642_use_vfs_init_setting, 3);
	return count;
}

static ssize_t ov5642_write_oemtreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov5642_get_param(buf, count, &ov5642_vfs_oem_settings_tbl[0], ov5642_vfs_oem_settings_tbl_size, &ov5642_use_vfs_oem_setting, 3);
	return count;
}

static ssize_t ov5642_write_previewreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov5642_get_param(buf, count, &ov5642_vfs_preview_settings_tbl[0], ov5642_vfs_preview_settings_tbl_size, &ov5642_use_vfs_preview_setting, 3);
	return count;
}

static ssize_t ov5642_write_snapreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov5642_get_param(buf, count, &ov5642_vfs_snap_settings_tbl[0], ov5642_vfs_snap_settings_tbl_size, &ov5642_use_vfs_snap_setting, 3);
	return count;
}

static ssize_t ov5642_write_snapaereg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov5642_get_param(buf, count, &ov5642_vfs_snapae_settings_tbl[0], ov5642_vfs_snapae_settings_tbl_size, &ov5642_use_vfs_snapae_setting, 3);
	return count;
}

static ssize_t ov5642_write_flashaereg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov5642_get_param(buf, count, &ov5642_vfs_flashae_settings_tbl[0], ov5642_vfs_flashae_settings_tbl_size, &ov5642_use_vfs_flashae_setting, 3);
	return count;
}

static ssize_t ov5642_write_iqreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov5642_get_param(buf, count, &ov5642_vfs_iq_settings_tbl[0], ov5642_vfs_iq_settings_tbl_size, &ov5642_use_vfs_iq_setting, 3);
	return count;
}

static ssize_t ov5642_write_lensreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov5642_get_param(buf, count, &ov5642_vfs_lens_settings_tbl[0], ov5642_vfs_lens_settings_tbl_size, &ov5642_use_vfs_lens_setting, 3);
	return count;
}

static ssize_t ov5642_write_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	long rc = 0;

	ov5642_get_param(buf, count, &ov5642_vfs_writereg_settings_tbl[0], ov5642_vfs_writereg_settings_tbl_size, &ov5642_use_vfs_writereg_setting, 3);
	if (ov5642_use_vfs_writereg_setting)
	{
		rc = ov5642_i2c_write_table(&ov5642_vfs_writereg_settings_tbl[0],
			ov5642_vfs_writereg_settings_tbl_size);
		ov5642_use_vfs_writereg_setting =0;
	}
	return count;
}

static ssize_t ov5642_setrange(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ov5642_get_param(buf, count, &ov5642_vfs_getreg_settings_tbl[0], ov5642_vfs_getreg_settings_tbl_size, &ov5642_use_vfs_getreg_setting, 3);
	return count;
}

static ssize_t ov5642_getrange(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i,rc;
	char *str = buf;

	if (ov5642_use_vfs_getreg_setting)
	{
		for (i=0;i<=ov5642_vfs_getreg_settings_tbl_size;i++)
		{
			if (ov5642_vfs_getreg_settings_tbl[i].waddr==0xFFFF)
				break;

			rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
				ov5642_vfs_getreg_settings_tbl[i].waddr, &(ov5642_vfs_getreg_settings_tbl[i].wdata), BYTE_LEN);
			CDBG("ov5642 reg 0x%4X = 0x%2X\n", ov5642_vfs_getreg_settings_tbl[i].waddr, ov5642_vfs_getreg_settings_tbl[i].wdata);

			str += sprintf(str, "%04X,%2X,%2X\n", ov5642_vfs_getreg_settings_tbl[i].waddr, 
				ov5642_vfs_getreg_settings_tbl[i].wdata, 
				ov5642_vfs_getreg_settings_tbl[i].mask);

			if (rc <0)
				break;
		}
	}
	return (str - buf);
}

static ssize_t ov5642_setmclk(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf,"%d",&ov5642_use_vfs_mclk_setting);
	return count;
}

static ssize_t ov5642_getmclk(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (sprintf(buf,"%d\n",ov5642_use_vfs_mclk_setting));
}

static ssize_t ov5642_setmultiple(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf,"%d",&ov5642_use_vfs_multiple_setting);
	return count;
}

static ssize_t ov5642_getmultiple(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (sprintf(buf,"%d\n",ov5642_use_vfs_multiple_setting));
}

static ssize_t ov5642_setflashtime(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf,"%d",&ov5642_use_vfs_flashtime_setting);
	return count;
}

static ssize_t ov5642_getflashtime(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (sprintf(buf,"%d\n",ov5642_use_vfs_flashtime_setting));
}

DEVICE_ATTR(initreg_5642, 0666, NULL, ov5642_write_initreg);
DEVICE_ATTR(oemreg_5642, 0666, NULL, ov5642_write_oemtreg);
DEVICE_ATTR(previewreg_5642, 0666, NULL, ov5642_write_previewreg);
DEVICE_ATTR(snapreg_5642, 0666, NULL, ov5642_write_snapreg);
DEVICE_ATTR(snapregae_5642, 0666, NULL, ov5642_write_snapaereg);
DEVICE_ATTR(flashregae_5642, 0666, NULL, ov5642_write_flashaereg);
DEVICE_ATTR(iqreg_5642, 0666, NULL, ov5642_write_iqreg);
DEVICE_ATTR(lensreg_5642, 0666, NULL, ov5642_write_lensreg);
DEVICE_ATTR(writereg_5642, 0666, NULL, ov5642_write_reg);
DEVICE_ATTR(getreg_5642, 0666, ov5642_getrange, ov5642_setrange);
DEVICE_ATTR(mclk_5642, 0666, ov5642_getmclk, ov5642_setmclk);
DEVICE_ATTR(multiple_5642, 0666, ov5642_getmultiple, ov5642_setmultiple);
DEVICE_ATTR(flashtime_5642, 0666, ov5642_getflashtime, ov5642_setflashtime);

#if 0
static struct kobject *android_ov5642 = NULL;
static int ov5642_sysfs_init(void)
{
	int ret ;
	android_ov5642 = kobject_create_and_add("android_camera", NULL);
	if (android_ov5642 == NULL) {
		ret = -ENOMEM;
		return ret ;
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_initreg_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_oemreg_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_previewreg_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_snapreg_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_snapregae_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_iqreg_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_lensreg_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_writereg_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_getreg_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_mclk_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_multiple_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	ret = sysfs_create_file(android_ov5642, &dev_attr_flashtime_5642.attr);
	if (ret) {
		kobject_del(android_ov5642);
	}

	return 0 ;
}

static int ov5642_sysfs_remove(void)
{
	sysfs_remove_file (android_ov5642, &dev_attr_initreg_5642.attr);
	sysfs_remove_file(android_ov5642, &dev_attr_oemreg_5642.attr);
	sysfs_remove_file(android_ov5642, &dev_attr_previewreg_5642.attr);
	sysfs_remove_file(android_ov5642, &dev_attr_snapreg_5642.attr);
	sysfs_remove_file(android_ov5642, &dev_attr_snapregae_5642.attr);
	sysfs_remove_file(android_ov5642, &dev_attr_iqreg_5642.attr);
	sysfs_remove_file(android_ov5642, &dev_attr_lensreg_5642.attr);
	sysfs_remove_file(android_ov5642, &dev_attr_writereg_5642.attr);
	sysfs_remove_file(android_ov5642, &dev_attr_getreg_5642.attr);
	sysfs_remove_file(android_ov5642, &dev_attr_mclk_5642.attr);
	sysfs_remove_file(android_ov5642, &dev_attr_multiple_5642.attr);
	sysfs_remove_file(android_ov5642, &dev_attr_flashtime_5642.attr);

	return 0 ;
}

#endif

static int create_attributes(struct i2c_client *client)
{
	int rc;

	dev_attr_initreg_5642.attr.name = OV5642_INITREG;
	dev_attr_oemreg_5642.attr.name = OV5642_OEMREG;
	dev_attr_previewreg_5642.attr.name = OV5642_PREVIEWREG;
	dev_attr_snapreg_5642.attr.name = OV5642_SNAPREG;
	dev_attr_snapregae_5642.attr.name = OV5642_SNAPAEREG;
	dev_attr_flashregae_5642.attr.name = OV5642_FLASHAEREG;
	dev_attr_iqreg_5642.attr.name = OV5642_IQREG;
	dev_attr_lensreg_5642.attr.name = OV5642_LENSREG;
	dev_attr_writereg_5642.attr.name = OV5642_WRITEREG;
	dev_attr_getreg_5642.attr.name = OV5642_GETREG;
	dev_attr_mclk_5642.attr.name = OV5642_MCLK;
	dev_attr_multiple_5642.attr.name = OV5642_MULTIPLE;
	dev_attr_flashtime_5642.attr.name = OV5642_FLASHTIME;
	
#if 0
	rc = ov5642_sysfs_init();
#else
	rc = device_create_file(&client->dev, &dev_attr_initreg_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"initreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}

	rc = device_create_file(&client->dev, &dev_attr_oemreg_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"oemreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}

	rc = device_create_file(&client->dev, &dev_attr_previewreg_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"previewreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}

	rc = device_create_file(&client->dev, &dev_attr_snapreg_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"snapreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_snapregae_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"snapregae\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_flashregae_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"flashregae\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_iqreg_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"iqreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_lensreg_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"lensreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_writereg_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"writereg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_getreg_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"getreg\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_mclk_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"mclk\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_multiple_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"multiple\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	

	rc = device_create_file(&client->dev, &dev_attr_flashtime_5642);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create ov5642 attribute \"flashtime\" failed!! <%d>", __func__, rc);
		
		return rc; 
	}	
#endif

	return rc;
}

static int remove_attributes(struct i2c_client *client)
{
#if 0
	ov5642_sysfs_remove();
	kobject_del(android_ov5642);
#else
	device_remove_file(&client->dev, &dev_attr_initreg_5642);
	device_remove_file(&client->dev, &dev_attr_oemreg_5642);
	device_remove_file(&client->dev, &dev_attr_previewreg_5642);
	device_remove_file(&client->dev, &dev_attr_snapreg_5642);
	device_remove_file(&client->dev, &dev_attr_snapregae_5642);
	device_remove_file(&client->dev, &dev_attr_flashregae_5642);
	device_remove_file(&client->dev, &dev_attr_iqreg_5642);
	device_remove_file(&client->dev, &dev_attr_lensreg_5642);
	device_remove_file(&client->dev, &dev_attr_writereg_5642);
	device_remove_file(&client->dev, &dev_attr_getreg_5642);
	device_remove_file(&client->dev, &dev_attr_mclk_5642);
	device_remove_file(&client->dev, &dev_attr_multiple_5642);
	device_remove_file(&client->dev, &dev_attr_flashtime_5642);
#endif

	return 0;
}
#endif /* OV5642_USE_VFS */
#endif
/* } FIH, Charles Huang, 2009/06/24 */

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
static enum hrtimer_restart ov5642_flashled_timer_func(struct hrtimer *timer)
{
	//static struct mpp *mpp_19;

	//mpp_19 = mpp_get(NULL, "mpp19");

	mpp_config_digital_out(18,MPP_CFG(MPP_DLOGIC_LVL_MSMP,
		MPP_DLOGIC_OUT_CTRL_HIGH));

	complete(&ov5642_flashled_comp);
	return HRTIMER_NORESTART;
}
#endif
/* } FIH, Charles Huang, 2009/09/01 */

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
static int ov5642_flashled_off_thread(void *arg)
{
	int ret = 0;
	//static struct mpp *mpp_19;


	daemonize("ov5642_flashled_off_thread");

	while (1) {
		wait_for_completion(&ov5642_flashled_comp);
		/* wait for flash on and turn off mpp */
		//msleep(400);
		msleep(380/ov5642_ledtime);
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

static int ov5642_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	ov5642_sensorw =
		kzalloc(sizeof(struct ov5642_work), GFP_KERNEL);

	if (!ov5642_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov5642_sensorw);
	ov5642_init_client(client);
	ov5642_client = client;
	
	CDBG("ov5642_probe successed!\n");

	return 0;

probe_failure:
	kfree(ov5642_sensorw);
	ov5642_sensorw = NULL;
	CDBG("ov5642_probe failed!\n");
	return rc;
}

static int __exit ov5642_i2c_remove(struct i2c_client *client)
{
	struct ov5642_work *sensorw = i2c_get_clientdata(client);

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
	remove_attributes(client);
#endif /* OV5642_USE_VFS */
#endif
/* } FIH, Charles Huang, 2009/06/24 */

	free_irq(client->irq, sensorw);
	ov5642_client = NULL;
	ov5642_sensorw = NULL;
	kfree(sensorw);
	return 0;
}

#ifdef CONFIG_PM
static int ov5642_suspend(struct i2c_client *client, pm_message_t mesg)
{
/* FIH, Charles Huang, 2009/06/25 { */
/* [FXX_CR], suspend/resume for pm */
#ifdef CONFIG_FIH_FXX
	/* sensor_pwd pin gpio31 */
	if (ov5642_ctrl)
	{
		gpio_direction_output(31,1);
	}
#endif
/* } FIH, Charles Huang, 2009/06/25 */
	return 0;
}

static int ov5642_resume(struct i2c_client *client)
{
/* FIH, Charles Huang, 2009/06/25 { */
/* [FXX_CR], suspend/resume for pm */
#ifdef CONFIG_FIH_FXX
	/* sensor_pwd pin gpio31 */
	/* Handle by sensor initialization */
	/* workable setting for waste power while resuming */
	if (ov5642_ctrl)
	{
		gpio_direction_output(31,0);
	}
#endif
/* } FIH, Charles Huang, 2009/06/25 */
	return 0;
}
#else
# define ov5642_suspend NULL
# define ov5642_resume  NULL
#endif

static const struct i2c_device_id ov5642_i2c_id[] = {
	{ "ov5642", 0},
	{ },
};

static struct i2c_driver ov5642_i2c_driver = {
	.id_table = ov5642_i2c_id,
	.probe  = ov5642_i2c_probe,
	.remove = __exit_p(ov5642_i2c_remove),
	.suspend  	= ov5642_suspend,
	.resume   	= ov5642_resume,
	.driver = {
		.name = "ov5642",
	},
};

static int ov5642_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = i2c_add_driver(&ov5642_i2c_driver);
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

	if (rc < 0 || ov5642_client == NULL) {
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

	rc = ov5642_reset(info);
	if (rc < 0) {
		CDBG("reset failed!\n");
		goto probe_fail;
	}

	/* EVB CAMIF cannont handle in 24MHz */
	/* EVB use 12.8MHz */
/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
	if (ov5642_use_vfs_mclk_setting!=0)
		msm_camio_clk_rate_set(ov5642_use_vfs_mclk_setting);
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
	/* Read REG_OV5642_MODEL_ID_HI & REG_OV5642_MODEL_ID_LO */
	rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
		REG_OV5642_MODEL_ID_HI, &model_id, WORD_LEN);
	if (rc < 0)
		goto probe_fail;

	CDBG("ov5642 model_id = 0x%x\n", model_id);

	/* Check if it matches it with the value in Datasheet */
	if (model_id != OV5642_MODEL_ID) {
		rc = -EFAULT;
		goto probe_fail;
	}

	/* Get version */
	rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
	0x302A, &model_id, BYTE_LEN);
	CDBG("ov5642 version reg 0x302A = 0x%x\n", model_id);

	/* Get OTP */
	rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
	0x3000, 0x00, BYTE_LEN);

	rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
	0x3004, 0xFF, BYTE_LEN);

	rc = ov5642_i2c_write(OV5642_I2C_WRITE_SLAVE_ID,
	0x3D10, 0x01, BYTE_LEN);

	/* Lens ID */
	rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
	0x3D06, &model_id, BYTE_LEN);
	msleep(1);
	CDBG("ov5642 version reg 0x3D06 = 0x%x\n", model_id);

	/* Module ID */
	rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
	0x3D07, &model_id, BYTE_LEN);
	msleep(1);
	CDBG("ov5642 version reg 0x3D07 = 0x%x\n", model_id);

	/* Check if it matches it with the value in FIH C265 OTP */
	if (model_id != 0x0)
	{
		if (model_id != 0x41) {
			rc = -EFAULT;
			goto probe_fail;
		}
	}

	/* time of making this module */
	rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
	0x3D08, &model_id, BYTE_LEN);
	CDBG("ov5642 version reg 0x3D08 = 0x%x\n", model_id);

	rc = ov5642_i2c_read(OV5642_I2C_READ_SLAVE_ID,
	0x3D09, &model_id, BYTE_LEN);
	CDBG("ov5642 version reg 0x3D09 = 0x%x\n", model_id);

	rc = gpio_request(info->sensor_reset, "ov5642");
	if (!rc) {
		rc = gpio_direction_output(info->sensor_reset, 0);
	}
	gpio_free(info->sensor_reset);

	rc = gpio_request(info->sensor_pwd, "ov5642");
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
		gpio_tlmm_config(GPIO_CFG(121, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_direction_output(121,1);
	}
#endif
/* } FIH, Charles Huang, 2009/06/09 */
       CDBG("[OV5642 5M]  gpio_get_value(0) = %d\n", gpio_get_value(0));
       CDBG("[OV5642 5M]  gpio_get_value(17) = %d\n", gpio_get_value(17));
       CDBG("[OV5642 5M]  gpio_get_value(31) = %d\n", gpio_get_value(31));
       CDBG("[OV5642 5M]  gpio_get_value(121) = %d\n", gpio_get_value(121));

/* FIH, Charles Huang, 2009/06/24 { */
/* [FXX_CR], add VFS */
#ifdef CONFIG_FIH_FXX
#ifdef OV5642_USE_VFS
	rc = create_attributes(ov5642_client);
	if (rc < 0) {
		dev_err(&ov5642_client->dev, "%s: create attributes failed!! <%d>", __func__, rc);
		goto probe_done;
	}
#endif /* OV5642_USE_VFS */
#endif
/* } FIH, Charles Huang, 2009/06/24 */

/* FIH, Charles Huang, 2009/09/01 { */
/* [FXX_CR], flashlight function  */
#ifdef CONFIG_FIH_FXX
	hrtimer_init(&ov5642_flashled_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ov5642_flashled_timer.function = ov5642_flashled_timer_func;
	ov5642_thread_id = kernel_thread(ov5642_flashled_off_thread, NULL, CLONE_FS | CLONE_FILES);
#endif
/* } FIH, Charles Huang, 2009/09/01 */

	s->s_init = ov5642_sensor_init;
	s->s_release = ov5642_sensor_release;
	s->s_config  = ov5642_sensor_config;

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

	dev_info(&ov5642_client->dev, "probe_done %s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;

probe_fail:
	dev_info(&ov5642_client->dev, "probe_fail %s %s:%d\n", __FILE__, __func__, __LINE__);
	gpio_tlmm_config(GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_direction_output(0,0);
	gpio_tlmm_config(GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_direction_output(31,1);

	if (HWID>=CMCS_7627_EVB1)
	{
		/* Switch disable */
		gpio_tlmm_config(GPIO_CFG(121, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_direction_output(121,1);
	}
       CDBG("[OV5642 5M]  gpio_get_value(0) = %d\n", gpio_get_value(0));
       CDBG("[OV5642 5M]  gpio_get_value(17) = %d\n", gpio_get_value(17));
       CDBG("[OV5642 5M]  gpio_get_value(31) = %d\n", gpio_get_value(31));
       CDBG("[OV5642 5M]  gpio_get_value(121) = %d\n", gpio_get_value(121));

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

static int __ov5642_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, ov5642_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov5642_probe,
	.driver = {
		.name = "msm_camera_ov5642",
		.owner = THIS_MODULE,
	},
};

static int __init ov5642_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov5642_init);
