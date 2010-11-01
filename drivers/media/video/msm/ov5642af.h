/*
 *     ov5642af.c - Camera Sensor Driver
 *
 *     Copyright (C) 2009 Charles YS Huang <charlesyshuang@fihtdc.com>
 *     Copyright (C) 2008 FIH CO., Inc.
 *
 *     This program is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; version 2 of the License.
 */

#ifndef OV5642AF_H
#define OV5642AF_H
#include <linux/types.h>
#include <mach/camera.h>

extern struct ov5642af_reg ov5642af_regs;

enum ov5642af_width {
	WORD_LEN,
	BYTE_LEN
};

struct ov5642af_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
	enum ov5642af_width width;
	unsigned short mdelay_time;
	unsigned short mask;
};


struct ov5642af_reg {
	struct ov5642af_i2c_reg_conf const *init_settings_tbl;
	uint16_t init_settings_tbl_size;
	struct ov5642af_i2c_reg_conf const *preview_settings_tbl;
	uint16_t preview_settings_tbl_size;
	struct ov5642af_i2c_reg_conf const *snapshot_settings_tbl;
	uint16_t snapshot_settings_tbl_size;
	struct ov5642af_i2c_reg_conf const *noise_reduction_reg_settings_tbl;
	uint16_t noise_reduction_reg_settings_tbl_size;
	struct ov5642af_i2c_reg_conf const *plltbl;
	uint16_t plltbl_size;
	struct ov5642af_i2c_reg_conf const *stbl;
	uint16_t stbl_size;
	struct ov5642af_i2c_reg_conf const *rftbl;
	uint16_t rftbl_size;
	struct ov5642af_i2c_reg_conf const *oem_settings_tbl;
	uint16_t oem_settings_tbl_size;
	struct ov5642af_i2c_reg_conf const *iq_settings_tbl;
	uint16_t iq_settings_tbl_size;
	struct ov5642af_i2c_reg_conf const *gseolens_settings_tbl;
	uint16_t gseolens_settings_tbl_size;
	struct ov5642af_i2c_reg_conf const *grdlens_settings_tbl;
	uint16_t grdlens_settings_tbl_size;
	struct ov5642af_i2c_reg_conf const *af_init_settings_tbl;
	uint16_t af_init_settings_tbl_size;
};

#endif /* OV5642AF_H */
