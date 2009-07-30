/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef MT9D112_H
#define MT9D112_H

#include <mach/board.h>
#include <mach/camera.h>

extern struct mt9d112_reg_t mt9d112_regs;

enum mt9d112_width_t {
	WORD_LEN,
	BYTE_LEN
};

struct mt9d112_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
	enum mt9d112_width_t width;
	unsigned short mdelay_time;
};


struct mt9d112_reg_t {

	struct register_address_value_pair_t const *prev_snap_reg_settings;
	uint16_t prev_snap_reg_settings_size;
	struct register_address_value_pair_t const
	  *noise_reduction_reg_settings;
	uint16_t noise_reduction_reg_settings_size;
	struct mt9d112_i2c_reg_conf const *plltbl;
	uint16_t plltbl_size;
	struct mt9d112_i2c_reg_conf const *stbl;
	uint16_t stbl_size;
	struct mt9d112_i2c_reg_conf const *rftbl;
	uint16_t rftbl_size;
};


#endif /* MT9D112_H */
