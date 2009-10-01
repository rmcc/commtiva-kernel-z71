/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <linux/io.h>
#include <linux/debugfs.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"

#define MDP4_RGB1_BASE 0x40000
#define MDP4_RGB2_BASE 0x50000

void mdp4_overlay_rgb_setup(struct mdp4_overlay_pipe *pipe)
{
	char *rgb_base;
	uint32 src_size, src_xy, dst_size, dst_xy;
	uint32 format, element, operation;

	if (pipe->pipe_num)
		rgb_base = MDP_BASE + MDP4_RGB2_BASE;
	else
		rgb_base = MDP_BASE + MDP4_RGB1_BASE;

	src_size = pipe->src_height & 0x07ff;	/* 11 bits */
	src_size <<= 16;
	src_size |= (pipe->src_width & 0x07ff);

	src_xy = pipe->src_y & 0x07ff;	/* 11 bits */
	src_xy <<= 16;
	src_xy |= (pipe->src_x & 0x07ff);

	dst_size = pipe->dst_height & 0x07ff;	/* 11 bits */
	dst_size <<= 16;
	dst_size |= (pipe->dst_width & 0x07ff);

	dst_xy = pipe->dst_y & 0x07ff;	/* 11 bits */
	dst_xy <<= 16;
	dst_xy |= (pipe->dst_x & 0x07ff);

	format = mdp4_overlay_format(pipe);
	element = mdp4_overlay_unpack_pattern(pipe);
	operation = mdp4_overlay_operation(pipe);

	outpdw(rgb_base + 0x0000, src_size);	/* MDP_RGB_SRC_SIZE */
	outpdw(rgb_base + 0x0004, src_xy);	/* MDP_RGB_SRC_XY */
	outpdw(rgb_base + 0x0008, dst_size);	/* MDP_RGB_DST_SIZE */
	outpdw(rgb_base + 0x000c, dst_xy);	/* MDP_RGB_DST_XY */

	outpdw(rgb_base + 0x0010, pipe->src_addr[0]);
	outpdw(rgb_base + 0x0040, pipe->y_stride[0]);

	outpdw(rgb_base + 0x0050, format);	/* MDP_RGB_SRC_FORMAT */
	outpdw(rgb_base + 0x0054, element);	/* MDP_RGB_SRC_UNPACK_PATTERN */
	outpdw(rgb_base + 0x0058, operation);/* MDP_RGB_OP_MODE */

	/* 16 bytes-burst x 3 req <= 48 bytes */
	outpdw(rgb_base + 0x1004, 0xc2);	/* MDP_RGB_FETCH_CFG */
}

#define C3_ALPHA	3	/* alpha */
#define C2_R_Cr		2	/* R/Cr */
#define C1_B_Cb		1	/* B/Cb */
#define C0_G_Y		0	/* G/luma */

void mdp4_overlay_format_to_pipe(uint32 format, struct mdp4_overlay_pipe *pipe)
{
	pipe->pipe_num = 0;	/* temporary */
	pipe->mixer_stage = 0;	/* temporary */

	switch (format) {
	case MDP_RGB_565:
		pipe->pipe_type = OVERLAY_TYPE_RGB;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 0;
		pipe->r_bit = 1;	/* R, 5 bits */
		pipe->b_bit = 1;	/* B, 5 bits */
		pipe->g_bit = 2;	/* G, 6 bits */
		pipe->alpha_enable = 0;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 2;
		pipe->element2 = C2_R_Cr;	/* R */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C1_B_Cb;	/* B */
		pipe->bpp = 1;	/* 2 bpp */
		break;
	case MDP_RGB_888:
		pipe->pipe_type = OVERLAY_TYPE_RGB;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 0;
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 0;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 2;
		pipe->element2 = C2_R_Cr;	/* R */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C1_B_Cb;	/* B */
		pipe->bpp = 2;	/* 3 bpp */
		break;
	case MDP_BGR_565:
		pipe->pipe_type = OVERLAY_TYPE_RGB;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 0;
		pipe->r_bit = 1;	/* R, 5 bits */
		pipe->b_bit = 1;	/* B, 5 bits */
		pipe->g_bit = 2;	/* G, 6 bits */
		pipe->alpha_enable = 0;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 2;
		pipe->element2 = C1_B_Cb;	/* B */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C2_R_Cr;	/* R */
		pipe->bpp = 1;	/* 2 bpp */
		break;
	case MDP_ARGB_8888:
		pipe->pipe_type = OVERLAY_TYPE_RGB;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 3;	/* alpha, 4 bits */
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 1;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 3;
		pipe->element3 = C3_ALPHA;	/* alpha */
		pipe->element2 = C2_R_Cr;	/* R */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C1_B_Cb;	/* B */
		pipe->bpp = 3;		/* 4 bpp */
		break;
	case MDP_RGBA_8888:
		pipe->pipe_type = OVERLAY_TYPE_RGB;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 3;	/* alpha, 4 bits */
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 1;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 3;
		pipe->element3 = C2_R_Cr;	/* R */
		pipe->element2 = C0_G_Y;	/* G */
		pipe->element1 = C1_B_Cb;	/* B */
		pipe->element0 = C3_ALPHA;	/* alpha */
		pipe->bpp = 3;		/* 4 bpp */
		break;
	case MDP_BGRA_8888:
		pipe->pipe_type = OVERLAY_TYPE_RGB;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 3;	/* alpha, 4 bits */
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 1;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 3;
		pipe->element3 = C1_B_Cb;	/* B */
		pipe->element2 = C0_G_Y;	/* G */
		pipe->element1 = C2_R_Cr;	/* R */
		pipe->element0 = C3_ALPHA;	/* alpha */
		pipe->bpp = 3;		/* 4 bpp */
		break;
	default:
		break;
	}
}

uint32 mdp4_overlay_format(struct mdp4_overlay_pipe *pipe)
{
	uint32	format;

	format = 0;
	if (pipe->solid_fill)
		format |= MDP4_FORMAT_SOLID_FILL;

	if (pipe->unpack_align_msb)
		format |= MDP4_FORMAT_UNPACK_ALIGN_MSB;

	if (pipe->unpack_tight)
		format |= MDP4_FORMAT_UNPACK_TIGHT;

	if (pipe->alpha_enable)
		format |= MDP4_FORMAT_ALPHA_ENABLE;

	format |= (pipe->unpack_count << 13);
	format |= (pipe->bpp << 9);
	format |= (pipe->a_bit << 6);
	format |= (pipe->r_bit << 4);
	format |= (pipe->b_bit << 2);
	format |= pipe->g_bit;

	if (pipe->fetch_plane == OVERLAY_PLANE_PSEUDO_PLANAR) {
		/* video/graphic */
		format |= (pipe->frame_format << 29);
		format |= (pipe->chroma_site << 28);
		format |= (pipe->chroma_sample << 26);
	}

	return format;
}

uint32 mdp4_overlay_unpack_pattern(struct mdp4_overlay_pipe *pipe)
{
	return (pipe->element3 << 24) | (pipe->element2 << 16) |
			(pipe->element1 << 8) | pipe->element0;
}
uint32 mdp4_overlay_operation(struct mdp4_overlay_pipe *pipe)
{
	uint32 operation = 0;

	if (pipe->flags & MDP4_OP_FLIP_UD)
		operation |= (1 << 14);

	if (pipe->flags & MDP4_OP_FLIP_LR)
		operation |= (1 << 13);

	if (pipe->flags & MDP4_OP_IGC_LUT_EN)
		operation |= (1 << 16);

	if (pipe->flags & MDP4_OP_SCALEY_EN)
		operation |= (1 << 1);

	if (pipe->flags & MDP4_OP_SCALEX_EN)
		operation |= (1 << 0);

	return operation;
}

#define MDP4_OVERLAYPROC0_BASE	0x10000
#define MDP4_OVERLAYPROC1_BASE	0x18000

void mdp4_overlayproc_cfg(struct mdp4_overlay_pipe *pipe)
{
	uint32 data;
	char *overlay_base;

	if (pipe->mixer_num) 	/* mixer number, /dev/fb0, /dev/fb1 */
		overlay_base = MDP_BASE + MDP4_OVERLAYPROC1_BASE;/* 0x18000 */
	else
		overlay_base = MDP_BASE + MDP4_OVERLAYPROC0_BASE;/* 0x10000 */

	/* MDP_OVERLAYPROC_CFG */
	outpdw(overlay_base + 0x0004, 0x01); /* directout */
	data = pipe->src_height;
	data <<= 16;
	data |= pipe->src_width;
	outpdw(overlay_base + 0x0008, data); /* ROI, height + width */
	outpdw(overlay_base + 0x000c, pipe->src_addr[0]);
	outpdw(overlay_base + 0x0010, pipe->y_stride[0]);
	outpdw(overlay_base + 0x0014, 0x01);	/* 565 */
}

void mdp4_mixer_stage_setup(struct mdp4_overlay_pipe *pipe)
{
	uint32 data, mask, snum, stage = 0;

	/* MDP_LAYERMIXER_IN_CFG, shard by both mixer 0 and 1  */
	data = inpdw(MDP_BASE + 0x10100);

	stage = pipe->mixer_stage;		/* mixer 0 */
	if (pipe->mixer_num)	 /* mixer 1 */
		stage += 8;

	if (pipe->pipe_type == OVERLAY_TYPE_VG) {/* VG1 and VG2 */
		snum = 0;
		snum += (4 * pipe->pipe_num);
	} else {
		snum = 8;
		snum += (4 * pipe->pipe_num);	/* RGB1 and RGB2 */
	}

	mask = 0x0f;
	mask <<= snum;
	stage <<= snum;

	data &= ~mask;	/* clear old bits */
	data |= stage;

	outpdw(MDP_BASE + 0x10100, data); /* MDP_LAYERMIXER_IN_CFG */
}

#ifdef MDP4_BLEND
void mdp4_mixer_blend_setup(struct mdp_rgb_req *req,
			struct mdp_overlay_pipe *pipe)
{
	uint32 overlay_base;

	if (pipe->mixer_stage <= MDP4_MIXER_STAGE_BASE)
		return;

	if (pipe->mixer_num) 	/* mixer number, /dev/fb0, /dev/fb1 */
		overlay_base = MDP_BASE + MDP4_OVERLAYPROC1_BASE;/* 0x18000 */
	else
		overlay_base = MDP_BASE + MDP4_OVERLAYPROC0_BASE;/* 0x10000 */

	/* stage 0 to stage 2 */
	blend_off = 0x20 * (pipe->mixer_stage - 2);

	outpdw(overlaybase + blend_off + 0x104, req->blend_flags);
	outpdw(overlaybase + blend_off + 0x108, req->fg_alpha);
	outpdw(overlaybase + blend_off + 0x10c, req->bg_alpha);

	color = req->transp_low_color1 & 0x0fff;	/* 12 bits */
	color <<= 16;
	color |= (req->transp_low_color0 & 0x0fff);	/* 12 bits */
	outpdw(overlay_base + blend_off + 0x110, color);
	color = (req->transp_low_color2 & 0x0fff);	/* 12 bits */
	outpdw(overlay_base + blend_off + 0x114, color);

	color = req->transp_high_color1 & 0x0fff;	/* 12 bits */
	color <<= 16;
	color |= (req->transp_high_color0 & 0x0fff);	/* 12 bits */
	outpdw(overlay_base + blend_off + 0x118, color);
	color = (req->transp_high_color2 & 0x0fff);	/* 12 bits */
	outpdw(overlay_base + blend_off + 0x11c, color);
}
#endif

void mdp4_overlay_reg_flush(uint32 bits)
{
	outpdw(MDP_BASE + 0x18000, bits);	/* MDP_OVERLAY_REG_FLUSH */

	while (inpdw(MDP_BASE + 0x18000) & bits) /* self clear when complete */
		;
}
