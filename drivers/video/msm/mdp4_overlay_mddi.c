/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>

#include <linux/fb.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"

static struct mdp4_overlay_pipe mddi_pipe;

static void mdp4_dmap_cfg(struct msm_fb_data_type *mfd)
{
	uint32	dma2_cfg_reg;

	dma2_cfg_reg = DMA_PACK_TIGHT | DMA_DITHER_EN;

	if (mfd->fb_imgType == MDP_BGR_565)
		dma2_cfg_reg |= DMA_PACK_PATTERN_BGR;
	else
		dma2_cfg_reg |= DMA_PACK_PATTERN_RGB;

	if (mfd->panel_info.bpp == 18) {
		dma2_cfg_reg |= DMA_DSTC0G_6BITS |	/* 666 18BPP */
		    DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;
	} else {
		dma2_cfg_reg |= DMA_DSTC0G_6BITS |	/* 565 16BPP */
		    DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
	}

	/* dma2 config register */
	MDP_OUTP(MDP_BASE + 0x90000, dma2_cfg_reg);

}

static void mdp4_dmap_xy(struct mdp4_overlay_pipe *pipe)
{

	/* dma_p source */
	MDP_OUTP(MDP_BASE + 0x90004,
			(pipe->src_height << 16 | pipe->src_width));
	MDP_OUTP(MDP_BASE + 0x90008, pipe->src_addr[0]);
	MDP_OUTP(MDP_BASE + 0x9000c, pipe->y_stride[0]);

	/* dma_p dest */
	MDP_OUTP(MDP_BASE + 0x90010, (pipe->dst_y << 16 | pipe->dst_x));
}

void mdp4_overlay_update_lcd(struct msm_fb_data_type *mfd)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	uint8 *src;
	int bpp;
	uint32 format;
	uint32 mddi_ld_param;
	uint16 mddi_vdo_packet_reg;
	struct mdp4_overlay_pipe *pipe;

	if (mfd->key != MFD_KEY)
		return;

	bpp = iBuf->bpp;

	if (bpp == 2)
		format = MDP_RGB_565;
	else if (bpp == 3)
		format = MDP_RGB_888;
	else
		format = MDP_ARGB_8888;

	pipe = &mddi_pipe;
	memset(pipe, 0, sizeof(*pipe));

	mdp4_overlay_format_to_pipe(format, pipe);

	src = (uint8 *) iBuf->buf;
	/* starting input address */
	src += (iBuf->dma_x + iBuf->dma_y * iBuf->ibuf_width) * bpp;


	/* use RGB1 pipe */
	pipe->pipe_type = OVERLAY_TYPE_RGB;
	pipe->pipe_num = OVERLAY_PIPE_RGB1;
	pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;
	pipe->mixer_num  = MDP4_MIXER0;
	pipe->src_height = iBuf->dma_h;
	pipe->src_width = iBuf->dma_w;
	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->dst_height = iBuf->dma_h;
	pipe->dst_width = iBuf->dma_w;
	pipe->dst_y = iBuf->dma_y;
	pipe->dst_x = iBuf->dma_x;
	pipe->src_addr[0] = (uint32) src;
	pipe->y_stride[0] = iBuf->ibuf_width * bpp;

	mddi_ld_param = 0;
	mddi_vdo_packet_reg = mfd->panel_info.mddi.vdopkt;

	if (mfd->panel_info.type == MDDI_PANEL) {
		if (mfd->panel_info.pdest == DISPLAY_1)
			mddi_ld_param = 0;
		else
			mddi_ld_param = 1;
	} else {
		mddi_ld_param = 2;
	}

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	MDP_OUTP(MDP_BASE + 0x00090, mddi_ld_param);
	MDP_OUTP(MDP_BASE + 0x00094,
		 (MDDI_VDO_PACKET_DESC << 16) | mddi_vdo_packet_reg);

	mdp4_dmap_xy(pipe);
	mdp4_dmap_cfg(mfd);

	mdp4_overlay_rgb_setup(pipe);

	mdp4_mixer_stage_setup(pipe);

	mdp4_overlayproc_cfg(pipe);

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
}

void mdp4_mddi_overlay(struct msm_fb_data_type *mfd)
{
	down(&mfd->dma->mutex);
	if ((mfd) && (!mfd->dma->busy) && (mfd->panel_power_on)) {
		down(&mfd->sem);
		mfd->ibuf_flushed = TRUE;
		mdp4_overlay_update_lcd(mfd);

		mdp_enable_irq(MDP_OVERLAY0_TERM);
		mfd->dma->busy = TRUE;
		INIT_COMPLETION(mfd->dma->comp);

		/* schedule DMA to start */
		mdp_pipe_kickoff(MDP_OVERLAY0_TERM, mfd);
		up(&mfd->sem);

		/* wait until DMA finishes the current job */
		wait_for_completion_killable(&mfd->dma->comp);
		mdp_disable_irq(MDP_OVERLAY0_TERM);

	/* signal if pan function is waiting for the update completion */
		if (mfd->pan_waiting) {
			mfd->pan_waiting = FALSE;
			complete(&mfd->pan_comp);
		}
	}
	up(&mfd->dma->mutex);
}
