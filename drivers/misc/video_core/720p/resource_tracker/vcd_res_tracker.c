/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 *
 */

#include "video_core_type.h"
#include "vcd_res_tracker.h"

static u32 Res_trk_convert_freq_to_perf_lvl(u64 n_freq)
{
	u64 n_perf_lvl;
	if (0 == n_freq)
		return 0;
	n_perf_lvl = n_freq;
	return (u32) n_perf_lvl;
}

static u32 Res_trk_convert_perf_lvl_to_freq(u64 n_perf_lvl)
{
	u64 n_freq;
	n_freq = n_perf_lvl;
	return (u32) n_freq;
}

u32 res_trk_power_up(void)
{
	VCDRES_MSG_LOW("clk_regime_rail_enable");
	VCDRES_MSG_LOW("clk_regime_sel_rail_control");
	return TRUE;
}

u32 res_trk_power_down(void)
{
	VCDRES_MSG_LOW("clk_regime_rail_disable");
	return TRUE;
}

u32 res_trk_enable_clock(void)
{
	VCDRES_MSG_LOW("clk_regime_msm_enable");
	return TRUE;
}

u32 res_trk_disable_clock(void)
{
	VCDRES_MSG_LOW("clk_regime_msm_disable");
	return TRUE;
}
u32 res_trk_get_max_perf_level(void)
{
	return VCD_RESTRK_MAX_PERF_LEVEL;
}

u32 res_trk_set_perf_level(u32 n_req_perf_lvl, u32 *pn_set_perf_lvl)
{
	u32 n_freq;
	n_freq = Res_trk_convert_perf_lvl_to_freq((u64) n_req_perf_lvl);
	if (VCD_RESTRK_MAX_FREQ_POINT < n_freq)
		n_freq = VCD_RESTRK_MAX_FREQ_POINT;
	VCDRES_MSG_LOW("clk_regime_msm_sel_clk_freq_hz. freq=%d", n_freq);
	if (0 == n_freq) {
		*pn_set_perf_lvl = 0;
		return FALSE;
	} else {
		*pn_set_perf_lvl =
		    Res_trk_convert_freq_to_perf_lvl((u64) n_freq);
		return TRUE;
	}
}

u32 res_trk_get_curr_perf_level(void)
{
	u32 n_freq;
	VCDRES_MSG_LOW("clk_regime_msm_get_clk_freq_hz");
	n_freq = 108000;
	return Res_trk_convert_freq_to_perf_lvl((u64) n_freq);
}
