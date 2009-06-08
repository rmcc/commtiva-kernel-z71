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

#ifndef __ADSP_AUDIO_DEVICE_H
#define __ADSP_AUDIO_DEVICE_H


#include <mach/qdsp6/msm8k_adsp_audio_types.h>


/* Device direction Rx/Tx flag */
#define ADSP_AUDIO_RX_DEVICE		0x00
#define ADSP_AUDIO_TX_DEVICE		0x01


/* Predefined Audio device ids. */


/* Default RX or TX device */
#define ADSP_AUDIO_DEVICE_ID_DEFAULT		0x1081679


/* Source (TX) devices */
#define ADSP_AUDIO_DEVICE_ID_HANDSET_MIC	0x107ac8d
#define ADSP_AUDIO_DEVICE_ID_HEADSET_MIC	0x1081510
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MIC	0x1081512
#define ADSP_AUDIO_DEVICE_ID_BT_SCO_MIC		0x1081518
#define ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_MIC	0x108151b
#define ADSP_AUDIO_DEVICE_ID_I2S_MIC		0x1089bf3


/* Special loopback pseudo device to be paired with an RX device */
/* with usage ADSP_AUDIO_DEVICE_USAGE_MIXED_PCM_LOOPBACK */
#define ADSP_AUDIO_DEVICE_ID_MIXED_PCM_LOOPBACK_TX	0x1089bf2


/* Sink (RX) devices */
#define ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR			0x107ac88
#define ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_MONO			0x1081511
#define ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_STEREO		0x107ac8a
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO			0x1081513
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_MONO_HEADSET     0x108c508
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_STEREO_HEADSET   0x108c894
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO			0x1081514
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_MONO_HEADSET   0x108c895
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_STEREO_HEADSET	0x108c509
#define ADSP_AUDIO_DEVICE_ID_BT_SCO_SPKR			0x1081519
#define ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_SPKR			0x108151c
#define ADSP_AUDIO_DEVICE_ID_I2S_SPKR				0x1089bf4


/* BT A2DP playback device. */
/* This device must be paired with */
/* ADSP_AUDIO_DEVICE_ID_MIXED_PCM_LOOPBACK_TX using  */
/* ADSP_AUDIO_DEVICE_USAGE_MIXED_PCM_LOOPBACK mode */
#define ADSP_AUDIO_DEVICE_ID_BT_A2DP_SPKR	0x108151a


/*  Audio device usage types. */
/*  This is a bit mask to determine which topology to use in the */
/* device session */
#define ADSP_AUDIO_DEVICE_CONTEXT_VOICE			0x01
#define ADSP_AUDIO_DEVICE_CONTEXT_PLAYBACK		0x02
#define ADSP_AUDIO_DEVICE_CONTEXT_RECORD		0x20
#define ADSP_AUDIO_DEVICE_CONTEXT_PCM_LOOPBACK		0x40

#endif

