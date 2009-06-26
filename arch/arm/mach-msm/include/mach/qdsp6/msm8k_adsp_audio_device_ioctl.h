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

#ifndef __ADSP_AUDIO_DEVICE_IOCTL_H
#define __ADSP_AUDIO_DEVICE_IOCTL_H


#include <mach/qdsp6/msm8k_adsp_audio_stream_ioctl.h>


/* Device control session only IOCTL command definitions */
/* These commands will affect a logical device and all its associated */
/* streams. */


/* Set device volume. */
/* This command has data payload struct adsp_audio_set_device_vol. */

#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_VOL		0x0107605c


struct adsp_audio_set_device_volume {
	/* Associated client data */
	struct adsp_audio_header	header;
	/* DeviceID for volume change */
	u32				device;
	/* 0 == Rx, 1 == Tx and 2 == both */
	u32				path;
	/* in mB. */
	s32				volume;
} __attribute__ ((packed));




/* Set device mute state. */
/* This command has data payload struct adsp_audio_set_device_mute. */

#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE		0x0107605f


struct adsp_audio_set_device_mute {
	/* Associated client data */
	struct adsp_audio_header	header;
	/* DeviceID for mute change */
	u32				device;
	/* 0 == Rx, 1 == Tx and 2 == both */
	u32				path;
	/* 0 == UnMute, 1 == Mute */
	u32				mute;
} __attribute__ ((packed));



/* Configure Equalizer for a device. */
/* This command has payload struct adsp_audio_device_eq_cfg. */

#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_EQ_CONFIG	0x0108b10e


struct adsp_audio_device_eq_cfg {
	/* Associated client data */
	struct adsp_audio_header	client_data;
	/* DeviceID for equalizer config */
	u32				device;
	/* Equalizer band data */
	struct adsp_audio_eq_cfg	eq_config;
} __attribute__ ((packed));


#endif


