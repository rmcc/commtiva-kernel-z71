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

#ifndef __ADSP_AUDIO_STREAM_IOCTL_H
#define __ADSP_AUDIO_STREAM_IOCTL_H


#include <mach/qdsp6/msm8k_adsp_audio_types.h>
#include <mach/qdsp6/msm8k_adsp_audio_device.h>


/* Stream only IOCTL command definitions. */
/* These commands will affect only a single stream. */



/* Start stream for audio device. */
/* This command has no payload.  */
/* struct adsp_audio_header can be sent as a payload if desired */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_START		0x010815c6


/* Stop stream for audio device. */
/* This command has no payload. */
/* struct adsp_audio_header can be sent as a payload if desired */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_STOP		0x01075c54


/* Pause the data flow for a stream. */
/* This command has no payload. */
/* struct adsp_audio_header can be sent as a payload if desired */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_PAUSE		0x01075ee8


/* Resume the data flow for a stream. */
/* This command has no payload. */
/* struct adsp_audio_header can be sent as a payload if desired */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_RESUME		0x01075ee9


/* Drop any unprocessed data buffers for a stream. */
/* This command has no payload. */
/* struct adsp_audio_header can be sent as a payload if desired */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_FLUSH		0x01075eea


/* End of stream reached. Client will not send any more data. */
/* This command has no payload. */
/* struct adsp_audio_header can be sent as a payload if desired */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_EOS			0x0108b150


/* Start Stream DTMF tone */
/* This command has payload struct adsp_audio_dtmf_start */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_DTMF_START		0x0108c0dd


struct adsp_audio_dtmf_start {
	/* Associated client data */
	struct adsp_audio_header	header;
	/* First tone in Hz */
	u32				tone1_hz;
	/* Second tone in Hz */
	u32				tone2_hz;
	/* Duration in microseconds */
	u32				duration_usec;
	/* Gain in millibels */
	s32				gain_mb;
} __attribute__ ((packed));



/* Stop Stream DTMF tone */
/* This command has no payload. */
/* struct adsp_audio_header can be sent as a payload if desired */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_DTMF_STOP		0x01087554


/* Set stream volume. */
/* This command has data payload, struct adsp_audio_set_stream_volume. */

#define ADSP_AUDIO_IOCTL_CMD_SET_STREAM_VOL		0x0108c0de


struct adsp_audio_set_stream_volume {
	struct adsp_audio_header	header;	/* Associated client data */
	s32				volume;	/* in mB */
} __attribute__ ((packed));



/* Set stream mute state. */
/* This command has data payload, struct adsp_audio_set_stream_mute. */

#define ADSP_AUDIO_IOCTL_CMD_SET_STREAM_MUTE		0x0108c0df


struct adsp_audio_set_stream_mute {
	struct adsp_audio_header	header;	/* Associated client data */
	u32				mute;	/* 0 == UnMute, 1 == Mute */
} __attribute__ ((packed));



/* Configure Equalizer for a stream. */
/* This command has payload struct adsp_audio_stream_eq_cfg. */

#define ADSP_AUDIO_IOCTL_CMD_SET_STREAM_EQ_CONFIG	0x0108c0e0



/* Equalizer filter band types */
#define ADSP_AUDIO_EQUALIZER_TYPE_NONE		0
#define ADSP_AUDIO_EQUALIZER_BASS_BOOST		1
#define ADSP_AUDIO_EQUALIZER_BASS_CUT		2
#define ADSP_AUDIO_EQUALIZER_TREBLE_BOOST	3
#define ADSP_AUDIO_EQUALIZER_TREBLE_CUT		4
#define ADSP_AUDIO_EQUALIZER_BAND_BOOST		5
#define ADSP_AUDIO_EQUALIZER_BAND_CUT		6


/* Definition for any one band of Equalizer. */

struct adsp_audio_eq_band {
	/* The band index, 0 .. 11 */
	u16	band_idx;
	/* Filter band type */
	u32	filter_type;
	/* Filter band center frequency */
	u32	center_freq_hz;
	/* Filter band initial gain (dB) */
	/* Range is +12 dB to -12 dB with 1dB increments. */
	s32	filter_gain;
	/* Filter band quality factor expressed as q-8 number, */
	/* i.e. fixed point number with q factor of 8, */
	/* e.g. 3000/(2^8) */
	s32	q_factor;
} __attribute__ ((packed));



#define ADSP_AUDIO_MAX_EQ_BANDS 12

#define ADSP_AUDIO_MAX_EQ_BANDS 12

#define CAD_EQ_INVALID_DATA       0xFFFFFFFF

struct adsp_audio_eq_cfg {
	u32				enable;
	/* Number of consequtive bands specified */
	u32				num_bands;
	struct adsp_audio_eq_band	eq_bands[ADSP_AUDIO_MAX_EQ_BANDS];
} __attribute__ ((packed));


struct adsp_audio_stream_eq_cfg {
	/* Associated client data */
	struct adsp_audio_header	header;
	/* Equalizer band data */
	struct adsp_audio_eq_cfg	ecfg;
} __attribute__ ((packed));


struct cad_filter_eq_driver_struct {
	/* this is the device control session */
	u32     device_session_id;
	/* indexed by session id */
	struct adsp_audio_stream_eq_cfg eq_stream_data[CAD_MAX_SESSION];
};


/* Set Audio Video sync information. */
/* This command has data payload, struct adsp_audio_stream_av_sync. */

#define ADSP_AUDIO_IOCTL_CMD_SET_STREAM_AV_SYNC		0x0107605c


struct adsp_audio_stream_av_sync {
	s64	relative_time;	/* Media time */
	s64	absolute_time;	/* Presentation time */
} __attribute__ ((packed));



/* Get Audio Media Session time. */
/* This command returns returns the s64 audioTime in adsp_audio_event_data */

#define ADSP_AUDIO_IOCTL_CMD_GET_AUDIO_TIME		0x0108c26c


#endif


