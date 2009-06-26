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

#ifndef __ADSP_AUDIO_TYPES_H
#define __ADSP_AUDIO_TYPES_H


/* This structure allows client to match requests with corresponding */
/* ack events */

struct adsp_audio_client_data {
	u32	context;	/* Clients Context */
	u32	data;		/* Associated data */
} __attribute__ ((packed));


/* This structure allows for future expansion of information that */
/* the client needs but ADSP is not concerned about. This header is */
/* passed back to the client in the event corresponding to the command */
/* received.*/

struct adsp_audio_header {
	/* Client specified data */
	struct adsp_audio_client_data	client_data;
	/* Room for expansion */
} __attribute__ ((packed));


/* Opcode to open a device stream session to capture audio */

#define ADSP_AUDIO_OPEN_OP_READ		0x01

/* Opcode to open a device stream session to render audio */
#define ADSP_AUDIO_OPEN_OP_WRITE	0x02


/* Opcode to open a control session for device operations. Operations */
/* oncontrol session will affect all stream sessions assoiciated with */
/* the device. */

/* Use this operation to set/update common postproc settings, */
/* set/update calibration params for a device, enable/disable common*/
/* postprocs */

#define ADSP_AUDIO_OPEN_OP_DEVICE_CTRL	0x04

/* Some encoders need configuration information in addition to format */
/* block */

/* AAC Encoder modes */
#define ADSP_AUDIO_ENC_AAC_LC_ONLY_MODE		0
#define ADSP_AUDIO_ENC_AAC_PLUS_MODE		1
#define ADSP_AUDIO_ENC_ENHANCED_AAC_PLUS_MODE	2

/* AAC Encoder configuration */

struct adsp_audio_aac_enc_cfg {
	u32	bit_rate;	/* bits per second */
	u32	encoder_mode;	/* ADSP_AUDIO_ENC_* */
} __attribute__ ((packed));


/* AMR NB encoder modes */
#define ADSP_AUDIO_AMR_MR475	0
#define ADSP_AUDIO_AMR_MR515	1
#define ADSP_AUDIO_AMR_MMR59	2
#define ADSP_AUDIO_AMR_MMR67	3
#define ADSP_AUDIO_AMR_MMR74	4
#define ADSP_AUDIO_AMR_MMR795	5
#define ADSP_AUDIO_AMR_MMR102	6
#define ADSP_AUDIO_AMR_MMR122	7


/* The following are valid AMR NB DTX modes */
#define ADSP_AUDIO_AMR_DTX_MODE_OFF		0
#define ADSP_AUDIO_AMR_DTX_MODE_ON_VAD1		1
#define ADSP_AUDIO_AMR_DTX_MODE_ON_VAD2		2
#define ADSP_AUDIO_AMR_DTX_MODE_ON_AUTO		3


/* AMR Encoder configuration */

struct adsp_audio_amr_enc_cfg {
	u32	mode;		/* ADSP_AUDIO_AMR_MR* */
	u32	dtx_mode;	/* ADSP_AUDIO_AMR_DTX_MODE* */
	u32	enable;		/* 1 = enable, 0 = disable */
} __attribute__ ((packed));


struct adsp_audio_qcelp13k_enc_cfg {
	u16	min_rate;
	u16	max_rate;
} __attribute__ ((packed));


struct adsp_audio_evrc_enc_cfg {
	u16	min_rate;
	u16	max_rate;
} __attribute__ ((packed));


struct adsp_audio_codec_config {
	union {
		struct adsp_audio_amr_enc_cfg		amr_cfg;
		struct adsp_audio_aac_enc_cfg		aac_cfg;
		struct adsp_audio_qcelp13k_enc_cfg	qcelp13k_cfg;
		struct adsp_audio_evrc_enc_cfg		evrc_cfg;
	};
} __attribute__ ((packed));




/* Bit masks for adsp_audio_open_stream_device.mode */


/* This is the default value. */
#define  ADSP_AUDIO_OPEN_STREAM_MODE_NONE       0x0000



/* This bit, if set, indicates that the AVSync mode is activated. */

#define  ADSP_AUDIO_OPEN_STREAM_MODE_AVSYNC     0x0001


/* Data for ADSPaudio_DriverOpen operation.  */

/* The client specifies the media format block as defined in  */
/* ADSPaudio_MediaFormat.h for OPEN_OP_READ/WRITE */


#define ADSP_AUDIO_MAX_DEVICES 4

struct adsp_audio_open_stream_device {
	/* Number of devices specified */
	u32				num_devices;
	/* List of stream device IDs */
	u32				device[ADSP_AUDIO_MAX_DEVICES];
	/* Stream usage type */
	u32				stream_context;
	/* Media Format Code */
	u32				format;
	/* Media Format Block */
	void				*format_block;
	/* Media Format Block len */
	u32				format_block_len;
	/* Max buf size in bytes */
	u32				buf_max_size;
	/* Desired stream priority */
	u32				priority;
	/* Encoder configuration for READ op */
	struct adsp_audio_codec_config	config;
	/* 1- indicates AVSync playback mode */
	u32				mode;
} __attribute__ ((packed));



/* adsp_audio_OpenStreamDevice is not specified for OPEN_OP_DEVICE_CTRL */


struct adsp_audio_open_device {
	/* Associated client data */
	struct adsp_audio_header		header;
	/* Render, Capture or Control */
	u32					op_code;
	/* Open for READ/WRITE */
	struct adsp_audio_open_stream_device	stream_device;
} __attribute__ ((packed));



/* Data buffer definition for read/write operations */



/* This flag, if set, indicates that the beginning of the data in the*/
/* buffer is a synchronization point or key frame, meaning no data */
/* before it in the stream is required in order to render the stream */
/* from this point onward. */
#define ADSP_AUDIO_BUFFER_FLAG_SYNC_POINT        0x01


/* This flag, if set, indicates that the buffer object is using valid */
/* physical address used to store the media data */
#define ADSP_AUDIO_BUFFER_FLAG_PHYS_ADDR         0x04


/* This flag, if set, indicates that a media start timestamp has been */
/* set for a buffer. */
#define ADSP_AUDIO_BUFFER_FLAG_START_SET         0x08


/* This flag, if set, indicates that a media stop timestamp has been set */
/* for a buffer. */
#define ADSP_AUDIO_BUFFER_FLAG_STOP_SET          0x10


/* This flag, if set, indicates that a preroll timestamp has been set */
/* for a buffer. */
#define ADSP_AUDIO_BUFFER_FLAG_PREROLL_SET       0x20


/* This flag, if set, indicates that the data in the buffer is a fragment of */
/* a larger block of data, and will be continued by the data in the next */
/* buffer to be delivered. */
#define ADSP_AUDIO_BUFFER_FLAG_CONTINUATION      0x40


struct adsp_audio_data_buffer {
	u32	buffer_addr;	/* Physical Address of buffer */
	u32	max_size;	/* Maximum size of buffer */
	u32	actual_size;	/* Actual size of valid data in the buffer */
	u32	offset;		/* Offset to the first valid byte */
	u32	flags;		/* ADSP_AUDIO_BUFFER_FLAGs that has been set */
	s64	start;		/* Start timestamp, if any */
	s64	stop;		/* Stop timestamp, if any */
	s64	preroll;	/* Preroll timestamp, if any */
} __attribute__ ((packed));



struct adsp_audio_buffer {
	struct adsp_audio_header	header;	/* Associated client data */
	struct adsp_audio_data_buffer	buffer;	/* media data buffer */
} __attribute__ ((packed));


#endif
