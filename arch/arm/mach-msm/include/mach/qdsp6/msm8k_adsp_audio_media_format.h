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

#ifndef __ADSP_AUDIO_MEDIA_FORMAT_H
#define __ADSP_AUDIO_MEDIA_FORMAT_H



/* Supported audio media formats */

/* adsp_audio_format_raw_pcm type */
#define ADSP_AUDIO_FORMAT_PCM		0x0103d2fd

/* adsp_audio_format_raw_pcm type */
#define ADSP_AUDIO_FORMAT_DTMF		0x01087725

/* adsp_audio_format_adpcm type */
#define ADSP_AUDIO_FORMAT_ADPCM		0x0103d2ff

/* ISO/IEC 11172 */
#define ADSP_AUDIO_FORMAT_MP3		0x0103d308

/* ISO/IEC 14496 */
#define ADSP_AUDIO_FORMAT_MPEG4_AAC	0x010422f1

/* AMR-NB audio in FS format */
#define ADSP_AUDIO_FORMAT_AMRNB_FS	0x0105c16c

/* QCELP 13k, IS733 */
#define ADSP_AUDIO_FORMAT_V13K_FS	0x01080b8a

/* EVRC   8k, IS127 */
#define ADSP_AUDIO_FORMAT_EVRC_FS	0x01080b89

/* MIDI command stream */
#define ADSP_AUDIO_FORMAT_MIDI		0x0103d300



/* Not yet supported audio media formats */


/* Yamaha PCM format */
#define ADSP_AUDIO_FORMAT_YADPCM	0x01089bf8

/* ISO/IEC 13818 */
#define ADSP_AUDIO_FORMAT_MPEG2_AAC	0x0103d309

/* 3GPP TS 26.101 Sec 4.0 */
#define ADSP_AUDIO_FORMAT_AMRNB_IF1	0x0103d305

/* 3GPP TS 26.101 Annex A */
#define ADSP_AUDIO_FORMAT_AMRNB_IF2	0x01057b31

/* 3GPP TS 26.201 */
#define ADSP_AUDIO_FORMAT_AMRWB_IF1	0x0103d306

/* 3GPP TS 26.201 */
#define ADSP_AUDIO_FORMAT_AMRWB_IF2	0x0105c16d

/* AMR-WB audio in FS format */
#define ADSP_AUDIO_FORMAT_AMRWB_FS	0x0105c16e

/* G.711 */
#define ADSP_AUDIO_FORMAT_G711		0x0106201d

/* QCELP  8k, IS96A */
#define ADSP_AUDIO_FORMAT_V8K_FS	0x01081d29

/* Version 1 codec */
#define ADSP_AUDIO_FORMAT_WMA_V1	0x01055b2b

/* Version 2, 7 & 8 codec */
#define ADSP_AUDIO_FORMAT_WMA_V8	0x01055b2c

/* Version 9 Professional codec */
#define ADSP_AUDIO_FORMAT_WMA_V9PRO	0x01055b2d

/* Version 9 Voice codec */
#define ADSP_AUDIO_FORMAT_WMA_SP1	0x01055b2e

/* Version 9 Lossless codec */
#define ADSP_AUDIO_FORMAT_WMA_LOSSLESS	0x01055b2f

/* Real Media content, low-bitrate */
#define ADSP_AUDIO_FORMAT_RA_SIPR	0x01042a0f

/* Real Media content */
#define ADSP_AUDIO_FORMAT_RA_COOK	0x01042a0e


/* For all of the audio formats, unless specified otherwise, */
/* the following apply: */
/* Format block bits are arranged in bytes and words in little-endian */
/* order, i.e., least-significant bit first and least-significant */
/* byte first. */



/* AAC Format Block. */

/* AAC format block consist of a format identifier followed by */
/* AudioSpecificConfig formatted according to ISO/IEC 14496-3 */

/* The following AAC format identifiers are supported */
#define ADSP_AUDIO_AAC_ADTS		0x010619cf
#define ADSP_AUDIO_AAC_MPEG4_ADTS	0x010619d0
#define ADSP_AUDIO_AAC_LOAS		0x010619d1
#define ADSP_AUDIO_AAC_ADIF		0x010619d2
#define ADSP_AUDIO_AAC_RAW		0x010619d3
#define ADSP_AUDIO_AAC_FRAMED_RAW	0x0108c1fb


/* Raw PCM, DTMF format block */

struct adsp_audio_format_raw_pcm {
	u16		channels;
	u16		bits_per_sample;
	u32		sampling_rate;
	bool		is_signed;
	bool		is_interleaved;
} __attribute__ ((packed));



/* ADPCM format block */

struct adsp_audio_format_adpcm {
	struct adsp_audio_format_raw_pcm	base;
	u32					block_size;
} __attribute__ ((packed));


/* G711 format block */

#define ADSP_AUDIO_COMPANDING_ALAW	0x10619cd
#define ADSP_AUDIO_COMPANDING_MLAW	0x10619ce


struct adsp_audio_format_g711 {
	u32	companding;
} __attribute__ ((packed));



#endif


