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

#ifndef CAD_AMR_FORMAT_H
#define CAD_AMR_FORMAT_H

#include <mach/qdsp6/msm8k_cad_format_comm.h>

/* cad AMR band mode */
enum cad_amr_band_mode {
	CAD_AMR_BM_UNUSED = 0,		/* AMRNB Mode unused / unknown */
	CAD_AMR_BM_NB0,			/* AMRNB Mode 0 =  4750 bps */
	CAD_AMR_BM_NB1,			/* AMRNB Mode 1 =  5150 bps */
	CAD_AMR_BM_NB2,			/* AMRNB Mode 2 =  5900 bps */
	CAD_AMR_BM_NB3,			/* AMRNB Mode 3 =  6700 bps */
	CAD_AMR_BM_NB4,			/* AMRNB Mode 4 =  7400 bps */
	CAD_AMR_BM_NB5,			/* AMRNB Mode 5 =  7950 bps */
	CAD_AMR_BM_NB6,			/* AMRNB Mode 6 = 10200 bps */
	CAD_AMR_BM_NB7,			/* AMRNB Mode 7 = 12200 bps */
	CAD_AMR_BM_WB0,			/* AMRWB Mode 0 =  6600 bps */
	CAD_AMR_BM_WB1,			/* AMRWB Mode 1 =  8850 bps */
	CAD_AMR_BM_WB2,			/* AMRWB Mode 2 = 12650 bps */
	CAD_AMR_BM_WB3,			/* AMRWB Mode 3 = 14250 bps */
	CAD_AMR_BM_WB4,			/* AMRWB Mode 4 = 15850 bps */
	CAD_AMR_BM_WB5,			/* AMRWB Mode 5 = 18250 bps */
	CAD_AMR_BM_WB6,			/* AMRWB Mode 6 = 19850 bps */
	CAD_AMR_BM_WB7,			/* AMRWB Mode 7 = 23050 bps */
	CAD_AMR_BM_WB8,			/* AMRWB Mode 8 = 23850 bps */
	CAD_AMR_BM_MAX = 0x7FFFFFFF
};


/* cad AMR DTX mode */
enum cad_amr_dtx_mode {
	CAD_AMR_DTX_OFF = 0,	/* DTX is disabled */
	CAD_AMR_DTX_VAD1,	/* Voice Activity Detector 1 is enabled */
	CAD_AMR_DTX_VAD2,	/* Voice Activity Detector 2 is enabled */
	CAD_AMR_DTX_AUTO,	/* The codec will automatically select */
	CAD_AMR_DTX_EFR,	/* DTX as EFR instead of AMR standard */
	CAD_AMR_DTX_MAX = 0x7FFFFFFF
};

/* cad AMR Frame format */
enum cad_amr_frame_format {
	/* AMR Conformance(Standard) Format */
	CAD_AMR_FF_CONFORMANCE = 0,
	/* AMR Interface Format 1 */
	CAD_AMR_FF_IF1,
	/* AMR Interface Format 2 */
	CAD_AMR_FF_IF2,
	/* AMR File Storage Format */
	CAD_AMR_FF_FSF,
	/* Real-time Transport Protocol Payload */
	CAD_AMR_FF_RTP,
	/* ITU Format */
	CAD_AMR_FF_ITU,
	CAD_AMR_FF_MAX = 0x7FFFFFFF
};


/* cad AMR format block */
struct cad_amr_format {
	/* size of the structure in bytes */
	u32          size;
	/* version */
	u32          version;
	/* number of channels */
	u32          channels;

	/* value is defined by enum cad_amr_band_mode */
	u32          amr_band_mode;

	/* value is defined by enum cad_amr_dtx_mode */
	u32          amr_dtx_mode;

	/* value is defined by enum cad_amr_frame_format */
	u32          amr_frame_format;
};



#endif



