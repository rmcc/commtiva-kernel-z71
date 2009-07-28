/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#ifndef __ADSP_AUDIO_CONFIG_IOCTL_H
#define __ADSP_AUDIO_CONFIG_IOCTL_H


#include <mach/qdsp6/msm8k_adsp_audio_types.h>


/* Device control session only IOCTL command definitions */


/* Set configuration data for an algorithm aspect of a device. */
/* This command has payload adsp_audio_device_config . */

#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_CONFIG		0x0108b6cb

struct adsp_phys_mem_type {
	u32	addr;	/* physical address */
	u32	total;	/* Length of allocated memory */
	u32	used;	/* Actual Length of buffer */
} __attribute__ ((packed));



struct adsp_audio_device_config {
	/* Associated client data */
	struct adsp_audio_header	header;
	/* ADSP Device ID */
	u32				device_id;
	/* Algorithm block ID */
	u32				block_id;
	/* Configurable element ID of a MM */
	u32				interface_id;
	/* Refer to DCN: TBD */
	struct adsp_phys_mem_type	phys_mem;
} __attribute__ ((packed));




/* Set configuration data for all interfaces of a device. */
/* This command has payload adsp_audio_device_config_table . */

#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_CONFIG_TABLE	0x0108b6bf


struct adsp_audio_device_config_table  {
	/* Associated client data */
	struct adsp_audio_header	header;
	/* ADSP Device ID */
	u32				device_id;
	/* Refer to DCN: TBD */
	struct adsp_phys_mem_type	phys_mem;
} __attribute__ ((packed));



/* Configurable Algorithm Aspect IDs */
#define ADSP_AUDIO_IID_ENABLE_FLAG			0x0108b6b9
#define ADSP_AUDIO_IID_ADRC_CONFIG_PARAM		0x0108b6ba
#define ADSP_AUDIO_IID_IIR_PREGAIN			0x0108b6bc
#define ADSP_AUDIO_IID_IIR_PARAM			0x0108b6be
#define ADSP_AUDIO_IID_AGC_PARAM			0x0108b6c0
#define ADSP_AUDIO_IID_AVC_TX_PARAM			0x0108b6c2
#define ADSP_AUDIO_IID_AVC_RX_PARAM			0x0108b6c3
#define ADSP_AUDIO_IID_EC_TX_PARAM			0x0108b6c1
#define ADSP_AUDIO_IID_EC_RX_PARAM			0x0108b6c4
#define ADSP_AUDIO_IID_VOICEFE_FILTERS_PARAM		0x0108b6c5
#define ADSP_AUDIO_IID_AUDIO_AGC_CONFIG_PARAMS		0x0108b6c6
#define ADSP_AUDIO_IID_VOLUME_CONTROL_PARAM		0x0108b6c7
#define ADSP_AUDIO_IID_AUDIO_MIXER_CONFIG_PARAMS	0x0108b6c8
#define ADSP_AUDIO_IID_AFE_INT_PARAM			0x0108b6c9
#define ADSP_AUDIO_IID_AFEI2S_PARAM			0x0108bd61
#define ADSP_AUDIO_IID_AFEPCM_PARAM			0x0108bd62



/* Algorithm Block IDs */

/* TX filters */
#define ADSP_AUDIO_UID_TX_AFE_PCM_SOURCE		0x01073d67
#define ADSP_AUDIO_UID_TX_AFE_I2S_SOURCE		0x01073d68
#define ADSP_AUDIO_UID_TX_AFE_INT_SOURCE		0x01073d71
#define ADSP_AUDIO_UID_TX_GLOBAL_IIR			0x01073d69
#define ADSP_AUDIO_UID_TX_AUDIO_AGC			0x01073d6a
#define ADSP_AUDIO_UID_TX_VFE				0x01073d6b
#define ADSP_AUDIO_UID_TX_EC				0x01073d6c
#define ADSP_AUDIO_UID_TX_AVC				0x01073d6d
#define ADSP_AUDIO_UID_TX_AGC				0x01073d6e
#define ADSP_AUDIO_UID_TX_VOICE_IIR			0x01073d6f
#define ADSP_AUDIO_UID_TX_VOLUME			0x01073d70

/* RX filters */
#define ADSP_AUDIO_UID_RX_AFE_PCM_SINK			0x01073d74
#define ADSP_AUDIO_UID_RX_AFE_I2S_SINK			0x01073d75
#define ADSP_AUDIO_UID_RX_COMMON_IIR			0x01073d76
#define ADSP_AUDIO_UID_RX_ADRC				0x01073d78
#define ADSP_AUDIO_UID_RX_GLOBAL_MIXER			0x01073d79
#define ADSP_AUDIO_UID_RX_VFE				0x01073d7a
#define ADSP_AUDIO_UID_RX_EC				0x01073d7b
#define ADSP_AUDIO_UID_RX_VOICE_IIR			0x01073d7c
#define ADSP_AUDIO_UID_RX_AVC				0x01073d7d
#define ADSP_AUDIO_UID_RX_AGC				0x01073d7e


#endif
