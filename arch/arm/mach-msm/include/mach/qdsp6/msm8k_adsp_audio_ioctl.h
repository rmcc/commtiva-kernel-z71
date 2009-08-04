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

#ifndef __ADSP_AUDIO_IOCTL_H
#define __ADSP_AUDIO_IOCTL_H


#include <mach/qdsp6/msm8k_adsp_audio_device_ioctl.h>


/* Control and Stream session IOCTL command definitions */


/* Opcode to open a device stream session to capture audio */
#define ADSP_AUDIO_IOCTL_CMD_OPEN_READ			0x0108dd79


/* Opcode to open a device stream session to render audio */
#define ADSP_AUDIO_IOCTL_CMD_OPEN_WRITE			0x0108dd7a


/* Opcode to open a device session, must open a device */
#define ADSP_AUDIO_IOCTL_CMD_OPEN_DEVICE		0x0108dd7b


/* Close an existing stream or device */
#define ADSP_AUDIO_IOCTL_CMD_CLOSE			0x0108d8bc



/* A device switch requires three IOCTL */
/* commands in the following sequence: */
/*    ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_PREPARE */
/*    ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_STANDBY */
/*    ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_COMMIT */

/* adsp_audio_device_switch_command structure is needed for */
/* DEVICE_SWITCH_PREPARE */

/* Device switch protocol step #1. Pause old device and */
/* generate silence for the old device. */

#define ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_PREPARE	0x010815c4


/* Device switch protocol step #2. Release old device, */
/* create new device and generate silence for the new device. */

/* When client receives ack for this IOCTL, the client can */
/* start sending IOCTL commands to configure, calibrate and */
/* change filter settings on the new device. */

#define ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_STANDBY	0x010815c5


/* Device switch protocol step #3. Start normal operations on new device */

#define ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_COMMIT	0x01075ee7


#endif


