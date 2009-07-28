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

#ifndef __ADSP_AUDIO_DRIVER_H
#define __ADSP_AUDIO_DRIVER_H


#include <mach/qdsp6/msm8k_adsp_audio_types.h>
#include <mach/qdsp6/msm8k_adsp_audio_error.h>
#include <mach/qdsp6/msm8k_adsp_audio_device.h>
#include <mach/qdsp6/msm8k_adsp_audio_ioctl.h>
#include <mach/qdsp6/msm8k_adsp_audio_media_format.h>
#include <mach/qdsp6/msm8k_adsp_audio_event.h>
#include <mach/qdsp6/msm8k_adsp_audio_cfg_ioctl.h>


/* This function initializes the ADSP Audio Driver Framework. */
/* This function must be called only once at the start of the process */
/* that hosts this driver. */
/* @param[in] cb_data   Client data for asynchronous event notifications */
/* @return    status of the operation */

extern s32 adsp_audio_driver_init(struct adsp_audio_event_cb *cb_data);


/* This function De-initializes the ADSP Audio Driver Framework and releases */
/* all its resources */
/* @return    status of the operation */

extern s32 adsp_audio_driver_release(void);


/* Request to open a media stream session on an audio device. */
/* Note: adsp_audio_driver_init must have been called before this method */
/* This is an asynchronous call, i.e. the client will get a callback when */
/* the stream session has been created. All subsequent requests to the */
/* stream session must be made with the same Handle. */
/* @param[in] handle        Handle for the new stream session. */
/* @param[in] open_param    Parameters to configure the stream session. */
/* @param[in] cb_data Client data for async responses to future requests */
/* @return    status of the operation. */

extern s32 adsp_audio_open(u32 handle,
				struct adsp_audio_open_device *open_param,
				struct adsp_audio_event_cb *cb_data);


/* Request to close a succesfully opened stream session associated with */
/* the Handle. */
/* This is a asynchronous call. Clients will get a callback when stream */
/* session has been closed. */
/* @param[in] handle  Handle of the Media Session. */
/* @return    status of the operation */

extern s32 adsp_audio_close(u32 handle);


/* This function writes data to an open stream session. Caller thread blocks */
/* till stream session has queued the buffer for rendering. */
/* @param[in] handle  Handle of the stream session. */
/* @param[in] buf Pointer to the media data buffer. */
/* @return    status of the operation */
/*            Clients must wait for CAD_MSG_STATUS_BUF_DONE before */
/*            freeing the memory associated with the data buffer. */

extern s32 adsp_audio_write(u32 handle, struct adsp_audio_buffer *buf);


/* This function reads data from an open stream session. */
/* @param[in] handle  Handle of the stream session. */
/* @parm[in]  buf Pointer to the data buffer were data needs to be copied. */
/* @return    status of the operation */
/*            Clients must wait for CAD_MSG_STATUS_BUF_DONE before */
/*            consuming the media data in the buffer. */

extern s32 adsp_audio_read(u32 handle, struct adsp_audio_buffer *buf);


/* Command interface to ADSP Audio Driver */
/* @param[in] handle		unique session id to the driver session. */
/* @param[in] cmd_code		requested operation. */
/* @param[in] cmd_buf		pointer to command payload. */
/* @param[in] cmd_buf_len	lenght of the payload in bytes. */
/* @return			status of the operation */

 extern s32 adsp_audio_ioctl(u32 handle, u32 cmd_code, void *cmd_buf,
				u32 cmd_buf_len);


#endif



