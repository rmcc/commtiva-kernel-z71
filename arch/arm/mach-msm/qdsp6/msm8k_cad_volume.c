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

#include <linux/module.h>
#include <linux/uaccess.h>

#include <mach/qdsp6/msm8k_cad_ioctl.h>
#include <mach/qdsp6/msm8k_cad_devices.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_cad_module.h>
#include <mach/qdsp6/msm8k_cad_itypes.h>
#include <mach/qdsp6/msm8k_cad_volume.h>
#include <mach/qdsp6/msm8k_q6_api_flip_utils.h>
#include <mach/qdsp6/msm8k_adsp_audio_stream_ioctl.h>
#include <mach/qdsp6/msm8k_adsp_audio_device_ioctl.h>
#include <mach/qdsp6/msm8k_ardi.h>

static struct cad_device_volume_cache
		qdsp6_volume_cache_tbl[QDSP6VOLUME_MAX_DEVICE_COUNT];

static s32 stream_volume_cache;

static u32 audio_ctrl_handle;


#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_vol: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

void set_audio_ctrl_handle(u32 handle)
{
	audio_ctrl_handle = handle;
}

enum cad_int_device_id {

	INT_CAD_HW_DEVICE_ID_DEFAULT_TX,
	INT_CAD_HW_DEVICE_ID_DEFAULT_RX,

	/* Internal devices */
	INT_CAD_HW_DEVICE_ID_HANDSET_MIC,
	INT_CAD_HW_DEVICE_ID_HANDSET_SPKR,
	INT_CAD_HW_DEVICE_ID_HEADSET_MIC,
	INT_CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO,
	INT_CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO,
	INT_CAD_HW_DEVICE_ID_SPKR_PHONE_MIC,
	INT_CAD_HW_DEVICE_ID_SPKR_PHONE_MONO,
	INT_CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO,
	INT_CAD_HW_DEVICE_ID_BT_SCO_MIC,
	INT_CAD_HW_DEVICE_ID_BT_SCO_SPKR,
	INT_CAD_HW_DEVICE_ID_TTY_HEADSET_MIC,
	INT_CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR,

	INT_CAD_HW_DEVICE_ID_BT_A2DP_SPKR,
	/* Logical Device to indicate A2DP routing */
	INT_CAD_HW_DEVICE_ID_BT_A2DP_TX,

	/* I2S */
	INT_CAD_HW_DEVICE_ID_I2S_RX,
	INT_CAD_HW_DEVICE_ID_I2S_TX,

	/* AUXPGA */
	INT_CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO_LB,
	INT_CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO_LB,
	INT_CAD_HW_DEVICE_ID_SPEAKER_SPKR_STEREO_LB,
	INT_CAD_HW_DEVICE_ID_SPEAKER_SPKR_MONO_LB,

	/* Currently support following Single Stream/Multiple Devices combo: */
	INT_CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_MONO_RX,
	INT_CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_STEREO_RX,
	INT_CAD_HW_DEVICE_ID_HEADSET_STEREO_PLUS_SPKR_MONO_RX,
	INT_CAD_HW_DEVICE_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX,

	INT_CAD_HW_DEVICE_ID_MAX_NUM,

	INT_CAD_HW_DEVICE_ID_INVALID
};

enum cad_int_device_id qdsp6_volume_device_id_mapping(u32 device_id)
{
	switch (device_id) {
	case CAD_HW_DEVICE_ID_HANDSET_MIC:
		return INT_CAD_HW_DEVICE_ID_HANDSET_MIC;
	case CAD_HW_DEVICE_ID_HANDSET_SPKR:
		return INT_CAD_HW_DEVICE_ID_HANDSET_SPKR;
	case CAD_HW_DEVICE_ID_HEADSET_MIC:
		return INT_CAD_HW_DEVICE_ID_HEADSET_MIC;
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO:
		return INT_CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO;
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO:
		return INT_CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO;
	case CAD_HW_DEVICE_ID_SPKR_PHONE_MIC:
		return INT_CAD_HW_DEVICE_ID_SPKR_PHONE_MIC;
	case CAD_HW_DEVICE_ID_SPKR_PHONE_MONO:
		return INT_CAD_HW_DEVICE_ID_SPKR_PHONE_MONO;
	case CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO:
		return INT_CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO;
	case CAD_HW_DEVICE_ID_BT_SCO_MIC:
		return INT_CAD_HW_DEVICE_ID_BT_SCO_MIC;
	case CAD_HW_DEVICE_ID_BT_SCO_SPKR:
		return INT_CAD_HW_DEVICE_ID_BT_SCO_SPKR;
	case CAD_HW_DEVICE_ID_BT_A2DP_SPKR:
		return INT_CAD_HW_DEVICE_ID_BT_A2DP_SPKR;
	case CAD_HW_DEVICE_ID_TTY_HEADSET_MIC:
		return INT_CAD_HW_DEVICE_ID_TTY_HEADSET_MIC;
	case CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR:
		return INT_CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR;
	case CAD_HW_DEVICE_ID_DEFAULT_TX:
		return INT_CAD_HW_DEVICE_ID_DEFAULT_TX;
	case CAD_HW_DEVICE_ID_DEFAULT_RX:
		return INT_CAD_HW_DEVICE_ID_DEFAULT_RX;
	case CAD_HW_DEVICE_ID_BT_A2DP_TX:
		return INT_CAD_HW_DEVICE_ID_BT_A2DP_TX;
	case CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_MONO_RX:
		return INT_CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_MONO_RX;
	case CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_STEREO_RX:
		return INT_CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_STEREO_RX;
	case CAD_HW_DEVICE_ID_HEADSET_STEREO_PLUS_SPKR_MONO_RX:
		return INT_CAD_HW_DEVICE_ID_HEADSET_STEREO_PLUS_SPKR_MONO_RX;
	case CAD_HW_DEVICE_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX:
		return INT_CAD_HW_DEVICE_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX;
	case CAD_HW_DEVICE_ID_I2S_RX:
		return INT_CAD_HW_DEVICE_ID_I2S_RX;
	case CAD_HW_DEVICE_ID_I2S_TX:
		return INT_CAD_HW_DEVICE_ID_I2S_TX;
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO_LB:
		return INT_CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO_LB;
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO_LB:
		return INT_CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO_LB;
	case CAD_HW_DEVICE_ID_SPEAKER_SPKR_STEREO_LB:
		return INT_CAD_HW_DEVICE_ID_SPEAKER_SPKR_STEREO_LB;
	case CAD_HW_DEVICE_ID_SPEAKER_SPKR_MONO_LB:
		return INT_CAD_HW_DEVICE_ID_SPEAKER_SPKR_MONO_LB;

	default:
		return INT_CAD_HW_DEVICE_ID_INVALID;
	}
}

/* This computes linear mapping device volume. */
s32 qdsp6_volume_mapping(u32 device_id, s32 percentage)
{
	s32 max_gain = 0;
	s32 min_gain = 0;
	u32 tmp_device_id = qdsp6_volume_device_id_mapping(device_id);

	if (tmp_device_id == INT_CAD_HW_DEVICE_ID_INVALID) {
		pr_err("%s: invalid device\n", __func__);
		return 0xFFFFFFFF;
	}

	if (percentage < 0 || percentage > 100) {
		pr_err("%s: invalid percentage\n", __func__);
		return 0xFFFFFFFF;
	}

	max_gain = qdsp6_volume_cache_tbl[device_id].max_gain;
	min_gain = qdsp6_volume_cache_tbl[device_id].min_gain;

	return min_gain + (((max_gain - min_gain) * percentage) / 100);
}

s32 qdsp6_volume_open(s32 session_id,
	struct cad_open_struct_type *open_param)
{
	return CAD_RES_SUCCESS;
}

s32 qdsp6_volume_close(s32 session_id)
{
	return CAD_RES_SUCCESS;
}

s32 qdsp6_volume_write(s32 session_id,
	struct cad_buf_struct_type *buf_ptr)
{
	return CAD_RES_SUCCESS;
}

s32 qdsp6_volume_read(s32 session_id,
	struct cad_buf_struct_type *buf_ptr)
{
	return CAD_RES_SUCCESS;
}

s32 qdsp6_volume_ioctl(s32 session_id, u32 cmd_code,
	void *cmd_buf, u32 cmd_len)
{
	enum cad_int_device_id device_id = INT_CAD_HW_DEVICE_ID_INVALID;
	struct cad_filter_struct *vol_flt = NULL;
	struct cad_flt_cfg_dev_vol *dev_vol_buf = NULL;
	struct cad_flt_cfg_strm_vol *stream_vol_buf = NULL;
	struct cad_flt_cfg_dev_mute *dev_mute_buf = NULL;
	struct cad_flt_cfg_strm_mute *stream_mute_buf = NULL;

	struct adsp_audio_set_device_volume *q6_set_dev_vol = NULL;
	struct adsp_audio_set_stream_volume *q6_set_strm_vol = NULL;
	struct adsp_audio_set_device_mute *q6_set_dev_mute = NULL;
	struct adsp_audio_set_stream_mute *q6_set_strm_mute = NULL;

	int rc = CAD_RES_SUCCESS;
	s32 device_volume = 0;
	s32 rpc_cmd_code = 0;
	u8 *rpc_cmd_buf = NULL;
	u32 rpc_cmd_buf_len = 0;
	struct adsp_audio_event event_payload;

	struct adsp_audio_set_device_volume *q6_set_dev_vol1 = NULL;
	struct adsp_audio_set_stream_volume *q6_set_strm_vol1 = NULL;
	struct adsp_audio_set_device_mute *q6_set_dev_mute1 = NULL;
	struct adsp_audio_set_stream_mute *q6_set_strm_mute1 = NULL;

	s32 rpc_cmd_code1 = 0;
	u8 *rpc_cmd_buf1 = NULL;
	u32 rpc_cmd_buf_len1 = 0;
	struct adsp_audio_event event_payload1;


	memset(&event_payload, 0, sizeof(struct adsp_audio_event));
	memset(&event_payload1, 0, sizeof(struct adsp_audio_event));
	/* Ensure session_id is valid. */
	if (session_id < 1 || session_id >= CAD_MAX_SESSION)
		return CAD_RES_FAILURE;

	/* Not handle request other than the following two. */
	/* Just silently succeed unrecognized IOCTLs. */
	if (cmd_code != CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG &&
		cmd_code != CAD_IOCTL_CMD_SET_DEVICE_FILTER_CONFIG &&
		cmd_code != CAD_IOCTL_CMD_STREAM_START)
		return CAD_RES_SUCCESS;

	/* Defensive programming. */
	if ((cmd_buf == NULL
		|| cmd_len != sizeof(struct cad_filter_struct))
		&& cmd_code != CAD_IOCTL_CMD_STREAM_START) {
		D("%s: invalid params\n", __func__);
		return CAD_RES_FAILURE;
	}

	/* Do not handle stream_start for device control session. */
	if (cmd_code == CAD_IOCTL_CMD_STREAM_START &&
		ardsession[session_id]->session_type == DEVICE_CTRL_TYPE) {
		D("%s: not handling for device control type\n", __func__);
		return CAD_RES_SUCCESS;
	}

	/* Handle stream start. */
	if (cmd_code == CAD_IOCTL_CMD_STREAM_START) {
		if (ardsession[session_id]->sess_open_info->cad_stream.app_type
				== CAD_STREAM_APP_VOICE) {
			D("%s: Do not handle voice session.\n", __func__);
			return CAD_RES_SUCCESS;
		}

		q6_set_strm_mute1 = kmalloc(
			sizeof(struct adsp_audio_set_stream_mute),
			GFP_KERNEL);
		if (!q6_set_strm_mute1)
			return CAD_RES_FAILURE;

		memset(q6_set_strm_mute1, 0,
			sizeof(struct adsp_audio_set_stream_mute));
		/* 2. Assign values to command buffer. */
		if (stream_volume_cache == CAD_STREAM_MIN_GAIN)
			q6_set_strm_mute1->mute = 1;
		else
			q6_set_strm_mute1->mute = 0;

		rpc_cmd_buf1 = (u8 *)q6_set_strm_mute1;
		rpc_cmd_buf_len1 = sizeof(struct adsp_audio_set_stream_mute);
		rpc_cmd_code1 = ADSP_AUDIO_IOCTL_CMD_SET_STREAM_MUTE;
		/* 3. Send command to Q6. */

		if (ardsession[session_id]->sess_open_info->cad_open.op_code
				== CAD_OPEN_OP_WRITE) {
			/* Only issue stream commands for Rx path. */
			rc = cad_rpc_ioctl(
				session_id,
				1,
				rpc_cmd_code1,
				rpc_cmd_buf1,
				rpc_cmd_buf_len1,
				&event_payload1);
			if (rc != CAD_RES_SUCCESS) {
				pr_err("%s: cad_rpc_ioctl() failure\n",
					__func__);
				return rc;
			}
		}

		if (stream_volume_cache != CAD_STREAM_MIN_GAIN) {
			q6_set_strm_vol1 = kmalloc(
				sizeof(struct adsp_audio_set_stream_volume),
				GFP_KERNEL);
			if (!q6_set_strm_vol1)
				return CAD_RES_FAILURE;

			/* 2. Assign values to command buffer. */
			q6_set_strm_vol1->volume = stream_volume_cache;
			rpc_cmd_buf1 = (u8 *)q6_set_strm_vol1;
			rpc_cmd_buf_len1 =
				sizeof(struct adsp_audio_set_stream_volume);
			rpc_cmd_code1 = ADSP_AUDIO_IOCTL_CMD_SET_STREAM_VOL;
			if (ardsession[session_id]->sess_open_info->
				cad_open.op_code == CAD_OPEN_OP_WRITE) {
				/* Only issue stream commands for Rx path. */
				rc = cad_rpc_ioctl(
					session_id,
					1,
					rpc_cmd_code1,
					rpc_cmd_buf1,
					rpc_cmd_buf_len1,
					&event_payload1);
				if (rc != CAD_RES_SUCCESS) {
					pr_err("%s: cad_rpc_ioctl() failure\n",
						__func__);
					return rc;
				}
			}
		}

		q6_set_dev_mute1 = kmalloc(
			sizeof(struct adsp_audio_set_device_mute), GFP_KERNEL);
		if (!q6_set_dev_mute1) {
			rc = CAD_RES_FAILURE;
			goto done;
		}

		memset(q6_set_dev_mute1, 0,
			sizeof(struct adsp_audio_set_device_mute));

		/* Send Device Volume during stream start. */
		if (ardsession[session_id]->sess_open_info->cad_open.op_code
				== CAD_OPEN_OP_READ) {
			device_id = ard_state.def_tx_device;

			if (device_id == INT_CAD_HW_DEVICE_ID_INVALID) {
				rc = CAD_RES_FAILURE;
				pr_err("%s: invalid device id %d\n", __func__,
					ard_state.def_tx_device);
				goto done;
			}
			q6_set_dev_mute1->path = CAD_TX_DEVICE;
		} else if (ardsession[session_id]->sess_open_info->
				cad_open.op_code == CAD_OPEN_OP_WRITE) {
			device_id = ard_state.def_rx_device;

			if (device_id == INT_CAD_HW_DEVICE_ID_INVALID) {
				rc = CAD_RES_FAILURE;
				pr_err("%s: invalid device id %d\n", __func__,
						ard_state.def_rx_device);
				goto done;
			}
			q6_set_dev_mute1->path = CAD_RX_DEVICE;
		}

		/* 2. Assign values to command buffer. */
		q6_set_dev_mute1->device = q6_device_id_mapping(device_id);

		device_id = qdsp6_volume_device_id_mapping(device_id);

		if (qdsp6_volume_cache_tbl[device_id].mute == 1)
			q6_set_dev_mute1->mute = 1;
		else
			q6_set_dev_mute1->mute = 0;

		rpc_cmd_buf1 = (u8 *)q6_set_dev_mute1;
		rpc_cmd_buf_len1 = sizeof(struct adsp_audio_set_device_mute);
		rpc_cmd_code1 = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE;

		rc = cad_rpc_ioctl(
			audio_ctrl_handle,
			1,
			rpc_cmd_code1,
			rpc_cmd_buf1,
			rpc_cmd_buf_len1,
			&event_payload1);
		if (rc != CAD_RES_SUCCESS) {
			pr_err("%s: cad_rpc_ioctl() failure\n",
				__func__);
			return rc;
		}

		if (qdsp6_volume_cache_tbl[device_id].mute == 0) {

			q6_set_dev_vol1 = kmalloc(
				sizeof(struct adsp_audio_set_device_volume),
				GFP_KERNEL);
			if (!q6_set_dev_vol1)
				return CAD_RES_FAILURE;

			memset(q6_set_dev_vol1, 0,
				sizeof(struct adsp_audio_set_device_volume));

			if (qdsp6_volume_cache_tbl[device_id].
						valid_current_volume == 1) {
				q6_set_dev_vol1->volume =
					qdsp6_volume_cache_tbl[device_id].
							current_volume;
				D("%s: current_volume is %d\n", __func__,
						q6_set_dev_vol1->volume);
			} else {
				q6_set_dev_vol1->volume =
					qdsp6_volume_cache_tbl[device_id].
							default_volume;
				D("%s: current_volume is %d (default)\n",
					__func__, q6_set_dev_vol1->volume);
			}

			q6_set_dev_vol1->path = q6_set_dev_mute1->path;
			q6_set_dev_vol1->device = q6_set_dev_mute1->device;
			rpc_cmd_buf1 = (u8 *)q6_set_dev_vol1;
			rpc_cmd_buf_len1 =
				sizeof(struct adsp_audio_set_device_volume);
			rpc_cmd_code1 = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_VOL;

			rc = cad_rpc_ioctl(
				audio_ctrl_handle,
				1,
				rpc_cmd_code1,
				rpc_cmd_buf1,
				rpc_cmd_buf_len1,
				&event_payload1);
			if (rc != CAD_RES_SUCCESS) {
				pr_err("%s: cad_rpc_ioctl() failure\n",
					__func__);
				return rc;
			}
		}

		rc = CAD_RES_SUCCESS;
		goto done;
	}

	if ((cmd_buf == NULL) || (cmd_len !=
			sizeof(struct cad_filter_struct))) {
		pr_err("%s: invalid ioctl params\n", __func__);
		rc = CAD_RES_FAILURE;
		goto done;
	}

	vol_flt = (struct cad_filter_struct *)cmd_buf;

	if (vol_flt->filter_type != CAD_DEVICE_FILTER_TYPE_VOL) {
		D("%s: not volume filter type\n", __func__);
		rc = CAD_RES_SUCCESS;
		goto done;
	}

	/* Find the appropriate command type. */
	if (cmd_code == CAD_IOCTL_CMD_SET_DEVICE_FILTER_CONFIG
		&& vol_flt->cmd == CAD_FILTER_CONFIG_DEVICE_VOLUME
		&& vol_flt->format_block_len ==
			sizeof(struct cad_flt_cfg_dev_vol))
		dev_vol_buf =
			(struct cad_flt_cfg_dev_vol *)
					vol_flt->format_block;
	else if (CAD_IOCTL_CMD_SET_DEVICE_FILTER_CONFIG == cmd_code
		&& vol_flt->cmd == CAD_FILTER_CONFIG_DEVICE_MUTE
		&& vol_flt->format_block_len ==
			sizeof(struct cad_flt_cfg_dev_mute))
		dev_mute_buf = (struct cad_flt_cfg_dev_mute *)
						vol_flt->format_block;

	/* stream session */
	else if (cmd_code == CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG
		&& vol_flt->cmd == CAD_FILTER_CONFIG_STREAM_VOLUME
		&& vol_flt->format_block_len ==
			sizeof(struct cad_flt_cfg_strm_vol))
		stream_vol_buf = (struct cad_flt_cfg_strm_vol *)(
					vol_flt->format_block);

	else if (cmd_code == CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG
		&& vol_flt->cmd == CAD_FILTER_CONFIG_STREAM_MUTE
		&& vol_flt->format_block_len ==
			sizeof(struct cad_flt_cfg_strm_mute))
		stream_mute_buf = (struct cad_flt_cfg_strm_mute *)
						(vol_flt->format_block);

	else {
		pr_err("CAD:VOL: Error: wrong type id.\n");
		return CAD_RES_FAILURE;
	}

	/* Handle volume control command. */
	switch (vol_flt->cmd) {
	case CAD_FILTER_CONFIG_DEVICE_VOLUME:
		D("CAD:VOL: Device Volume\n");

		if (dev_vol_buf->device_id == 0 ||
			dev_vol_buf->device_id > CAD_HW_DEVICE_ID_MAX_NUM) {
			pr_err("%s: invalid device id %d.\n",
					__func__, dev_vol_buf->device_id);
			rc = CAD_RES_FAILURE;
			goto done;
		}

		if (dev_vol_buf->device_id == CAD_HW_DEVICE_ID_DEFAULT_TX)
			dev_vol_buf->device_id = ard_state.def_tx_device;
		else if (dev_vol_buf->device_id == CAD_HW_DEVICE_ID_DEFAULT_RX)
			dev_vol_buf->device_id = ard_state.def_rx_device;
		else
			; /* Do nothing. */

		if (ardsession[session_id]->sess_open_info->cad_open.op_code
				== CAD_OPEN_OP_READ) {
			if (dev_vol_buf->device_id !=
					ard_state.def_tx_device) {
				pr_err("%s: %d is not current device id.\n",
					__func__, dev_vol_buf->device_id);
				rc = CAD_RES_FAILURE;
				goto done;
			}
		} else if (ardsession[session_id]->sess_open_info->
				cad_open.op_code == CAD_OPEN_OP_WRITE) {
			if (dev_vol_buf->device_id !=
					ard_state.def_rx_device) {
				pr_err("%s: %d is not current device id.\n",
					__func__, dev_vol_buf->device_id);
				rc = CAD_RES_FAILURE;
				goto done;
			}
		}

		/* For volume != 0%: send unmute command. */
		if (dev_vol_buf->volume != 0) {
			/* Construct QDSP6 device mute command. */
			/* 1. Allocate memory for command buffer. */
			q6_set_dev_mute = kmalloc(
				sizeof(struct adsp_audio_set_device_mute),
				GFP_KERNEL);
			if (!q6_set_dev_mute)
				return CAD_RES_FAILURE;

			memset(q6_set_dev_mute, 0,
				sizeof(struct adsp_audio_set_device_mute));
			/* 2. Assign values to command buffer. */
			q6_set_dev_mute->device =
				q6_device_id_mapping(dev_vol_buf->device_id);
			q6_set_dev_mute->path = dev_vol_buf->path;
			q6_set_dev_mute->mute = 0;
			rpc_cmd_buf = (u8 *)q6_set_dev_mute;
			rpc_cmd_buf_len =
				sizeof(struct adsp_audio_set_device_mute);
			rpc_cmd_code = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE;
			/* 3. Send command to Q6. */
			rc = cad_rpc_ioctl(
				session_id,
				1,
				rpc_cmd_code,
				rpc_cmd_buf,
				rpc_cmd_buf_len,
				&event_payload);
			if (rc != CAD_RES_SUCCESS) {
				pr_err("%s: cad_rpc_ioctl() failure\n",
					__func__);
				return rc;
			}

		}

		/* Map the device volume to QDSP6. */
		device_volume = qdsp6_volume_mapping(
			dev_vol_buf->device_id,
			dev_vol_buf->volume);

		/* Cache the device volume. */
		device_id = qdsp6_volume_device_id_mapping(
				dev_vol_buf->device_id);
		qdsp6_volume_cache_tbl[device_id]
			.current_volume = device_volume;
		qdsp6_volume_cache_tbl[device_id]
			.valid_current_volume = 1;
		qdsp6_volume_cache_tbl[device_id]
			.mute = 0;

		/* Construct QDSP6 device volume command:	*/
		/* 1. Allocate memory for command buffer.	*/
		q6_set_dev_vol = kmalloc(
			sizeof(struct cad_flt_cfg_dev_vol),
			GFP_KERNEL);
		if (!q6_set_dev_vol)
			return CAD_RES_FAILURE;

		memset(q6_set_dev_vol, 0,
			sizeof(struct adsp_audio_set_device_volume));

		/* 2. Assign values to command buffer. */
		q6_set_dev_vol->device =
			q6_device_id_mapping(dev_vol_buf->device_id);
		q6_set_dev_vol->path = dev_vol_buf->path;
		q6_set_dev_vol->volume = device_volume;
		rpc_cmd_buf = (u8 *)q6_set_dev_vol;
		rpc_cmd_buf_len =
			sizeof(struct adsp_audio_set_device_volume);
		rpc_cmd_code = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_VOL;

		/* HACK: for volume = 0%: send mute command instead. */
		if (dev_vol_buf->volume == 0) {
			/* Construct QDSP6 device mute command. */
			/* 1. Allocate memory for command buffer. */
			q6_set_dev_mute = kmalloc(
				sizeof(struct adsp_audio_set_device_mute),
				GFP_KERNEL);
			if (!q6_set_dev_mute)
				return CAD_RES_FAILURE;

			memset(q6_set_dev_mute, 0,
				sizeof(struct adsp_audio_set_device_mute));
			/* 2. Assign values to command buffer. */
			q6_set_dev_mute->device = q6_set_dev_vol->device;
			q6_set_dev_mute->path = q6_set_dev_vol->path;
			q6_set_dev_mute->mute = 1; /* mute */
			rpc_cmd_buf = (u8 *)q6_set_dev_mute;
			rpc_cmd_buf_len =
				sizeof(struct adsp_audio_set_device_mute);
			rpc_cmd_code = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE;
		}

		break;
	case CAD_FILTER_CONFIG_DEVICE_MUTE:
		D("CAD:VOL: Device Mute\n");

		device_id = qdsp6_volume_device_id_mapping(
				dev_mute_buf->device_id);
		qdsp6_volume_cache_tbl[device_id].mute = dev_mute_buf->mute;

		/* Construct QDSP6 device mute command. */
		/* 1. Allocate memory for command buffer. */
		q6_set_dev_mute = kmalloc(
			sizeof(struct adsp_audio_set_device_mute),
			GFP_KERNEL);
		if (!q6_set_dev_mute)
			return CAD_RES_FAILURE;

		memset(q6_set_dev_mute, 0,
			sizeof(struct adsp_audio_set_device_mute));
		/* 2. Assign values to command buffer. */
		q6_set_dev_mute->device =
			q6_device_id_mapping(dev_mute_buf->device_id);
		q6_set_dev_mute->path = dev_mute_buf->path;
		q6_set_dev_mute->mute = dev_mute_buf->mute;
		rpc_cmd_buf = (u8 *)q6_set_dev_mute;
		rpc_cmd_buf_len =
			sizeof(struct adsp_audio_set_device_mute);
		rpc_cmd_code = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE;

		break;
	case CAD_FILTER_CONFIG_STREAM_VOLUME:
		D("CAD:VOL: Stream Volume\n");

		stream_volume_cache = stream_vol_buf->volume;

		if (ardsession[session_id]->active != ARD_TRUE) {
			rc = CAD_RES_SUCCESS;
			D("not active session, cached stream volume.\n");
			goto done;
		}

		/* For volume != min: send unmute command. */
		if (stream_vol_buf->volume != CAD_STREAM_MIN_GAIN) {
			/* Construct QDSP6 stream mute command. */
			/* 1. Allocate memory for command buffer. */
			q6_set_strm_mute = kmalloc(
				sizeof(struct adsp_audio_set_stream_mute),
				GFP_KERNEL);
			if (!q6_set_strm_mute)
				return CAD_RES_FAILURE;

			/* 2. Assign values to command buffer.	*/
			q6_set_strm_mute->mute = 0;
			rpc_cmd_buf = (u8 *)q6_set_strm_mute;
			rpc_cmd_buf_len =
				sizeof(struct adsp_audio_set_stream_mute);
			rpc_cmd_code = ADSP_AUDIO_IOCTL_CMD_SET_STREAM_MUTE;
			/* 3. Send command to Q6. */
			rc = cad_rpc_ioctl(session_id,
					     1,
					     rpc_cmd_code,
					     rpc_cmd_buf,
					     rpc_cmd_buf_len,
					     &event_payload);
			if (rc != CAD_RES_SUCCESS) {
				pr_err("%s: cad_rpc_ioctl() failure\n",
					__func__);
				return rc;
			}
		}

		/* Construct QDSP6 stream volume command. */
		/* 1. Allocate memory for command buffer. */
		q6_set_strm_vol = kmalloc(
			sizeof(struct adsp_audio_set_stream_volume),
			GFP_KERNEL);
		if (!q6_set_strm_vol)
			return CAD_RES_FAILURE;

		/* 2. Assign values to command buffer. */
		q6_set_strm_vol->volume = stream_vol_buf->volume;
		rpc_cmd_buf = (u8 *)q6_set_strm_vol;
		rpc_cmd_buf_len = sizeof(struct adsp_audio_set_stream_volume);
		rpc_cmd_code = ADSP_AUDIO_IOCTL_CMD_SET_STREAM_VOL;

		/* For volume = min: send mute command instead. */
		if (stream_vol_buf->volume == CAD_STREAM_MIN_GAIN) {
			/* Construct QDSP6 stream mute command. */
			/* 1. Allocate memory for command buffer. */
			q6_set_strm_mute = kmalloc(
					sizeof(
					struct adsp_audio_set_stream_mute),
					GFP_KERNEL);

			if (!q6_set_strm_mute)
				return CAD_RES_FAILURE;

			/* 2. Assign values to command buffer. */
			q6_set_strm_mute->mute = 1;
			rpc_cmd_buf = (u8 *)q6_set_strm_mute;
			rpc_cmd_buf_len =
				sizeof(struct adsp_audio_set_stream_mute);
			rpc_cmd_code = ADSP_AUDIO_IOCTL_CMD_SET_STREAM_MUTE;
		}

		break;
	case CAD_FILTER_CONFIG_STREAM_MUTE:
		D("CAD:VOL: Stream Mute\n");

		/* Construct QDSP6 stream mute command. */
		/* 1. Allocate memory for command buffer. */
		q6_set_strm_mute = kmalloc(
					sizeof(
					struct adsp_audio_set_stream_mute),
					GFP_KERNEL);
		if (!q6_set_strm_mute)
			return CAD_RES_FAILURE;

		/* 2. Assign values to command buffer. */
		q6_set_strm_mute->mute = stream_mute_buf->mute;
		rpc_cmd_buf = (u8 *)q6_set_strm_mute;
		rpc_cmd_buf_len = sizeof(struct adsp_audio_set_stream_mute);
		rpc_cmd_code = ADSP_AUDIO_IOCTL_CMD_SET_STREAM_MUTE;

		break;
	default:
		/* Just return without error. */
		return CAD_RES_SUCCESS;
	}

	/* Always send device/stream volume command to Q6 for now. */
	rc = cad_rpc_ioctl(
			session_id,
			1,
			rpc_cmd_code,
			rpc_cmd_buf,
			rpc_cmd_buf_len,
			&event_payload);
	if (rc != CAD_RES_SUCCESS)
		pr_err("%s: cad_rpc_ioctl() failure\n", __func__);

done:
	D("%s: ioctl() processed.\n", __func__);

	kfree(q6_set_dev_vol);
	kfree(q6_set_strm_vol);
	kfree(q6_set_dev_mute);
	kfree(q6_set_strm_mute);
	kfree(q6_set_dev_vol1);
	kfree(q6_set_strm_vol1);
	kfree(q6_set_dev_mute1);
	kfree(q6_set_strm_mute1);

	return rc;
}


int cad_volume_dinit(void)
{
	memset(qdsp6_volume_cache_tbl, 0,
		sizeof(struct cad_device_volume_cache) *
			QDSP6VOLUME_MAX_DEVICE_COUNT);
	stream_volume_cache = 0;
	return CAD_RES_SUCCESS;
}


int cad_volume_init(struct cad_func_tbl_type **func_tbl)
{

	static struct cad_func_tbl_type vtable = {
		qdsp6_volume_open,
		qdsp6_volume_close,
		qdsp6_volume_write,
		qdsp6_volume_read,
		qdsp6_volume_ioctl
	};

	*func_tbl = &vtable;

	/* Set up the volume cache table by default values. */
	memset(qdsp6_volume_cache_tbl, 0,
		sizeof(struct cad_device_volume_cache)
			* QDSP6VOLUME_MAX_DEVICE_COUNT);

	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HANDSET_SPKR].max_gain
		= CAD_DEVICE_HANDSET_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HANDSET_SPKR].min_gain
		= CAD_DEVICE_HANDSET_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO].max_gain
		= CAD_DEVICE_HEADSET_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO].min_gain
		= CAD_DEVICE_HEADSET_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO].max_gain
		= CAD_DEVICE_HEADSET_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO].min_gain
		= CAD_DEVICE_HEADSET_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_SPKR_PHONE_MONO].max_gain
		= CAD_DEVICE_SPEAKER_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_SPKR_PHONE_MONO].min_gain
		= CAD_DEVICE_SPEAKER_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO].max_gain
		= CAD_DEVICE_SPEAKER_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO].min_gain
		= CAD_DEVICE_SPEAKER_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_BT_SCO_SPKR].max_gain
		= CAD_DEVICE_BT_SCO_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_BT_SCO_SPKR].min_gain
		= CAD_DEVICE_BT_SCO_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_BT_A2DP_SPKR].max_gain
		= CAD_DEVICE_BT_A2DP_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_BT_A2DP_SPKR].min_gain
		= CAD_DEVICE_BT_A2DP_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR].max_gain
		= CAD_DEVICE_TTY_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR].min_gain
		= CAD_DEVICE_TTY_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HANDSET_MIC].max_gain
		= CAD_DEVICE_HANDSET_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HANDSET_MIC].min_gain
		= CAD_DEVICE_HANDSET_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HEADSET_MIC].max_gain
		= CAD_DEVICE_HEADSET_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HEADSET_MIC].min_gain
		= CAD_DEVICE_HEADSET_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_SPKR_PHONE_MIC].max_gain
		= CAD_DEVICE_SPEAKER_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_SPKR_PHONE_MIC].min_gain
		= CAD_DEVICE_SPEAKER_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_BT_SCO_MIC].max_gain
		= CAD_DEVICE_BT_SCO_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_BT_SCO_MIC].min_gain
		= CAD_DEVICE_BT_SCO_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_TTY_HEADSET_MIC].max_gain
		= CAD_DEVICE_TTY_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_TTY_HEADSET_MIC].min_gain
		= CAD_DEVICE_TTY_MIN_GAIN;

	stream_volume_cache = 0;

	return CAD_RES_SUCCESS;
}

int volume_set_max_vol_all(void)
{
	int i;

	for (i = 0; i < CAD_HW_DEVICE_ID_MAX_NUM; i++) {
		qdsp6_volume_cache_tbl[i].valid_current_volume = 1;
		qdsp6_volume_cache_tbl[i].current_volume =
			qdsp6_volume_cache_tbl[i].max_gain;
	}

	return 0;
}
EXPORT_SYMBOL(volume_set_max_vol_all);

