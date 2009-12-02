/*
 * Common code to deal with the AUDPREPROC dsp task (audio preprocessing)
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Based on the audpp layer in arch/arm/mach-msm/qdsp5/audpp.c
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <mach/debug_audio_mm.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <mach/msm_adsp.h>

#include <mach/qdsp5v2/audpreproc.h>

#define MAX_ENC_COUNT 2

#define MSM_ADSP_ENC_CODEC_WAV 0
#define MSM_ADSP_ENC_CODEC_AAC 1
#define MSM_ADSP_ENC_CODEC_SBC 2
#define MSM_ADSP_ENC_CODEC_AMRNB 3
#define MSM_ADSP_ENC_CODEC_EVRC 4
#define MSM_ADSP_ENC_CODEC_QCELP 5

static DEFINE_MUTEX(audpreproc_lock);

struct msm_adspenc_info {
	const char *module_name;
	unsigned module_queueids;
	int module_encid; /* streamid */
	int enc_formats; /* supported formats */
};

#define ENC_MODULE_INFO(name, queueids, encid, formats) { .module_name = name, \
	.module_queueids = queueids, .module_encid = encid, \
	.enc_formats = formats}

struct msm_adspenc_database {
	unsigned num_enc;
	struct msm_adspenc_info *enc_info_list;
};

static struct msm_adspenc_info enc_info_list[] = {
	ENC_MODULE_INFO("AUDREC0TASK", \
			 ((QDSP_uPAudRec0BitStreamQueue << 16)| \
			   QDSP_uPAudRec0CmdQueue), 0, \
			 ((1 << MSM_ADSP_ENC_CODEC_WAV) | \
			  (1 << MSM_ADSP_ENC_CODEC_SBC))),
	ENC_MODULE_INFO("AUDREC1TASK", \
			 ((QDSP_uPAudRec1BitStreamQueue << 16)| \
			   QDSP_uPAudRec1CmdQueue), 1, \
			 ((1 << MSM_ADSP_ENC_CODEC_WAV) | \
			  (1 << MSM_ADSP_ENC_CODEC_AAC) | \
			  (1 << MSM_ADSP_ENC_CODEC_AMRNB) | \
			  (1 << MSM_ADSP_ENC_CODEC_EVRC) | \
			  (1 << MSM_ADSP_ENC_CODEC_QCELP))),
};

static struct msm_adspenc_database msm_enc_database = {
	.num_enc = ARRAY_SIZE(enc_info_list),
	.enc_info_list = enc_info_list,
};

struct audpreproc_state {
	struct msm_adsp_module *mod;
	audpreproc_event_func func[MAX_ENC_COUNT];
	void *private[MAX_ENC_COUNT];
	struct mutex *lock;
	unsigned open_count;
	unsigned enc_inuse;
};

static struct audpreproc_state the_audpreproc_state = {
	.lock = &audpreproc_lock,
};

/* DSP preproc event handler */
static void audpreproc_dsp_event(void *data, unsigned id, size_t len,
			    void (*getevent)(void *ptr, size_t len))
{
	struct audpreproc_state *audpreproc = data;

	switch (id) {
	case AUDPREPROC_CMD_CFG_DONE_MSG: {
		struct audpreproc_cmd_cfg_done_msg cfg_done_msg;

		getevent(&cfg_done_msg, AUDPREPROC_CMD_CFG_DONE_MSG_LEN);
		MM_DBG("AUDPREPROC_CMD_CFG_DONE_MSG: stream id %d preproc \
			type %x\n", cfg_done_msg.stream_id, \
			cfg_done_msg.aud_preproc_type);
		if ((cfg_done_msg.stream_id < MAX_ENC_COUNT) &&
				audpreproc->func[cfg_done_msg.stream_id])
			audpreproc->func[cfg_done_msg.stream_id](
			audpreproc->private[cfg_done_msg.stream_id], id,
			&cfg_done_msg);
		break;
	}
	case AUDPREPROC_ERROR_MSG: {
		struct audpreproc_err_msg err_msg;

		getevent(&err_msg, AUDPREPROC_ERROR_MSG_LEN);
		MM_DBG("AUDPREPROC_ERROR_MSG: stream id %d err idx %d\n",
		err_msg.stream_id, err_msg.aud_preproc_err_idx);
		if ((err_msg.stream_id < MAX_ENC_COUNT) &&
				audpreproc->func[err_msg.stream_id])
			audpreproc->func[err_msg.stream_id](
			audpreproc->private[err_msg.stream_id], id,
			&err_msg);
		break;
	}
	case AUDPREPROC_CMD_ENC_CFG_DONE_MSG: {
		struct audpreproc_cmd_enc_cfg_done_msg enc_cfg_msg;

		getevent(&enc_cfg_msg, AUDPREPROC_CMD_ENC_CFG_DONE_MSG_LEN);
		MM_DBG("AUDPREPROC_CMD_ENC_CFG_DONE_MSG: stream id %d enc type \
			%d\n", enc_cfg_msg.stream_id, enc_cfg_msg.rec_enc_type);
		if ((enc_cfg_msg.stream_id < MAX_ENC_COUNT) &&
				audpreproc->func[enc_cfg_msg.stream_id])
			audpreproc->func[enc_cfg_msg.stream_id](
			audpreproc->private[enc_cfg_msg.stream_id], id,
			&enc_cfg_msg);
		break;
	}
	case AUDPREPROC_CMD_ENC_PARAM_CFG_DONE_MSG: {
		struct audpreproc_cmd_enc_param_cfg_done_msg enc_param_msg;

		getevent(&enc_param_msg,
				AUDPREPROC_CMD_ENC_PARAM_CFG_DONE_MSG_LEN);
		MM_DBG("AUDPREPROC_CMD_ENC_PARAM_CFG_DONE_MSG: stream id %d\n",
				 enc_param_msg.stream_id);
		if ((enc_param_msg.stream_id < MAX_ENC_COUNT) &&
				audpreproc->func[enc_param_msg.stream_id])
			audpreproc->func[enc_param_msg.stream_id](
			audpreproc->private[enc_param_msg.stream_id], id,
			&enc_param_msg);
		break;
	}
	case AUDPREPROC_AFE_CMD_AUDIO_RECORD_CFG_DONE_MSG: {
		struct audpreproc_afe_cmd_audio_record_cfg_done
						record_cfg_done;
		getevent(&record_cfg_done,
			AUDPREPROC_AFE_CMD_AUDIO_RECORD_CFG_DONE_MSG_LEN);
		MM_DBG("AUDPREPROC_AFE_CMD_AUDIO_RECORD_CFG_DONE_MSG: \
			stream id %d\n", record_cfg_done.stream_id);
		if ((record_cfg_done.stream_id < MAX_ENC_COUNT) &&
				audpreproc->func[record_cfg_done.stream_id])
			audpreproc->func[record_cfg_done.stream_id](
			audpreproc->private[record_cfg_done.stream_id], id,
			&record_cfg_done);
		break;
	}
	default:
		MM_ERR("Unknown Event %d\n", id);
	}
	return;
}

static struct msm_adsp_ops adsp_ops = {
	.event = audpreproc_dsp_event,
};

/* EXPORTED API's */
int audpreproc_enable(int enc_id, audpreproc_event_func func, void *private)
{
	struct audpreproc_state *audpreproc = &the_audpreproc_state;
	int res = 0;

	if (enc_id < 0 || enc_id > MAX_ENC_COUNT)
		return -EINVAL;

	mutex_lock(audpreproc->lock);
	if (audpreproc->func[enc_id]) {
		res = -EBUSY;
		goto out;
	}

	audpreproc->func[enc_id] = func;
	audpreproc->private[enc_id] = private;

	/* First client to enable preproc task */
	if (audpreproc->open_count++ == 0) {
		MM_DBG("Get AUDPREPROCTASK\n");
		res = msm_adsp_get("AUDPREPROCTASK", &audpreproc->mod,
				&adsp_ops, audpreproc);
		if (res < 0) {
			MM_ERR("Can not get AUDPREPROCTASK\n");
			audpreproc->open_count = 0;
			audpreproc->func[enc_id] = NULL;
			audpreproc->private[enc_id] = NULL;
			goto out;
		}
		if (msm_adsp_enable(audpreproc->mod)) {
			MM_ERR("Can not enable AUDPREPROCTASK\n");
			audpreproc->open_count = 0;
			audpreproc->func[enc_id] = NULL;
			audpreproc->private[enc_id] = NULL;
			msm_adsp_put(audpreproc->mod);
			audpreproc->mod = NULL;
			res = -ENODEV;
			goto out;
		}
	}
	res = 0;
out:
	mutex_unlock(audpreproc->lock);
	return res;
}
EXPORT_SYMBOL(audpreproc_enable);

void audpreproc_disable(int enc_id, void *private)
{
	struct audpreproc_state *audpreproc = &the_audpreproc_state;

	if (enc_id < 0 || enc_id > MAX_ENC_COUNT)
		return;

	mutex_lock(audpreproc->lock);
	if (!audpreproc->func[enc_id])
		goto out;
	if (audpreproc->private[enc_id] != private)
		goto out;

	audpreproc->func[enc_id] = NULL;
	audpreproc->private[enc_id] = NULL;

	/* Last client then disable preproc task */
	if (--audpreproc->open_count == 0) {
		msm_adsp_disable(audpreproc->mod);
		MM_DBG("Put AUDPREPROCTASK\n");
		msm_adsp_put(audpreproc->mod);
		audpreproc->mod = NULL;
	}
out:
	mutex_unlock(audpreproc->lock);
	return;
}
EXPORT_SYMBOL(audpreproc_disable);

/* enc_type = supported encode format *
 * like pcm, aac, sbc, evrc, qcelp, amrnb etc ... *
 */
int audpreproc_aenc_alloc(unsigned enc_type, const char **module_name,
		     unsigned *queue_ids)
{
	struct audpreproc_state *audpreproc = &the_audpreproc_state;
	int encid = -1, idx;

	mutex_lock(audpreproc->lock);
	for (idx = (msm_enc_database.num_enc - 1);
		idx >= 0; idx--) {
		/* encoder free and supports the format */
		if ((!(audpreproc->enc_inuse & (1 << idx))) &&
			(msm_enc_database.enc_info_list[idx].enc_formats &
				(1 << enc_type))) {
				break;
		}
	}

	if (idx >= 0) {
		audpreproc->enc_inuse |= (1 << idx);
		*module_name =
		    msm_enc_database.enc_info_list[idx].module_name;
		*queue_ids =
		    msm_enc_database.enc_info_list[idx].module_queueids;
		encid = msm_enc_database.enc_info_list[idx].module_encid;
	}
	mutex_unlock(audpreproc->lock);
	return encid;
}
EXPORT_SYMBOL(audpreproc_aenc_alloc);

void audpreproc_aenc_free(int enc_id)
{
	struct audpreproc_state *audpreproc = &the_audpreproc_state;
	int idx;

	mutex_lock(audpreproc->lock);
	for (idx = 0; idx < msm_enc_database.num_enc; idx++) {
		if (msm_enc_database.enc_info_list[idx].module_encid ==
		    enc_id) {
			audpreproc->enc_inuse &= ~(1 << idx);
			break;
		}
	}
	mutex_unlock(audpreproc->lock);
	return;

}
EXPORT_SYMBOL(audpreproc_aenc_free);

int audpreproc_send_preproccmdqueue(void *cmd, unsigned len)
{
	return msm_adsp_write(the_audpreproc_state.mod,
				QDSP_uPAudPreProcCmdQueue, cmd, len);
}
EXPORT_SYMBOL(audpreproc_send_preproccmdqueue);

int audpreproc_send_audreccmdqueue(void *cmd, unsigned len)
{
	return msm_adsp_write(the_audpreproc_state.mod,
			      QDSP_uPAudPreProcAudRecCmdQueue, cmd, len);
}
EXPORT_SYMBOL(audpreproc_send_audreccmdqueue);
