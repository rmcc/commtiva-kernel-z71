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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/msm_audio.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/dal.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <mach/qdsp5v2/voice.h>
#include <mach/debug_audio_mm.h>

struct voice_data {
	void *handle; /* DALRPC handle */
	void *cb_handle; /* DALRPC callback handle */
	int network; /* Network information */
	int opened;
	int dev_state;/*Ready, Change, Release, Concurrent*/
	int voc_state;
	struct mutex lock;
	int voc_event;
	int dev_event;
	int freq;
	atomic_t rel_start_flag;
	struct auddev_evt_voc_devinfo dev_info;
	struct task_struct *task;
	struct completion complete;
	wait_queue_head_t dev_wait;
	wait_queue_head_t voc_wait;
	uint32_t device_events;
};

static struct voice_data voice;

static int voice_cmd_device_info(struct voice_data *);
static int voice_cmd_acquire_done(struct voice_data *);
static void voice_auddev_cb_function(u32 evt_id,
			union auddev_evt_data *evt_payload,
			void *private_data);

static int voice_cmd_change(void)
{

	struct voice_header hdr;
	struct voice_data *v = &voice;
	int err;

	hdr.id = CMD_DEVICE_CHANGE;
	hdr.data_len = 0;

	MM_INFO("%s()\n", __func__);

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &hdr,
			sizeof(struct voice_header));

	if (err)
		MM_ERR("Voice change command failed\n");
	return err;
}

static void voice_auddev_cb_function(u32 evt_id,
			union auddev_evt_data *evt_payload,
			void *private_data)
{
	struct voice_data *v = &voice;
	int rc = 0;

	MM_INFO("auddev_cb_function, evt_id=%d, dev_state=%d\n",
		evt_id, v->dev_state);
	switch (evt_id) {
	case AUDDEV_EVT_DEV_CHG_VOICE:
		if ((v->dev_state == DEV_READY) ||
			(v->dev_state == DEV_CONCUR) ||
			(v->dev_state == DEV_CHANGE)) {
			v->dev_state = DEV_CHANGE;
			if (v->voc_state == VOICE_ACQUIRE) {
				/* send device change to modem */
				voice_cmd_change();
				/* block to wait for DEVICE START */
				rc = wait_event_interruptible(
				v->voc_wait, v->voc_state == VOICE_CHANGE);
			} else
				MM_ERR(" Voice is not at ACQUIRE state\n");
		} else
			MM_ERR(" device is not at READY or CONCUR state\n");
		break;
	case AUDDEV_EVT_DEV_RDY:
		/* update the dev info */
		memcpy((void *)&v->dev_info,
			(void *)&evt_payload->voc_devinfo,
			sizeof(struct auddev_evt_voc_devinfo));
		if (v->dev_state == DEV_CHANGE) {
			mutex_lock(&voice.lock);
			v->dev_event = DEV_CHANGE_READY;
			mutex_unlock(&voice.lock);
			complete(&v->complete);
			v->dev_state = DEV_READY;
		} else if ((v->dev_state == DEV_INIT) ||
			(v->dev_state == DEV_REL_DONE)) {
				v->dev_state = DEV_READY;
				wake_up(&v->dev_wait);
		} else
			MM_ERR("Receive READY not at the proper state =%d\n",
				v->dev_state);
		break;
	case AUDDEV_EVT_DEV_RLS:
		if (v->dev_state != DEV_CHANGE) {
			v->dev_state = DEV_REL_DONE;
			wake_up(&v->dev_wait);
		}
		break;
	case AUDDEV_EVT_DEV_CONCURRENT:
		if (v->dev_state == DEV_READY)
			v->dev_state = DEV_CONCUR;
		else if (v->dev_state == DEV_INIT) {
			memcpy((void *)&v->dev_info,
				(void *)&evt_payload->voc_devinfo,
				sizeof(struct auddev_evt_voc_devinfo));
				wake_up(&v->dev_wait);
		} else
			MM_ERR("Receivee CONCUR not at the proper state =%d\n",
				v->dev_state);
		break;
	case AUDDEV_EVT_DEV_NON_CONCURRENT:
		if (v->dev_state == DEV_CONCUR)
			v->dev_state = DEV_READY;
		break;
	case AUDDEV_EVT_DEVICE_VOL_MUTE_CHG:
		/* send device info */
		memcpy((void *)&v->dev_info,
			(void *)&evt_payload->voc_devinfo,
			sizeof(struct auddev_evt_voc_devinfo));
		voice_cmd_device_info(v);
		break;
	case AUDDEV_EVT_REL_PENDING:
		/* check if release start is reveiced */
		if (v->dev_state == DEV_CHANGE)
			break;
		else if (v->dev_state != DEV_REL_DONE) {
			if (atomic_read(&v->rel_start_flag)) {
				atomic_dec(&v->rel_start_flag);
				break;
			} else if ((v->voc_state == VOICE_RELEASE) ||
					(v->voc_state == VOICE_INIT))
				break;
			else {
				rc = wait_event_interruptible(
				v->voc_wait,
				atomic_read(&v->rel_start_flag) == 1);
				atomic_dec(&v->rel_start_flag);
				break;
			}
		}

	default:
		MM_ERR("UNKNOWN EVENT\n");
	}
	return;
}
EXPORT_SYMBOL(voice_auddev_cb_function);

static void remote_cb_function(void *context, u32 param,
				void *evt_buf, u32 len)
{
	struct voice_header *hdr;
	struct voice_data *v = context;

	hdr = (struct voice_header *)evt_buf;

	MM_INFO("%s() len=%d id=%d\n", __func__, len, hdr->id);

	if (len <= 0) {
		MM_ERR("unexpected event with length %d \n", len);
		return;
	}

	switch (hdr->id) {
	case EVENT_ACQUIRE_START:
		mutex_lock(&voice.lock);
		v->voc_event = VOICE_ACQUIRE_START;
		v->network = ((struct voice_network *)evt_buf)->network_info;
		MM_INFO(" network=%d\n", v->network);
		mutex_unlock(&voice.lock);
		complete(&v->complete);
		break;
	case EVENT_RELEASE_START:
		/* If ACQUIRED come in before the RELEASE,
		* will only services the RELEASE */
		atomic_inc(&v->rel_start_flag);
		wake_up(&v->voc_wait);
		wake_up(&v->dev_wait);
		mutex_lock(&voice.lock);
		v->voc_event = VOICE_RELEASE_START;
		mutex_unlock(&voice.lock);
		complete(&v->complete);
		break;
	case EVENT_CHANGE_START:
		MM_INFO(" receive Change Start \n");
		mutex_lock(&voice.lock);
		v->voc_event = VOICE_CHANGE_START;
		mutex_unlock(&voice.lock);
		complete(&v->complete);
		break;
	case EVENT_NETWORK_RECONFIG:
		/* send network change to audio_dev,
		if sample rate is less than 16k,
		otherwise, send acquire done */
		v->voc_event = VOICE_NETWORK_RECONFIG;
		v->network = ((struct voice_network *)evt_buf)->network_info;
		complete(&v->complete);
		break;
	default:
		MM_ERR("Undefined event %d \n", hdr->id);
	}

}

static int voice_cmd_init(struct voice_data *v)
{

	struct voice_init cmd;
	int err;

	MM_INFO("%s()\n", __func__);

	cmd.hdr.id = CMD_VOICE_INIT;
	cmd.hdr.data_len = sizeof(struct voice_init) -
				sizeof(struct voice_header);
	cmd.cb_handle = v->cb_handle;

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &cmd,
			 sizeof(struct voice_init));

	if (err)
		MM_ERR("Voice init command failed\n");
	return err;
}

static int voice_cmd_acquire_done(struct voice_data *v)
{
	struct voice_header hdr;
	int err;

	hdr.id = CMD_ACQUIRE_DONE;
	hdr.data_len = 0;

	MM_INFO("%s()\n", __func__);

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &hdr,
			 sizeof(struct voice_header));

	if (err)
		MM_ERR("Voice acquire done command failed\n");
	return err;
}

static int voice_cmd_release_done(struct voice_data *v)
{
	struct voice_header hdr;
	int err;

	MM_INFO("%s()\n", __func__);

	hdr.id = CMD_RELEASE_DONE;
	hdr.data_len = 0;

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &hdr,
		sizeof(struct voice_header));

	if (err)
		MM_ERR("Voice release done command failed\n");
	return err;
}

static int voice_cmd_device_info(struct voice_data *v)
{
	struct voice_device cmd;
	int err;

	MM_INFO("%s(), tx_dev=%d, rx_dev=%d, tx_sample=%d, rx_sample=%d\n",
	__func__, v->dev_info.txdev_id, v->dev_info.rxdev_id,
	v->dev_info.txdev_sample, v->dev_info.rxdev_sample);

	cmd.hdr.id = CMD_DEVICE_INFO;
	cmd.hdr.data_len = sizeof(struct voice_device) -
			sizeof(struct voice_header);
	cmd.tx_device = v->dev_info.txdev_id;
	cmd.rx_device = v->dev_info.rxdev_id;
	cmd.rx_volume = v->dev_info.rxdev_vol;
	cmd.rx_mute = v->dev_info.rx_mute;
	cmd.tx_mute = v->dev_info.tx_mute;
	cmd.rx_sample = v->dev_info.rxdev_sample/1000;
	cmd.tx_sample = v->dev_info.txdev_sample/1000;

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &cmd,
			 sizeof(struct voice_device));

	if (err)
		MM_ERR("Voice device command failed\n");
	return err;
}
EXPORT_SYMBOL(voice_cmd_device_info);

void voice_change_sample_rate(struct voice_data *v)
{
	int freq = 8000;
	int rc = 0;

	if (v->network == NETWORK_WCDMA_WB)
		freq = 16000;
	MM_INFO(" network =%d, vote freq=%d\n", v->network, freq);
	if (freq != v->dev_info.txdev_sample) {
		rc = msm_snddev_request_freq(&freq, 0, AUDDEV_CLNT_VOC);
		if (rc >= 0) {
			v->dev_info.txdev_sample = freq;
			MM_INFO(" vote for freq=%d successfully \n", freq);
		} else
			MM_ERR(" voting for freq=%d failed.\n", freq);
	}
}

static int voice_thread(void *data)
{
	struct voice_data *v = (struct voice_data *)data;
	int rc = 0;

	MM_INFO("voice_thread() start\n");

	while (!kthread_should_stop()) {
		wait_for_completion(&v->complete);
		init_completion(&v->complete);

		MM_INFO(" voc_event=%d, voice state =%d, dev_event=%d/n",
				v->voc_event, v->voc_state, v->dev_event);
		switch (v->voc_event) {
		case VOICE_ACQUIRE_START:
			/* check if dev_state = READY */
			/* if ready, send device_info and acquire_done */
			/* if not ready, block to wait the dev_state = READY */
			if ((v->voc_state == VOICE_INIT) ||
				(v->voc_state == VOICE_RELEASE)) {
				if ((v->dev_state == DEV_READY)
					|| (v->dev_state == DEV_CONCUR)) {
					voice_change_sample_rate(v);
					rc = voice_cmd_device_info(v);
					rc = voice_cmd_acquire_done(v);
					v->voc_state = VOICE_ACQUIRE;
				} else {
					rc = wait_event_interruptible(
					v->dev_wait,
					(v->dev_state == DEV_READY)
					|| (v->dev_state == DEV_CONCUR)
					|| (atomic_read(&v->rel_start_flag)
						== 1));
					if (atomic_read(&v->rel_start_flag)
						== 1) {
						voice_cmd_release_done(v);
						v->voc_state = VOICE_RELEASE;
						atomic_dec(&v->rel_start_flag);
					} else {
						voice_change_sample_rate(v);
						rc = voice_cmd_device_info(v);
						rc = voice_cmd_acquire_done(v);
						v->voc_state = VOICE_ACQUIRE;
					}
				}
			} else
				MM_ERR("Get this event at the wrong state\n");
			break;
		case VOICE_RELEASE_START:
			voice_cmd_release_done(v);
			v->voc_state = VOICE_RELEASE;
			if (atomic_read(&v->rel_start_flag))
				atomic_dec(&v->rel_start_flag);
			break;
		case VOICE_CHANGE_START:
			if ((v->voc_state == VOICE_ACQUIRE)
				|| (v->dev_state == DEV_CHANGE)) {
				v->voc_state = VOICE_CHANGE;
				wake_up(&v->voc_wait);
			} else
				MM_ERR("Get this event at the wrong state\n");
			break;
		case VOICE_NETWORK_RECONFIG:
			if ((v->voc_state == VOICE_ACQUIRE)
				|| (v->voc_state == VOICE_CHANGE)) {
				voice_change_sample_rate(v);
				rc = voice_cmd_device_info(v);
				rc = voice_cmd_acquire_done(v);
			}
			break;
		default:
			break;
		}

		switch (v->dev_event) {
		case DEV_CHANGE_READY:
			if (v->voc_state == VOICE_CHANGE) {
				/* send device info to modem */
				voice_cmd_device_info(v);
				/* update voice state */
				v->voc_state = VOICE_ACQUIRE;
			} else
				MM_ERR("Get this event at the wrong state\n");
			break;
		default:
			break;
		}
		mutex_lock(&voice.lock);
		v->dev_event = 0;
		v->voc_event = 0;
		mutex_unlock(&voice.lock);
	}
	return 0;
}

static int __init voice_init(void)
{
	int rc;
	struct voice_data *v = &voice;
	MM_INFO("%s\n", __func__);

	mutex_init(&voice.lock);
	v->opened = 0;
	v->handle = NULL;
	v->cb_handle = NULL;
	/* assign the default value to dev_info */
	v->dev_info.txdev_id = 0;
	v->dev_info.rxdev_id = 1;
	v->dev_info.rxdev_vol = 0;
	v->dev_info.txdev_sample = 8000; /* 8k */
	v->dev_info.rxdev_sample = 8000; /* 8k */
	v->dev_info.rx_mute = 0;
	v->dev_info.tx_mute = 0;

	v->dev_state = DEV_INIT;
	v->voc_state = VOICE_INIT;
	atomic_set(&v->rel_start_flag, 0);
	v->dev_event = 0;
	v->voc_event = 0;
	init_completion(&voice.complete);
	init_waitqueue_head(&v->dev_wait);
	init_waitqueue_head(&v->voc_wait);

	 /* get device handle */
	rc = daldevice_attach(VOICE_DALRPC_DEVICEID,
				VOICE_DALRPC_PORT_NAME,
				VOICE_DALRPC_CPU,
				&v->handle);
	if (rc) {
		MM_ERR("Voc DALRPC call to Modem attach failed\n");
		goto done;
	}

	/* Allocate the callback handle */
	v->cb_handle = dalrpc_alloc_cb(v->handle, remote_cb_function, v);
	if (v->cb_handle == NULL) {
		MM_ERR("Allocate Callback failure\n");
		goto err;
	}

	/* setup the callback */
	rc = voice_cmd_init(v);
	if (rc)
		goto err1;

	v->device_events = AUDDEV_EVT_DEV_CHG_VOICE |
			AUDDEV_EVT_DEV_RDY |
			AUDDEV_EVT_DEV_RLS |
			AUDDEV_EVT_DEV_CONCURRENT |
			AUDDEV_EVT_DEV_NON_CONCURRENT |
			AUDDEV_EVT_REL_PENDING |
			AUDDEV_EVT_DEVICE_VOL_MUTE_CHG;

	/* register callback to auddev */
	auddev_register_evt_listner(v->device_events, AUDDEV_CLNT_VOC,
				0, voice_auddev_cb_function, v);

	/* create and start thread */
	v->task = kthread_run(voice_thread, v, "voice");
	if (IS_ERR(v->task)) {
		rc = PTR_ERR(v->task);
		v->task = NULL;
	} else
		goto done;

err1:   dalrpc_dealloc_cb(v->handle, v->cb_handle);
err:
	daldevice_detach(v->handle);
	v->handle = NULL;
done:
	return rc;
}

late_initcall(voice_init);
