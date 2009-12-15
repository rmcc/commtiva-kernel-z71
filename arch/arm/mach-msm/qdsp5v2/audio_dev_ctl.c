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
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/msm_audio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <mach/debug_audio_mm.h>
#include <mach/qdsp5v2/qdsp5audppmsg.h>
#include <mach/qdsp5v2/audpp.h>

#ifndef MAX
#define  MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif

struct audio_dev_ctrl_state {
	struct mutex lock;
	struct msm_snddev_info *devs[AUDIO_DEV_CTL_MAX_DEV];
	u32 num_dev;
	atomic_t opened;
	struct msm_snddev_info *voice_rx_dev;
	struct msm_snddev_info *voice_tx_dev;
	wait_queue_head_t      wait;
};

static struct audio_dev_ctrl_state audio_dev_ctrl;
struct event_listner event;

struct audio_routing_info {
	int running;
	unsigned short mixer_mask[6];
	unsigned short audrec_mixer_mask[2];
	int voice_tx_dev_id;
	int voice_rx_dev_id;
};

static struct audio_routing_info routing_info;

void msm_snddev_register(struct msm_snddev_info *dev_info)
{
	mutex_lock(&audio_dev_ctrl.lock);
	if (audio_dev_ctrl.num_dev < AUDIO_DEV_CTL_MAX_DEV) {
		audio_dev_ctrl.devs[audio_dev_ctrl.num_dev] = dev_info;
		dev_info->dev_volume = 0; /* 0 db */
		audio_dev_ctrl.num_dev++;
	} else
		MM_ERR("%s: device registry max out\n", __func__);
	mutex_unlock(&audio_dev_ctrl.lock);
}
EXPORT_SYMBOL(msm_snddev_register);

int msm_snddev_devcount(void)
{
	return audio_dev_ctrl.num_dev;
}
EXPORT_SYMBOL(msm_snddev_devcount);

int msm_snddev_query(int dev_id)
{
	if (dev_id <= audio_dev_ctrl.num_dev)
			return 0;
	return -ENODEV;
}
EXPORT_SYMBOL(msm_snddev_query);

int msm_snddev_is_set(int popp_id, int copp_id)
{
	return routing_info.mixer_mask[popp_id] & (0x1 << copp_id);
}
EXPORT_SYMBOL(msm_snddev_is_set);

unsigned short msm_snddev_route_enc(int enc_id)
{
	return routing_info.audrec_mixer_mask[enc_id];
}
EXPORT_SYMBOL(msm_snddev_route_enc);

unsigned short msm_snddev_route_dec(int popp_id)
{
	return routing_info.mixer_mask[popp_id];
}
EXPORT_SYMBOL(msm_snddev_route_dec);

int msm_snddev_set_dec(int popp_id, int copp_id, int set)
{
	if (set)
		routing_info.mixer_mask[popp_id] |= (0x1 << copp_id);
	else
		routing_info.mixer_mask[popp_id] &= ~(0x1 << copp_id);

	if (routing_info.running & (0x1 << popp_id))
		audpp_route_stream(popp_id, routing_info.mixer_mask[popp_id]);
	return 0;
}
EXPORT_SYMBOL(msm_snddev_set_dec);

int msm_snddev_set_enc(int popp_id, int copp_id, int set)
{
	if (set)
		routing_info.audrec_mixer_mask[popp_id] |= (0x1 << copp_id);
	else
		routing_info.audrec_mixer_mask[popp_id] &= ~(0x1 << copp_id);
	return 0;
}
EXPORT_SYMBOL(msm_snddev_set_enc);

int msm_set_voc_route(struct msm_snddev_info *dev_info,
			int stream_type, int dev_id)
{
	int rc = 0;

	mutex_lock(&audio_dev_ctrl.lock);
	switch (stream_type) {
	case AUDIO_ROUTE_STREAM_VOICE_RX:
		if (!(dev_info->capability & SNDDEV_CAP_RX) |
		    !(dev_info->capability & SNDDEV_CAP_VOICE)) {
			rc = -EINVAL;
			break;
		}
		audio_dev_ctrl.voice_rx_dev = dev_info;
		routing_info.voice_rx_dev_id = dev_id;
		break;
	case AUDIO_ROUTE_STREAM_VOICE_TX:
		if (!(dev_info->capability & SNDDEV_CAP_TX) |
		    !(dev_info->capability & SNDDEV_CAP_VOICE)) {
			rc = -EINVAL;
			break;
		}
		audio_dev_ctrl.voice_tx_dev = dev_info;
		routing_info.voice_tx_dev_id = dev_id;
		break;
	}
	mutex_unlock(&audio_dev_ctrl.lock);
	return rc;
}
EXPORT_SYMBOL(msm_set_voc_route);

void msm_release_voc_thread(void)
{
	wake_up(&audio_dev_ctrl.wait);
}
EXPORT_SYMBOL(msm_release_voc_thread);

int msm_get_voc_route(u32 *rx_id, u32 *tx_id)
{
	int rc = 0;

	if (!rx_id || !tx_id)
		return -EINVAL;

	mutex_lock(&audio_dev_ctrl.lock);
	if (!audio_dev_ctrl.voice_rx_dev || !audio_dev_ctrl.voice_tx_dev) {
		rc = -ENODEV;
		mutex_unlock(&audio_dev_ctrl.lock);
		return rc;
	}

	*rx_id = audio_dev_ctrl.voice_rx_dev->acdb_id;
	*tx_id = audio_dev_ctrl.voice_tx_dev->acdb_id;

	mutex_unlock(&audio_dev_ctrl.lock);

	return rc;
}
EXPORT_SYMBOL(msm_get_voc_route);

struct msm_snddev_info *audio_dev_ctrl_find_dev(u32 dev_id)
{
	struct msm_snddev_info *info;

	if ((audio_dev_ctrl.num_dev - 1) < dev_id) {
		info = ERR_PTR(-ENODEV);
		goto error;
	}

	info = audio_dev_ctrl.devs[dev_id];
error:
	return info;

}
EXPORT_SYMBOL(audio_dev_ctrl_find_dev);

int snddev_voice_set_volume(int vol, int path)
{
	if (audio_dev_ctrl.voice_rx_dev
		&& audio_dev_ctrl.voice_tx_dev) {
		if (path)
			audio_dev_ctrl.voice_tx_dev->dev_volume = vol;
		else
			audio_dev_ctrl.voice_rx_dev->dev_volume = vol;
	} else
		return -ENODEV;
	return 0;
}
EXPORT_SYMBOL(snddev_voice_set_volume);

static int audio_dev_ctrl_get_devices(struct audio_dev_ctrl_state *dev_ctrl,
				      void __user *arg)
{
	int rc = 0;
	u32 index;
	struct msm_snd_device_list work_list;
	struct msm_snd_device_info *work_tbl;

	if (copy_from_user(&work_list, arg, sizeof(work_list))) {
		rc = -EFAULT;
		goto error;
	}

	if (work_list.num_dev > dev_ctrl->num_dev) {
		rc = -EINVAL;
		goto error;
	}

	work_tbl = kmalloc(work_list.num_dev *
		sizeof(struct msm_snd_device_info), GFP_KERNEL);
	if (!work_tbl) {
		rc = -ENOMEM;
		goto error;
	}

	for (index = 0; index < dev_ctrl->num_dev; index++) {
		work_tbl[index].dev_id = index;
		work_tbl[index].dev_cap = dev_ctrl->devs[index]->capability;
		strlcpy(work_tbl[index].dev_name, dev_ctrl->devs[index]->name,
		64);
	}

	if (copy_to_user((void *) (work_list.list), work_tbl,
		 work_list.num_dev * sizeof(struct msm_snd_device_info)))
		rc = -EFAULT;
	kfree(work_tbl);
error:
	return rc;
}


int auddev_register_evt_listner(u32 evt_id, u32 clnt_type, u32 clnt_id,
		void (*listner)(u32 evt_id,
			union auddev_evt_data *evt_payload,
			void *private_data),
		void *private_data)
{
	int rc;
	struct msm_snd_evt_listner *callback = NULL;
	struct msm_snd_evt_listner *new_cb;

	new_cb = kzalloc(sizeof(struct msm_snd_evt_listner), GFP_KERNEL);
	if (!new_cb) {
		MM_ERR("%s: listner registry max out\n", __func__);
		return -ENOMEM;
	}

	mutex_lock(&event.listner_lock);
	new_cb->cb_next = NULL;
	new_cb->auddev_evt_listener = listner;
	new_cb->evt_id = evt_id;
	new_cb->clnt_type = clnt_type;
	new_cb->clnt_id = clnt_id;
	new_cb->private_data = private_data;
	if (event.cb == NULL) {
		event.cb = new_cb;
		new_cb->cb_prev = NULL;
	} else {
		callback = event.cb;
		for (; ;) {
			if (callback->cb_next == NULL)
				break;
			else {
				callback = callback->cb_next;
				continue;
			}
		}
		callback->cb_next = new_cb;
		new_cb->cb_prev = callback;
	}
	event.num_listner++;
	mutex_unlock(&event.listner_lock);
	MM_ERR("added one listner handle\n");
	rc = 0;
	return rc;
}
EXPORT_SYMBOL(auddev_register_evt_listner);

int auddev_unregister_evt_listner(u32 clnt_type, u32 clnt_id)
{
	struct msm_snd_evt_listner *callback = event.cb;

	while (callback != NULL) {
		if ((callback->clnt_type == clnt_type)
			&& (callback->clnt_id == clnt_id))
			break;
		 callback = callback->cb_next;
	}
	if (callback == NULL)
		return -EINVAL;

	if ((callback->cb_next == NULL) && (callback->cb_prev == NULL))
		event.cb = NULL;
	else if (callback->cb_next == NULL)
		callback->cb_prev->cb_next = NULL;
	else if (callback->cb_prev == NULL) {
		callback->cb_next->cb_prev = NULL;
		event.cb = callback->cb_next;
	} else {
		callback->cb_prev->cb_next = callback->cb_next;
		callback->cb_next->cb_prev = callback->cb_prev;
	}
	kfree(callback);
	return 0;
}
EXPORT_SYMBOL(auddev_unregister_evt_listner);

int msm_snddev_request_freq(int *freq, u32 session_id, u32 clnt_type)
{
	int i = 0;
	int rc = 0;
	u32 capability = 0;
	struct msm_snddev_info *info;
	u32 set_freq;
	int tmp_freq_index = 0;

	if (clnt_type == AUDDEV_CLNT_VOC)
		capability = SNDDEV_CAP_VOICE;
	else if (clnt_type == AUDDEV_CLNT_DEC)
		capability = SNDDEV_CAP_RX;
	else if (clnt_type == AUDDEV_CLNT_ENC)
		capability = SNDDEV_CAP_TX;

	for (i = 0; i < audio_dev_ctrl.num_dev; i++) {
		info = audio_dev_ctrl.devs[i];
		if ((info->sessions & session_id)
			&& (info->capability & capability)) {
			if (*freq % 8000) {
				tmp_freq_index = *freq/8000;
				*freq = (tmp_freq_index * 8000) + 8000;
			}
			set_freq = MAX(*freq, info->sample_rate);
			rc = info->dev_ops.set_freq(info, set_freq);
			if (rc >= 0) {
				info->sample_rate = rc;
				*freq = info->sample_rate;
				return rc;
			}
		}
	}
	return rc;
}
EXPORT_SYMBOL(msm_snddev_request_freq);

static int audio_dev_ctrl_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	struct audio_dev_ctrl_state *dev_ctrl = file->private_data;

	mutex_lock(&audio_dev_ctrl.lock);
	switch (cmd) {
	case AUDIO_GET_NUM_SND_DEVICE:
		rc = put_user(dev_ctrl->num_dev, (uint32_t __user *) arg);
		break;
	case AUDIO_GET_SND_DEVICES:
		rc = audio_dev_ctrl_get_devices(dev_ctrl, (void __user *) arg);
		break;
	case AUDIO_ENABLE_SND_DEVICE: {
		struct msm_snddev_info *dev_info;
		u32 dev_id;

		if (get_user(dev_id, (u32 __user *) arg)) {
			rc = -EFAULT;
			break;
		}
		dev_info = audio_dev_ctrl_find_dev(dev_id);
		if (IS_ERR(dev_info))
			rc = PTR_ERR(dev_info);
		else {
			rc = dev_info->dev_ops.open(dev_info);
			if (!rc)
				dev_info->opened = 1;
			wake_up(&audio_dev_ctrl.wait);
		}
		break;

	}

	case AUDIO_DISABLE_SND_DEVICE: {
		struct msm_snddev_info *dev_info;
		u32 dev_id;

		if (get_user(dev_id, (u32 __user *) arg)) {
			rc = -EFAULT;
			break;
		}
		dev_info = audio_dev_ctrl_find_dev(dev_id);
		if (IS_ERR(dev_info))
			rc = PTR_ERR(dev_info);
		else {
			rc = dev_info->dev_ops.close(dev_info);
			dev_info->opened = 0;
		}
		break;
	}

	case AUDIO_ROUTE_STREAM: {
		struct msm_audio_route_config route_cfg;
		struct msm_snddev_info *dev_info;

		if (copy_from_user(&route_cfg, (void __user *) arg,
			sizeof(struct msm_audio_route_config))) {
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: route cfg %d %d type\n", __func__,
		route_cfg.dev_id, route_cfg.stream_type);
		dev_info = audio_dev_ctrl_find_dev(route_cfg.dev_id);
		if (IS_ERR(dev_info)) {
			MM_ERR("%s: pass invalid dev_id\n", __func__);
			rc = PTR_ERR(dev_info);
			break;
		}

		switch (route_cfg.stream_type) {

		case AUDIO_ROUTE_STREAM_VOICE_RX:
			if (!(dev_info->capability & SNDDEV_CAP_RX) |
			    !(dev_info->capability & SNDDEV_CAP_VOICE)) {
				rc = -EINVAL;
				break;
			}
			dev_ctrl->voice_rx_dev = dev_info;
			break;
		case AUDIO_ROUTE_STREAM_VOICE_TX:
			if (!(dev_info->capability & SNDDEV_CAP_TX) |
			    !(dev_info->capability & SNDDEV_CAP_VOICE)) {
				rc = -EINVAL;
				break;
			}
			dev_ctrl->voice_tx_dev = dev_info;
			break;
		}
		break;
	}

	default:
		rc = -EINVAL;
	}
	mutex_unlock(&audio_dev_ctrl.lock);
	return rc;
}

static int audio_dev_ctrl_open(struct inode *inode, struct file *file)
{
	MM_DBG("open audio_dev_ctrl\n");
	atomic_inc(&audio_dev_ctrl.opened);
	file->private_data = &audio_dev_ctrl;
	return 0;
}

static int audio_dev_ctrl_release(struct inode *inode, struct file *file)
{
	MM_DBG("release audio_dev_ctrl\n");
	atomic_dec(&audio_dev_ctrl.opened);
	return 0;
}

static const struct file_operations audio_dev_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = audio_dev_ctrl_open,
	.release = audio_dev_ctrl_release,
	.ioctl = audio_dev_ctrl_ioctl,
};


struct miscdevice audio_dev_ctrl_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_audio_dev_ctrl",
	.fops	= &audio_dev_ctrl_fops,
};

static void broadcast_event(u32 evt_id, u32 dev_id)
{
	int clnt_id = 0;
	union auddev_evt_data *evt_payload;
	struct msm_snd_evt_listner *callback;

	if (event.cb != NULL)
		callback = event.cb;
	else {
		MM_ERR("returning as event.cb is NULL\n");
		return;
	}
	evt_payload = kzalloc(sizeof(union auddev_evt_data),
			GFP_KERNEL);

	for (; ;) {
		if (!(evt_id & callback->evt_id)) {
			if (callback->cb_next == NULL)
				break;
			else {
				callback = callback->cb_next;
				continue;
			}
		}

		memset(evt_payload, 0, sizeof(union auddev_evt_data));
		clnt_id = callback->clnt_id;
		if (callback->clnt_type == AUDDEV_CLNT_DEC) {
			evt_payload->routing_id =
				routing_info.mixer_mask[clnt_id];
			callback->auddev_evt_listener(
				evt_id,
				evt_payload,
				callback->private_data);
			if (callback->cb_next == NULL)
				break;
			else {
				callback = callback->cb_next;
				continue;
			}
		}
		if (callback->clnt_type == AUDDEV_CLNT_ENC) {
			evt_payload->routing_id =
				routing_info.audrec_mixer_mask[clnt_id];
			callback->auddev_evt_listener(
				evt_id,
				evt_payload,
				callback->private_data);
			if (callback->cb_next == NULL)
				break;
			else {
				callback = callback->cb_next;
				continue;
			}
		}
		if (callback->clnt_type == AUDDEV_CLNT_VOC) {
			if ((!audio_dev_ctrl.voice_tx_dev)
				|| (!audio_dev_ctrl.voice_rx_dev))
				goto sent_voc;


			evt_payload->voc_devinfo.rxdev_id =
				audio_dev_ctrl.voice_rx_dev->acdb_id;
			evt_payload->voc_devinfo.txdev_id =
				audio_dev_ctrl.voice_tx_dev->acdb_id;
			evt_payload->voc_devinfo.rxdev_vol =
				audio_dev_ctrl.voice_rx_dev->dev_volume;
			evt_payload->voc_devinfo.txdev_vol =
				audio_dev_ctrl.voice_tx_dev->dev_volume;
			evt_payload->voc_devinfo.rxdev_sample =
				audio_dev_ctrl.voice_rx_dev->sample_rate;
			evt_payload->voc_devinfo.txdev_sample =
				audio_dev_ctrl.voice_tx_dev->sample_rate;
			evt_payload->voc_devinfo.rx_mute = 0;
			evt_payload->voc_devinfo.tx_mute = 0;

			if (evt_id == AUDDEV_EVT_REL_PENDING) {
				if (audio_dev_ctrl.voice_tx_dev->opened &&
				    audio_dev_ctrl.voice_rx_dev->opened) {
					callback->auddev_evt_listener(
					evt_id,	evt_payload,
					callback->private_data);
				}
				goto sent_voc;
			}
			if (evt_id == AUDDEV_EVT_DEV_RLS) {
				if ((!audio_dev_ctrl.voice_tx_dev->opened) &&
					(!audio_dev_ctrl.voice_rx_dev->opened)
					) {
					callback->auddev_evt_listener(
					evt_id,	evt_payload,
					callback->private_data);
				}
				goto sent_voc;
			}

			if (evt_id == AUDDEV_EVT_DEV_RDY) {
				if (!audio_dev_ctrl.voice_rx_dev->opened)
					goto sent_voc;
				if (!audio_dev_ctrl.voice_tx_dev->opened)
					goto sent_voc;
			}
			callback->auddev_evt_listener(
				evt_id,
				evt_payload,
				callback->private_data);
sent_voc:
			if (callback->cb_next == NULL)
				break;
			else {
				callback = callback->cb_next;
				continue;
			}
		}
		if (callback->clnt_type == AUDDEV_CLNT_AUDIOCAL) {
			struct msm_snddev_info *dev_info;

			dev_info = audio_dev_ctrl_find_dev(dev_id);
			evt_payload->audcal_info.dev_id = dev_id;
			evt_payload->audcal_info.acdb_id =
				audio_dev_ctrl.voice_rx_dev->acdb_id;
			evt_payload->audcal_info.sample_rate = 8000;
			evt_payload->audcal_info.dev_type =
				audio_dev_ctrl.voice_rx_dev->capability;

			callback->auddev_evt_listener(
				evt_id,
				evt_payload,
				callback->private_data);
			if (callback->cb_next == NULL)
				break;
			else {
				callback = callback->cb_next;
				continue;
			}
		}
	}
}


void mixer_post_event(u32 evt_id, u32 dev_id)
{
	u32 voc_tx_id, voc_rx_id, mixer_mask;
	static int concurrent;
	int current_dec_concurrency = 0;
	int current_enc_concurrency = 0;
	int i = 0;

	MM_ERR("%s: evt_id = %d\n", __func__, evt_id);
	mutex_lock(&event.listner_lock);
	switch (evt_id) {
	case AUDDEV_EVT_DEV_CHG_AUDIO:
		broadcast_event(AUDDEV_EVT_DEV_CHG_AUDIO, dev_id);
		break;
	case AUDDEV_EVT_DEV_CHG_VOICE: /* Called from Voice_route */
		if ((!audio_dev_ctrl.voice_tx_dev)
			|| (!audio_dev_ctrl.voice_rx_dev)) {
			break;
		}
		voc_tx_id = audio_dev_ctrl.voice_tx_dev->copp_id;
		/* TODO: check the default value */
		voc_rx_id = audio_dev_ctrl.voice_rx_dev->copp_id;
		for (i = 0; i < 6; i++)	{
			mixer_mask = routing_info.mixer_mask[i];
			if (((0x1 << voc_tx_id) & mixer_mask) ||
				((0x1 << voc_rx_id) & mixer_mask)) {
				evt_id = AUDDEV_EVT_DEV_CONCURRENT;
				current_dec_concurrency = 1;
			} else if (!current_dec_concurrency)
				current_dec_concurrency = 0;
		}
		for (i = 0; i < 2; i++)	{
			mixer_mask = routing_info.audrec_mixer_mask[i];
			if (((0x1 << voc_tx_id) & mixer_mask) ||
				((0x1 << voc_rx_id) & mixer_mask)) {
				evt_id = AUDDEV_EVT_DEV_CONCURRENT;
				current_enc_concurrency = 1;
			} else if (!current_enc_concurrency)
				current_enc_concurrency = 0;
		}
		broadcast_event(AUDDEV_EVT_DEV_CHG_VOICE, dev_id);

		if (current_dec_concurrency || current_enc_concurrency) {
			broadcast_event(AUDDEV_EVT_DEV_CONCURRENT, dev_id);
			concurrent = 1;
		} else if (concurrent) {
			broadcast_event(AUDDEV_EVT_DEV_NON_CONCURRENT, dev_id);
			concurrent = 0;
		}
		break;
	case AUDDEV_EVT_DEV_RDY:
		broadcast_event(AUDDEV_EVT_DEV_RDY, dev_id);
		break;
	case AUDDEV_EVT_DEV_RLS:
		broadcast_event(AUDDEV_EVT_DEV_RLS, dev_id);
		break;
	case AUDDEV_EVT_REL_PENDING:
		broadcast_event(AUDDEV_EVT_REL_PENDING, dev_id);
		break;
	case AUDDEV_EVT_DEVICE_VOL_MUTE_CHG:
		broadcast_event(AUDDEV_EVT_DEVICE_VOL_MUTE_CHG, dev_id);
		break;
	default:
		break;
	}
	mutex_unlock(&event.listner_lock);
}
EXPORT_SYMBOL(mixer_post_event);

static int __init audio_dev_ctrl_init(void)
{
	mutex_init(&audio_dev_ctrl.lock);
	mutex_init(&event.listner_lock);
	init_waitqueue_head(&audio_dev_ctrl.wait);

	event.cb = NULL;

	atomic_set(&audio_dev_ctrl.opened, 0);
	audio_dev_ctrl.num_dev = 0;
	audio_dev_ctrl.voice_tx_dev = NULL;
	audio_dev_ctrl.voice_rx_dev = NULL;
	return misc_register(&audio_dev_ctrl_misc);
}

static void __exit audio_dev_ctrl_exit(void)
{
}
module_init(audio_dev_ctrl_init);
module_exit(audio_dev_ctrl_exit);

MODULE_DESCRIPTION("MSM 7K Audio Device Control driver");
MODULE_LICENSE("Dual BSD/GPL");
