/* sound/soc/msm/msm-pcm.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2008 HTC Corporation
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */
#define DEBUG

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>

#include "msm7kv2-pcm.h"

#define MAX_DATA_SIZE 496
#define AUDPP_ALSA_DECODER	(-1)


#define DB_TABLE_INDEX		(50)

#define audplay_send_queue0(audio, cmd, len) \
	msm_adsp_write(audio->audplay, audio->queue_id, \
			cmd, len)

int intcnt;
static int audio_dsp_send_buffer(struct msm_audio *prtd,
			unsigned idx, unsigned len);
static void alsa_adec_params(struct msm_audio *prtd);

struct audio_frame {
	uint16_t count_low;
	uint16_t count_high;
	uint16_t bytes;
	uint16_t unknown;
	unsigned char samples[];
} __attribute__ ((packed));

static void alsa_audplay_event(void *data, unsigned id, size_t len,
		void (*getevent) (void *ptr, size_t len))
{
	struct msm_audio *prtd = data;
	struct buffer *frame;
	unsigned long flag;

	pr_info("alsa_audplay_event: msg_id=%x\n", id);

	switch (id) {
	case AUDPLAY_MSG_DEC_NEEDS_DATA:
		pr_debug("%s()AUDPLAY_MSG_DEC_NEEDS_DATA\n", __func__);
		/* Update with actual sent buffer size */
		if (prtd->out[prtd->out_tail].used == BUF_INVALID_LEN) {
			prtd->pcm_irq_pos += prtd->out[prtd->out_tail].used;
			prtd->out[prtd->out_tail].used = 0;
			prtd->out_tail ^= 1;
			wake_up(&the_locks.write_wait);
		}

		if (prtd->pcm_irq_pos > prtd->pcm_size)
			prtd->pcm_irq_pos = prtd->pcm_count;

		if (prtd->ops->playback)
			prtd->ops->playback(prtd);

		spin_lock_irqsave(&the_locks.write_dsp_lock, flag);
		if (prtd->running) {
			prtd->out[prtd->out_tail].used = 0;
			frame = prtd->out + prtd->out_tail;
			if (frame->used) {
				audio_dsp_send_buffer(prtd, prtd->out_tail,
					      frame->used);
				frame->used = BUF_INVALID_LEN;
			} else {
				prtd->out_needed++;
			}
		}
		spin_unlock_irqrestore(&the_locks.write_dsp_lock, flag);
		break;
	default:
		pr_info("unexpected message from decoder \n");
		break;
	}
}

struct msm_adsp_ops alsa_adsp_ops = {
	.event = alsa_audplay_event
};

void alsa_dsp_event(void *data, unsigned id, uint16_t *msg)
{
	struct msm_audio *prtd = data;
	pr_info("alsa_dsp_event: msg_id=%x\n", id);

	switch (id) {
	case AUDPP_MSG_STATUS_MSG: {
			unsigned status = msg[1];
			switch (status) {
			case AUDPP_DEC_STATUS_SLEEP: {
				uint16_t reason = msg[2];
				pr_info("decoder status:sleep reason=0x%04x\n",
						reason);
				if ((reason == AUDPP_MSG_REASON_MEM)
						|| (reason ==
						AUDPP_MSG_REASON_NODECODER)) {
					prtd->dec_state =
						MSM_AUD_DECODER_STATE_FAILURE;
					wake_up(&the_locks.wait);
				}
				break;
			}
			case AUDPP_DEC_STATUS_INIT:
				pr_info("decoder status: init \n");
				alsa_adec_params(prtd);
				break;

			case AUDPP_DEC_STATUS_CFG:
				pr_info("decoder status: cfg \n");
				break;
			case AUDPP_DEC_STATUS_PLAY:
				pr_info("decoder status: play \n");
				prtd->dec_state =
					MSM_AUD_DECODER_STATE_SUCCESS;
				wake_up(&the_locks.wait);
				break;
			default:
				pr_info("unknown decoder status \n");
				break;
			}
			break;
		}
	case AUDPP_MSG_PCMDMAMISSED:
		pr_info("alsa_dsp_event: PCMDMAMISSED %d\n", msg[0]);
		prtd->eos_ack = 1;
		wake_up(&the_locks.eos_wait);
		break;
	case AUDPP_MSG_CFG_MSG:
		if (msg[0] == AUDPP_MSG_ENA_ENA) {
			pr_info("audio_dsp_event: AUDPP_MSG_ENA_ENA\n");
			prtd->out_needed = 0;
			prtd->running = 1;
			audio_dsp_out_enable(prtd, 1);
		} else if (msg[0] == AUDPP_MSG_ENA_DIS) {
			prtd->running = 0;
		} else {
			printk(KERN_ERR "alsa_dsp_event:CFG_MSG=%d\n", msg[0]);
		}
		break;
	case EVENT_MSG_ID:
		printk(KERN_INFO"alsa_dsp_event: arm9 event\n");
		break;
	default:
		printk(KERN_ERR "alsa_dsp_event: UNKNOWN (%d)\n", id);
	}
}

void alsa_audpre_dsp_event(void *data, unsigned id, size_t len,
		      void (*getevent) (void *ptr, size_t len))
{
}

void audrec_dsp_event(void *data, unsigned id, size_t len,
		      void (*getevent) (void *ptr, size_t len))
{

}

struct msm_adsp_ops aud_pre_adsp_ops = {
	.event = alsa_audpre_dsp_event,
};

struct msm_adsp_ops aud_rec_adsp_ops = {
	.event = audrec_dsp_event,
};

int alsa_adsp_configure(struct msm_audio *prtd)
{
	int ret;
	pr_debug("%s()\n", __func__);

	if (prtd->dir == SNDRV_PCM_STREAM_PLAYBACK) {
		prtd->data = prtd->substream->dma_buffer.area;
		prtd->phys = prtd->substream->dma_buffer.addr;
	}
	if (prtd->dir == SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;

	if (!prtd->data) {
		ret = -ENOMEM;
		goto err1;
	}

	if (prtd->dir == SNDRV_PCM_STREAM_PLAYBACK) {
		prtd->out_buffer_size = PLAYBACK_DMASZ;
		prtd->out_sample_rate = 44100;
		prtd->out_channel_mode = AUDPP_CMD_PCM_INTF_STEREO_V;
		prtd->out_weight = 100;

		prtd->out[0].data = prtd->data + 0;
		prtd->out[0].addr = prtd->phys + 0;
		prtd->out[0].size = BUFSZ;
		prtd->out[1].data = prtd->data + BUFSZ;
		prtd->out[1].addr = prtd->phys + BUFSZ;
		prtd->out[1].size = BUFSZ;
		ret = msm_adsp_get(prtd->module_name, &prtd->audplay,
					&alsa_adsp_ops, prtd);
		if (ret) {
			pr_err("%s: failed to get %s module\n", __func__,
				prtd->module_name);
			goto err1;
		}
	}
	if (prtd->dir == SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;

	prtd->data = NULL;
	return 0;

err1:
	return ret;
}
EXPORT_SYMBOL(alsa_adsp_configure);

int alsa_audio_configure(struct msm_audio *prtd)
{
	if (prtd->enabled)
		return 0;

	pr_debug("%s()\n", __func__);
	/* refuse to start if we're not ready with first buffer */
	if (!prtd->out[0].used)
		return -EIO;

	if (msm_adsp_enable(prtd->audplay)) {
		pr_err("audio: msm_adsp_enable(audplay) failed\n");
		return -ENODEV;
	}

	if (audpp_enable(prtd->dec_id, alsa_dsp_event, prtd)) {
		printk(KERN_ERR "audio: audpp_enable() failed\n");
		msm_adsp_put(prtd->audplay);
		return -ENODEV;
	}

	prtd->enabled = 1;
	pr_debug("%s()\n", __func__);
	return 0;
}
EXPORT_SYMBOL(alsa_audio_configure);

ssize_t alsa_send_buffer(struct msm_audio *prtd, const char __user *buf,
			  size_t count, loff_t *pos)
{
	unsigned long flag;
	const char __user *start = buf;
	struct buffer *frame;
	size_t xfer;
	int ret = 0;

	pr_debug("%s()\n", __func__);
	mutex_lock(&the_locks.write_lock);
	while (count > 0) {
		frame = prtd->out + prtd->out_head;
		ret = wait_event_interruptible(the_locks.write_wait,
					      (frame->used == 0)
					      || (prtd->stopped));
		if (ret < 0)
			break;
		if (prtd->stopped) {
			ret = -EBUSY;
			break;
		}
		xfer = count > frame->size ? frame->size : count;
		if (copy_from_user(frame->data, buf, xfer)) {
			ret = -EFAULT;
			break;
		}
		frame->used = xfer;
		prtd->out_head ^= 1;
		count -= xfer;
		buf += xfer;

		spin_lock_irqsave(&the_locks.write_dsp_lock, flag);
		frame = prtd->out + prtd->out_tail;
		if (frame->used && prtd->out_needed) {
			audio_dsp_send_buffer(prtd, prtd->out_tail,
					      frame->used);
			frame->used = BUF_INVALID_LEN;
			prtd->out_needed--;
		}
		spin_unlock_irqrestore(&the_locks.write_dsp_lock, flag);
	}
	mutex_unlock(&the_locks.write_lock);
	if (buf > start)
		return buf - start;
	return ret;
}
EXPORT_SYMBOL(alsa_send_buffer);

int alsa_audio_disable(struct msm_audio *prtd)
{
	if (prtd->enabled) {
		pr_debug("%s()\n", __func__);
		mutex_lock(&the_locks.lock);
		prtd->enabled = 0;
		audio_dsp_out_enable(prtd, 0);
		wake_up(&the_locks.write_wait);
		audpp_disable(prtd->dec_id, prtd);
		prtd->out_needed = 0;
		mutex_unlock(&the_locks.lock);
	}
	return 0;
}
EXPORT_SYMBOL(alsa_audio_disable);

int alsa_audrec_disable(struct msm_audio *prtd)
{
	return 0;
}
EXPORT_SYMBOL(alsa_audrec_disable);

int audrec_encoder_config(struct msm_audio *prtd)
{
	return 0;
}

int audio_dsp_out_enable(struct msm_audio *prtd, int yes)
{
	struct audpp_cmd_cfg_dec_type cfg_dec_cmd;

	memset(&cfg_dec_cmd, 0, sizeof(cfg_dec_cmd));

	cfg_dec_cmd.cmd_id = AUDPP_CMD_CFG_DEC_TYPE;
	if (yes)
		cfg_dec_cmd.dec_cfg = AUDPP_CMD_UPDATDE_CFG_DEC |
			AUDPP_CMD_ENA_DEC_V | AUDDEC_DEC_PCM;
	else
		cfg_dec_cmd.dec_cfg = AUDPP_CMD_UPDATDE_CFG_DEC |
				AUDPP_CMD_DIS_DEC_V;
	cfg_dec_cmd.dm_mode = 0x0;
	cfg_dec_cmd.stream_id = prtd->dec_id;
	return audpp_send_queue1(&cfg_dec_cmd, sizeof(cfg_dec_cmd));
}

static void alsa_adec_params(struct msm_audio *prtd)
{
	struct audpp_cmd_cfg_adec_params_wav cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.common.cmd_id = AUDPP_CMD_CFG_ADEC_PARAMS;
	cmd.common.length = AUDPP_CMD_CFG_ADEC_PARAMS_WAV_LEN >> 1;
	cmd.common.dec_id = prtd->dec_id;
	cmd.common.input_sampling_frequency = prtd->out_sample_rate;
	cmd.stereo_cfg = prtd->out_channel_mode;
	cmd.pcm_width = prtd->out_bits;
	cmd.sign = 0;
	audpp_send_queue2(&cmd, sizeof(cmd));
}

int alsa_buffer_read(struct msm_audio *prtd, void __user *buf,
		      size_t count, loff_t *pos)
{
	unsigned long flag;
	void *data;
	uint32_t index;
	uint32_t size;
	int ret = 0;

	mutex_lock(&the_locks.read_lock);
	while (count > 0) {
		ret = wait_event_interruptible(the_locks.read_wait,
					      (prtd->in_count > 0)
					      || prtd->stopped);
		if (ret < 0)
			break;

		if (prtd->stopped) {
			ret = -EBUSY;
			break;
		}

		index = prtd->in_tail;
		data = (uint8_t *) prtd->in[index].data;
		size = prtd->in[index].size;
		if (count >= size) {
			if (copy_to_user(buf, data, size)) {
				ret = -EFAULT;
				break;
			}
			spin_lock_irqsave(&the_locks.read_dsp_lock, flag);
			if (index != prtd->in_tail) {
				/* overrun: data is invalid, we need to retry */
				spin_unlock_irqrestore(&the_locks.read_dsp_lock,
						       flag);
				continue;
			}
			prtd->in[index].size = 0;
			prtd->in_tail = (prtd->in_tail + 1) & (FRAME_NUM - 1);
			prtd->in_count--;
			spin_unlock_irqrestore(&the_locks.read_dsp_lock, flag);
			count -= size;
			buf += size;
		} else {
			break;
		}
	}
	mutex_unlock(&the_locks.read_lock);
	return ret;
}
EXPORT_SYMBOL(alsa_buffer_read);

static int audio_dsp_send_buffer(struct msm_audio *prtd,
					unsigned idx, unsigned len)
{
	struct audplay_cmd_bitstream_data_avail cmd;

	cmd.cmd_id              = AUDPLAY_CMD_BITSTREAM_DATA_AVAIL;
	cmd.decoder_id          = prtd->dec_id;
	cmd.buf_ptr             = prtd->out[idx].addr;
	cmd.buf_size            = len/2;
	cmd.partition_number    = 0;
	return audplay_send_queue0(prtd, &cmd, sizeof(cmd));
}

int alsa_rec_dsp_enable(struct msm_audio *prtd, int enable)
{
	return 0;
}
EXPORT_SYMBOL(alsa_rec_dsp_enable);

void alsa_get_dsp_frames(struct msm_audio *prtd)
{

}
EXPORT_SYMBOL(alsa_get_dsp_frames);
