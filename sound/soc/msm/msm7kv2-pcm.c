/* linux/sound/soc/msm/msm7kv2-pcm.c
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
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
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <linux/android_pmem.h>

#include "msm7kv2-pcm.h"

struct snd_msm {
	struct snd_card *card;
	struct snd_pcm *pcm;
};

int copy_count;

static struct snd_pcm_hardware msm_pcm_playback_hardware = {
	.info =                 SNDRV_PCM_INFO_INTERLEAVED,
	.formats =              USE_FORMATS,
	.rates =                USE_RATE,
	.rate_min =             USE_RATE_MIN,
	.rate_max =             48000,
	.channels_min =         1,
	.channels_max =         1,
	.buffer_bytes_max =     MAX_BUFFER_PLAYBACK_SIZE,
	.period_bytes_min =     1024,
	.period_bytes_max =     1024,
	.periods_min =          2,
	.periods_max =          2,
	.fifo_size =            0,
};


/* Conventional and unconventional sample rate supported */
static unsigned int supported_sample_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(supported_sample_rates),
	.list = supported_sample_rates,
	.mask = 0,
};

static void playback_event_handler(void *data)
{
	struct msm_audio *prtd = data;
	snd_pcm_period_elapsed(prtd->substream);
}

static int msm_pcm_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msm_audio *prtd = runtime->private_data;

	pr_debug("%s()\n", __func__);
	prtd->pcm_size = snd_pcm_lib_buffer_bytes(substream);
	prtd->pcm_count = snd_pcm_lib_period_bytes(substream);
	prtd->pcm_irq_pos = 0;
	prtd->pcm_buf_pos = 0;

	/* rate and channels are sent to audio driver */
	prtd->out_sample_rate = runtime->rate;
	prtd->out_channel_mode = runtime->channels;

	return 0;
}

static int msm_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

	pr_debug("%s()\n", __func__);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t
msm_pcm_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msm_audio *prtd = runtime->private_data;

	pr_debug("%s()\n", __func__);
	if (prtd->pcm_irq_pos == prtd->pcm_size)
		prtd->pcm_irq_pos = 0;
	return bytes_to_frames(runtime, (prtd->pcm_irq_pos));
}

struct  msm_audio_event_callbacks snd_msm_audio_ops = {
	.playback = playback_event_handler,
	.capture = NULL,
};

static int msm_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msm_audio *prtd;
	int ret = 0;
	int dec_attrb, decid;

	pr_debug("%s()\n", __func__);
	prtd = kzalloc(sizeof(struct msm_audio), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		return ret;
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw = msm_pcm_playback_hardware;
		prtd->dir = SNDRV_PCM_STREAM_PLAYBACK;
		prtd->eos_ack = 0;
		dec_attrb = AUDDEC_DEC_PCM;
		dec_attrb |= MSM_AUD_MODE_TUNNEL;
		decid = audpp_adec_alloc(dec_attrb, &prtd->module_name,
				&prtd->queue_id);
		if (decid < 0) {
			pr_err("%s: No free decoder available\n", __func__);
			kfree(prtd);
			return -ENODEV;
		}
		prtd->dec_id = decid & MSM_AUD_DECODER_MASK;

	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		return -EINVAL;
	}
	prtd->substream = substream;
	ret = snd_pcm_hw_constraint_list(runtime, 0,
						SNDRV_PCM_HW_PARAM_RATE,
						&constraints_sample_rates);
	if (ret < 0)
		goto out;
	/* Ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	prtd->ops = &snd_msm_audio_ops;
	prtd->out[0].used = BUF_INVALID_LEN;
	prtd->out[1].used = 0;
	prtd->out_head = 1; /* point to second buffer on startup */
	prtd->out_tail = 0;
	runtime->private_data = prtd;

	ret = alsa_adsp_configure(prtd);
	if (ret)
		goto out;
	copy_count = 0;
	return 0;

 out:
	kfree(prtd);
	return ret;
}

static int msm_pcm_playback_copy(struct snd_pcm_substream *substream, int a,
	snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames)
{
	int ret = 0;
	int fbytes = 0;

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msm_audio *prtd = runtime->private_data;

	pr_debug("%s()\n", __func__);
	fbytes = frames_to_bytes(runtime, frames);
	ret = alsa_send_buffer(prtd, buf, fbytes, NULL);
	++copy_count;
	prtd->pcm_buf_pos += fbytes;
	if (copy_count == 1) {
		mutex_lock(&the_locks.lock);
		ret = alsa_audio_configure(prtd);
		mutex_unlock(&the_locks.lock);
	}
	return  ret;
}

static int msm_pcm_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msm_audio *prtd = runtime->private_data;

	int ret = 0;

	pr_debug("%s()\n", __func__);

	/* pcm dmamiss message is sent continously
	 * when decoder is starved so no race
	 * condition concern
	 */
	if (prtd->enabled)
		ret = wait_event_interruptible(the_locks.eos_wait,
					prtd->eos_ack);

	alsa_audio_disable(prtd);
	msm_adsp_put(prtd->audplay);
	audpp_adec_free(prtd->dec_id);
	kfree(prtd);

	return 0;
}


static int msm_pcm_copy(struct snd_pcm_substream *substream, int a,
	 snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames)
{
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = msm_pcm_playback_copy(substream, a, hwoff, buf, frames);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = -EINVAL;
	return ret;
}

static int msm_pcm_close(struct snd_pcm_substream *substream)
{
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = msm_pcm_playback_close(substream);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = -EINVAL;
	return ret;
}
static int msm_pcm_prepare(struct snd_pcm_substream *substream)
{
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = msm_pcm_playback_prepare(substream);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = -EINVAL;
	return ret;
}

static snd_pcm_uframes_t msm_pcm_pointer(struct snd_pcm_substream *substream)
{
	snd_pcm_uframes_t ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = msm_pcm_playback_pointer(substream);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = -EINVAL;
	return ret;
}

int msm_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (substream->pcm->device & 1) {
		runtime->hw.info &= ~SNDRV_PCM_INFO_INTERLEAVED;
		runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
	}
	return 0;

}

static struct snd_pcm_ops msm_pcm_ops = {
	.open           = msm_pcm_open,
	.copy		= msm_pcm_copy,
	.hw_params	= msm_pcm_hw_params,
	.close          = msm_pcm_close,
	.ioctl          = snd_pcm_lib_ioctl,
	.prepare        = msm_pcm_prepare,
	.trigger        = msm_pcm_trigger,
	.pointer        = msm_pcm_pointer,
};



static int msm_pcm_remove(struct platform_device *devptr)
{
	struct snd_soc_device *socdev = platform_get_drvdata(devptr);
	snd_soc_free_pcms(socdev);
	kfree(socdev->codec);
	platform_set_drvdata(devptr, NULL);
	return 0;
}

static int pcm_preallocate_buffer(struct snd_pcm *pcm,
	int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size;
	if (!stream)
		size = PLAYBACK_DMASZ;
	else
		size = CAPTURE_DMASZ;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->addr = pmem_kalloc(size, PMEM_MEMTYPE_EBI1 | PMEM_ALIGNMENT_4K);
	buf->area = ioremap(buf->addr, size);
	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;
	return 0;
}

static void msm_pcm_free_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		iounmap(buf->area);
		pmem_kfree(buf->addr);
		buf->area = NULL;
	}
}

static int msm_pcm_new(struct snd_card *card,
			struct snd_soc_dai *codec_dai,
			struct snd_pcm *pcm)
{
	int ret = 0;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_32BIT_MASK;

	if (codec_dai->playback.channels_min) {
		ret = pcm_preallocate_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			return ret;
	}

	return ret;
}

struct snd_soc_platform msm_soc_platform = {
	.name		= "msm-audio",
	.remove         = msm_pcm_remove,
	.pcm_ops 	= &msm_pcm_ops,
	.pcm_new	= msm_pcm_new,
	.pcm_free	= msm_pcm_free_buffers,
};
EXPORT_SYMBOL(msm_soc_platform);

static int __init msm_soc_platform_init(void)
{
	return snd_soc_register_platform(&msm_soc_platform);
}
module_init(msm_soc_platform_init);

static void __exit msm_soc_platform_exit(void)
{
	snd_soc_unregister_platform(&msm_soc_platform);
}
module_exit(msm_soc_platform_exit);

MODULE_DESCRIPTION("PCM module platform driver");
MODULE_LICENSE("GPL v2");
