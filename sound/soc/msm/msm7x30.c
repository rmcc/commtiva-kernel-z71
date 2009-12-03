/* linux/sound/soc/msm/msm7x30.c
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

#include <mach/debug_audio_mm.h>
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
#include <sound/tlv.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <linux/msm_audio.h>

#include "msm7kv2-pcm.h"
#include <asm/mach-types.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>

static struct platform_device *msm_audio_snd_device;
struct audio_locks the_locks;
EXPORT_SYMBOL(the_locks);
struct msm_volume msm_vol_ctl;
static struct snd_kcontrol_new snd_msm_controls[];

char snddev_name[AUDIO_DEV_CTL_MAX_DEV][44];

static int msm_volume_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2; /* Volume */
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 16383;
	return 0;
}
static int msm_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	spin_lock_irq(&the_locks.mixer_lock);
	ucontrol->value.integer.value[0] = 0;
	spin_unlock_irq(&the_locks.mixer_lock);
	return 0;
}

static int msm_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	int dec_id = ucontrol->value.integer.value[0];
	int volume = ucontrol->value.integer.value[1];

	spin_lock_irq(&the_locks.mixer_lock);
	ret = audpp_set_volume_and_pan(dec_id, (unsigned) volume, 0,
			POPP);
	spin_unlock_irq(&the_locks.mixer_lock);
	if (!ret)
		msm_vol_ctl.volume = volume;

	return ret;
}

static int msm_voice_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 3; /* Device */

	/*
	 * The number of devices supported is (0 to 5)
	 */
	uinfo->value.integer.min = 1;
	uinfo->value.integer.max = msm_snddev_devcount();
	return 0;
}

static int msm_voice_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	struct msm_audio_route_config route_cfg;
	struct msm_snddev_info *dev_info;
	int set = ucontrol->value.integer.value[2];

	route_cfg.dev_id = ucontrol->value.integer.value[0];
	dev_info = audio_dev_ctrl_find_dev(route_cfg.dev_id);

	if (!(dev_info->capability & SNDDEV_CAP_RX)) {
		MM_ERR("First Dev is supposed to be RX\n");
		return -EFAULT;
	} else {
		route_cfg.stream_type = AUDIO_ROUTE_STREAM_VOICE_RX;
	}

	MM_DBG("route cfg %d %d type\n",
		route_cfg.dev_id, route_cfg.stream_type);

	if (IS_ERR(dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		rc = PTR_ERR(dev_info);
		return rc;
	}

	if (set)
		rc = msm_set_voc_route(dev_info, route_cfg.stream_type,
					route_cfg.dev_id);

	route_cfg.dev_id = ucontrol->value.integer.value[1];
	dev_info = audio_dev_ctrl_find_dev(route_cfg.dev_id);

	if (!(dev_info->capability & SNDDEV_CAP_TX)) {
		MM_ERR("Second Dev is supposed to be Tx\n");
		return -EFAULT;
	} else {
		route_cfg.stream_type = AUDIO_ROUTE_STREAM_VOICE_TX;
	}

	MM_DBG("route cfg %d %d type\n",
		route_cfg.dev_id, route_cfg.stream_type);

	if (IS_ERR(dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		rc = PTR_ERR(dev_info);
		return rc;
	}
	if (set)
		rc = msm_set_voc_route(dev_info, route_cfg.stream_type,
					route_cfg.dev_id);
	else
		mixer_post_event(AUDDEV_EVT_DEV_CHG_VOICE, route_cfg.dev_id);

	return rc;
}

static int msm_voice_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	/* TODO: query Device list */
	return 0;
}

static int msm_device_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1; /* Device */

	/*
	 * The number of devices supported is (0 to 5)
	 */
	uinfo->value.integer.min = 1;
	uinfo->value.integer.max = msm_snddev_devcount();
	return 0;
}

static int msm_device_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	int set = 0;
	struct msm_audio_route_config route_cfg;
	struct msm_snddev_info *dev_info;

	set = ucontrol->value.integer.value[0];
	route_cfg.dev_id = ucontrol->id.numid - 5;
	dev_info = audio_dev_ctrl_find_dev(route_cfg.dev_id);
	if (IS_ERR(dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		rc = PTR_ERR(dev_info);
		return rc;
	}

	if (set) {
		if (!dev_info->opened) {
			rc = dev_info->dev_ops.set_freq(dev_info,
					dev_info->sample_rate);
			if (rc < 0) {
				MM_ERR("device freq failed!\n");
				return rc;
			}
			dev_info->sample_rate = rc;
			rc = 0;
			rc = dev_info->dev_ops.open(dev_info);

			if (rc < 0) {
				MM_ERR("Enabling %s\n", dev_info->name);
				MM_ERR("device open failed!\n");
				return rc;
			}
		}
		dev_info->opened = 1;
		mixer_post_event(AUDDEV_EVT_DEV_RDY, route_cfg.dev_id);

	} else {
		if (dev_info->opened) {
			mixer_post_event(AUDDEV_EVT_REL_PENDING,
						route_cfg.dev_id);
			rc = dev_info->dev_ops.close(dev_info);
			if (rc < 0) {
				MM_ERR("Snd device failed close!\n");
				return rc;
			} else {
				MM_ERR("Disabling %s\n", dev_info->name);
				dev_info->opened = 0;
				mixer_post_event(AUDDEV_EVT_DEV_RLS,
						route_cfg.dev_id);
			}
		}
	}
	return rc;
}

static int msm_device_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	struct msm_audio_route_config route_cfg;
	struct msm_snddev_info *dev_info;

	route_cfg.dev_id = ucontrol->id.numid - 5;
	dev_info = audio_dev_ctrl_find_dev(route_cfg.dev_id);

	if (IS_ERR(dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		rc = PTR_ERR(dev_info);
		return rc;
	}

	ucontrol->value.integer.value[0] = dev_info->copp_id;
	ucontrol->value.integer.value[1] = dev_info->capability;

	return 0;
}

static int msm_route_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 3; /* Device */

	/*
	 * The number of devices supported is (0 to 5)
	 */
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = msm_snddev_devcount();
	return 0;
}

static int msm_route_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	/* TODO: query Device list */
	return 0;
}

static int msm_route_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	struct msm_audio_route_config route_cfg;
	struct msm_snddev_info *dev_info;
	int popp_id = ucontrol->value.integer.value[0];
	int set = ucontrol->value.integer.value[2];
	route_cfg.dev_id = ucontrol->value.integer.value[1];

	if (ucontrol->id.numid == 1)
		route_cfg.stream_type =	AUDIO_ROUTE_STREAM_PLAYBACK;
	else
		route_cfg.stream_type =	AUDIO_ROUTE_STREAM_REC;

	MM_DBG("route cfg %d %d type for popp %d\n",
		route_cfg.dev_id, route_cfg.stream_type, popp_id);
	dev_info = audio_dev_ctrl_find_dev(route_cfg.dev_id);

	if (IS_ERR(dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		rc = PTR_ERR(dev_info);
		return rc;
	}
	if (route_cfg.stream_type == AUDIO_ROUTE_STREAM_PLAYBACK)
		rc = msm_snddev_set_dec(popp_id, dev_info->copp_id, set);
	else
		rc = msm_snddev_set_enc(popp_id, dev_info->copp_id, set);

	if (rc < 0)
		printk(KERN_ERR "device could not be assigned!\n");

	mixer_post_event(AUDDEV_EVT_DEV_CHG_AUDIO, route_cfg.dev_id);
	return rc;
}

static struct snd_kcontrol_new snd_dev_controls[AUDIO_DEV_CTL_MAX_DEV];
#define CIMP_CONTROLS	ARRAY_SIZE(snd_msm_controls)

static int snd_dev_ctl_index(int idx)
{
	struct msm_snddev_info *dev_info;

	dev_info = audio_dev_ctrl_find_dev(idx + 0);
	if (IS_ERR(dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		return PTR_ERR(dev_info);
	}
	if (sizeof(dev_info->name) <= 44)
		sprintf(&snddev_name[idx][0] , "%s", dev_info->name);

	snd_dev_controls[idx].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	snd_dev_controls[idx].access = SNDRV_CTL_ELEM_ACCESS_READWRITE;
	snd_dev_controls[idx].name = &snddev_name[idx][0];
	snd_dev_controls[idx].index = 5 + idx;
	snd_dev_controls[idx].info = msm_device_info;
	snd_dev_controls[idx].get = msm_device_get;
	snd_dev_controls[idx].put = msm_device_put;
	snd_dev_controls[idx].private_value = 0;
	return 0;

}

#define MSM_EXT(xname, xindex, fp_info, fp_get, fp_put, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
  .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
  .name = xname, .index = xindex, \
  .info = fp_info,\
  .get = fp_get, .put = fp_put, \
  .private_value = addr, \
}

static struct snd_kcontrol_new snd_msm_controls[] = {
	MSM_EXT("Stream", 1, msm_route_info, msm_route_get, \
						 msm_route_put, 0),
	MSM_EXT("Record", 2, msm_route_info, msm_route_get, \
						 msm_route_put, 0),
	MSM_EXT("Voice", 3, msm_voice_info, msm_voice_get, \
						 msm_voice_put, 0),
	MSM_EXT("Volume", 4, msm_volume_info, msm_volume_get, \
						 msm_volume_put, 0),
};

static int msm_new_mixer(struct snd_card *card)
{
	unsigned int idx;
	int err;
	int dev_cnt;

	strcpy(card->mixername, "MSM Mixer");
	for (idx = 0; idx < ARRAY_SIZE(snd_msm_controls); idx++) {
		err = snd_ctl_add(card,	snd_ctl_new1(&snd_msm_controls[idx],
					NULL));
		if (err < 0)
			MM_ERR("ERR adding ctl\n");
	}
	dev_cnt = msm_snddev_devcount();

	for (idx = 0; idx < dev_cnt; idx++) {
		if (!snd_dev_ctl_index(idx)) {
			err = snd_ctl_add(card, snd_ctl_new1(
				&snd_dev_controls[idx], NULL));
			if (err < 0)
				MM_ERR("ERR adding ctl\n");
		} else
			return 0;
	}
	return 0;
}

static int msm_soc_dai_init(struct snd_soc_codec *codec)
{

	int ret = 0;
	ret = msm_new_mixer(codec->card);
	if (ret < 0)
		MM_ERR("msm_soc: ALSA MSM Mixer Fail\n");

	return ret;
}

static struct snd_soc_dai_link msm_dai = {
	.name = "ASOC",
	.stream_name = "ASOC",
	.codec_dai = &msm_dais[0],
	.cpu_dai = &msm_dais[1],
	.init	= msm_soc_dai_init,
};

struct snd_soc_card snd_soc_card_msm = {
	.name 		= "msm-audio",
	.dai_link	= &msm_dai,
	.num_links = 1,
	.platform = &msm_soc_platform,
};

/* msm_audio audio subsystem */
static struct snd_soc_device msm_audio_snd_devdata = {
	.card = &snd_soc_card_msm,
	.codec_dev = &soc_codec_dev_msm,
};


static int __init msm_audio_init(void)
{
	int ret;

	msm_audio_snd_device = platform_device_alloc("soc-audio", -1);
	if (!msm_audio_snd_device)
		return -ENOMEM;

	platform_set_drvdata(msm_audio_snd_device, &msm_audio_snd_devdata);
	msm_audio_snd_devdata.dev = &msm_audio_snd_device->dev;
	ret = platform_device_add(msm_audio_snd_device);
	if (ret) {
		platform_device_put(msm_audio_snd_device);
		return ret;
	}
	mutex_init(&the_locks.lock);
	mutex_init(&the_locks.write_lock);
	mutex_init(&the_locks.read_lock);
	spin_lock_init(&the_locks.read_dsp_lock);
	spin_lock_init(&the_locks.write_dsp_lock);
	spin_lock_init(&the_locks.mixer_lock);
	init_waitqueue_head(&the_locks.eos_wait);
	init_waitqueue_head(&the_locks.write_wait);
	init_waitqueue_head(&the_locks.read_wait);

	return ret;
}

static void __exit msm_audio_exit(void)
{
	platform_device_unregister(msm_audio_snd_device);
}

module_init(msm_audio_init);
module_exit(msm_audio_exit);

MODULE_DESCRIPTION("PCM module");
MODULE_LICENSE("GPL v2");
