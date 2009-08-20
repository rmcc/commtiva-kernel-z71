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

#define VOICE_DALRPC_DEVICEID 0x02000075
#define VOICE_DALRPC_PORT_NAME "DAL00"
#define VOICE_DALRPC_CPU 0


/* VOICE Command */
#define CMD_VOICE_INIT			0x1
#define CMD_ACQUIRE_DONE		0x2
#define CMD_RELEASE_DONE		0x3
#define CMD_DEVICE_INFO			0x4

/* VOICE Event */
#define EVENT_ACQUIRE_START		0x6
#define EVENT_RELEASE_START		0x7
#define EVENT_DEVICE_DONE		0x8

/* Voice state */
#define VOICE_ACQUIRED 1
#define VOICE_RELEASED 2

enum {
	NETWORK_CDMA = 0,
	NETWORK_GSM,
	NETWORK_WCDMA,
	NETWORK_WCDMA_WB,
};

enum {
	VOICE_DALRPC_CMD = DALDEVICE_FIRST_DEVICE_API_IDX
};

struct voice_header {
	uint32_t id;
	uint32_t data_len;
};


struct voice_init {
	struct voice_header hdr;
	void *cb_handle;
};

/* Device information payload structure */
struct voice_device {
	struct voice_header hdr;
	uint32_t rx_device;
	uint32_t tx_device;
	uint32_t rx_volume;
	uint32_t tx_volume;
};

/*Voice command structure*/
struct voice_network {
	struct voice_header hdr;
	uint32_t network_info;
};

struct voice_data {
	void *handle; /* DALRPC handle */
	void *cb_handle; /* DALRPC callback handle */
	int state; /* Call state */
	int network; /* Network information */
	int opened;
	struct mutex lock;
	wait_queue_head_t wait;
	uint32_t rx_device;
	uint32_t tx_device;
	uint32_t rx_volume;
	uint32_t tx_volume;
};

static struct voice_data voice;

static void remote_cb_function(void *context, u32 param,
				void *evt_buf, u32 len)
{
	struct voice_header *hdr;
	struct voice_data *v = context;

	hdr = (struct voice_header *)evt_buf;

	pr_info("%s() len=%d id=%d\n", __func__, len, hdr->id);

	if (len <= 0) {
		pr_err("unexpected event with length %d \n", len);
		return;
	}

	switch (hdr->id) {
	case EVENT_ACQUIRE_START:
		v->state = VOICE_ACQUIRED;
		v->network = ((struct voice_network *)evt_buf)->network_info;
		wake_up(&v->wait);
		break;
	case EVENT_DEVICE_DONE:
		break;
	case EVENT_RELEASE_START:
		v->state = VOICE_RELEASED;
		wake_up(&v->wait);
		break;
	default:
		pr_err("Undefined event %d \n", hdr->id);
	}

}

static int voice_cmd_init(struct voice_data *v)
{

	struct voice_init cmd;
	int err;

	pr_debug("%s()\n", __func__);

	cmd.hdr.id = CMD_VOICE_INIT;
	cmd.hdr.data_len = sizeof(struct voice_init) -
				sizeof(struct voice_header);
	cmd.cb_handle = v->cb_handle;

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &cmd,
			 sizeof(struct voice_init));

	if (err)
		pr_err("Voice init command failed\n");
	return err;
}

static int voice_cmd_acquire_done(struct voice_data *v)
{
	struct voice_header hdr;
	int err;

	hdr.id = CMD_ACQUIRE_DONE;
	hdr.data_len = 0;

	pr_debug("%s()\n", __func__);

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &hdr,
			 sizeof(struct voice_header));

	if (err)
		pr_err("Voice acquire done command failed\n");
	return err;
}

static int voice_cmd_release_done(struct voice_data *v)
{
	struct voice_header hdr;
	int err;

	pr_debug("%s()\n", __func__);

	hdr.id = CMD_RELEASE_DONE;
	hdr.data_len = 0;

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &hdr,
		sizeof(struct voice_header));

	if (err)
		pr_err("Voice release done command failed\n");
	return err;
}

static int voice_cmd_device_info(struct voice_data *v)
{
	struct voice_device cmd;
	int err;

	pr_debug("%s()\n", __func__);

	err = msm_get_voc_route(&v->rx_device, &v->tx_device);
	if (err)
		return err;

	cmd.hdr.id = CMD_DEVICE_INFO;
	cmd.hdr.data_len = sizeof(struct voice_device) -
			sizeof(struct voice_header);
	cmd.tx_device = v->tx_device;
	cmd.rx_device = v->rx_device;
	cmd.rx_volume = v->rx_volume;
	cmd.tx_volume = v->tx_volume;

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &cmd,
			 sizeof(struct voice_device));

	if (err)
		pr_err("Voice device command failed\n");
	return err;
}

static int voice_open(struct inode *inode, struct file *f)
{
	int rc;
	int err;

	struct voice_data *v = &voice;

	pr_debug("%s()\n", __func__);

	mutex_lock(&v->lock);
	if (v->opened) {
		mutex_unlock(&v->lock);
		return -EBUSY;
	}

	v->opened = 1;

	if (v->handle == NULL) {
		/* get device handle */
		rc = daldevice_attach(VOICE_DALRPC_DEVICEID,
				VOICE_DALRPC_PORT_NAME,
				VOICE_DALRPC_CPU,
				&v->handle);
		if (rc) {
			pr_err("Voc DALRPC call to Modem attach failed\n");
			mutex_unlock(&v->lock);
			return -ENODEV;
		}
		pr_debug("%s: v->handle =%d\n", __func__, (int) v->handle);
		/* Allocate the callback handle */
		v->cb_handle = dalrpc_alloc_cb(v->handle, remote_cb_function,
						v);
		if (v->cb_handle == NULL) {
			pr_err("Allocate Callback failure\n");
			daldevice_detach(v->handle);
			v->handle = NULL;
			mutex_unlock(&v->lock);
			return -ENODEV;
		}

		/* setup the callback */
		err = voice_cmd_init(v);
		if (err)
			return -ENODEV;
	}

	rc = wait_event_timeout(v->wait, v->state == VOICE_ACQUIRED,
				15 * HZ);
	if (rc == 0) {
		pr_err("voice_open: ARM9 did not acquired the voice call");
		mutex_unlock(&v->lock);
		return -ENODEV;
	}

	f->private_data = v;
	mutex_unlock(&v->lock);

	return 0;
}

static int voice_release(struct inode *inode, struct file *f)
{

	int rc = 0;
	struct voice_data *v = f->private_data;

	pr_debug("%s()\n", __func__);

	mutex_lock(&v->lock);

	/* Need to send command to ADSP to stop */
	rc = wait_event_timeout(v->wait, v->state == VOICE_RELEASED,
				15 * HZ);
	if (rc == 0)
		pr_err("voice_open: voice call release fail");

	v->opened = 0;

	mutex_unlock(&v->lock);

	voice_cmd_release_done(v);
	dalrpc_dealloc_cb(v->handle, v->cb_handle);
	daldevice_detach(v->handle);

	return 0;
}

static ssize_t voice_read(struct file *f, char __user *buf, size_t cnt,
			loff_t *pos)
{
	pr_debug("%s()\n", __func__);
	return -EINVAL;
}

static ssize_t voice_write(struct file *f, const char __user *buf, size_t cnt,
			loff_t *pos)
{
	pr_debug("%s()\n", __func__);
	return -EINVAL;
}

static int voice_ioctl(struct inode *ip, struct file *f,
			 unsigned int cmd, unsigned long arg)
{
	int rc;
	struct voice_data *v = f->private_data;

	pr_debug("%s()\n", __func__);

	mutex_lock(&v->lock);
	switch (cmd) {
	case AUDIO_START:
		rc = voice_cmd_device_info(v);
		if (rc)
			break;
		rc = voice_cmd_acquire_done(v);
		break;
	case AUDIO_STOP:
		v->tx_volume = 0;
		v->rx_volume = 0;
		rc = voice_cmd_device_info(v);
		break;
	case AUDIO_GET_CONFIG: {
		struct msm_audio_config config;
		if (v->network == NETWORK_WCDMA_WB)
			config.sample_rate = 16000;
		else
			config.sample_rate = 8000;
		if (copy_to_user((void *) arg, &config, sizeof(config)))
			rc = -EFAULT;
		else
			rc = 0;
		}
		break;
	case AUDIO_SET_CONFIG:
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&v->lock);
	return rc;
}

static const struct file_operations voice_fops = {
	.owner = THIS_MODULE,
	.open = voice_open,
	.release = voice_release,
	.read = voice_read,
	.write = voice_write,
	.ioctl = voice_ioctl,
};

struct miscdevice voice_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msm_voice",
	.fops = &voice_fops,
};


static int __init voice_init(void)
{
	int rc;
	pr_debug("%s\n", __func__);

	mutex_init(&voice.lock);
	init_waitqueue_head(&voice.wait);
	voice.opened = 0;
	voice.handle = NULL;
	voice.cb_handle = NULL;
	voice.state = VOICE_RELEASED;
	voice.tx_device = 0;
	voice.rx_device = 1;
	voice.tx_volume = 5;
	voice.rx_volume = 5;

	rc = misc_register(&voice_misc);

	return rc;
}

static void __exit voice_exit(void)
{
	pr_debug("%s\n", __func__);
}

module_init(voice_init);
module_exit(voice_exit);

MODULE_DESCRIPTION("MSM Voice driver");
MODULE_LICENSE("Dual BSD/GPL");
