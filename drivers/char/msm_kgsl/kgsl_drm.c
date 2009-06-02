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

/* Implements an interface between KGSL and the DRM subsystem.  For now this
 * is pretty simple, but it will take on more of the workload as time goes
 * on
 */

#include "drmP.h"
#include "drm.h"

#define DRIVER_AUTHOR           "Qualcomm"
#define DRIVER_NAME             "kgsl"
#define DRIVER_DESC             "KGSL DRM"
#define DRIVER_DATE             "20090313"

#define DRIVER_MAJOR            1
#define DRIVER_MINOR            0
#define DRIVER_PATCHLEVEL       0

/* TODO:
 * Add vsync wait */

static int kgsl_library_name(struct drm_device *dev, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "yamato");
}

static int kgsl_drm_load(struct drm_device *dev, unsigned long flags)
{
	return 0;
}

static int kgsl_drm_unload(struct drm_device *dev)
{
	return 0;
}

void kgsl_drm_lastclose(struct drm_device *dev)
{
}

void kgsl_drm_preclose(struct drm_device *dev, struct drm_file *file_priv)
{
}

static int kgsl_drm_suspend(struct drm_device *dev, pm_message_t state)
{
	return 0;
}

static int kgsl_drm_resume(struct drm_device *dev)
{
	return 0;
}

/* int kgsl_drm_max_ioctl = DRM_ARRAY_SIZE(kgsl_drm_ioctls) */

static struct drm_driver driver = {
	.driver_features = DRIVER_USE_PLATFORM_DEVICE,
	.load = kgsl_drm_load,
	.unload = kgsl_drm_unload,
	.lastclose = kgsl_drm_lastclose,
	.preclose = kgsl_drm_preclose,
	.suspend = kgsl_drm_suspend,
	.resume = kgsl_drm_resume,
	.reclaim_buffers = drm_core_reclaim_buffers,
	.get_map_ofs = drm_core_get_map_ofs,
	.get_reg_ofs = drm_core_get_reg_ofs,
	.dri_library_name = kgsl_library_name,
	/* .ioctls = kgsl_drm_ioctls, */

	.fops = {
		 .owner = THIS_MODULE,
		 .open = drm_open,
		 .release = drm_release,
		 .ioctl = drm_ioctl,
		 .mmap = drm_mmap,
		 .poll = drm_poll,
		 .fasync = drm_fasync,
		 },

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

int kgsl_drm_init(struct platform_device *dev)
{
	driver.num_ioctls = 0;
	driver.platform_device = dev;
	return drm_init(&driver);
}

void kgsl_drm_exit(void)
{
	drm_exit(&driver);
}
