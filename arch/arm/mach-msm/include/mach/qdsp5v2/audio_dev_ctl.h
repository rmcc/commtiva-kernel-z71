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
#ifndef __MACH_QDSP5_V2_SNDDEV_H
#define __MACH_QDSP5_V2_SNDDEV_H
#include <mach/qdsp5v2/audio_def.h>

struct msm_snddev_info {
	const char *name;
	u32 capability;
	u32 copp_id;
	u32 acdb_id;
	struct msm_snddev_ops {
		int (*open)(struct msm_snddev_info *);
		int (*close)(struct msm_snddev_info *);
	  int (*set_freq)(struct msm_snddev_info *, u32);
	} dev_ops;
	u8 opened;
	void *private_data;
	bool state;
	u32 sample_rate;
};

void msm_snddev_register(struct msm_snddev_info *);
void msm_snddev_unregister(struct msm_snddev_info *);
int msm_snddev_devcount(void);
int msm_snddev_query(int dev_id);
unsigned short msm_snddev_route_dec(int popp_id);
unsigned short msm_snddev_route_enc(int enc_id);
int msm_snddev_set_dec(int popp_id, int copp_id, int set);
int msm_snddev_set_enc(int popp_id, int copp_id, int set);
int msm_snddev_is_set(int popp_id, int copp_id);
int msm_get_voc_route(u32 *rx_id, u32 *tx_id);
int msm_set_voc_route(struct msm_snddev_info *dev_info, int stream_type);
struct msm_snddev_info *audio_dev_ctrl_find_dev(u32 dev_id);
void msm_release_voc_thread(void);
#endif
