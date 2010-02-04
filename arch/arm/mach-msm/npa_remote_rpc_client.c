/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 *
 */

/*
 * Node Power Architecture (NPA) ONCRPC protocol support.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/npa_remote.h>
#include <mach/msm_rpcrouter.h>

#define NPA_REMOTEPROG			0x300000A4
#define NPA_REMOTEVERS			0x00010001
#define NPA_REMOTE_NULL_VERS		0x00010001

#define NPA_REMOTE_NULL_PROC				0
#define NPA_REMOTE_RPC_GLUE_CODE_INFO_REMOTE_PROC	1
#define NPA_REMOTE_INIT_PROC				2
#define NPA_REMOTE_CREATE_SYNC_CLIENT_PROC		3
#define NPA_REMOTE_DESTROY_CLIENT_PROC			5
#define NPA_REMOTE_ISSUE_REQUIRED_REQUEST_PROC		6
#define NPA_REMOTE_RESOURCE_AVAILABLE_PROC		22

#define NPA_REMOTE_API_VERSIONS_PROC		0xFFFFFFFF

#define NPA_REMOTE_CALLBACK_TYPE_PROC		1

#define SEND_UINT32(buf, i, size)  do { \
	*((uint32_t *)buf) = cpu_to_be32(i); \
	size += sizeof(uint32_t); \
	buf += sizeof(uint32_t); \
} while (0);

#define SEND_INT32(buf, i, size)  do { \
	*((int32_t *)buf) = cpu_to_be32(i); \
	size += sizeof(int32_t); \
	buf += sizeof(int32_t); \
} while (0);

#define SEND_CALLBACK(buf, cb, size)  do { \
	uint32_t cb_id = msm_rpc_add_cb_func(client, (void *)cb); \
	if (cb_id < 0 && (cb_id != MSM_RPC_CLIENT_NULL_CB_ID)) \
		return cb_id; \
	*((uint32_t *)buf) = cpu_to_be32((uint32_t)cb_id);\
	size += sizeof(uint32_t); \
	buf += sizeof(uint32_t);\
} while (0);

#define SEND_STRING(buf, str, size) do { \
	uint32_t len = 0; \
	if (str) { \
		len = strlen((const char *)str) + 1; \
		SEND_UINT32(buf, len, size); \
		memcpy(buf, str, len); \
		size += len; \
		buf += len; \
		if (len & 0x3) { \
			memset(buf, 0, 4 - (len & 0x3)); \
			buf += 4 - (len & 0x3); \
			size += 4 - (len & 0x3); \
		} \
	} else { \
		SEND_UINT32(buf, len, size); \
	} \
} while (0);

#define RECV_UINT32(buf, r) do { \
	r = be32_to_cpu(*(uint32_t *)buf); \
	buf += sizeof(uint32_t); \
} while (0);

#define RECV_INT32(buf, r) do { \
	r = be32_to_cpu(*(int32_t *)buf); \
	buf += sizeof(int32_t); \
} while (0);

struct npa_remote_cb_data {
	uint32_t cb_id;
	uint32_t context;
	uint32_t type;
	int32_t *buffer;
	uint32_t size;
};

struct npa_remote_init_arg {
	uint32_t major;
	uint32_t minor;
	uint32_t build;
	void *callback;
	uint32_t context;
};

struct npa_remote_init_ret {
	int32_t result;
};

struct npa_remote_resource_available_arg {
	const char *resource_name;
	void *callback;
	uint32_t context;
};

struct npa_remote_resource_available_ret {
	int32_t result;
};

struct npa_remote_create_sync_client_arg {
	const char *resource_name;
	const char *client_name;
	uint32_t client_type;
	uint32_t handle_is_valid;
};

struct npa_remote_create_sync_client_ret {
	int32_t result;
	uint32_t handle_is_valid;
	uint32_t handle;
};

struct npa_remote_destroy_client_arg {
	uint32_t handle;
};

struct npa_remote_destroy_client_ret {
	int32_t result;
};

struct npa_remote_issue_required_request_arg {
	uint32_t handle;
	uint32_t state;
	uint32_t new_state_valid;
};

struct npa_remote_issue_required_request_ret {
	int32_t result;
	uint32_t new_state_is_valid;
	uint32_t new_state;
};

static struct msm_rpc_client *npa_rpc_client;/* TODO: Use a pool or array? */
DECLARE_COMPLETION(npa_rpc_init_complete);

static int npa_remote_cb(struct msm_rpc_client *client,
		void *buffer, int in_size)
{
	int err = 0;
	int array_length = 0;
	void *buf;
	unsigned int status = RPC_ACCEPTSTAT_SYSTEM_ERR;
	struct rpc_request_hdr *req = (struct rpc_request_hdr *)buffer;
	struct npa_remote_cb_data arg;
	npa_remote_callback cb_fn = NULL;

	buf = (void *)(req + 1);

	RECV_UINT32(buf, arg.cb_id);
	RECV_UINT32(buf, arg.context);
	RECV_UINT32(buf, arg.type);
	RECV_UINT32(buf, array_length);
	if (array_length > 0) {
		int i;
		arg.buffer = kzalloc(array_length * sizeof(int32_t),
				GFP_KERNEL);
		for (i = 0; i < array_length; i++)
			RECV_INT32(buf, arg.buffer[i]);
	}
	RECV_UINT32(buf, arg.size);

	cb_fn = (npa_remote_callback) msm_rpc_get_cb_func(client, arg.cb_id);
	if (cb_fn) {
		cb_fn((void *)arg.context, arg.type, arg.buffer, arg.size);
		status = RPC_ACCEPTSTAT_SUCCESS;
	}

	msm_rpc_start_accepted_reply(client, be32_to_cpu(req->xid), status);
	err = msm_rpc_send_accepted_reply(client, 0);
	if (err) {
		pr_err("NPA Remote callback %s: send accepted reply failed: "
				"%d\n", __func__, err);
		BUG();
	}

	kfree(arg.buffer);

	return 0;
}

static int npa_remote_cb_fn(struct msm_rpc_client *client,
		void *buffer, int in_size)
{
	int ret = 0;
	struct rpc_request_hdr *req = (struct rpc_request_hdr *)buffer;

	switch (be32_to_cpu(req->procedure)) {
	case NPA_REMOTE_CALLBACK_TYPE_PROC:
		ret = npa_remote_cb(client, buffer, in_size);
		break;
	default:
		break;
	}

	return ret;
}

int npa_remote_null(void)
{
	wait_for_completion(&npa_rpc_init_complete);
	return msm_rpc_client_req(npa_rpc_client, NPA_REMOTE_NULL_PROC,
			NULL, NULL, NULL, NULL, -1);
}

static int npa_remote_init_arg_fn(struct msm_rpc_client *client,
		void *buf, void *data)
{
	int size = 0;
	struct npa_remote_init_arg *arg = (struct npa_remote_init_arg *)data;

	SEND_UINT32(buf, arg->major, size);
	SEND_UINT32(buf, arg->minor, size);
	SEND_UINT32(buf, arg->build, size);
	SEND_CALLBACK(buf, arg->callback, size);
	SEND_UINT32(buf, arg->context, size);

	return size;
}

static int npa_remote_init_ret_fn(struct msm_rpc_client *client,
		void *buf, void *data)
{
	struct npa_remote_init_ret *ret = (struct npa_remote_init_ret *)data;

	RECV_INT32(buf, ret->result);

	return 0;
}

int npa_remote_init(unsigned int major, unsigned int minor, unsigned int build,
		npa_remote_callback callback, void *context)
{
	int err = 0;
	struct npa_remote_init_arg arg;
	struct npa_remote_init_ret ret;

	wait_for_completion(&npa_rpc_init_complete);

	arg.major = major;
	arg.minor = minor;
	arg.build = build;
	arg.callback = (void *)callback;
	arg.context = (uint32_t)context;

	err = msm_rpc_client_req(npa_rpc_client,
			NPA_REMOTE_INIT_PROC,
			npa_remote_init_arg_fn, &arg,
			npa_remote_init_ret_fn, &ret, -1);
	if (err) {
		pr_err("NPA Remote func %s returned error %d\n", __func__, err);
		BUG();
	}

	return ret.result;
}
EXPORT_SYMBOL(npa_remote_init);

static int npa_remote_resource_available_arg_fn(struct msm_rpc_client *client,
		void *buf, void *data)
{
	int size = 0;
	struct npa_remote_resource_available_arg *arg =
		(struct npa_remote_resource_available_arg *)data;

	SEND_STRING(buf, arg->resource_name, size);
	SEND_CALLBACK(buf, arg->callback, size);
	SEND_UINT32(buf, arg->context, size);

	return size;
}

static int npa_remote_resource_available_ret_fn(struct msm_rpc_client *client,
		void *buf, void *data)
{
	struct npa_remote_resource_available_ret *ret =
		(struct npa_remote_resource_available_ret *)data;

	RECV_UINT32(ret, ret->result);

	return 0;
}

int npa_remote_resource_available(const char *resource_name,
		npa_remote_callback callback, void *context)
{
	int err = 0;
	struct npa_remote_resource_available_arg arg;
	struct npa_remote_resource_available_ret ret;

	arg.resource_name = resource_name;
	arg.callback = (void *)callback;
	arg.context = (uint32_t)context;

	err = msm_rpc_client_req(npa_rpc_client,
			NPA_REMOTE_RESOURCE_AVAILABLE_PROC,
			npa_remote_resource_available_arg_fn, &arg,
			npa_remote_resource_available_ret_fn, &ret, -1);
	if (err) {
		pr_err("NPA Remote func %s returned error %d\n", __func__, err);
		BUG();
	}

	return ret.result;
}
EXPORT_SYMBOL(npa_remote_resource_available);

static int npa_remote_create_sync_client_arg_fn(struct msm_rpc_client *client,
		void *buf, void *data)
{
	int size = 0;
	struct npa_remote_create_sync_client_arg *arg =
		(struct npa_remote_create_sync_client_arg *)data;

	SEND_STRING(buf, arg->resource_name, size);
	SEND_STRING(buf, arg->client_name, size);
	SEND_UINT32(buf, arg->client_type, size);
	SEND_UINT32(buf, arg->handle_is_valid, size);

	return size;
}

static int npa_remote_create_sync_client_ret_fn(struct msm_rpc_client *client,
		void *buf, void *data)
{
	struct npa_remote_create_sync_client_ret *ret =
		(struct npa_remote_create_sync_client_ret *)data;

	RECV_INT32(buf, ret->result);
	RECV_UINT32(buf, ret->handle_is_valid);
	if (ret->handle_is_valid)
		RECV_UINT32(buf, ret->handle);

	return 0;
}

int npa_remote_create_sync_client(const char *resource_name,
		const char *client_name,
		enum npa_remote_client_type client_type,
		void **handle)
{
	int err = 0;
	struct npa_remote_create_sync_client_arg arg;
	struct npa_remote_create_sync_client_ret ret;

	arg.resource_name = resource_name;
	arg.client_name = client_name;
	arg.client_type = client_type;
	arg.handle_is_valid = handle != NULL;

	err = msm_rpc_client_req(npa_rpc_client,
			NPA_REMOTE_CREATE_SYNC_CLIENT_PROC,
			npa_remote_create_sync_client_arg_fn, &arg,
			npa_remote_create_sync_client_ret_fn, &ret, -1);
	if (err) {
		pr_err("NPA Remote func %s returned error %d\n", __func__, err);
		BUG();
	}

	if (ret.handle_is_valid)
		*handle = (void *)ret.handle;

	return ret.result;
}
EXPORT_SYMBOL(npa_remote_create_sync_client);

static int npa_remote_destroy_client_arg_fn(struct msm_rpc_client *client,
		void *buf, void *data)
{
	int size = 0;
	struct npa_remote_destroy_client_arg *arg =
		(struct npa_remote_destroy_client_arg *)data;

	SEND_UINT32(buf, arg->handle, size);

	return size;
}

static int npa_remote_destroy_client_ret_fn(struct msm_rpc_client *client,
		void *buf, void *data)
{
	struct npa_remote_destroy_client_ret *ret =
		(struct npa_remote_destroy_client_ret *)data;

	RECV_INT32(buf, ret->result);

	return 0;
}

int npa_remote_destroy_client(void *handle)
{
	int err = 0;
	struct npa_remote_destroy_client_arg arg;
	struct npa_remote_destroy_client_ret ret;

	arg.handle = (uint32_t)handle;

	err = msm_rpc_client_req(npa_rpc_client,
			NPA_REMOTE_DESTROY_CLIENT_PROC,
			npa_remote_destroy_client_arg_fn, &arg,
			npa_remote_destroy_client_ret_fn, &ret, -1);
	if (err) {
		pr_err("NPA Remote func %s returned error %d\n", __func__, err);
		BUG();
	}

	return ret.result;
}
EXPORT_SYMBOL(npa_remote_destroy_client);

static int npa_remote_issue_required_request_arg_fn(
		struct msm_rpc_client *client,
		void *buf, void *data)
{
	int size = 0;
	struct npa_remote_issue_required_request_arg *arg =
		(struct npa_remote_issue_required_request_arg *)data;

	SEND_UINT32(buf, arg->handle, size);
	SEND_UINT32(buf, arg->state, size);
	SEND_UINT32(buf, arg->new_state_valid, size);

	return size;
}

static int npa_remote_issue_required_request_ret_fn(
		struct msm_rpc_client *client,
		void *buf, void *data)
{
	struct npa_remote_issue_required_request_ret *ret =
		(struct npa_remote_issue_required_request_ret *)data;

	RECV_INT32(buf, ret->result);
	RECV_UINT32(buf, ret->new_state_is_valid);
	if (ret->new_state_is_valid)
		RECV_UINT32(buf, ret->new_state);

	return 0;
}

int npa_remote_issue_required_request(void *handle, unsigned int state,
		unsigned int *new_state)
{
	int err = 0;
	struct npa_remote_issue_required_request_arg arg;
	struct npa_remote_issue_required_request_ret ret;

	arg.handle = (uint32_t)handle;
	arg.state = state;
	arg.new_state_valid = new_state != NULL;

	err = msm_rpc_client_req(npa_rpc_client,
			NPA_REMOTE_ISSUE_REQUIRED_REQUEST_PROC,
			npa_remote_issue_required_request_arg_fn, &arg,
			npa_remote_issue_required_request_ret_fn, &ret, -1);
	if (err) {
		pr_err("NPA Remote func %s returned error %d\n", __func__, err);
		BUG();
	}

	if (ret.new_state_is_valid)
		*new_state = ret.new_state;

	return ret.result;
}
EXPORT_SYMBOL(npa_remote_issue_required_request);

/* Registration with the platform driver for notification on the availability
 * of the NPA remote server
 */

static int __devinit npa_rpc_init_probe(struct platform_device *pdev)
{
	npa_rpc_client = msm_rpc_register_client(
					"npa-remote-client",
					NPA_REMOTEPROG,
					NPA_REMOTEVERS,
					1, npa_remote_cb_fn);

	if (IS_ERR(npa_rpc_client)) {
		pr_err("NPA REMOTE: RPC client creation failed\n");
		return -ENODEV;
	}
	complete(&npa_rpc_init_complete);

	return 0;
}

static char npa_rpc_driver_name[] = "rs00000000:00000000";

static struct platform_driver npa_rpc_init_driver = {
	.probe = npa_rpc_init_probe,
	.driver = {
		.owner = THIS_MODULE,
	},
};

static int __init npa_rpc_init(void)
{
	int err = 0;

	snprintf(npa_rpc_driver_name, sizeof(npa_rpc_driver_name),
		"rs%08x:%08x", NPA_REMOTEPROG,
		NPA_REMOTEVERS & RPC_VERSION_MAJOR_MASK);
	npa_rpc_init_driver.driver.name = npa_rpc_driver_name;

	err = platform_driver_register(&npa_rpc_init_driver);

	return err;
}
arch_initcall(npa_rpc_init);
