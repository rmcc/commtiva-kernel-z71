/* arch/arm/mach-msm/rpc_server_time_remote.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Author: Iliyan Malchev <ibm@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <mach/msm_rpcrouter.h>
#include "rpc_server_time_remote.h"

/* time_remote_mtoa server definitions. */

#define TIME_REMOTE_MTOA_PROG 0x3000005d
#if CONFIG_MSM_AMSS_VERSION==6210
#define TIME_REMOTE_MTOA_VERS 0
#elif (CONFIG_MSM_AMSS_VERSION==6220) || (CONFIG_MSM_AMSS_VERSION==6225)
#define TIME_REMOTE_MTOA_VERS 0x9202a8e4
#else
#error "Unknown AMSS version"
#endif
#define TIME_REMOTE_MTOA_VERS_COMP 0x00010001
#define RPC_TIME_REMOTE_MTOA_NULL   0
#define RPC_TIME_TOD_SET_APPS_BASES 2

#define RTC_REMOTE_ATOM_PROG		0x30000048
#define RTC_REMOTE_ATOM_VERS		0x00040001
#define RTC_REQUEST_CB_PROC		0x17
#define RTC_CLIENT_INIT_PROC		0x12
#define RTC_EVENT_CB_PROC		0x1
#define RTC_CB_ID			0x1

/* Client request errors */
enum rtc_rpc_err {
	ERR_NONE,
	ERR_CLIENT_ID_PTR,		/* Invalid client ID pointer */
	ERR_CLIENT_TYPE,		/* Invalid client type */
	ERR_CLIENT_ID,			/* Invalid client ID */
	ERR_TASK_NOT_READY,		/* task is not ready for clients */
	ERR_INVALID_PROCESSOR,		/* Invalid processor id */
	ERR_UNSUPPORTED,		/* Unsupported request */
	ERR_GENERAL,			/* Any General Error */
	ERR_RPC,			/* Any ONCRPC Error */
	ERR_ALREADY_REG,		/* Client already registered */
	ERR_MAX
};

enum processor_type {
	CLIENT_PROCESSOR_NONE   = 0,
	CLIENT_PROCESSOR_MODEM,
	CLIENT_PROCESSOR_APP1,
	CLIENT_PROCESSOR_APP2,
	CLIENT_PROCESSOR_MAX
};

/* Client types */
enum client_type {
	CLIENT_TYPE_GEN1 = 0,
	CLIENT_FLOATING1,
	CLIENT_FLOATING2,
	CLIENT_TYPE_INTERNAL,
	CLIENT_TYPE_GENOFF_UPDATE,
	CLIENT_TYPE_MAX
};

/* Event types */
enum event_type {
	EVENT_TOD_CHANGE = 0,
	EVENT_GENOFF_CHANGE,
	EVENT_MAX
};

struct tod_update_info {
	uint32_t	tick;
	uint64_t	stamp;
	uint32_t	freq;
};

enum time_bases_info {
	TIME_RTC = 0,
	TIME_TOD,
	TIME_USER,
	TIME_SECURE,
	TIME_INVALID
};

struct genoff_update_info {
	enum time_bases_info time_base;
	uint64_t	offset;
};

union cb_info {
	struct tod_update_info tod_update;
	struct genoff_update_info genoff_update;
};

struct rtc_cb_recv {
	uint32_t client_cb_id;
	enum event_type event;
	uint32_t cb_info_ptr;
	union cb_info cb_info_data;
};

static struct msm_rpc_client *rpc_client;

static u8 client_id;

struct rpc_time_tod_set_apps_bases_args {
	uint32_t tick;
	uint64_t stamp;
};

static int handle_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	switch (req->procedure) {
	case RPC_TIME_REMOTE_MTOA_NULL:
		return 0;

	case RPC_TIME_TOD_SET_APPS_BASES: {
		struct rpc_time_tod_set_apps_bases_args *args;
		args = (struct rpc_time_tod_set_apps_bases_args *)(req + 1);
		args->tick = be32_to_cpu(args->tick);
		args->stamp = be64_to_cpu(args->stamp);
		printk(KERN_INFO "RPC_TIME_TOD_SET_APPS_BASES:\n"
		       "\ttick = %d\n"
		       "\tstamp = %lld\n",
		       args->tick, args->stamp);
		rtc_hctosys();
		return 0;
	}
	default:
		return -ENODEV;
	}
}


static void process_cb_request(void *buffer)
{
	struct rtc_cb_recv *rtc_cb = buffer;

	rtc_cb->client_cb_id = be32_to_cpu(rtc_cb->client_cb_id);
	rtc_cb->event = be32_to_cpu(rtc_cb->event);
	rtc_cb->cb_info_ptr = be32_to_cpu(rtc_cb->cb_info_ptr);

	if (rtc_cb->event == EVENT_TOD_CHANGE) {
		/* A TOD update has been received from the Modem */
		rtc_cb->cb_info_data.tod_update.tick =
			be32_to_cpu(rtc_cb->cb_info_data.tod_update.tick);
		rtc_cb->cb_info_data.tod_update.stamp =
			be64_to_cpu(rtc_cb->cb_info_data.tod_update.stamp);
		rtc_cb->cb_info_data.tod_update.freq =
			be32_to_cpu(rtc_cb->cb_info_data.tod_update.freq);
		printk(KERN_INFO "RPC CALL -- TOD TIME UPDATE: ttick = %d\n"
			"stamp=%lld, freq = %d \n",
			rtc_cb->cb_info_data.tod_update.tick,
			rtc_cb->cb_info_data.tod_update.stamp,
			rtc_cb->cb_info_data.tod_update.freq);
		/* Do an update of xtime */
		rtc_hctosys();
	} else
		pr_err("%s: Unknown event EVENT=%x\n",
					__func__, rtc_cb->event);
}

static int rtc_cb_func(struct msm_rpc_client *client, void *buffer, int size)
{
	int rc = -1;
	struct rpc_request_hdr *recv = buffer;

	recv->xid = be32_to_cpu(recv->xid);
	recv->type = be32_to_cpu(recv->type);
	recv->rpc_vers = be32_to_cpu(recv->rpc_vers);
	recv->prog = be32_to_cpu(recv->prog);
	recv->vers = be32_to_cpu(recv->vers);
	recv->procedure = be32_to_cpu(recv->procedure);

	if (recv->procedure == RTC_EVENT_CB_PROC)
		process_cb_request((void *) (recv + 1));

	msm_rpc_start_accepted_reply(client, recv->xid,
				RPC_ACCEPTSTAT_SUCCESS);

	rc = msm_rpc_send_accepted_reply(client, 0);
	if (rc) {
		pr_err("%s: sending reply failed: %d\n", __func__, rc);
		return rc;
	}

	return 0;
}

static int rtc_rpc_proc_args(struct msm_rpc_client *client, void *buff,
							void *data)
{
	if (*(uint32_t *)data == RTC_CLIENT_INIT_PROC) {
		/* arguments passed to the client_init function */
		struct rtc_client_init_req {
			enum client_type client;
			uint32_t client_id_ptr;
			u8 client_id;
			enum processor_type processor;
		};
		struct rtc_client_init_req *req_1 = buff;

		req_1->client = cpu_to_be32(CLIENT_TYPE_INTERNAL);
		req_1->client_id_ptr = cpu_to_be32(0x1);
		req_1->client_id = (u8) cpu_to_be32(0x1);
		req_1->processor = cpu_to_be32(CLIENT_PROCESSOR_APP1);

		return sizeof(*req_1);

	} else if (*(uint32_t *)data == RTC_REQUEST_CB_PROC) {
		/* arguments passed to the request_cb function */
		struct rtc_event_req {
			u8 client_id;
			uint32_t rtc_cb_id;
		};
		struct rtc_event_req *req_2 = buff;

		req_2->client_id =  (u8) cpu_to_be32(client_id);
		req_2->rtc_cb_id = cpu_to_be32(RTC_CB_ID);

		return sizeof(*req_2);
	} else
		return 0;
}

static int rtc_rpc_proc_result(struct msm_rpc_client *client, void *buff,
							void *data)
{
	uint32_t result = -EINVAL;

	if (*(uint32_t *)data == RTC_CLIENT_INIT_PROC) {
		/* process reply received from client_init function */
		uint32_t client_id_ptr;
		result = be32_to_cpu(*(uint32_t *)buff);
		buff += sizeof(uint32_t);
		client_id_ptr = be32_to_cpu(*(uint32_t *)(buff));
		buff += sizeof(uint32_t);
		if (client_id_ptr == 1)
			client_id = (u8) be32_to_cpu(*(uint32_t *)(buff));
		else {
			pr_err("%s: Client-id not received from Modem \n",
								__func__);
			return -EINVAL;
		}
	} else if (*(uint32_t *)data == RTC_REQUEST_CB_PROC) {
		/* process reply received from request_cb function */
		result = be32_to_cpu(*(uint32_t *)buff);
	}

	if (result == ERR_NONE) {
		pr_info("%s: RPC client reply for PROC=%x success \n",
					 __func__, *(uint32_t *)data);
		return 0;
	}

	pr_err("%s: RPC client registration failed ERROR=%x \n",
						__func__, result);
	return -EINVAL;
}

static int rtc_rpc_client_init(void)
{
	int rc, proc;

	/* Create a RPC client to handle MTOA RPC call backs */
	/* Donot create a callback thread as updates are not frequent */
	rpc_client = msm_rpc_register_client("rtc", RTC_REMOTE_ATOM_PROG,
					RTC_REMOTE_ATOM_VERS, 0, rtc_cb_func);
	if (IS_ERR(rpc_client)) {
		pr_err("rpc client reg. failed - prog=%x, vers=%x \n",
				RTC_REMOTE_ATOM_PROG, RTC_REMOTE_ATOM_VERS);
		return PTR_ERR(rpc_client);
	}

	/* Register with the server with client specific info */
	proc = RTC_CLIENT_INIT_PROC;
	rc = msm_rpc_client_req(rpc_client, RTC_CLIENT_INIT_PROC,
				rtc_rpc_proc_args, &proc,
				rtc_rpc_proc_result, &proc, -1);
	if (rc) {
		pr_err("%s: rpc client registration for PROC:%x failed \n",
					__func__, RTC_CLIENT_INIT_PROC);
		return rc;
	}

	/* Register with server for the callback event */
	proc = RTC_REQUEST_CB_PROC;
	rc = msm_rpc_client_req(rpc_client, RTC_REQUEST_CB_PROC,
				rtc_rpc_proc_args, &proc,
				rtc_rpc_proc_result, &proc, -1);
	if (rc) {
		pr_err("%s: rpc client registration for PROC:%x failed \n",
					__func__, RTC_REQUEST_CB_PROC);
	}

	return rc;
}

static struct msm_rpc_server rpc_server[] = {
	{
		.prog = TIME_REMOTE_MTOA_PROG,
		.vers = TIME_REMOTE_MTOA_VERS,
		.rpc_call = handle_rpc_call,
	},
	{
		.prog = TIME_REMOTE_MTOA_PROG,
		.vers = TIME_REMOTE_MTOA_VERS_COMP,
		.rpc_call = handle_rpc_call,
	},
};

/*
 * TODO: Move the RPC client initialization and call back handling
 *	 to drivers/rtc/rtc-msm.c. This chnage will provide  -
 *	 1. RTC update handling to a single file.
 *	 2. Single RPC client to handle all messages from  modem.
 */
static int __init rpc_server_init(void)
{
	int ret;

	/* Create server for backward compatibility*/
	/* Dual server registration to support backwards compatibility vers */
	ret = msm_rpc_create_server(&rpc_server[1]);
	if (ret < 0) {
		pr_err("%s:create rpc_server - prog=%x, vers=%x failed \n",
			__func__, rpc_server[1].prog, rpc_server[1].vers);
		return ret;
	}
	ret = msm_rpc_create_server(&rpc_server[0]);
	if (ret < 0) {
		pr_err("%s: create rpc_server - prog=%x, vers=%x failed \n",
			__func__, rpc_server[0].prog, rpc_server[0].vers);
		return ret;
	}

	/* Create callback rpc client */
	ret = rtc_rpc_client_init();
	if (ret)
		pr_err("%s: Client registration failed \n", __func__);

	/*
	 * For older targets SERVER = pass, CLIENT = fail
	 * For 7x30 and ahead, SERVER and CLIENT = pass
	 */
	return 0;
}

/* Add late init call for client registration to succeed */
late_initcall(rpc_server_init);
