/*
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * SMD RPCROUTER CLIENTS module.
 *
 */

#include <linux/kernel.h>
#include <linux/kthread.h>

#include <mach/msm_rpcrouter.h>
#include "smd_rpcrouter.h"

struct msm_rpc_client_cb_item {
	struct list_head list;

	void *buf;
	int size;
};

static int rpc_clients_cb_thread(void *data)
{
	struct msm_rpc_client_cb_item *cb_item;
	struct msm_rpc_client *client;
	struct rpc_request_hdr *req;

	client = data;
	for (;;) {
		wait_event(client->cb_wait, client->cb_avail);
		if (client->exit_flag)
			break;

		client->cb_avail = 0;
		mutex_lock(&client->cb_item_list_lock);
		while (!list_empty(&client->cb_item_list)) {
			cb_item = list_first_entry(
				&client->cb_item_list,
				struct msm_rpc_client_cb_item,
				list);
			list_del(&cb_item->list);
			mutex_unlock(&client->cb_item_list_lock);
			req = (struct rpc_request_hdr *)cb_item->buf;

			if (be32_to_cpu(req->type) != 0)
				goto bad_rpc;
			if (be32_to_cpu(req->rpc_vers) != 2)
				goto bad_rpc;
			if (be32_to_cpu(req->prog) !=
			    (client->prog | 0x01000000))
				goto bad_rpc;

			client->cb_func(client,
					cb_item->buf, cb_item->size);
 bad_rpc:
			kfree(cb_item->buf);
			kfree(cb_item);
			mutex_lock(&client->cb_item_list_lock);
		}
		mutex_unlock(&client->cb_item_list_lock);
	}
	complete_and_exit(&client->cb_complete, 0);
}

static int rpc_clients_thread(void *data)
{
	void *buffer;
	uint32_t type;
	struct msm_rpc_client *client;
	int rc = 0;
	struct msm_rpc_client_cb_item *cb_item;

	client = data;
	for (;;) {
		rc = msm_rpc_read(client->ept, &buffer, -1, HZ);
		if (client->exit_flag)
			break;
		if (rc < ((int)(sizeof(uint32_t) * 2)))
			continue;

		type = be32_to_cpu(*((uint32_t *)buffer + 1));
		if (type == 1) {
			client->buf = buffer;
			client->read_avail = 1;
			wake_up(&client->reply_wait);
		} else if (type == 0) {
			cb_item = kmalloc(sizeof(*cb_item), GFP_KERNEL);
			if (!cb_item) {
				pr_err("%s: no memory for cb item\n",
				       __func__);
				continue;
			}

			if (client->cb_thread == NULL) {
				client->cb_func(client, buffer, rc);
				kfree(buffer);
			} else {
				INIT_LIST_HEAD(&cb_item->list);
				cb_item->buf = buffer;
				cb_item->size = rc;
				mutex_lock(&client->cb_item_list_lock);
				list_add_tail(&cb_item->list,
					      &client->cb_item_list);
				mutex_unlock(&client->cb_item_list_lock);
				client->cb_avail = 1;
				wake_up(&client->cb_wait);
			}
		}
	}
	complete_and_exit(&client->complete, 0);
}

static struct msm_rpc_client *msm_rpc_create_client(void)
{
	struct msm_rpc_client *client;

	client = kmalloc(sizeof(struct msm_rpc_client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	init_waitqueue_head(&client->reply_wait);
	mutex_init(&client->req_lock);
	client->buf = NULL;
	client->read_avail = 0;
	client->cb_buf = NULL;
	client->cb_size = 0;
	client->exit_flag = 0;
	init_completion(&client->complete);
	init_completion(&client->cb_complete);
	INIT_LIST_HEAD(&client->cb_item_list);
	mutex_init(&client->cb_item_list_lock);
	client->cb_avail = 0;
	init_waitqueue_head(&client->cb_wait);

	return client;
}

/*
 * Interface to be used to register the client.
 *
 * name: string representing the client
 *
 * prog: program number of the client
 *
 * ver: version number of the client
 *
 * create_cb_thread: if set calls the callback function from a seprate thread
 *                   which helps the client requests to be processed without
 *                   getting loaded by callback handling.
 *
 * cb_func: function to be called if callback request is received.
 *          unmarshaling should be handled by the user in callback function
 *
 * Return Value:
 *        Pointer to initialized client data sturcture
 *        Or, the error code if registration fails.
 *
 */
struct msm_rpc_client *msm_rpc_register_client(
	const char *name,
	uint32_t prog, uint32_t ver,
	uint32_t create_cb_thread,
	int (*cb_func)(struct msm_rpc_client *, void *, int))
{
	struct msm_rpc_client *client;
	struct msm_rpc_endpoint *ept;
	int rc;

	client = msm_rpc_create_client();
	if (IS_ERR(client))
		return client;

	ept = msm_rpc_connect_compatible(prog, ver, MSM_RPC_UNINTERRUPTIBLE);
	if (IS_ERR(ept)) {
		kfree(client);
		return (struct msm_rpc_client *)ept;
	}

	client->prog = prog;
	client->ver = ver;
	client->ept = ept;
	client->cb_func = cb_func;

	/* start the read thread */
	client->read_thread = kthread_run(rpc_clients_thread, client,
					  "k%sclntd", name);
	if (IS_ERR(client->read_thread)) {
		rc = PTR_ERR(client->read_thread);
		msm_rpc_close(client->ept);
		kfree(client);
		return ERR_PTR(rc);
	}

	if (!create_cb_thread || (cb_func == NULL)) {
		client->cb_thread = NULL;
		return client;
	}

	/* start the callback thread */
	client->cb_thread = kthread_run(rpc_clients_cb_thread, client,
					"k%sclntcbd", name);
	if (IS_ERR(client->cb_thread)) {
		rc = PTR_ERR(client->cb_thread);
		client->exit_flag = 1;
		wait_for_completion(&client->complete);
		msm_rpc_close(client->ept);
		kfree(client);
		return ERR_PTR(rc);
	}

	return client;
}
EXPORT_SYMBOL(msm_rpc_register_client);

/*
 * Interface to be used to unregister the client
 * No client operations should be done once the unregister function
 * is called.
 *
 * client: pointer to client data structure.
 *
 * Return Value:
 *        Always returns 0 (success).
 */
int msm_rpc_unregister_client(struct msm_rpc_client *client)
{
	pr_info("%s: stopping client...\n", __func__);
	client->exit_flag = 1;
	if (client->cb_thread) {
		client->cb_avail = 1;
		wake_up(&client->cb_wait);
		wait_for_completion(&client->cb_complete);
	}

	wait_for_completion(&client->complete);

	msm_rpc_close(client->ept);
	kfree(client);
	return 0;
}
EXPORT_SYMBOL(msm_rpc_unregister_client);

/*
 * Interface to be used to send a client request.
 * If the request takes any arguments or expects any results, the user
 * should handle it in 'arg_func' and 'result_func' respectively.
 * Marshaling and Unmarshaling should be handled by the user in argument
 * and result functions.
 *
 * client: pointer to client data sturcture
 *
 * proc: procedure being requested
 *
 * arg_func: argument function pointer
 *
 * result_func: result function pointer
 *
 * data: passed as an input parameter to argument and result functions.
 *
 * Return Value:
 *        0 on success, otherwise an error code is returned.
 */
int msm_rpc_client_req(struct msm_rpc_client *client, uint32_t proc,
		       int (*arg_func)(void *buf, void *data),
		       int (*result_func)(void *buf, void *data),
		       void *data)
{
	int size = 0;
	struct rpc_reply_hdr *rpc_rsp;
	int rc = 0;

	mutex_lock(&client->req_lock);

	msm_rpc_setup_req((struct rpc_request_hdr *)client->req, client->prog,
			  client->ver, proc);
	size = sizeof(struct rpc_request_hdr);

	if (arg_func)
		size += arg_func((void *)((struct rpc_request_hdr *)
					  client->req + 1), data);

	rc = msm_rpc_write(client->ept, client->req, size);
	if (rc < 0) {
		pr_err("%s: couldn't send RPC request:%d\n", __func__, rc);
		goto release_locks;
	} else
		rc = 0;

	rc = wait_event_timeout(client->reply_wait,
				client->read_avail, msecs_to_jiffies(10000));
	if (rc == 0) {
		rc = -ETIMEDOUT;
		goto release_locks;
	} else
		rc = 0;

	client->read_avail = 0;

	rpc_rsp = (struct rpc_reply_hdr *)client->buf;
	if (be32_to_cpu(rpc_rsp->reply_stat) != RPCMSG_REPLYSTAT_ACCEPTED) {
		pr_err("%s: RPC call was denied! %d\n", __func__,
		       be32_to_cpu(rpc_rsp->reply_stat));
		rc = -EPERM;
		goto free_and_release;
	}

	if (be32_to_cpu(rpc_rsp->data.acc_hdr.accept_stat) !=
	    RPC_ACCEPTSTAT_SUCCESS) {
		pr_err("%s: RPC call was not successful (%d)\n", __func__,
		       be32_to_cpu(rpc_rsp->data.acc_hdr.accept_stat));
		rc = -EINVAL;
		goto free_and_release;
	}

	if (result_func)
		result_func((void *)(rpc_rsp + 1), data);

 free_and_release:
	kfree(client->buf);
 release_locks:
	mutex_unlock(&client->req_lock);
	return rc;
}
EXPORT_SYMBOL(msm_rpc_client_req);

/*
 * Interface to be used to send accepted reply required in callback handling.
 * Additional payload may be passed in through the 'buf' and 'size'.
 * Marshaling should be handled by user for the payload.
 *
 * client: pointer to client data structure
 *
 * xid: transaction id. Has to be same as the one in callback request.
 *
 * accept_status: acceptance status
 *
 * buf: additional payload
 *
 * size: additional payload size
 *
 * Return Value:
 *        0 on success, otherwise returns an error code.
 */
int msm_rpc_send_accepted_reply(struct msm_rpc_client *client,
				uint32_t xid, uint32_t accept_status,
				char *buf, uint32_t size)
{
	int rc = 0;
	struct rpc_reply_hdr *reply;

	reply = kmalloc(sizeof(struct rpc_reply_hdr) + size, GFP_KERNEL);
	if (!reply)
		return -ENOMEM;

	reply->xid = cpu_to_be32(xid);
	reply->type = cpu_to_be32(1); /* reply */
	reply->reply_stat = cpu_to_be32(RPCMSG_REPLYSTAT_ACCEPTED);

	reply->data.acc_hdr.accept_stat = cpu_to_be32(accept_status);
	reply->data.acc_hdr.verf_flavor = 0;
	reply->data.acc_hdr.verf_length = 0;

	memcpy((char *)(reply + 1), buf, size);
	size += sizeof(struct rpc_reply_hdr);
	rc = msm_rpc_write(client->ept, reply, size);
	if (rc > 0)
		rc = 0;

	return rc;
}
EXPORT_SYMBOL(msm_rpc_send_accepted_reply);
