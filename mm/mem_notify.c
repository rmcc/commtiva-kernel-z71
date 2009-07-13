/*
 * Notify applications of memory pressure via /dev/mem_notify
 *
 * Copyright (C) 2008 Marcelo Tosatti <marcelo@kvack.org>,
 *                    KOSAKI Motohiro <kosaki.motohiro@jp.fujitsu.com>
 *
 * Released under the GPL, see the file COPYING for details.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/vmstat.h>
#include <linux/percpu.h>
#include <linux/timer.h>
#include <linux/mem_notify.h>

#include <asm/atomic.h>

#define MAX_PROC_WAKEUP_GUARD  (10*HZ)
#define MAX_WAKEUP_TASKS (100)

struct mem_notify_file_info {
	unsigned long last_proc_notify;
};

static DECLARE_WAIT_QUEUE_HEAD(mem_wait);
static atomic_long_t nr_under_memory_pressure_zones = ATOMIC_LONG_INIT(0);
static atomic_t nr_watcher_task = ATOMIC_INIT(0);

atomic_long_t last_mem_notify = ATOMIC_LONG_INIT(INITIAL_JIFFIES);

void __memory_pressure_notify(struct zone *zone, int pressure)
{
	int nr_wakeup;
	unsigned long flags;

	spin_lock_irqsave(&mem_wait.lock, flags);

	if (pressure != zone->mem_notify_status) {
		long val = pressure ? 1 : -1;
		atomic_long_add(val, &nr_under_memory_pressure_zones);
		zone->mem_notify_status = pressure;
	}

	if (pressure) {
		int nr_watcher = atomic_read(&nr_watcher_task);

		atomic_long_set(&last_mem_notify, jiffies);
		if (!nr_watcher)
			goto out;

		nr_wakeup = (nr_watcher >> 4) + 1;
		if (unlikely(nr_wakeup > MAX_WAKEUP_TASKS))
			nr_wakeup = MAX_WAKEUP_TASKS;

		wake_up_locked_nr(&mem_wait, nr_wakeup);
	}
out:
	spin_unlock_irqrestore(&mem_wait.lock, flags);
}

static int mem_notify_open(struct inode *inode, struct file *file)
{
	struct mem_notify_file_info *info;
	int    err = 0;

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto out;
	}

	info->last_proc_notify = INITIAL_JIFFIES;
	file->private_data = info;
	atomic_inc(&nr_watcher_task);
out:
	return err;
}

static int mem_notify_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	atomic_dec(&nr_watcher_task);
	return 0;
}

static unsigned int mem_notify_poll(struct file *file, poll_table *wait)
{
	struct mem_notify_file_info *info = file->private_data;
	unsigned long now = jiffies;
	unsigned long timeout;
	unsigned int retval = 0;
	unsigned long guard_time;

	poll_wait_exclusive(file, &mem_wait, wait);

	guard_time = min_t(unsigned long,
			   MEM_NOTIFY_FREQ * atomic_read(&nr_watcher_task),
			   MAX_PROC_WAKEUP_GUARD);
	timeout = info->last_proc_notify + guard_time;
	if (time_before(now, timeout))
		goto out;

	if (atomic_long_read(&nr_under_memory_pressure_zones) != 0) {
		info->last_proc_notify = now;
		retval = POLLIN;
	}

out:
	return retval;
}

const struct file_operations mem_notify_fops = {
	.open = mem_notify_open,
	.release = mem_notify_release,
	.poll = mem_notify_poll,
};
EXPORT_SYMBOL(mem_notify_fops);
