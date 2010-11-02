/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
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
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>

#include <mach/msm_rpcrouter.h>

#include <mach/msm_iomap.h>


/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{*/
#ifdef CONFIG_FIH_FXX
#include <linux/wakelock.h>
#include <linux/completion.h>
#endif
/* } FIH, KennyChu, 2009/07/23 */

/* FIH, KennyChu, 2010/04/30, change vibrate level in FA3 model {*/
#include <mach/msm_smd.h>
/* } FIH, KennyChu, 2010/04/30*/

#define PM_LIBPROG      0x30000061

/* FIH, KennyChu, 2009/06/04, add vibrator module {*/
#ifdef CONFIG_FIH_FXX
#define PM_LIBVERS      0x10001
#else
#if (CONFIG_MSM_AMSS_VERSION == 6220) || (CONFIG_MSM_AMSS_VERSION == 6225)
#define PM_LIBVERS      0xfb837d0b
#else
#define PM_LIBVERS      0x10001
#endif
#endif
/* } FIH, KennyChu, 2009/06/04*/


#define PROCEDURE_SET_VIB_ON_OFF	22
#define PMIC_VIBRATOR_LEVEL	(3000)

/* FIH, KennyChu, 2010/04/30, change vibrate level in FA3 model {*/
#define PMIC_VIBRATOR_LEVEL_FA3	(2000)
/* } FIH, KennyChu, 2010/04/30*/

static struct work_struct work_vibrator_on;
static struct work_struct work_vibrator_off;
static struct hrtimer vibe_timer;

/* FIH, KennyChu, 2010/04/30, change vibrate level in FA3 model {*/
int g_viHWID=0;
/* } FIH, KennyChu, 2010/04/30*/

/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
#ifdef CONFIG_FIH_FXX

static struct wake_lock vibrator_suspend_wake_lock;
static pid_t thread_id;
DEFINE_MUTEX(vibrator_lock);
DECLARE_COMPLETION(vibrator_comp);

#endif
/* } FIH, KennyChu, 2009/07/23 */

static void set_pmic_vibrator(int on)
{
	static struct msm_rpc_endpoint *vib_endpoint;
	struct set_vib_on_off_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	if (!vib_endpoint) {
		vib_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
		if (IS_ERR(vib_endpoint)) {
            		printk(KERN_ERR "init vib rpc failed!\n");
			vib_endpoint = 0;
			return;
		}
	}

	if (on)
    {
/* FIH, KennyChu, 2010/04/30, change vibrate level in FA3 model {*/
        if (g_viHWID >= CMCS_7627_EVB1 && g_viHWID <= CMCS_F913_MP1)
        {
            //printk(KERN_ERR "FA3\n");
		    req.data = cpu_to_be32(PMIC_VIBRATOR_LEVEL_FA3);
        }
        else
        {
            //printk(KERN_ERR "F0X\n");
		    req.data = cpu_to_be32(PMIC_VIBRATOR_LEVEL);
        }
/* } FIH, KennyChu, 2010/04/30*/
    }
	else
		req.data = cpu_to_be32(0);

    msm_rpc_call(vib_endpoint, PROCEDURE_SET_VIB_ON_OFF, &req,
		sizeof(req), 5 * HZ);
}

static void pmic_vibrator_on(struct work_struct *work)
{
	set_pmic_vibrator(1);
}

static void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(0);
}

/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
#ifdef CONFIG_FIH_FXX

static int pmic_vibrator_off_thread(void *arg)
{
    //printk(KERN_INFO "%s: vib_off_thread running\n", __func__);

    daemonize("vib_off_thread");
    //allow_signal(SIGKILL);

    while (1) {        
        wait_for_completion(&vibrator_comp);
        //printk(KERN_INFO "%s: Got complete signal\n", __func__);

        wake_lock(&vibrator_suspend_wake_lock);

        mutex_lock(&vibrator_lock);
        set_pmic_vibrator(0);
        mutex_unlock(&vibrator_lock);

        wake_unlock(&vibrator_suspend_wake_lock);

    }
	
    return 0;
}

#endif
/* } FIH, KennyChu, 2009/07/23 */

/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
#ifndef CONFIG_FIH_FXX
static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_on);
}

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_off);
}
#endif
/* } FIH, KennyChu, 2009/07/23 */

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
#ifdef CONFIG_FIH_FXX

    wake_lock(&vibrator_suspend_wake_lock);

    hrtimer_cancel(&vibe_timer);
    //printk(KERN_INFO "%s: TIMER canceled\n", __func__);

    //printk(KERN_INFO "%s: Vibration period %d ms\n", __func__, value);
    value = (value > 15000 ? 15000 : value);
    //value = (value > 10 ? (value - 10) : value);

    if (value != 0) {
        //printk(KERN_INFO "%s: execute pmic_vibrator_on\n", __func__);

        // force to vibrate at least 20 ms
        if (0 < value && 20 > value)
        {
            value = 20;
        }

        mutex_lock(&vibrator_lock);
 
        set_pmic_vibrator(1);

        hrtimer_start(&vibe_timer,
                      ktime_set(value / 1000, (value % 1000) * 1000000),
                      HRTIMER_MODE_REL);
        mutex_unlock(&vibrator_lock);

        if (hrtimer_active(&vibe_timer))
        {
            //printk(KERN_INFO "%s: TIMER running\n", __func__);
        }
  
    }
    else
    {
        //printk(KERN_INFO "%s: stop vibrator directly\n", __func__);
        mutex_lock(&vibrator_lock);
        set_pmic_vibrator(0);
        mutex_unlock(&vibrator_lock);

        wake_unlock(&vibrator_suspend_wake_lock);
        
    }

#else
    hrtimer_cancel(&vibe_timer);
	if (value == 0)
		timed_vibrator_off(dev);
	else {
		value = (value > 15000 ? 15000 : value);

		timed_vibrator_on(dev);

		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
#endif
/* } FIH, KennyChu, 2009/07/023 */
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
#ifdef CONFIG_FIH_FXX

	wake_unlock(&vibrator_suspend_wake_lock);
	complete(&vibrator_comp);

#endif
/* } FIH, KennyChu, 2009/07/23 */

	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

void __init msm_init_pmic_vibrator(void)
{

/* FIH, KennyChu, 2010/04/30, change vibrate level in FA3 model {*/
    g_viHWID = FIH_READ_HWID_FROM_SMEM();
    //printk(KERN_ERR "*****g_viHWID = %d\n", g_viHWID);
/* } FIH, KennyChu, 2010/04/30*/

	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&pmic_vibrator);
/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
#ifdef CONFIG_FIH_FXX

	wake_lock_init(&vibrator_suspend_wake_lock, WAKE_LOCK_SUSPEND, "vibrator_suspend_work");
	thread_id = kernel_thread(pmic_vibrator_off_thread, NULL, CLONE_FS | CLONE_FILES);

#endif
/* } FIH, KennyChu, 2009/07/23 */
}

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

