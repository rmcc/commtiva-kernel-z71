/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

// +++ FIH, KarenLiao, @20090603: Control msm_mic_en_proc function.
#ifdef CONFIG_FIH_FXX

//+++ FIH, KarenLiao @2009/09/21: Enable headset mic bias by smem command for F0X.B-4104.
//#include <mach/msm_rpcrouter.h>
//#include <linux/err.h>
//--- FIH, KarenLiao @2009/09/21: Enable headset mic bias by smem command for F0X.B-4104.

#include <linux/delay.h> // FIH, KarenLiao, @20090719: [F0X.FC-111]: [Audio] Add headset debounce mechanism.

// +++ FIH, KarenLiao, @20091023: [F0X.FC-663]: Allow user to use the normal headset.
#define HOOK_SWITCH_PIN 94
extern bool EnableAbnormalHS;
// --- FIH, KarenLiao, @20091023: [F0X.FC-663]: Allow user to use the normal headset.

#endif
// +++ FIH, KarenLiao, @20090603: Control msm_mic_en_proc function.

struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
	
#ifdef CONFIG_FIH_FXX // FIH, KarenLiao, @20090719: [F0X.FC-111]: [Audio] Add headset debounce mechanism.
	bool bHeadsetInserted;
#endif	
};


// +++ FIH, KarenLiao, @20090603: Control msm_mic_en_proc function.

#ifdef CONFIG_FIH_FXX
//extern int msm_mic_en_proc(bool disable_enable); // FIH, KarenLiao, 20090521: Mic bias control based on headset state.
extern int Q7x27_kybd_hookswitch_irqsetup(bool activate_irq);

//+++ FIH, KarenLiao @2009/09/21: Enable headset mic bias by smem command for F0X.B-4104.
extern int proc_comm_enable_pm_mic(unsigned *cmd_parameter);
static int EnableMic = 0;

struct workqueue_struct *headset_switch_wq; //FIH, Karen Liao, 2010/04/29, F0XE.B-697: [Audio] The music from BT headset is not smooth when plug in wired headset.

#if 0
#define PM_MIC_EN_API_PROG              0x30000061
#define PM_MIC_EN_API_VERS              0x00010001 
//+++ FIH, KarenLiao@20090806: [Audio] 4415 audio driver porting
//#define PM_MIC_EN_CONFIG_PROC           27
#define PM_MIC_EN_CONFIG_PROC           28
//--- FIH, KarenLiao@20090806: [Audio] 4415 audio driver porting

static struct msm_rpc_endpoint *pm_mic_en_ep;

int msm_mic_en_proc(bool disable_enable)
{

    int rc = 0;
	
    struct msm_fb_pm_mic_en_req {
        struct rpc_request_hdr hdr;
        uint32_t on;
    } req;	
	
    pm_mic_en_ep = msm_rpc_connect(PM_MIC_EN_API_PROG,
                                   PM_MIC_EN_API_VERS, 0);

    if (IS_ERR(pm_mic_en_ep)) {
        printk(KERN_ERR "%s: msm_rpc_connect failed! rc = %ld\n",
               __func__, PTR_ERR(pm_mic_en_ep));
        return -EINVAL;
    }

    req.on = cpu_to_be32((uint32_t)disable_enable);
    rc = msm_rpc_call(pm_mic_en_ep, PM_MIC_EN_CONFIG_PROC,
		&req, sizeof(req), 5 * HZ);

    if (rc)
        printk(KERN_ERR "%s: msm_rpc_call failed! rc = %d\n", __func__, rc);
                
    msm_rpc_close(pm_mic_en_ep);
    return rc;
}
#endif
//--- FIH, KarenLiao @2009/09/21: Enable headset mic bias by smem command for F0X.B-4104.

// --- FIH, KarenLiao, @20090603: Control msm_mic_en_proc function.

// +++ FIH, KarenLiao, @20090719: [F0X.FC-111]: [Audio] Add headset debounce mechanism.
void hook_switch_enable(bool bHookSwitchEnabled, bool HeadsetState)
{
//+++ FIH, KarenLiao@20090909: FA3.B-1107: [ACC] Playing Video and plug in/unplug HS many times, DUT works abnormality.
#if 0
	if(bHookSwitchEnabled != HeadsetState)
	{
		msm_mic_en_proc(bHookSwitchEnabled);
		msleep(100); //FA3.B-64: [Call Control]Pressing hook key of the headset can not disconnected the call.
		Q7x27_kybd_hookswitch_irqsetup(bHookSwitchEnabled);
		printk(KERN_INFO "Headset: hook_switch_enabled = %d\n", bHookSwitchEnabled);
	}
#else

	if(bHookSwitchEnabled != HeadsetState)
	{
		if(bHookSwitchEnabled) {
			//+++ FIH, KarenLiao @2009/09/21: Enable headset mic bias by smem command for F0X.B-4104.
			//msm_mic_en_proc(bHookSwitchEnabled);
			EnableMic = 1;
			proc_comm_enable_pm_mic(&EnableMic);
			//--- FIH, KarenLiao @2009/09/21: Enable headset mic bias by smem command for F0X.B-4104.
			msleep(800);
			
			// +++ FIH, KarenLiao, @20091023: [F0X.FC-663]: Allow user to use the normal headset.
			EnableAbnormalHS = (gpio_get_value(HOOK_SWITCH_PIN) == 0)? true: false; 

			//printk(KERN_INFO "Headset: EnableAbnormalHS == %d\n", EnableAbnormalHS);
			//Q7x27_kybd_hookswitch_irqsetup(bHookSwitchEnabled);

			if(EnableAbnormalHS == true){
			    EnableMic = 0;
			    proc_comm_enable_pm_mic(&EnableMic);			    
			}else{
			    Q7x27_kybd_hookswitch_irqsetup(bHookSwitchEnabled);
			}  	
			// --- FIH, KarenLiao, @20091023: [F0X.FC-663]: Allow user to use the normal headset.
			
		}else {
			// +++ FIH, KarenLiao, @20091023: [F0X.FC-663]: Allow user to use the normal headset.
            if(EnableAbnormalHS == false){
                Q7x27_kybd_hookswitch_irqsetup(bHookSwitchEnabled);
    			
                //+++ FIH, KarenLiao @2009/09/21: Enable headset mic bias by smem command for F0X.B-4104.
                //msm_mic_en_proc(bHookSwitchEnabled);
                EnableMic = 0;
                proc_comm_enable_pm_mic(&EnableMic);
                //--- FIH, KarenLiao @2009/09/21: Enable headset mic bias by smem command for F0X.B-4104.            
            }
            else{ //EnableAbnormalHS == true
                EnableAbnormalHS = false;
            }
			// --- FIH, KarenLiao, @20091023: [F0X.FC-663]: Allow user to use the normal headset.
		}
		//printk(KERN_INFO "Headset: hook_switch_enabled = %d\n", bHookSwitchEnabled);
	}

#endif
//--- FIH, KarenLiao@20090909: FA3.B-1107: [ACC] Playing Video and plug in/unplug HS many times, DUT works abnormality.
	
}

#endif
// --- FIH, KarenLiao, @20090719: [F0X.FC-111]: [Audio] Add headset debounce mechanism.

static void gpio_switch_work(struct work_struct *work)
{
	int state;
	struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);

	state = gpio_get_value(data->gpio);
	
// +++ FIH, KarenLiao, @20090719: [F0X.FC-111]: [Audio] Add headset debounce mechanism.

/*
	printk(KERN_INFO "Headset: switch_set_state = %d\n", state);
	
	switch_set_state(&data->sdev, state);

#ifdef CONFIG_FIH_FXX
	msm_mic_en_proc(state == 0); // FIH, KarenLiao, 20090521: Mic bias control based on headset state.	
	Q7x27_kybd_hookswitch_irqsetup(state == 0); // FIH, KarenLiao, @20090603: Set up hoow switch irq according to headset state.
#endif
*/	

#ifdef CONFIG_FIH_FXX

	disable_irq(data->irq); //FA3.B-64: [Call Control]Pressing hook key of the headset can not disconnected the call.//FA3.FC-282: disable_irq in gpio_switch_work().

	/* FIH, Karen Liao, 2010/05/05 { */
	/* F0XE.B-755: [Audio] Modify headset hook detect. */
	/* Remove plug-out debounce for hook key event when plug-out headset. */
	//msleep(500);
	if(state == 0)
		msleep(500);
	/* } FIH, Karen Liao, 2010/05/05 */
	
	if(gpio_get_value(data->gpio) == state)
	{
		// +++ FIH, KarenLiao, @20091023: [F0X.FC-663]: Allow user to use the normal headset.
		//switch_set_state(&data->sdev, state);
		//hook_switch_enable(state == 0, data->bHeadsetInserted);
		hook_switch_enable(state == 0, data->bHeadsetInserted);
		switch_set_state(&data->sdev, state);
		
		//change the sequence to delay switch_set_state for deciding the value of EnableAbnormalHS.
		// --- FIH, KarenLiao, @20091023: [F0X.FC-663]: Allow user to use the normal headset.
		
		data->bHeadsetInserted = (state == 0) ? true : false;		
		//printk(KERN_INFO "Headset: HeadsetInserted = %d\n", data->bHeadsetInserted);
	}

	enable_irq(data->irq);
	
#endif

// --- FIH, KarenLiao, @20090719: [F0X.FC-111]: [Audio] Add headset debounce mechanism.



}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;

//+++ FA3.FC-282: disable_irq in gpio_switch_work().
/*
#ifdef CONFIG_FIH_FXX //FA3.B-64: [Call Control]Pressing hook key of the headset can not disconnected the call.
	disable_irq(switch_data->irq);
#endif
*/
//--- FA3.FC-282: disable_irq in gpio_switch_work().

	/* FIH, Karen Liao, 2010/04/29 { */
	/* F0XE.B-697: [Audio] The music from BT headset is not smooth when plug in wired headset. */
	//schedule_work(&switch_data->work);
	queue_work(headset_switch_wq, &switch_data->work);
	/* } FIH, Karen Liao, 2010/04/29 */
	
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int gpio_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	int ret = 0;
	int r; //FIH, KarenLiao, @20090731: [F0X.FC-41]: The action of Inserting headset is the wake up action.

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_gpio_print_state;
	switch_data->bHeadsetInserted = false; // FIH, KarenLiao, @20090719: [F0X.FC-111]: [Audio] Add headset debounce mechanism.
		
    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = gpio_request(switch_data->gpio, pdev->name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	INIT_WORK(&switch_data->work, gpio_switch_work);
	
	/* FIH, Karen Liao, 2010/04/29 { */
	/* F0XE.B-697: [Audio] The music from BT headset is not smooth when plug in wired headset. */
	
	headset_switch_wq = create_singlethread_workqueue("switch_gpio");
	
	if (!headset_switch_wq) {
		return -EBUSY;
	}
	
	/* } FIH, Karen Liao, 2010/04/29 */

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

	// +++ FIH, KarenLiao, @20090518: Add for headset detection.

#ifdef CONFIG_FIH_FXX
	ret = request_irq(switch_data->irq, gpio_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, pdev->name, switch_data);
#else
	ret = request_irq(switch_data->irq, gpio_irq_handler,
            IRQF_TRIGGER_LOW, pdev->name, switch_data);
#endif

	// --- FIH, KarenLiao, @20090518: Add for headset detection.
	
	if (ret < 0)
		goto err_request_irq;


//+++ FIH, KarenLiao, @20090731: [F0X.FC-41]: The action of Inserting headset is the wake up action.
#ifdef CONFIG_FIH_FXX   
	r = enable_irq_wake(MSM_GPIO_TO_INT(switch_data->gpio));

	if (r < 0){
		//printk(KERN_ERR "gpio_switch_prob: "
		//	"enable_irq_wake failed for HS detect\n");
	}
#endif			
//--- FIH, KarenLiao, @20090731: [F0X.FC-41]: The action of Inserting headset is the wake up action.


	/* Perform initial detection */
	gpio_switch_work(&switch_data->work);

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
    switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= __devexit_p(gpio_switch_remove),
	.driver		= {
// +++ FIH, KarenLiao, @20090518: Add for headset detection.	

#ifdef CONFIG_FIH_FXX	
	.name	= "switch_gpio",
#else
	.name	= "switch-gpio",
#endif

// --- FIH, KarenLiao, @20090518: Add for headset detection.
	
	.owner	= THIS_MODULE,
	},
};

static int __init gpio_switch_init(void)
{
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

module_init(gpio_switch_init);
module_exit(gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
