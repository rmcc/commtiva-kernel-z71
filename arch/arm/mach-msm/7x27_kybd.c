#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/bootmem.h>
#include <linux/slab.h>

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/delay.h>

#include <mach/gpio.h>
#include <mach/7x27_kybd.h>

#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>

/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change keypad suspend/resume function
to earlysuspend */
#include <linux/earlysuspend.h>
/* } FIH, SimonSSChang, 2009/09/04 */

#include "smd_rpcrouter.h"
//++++++++++++++++++++++++FIH_F0X_misty
#include "../../../kernel/power/power.h"
#include <linux/suspend.h>

bool b_EnableWakeKey = false;
bool b_EnableIncomingCallWakeKey = false;

bool b_Key1_DisableIrq = false;
bool b_Key2_DisableIrq = false;
bool b_Center_DisableIrq = false;

bool b_Key1_EnableWakeIrq = false;
bool b_Key2_EnableWakeIrq = false;
bool b_VolUp_EnableWakeIrq = false;
bool b_VolDown_EnableWakeIrq = false;
bool b_HookKey_EnableWakeIrq = false;  //FIH, KarenLiao@20100304: F0X.B-9873: [Call control]Cannot end the call when long press hook key.
static bool SetupKeyFail=false;
void KeySetup(void);
//-----------------------FIH_F0X_misty
// FIH, WillChen, 2009/08/21 ++
#ifdef CONFIG_FIH_FXX_FORCEPANIC
#include <linux/proc_fs.h>
// FIH, WillChen, 2009/08/21 --

// FIH, WillChen, 2009/08/14 ++
//Press VolumeUp+VolumeDown+End key to force panic and dump log
bool VUP_Key = false;
bool VDN_Key = false;
bool END_key = false;
static int flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(wq);
// FIH, WillChen, 2009/08/14 --
#endif//#ifdef CONFIG_FIH_FXX_FORCEPANIC
#ifdef CONFIG_FIH_FXX
//+FIH_Misty
int g_center_pin = 124;
bool g_centerkey = true;
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 { 
bool g_End = true;
bool g_Send=true;
//bool g_SndEndkey = true;
// } FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 
int g_HWID=0;
int g_ORIGHWID=0;
static int EnableKeyInt = 0; // 1:enable key interrupt
//-FIH_Misty
#endif
#define Q7x27_kybd_name "7x27_kybd"

#define VOLUME_KEY_ENABLE               1
#define CAMERA_KEY_ENABLE               1
#define SWITCH_KEY_ENABLE               1
#define CENTER_KEY_ENABLE               1

#define HEADSET_DECTECT_SUPPORT         1

#define KEYPAD_DEBUG                    1

/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#define ACTIVE_MODE_ENABLE              1

#define ACTIVE_LOW  0
#define ACTIVE_HIGH 1
/* } FIH, PeterKCTseng, @20090520 */
#define GPIO_NEG_Y 93

/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
    struct  input_active_type   {
	    int key_1_pin_actype;
	    int key_2_pin_actype;
    	int volup_pin_actype;
	    int voldn_pin_actype;
	    int cam_sw_t_pin_actype;
	    int cam_sw_f_pin_actype;
	    int hook_sw_pin_actype;            
        int center_pin_actype;
    };        
#endif
/* } FIH, PeterKCTseng, @20090520 */


struct Q7x27_kybd_record {
	struct	input_dev *Q7x27_kybd_idev;
	int key_1_pin;
	int key_2_pin;
	int volup_pin;
	int voldn_pin;
	int cam_sw_t_pin;
	int cam_sw_f_pin;
	int hook_sw_pin;

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
	int center_pin;
/* } FIH, PeterKCTseng, @20090527 */

	uint8_t kybd_connected;
	struct	delayed_work kb_cmdq;

	//struct	work_struct kybd_generalkey;
    struct  work_struct kybd_generalkey1;   //FIH_key
    struct  work_struct kybd_generalkey2;   //FIH_key
	//struct	work_struct kybd_volkey;

#if VOLUME_KEY_ENABLE // Peter, Debug
	struct	work_struct kybd_volkey1;
	struct	work_struct kybd_volkey2;    
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
	struct	work_struct kybd_camkey1;
	struct	work_struct kybd_camkey2;
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
	struct	work_struct hook_switchkey;
#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
	struct	work_struct kybd_centerkey;
#endif
/* } FIH, PeterKCTseng, @20090527 */

/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
    struct  input_active_type   active;
#endif
/* } FIH, PeterKCTseng, @20090520 */

/* FIH, PeterKCTseng, @20090527 { */
/* phone jack dectect             */
#if HEADSET_DECTECT_SUPPORT
	int	bHookSWIRQEnabled;
#endif
/* } FIH, PeterKCTseng, @20090527 */

/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change keypad suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend Q7x27_kybd_early_suspend_desc;
    struct platform_device *pdev;
#endif
#endif
/* } FIH, SimonSSChang, 2009/09/04 */
};

struct input_dev *kpdev		= NULL;
struct Q7x27_kybd_record *rd	= NULL;

enum kbd_inevents {
/* FIH, NicoleWeng, @20100506 { */	
	KBD_DEBOUNCE_TIME	= 5,  // 50ms handle in irqhandler
/* } FIH, NicoleWeng, @20100506  */		
	KBD_IN_KEYPRESS		= 1,
	KBD_IN_KEYRELEASE	= 0,
};

#if KEYPAD_DEBUG    // debugging
bool    hookswitchflag = true;
#endif



#if SWITCH_KEY_ENABLE // Peter, Debug

struct workqueue_struct *headset_hook_wq;  //FIH, Karen Liao, 2010/05/05, F0XE.B-755: [Audio] Modify headset hook detect. 

#define KEY_HEADSETHOOK                 227 // KEY_F15
#define KEY_RINGSWITCH                  184 //FIH, KarenLiao, @200907xx: [F0X.FC-41]: The action of Inserting headset is the wake up action.
/* FIH, PeterKCTseng, @20090603 { */
/* let switch_gpio.c control it   */
#if 0
#define PM_MIC_EN_API_PROG              0x30000061
#define PM_MIC_EN_API_VERS              0x00010001 
#define PM_MIC_EN_CONFIG_PROC           27

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
/* } FIH, PeterKCTseng, @20090603 */

#endif


/*
 * For END_KEY
 */
//struct input_dev *msm_keypad_get_input_dev(void)
//{	
//	return kpdev;
//}
//EXPORT_SYMBOL(msm_keypad_get_input_dev);
/*
 * For POWER_KEY/END_KEY
 */
/* FIH, PeterKCTseng, @20090526 { */
/* power key support              */
struct input_dev *fih_msm_keypad_get_input_dev(void)
{	
    printk(KERN_INFO "fih_msm_keypad_get_input_dev: Pressed POWER_KEY\n");
	return kpdev;
}
EXPORT_SYMBOL(fih_msm_keypad_get_input_dev);
/* } FIH, PeterKCTseng, @20090526 */
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 { 
static int g_BackCover=0;
static ssize_t show_7X27_backcover_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	
      if(g_Send && !g_End)//FST
      {
		g_BackCover= gpio_get_value(41);
	  }
	   printk(KERN_INFO "@@@@@@@@@@@@@@@g_BackCover:%d@@@@@@@@@@@@@@@@@\n",g_BackCover);
       return sprintf(buf, "%d\r\n", g_BackCover);
}
static DEVICE_ATTR(7X27_backcover, S_IRUGO, show_7X27_backcover_state, NULL);

static struct attribute *dev_attrs[] = {
        &dev_attr_7X27_backcover.attr,
        NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};
// } FIH, NicoleWeng, 2010/05/26 for F0XE.B-955

/* FIH, SimonSSChang, 2009/07/28 { */
/* [FXX_CR], F0X.FC-116 Add option for wake up source*/
#ifdef CONFIG_FIH_FXX
bool key_wakeup_get(void)
{
    //printk(KERN_INFO "Simon: key_wakeup_get() return %d\n", b_EnableWakeKey);
    return b_EnableWakeKey;
}
EXPORT_SYMBOL(key_wakeup_get);

int key_wakeup_set(int on)
{
    if(on)
    {
        b_EnableWakeKey = true;
        //printk(KERN_INFO "Simon: key_wakeup_set() %d\n", b_EnableWakeKey);
        return 0;
    }
    else
    {
        b_EnableWakeKey = false;
        //printk(KERN_INFO "Simon: key_wakeup_set() %d\n", b_EnableWakeKey);        
        return 0;
    }
}    
EXPORT_SYMBOL(key_wakeup_set);
#endif
/* } FIH, SimonSSChang, 2009/07/28 */

/* FIH, SimonSSChang, 2009/09/10 { */
/* [FXX_CR], To enable Send & End key wakeup when incoming call*/
#ifdef CONFIG_FIH_FXX
bool incoming_call_get(void)
{
    //printk(KERN_INFO "Simon: incoming_call_get() return %d\n", b_EnableIncomingCallWakeKey);
    return b_EnableIncomingCallWakeKey;
}
EXPORT_SYMBOL(incoming_call_get);

int incoming_call_set(int on)
{
    if(on)
    {
        b_EnableIncomingCallWakeKey = true;
        //printk(KERN_INFO "Simon: incoming_call_set() %d\n", b_EnableIncomingCallWakeKey);
        return 0;
    }
    else
    {
        b_EnableIncomingCallWakeKey = false;
        //printk(KERN_INFO "Simon: incoming_call_set() %d\n", b_EnableIncomingCallWakeKey);        
        return 0;
    }
}    
EXPORT_SYMBOL(incoming_call_set);
#endif/* } FIH, SimonSSChang, 2009/09/10 */

/* FIH, NicoleWeng, @20100506 { */	
unsigned long lastTime = 0;
/* } FIH, NicoleWeng, @20100506  */	

static irqreturn_t Q7x27_kybd_irqhandler(int irq, void *dev_id)
{
	struct Q7x27_kybd_record *kbdrec = dev_id;
	
    printk(KERN_INFO "irqreturn_t Q7x27_kybd_irqhandler+, irq= %X \n", irq);
    
/* FIH, NicoleWeng, @20100506 { */	//handle debounce time here and remove debonce time in work queue
	if(lastTime!=0 && time_after(lastTime+KBD_DEBOUNCE_TIME, jiffies)){
		return IRQ_HANDLED;
	}	
	lastTime = jiffies;
/* } FIH, NicoleWeng, @20100506  */	

    if (kbdrec->kybd_connected) {

		if (MSM_GPIO_TO_INT(kbdrec->key_1_pin) == irq){
			schedule_work(&kbdrec->kybd_generalkey1);				
		} else if (MSM_GPIO_TO_INT(kbdrec->key_2_pin) == irq){
			schedule_work(&kbdrec->kybd_generalkey2);
		}
		
#if VOLUME_KEY_ENABLE // Peter, Debug
		else if (MSM_GPIO_TO_INT(kbdrec->volup_pin) == irq){
			schedule_work(&kbdrec->kybd_volkey1);
        } else if (MSM_GPIO_TO_INT(kbdrec->voldn_pin) == irq) {
			schedule_work(&kbdrec->kybd_volkey2);            
		}
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
		else if (MSM_GPIO_TO_INT(kbdrec->cam_sw_t_pin) == irq){
			schedule_work(&kbdrec->kybd_camkey1);
        } else if (MSM_GPIO_TO_INT(kbdrec->cam_sw_f_pin) == irq) {
			schedule_work(&kbdrec->kybd_camkey2);            
		}
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
		else if (MSM_GPIO_TO_INT(kbdrec->hook_sw_pin) == irq){
			/* FIH, Karen Liao, 2010/05/05 { */
    		/* F0XE.B-755: [Audio] Modify headset hook detect. */
			//schedule_work(&kbdrec->hook_switchkey);
			queue_work(headset_hook_wq, &kbdrec->hook_switchkey);
			/* } FIH, Karen Liao, 2010/05/05 */
        }
#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
		else if (MSM_GPIO_TO_INT(kbdrec->center_pin) == irq){
			schedule_work(&kbdrec->kybd_centerkey);
        }
#endif
/* } FIH, PeterKCTseng, @20090527 */
	
	}	
	return IRQ_HANDLED;
}

static int Q7x27_kybd_irqsetup(struct Q7x27_kybd_record *kbdrec)
{
	int rc;
		
	/* KEY 1 and KEY 2 keys interrupt */
	//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 { 
	//if(g_SndEndkey)
	if(g_Send)
	//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 { 	
	{
    	rc = request_irq(MSM_GPIO_TO_INT(kbdrec->key_1_pin), &Q7x27_kybd_irqhandler,
    			     (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
    			     Q7x27_kybd_name, kbdrec);
    	if (rc < 0) {
    		//printk(KERN_ERR
    		//       "Could not register for  %s interrupt "
    		//       "(rc = %d)\n", Q7x27_kybd_name, rc);
    		rc = -EIO;
    	}
    //FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 { 
    }
    if(g_End)
    {
    //FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 { 
    	rc = request_irq(MSM_GPIO_TO_INT(kbdrec->key_2_pin), &Q7x27_kybd_irqhandler,
    			     (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
    			     Q7x27_kybd_name, kbdrec);
    	if (rc < 0) {
    		//printk(KERN_ERR
    		//       "Could not register for  %s interrupt "
    		//       "(rc = %d)\n", Q7x27_kybd_name, rc);
    		rc = -EIO;
        }
    }
	
	
#if VOLUME_KEY_ENABLE // Peter, Debug
	/* Vol UP and Vol DOWN keys interrupt */
	rc = request_irq(MSM_GPIO_TO_INT(kbdrec->volup_pin), &Q7x27_kybd_irqhandler,
			     (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING), 
			     Q7x27_kybd_name, kbdrec);
	if (rc < 0) {
		//printk(KERN_ERR
		//       "Could not register for  %s interrupt "
		//       "(rc = %d)\n", Q7x27_kybd_name, rc);
		rc = -EIO;
	}
	rc = request_irq(MSM_GPIO_TO_INT(kbdrec->voldn_pin), &Q7x27_kybd_irqhandler,
			     (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING), 
			     Q7x27_kybd_name, kbdrec);
	if (rc < 0) {
		//printk(KERN_ERR
		//       "Could not register for  %s interrupt "
		//       "(rc = %d)\n", Q7x27_kybd_name, rc);
		rc = -EIO;
	}
#endif
	
#if CAMERA_KEY_ENABLE // Peter, Debug
	/* CAMERA and FOCUS keys interrupt */
	rc = request_irq(MSM_GPIO_TO_INT(kbdrec->cam_sw_t_pin), &Q7x27_kybd_irqhandler,
			     (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING), 
			     Q7x27_kybd_name, kbdrec);
	if (rc < 0) {
		//printk(KERN_ERR
		//       "Could not register for  %s interrupt "
		//       "(rc = %d)\n", Q7x27_kybd_name, rc);
		rc = -EIO;
	}
	rc = request_irq(MSM_GPIO_TO_INT(kbdrec->cam_sw_f_pin), &Q7x27_kybd_irqhandler,
			     (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING), 
			     Q7x27_kybd_name, kbdrec);
	if (rc < 0) {
		//printk(KERN_ERR
		//       "Could not register for  %s interrupt "
		//       "(rc = %d)\n", Q7x27_kybd_name, rc);
		rc = -EIO;
	}
#endif
	
/* FIH, PeterKCTseng, @20090603 { */
/* let switch_gpio.c control it   */
#if 0 // SWITCH_KEY_ENABLE // Peter, Debug
	/* Hook Switch key interrupt */
	rc = request_irq(MSM_GPIO_TO_INT(kbdrec->hook_sw_pin), &Q7x27_kybd_irqhandler,
			     (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING), 
			     Q7x27_kybd_name, kbdrec);
	if (rc < 0) {
		printk(KERN_ERR
		       "Could not register for  %s interrupt "
		       "(rc = %d)\n", Q7x27_kybd_name, rc);
		rc = -EIO;
	}
#endif
/* } FIH, PeterKCTseng, @20090603 */

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
	if(g_centerkey)//to avoid F903 enable center key IST
	{
		rc = request_irq(MSM_GPIO_TO_INT(kbdrec->center_pin), &Q7x27_kybd_irqhandler,
				     (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING), 
				     Q7x27_kybd_name, kbdrec);
		if (rc < 0) {
			//printk(KERN_ERR
			//       "Could not register for  %s interrupt "
			//       "(rc = %d)\n", Q7x27_kybd_name, rc);
			rc = -EIO;
		}
	}
#endif
/* } FIH, PeterKCTseng, @20090527 */

	return rc;
}


/* FIH, Henry Juang, 2009/11/20 ++*/
/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
int Q7x27_kybd_proximity_irqsetup(void)
{
	if(kpdev)
	{
		input_report_key(kpdev, KEY_RINGSWITCH, KBD_IN_KEYPRESS);
		//printk(KERN_INFO "FIH:Q7x27_kybd_proximity_irqsetup keypress KEY_RINGSWITCH = %d\n", KEY_LEFTSHIFT);
		
		input_report_key(kpdev, KEY_RINGSWITCH, KBD_IN_KEYRELEASE);
		//printk(KERN_INFO "FIH:Q7x27_kybd_proximity_irqsetup keyrelease KEY_RINGSWITCH = %d\n", KEY_LEFTSHIFT);
		input_sync(kpdev);
		return 0;
	}
	//printk(KERN_INFO "FIH:Q7x27_kybd_proximity_irqsetup does nothing.\n");
	return 0;

}
/* FIH, Henry Juang, 2009/11/20 --*/

/* FIH, PeterKCTseng, @20090527 { */
/* phone jack dectect             */
#if HEADSET_DECTECT_SUPPORT
int Q7x27_kybd_hookswitch_irqsetup(bool activate_irq)
{
	int rc = 0;
    bool  hook_sw_val;

	suspend_state_t SuspendState = PM_SUSPEND_ON;//FIH, KarenLiao, @20090731: [F0X.FC-41]: The action of Inserting headset is the wake up action.

	
    printk(KERN_INFO "Q7x27_kybd_hookswitch_irqsetup \n"); 

//+++ FIH, KarenLiao, @20090731: [F0X.FC-41]: The action of Inserting headset is the wake up action.
	SuspendState = get_suspend_state();
	if(SuspendState == PM_SUSPEND_MEM)//3
	{
		if(kpdev)
		{
			input_report_key(kpdev, KEY_RINGSWITCH, KBD_IN_KEYPRESS);
			//printk(KERN_INFO "FIH: keypress KEY_RINGSWITCH = %d\n", KEY_LEFTSHIFT);
			
			input_report_key(kpdev, KEY_RINGSWITCH, KBD_IN_KEYRELEASE);
			//printk(KERN_INFO "FIH: keyrelease KEY_RINGSWITCH = %d\n", KEY_LEFTSHIFT);
			input_sync(kpdev);
		}
	}
//--- FIH, KarenLiao, @20090731: [F0X.FC-41]: The action of Inserting headset is the wake up action.

	if (activate_irq && (gpio_get_value(rd->hook_sw_pin) == 1)) {

/* FIH, PeterKCTseng, @20090603 { */
/* clear pending interrupt        */
        //gpio_clear_detect_status(rd->hook_sw_pin);
   	    hook_sw_val = (bool)gpio_get_value(rd->hook_sw_pin);
  	    //printk(KERN_INFO "Read back hook switch eky <%d>\n", hook_sw_val);
        mdelay(250);
/* } FIH, PeterKCTseng, @20090603 */

    	rc = request_irq(MSM_GPIO_TO_INT(rd->hook_sw_pin), &Q7x27_kybd_irqhandler,
			        (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING), 
			        Q7x27_kybd_name, rd);
	    if (rc < 0) {
    		//printk(KERN_ERR
		    //    "Could not register for  %s interrupt "
		    //    "(rc = %d)\n", Q7x27_kybd_name, rc);
		    rc = -EIO;
	    }
        //printk(KERN_INFO "Hook Switch IRQ Enable! \n");
		rd->bHookSWIRQEnabled = true;
	} else {
		if (rd->bHookSWIRQEnabled)  {
			//printk(KERN_INFO "Free IRQ\n");
    		free_irq(MSM_GPIO_TO_INT(rd->hook_sw_pin), rd);
            //printk(KERN_INFO "Hook Switch IRQ disable! \n");
			rd->bHookSWIRQEnabled = false;
		}
	}
	
	return rc;

}
#endif
//EXPORT_SYMBOL(Q7x27_kybd_hookswitch_irqsetup);
/* } FIH, PeterKCTseng, @20090527 */

static int Q7x27_kybd_release_gpio(struct Q7x27_kybd_record *kbrec)
{
	int kbd_key_1_pin	= kbrec->key_1_pin;
	int kbd_key_2_pin	= kbrec->key_2_pin;

#if VOLUME_KEY_ENABLE // Peter, Debug
	int kbd_volup_pin	= kbrec->volup_pin;
	int kbd_voldn_pin	= kbrec->voldn_pin;
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
	int kbd_cam_sw_f_pin	= kbrec->cam_sw_f_pin;
	int kbd_cam_sw_t_pin	= kbrec->cam_sw_t_pin;
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
	int kbd_hook_sw_pin	= kbrec->hook_sw_pin;
#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
	int kbd_center_pin	= kbrec->center_pin;
#endif
/* } FIH, PeterKCTseng, @20090527 */

//	printk(KERN_INFO
//		 "releasing keyboard gpio pins %d,%d,%d,%d,%d,%d\n",
//		 kbd_volup_pin, kbd_voldn_pin, kbd_key_1_pin, kbd_key_2_pin, kbd_cam_sw_f_pin, kbd_cam_sw_t_pin);

	gpio_free(kbd_key_1_pin);
	gpio_free(kbd_key_2_pin);

#if VOLUME_KEY_ENABLE // Peter, Debug
	gpio_free(kbd_volup_pin);
	gpio_free(kbd_voldn_pin);
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
	gpio_free(kbd_cam_sw_t_pin);
	gpio_free(kbd_cam_sw_f_pin);
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
	gpio_free(kbd_hook_sw_pin);
#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
	gpio_free(kbd_center_pin);
#endif
/* } FIH, PeterKCTseng, @20090527 */

	return 0;
}

static int Q7x27_kybd_config_gpio(struct Q7x27_kybd_record *kbrec)
{
	int kbd_volup_pin	= kbrec->volup_pin;
	int kbd_voldn_pin	= kbrec->voldn_pin;
	int kbd_key_1_pin	= kbrec->key_1_pin;
	int kbd_key_2_pin	= kbrec->key_2_pin;
	int kbd_cam_sw_f_pin	= kbrec->cam_sw_f_pin;
	int kbd_cam_sw_t_pin	= kbrec->cam_sw_t_pin;
	int kbd_hook_sw_pin	= kbrec->hook_sw_pin;

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
	int kbd_center_pin	= kbrec->center_pin;
/* } FIH, PeterKCTseng, @20090527 */

	int rc;

    //printk(KERN_INFO "FIH: enter 1\n");
	rc = gpio_request(kbd_key_1_pin, "gpio_key_1_pin");
	if (rc) {
		//printk(KERN_ERR "gpio_request failed on pin %d (rc=%d)\n",
		//	kbd_key_1_pin, rc);
		goto err_gpioconfig;
	}
    	//printk(KERN_INFO "FIH: enter 2\n");
	rc = gpio_request(kbd_key_2_pin, "gpio_key_2_pin");
	if (rc) {
		//printk(KERN_ERR "gpio_request failed on pin %d (rc=%d)\n",
		//	kbd_key_2_pin, rc);
		goto err_gpioconfig;
	}

#if VOLUME_KEY_ENABLE // Peter, Debug
	rc = gpio_request(kbd_volup_pin, "gpio_keybd_volup");
	if (rc) {
		//printk(KERN_ERR "gpio_request failed on pin %d (rc=%d)\n",
		//	kbd_volup_pin, rc);
		goto err_gpioconfig;
	}
	rc = gpio_request(kbd_voldn_pin, "gpio_keybd_voldn");
	if (rc) {
		//printk(KERN_ERR "gpio_request failed on pin %d (rc=%d)\n",
		//	kbd_voldn_pin, rc);
		goto err_gpioconfig;
	}
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
	rc = gpio_request(kbd_cam_sw_f_pin, "gpio_cam_sw_f");
	if (rc) {
		//printk(KERN_ERR "gpio_request failed on pin %d (rc=%d)\n",
		//	kbd_cam_sw_f_pin, rc);
		goto err_gpioconfig;
	}
	rc = gpio_request(kbd_cam_sw_t_pin, "gpio_cam_sw_t");
	if (rc) {
		//printk(KERN_ERR "gpio_request failed on pin %d (rc=%d)\n",
		//	kbd_cam_sw_t_pin, rc);
		goto err_gpioconfig;
	}
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
	rc = gpio_request(kbd_hook_sw_pin, "gpio_hook_sw");
	if (rc) {
		//printk(KERN_ERR "gpio_request failed on pin %d (rc=%d)\n",
		//	kbd_hook_sw_pin, rc);
		goto err_gpioconfig;
	}
#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
	rc = gpio_request(kbd_center_pin, "gpio_center");
	if (rc) {
		//printk(KERN_ERR "gpio_request failed on pin %d (rc=%d)\n",
		//	kbd_center_pin, rc);
		goto err_gpioconfig;
	}
#endif
/* } FIH, PeterKCTseng, @20090527 */

    //printk(KERN_INFO "FIH: enter 3\n");
	rc = gpio_direction_input(kbd_key_1_pin);
	if (rc) {
		//printk(KERN_ERR "gpio_direction_input failed on "
		//       "pin %d (rc=%d)\n", kbd_key_1_pin, rc);
		goto err_gpioconfig;
	}
    	//printk(KERN_INFO "FIH: enter 4\n");
	rc = gpio_direction_input(kbd_key_2_pin);
	if (rc) {
		//printk(KERN_ERR "gpio_direction_output failed on "
		//       "pin %d (rc=%d)\n", kbd_key_2_pin, rc);
		goto err_gpioconfig;
	}

#if VOLUME_KEY_ENABLE // Peter, Debug
	rc = gpio_direction_input(kbd_volup_pin);
	if (rc) {
		//printk(KERN_ERR "gpio_direction_input failed on "
		//       "pin %d (rc=%d)\n", kbd_volup_pin, rc);
		goto err_gpioconfig;
	}
	rc = gpio_direction_input(kbd_voldn_pin);
	if (rc) {
		//printk(KERN_ERR "gpio_direction_input failed on "
		//       "pin %d (rc=%d)\n", kbd_voldn_pin, rc);
		goto err_gpioconfig;
	}
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
	rc = gpio_direction_input(kbd_cam_sw_f_pin);
	if (rc) {
		//printk(KERN_ERR "gpio_direction_input failed on "
		//       "pin %d (rc=%d)\n", kbd_cam_sw_f_pin, rc);
		goto err_gpioconfig;
	}
	rc = gpio_direction_input(kbd_cam_sw_t_pin);
	if (rc) {
		//printk(KERN_ERR "gpio_direction_input failed on "
		//       "pin %d (rc=%d)\n", kbd_cam_sw_t_pin, rc);
		goto err_gpioconfig;
	}
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
	rc = gpio_direction_input(kbd_hook_sw_pin);
	if (rc) {
		//printk(KERN_ERR "gpio_direction_input failed on "
		//       "pin %d (rc=%d)\n", kbd_hook_sw_pin, rc);
		goto err_gpioconfig;
	}
#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
	rc = gpio_direction_input(kbd_center_pin);
	if (rc) {
		//printk(KERN_ERR "gpio_direction_input failed on "
		//       "pin %d (rc=%d)\n", kbd_center_pin, rc);
		goto err_gpioconfig;
	}

/* FIH, PeterKCTseng, @20090601 { */
//	rc = gpio_request(kbd_center_pin, "gpio_center");
//	if (rc) {
//		printk(KERN_ERR "gpio_request failed on pin %d (rc=%d)\n",
//			kbd_center_pin, rc);
//		goto err_gpioconfig;
//	}
/* } FIH, PeterKCTseng, @20090601 */
#endif
/* } FIH, PeterKCTseng, @20090527 */

    //printk(KERN_INFO "FIH: enter 5\n");
    return rc;
    
err_gpioconfig:
    //printk(KERN_INFO "FIH: enter 6\n");    
	Q7x27_kybd_release_gpio(kbrec);
	return rc;
}

static void Q7627_kybd_generalkey1(struct work_struct *work)
{
	struct Q7x27_kybd_record *kbdrec	= container_of(work, struct Q7x27_kybd_record, kybd_generalkey1);
	struct input_dev *idev		= kbdrec->Q7x27_kybd_idev;	
	
	bool key_1_val			= (bool)gpio_get_value(kbdrec->key_1_pin);
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++misty
	suspend_state_t SuspendState = PM_SUSPEND_ON;//0
	//-----------------------------------------------------------------misty
/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
	bool state;
   // int HWID = FIH_READ_HWID_FROM_SMEM(); 
#endif
/* } FIH, PeterKCTseng, @20090520 */
	
	//printk(KERN_INFO "KEY 1 <%d>\n", key_1_val);

	disable_irq(MSM_GPIO_TO_INT(kbdrec->key_1_pin));
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++misty
    if(EnableKeyInt)
    {
        SuspendState = get_suspend_state();
    
        if(SuspendState == PM_SUSPEND_MEM)//3
        {
            if(idev)
            {
            	input_report_key(idev, KEY_END, KBD_IN_KEYPRESS);
               		printk(KERN_INFO "FIH: keypress KEY_END\n");
            	
            	input_report_key(idev, KEY_END, KBD_IN_KEYRELEASE);
               		printk(KERN_INFO "FIH: keyrelease KEY_END\n");
            	input_sync(idev);
            }
        }
        //-----------------------------------------------------------------misty
        else
        {
        /* FIH, PeterKCTseng, @20090520 { */
        /* The active type of input pin   */
        #if ACTIVE_MODE_ENABLE // Peter, Debug
            state = (kbdrec->active.key_1_pin_actype == ACTIVE_HIGH) ? key_1_val : !key_1_val;
        	//printk(KERN_INFO "active type= %d\n", kbdrec->active.key_1_pin_actype);
        #endif
        /* } FIH, PeterKCTseng, @20090520 */
            if(idev)
            {
            	if (state) {
            		input_report_key(idev, KEY_END, KBD_IN_KEYPRESS);
            		printk(KERN_INFO "FIH: keypress KEY_END\n");
        // FIH, WillChen, 2009/08/21 ++
        //Press VolumeUp+VolumeDown+End key to force panic and dump log
        #ifdef CONFIG_FIH_FXX_FORCEPANIC
            		END_key = true;
            		if (VUP_Key && VDN_Key && END_key)
            		{
            			printk(KERN_ERR "KPD: Three key panic!!\n");
            			flag = 1;
        				wake_up(&wq);
        				msleep(5000);
        				panic("Three key panic");
        			}
        #endif
        // FIH, WillChen, 2009/08/21 --			
            	} else {
            		input_report_key(idev, KEY_END, KBD_IN_KEYRELEASE);
            		printk(KERN_INFO "FIH: keyrelease KEY_END\n");
            		
        // FIH, WillChen, 2009/08/21 ++
        //Press VolumeUp+VolumeDown+End key to force panic and dump log
        #ifdef CONFIG_FIH_FXX_FORCEPANIC
            		END_key = false;
        #endif
        // FIH, WillChen, 2009/08/21 --
            	}
            	
            	input_sync(idev);
            }
        	
        	
        }	
    }//if(EnableKeyInt)
	enable_irq(MSM_GPIO_TO_INT(kbdrec->key_1_pin));
}

static void Q7627_kybd_generalkey2(struct work_struct *work)
{
	struct Q7x27_kybd_record *kbdrec	= container_of(work, struct Q7x27_kybd_record, kybd_generalkey2);
	struct input_dev *idev		= kbdrec->Q7x27_kybd_idev;	
	
	bool key_2_val			= (bool)gpio_get_value(kbdrec->key_2_pin);
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++misty
	suspend_state_t SuspendState = PM_SUSPEND_ON;//0
	//-----------------------------------------------------------------misty
/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
	bool state;
#endif
/* } FIH, PeterKCTseng, @20090520 */

	//printk(KERN_INFO "KEY 2 <%d>\n", key_2_val);

	disable_irq(MSM_GPIO_TO_INT(kbdrec->key_2_pin));
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++misty
    if(EnableKeyInt)
    {
        SuspendState = get_suspend_state();
        if(SuspendState == PM_SUSPEND_MEM)
        {
            if(idev)
            {
            	input_report_key(idev, KEY_SEND, KBD_IN_KEYPRESS);
            		printk(KERN_INFO "FIH: keypress KEY_SEND\n");
            	input_report_key(idev, KEY_SEND, KBD_IN_KEYRELEASE);
            		printk(KERN_INFO "FIH: keyrelease KEY_SEND\n");
            	input_sync(idev);
            }
        }
        //-----------------------------------------------------------------misty
        else
        {
        /* FIH, PeterKCTseng, @20090520 { */
        /* The active type of input pin   */
        #if ACTIVE_MODE_ENABLE // Peter, Debug
            state = (kbdrec->active.key_2_pin_actype == ACTIVE_HIGH) ? key_2_val : !key_2_val;
        	//printk(KERN_INFO "active type= %d \n", state);
        #endif
        /* } FIH, PeterKCTseng, @20090520 */
            if(idev)
            {
            	if (state) {
            		input_report_key(idev, KEY_SEND, KBD_IN_KEYPRESS);
            		printk(KERN_INFO "FIH: keypress KEY_SEND\n");
            
            	} else {
            		input_report_key(idev, KEY_SEND, KBD_IN_KEYRELEASE);
            		printk(KERN_INFO "FIH: keyrelease KEY_SEND\n");
            	}
            
            	input_sync(idev);
            }        	
        	
        }
    }//if(EnableKeyInt)
    
	enable_irq(MSM_GPIO_TO_INT(kbdrec->key_2_pin));
}

static void Q7x27_kybd_generalkey1(struct work_struct *work)
{
	struct Q7x27_kybd_record *kbdrec	= container_of(work, struct Q7x27_kybd_record, kybd_generalkey1);
	struct input_dev *idev		= kbdrec->Q7x27_kybd_idev;	
	
	bool key_1_val			= (bool)gpio_get_value(kbdrec->key_1_pin);
	//+++++++++++++++++++++++++++++++++FIH_F0X_misty
	suspend_state_t SuspendState = PM_SUSPEND_ON;//0
	//------------------------------FIH_F0X_misty
/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
	bool state;
 //   int HWID = FIH_READ_HWID_FROM_SMEM(); 
#endif
/* } FIH, PeterKCTseng, @20090520 */
	//printk(KERN_INFO "KEY 1 <%d>\n", key_1_val);

	disable_irq(MSM_GPIO_TO_INT(kbdrec->key_1_pin));
//+++++++++++++++++++++++++++++++FIH_F0X_misty
    if(EnableKeyInt)
    {
        SuspendState = get_suspend_state();
        if(SuspendState == PM_SUSPEND_MEM)
        {
            if(idev)
            {
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
                 state = (kbdrec->active.key_1_pin_actype == ACTIVE_HIGH) ? key_1_val : !key_1_val;
                if(g_Send && !g_End)//for FST,to judge back cover whether is open
                {   
                    if (state) {
                        input_report_key(idev, KEY_F23, KBD_IN_KEYPRESS);
                			printk(KERN_INFO "FIH: sss keypress KEY_SEND as KEY_F23(%d)\n",KEY_F23);
                		}
                		else
                		{
                	    	input_report_key(idev, KEY_F23, KBD_IN_KEYRELEASE);
                			printk(KERN_INFO "FIH: ssskeyrelease KEY_SEND as KEY_F23\n");                			
                		}
                		input_sync(idev);                    
                }
                else
                {
// } FIH, NicoleWeng, 2010/05/26 for F0XE.B-955                
            			input_report_key(idev, KEY_SEND, KBD_IN_KEYPRESS);
                		printk(KERN_INFO "FIH: keypress KEY_SEND\n");
            			input_report_key(idev, KEY_SEND, KBD_IN_KEYRELEASE);
                		printk(KERN_INFO "FIH: keyrelease KEY_SEND\n");
            			input_sync(idev);
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
                	}
// } FIH, NicoleWeng, 2010/05/26 for F0XE.B-955
            }
        }
        //-------------------------------FIH_F0X_misty
        else
        {
        /* FIH, PeterKCTseng, @20090520 { */
        /* The active type of input pin   */
        #if ACTIVE_MODE_ENABLE // Peter, Debug
            state = (kbdrec->active.key_1_pin_actype == ACTIVE_HIGH) ? key_1_val : !key_1_val;        	
        #endif
        /* } FIH, PeterKCTseng, @20090520 */
            if(idev)
            {
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
                if(g_Send && !g_End)//for FST,to judge back cover whether is open
                {   
                    if (state) {
                        input_report_key(idev, KEY_F23, KBD_IN_KEYPRESS);
                		printk(KERN_INFO "FIH: keypress KEY_SEND as KEY_F23\n");
                	}
                	else
                	{
                	    input_report_key(idev, KEY_F23, KBD_IN_KEYRELEASE);
                		printk(KERN_INFO "FIH: keyrelease KEY_SEND as KEY_F23\n");                		
                	}
                	input_sync(idev);
                    
                }
                else
                {
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
            			if (state) {
            				input_report_key(idev, KEY_SEND, KBD_IN_KEYPRESS);
                		printk(KERN_INFO "FIH: keypress KEY_SEND\n");
              		
            			} else {
            				input_report_key(idev, KEY_SEND, KBD_IN_KEYRELEASE);
                		printk(KERN_INFO "FIH: keyrelease KEY_SEND\n");
            			}            			
            			input_sync(idev);
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {            			
            		}	
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {        	
            }
        }
    }//if(EnableKeyInt)
	enable_irq(MSM_GPIO_TO_INT(kbdrec->key_1_pin));
}

static void Q7x27_kybd_generalkey2(struct work_struct *work)
{
	struct Q7x27_kybd_record *kbdrec	= container_of(work, struct Q7x27_kybd_record, kybd_generalkey2);
	struct input_dev *idev		= kbdrec->Q7x27_kybd_idev;	
	
	bool key_2_val			= (bool)gpio_get_value(kbdrec->key_2_pin);
	//+++++++++++++++++++++++++++++FIH_F0X_misty
	suspend_state_t SuspendState = PM_SUSPEND_ON;//0
	//-------------------------------FIH_F0X_misty
/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
	bool state;
#endif
/* } FIH, PeterKCTseng, @20090520 */
	//printk(KERN_INFO "KEY 2 <%d>\n", key_2_val);

	disable_irq(MSM_GPIO_TO_INT(kbdrec->key_2_pin));
//++++++++++++++++++++++++++++++FIH_F0X_misty
    if(EnableKeyInt)
    {
        SuspendState = get_suspend_state();
        if(SuspendState == PM_SUSPEND_MEM)//3
        {
            if(idev)
            {
            	input_report_key(idev, KEY_END, KBD_IN_KEYPRESS);
            		printk(KERN_INFO "FIH: keypress KEY_END\n");
            	
            	input_report_key(idev, KEY_END, KBD_IN_KEYRELEASE);
            		printk(KERN_INFO "FIH: keyrelease KEY_END\n");
            	input_sync(idev);
            }
        }
        //-------------------------------FIH_F0X_misty
        else
        {
        /* FIH, PeterKCTseng, @20090520 { */
        /* The active type of input pin   */
        #if ACTIVE_MODE_ENABLE // Peter, Debug
            state = (kbdrec->active.key_2_pin_actype == ACTIVE_HIGH) ? key_2_val : !key_2_val;
        	//printk(KERN_INFO "active type= %d \n", state);
        #endif
        /* } FIH, PeterKCTseng, @20090520 */
            if(idev)
            {
            	if (state) {
            		input_report_key(idev, KEY_END, KBD_IN_KEYPRESS);
            		printk(KERN_INFO "FIH: keypress KEY_END\n");
        // FIH, WillChen, 2009/08/14 ++
        //Press VolumeUp+VolumeDown+End key to force panic and dump log
        #ifdef CONFIG_FIH_FXX_FORCEPANIC
            		END_key = true;
            		if (VUP_Key && VDN_Key && END_key)
            		{
            			printk(KERN_ERR "KPD: Three key panic!!\n");
            			flag = 1;
        				wake_up(&wq);
        				msleep(5000);
        				panic("Three key panic");
        			}
        #endif
        // FIH, WillChen, 2009/08/14 --			
            	} else {
            		input_report_key(idev, KEY_END, KBD_IN_KEYRELEASE);
            		printk(KERN_INFO "FIH: keyrelease KEY_END\n");
            		
        // FIH, WillChen, 2009/08/14 ++
        //Press VolumeUp+VolumeDown+End key to force panic and dump log
        #ifdef CONFIG_FIH_FXX_FORCEPANIC
            		END_key = false;
        #endif
        // FIH, WillChen, 2009/08/14 --
            	}
            
            	input_sync(idev);
            }    	
        
        }	
    }//if(EnableKeyInt)
	enable_irq(MSM_GPIO_TO_INT(kbdrec->key_2_pin));
}

#if VOLUME_KEY_ENABLE // Peter, Debug
static void Q7x27_kybd_volkey1(struct work_struct *work)
{
	struct Q7x27_kybd_record *kbdrec= container_of(work, struct Q7x27_kybd_record, kybd_volkey1);
	struct input_dev *idev		= kbdrec->Q7x27_kybd_idev;	
	
	bool volup_val			= (bool)gpio_get_value(kbdrec->volup_pin);
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++misty
	suspend_state_t SuspendState = PM_SUSPEND_ON;//0
	//-----------------------------------------------------------------misty
/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
	bool state;
#endif
/* } FIH, PeterKCTseng, @20090520 */
	//printk(KERN_INFO "VOL UP <%d>\n", volup_val);

	disable_irq(MSM_GPIO_TO_INT(kbdrec->volup_pin));
//+++++++++++++++++++++++++++++++FIH_F0X_misty
    if(EnableKeyInt)
    {
        SuspendState = get_suspend_state();
        if(SuspendState == PM_SUSPEND_MEM)
        {
            if(idev)
            {
            	input_report_key(idev, KEY_VOLUMEUP, KBD_IN_KEYPRESS);
            		printk(KERN_INFO "FIH: keypress KEY_VOLUMEUP\n");
            	input_report_key(idev, KEY_VOLUMEUP, KBD_IN_KEYRELEASE);
            		printk(KERN_INFO "FIH: keyrelease KEY_VOLUMEUP\n");
            	input_sync(idev);
            }
        }
        //-------------------------------FIH_F0X_misty
        else
        {
        /* FIH, PeterKCTseng, @20090520 { */
        /* The active type of input pin   */
        #if ACTIVE_MODE_ENABLE // Peter, Debug
            state = (kbdrec->active.volup_pin_actype == ACTIVE_HIGH) ? volup_val : !volup_val;
        	//printk(KERN_INFO "active type= %d \n", state);
        #endif
        /* } FIH, PeterKCTseng, @20090520 */
            if(idev)
            {
            	if (state) {
            		input_report_key(idev, KEY_VOLUMEUP, KBD_IN_KEYPRESS);
            		//printk(KERN_INFO "FIH: keypress KEY_VOLUMEUP\n");
        // FIH, WillChen, 2009/08/14 ++
        //Press VolumeUp+VolumeDown+End key to force panic and dump log
        #ifdef CONFIG_FIH_FXX_FORCEPANIC
            		VUP_Key = true;
            		if (VUP_Key && VDN_Key && END_key)
            		{
            			printk(KERN_ERR "KPD: Three key panic!!\n");
            			flag = 1;
        				wake_up(&wq);
        				msleep(5000);
        				panic("Three key panic");
        			}
        #endif
        // FIH, WillChen, 2009/08/14 --
            	} else {
            		input_report_key(idev, KEY_VOLUMEUP, KBD_IN_KEYRELEASE);
            		//printk(KERN_INFO "FIH: keyrelease KEY_VOLUMEUP\n");		
            		
        // FIH, WillChen, 2009/08/14 ++
        //Press VolumeUp+VolumeDown+End key to force panic and dump log
        #ifdef CONFIG_FIH_FXX_FORCEPANIC
            		VUP_Key = false;
        #endif
        // FIH, WillChen, 2009/08/14 --
            	}
            	
            	input_sync(idev);
            }        	
        	
           }
    }//if(EnableKeyInt)
    	
	enable_irq(MSM_GPIO_TO_INT(kbdrec->volup_pin));
}

static void Q7x27_kybd_volkey2(struct work_struct *work)
{
	struct Q7x27_kybd_record *kbdrec= container_of(work, struct Q7x27_kybd_record, kybd_volkey2);
	struct input_dev *idev		= kbdrec->Q7x27_kybd_idev;	
	
	bool voldn_val			= (bool)gpio_get_value(kbdrec->voldn_pin);
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++misty
	suspend_state_t SuspendState = PM_SUSPEND_ON;//0
	//-----------------------------------------------------------------misty
/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
	bool state;
#endif
/* } FIH, PeterKCTseng, @20090520 */
	//printk(KERN_INFO "VOL DN <%d>\n", voldn_val);

	disable_irq(MSM_GPIO_TO_INT(kbdrec->voldn_pin));
//+++++++++++++++++++++++++++++++FIH_F0X_misty
    if(EnableKeyInt)
    {
        SuspendState = get_suspend_state();
        if(SuspendState == PM_SUSPEND_MEM)
        {
            if(idev)
            {
            	input_report_key(idev, KEY_VOLUMEDOWN, KBD_IN_KEYPRESS);
            		printk(KERN_INFO "FIH: keypress KEY_VOLUMEDOWN\n");
            	input_report_key(idev, KEY_VOLUMEDOWN, KBD_IN_KEYRELEASE);
            		printk(KERN_INFO "FIH: keyrelease KEY_VOLUMEDOWN\n");
            	input_sync(idev);
            }
        }
        //-------------------------------FIH_F0X_misty
        else
        {
        /* FIH, PeterKCTseng, @20090520 { */
        /* The active type of input pin   */
        #if ACTIVE_MODE_ENABLE // Peter, Debug
            state = (kbdrec->active.voldn_pin_actype == ACTIVE_HIGH) ? voldn_val : !voldn_val;
        	//printk(KERN_INFO "active type= %d \n", state);
        #endif
        /* } FIH, PeterKCTseng, @20090520 */
            if(idev)
            {
            	if (state) {
            		input_report_key(idev, KEY_VOLUMEDOWN, KBD_IN_KEYPRESS);
            		printk(KERN_INFO "FIH: keypress KEY_VOLUMEDOWN\n");
        // FIH, WillChen, 2009/08/14 ++
        //Press VolumeUp+VolumeDown+End key to force panic and dump log
        #ifdef CONFIG_FIH_FXX_FORCEPANIC
            		VDN_Key = true;
            		if (VUP_Key && VDN_Key && END_key)
            		{
            			printk(KERN_ERR "KPD: Three key panic!!\n");
            			flag = 1;
        				wake_up(&wq);
        				msleep(5000);
        				panic("Three key panic");
        			}
        #endif
        // FIH, WillChen, 2009/08/14 --
            	} else {
            		input_report_key(idev, KEY_VOLUMEDOWN, KBD_IN_KEYRELEASE);
            		printk(KERN_INFO "FIH: keyrelease KEY_VOLUMEDOWN\n");		
            		
        // FIH, WillChen, 2009/08/14 ++
        //Press VolumeUp+VolumeDown+End key to force panic and dump log
        #ifdef CONFIG_FIH_FXX_FORCEPANIC
            		VDN_Key = false;
        #endif
        // FIH, WillChen, 2009/08/14 --
            	}
            
            	input_sync(idev);
            			printk(KERN_INFO "FIH: keypress KEY_VOLUMEDOWN\n");	
            }	        
        	
         }
    }//if(EnableKeyInt)
	enable_irq(MSM_GPIO_TO_INT(kbdrec->voldn_pin));
}
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug

#define KEY_FOCUS                       KEY_F13

static void Q7x27_kybd_camkey1(struct work_struct *work)
{
	struct Q7x27_kybd_record *kbdrec= container_of(work, struct Q7x27_kybd_record, kybd_camkey1);
	struct input_dev *idev		= kbdrec->Q7x27_kybd_idev;	
	
	bool cam_sw_t_val		= (bool)gpio_get_value(kbdrec->cam_sw_t_pin);
/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
	bool state;
#endif
/* } FIH, PeterKCTseng, @20090520 */

	//printk(KERN_INFO "CAMERA KEY <%d>\n", cam_sw_t_val);

	disable_irq(MSM_GPIO_TO_INT(kbdrec->cam_sw_t_pin));

/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
    state = (kbdrec->active.cam_sw_t_pin_actype == ACTIVE_HIGH) ? cam_sw_t_val : !cam_sw_t_val;
	//printk(KERN_INFO "active type= %d \n", state);
#endif
/* } FIH, PeterKCTseng, @20090520 */
    if(EnableKeyInt)
    {
        if(idev)
        {
        	if (state) {
        		input_report_key(idev, KEY_CAMERA, KBD_IN_KEYPRESS); //report KEY CAMERA pressing
        		printk(KERN_INFO "FIH: keypress KEY_CAMERA\n");		
        	} else {
        		input_report_key(idev, KEY_CAMERA, KBD_IN_KEYRELEASE); //report KEY CAMERA releasing
        		printk(KERN_INFO "FIH: keyrelease KEY_CAMERA\n");		
        	}
        
        	input_sync(idev);
        }    	
    
    }//if(EnableKeyInt)
    
	enable_irq(MSM_GPIO_TO_INT(kbdrec->cam_sw_t_pin));
}	


static void Q7x27_kybd_camkey2(struct work_struct *work)
{
	struct Q7x27_kybd_record *kbdrec= container_of(work, struct Q7x27_kybd_record, kybd_camkey2);
	struct input_dev *idev		= kbdrec->Q7x27_kybd_idev;	
	
	bool cam_sw_f_val		= (bool)gpio_get_value(kbdrec->cam_sw_f_pin);
/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
	bool state;
#endif
/* } FIH, PeterKCTseng, @20090520 */

	//printk(KERN_INFO "FOCUS KEY <%d>\n", cam_sw_f_val);

	disable_irq(MSM_GPIO_TO_INT(kbdrec->cam_sw_f_pin));

/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
    state = (kbdrec->active.cam_sw_f_pin_actype == ACTIVE_HIGH) ? cam_sw_f_val : !cam_sw_f_val;
	//printk(KERN_INFO "active type= %d \n", state);
#endif
/* } FIH, PeterKCTseng, @20090520 */
    if(EnableKeyInt)
    {
        if(idev)
        {
        	if (state) {
        		input_report_key(idev, KEY_FOCUS, KBD_IN_KEYPRESS); //report KEY FOCUS pressing
        		printk(KERN_INFO "FIH: keypress KEY_FOCUS\n");		
        	} else {
        		input_report_key(idev, KEY_FOCUS, KBD_IN_KEYRELEASE); //report KEY FOCUS releasing
        		printk(KERN_INFO "FIH: keyrelease KEY_FOCUS\n");		
        	}
        
        	input_sync(idev);
        }    	
    	
    }//if(EnableKeyInt)
    
	enable_irq(MSM_GPIO_TO_INT(kbdrec->cam_sw_f_pin));
}
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
static void Q7x27_hook_switchkey(struct work_struct *work)
{
	struct Q7x27_kybd_record *kbdrec= container_of(work, struct Q7x27_kybd_record, hook_switchkey);
	struct input_dev *idev		= kbdrec->Q7x27_kybd_idev;	
	bool hook_sw_val			= (bool)gpio_get_value(kbdrec->hook_sw_pin);
	
/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
	bool state;
#endif
/* } FIH, PeterKCTseng, @20090520 */

	/* FIH, Karen Liao, 2010/05/05 { */
    /* F0XE.B-755: [Audio] Modify headset hook detect. */
	/* Avoid hook key press event when plug-out headset. */
	if(!hook_sw_val)
		msleep(500);
	/* } FIH, Karen Liao, 2010/05/05 */
		
	if(rd->bHookSWIRQEnabled == true){  //FA3.FC-282: report key only when bHookSWIRQEnabled is true.
	
		//printk(KERN_INFO "HOOK SW <%d>\n", hook_sw_val);	
		disable_irq(MSM_GPIO_TO_INT(kbdrec->hook_sw_pin));
		
	/* FIH, PeterKCTseng, @20090520 { */
	/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
	    state = (kbdrec->active.hook_sw_pin_actype == ACTIVE_HIGH) ? hook_sw_val : !hook_sw_val;
		//printk(KERN_INFO "active type= %d \n", state);
#endif
	/* } FIH, PeterKCTseng, @20090520 */
	    if(idev)
	    {
	    	if (state) {
	    		input_report_key(idev, KEY_HEADSETHOOK, KBD_IN_KEYPRESS); //report KEY_HEADSETHOOK pressing
	    		//printk(KERN_INFO "FIH: keypress KEY_HEADSETHOOK= %d\n", KEY_HEADSETHOOK);
	    	} else {
	    		input_report_key(idev, KEY_HEADSETHOOK, KBD_IN_KEYRELEASE); //report KEY_HEADSETHOOK releasing
	    		//printk(KERN_INFO "FIH: keyrelease KEY_HEADSETHOOK= %d\n", KEY_HEADSETHOOK);
	    	}
	    
	    	input_sync(idev);	    }
		
		
		
		enable_irq(MSM_GPIO_TO_INT(kbdrec->hook_sw_pin));
	}//FA3.FC-282: report key only when bHookSWIRQEnabled is true.
}
#endif


/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug

#define SELECT_INPUT_BTN  230//BTN_LEFT	//misty change HOME key/* This is chosen to match with the mousedev driver */
#define SELECT_INPUT_MENU 229 //MENU key //used for F902 PR2
#define SELECT_INPUT_ENTER 28 //Enter key //used for F910/F911 jogball
static void Q7x27_kybd_centerkey(struct work_struct *work)
{
	struct Q7x27_kybd_record *kbdrec= container_of(work, struct Q7x27_kybd_record, kybd_centerkey);
	struct input_dev *idev		= kbdrec->Q7x27_kybd_idev;	
	
	bool center_val			= (bool)gpio_get_value(kbdrec->center_pin);
	//int HWID = FIH_READ_HWID_FROM_SMEM();
	suspend_state_t SuspendState = PM_SUSPEND_ON;
    
/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
	bool state;
#endif
/* } FIH, PeterKCTseng, @20090520 */
    SuspendState = get_suspend_state();
    /*if(!gpio_get_value(GPIO_NEG_Y))
    {

        bIsJBallExist = 1;  //F910
    }
    else
    {

        bIsJBallExist = 0;  //F902 
    }*/
	//printk(KERN_INFO "CENTER KEY <%d>\n", center_val);

	disable_irq(MSM_GPIO_TO_INT(kbdrec->center_pin));

/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
    state = (kbdrec->active.center_pin_actype == ACTIVE_HIGH) ? center_val : !center_val;
	//printk(KERN_INFO "active type= %d \n", state);
#endif
/* } FIH, PeterKCTseng, @20090520 */
    //if(gpio_get_value(GPIO_NEG_Y))
    if(EnableKeyInt)
    {
//+++++misty tmp mark    
//#if 0
        if(input_jogball_exist()==1)//F910/F911 jogball
        {
            if(idev)
             {
            	if (state) {
            		input_report_key(idev, SELECT_INPUT_ENTER, KBD_IN_KEYPRESS); //report SELECT_INPUT_ENTER pressing
            		printk(KERN_INFO "FIH: keypress SELECT_INPUT_ENTER= %d\n", SELECT_INPUT_ENTER);
            	} else {
            		input_report_key(idev, SELECT_INPUT_ENTER, KBD_IN_KEYRELEASE); //report SELECT_INPUT_ENTER releasing
            		printk(KERN_INFO "FIH: keyrelease SELECT_INPUT_ENTER= %d\n", SELECT_INPUT_ENTER);
            	}
            
            	input_sync(idev);
            }
            
        }
        else
        {
//#endif
    //----misty tmp mark    
//     if(1)//misty tmp add
//        {//misty tmp add
            if(idev)
            {
                if(SuspendState == PM_SUSPEND_MEM)
                {
                    if((g_HWID == CMCS_RTP_PR2))
                    {   
                       	input_report_key(idev, SELECT_INPUT_MENU, KBD_IN_KEYPRESS);
                        printk(KERN_INFO "FIH: keypress KEY_Center(MENU)\n");
                        input_report_key(idev, SELECT_INPUT_MENU, KBD_IN_KEYRELEASE);
                        printk(KERN_INFO "FIH: keyrelease KEY_Center(MENU)\n");
                        input_sync(idev);
                    }
                    else
                    {
                        input_report_key(idev, SELECT_INPUT_BTN, KBD_IN_KEYPRESS);
                        printk(KERN_INFO "FIH: keypress KEY_Center(HOME)\n");
                        input_report_key(idev, SELECT_INPUT_BTN, KBD_IN_KEYRELEASE);
                        printk(KERN_INFO "FIH: keyrelease KEY_Center(HOME)\n");
                        input_sync(idev);    
                    }
                    
                }
                else
                {
                    if((g_HWID == CMCS_RTP_PR2))
                    {
                    	if (state) {
                    		input_report_key(idev, SELECT_INPUT_MENU, KBD_IN_KEYPRESS); //report SELECT_INPUT_MENU pressing
                    		printk(KERN_INFO "FIH: keypress SELECT_INPUT_MENU= %d\n", SELECT_INPUT_MENU);
                    	} else {
                    		input_report_key(idev, SELECT_INPUT_MENU, KBD_IN_KEYRELEASE); //report SELECT_INPUT_MENU releasing
                    		printk(KERN_INFO "FIH: keyrelease SELECT_INPUT_MENU= %d\n", SELECT_INPUT_MENU);
                    	}
                    	input_sync(idev);
                    }
                    else
                    {
                        if (state) {
                		    input_report_key(idev, SELECT_INPUT_BTN, KBD_IN_KEYPRESS); //report SELECT_INPUT_BTN pressing
                		    printk(KERN_INFO "FIH: keypress SELECT_INPUT_BTN= %d\n", SELECT_INPUT_BTN);
                	    } else {
                		    input_report_key(idev, SELECT_INPUT_BTN, KBD_IN_KEYRELEASE); //report SELECT_INPUT_BTN releasing
                		    printk(KERN_INFO "FIH: keyrelease SELECT_INPUT_BTN= %d\n", SELECT_INPUT_BTN);
                	    }
                
                	    input_sync(idev);
                        
                        
                    }
                 }
             }//if(idev)
        }//else //if(input_jogball_exist()==1)
    	
}//if(EnableKeyInt)
    
	enable_irq(MSM_GPIO_TO_INT(kbdrec->center_pin));
}
#endif
/* } FIH, PeterKCTseng, @20090527 */



static void Q7x27_kybd_shutdown(struct Q7x27_kybd_record *rd)
{
	if (rd->kybd_connected) {
		printk(KERN_INFO "disconnecting keyboard\n");
		rd->kybd_connected = 0;

		free_irq(MSM_GPIO_TO_INT(rd->key_1_pin), rd);
		free_irq(MSM_GPIO_TO_INT(rd->key_2_pin), rd);

#if VOLUME_KEY_ENABLE // Peter, Debug
		free_irq(MSM_GPIO_TO_INT(rd->volup_pin), rd);
		free_irq(MSM_GPIO_TO_INT(rd->voldn_pin), rd);
#endif
		
#if CAMERA_KEY_ENABLE // Peter, Debug
		free_irq(MSM_GPIO_TO_INT(rd->cam_sw_t_pin), rd);
		free_irq(MSM_GPIO_TO_INT(rd->cam_sw_f_pin), rd);
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
		free_irq(MSM_GPIO_TO_INT(rd->hook_sw_pin), rd);
#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
		free_irq(MSM_GPIO_TO_INT(rd->center_pin), rd);
#endif
/* } FIH, PeterKCTseng, @20090527 */

		flush_work(&rd->kybd_generalkey1);
		flush_work(&rd->kybd_generalkey2);        

#if VOLUME_KEY_ENABLE // Peter, Debug
		flush_work(&rd->kybd_volkey1);
       	flush_work(&rd->kybd_volkey2);
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
		flush_work(&rd->kybd_camkey1);
		flush_work(&rd->kybd_camkey2);
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
		flush_work(&rd->hook_switchkey);
#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
		flush_work(&rd->kybd_centerkey);
#endif
/* } FIH, PeterKCTseng, @20090527 */

	}
}

static int Q7x27_kybd_opencb(struct input_dev *idev)
{
	struct Q7x27_kybd_record *kbdrec	= input_get_drvdata(idev);
		__set_bit(KEY_VOLUMEDOWN, idev->keybit);
		__set_bit(KEY_VOLUMEUP, idev->keybit);

	kbdrec->kybd_connected = 1;

	return 0;
}

static void Q7x27_kybd_closecb(struct input_dev *idev)
{

}

static struct input_dev *create_inputdev_instance(struct Q7x27_kybd_record *kbdrec)
{
	struct input_dev *idev	= NULL;

	idev = input_allocate_device();
	if (idev != NULL) {
		idev->name		= Q7x27_kybd_name;
		idev->open		= Q7x27_kybd_opencb;
		idev->close		= Q7x27_kybd_closecb;
		idev->keycode		= NULL;
		idev->keycodesize	= sizeof(uint8_t);
		idev->keycodemax	= 256;
/* FIH, PeterKCTseng, @20090603 { */
//		idev->evbit[0]		= BIT(EV_KEY);
        idev->evbit[0] = BIT(EV_KEY) | BIT(EV_REL);
        idev->relbit[0] = BIT(REL_X) | BIT(REL_Y);
/* } FIH, PeterKCTseng, @20090603 */

		/* a few more misc keys */
//		__set_bit(KEY_MENU, idev->keybit);
//		__set_bit(KEY_BACK, idev->keybit);
		__set_bit(KEY_SEND, idev->keybit);
		__set_bit(KEY_END, idev->keybit);
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
		__set_bit(KEY_F23, idev->keybit);
// } FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 
#if VOLUME_KEY_ENABLE // Peter, Debug
//		__set_bit(KEY_VOLUMEDOWN, idev->keybit);
//		__set_bit(KEY_VOLUMEUP, idev->keybit);
		__set_bit(KEY_VOLUMEDOWN, idev->keybit);
		__set_bit(KEY_VOLUMEUP, idev->keybit);
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
		__set_bit(KEY_CAMERA, idev->keybit);
		__set_bit(KEY_FOCUS, idev->keybit);
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
		__set_bit(KEY_HEADSETHOOK, idev->keybit);
		__set_bit(KEY_RINGSWITCH, idev->keybit); //FIH, KarenLiao, @20090731: [F0X.FC-41]: The action of Inserting headset is the wake up action.
#endif


/* FIH, PeterKCTseng, @20090526 { */
/* power key support              */
		__set_bit(KEY_POWER, idev->keybit);
/* } FIH, PeterKCTseng, @20090526 */

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
		__set_bit(SELECT_INPUT_BTN, idev->keybit);
		__set_bit(SELECT_INPUT_MENU, idev->keybit);
		__set_bit(SELECT_INPUT_ENTER, idev->keybit);
#endif
/* } FIH, PeterKCTseng, @20090527 */

		input_set_drvdata(idev, kbdrec);
		kpdev = idev;
	} else {
		//printk(KERN_ERR "Failed to allocate input device for %s\n",
		//	Q7x27_kybd_name);
	}
	
	return idev;
}

static void Q7x27_kybd_connect2inputsys(struct work_struct *work)
{
	struct Q7x27_kybd_record *kbdrec =
		container_of(work, struct Q7x27_kybd_record, kb_cmdq.work);	

	kbdrec->Q7x27_kybd_idev = create_inputdev_instance(kbdrec);
	if (kbdrec->Q7x27_kybd_idev) {
		if (input_register_device(kbdrec->Q7x27_kybd_idev) != 0) {
			//printk(KERN_ERR "Failed to register with"
			//	" input system\n");
			input_free_device(kbdrec->Q7x27_kybd_idev);
		}
	}
}

static int testfor_keybd(void)
{
	int rc = 0;

	INIT_DELAYED_WORK(&rd->kb_cmdq, Q7x27_kybd_connect2inputsys);
	schedule_delayed_work(&rd->kb_cmdq, msecs_to_jiffies(600));

	return rc;
}
/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change keypad suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
void Q7x27_kybd_early_suspend(struct early_suspend *h)
{
    //printk(KERN_INFO "Q7x27_kybd_early_suspend()(%d)\n",rd->kybd_connected);

 if(SetupKeyFail)
 {
     SetupKeyFail=false;
     KeySetup();
    
 }
 //printk(KERN_ERR "%s""@@@g_center_pin:%d \n", __func__,g_center_pin);
 if(b_EnableWakeKey)
 {
	 //Set this if you want use IRQs to wake the system up
	 if(device_may_wakeup(&rd->pdev->dev)) 
     {
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
       //if(g_SndEndkey)
       if(g_Send && g_End)
// } FIH, NicoleWeng, 2010/05/26 for F0XE.B-955
       {
           //printk(KERN_INFO "enable key1   wakeup pin: %d\n", rd->key_1_pin);
    	   enable_irq_wake(MSM_GPIO_TO_INT(rd->key_1_pin));
           b_Key1_EnableWakeIrq = true;
    
           //printk(KERN_INFO "enable key2   wakeup pin: %d\n", rd->key_2_pin);  
           enable_irq_wake(MSM_GPIO_TO_INT(rd->key_2_pin));
           b_Key2_EnableWakeIrq = true;
       }
       if(g_centerkey)
       {
           //printk(KERN_INFO "diable center interrupt pin: %d\n", g_center_pin);
           disable_irq(MSM_GPIO_TO_INT(g_center_pin));
           b_Center_DisableIrq = true;
       }
	 }
 }
 else
 {
    if(b_EnableIncomingCallWakeKey)
    {
	  if(device_may_wakeup(&rd->pdev->dev)) 
      {
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
       //if(g_SndEndkey)
       if(g_Send && g_End)
// } FIH, NicoleWeng, 2010/05/26 for F0XE.B-955
        {
            //printk(KERN_INFO "enable key1   wakeup pin: %d\n", rd->key_1_pin);
	        enable_irq_wake(MSM_GPIO_TO_INT(rd->key_1_pin));
            b_Key1_EnableWakeIrq = true;

            //printk(KERN_INFO "enable key2   wakeup pin: %d\n", rd->key_2_pin); 
	        enable_irq_wake(MSM_GPIO_TO_INT(rd->key_2_pin));
            b_Key2_EnableWakeIrq = true;
        }
        if(g_centerkey)
        {
            //printk(KERN_INFO "diable center interrupt pin: %d\n", g_center_pin);
            disable_irq(MSM_GPIO_TO_INT(g_center_pin));
            b_Center_DisableIrq = true;
        }
	    //printk(KERN_INFO "enable VolUp   wakeup pin: %d\n", rd->volup_pin);
	    enable_irq_wake(MSM_GPIO_TO_INT(rd->volup_pin));
        b_VolUp_EnableWakeIrq = true;

	    //printk(KERN_INFO "enable VolDown   wakeup pin: %d\n", rd->voldn_pin); 
	    enable_irq_wake(MSM_GPIO_TO_INT(rd->voldn_pin));
        b_VolDown_EnableWakeIrq = true;
      }
    }
    else
    {
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
       //if(g_SndEndkey)
      if(g_Send && g_End)
// } FIH, NicoleWeng, 2010/05/26 for F0XE.B-955
      {
          //printk(KERN_INFO "diable key1 interrupt   pin: %d\n", rd->key_1_pin);
          disable_irq(MSM_GPIO_TO_INT(rd->key_1_pin));
          b_Key1_DisableIrq = true;
         // printk(KERN_INFO "diable key2 interrupt   pin: %d\n", rd->key_2_pin);
          disable_irq(MSM_GPIO_TO_INT(rd->key_2_pin));  
          b_Key2_DisableIrq = true;  
      }    
      if(g_centerkey)
      {
         //printk(KERN_INFO "diable center interrupt pin: %d\n", g_center_pin);
         disable_irq(MSM_GPIO_TO_INT(g_center_pin));       
         b_Center_DisableIrq = true;      
      }
    }
 }
 
 //+++ FIH, KarenLiao@20100304: F0X.B-9873: [Call control]Cannot end the call when long press hook key.
 if((b_EnableIncomingCallWakeKey ==true) && (rd->bHookSWIRQEnabled == true) && device_may_wakeup(&rd->pdev->dev) )
 {
     printk(KERN_INFO "enable Hook Key   wakeup pin: %d\n", rd->hook_sw_pin);
     enable_irq_wake(MSM_GPIO_TO_INT(rd->hook_sw_pin));
     b_HookKey_EnableWakeIrq = true;
 }
 //--- FIH, KarenLiao@20100304: F0X.B-9873: [Call control]Cannot end the call when long press hook key.
 //FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
 //+++add for FST back cover key
 if(g_Send && !g_End)
 {
 	enable_irq_wake(MSM_GPIO_TO_INT(rd->key_1_pin));
 	printk(KERN_INFO "enable FST SEND key(F23) as  wakeup pin: %d\n", rd->key_1_pin);
 }
 //--add for FST back cover key
 //FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
 
 
}
void Q7x27_kybd_late_resume(struct early_suspend *h)
{
 //printk(KERN_INFO "Q7x27_kybd_late_resume()(%d)\n",rd->kybd_connected);
// printk(KERN_ERR "%s""#######################g_center_pin:%d \n", __func__,g_center_pin);
  if(SetupKeyFail)
 {
     SetupKeyFail=false;
     KeySetup();
 }
 if (device_may_wakeup(&rd->pdev->dev))
 {
   //printk(KERN_INFO "b_Key1_EnableWakeIrq = %d\n", b_Key1_EnableWakeIrq);
   if(b_Key1_EnableWakeIrq)
   {
     //printk(KERN_INFO "disable key1   wakeup pin: %d\n", rd->key_1_pin);
     disable_irq_wake(MSM_GPIO_TO_INT(rd->key_1_pin));
     b_Key1_EnableWakeIrq = false;
   }

   //printk(KERN_INFO "b_Key2_EnableWakeIrq = %d\n", b_Key2_EnableWakeIrq);
   if(b_Key2_EnableWakeIrq)
   {
     //printk(KERN_INFO "disable key2   wakeup pin: %d\n", rd->key_2_pin);
	 disable_irq_wake(MSM_GPIO_TO_INT(rd->key_2_pin));
     b_Key2_EnableWakeIrq = false;     
   }

   if(b_VolUp_EnableWakeIrq)
   {
     //printk(KERN_INFO "disable VolUp   wakeup pin: %d\n", rd->volup_pin);
	 disable_irq_wake(MSM_GPIO_TO_INT(rd->volup_pin));
     b_VolUp_EnableWakeIrq = false;     
   }

   if(b_VolDown_EnableWakeIrq)
   {
     //printk(KERN_INFO "disable VolDown   wakeup pin: %d\n", rd->voldn_pin);
	 disable_irq_wake(MSM_GPIO_TO_INT(rd->voldn_pin));
     b_VolDown_EnableWakeIrq = false;     
   }
   
   //+++ FIH, KarenLiao@20100304: F0X.B-9873: [Call control]Cannot end the call when long press hook key.
   if(b_HookKey_EnableWakeIrq == true)
   {
     printk(KERN_INFO "disable HookKey   wakeup pin: %d\n", rd->hook_sw_pin);
	 disable_irq_wake(MSM_GPIO_TO_INT(rd->hook_sw_pin));
     b_HookKey_EnableWakeIrq = false;
   }
   //--- FIH, KarenLiao@20100304: F0X.B-9873: [Call control]Cannot end the call when long press hook key.
   
 }

 //printk(KERN_INFO "b_Key1_DisableIrq = %d\n", b_Key1_DisableIrq);
 if(b_Key1_DisableIrq)
 {
   //printk(KERN_INFO "enable key1 interrupt   pin: %d\n", rd->key_1_pin);
   enable_irq(MSM_GPIO_TO_INT(rd->key_1_pin)); 
   b_Key1_DisableIrq = false; 
 }

 //printk(KERN_INFO "b_Key2_DisableIrq = %d\n", b_Key2_DisableIrq);
 if(b_Key2_DisableIrq)
 {
   //printk(KERN_INFO "enable key2 interrupt   pin: %d\n", rd->key_2_pin);
   enable_irq(MSM_GPIO_TO_INT(rd->key_2_pin));  
   b_Key2_DisableIrq = false; 
 }

 //printk(KERN_INFO "b_Center_DisableIrq = %d\n", b_Center_DisableIrq); 
 if(b_Center_DisableIrq)
 {
   //printk(KERN_INFO "enable center interrupt pin: %d\n", g_center_pin);
   enable_irq(MSM_GPIO_TO_INT(g_center_pin));
   b_Center_DisableIrq = false; 
 }
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
 if(g_Send && !g_End)
 {
	disable_irq_wake(MSM_GPIO_TO_INT(rd->key_1_pin));
 	 printk(KERN_INFO "disable SEND key(F23)  wakeup pin: %d\n", rd->key_1_pin);
 }
// } FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 
}
#endif
#endif
/* } FIH, SimonSSChang, 2009/09/04 */

static int Q7x27_kybd_remove(struct platform_device *pdev)
{
	//printk(KERN_INFO "removing keyboard driver\n");

/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change keypad suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
    //printk(KERN_INFO "Keypad unregister_early_suspend()\n");
	unregister_early_suspend(&rd->Q7x27_kybd_early_suspend_desc);
#endif
#endif
/* } FIH, SimonSSChang, 2009/09/04 */

	if (rd->Q7x27_kybd_idev) {
		//printk(KERN_INFO "deregister from input system\n");
		input_unregister_device(rd->Q7x27_kybd_idev);
		rd->Q7x27_kybd_idev = NULL;
	}
	Q7x27_kybd_shutdown(rd);
	Q7x27_kybd_release_gpio(rd);

	kfree(rd);

	return 0;
}

//++++++++++++++++++++++++++++FIH_F0X_misty
/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change keypad suspend/resume function
to earlysuspend*/
//#ifdef CONFIG_PM
#ifdef CONFIG_PM__
/* } FIH, SimonSSChang, 2009/09/04 */
static int Q7x27_kybd_suspend(struct platform_device *pdev, pm_message_t state)
{
 struct Q7x27_kybd_platform_data *kbdrec;
 kbdrec       = pdev->dev.platform_data;
 
 printk(KERN_ERR "%s""@@@@@@@@@@@@@@@@@@@@@@@@g_center_pin:%d \n", __func__,g_center_pin);
 if(b_EnableWakeKey)
 {
	 //Set this if you want use IRQs to wake the system up
	 if (device_may_wakeup(&pdev->dev)) {
	  printk(KERN_ERR "%s++++++"" \n", __func__);
	  enable_irq_wake(MSM_GPIO_TO_INT(kbdrec->key_1_pin));
	  enable_irq_wake(MSM_GPIO_TO_INT(kbdrec->key_2_pin));
	  enable_irq_wake(MSM_GPIO_TO_INT(g_center_pin));
	  printk(KERN_ERR "%s--------"" \n", __func__);
	 }
 }
 return 0;
}
static int Q7x27_kybd_resume(struct platform_device *pdev)
{
 struct Q7x27_kybd_platform_data *kbdrec;
 kbdrec       = pdev->dev.platform_data;
 
 printk(KERN_ERR "%s""#######################g_center_pin:%d \n", __func__,g_center_pin);
 if(b_EnableWakeKey)
 {
	 if (device_may_wakeup(&pdev->dev)) {
	  printk(KERN_ERR "%s++++++disable irq_wake"" \n", __func__);
	  disable_irq_wake(MSM_GPIO_TO_INT(kbdrec->key_1_pin));
	  disable_irq_wake(MSM_GPIO_TO_INT(kbdrec->key_2_pin));
	  disable_irq_wake(MSM_GPIO_TO_INT(g_center_pin));
	  printk(KERN_ERR "%s--------disable irq_wake"" \n", __func__);
	  
	 }
 }
 return 0;
}
#else
# define Q7x27_kybd_suspend NULL
# define Q7x27_kybd_resume  NULL
#endif
//---------------------------FIH_F0X_misty
static int Q7x27_kybd_probe(struct platform_device *pdev)
{
	struct Q7x27_kybd_platform_data *setup_data;
	int rc = -ENOMEM;

/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
    g_HWID = FIH_READ_HWID_FROM_SMEM();
    g_ORIGHWID = FIH_READ_ORIG_HWID_FROM_SMEM();
#endif
/* } FIH, PeterKCTseng, @20090520 */

    //FIH_debug_log
    //printk(KERN_INFO "FIH: enter Q7x27_kybd_probe()\n");
	rd = kzalloc(sizeof(struct Q7x27_kybd_record), GFP_KERNEL);
	if (!rd) {
		//printk(KERN_ERR "i2ckybd_record memory allocation failed!!\n");
		return rc;
	}
//FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {
	if(sysfs_create_group(&pdev->dev.kobj, &dev_attr_grp)) {
		printk(KERN_INFO "[Keypad]Create attributes in sysfs fail\n");
	}
// } FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 
	setup_data 		    = pdev->dev.platform_data;
	//+++++++++++++++++++++++++++FIH_F0X_misty
	//printk(KERN_ERR "++++++++++++++++++++++++add init_wakeup\n");
	device_init_wakeup(&pdev->dev, 1);
	//printk(KERN_ERR "---------------------add init_wakeup\n");
	//------------------------------FIH_F0X_misty
/* FIH, PeterKCTseng, @20090520 { */
/* The active type of input pin   */
#if ACTIVE_MODE_ENABLE // Peter, Debug
    
    if (g_HWID  == CMCS_HW_VER_EVB1) {
		//printk(KERN_ERR "FIH: CMCS_HW_VER_EVB1, g_HWID= %d \n", g_HWID);
        rd->active.volup_pin_actype     = ACTIVE_LOW;
        rd->active.voldn_pin_actype     = ACTIVE_LOW;
        rd->active.key_1_pin_actype     = ACTIVE_LOW;
        rd->active.key_2_pin_actype     = ACTIVE_LOW;
        rd->active.cam_sw_t_pin_actype  = ACTIVE_LOW;
        rd->active.cam_sw_f_pin_actype  = ACTIVE_LOW;
        rd->active.hook_sw_pin_actype   = ACTIVE_LOW;
    
/* FIH, PeterKCTseng, @20090601 { */
        rd->active.center_pin_actype    = ACTIVE_LOW;
/* } FIH, PeterKCTseng, @20090601 */
    } else if (g_HWID >= CMCS_RTP_PR1) {
		//printk(KERN_ERR "FIH: CMCS_RTP_PR1, g_HWID= %d \n", g_HWID);
        rd->active.volup_pin_actype     = ACTIVE_HIGH;
        rd->active.voldn_pin_actype     = ACTIVE_HIGH;
        rd->active.key_1_pin_actype     = ACTIVE_HIGH;
        rd->active.key_2_pin_actype     = ACTIVE_HIGH;
        rd->active.cam_sw_t_pin_actype  = ACTIVE_HIGH;
        rd->active.cam_sw_f_pin_actype  = ACTIVE_HIGH;
/* FIH, PeterKCTseng, @20090527 { */
//      rd->active.hook_sw_pin_actype   = ACTIVE_HIGH;
        rd->active.hook_sw_pin_actype   = ACTIVE_LOW;
/* } FIH, PeterKCTseng, @20090527 */

/* FIH, PeterKCTseng, @20090601 { */
         rd->active.center_pin_actype    = ACTIVE_HIGH;
/* } FIH, PeterKCTseng, @20090601 */
    }          
    else {
        // target board undefined 
		//printk(KERN_ERR "target borad can not be recognized!! \n");

    }  
#endif
/* } FIH, PeterKCTseng, @20090520 */

	rd->key_1_pin		= setup_data->key_1_pin;
	rd->key_2_pin		= setup_data->key_2_pin;

#if VOLUME_KEY_ENABLE // Peter, Debug
	rd->volup_pin		= setup_data->volup_pin;
	rd->voldn_pin		= setup_data->voldn_pin;
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
	rd->cam_sw_f_pin	= setup_data->cam_sw_f_pin;
	rd->cam_sw_t_pin	= setup_data->cam_sw_t_pin;
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
	rd->hook_sw_pin     = setup_data->hook_sw_pin;

/* FIH, PeterKCTseng, @20090603 { */
	rd->bHookSWIRQEnabled = false; //FIH, KarenLiao, @20100520: F0XE.B-850: Set initial value to false to fix calling enable_irq_wake() without headset.
/* } FIH, PeterKCTseng, @20090603 */

	/* FIH, Karen Liao, 2010/05/05 { */
	/* F0XE.B-755: [Audio] Modify headset hook detect. */
	headset_hook_wq = create_singlethread_workqueue("headset_hook");
	
	if (!headset_hook_wq) {
		return -EBUSY;
	}
	/* } FIH, Karen Liao, 2010/05/05 */
	
#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
//	rd->center_pin     = setup_data->center_pin;
    if (g_HWID  == CMCS_HW_VER_EVB1) {
    	rd->center_pin     = 92;
    	g_center_pin = 92;
    }
    else if (g_HWID >= CMCS_RTP_PR1) {
    	rd->center_pin     = 124;
    	g_center_pin =124;
    }
    //printk(KERN_ERR "g_ORIGHWID:0x%x\n",g_ORIGHWID);
	if((g_HWID >= CMCS_CTP_PR1)&&(g_HWID <= CMCS_CTP_MP3))
	{
	    //doesn't inlcude FST/F917/FA9/FAA
		if(!(g_ORIGHWID >= CMCS_125_FST_PR1 &&  g_ORIGHWID <= CMCS_128_FST_MP1) 
		  && !(g_ORIGHWID >= CMCS_CTP_F917_PR1 &&  g_ORIGHWID <= CMCS_CTP_F917_MP3) 
		  && !(g_ORIGHWID >= CMCS_125_FA9_PR1 &&  g_ORIGHWID <= CMCS_125_FA9_MP1) 
		  && !(g_ORIGHWID >= CMCS_125_4G4G_FAA_PR1 &&  g_ORIGHWID <= CMCS_125_4G4G_FAA_MP1)
		  && !(g_ORIGHWID >= CMCS_128_4G4G_FAA_PR1 &&  g_ORIGHWID <= CMCS_128_4G4G_FAA_MP1))
	        {

			//if(input_jogball_exist()==0)//F903,center key is floating.to disable the key IST
			if(gpio_get_value(GPIO_NEG_Y))
			{
				g_centerkey=false;
				//printk(KERN_ERR "OFF center key\n");
			}
		}
	}
	////for FST,disable send/end key to avoid ending call automatically when go into suspend.
	//Add FA9 case which comes from FST
	if((g_ORIGHWID >= CMCS_125_FST_PR1 &&  g_ORIGHWID <= CMCS_128_FST_MP1)
	|| (g_ORIGHWID >= CMCS_125_FA9_PR1 &&  g_ORIGHWID <= CMCS_125_FA9_MP1))
	{
//  FIH, NicoleWeng, 2010/05/26 for F0XE.B-955 {          
           if(g_ORIGHWID == CMCS_125_FST_MP1)
           {
           	g_Send=true;
	    		g_End=false;
				printk(KERN_ERR "OFF END key\n");
           }
	    	else
	    	{
				g_Send=false;
				g_End=false;
			    printk(KERN_ERR "OFF SND/END key\n");
	    	}
	   	 //g_SndEndkey=false;
	   // printk(KERN_ERR "OFF SND/END key\n");
// } FIH, NicoleWeng, 2010/05/26 for F0XE.B-955     
	}
	
	gpio_tlmm_config(GPIO_CFG(rd->center_pin, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#endif
/* } FIH, PeterKCTseng, @20090527 */

	//Initialize GPIO
	rc = Q7x27_kybd_config_gpio(rd);
	if (rc)
		goto failexit1;

	//printk(KERN_ERR "[+++++++++++++++++keypad++++++++++++++] g_HWID  == %d\n",g_HWID);
	
	//if(g_HWID  == CMCS_7627_EVB1 || g_HWID  == CMCS_7627_PR1)
	if(g_HWID  >= CMCS_7627_EVB1)
	{
		//printk(KERN_ERR "[+++++++++++++++++keypad++++++++++++++] change send and end key\n");
		//Initialize IRQ
		INIT_WORK(&rd->kybd_generalkey1, Q7627_kybd_generalkey1);
    	INIT_WORK(&rd->kybd_generalkey2, Q7627_kybd_generalkey2);
		
	}
	else
	{
		
		//Initialize IRQ
		INIT_WORK(&rd->kybd_generalkey1, Q7x27_kybd_generalkey1);
    	INIT_WORK(&rd->kybd_generalkey2, Q7x27_kybd_generalkey2);
	}

#if VOLUME_KEY_ENABLE // Peter, Debug
	INIT_WORK(&rd->kybd_volkey1, Q7x27_kybd_volkey1);
    INIT_WORK(&rd->kybd_volkey2, Q7x27_kybd_volkey2);
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
	INIT_WORK(&rd->kybd_camkey1, Q7x27_kybd_camkey1);
	INIT_WORK(&rd->kybd_camkey2, Q7x27_kybd_camkey2);
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
	INIT_WORK(&rd->hook_switchkey, Q7x27_hook_switchkey);


//  msm_mic_en_proc(true);

#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
	
	INIT_WORK(&rd->kybd_centerkey, Q7x27_kybd_centerkey);
#endif
/* } FIH, PeterKCTseng, @20090527 */
    KeySetup();

    /* FIH, Debbie, 2010/01/05 { */
    /* modify for key definition of OTA update*/
    if(fih_read_kpd_from_smem())
    {
    	   EnableKeyInt = 1;
    }
    /* FIH, Debbie, 2010/01/05 } */
#if 0//misty +++
    printk(KERN_INFO "FIH: enter 7\n");    
	rc = Q7x27_kybd_irqsetup(rd);
    printk(KERN_INFO "FIH: enter 8\n");        
	if (rc)
		goto failexit2;

	rc = testfor_keybd();
    printk(KERN_INFO "FIH: enter 9\n");    
	if (rc)
		goto failexit2;
	
	rd->kybd_connected = 1;
#endif//misty ---
/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change keypad suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
    rd->Q7x27_kybd_early_suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 10;
	rd->Q7x27_kybd_early_suspend_desc.suspend = Q7x27_kybd_early_suspend;
	rd->Q7x27_kybd_early_suspend_desc.resume = Q7x27_kybd_late_resume;
    //printk(KERN_INFO "Keypad register_early_suspend()\n");
	register_early_suspend(&rd->Q7x27_kybd_early_suspend_desc);
    rd->pdev = pdev;
#endif
#endif
/* } FIH, SimonSSChang, 2009/09/04 */

    //FIH_debug_log
    //printk(KERN_INFO "FIH: out Q7x27_kybd_probe()\n");
    return 0;
#if 0//misty+++
 failexit2:
    //FIH_debug_log#if CAMERA_KEY_ENABLE // Peter, Debug

    printk(KERN_INFO "FIH: error out failexit2\n");
    
	free_irq(MSM_GPIO_TO_INT(rd->key_1_pin), rd);
	free_irq(MSM_GPIO_TO_INT(rd->key_2_pin), rd);

#if VOLUME_KEY_ENABLE // Peter, Debug
	free_irq(MSM_GPIO_TO_INT(rd->volup_pin), rd);
	free_irq(MSM_GPIO_TO_INT(rd->voldn_pin), rd);
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
	free_irq(MSM_GPIO_TO_INT(rd->cam_sw_t_pin), rd);
	free_irq(MSM_GPIO_TO_INT(rd->cam_sw_f_pin), rd);
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
	free_irq(MSM_GPIO_TO_INT(rd->hook_sw_pin), rd);
#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
	free_irq(MSM_GPIO_TO_INT(rd->center_pin), rd);
#endif
/* } FIH, PeterKCTseng, @20090527 */

#endif//misty----
 failexit1:
    //FIH_debug_log
    //printk(KERN_INFO "FIH: error out failexit1\n");
	Q7x27_kybd_release_gpio(rd);
	kfree(rd);

    //FIH_debug_log
    //printk(KERN_INFO "FIH: error out Q7x27_kybd_probe()\n");
    return 0;
    
	return rc;
}
// FIH, WillChen, 2009/08/21 ++
//Dump cpu and mem info when three key panic
#ifdef CONFIG_FIH_FXX_FORCEPANIC
static int panic_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	printk("device_panic_read_proc\n");
	wait_event(wq,flag != 0);

	return 0;
}
#endif
// FIH, WillChen, 2009/08/21 --
static struct platform_driver Q7x27_kybd_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = Q7x27_kybd_name,
	},
	.probe	  = Q7x27_kybd_probe,
	.remove	  = Q7x27_kybd_remove,
	.suspend  = Q7x27_kybd_suspend,
	.resume   = Q7x27_kybd_resume,
};

static int __init Q7x27_kybd_init(void)
{
// FIH, WillChen, 2009/08/21 ++
//Dump cpu and mem info when three key panic
#ifdef CONFIG_FIH_FXX_FORCEPANIC
	create_proc_read_entry("panicdump", 0, NULL, panic_read_proc, NULL);
#endif
// FIH, WillChen, 2009/08/21 --
	return platform_driver_register(&Q7x27_kybd_driver);
}

static void __exit Q7x27_kybd_exit(void)
{
	platform_driver_unregister(&Q7x27_kybd_driver);
}
//+++FIH_misty enable keypad interrupt until boot complete
void KeySetup(void)
{
	 int rc = -ENOMEM;
     int count=0;
     int count1=0;
retry1:
        rc = Q7x27_kybd_irqsetup(rd);
        //printk(KERN_INFO "KeySetup/Q7x27_kybd_irqsetup\n");   
        if (rc)
        {
            goto retry1;
            count++;    
            if(count > 6)
            {
                //printk(KERN_INFO "retry FAIL======>Q7x27_kybd_irqsetup\n"); 
                count=0;
	            SetupKeyFail=true;
                goto failexit2;
            }
        }
retry2: 
        rc = testfor_keybd();
        //printk(KERN_INFO "KeySetup/testfor_keybd\n");  
        if (rc)
        {
            goto retry2;
            count1++;
            if(count1 > 6)
            {
                count1=0;
                //printk(KERN_INFO "retry FAIL======>testfor_keybd\n");  
		        SetupKeyFail=true;
                goto failexit2;
            }
        }
        rd->kybd_connected = 1;
	 return;
failexit2:
    //printk(KERN_INFO "FIH: error out failexit2\n");
	free_irq(MSM_GPIO_TO_INT(rd->key_1_pin), rd);
	free_irq(MSM_GPIO_TO_INT(rd->key_2_pin), rd);

#if VOLUME_KEY_ENABLE // Peter, Debug
	free_irq(MSM_GPIO_TO_INT(rd->volup_pin), rd);
	free_irq(MSM_GPIO_TO_INT(rd->voldn_pin), rd);
#endif

#if CAMERA_KEY_ENABLE // Peter, Debug
	free_irq(MSM_GPIO_TO_INT(rd->cam_sw_t_pin), rd);
	free_irq(MSM_GPIO_TO_INT(rd->cam_sw_f_pin), rd);
#endif

#if SWITCH_KEY_ENABLE // Peter, Debug
	free_irq(MSM_GPIO_TO_INT(rd->hook_sw_pin), rd);
#endif

/* FIH, PeterKCTseng, @20090527 { */
/* add center key                 */
#if CENTER_KEY_ENABLE // Peter, Debug
	free_irq(MSM_GPIO_TO_INT(rd->center_pin), rd);
#endif
/* } FIH, PeterKCTseng, @20090527 */
    return ;


}
static int Q7x27_kybd_param_set(const char *val, struct kernel_param *kp)
{
    int ret=1;
    if(!EnableKeyInt)
    {
        ret = param_set_bool(val, kp);
        //printk(KERN_ERR "%s: EnableKeyInt= %d\n", __func__, EnableKeyInt); 
        
        if(ret)
        {
            //printk(KERN_ERR "%s param set bool failed (%d)\n",
            //			__func__, ret);    
    	    EnableKeyInt = 1;
    	}
	/* FIH, Debbie, 2010/01/05 { */
	/* modify for key definition of OTA update*/
	else
	{
           if(fih_read_kpd_from_smem())
    	    {
    	        EnableKeyInt = 1;
    	    }
	}
	/* FIH, Debbie, 2010/01/05 } */
    	return 0;
    }
    else
    {
        //printk(KERN_ERR "has alreay set EnableKeyInt\n"); 
        return 0;    
    }

}
module_param_call(EnableKeyIntrrupt, Q7x27_kybd_param_set, param_get_int,
		  &EnableKeyInt, S_IWUSR | S_IRUGO);
//---FIH_misty enable keypad interrupt until boot complete
module_init(Q7x27_kybd_init);
module_exit(Q7x27_kybd_exit);
MODULE_VERSION("1.0");
MODULE_DESCRIPTION("Q7x27 keyboard driver");
MODULE_LICENSE("GPL v2");
