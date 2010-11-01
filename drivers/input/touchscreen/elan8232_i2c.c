#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#include <linux/elan_i2c.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
//Dynamic to load RES or CAP touch driver++
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>
//Dynamic to load RES or CAP touch driver--
#include <mach/vreg.h>  //Add for VREG_WLAN power in, 07/08
#include <mach/7x27_kybd.h>  //Added for FST by Stanley (F0X_2.B-414)
/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change CAP touch suspend/resume function
to earlysuspend */
#include <linux/earlysuspend.h>
/* } FIH, SimonSSChang, 2009/09/04 */
//Added by Stanley for dump scheme++
#include <linux/unistd.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
//Added by Stanley for dump scheme--


/************************************************
 *  attribute marco
 ************************************************/
#define TOUCH_NAME		"bi8232"	/* device name */
#define BI8232_DEBUG	1			/* print out receive packet */
#define KEYCODE_BROWSER 192  //Modified for Home and AP key (2009/07/31)
#define KEYCODE_SEACHER 217  //Modified for Home and AP key (2009/07/31)

/************************************************
 *  function marco
 ************************************************/
#define bi8232_msg(lv, fmt, arg...)	\
	printk(KERN_ ##lv"[%s@%s@%d] "fmt"\n",__FILE__,__func__,__LINE__,##arg)

static struct proc_dir_entry *msm_touch_proc_file = NULL;  //Added by Stanley for dump scheme++
/************************************************
 *  Global variable
 ************************************************/
u8 buffer[9];
bool bSoft1CapKeyPressed = 0;  //Modified for new CAP sample by Stanley (2009/05/25)	
bool bSoft2CapKeyPressed = 0;  //Modified for new CAP sample by Stanley (2009/05/25)
bool bHomeCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
bool bApCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
bool bCenterKeyPressed = 0;  //Added for FST
bool bIsF913 = 0;  //Modified for Home and AP key (2009/07/31)
bool bIsFST = 0;  //Added for FST
bool bIsGRECO = 0;  //Added for GRECO
bool bIsFxxPR2 = 0;  //Added for PR2
bool bIsPenUp = 1;  //Added for touch behavior (2009/08/14)
bool bIsKeyLock = 0;  //Added for new behavior (2009/09/27)
bool bIsNewFWVer = 0;  //Add for detect firmware version
bool bPrintPenDown = 0;  //Added log for debugging
/* FIH, Henry Juang, 2009/11/20 ++*/
/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
bool bIsNeedSkipTouchEvent = 0;  
/* FIH, Henry Juang, 2009/11/20 --*/
bool bPhoneCallState = 0;  //Added for FST by Stanley (F0X_2.B-414)
bool bIsNeedtoEnableIRQ = 0;  //Added to fix IRQ sync issue for F0X_2.B-414
bool bIsFSTDisableWLAN = 0;  //Added for VREG sync by Stanley
short vreg_refcnt = 0;  //Added for VREG sync by Stanley

//Added for debug mask definitions++
/************************************************
 * Debug Definitions
 ************************************************/
enum {
	MSM_PM_DEBUG_COORDINATES = 1U << 0,
};

//
static int msm_cap_touch_gpio_pull_fail;
module_param_named(
    gpio_pull, msm_cap_touch_gpio_pull_fail, int, S_IRUGO | S_IWUSR | S_IWGRP
);
//

//Added for show FW version on FQC++
static int msm_cap_touch_fw_version;
module_param_named(
    fw_version, msm_cap_touch_fw_version, int, S_IRUGO | S_IWUSR | S_IWGRP
);
//Added for show FW version on FQC--
//Added for debug mask definitions--

/************************************************
 * Control structure
 ************************************************/
struct bi8232_m32emau {
	struct i2c_client *client;
	struct input_dev  *input;
	struct work_struct wqueue;
	struct completion data_ready;
	struct completion data_complete;
/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change CAP touch suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend bi8232_early_suspend_desc;
#endif
#endif
/* } FIH, SimonSSChang, 2009/09/04 */    
} *bi8232;

static int bi8232_recv(u8 *buf, u32 count)
{
	struct i2c_adapter *adap = bi8232->client->adapter;
    struct i2c_msg msgs[]= {
		[0] = {
			.addr = bi8232->client->addr,
            .flags = I2C_M_RD,
            .len = count,
            .buf = buf,
		},
	};

	return (i2c_transfer(adap, msgs, 1) < 0) ? -EIO : 0;
}

static int bi8232_send(u8 *buf, u32 count)
{
	struct i2c_adapter *adap = bi8232->client->adapter;
	struct i2c_msg msgs[]= {
		[0] = {
            .addr = bi8232->client->addr,
			.flags = 0,
			.len = count,
			.buf = buf,
        },
    };

	return (i2c_transfer(adap, msgs, 1) < 0) ? -EIO : 0;
}

#define FW_MAJOR(x) ((((int)((x)[1])) & 0xF) << 4) + ((((int)(x)[2]) & 0xF0) >> 4)
#define FW_MINOR(x) ((((int)((x)[2])) & 0xF) << 4) + ((((int)(x)[3]) & 0xF0) >> 4)
static int bi8232_get_fw_version(int *version)
{
	int major, minor;
	char cmd[4] = { 0x53, 0x00, 0x00, 0x01 };

	bi8232_send(cmd, ARRAY_SIZE(cmd));
	wait_for_completion(&bi8232->data_ready);

	if (buffer[0] != 0x52) {
		complete(&bi8232->data_complete);
		return -1;
	}

	major = FW_MAJOR(buffer);
	minor = FW_MINOR(buffer);
	*version = major * 100 + minor;
	complete(&bi8232->data_complete);

	return 0;
}

static int bi8232_get_pw_state(int *state)
{
	char cmd[4] = { 0x53, 0x50, 0x00, 0x01 };

	bi8232_send(cmd, ARRAY_SIZE(cmd));
	wait_for_completion(&bi8232->data_ready);

	if (buffer[0] != 0x52) {
        complete(&bi8232->data_complete);
        return -1;
    }

	*state = (buffer[1] & 0x8) >> 3;
	complete(&bi8232->data_complete);

    return 0;
}

static int bi8232_set_pw_state(unsigned state)
{
	char cmd[4] = { 0x54, 0x50, 0x00, 0x01 };

	state = state ? 1 : 0;
	cmd[1] |= ((char)state << 3);

	if (bi8232_send(cmd, ARRAY_SIZE(cmd)) < 0) {
        bi8232_msg(ERR, "set power state failed");
        return -1;
    }

	return 0;
}

static int bi8232_get_orientation(int *mode)
{
	char cmd[4] = { 0x53, 0xB1, 0x00, 0x01 };

	bi8232_send(cmd, ARRAY_SIZE(cmd));
	wait_for_completion(&bi8232->data_ready);

	if (buffer[0] != 0x52) {
        complete(&bi8232->data_complete);
        return -1;
    }

	*mode = buffer[2] & 0x03;
	complete(&bi8232->data_complete);

	return 0;
}

static int bi8232_set_orientation(unsigned mode)
{
	char cmd[4] = { 0x54, 0xB1, 0x00, 0x01 };

	if (mode > 3) mode = 3;
	cmd[2] |= (char)mode;

	if (bi8232_send(cmd, ARRAY_SIZE(cmd)) < 0) {
		bi8232_msg(ERR, "set orientation failed");
		return -1;
	}
	msleep(250);

	return 0;
}

static int bi8232_get_resolution(struct elan_i2c_resolution *res)
{
	char cmd1[4] = { 0x53, 0x60, 0x00, 0x01 },	// x resolution
		 cmd2[4] = { 0x53, 0x63, 0x00, 0x01 };	// y resolution	

	bi8232_send(cmd1, ARRAY_SIZE(cmd1));
	wait_for_completion(&bi8232->data_ready);

	if (buffer[0] != 0x52) {
        complete(&bi8232->data_complete);
        return -1;
    }
	res->x = ((buffer[4] & 0xF0) << 4) | buffer[3];
	complete(&bi8232->data_complete);

	bi8232_send(cmd2, ARRAY_SIZE(cmd2));
	wait_for_completion(&bi8232->data_ready);

	if (buffer[0] != 0x52) {
		complete(&bi8232->data_complete);
		return -1;
	}
	res->y = ((buffer[4] & 0xF0) << 4) | buffer[3];
	complete(&bi8232->data_complete);

	return 0;
}

#if 0
static int bi8232_set_resolution(unsigned mode)
{
	char cmd[4] = { 0x54, 0x82, 0x80, 0x01 };

	if (mode > 2) mode = 2;
	cmd[2] |= ((char)mode << 4);

	if (bi8232_send(cmd, ARRAY_SIZE(cmd)) < 0) {
        bi8232_msg(ERR, "set resolution type failed");
        return -1;
    }

	return 0;
}

static int bi8232_get_deepsleep_mode(int *mode)
{
	char cmd[4] = { 0x53, 0xC0, 0x00, 0x01 };

	bi8232_send(cmd, ARRAY_SIZE(cmd));
	wait_for_completion(&bi8232->data_ready);

	if (buffer[0] != 0x52) {
        complete(&bi8232->data_complete);
        return -1;
    }

	*mode = (buffer[1] & 0x08) >> 3;
	complete(&bi8232->data_complete);

	return 0;
}

static int bi8232_set_deepsleep_mode(unsigned mode)
{
	char cmd[4] = { 0x54, 0xC0, 0x00, 0x01 };

	mode = mode ? 1 : 0;
	cmd[1] |= ((char)mode << 3);

	if (bi8232_send(cmd, ARRAY_SIZE(cmd)) < 0) {
        bi8232_msg(ERR, "set deepsleep function failed");
        return -1;
    }

    return 0;
}
#endif

#define FWID_HBYTE(x) (((((int)(x)[1]) & 0xF) << 4) + ((((int)(x)[2]) & 0xF0) >> 4))
#define FWID_LBYTE(x) (((((int)(x)[2]) & 0xF) << 4) + ((((int)(x)[3]) & 0xF0) >> 4))
static int bi8232_get_fw_id(int *id)
{
	char cmd[4] = { 0x53, 0xF0, 0x00, 0x01 };

	bi8232_send(cmd, ARRAY_SIZE(cmd));
	wait_for_completion(&bi8232->data_ready);

	if (buffer[0] != 0x52) {
        complete(&bi8232->data_complete);
        return -1;
    }

	*id = (FWID_HBYTE(buffer) << 8) + FWID_LBYTE(buffer);
	complete(&bi8232->data_complete);

	return 0;
}

#if 0
static int bi8232_get_report_rate(int *rate)
{
	char cmd[4] = { 0x53, 0xB0, 0x00, 0x01 };

	bi8232_send(cmd, ARRAY_SIZE(cmd));
	wait_for_completion(&bi8232->data_ready);

	if (res[0] != 0x52) {
        complete(&bi8232->data_complete);
        return -1;
    }

	*rate = buffer[1] & 0xF;
	complete(&bi8232->data_complete);

	return 0;
}

static int bi8232_set_report_rate(unsigned rate)
{
	char cmd[4] = { 0x54, 0xE0, 0x00, 0x01 };

	if (rate > 5) rate = 5;
	cmd[1] |= (char)rate;

	if (bi8232_send(cmd, ARRAY_SIZE(cmd)) < 0) {
        bi8232_msg(ERR, "set report rate failed");
        return -1;
    }

    return 0;
}
#endif

static int bi8232_get_sensitivity(struct elan_i2c_sensitivity *sen)
{
	char cmd[4] = { 0x53, 0x40, 0x00, 0x01 };

	bi8232_send(cmd, ARRAY_SIZE(cmd));
	wait_for_completion(&bi8232->data_ready);

	if (buffer[0] != 0x52) {
		complete(&bi8232->data_complete);
		return -1;
	}

	sen->x = buffer[2] & 0x0F;
	sen->y = (buffer[2] & 0xF0) >> 4;
	complete(&bi8232->data_complete);

	return 0;
}

static int bi8232_set_sensitivity(struct elan_i2c_sensitivity *sen)
{
	char cmd[4] = { 0x54, 0x40, 0x00, 0xA1 };  //Modified for new FW V1.1

	if (sen->x > 0x0F) sen->x = 0x0F;
	if (sen->y > 0x0F) sen->y = 0x0F;

	cmd[2] = sen->x | (sen->y << 4);
	if (bi8232_send(cmd, ARRAY_SIZE(cmd)) < 0) {
		bi8232_msg(ERR, "set sensitivity failed");
		return -1;
	}

	return 0;
}

/* FIH, Henry Juang, 2009/11/20 ++*/
/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
int notify_from_proximity(bool bFlag)
{
	bIsNeedSkipTouchEvent = bFlag;
    	bi8232_msg(ERR, "[TOUCH]notify_from_proximity = %d\r\n", bIsNeedSkipTouchEvent);
    	printk("[TOUCH]notify_from_proximity = %d\r\n", bIsNeedSkipTouchEvent);
    
	return 1;
}
EXPORT_SYMBOL(notify_from_proximity);
/* FIH, Henry Juang, 2009/11/20 --*/

#define XCORD1(x) ((((int)((x)[1]) & 0xF0) << 4) + ((int)((x)[2])))
#define YCORD1(y) ((((int)((y)[1]) & 0x0F) << 8) + ((int)((y)[3])))
#define XCORD2(x) ((((int)((x)[4]) & 0xF0) << 4) + ((int)((x)[5])))
#define YCORD2(y) ((((int)((y)[4]) & 0x0F) << 8) + ((int)((y)[6])))
static void bi8232_isr_workqueue(struct work_struct *work)
{
	struct input_dev *input = bi8232->input;
	int cnt, virtual_button;  //Modified for new CAP sample by Stanley (2009/05/25)
	
	disable_irq(bi8232->client->irq);
	bi8232_recv(buffer, ARRAY_SIZE(buffer));
	if (buffer[0] == 0x5A) {
		cnt = (buffer[8] ^ 0x01) >> 1;
		virtual_button = (buffer[8]) >> 3;  //Modified for Home and AP key (2009/07/31)
        //bi8232_msg(INFO, "[TOUCH-CAP]virtual button = %d\r\n", virtual_button); 
        //Modified for new CAP sample by Stanley ++(2009/05/25)
        if((virtual_button == 0) && !bSoft1CapKeyPressed && !bSoft2CapKeyPressed && !bHomeCapKeyPressed && !bApCapKeyPressed && !bCenterKeyPressed && !bIsNeedSkipTouchEvent)  /* FIH, Henry Juang, 2009/11/20 ++*/ /* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
        {
            #if 0
		    input_report_key(input, BTN_TOUCH, cnt > 0);
		    input_report_key(input, BTN_2, cnt == 2);
		    input_report_key(input, BTN_3, cnt == 3);
		    
		    if (cnt) {
			    input_report_abs(input, ABS_X, (1792 - XCORD1(buffer)));  //Add for protect origin point
			    input_report_abs(input, ABS_Y, YCORD1(buffer));
		    }
		    if (cnt > 1) {
                input_report_abs(input, ABS_HAT0X, (1792 - XCORD2(buffer)));  //Add for protect origin point
                input_report_abs(input, ABS_HAT0Y, YCORD2(buffer));
            }
            #endif
            //Added the MT protocol for Eclair by Stanley (2010/03/23)++
			if (cnt) {
				input_report_abs(input, ABS_MT_TOUCH_MAJOR, 255);
				input_report_abs(input, ABS_MT_POSITION_X, (1792 - XCORD1(buffer)));
				input_report_abs(input, ABS_MT_POSITION_Y, YCORD1(buffer));
				input_mt_sync(input);
			} else {
				input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(input, ABS_MT_POSITION_X, (1792 - XCORD1(buffer)));
				input_report_abs(input, ABS_MT_POSITION_Y, YCORD1(buffer));
				input_mt_sync(input);
			}
			if (cnt > 1) {
				input_report_abs(input, ABS_MT_TOUCH_MAJOR, 255);
				input_report_abs(input, ABS_MT_POSITION_X, (1792 - XCORD2(buffer)));
				input_report_abs(input, ABS_MT_POSITION_Y, YCORD2(buffer));
				input_mt_sync(input);
			} else {
				input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(input, ABS_MT_POSITION_X, (1792 - XCORD2(buffer)));
				input_report_abs(input, ABS_MT_POSITION_Y, YCORD2(buffer));
				input_mt_sync(input);
			}
			//Added the MT protocol for Eclair by Stanley (2010/03/23)--
            //Added log for debugging++
            //{
                if (cnt && bPrintPenDown)
                {
                    //bi8232_msg(INFO, "[TOUCH-CAP]Pen down x = %d, y = %d", 1792 - XCORD1(buffer), YCORD1(buffer));
                    bPrintPenDown = 0;
                }
                else if (!cnt)
                {
                    //bi8232_msg(INFO, "[TOUCH-CAP]Pen up x = %d, y = %d", XCORD1(buffer), YCORD1(buffer));
                    bPrintPenDown = 1;
                }
            //}
            //Added log for debugging--
            //Added for touch behavior (2009/08/14)++
            if(!cnt)
                bIsPenUp = 1;
            else
                bIsPenUp = 0;
            //Added for touch behavior (2009/08/14)--
        }
        else if ((virtual_button == 0) && (bSoft1CapKeyPressed || bSoft2CapKeyPressed || bHomeCapKeyPressed || bApCapKeyPressed || bCenterKeyPressed) && !bIsKeyLock)  //Button up  //Modified for Home and AP key (2009/07/31)
        {
            if(bSoft1CapKeyPressed)
            {
                //input_report_key(input, KEY_KBDILLUMDOWN, 0);
                //Added for FST++ 
                if(bIsFST)
                    input_report_key(input, KEY_SEND, 0);  //FST
                else
                    input_report_key(input, KEY_KBDILLUMDOWN, 0);
                //Added for FST--
                bi8232_msg(INFO, "[TOUCH-CAP]virtual button SOFT1 - up!\r\n");
                bSoft1CapKeyPressed = 0;
                bSoft2CapKeyPressed = 0;
                bHomeCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
                bApCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
                bCenterKeyPressed = 0;  //Added for FST
            }
            else if(bSoft2CapKeyPressed)
            {
                //input_report_key(input, KEY_BACK, 0);
                //Added for FST++
                if(bIsFST)
                    input_report_key(input, KEY_END, 0);  //FST
                else
                    input_report_key(input, KEY_BACK, 0);
                //Added for FST--
                bi8232_msg(INFO, "[TOUCH-CAP]virtual button SOFT2 - up!\r\n");
                bSoft2CapKeyPressed = 0;
                bSoft1CapKeyPressed = 0;
                bHomeCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
                bApCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
                bCenterKeyPressed = 0;  //Added for FST
            }
            //Modified for Home and AP key (2009/07/31)++
            else if(bHomeCapKeyPressed)
            {
                //if(bIsF913)
                    //input_report_key(input, KEYCODE_SEACHER, 0);  //F913
                //else
                    //input_report_key(input, KEY_HOME, 0);  //F905 or other
                //Added for FST++
                if(bIsF913 && !bIsFST)
                    input_report_key(input, KEYCODE_SEACHER, 0);  //F913
                else if(bIsFST)
                    input_report_key(input, KEY_KBDILLUMDOWN, 0);  //FST
                else
                    input_report_key(input, KEY_HOME, 0);  //F905 or other
                //Added for FST--
                bi8232_msg(INFO, "[TOUCH-CAP]virtual button HOME key - up!\r\n");
                bSoft2CapKeyPressed = 0;
                bSoft1CapKeyPressed = 0;
                bHomeCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
                bApCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
                bCenterKeyPressed = 0;  //Added for FST
            }
            else if(bApCapKeyPressed)
            {
                //input_report_key(input, KEYCODE_BROWSER, 0);
                //Added for FST++
                if(bIsFST)
                    input_report_key(input, KEY_BACK, 0);  //FST
                else if(bIsGRECO)
                    input_report_key(input, KEY_SEARCH, 0);  //Added for GRECO
                else
                    input_report_key(input, KEYCODE_BROWSER, 0);
                //Added for FST--
                bi8232_msg(INFO, "[TOUCH-CAP]virtual button AP key - up!\r\n");
                bSoft2CapKeyPressed = 0;
                bSoft1CapKeyPressed = 0;
                bHomeCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
                bApCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
                bCenterKeyPressed = 0;  //Added for FST
            }
            //Added for FST++
            else if(bCenterKeyPressed)
            {
                input_report_key(input, KEYCODE_SEACHER, 0);
                bi8232_msg(INFO, "[TOUCH-CAP]virtual button Center key - up!\r\n");
                bSoft2CapKeyPressed = 0;
                bSoft1CapKeyPressed = 0;
                bHomeCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
                bApCapKeyPressed = 0;  //Modified for Home and AP key (2009/07/31)
                bCenterKeyPressed = 0;  //Added for FST
            }
            //Added for FST--
            //Modified for Home and AP key (2009/07/31)--
        }
        else if(virtual_button == 16 && !bSoft2CapKeyPressed && !bIsNeedSkipTouchEvent)  //Button 4 //Modified for Home and AP key (2009/07/31)  /* FIH, Henry Juang, 2009/11/20 ++*/ /* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
        {
            //Added for touch behavior (2009/08/14)++
            if(!bIsPenUp)
            {
                //input_report_key(input, BTN_TOUCH, 0);
                //Added for F0XE.B-346++
                input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
                input_mt_sync(input);
                //Added for F0XE.B-346--
                bIsPenUp = 1;
                bi8232_msg(INFO, "[TOUCH-CAP]Send BTN touch - up!\r\n");
                bIsKeyLock = 1;  //Added for new behavior (2009/09/27)
            }
            //Added for touch behavior (2009/08/14)--
            //Added for FST++
            if(!bIsKeyLock)  //Added for new behavior (2009/09/27)
            {
                if(bIsFST)
                    input_report_key(input, KEY_END, 1);  //FST
                else
                    input_report_key(input, KEY_BACK, 1);
            }
            //Added for FST--
            bIsKeyLock = 0;  //Added for new behavior (2009/09/27)
            bi8232_msg(INFO, "[TOUCH-CAP]virtual button SOFT2 - down!\r\n");
            bSoft2CapKeyPressed = 1;
        }
        //Modified for Home and AP key (2009/07/31)++
        else if(virtual_button == 8 && !bApCapKeyPressed && !bIsNeedSkipTouchEvent)  //Button 3 //Modified for Home and AP key (2009/07/31)  /* FIH, Henry Juang, 2009/11/20 ++*/ /* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
        {
            //Added for touch behavior (2009/08/14)++
            if(!bIsPenUp)
            {
                //input_report_key(input, BTN_TOUCH, 0);
                //Added for F0XE.B-346++
                input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
                input_mt_sync(input);
                //Added for F0XE.B-346--
                bIsPenUp = 1;
                bi8232_msg(INFO, "[TOUCH-CAP]Send BTN touch - up!\r\n");
                bIsKeyLock = 1;  //Added for new behavior (2009/09/27)
            }
            //Added for touch behavior (2009/08/14)--
            //Added for FST++
            if(!bIsKeyLock)  //Added for new behavior (2009/09/27)
            {
                if(bIsFST)
                    input_report_key(input, KEY_BACK, 1);  //FST
                else if(bIsGRECO)
                    input_report_key(input, KEY_SEARCH, 1);  //Added for GRECO
                else
                    input_report_key(input, KEYCODE_BROWSER, 1);
            }
            //Added for FST--
            bIsKeyLock = 0;  //Added for new behavior (2009/09/27)
            bi8232_msg(INFO, "[TOUCH-CAP]virtual button AP key - down!\r\n");
            bApCapKeyPressed = 1;
        }
        else if(virtual_button == 4 && !bHomeCapKeyPressed && !bIsNeedSkipTouchEvent)  //Button 2 //Modified for Home and AP key (2009/07/31)  /* FIH, Henry Juang, 2009/11/20 ++*/ /* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
        {
            //Added for touch behavior (2009/08/14)++
            if(!bIsPenUp)
            {
                //input_report_key(input, BTN_TOUCH, 0);
                //Added for F0XE.B-346++
                input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
                input_mt_sync(input);
                //Added for F0XE.B-346--
                bIsPenUp = 1;
                bi8232_msg(INFO, "[TOUCH-CAP]Send BTN touch - up!\r\n");
                bIsKeyLock = 1;  //Added for new behavior (2009/09/27)
            }
            //Added for touch behavior (2009/08/14)--
            if(!bIsKeyLock)  //Added for new behavior (2009/09/27)
            {
                if(bIsF913 && !bIsFST)  //Added for FST
                    input_report_key(input, KEYCODE_SEACHER, 1);  //F913
                //Added for FST++
                else if(bIsFST)
                    input_report_key(input, KEY_KBDILLUMDOWN, 1);  //FST
                //Added for FST--
                else
                    input_report_key(input, KEY_HOME, 1);  //F905 or other
            }
            bIsKeyLock = 0;  //Added for new behavior (2009/09/27)
            bi8232_msg(INFO, "[TOUCH-CAP]virtual button HOME key - down!\r\n");
            bHomeCapKeyPressed = 1;
        }
        //Modified for Home and AP key (2009/07/31)--
        else if(virtual_button == 2 && !bSoft1CapKeyPressed && !bIsNeedSkipTouchEvent)  //Button 1 //Modified for Home and AP key (2009/07/31)  /* FIH, Henry Juang, 2009/11/20 ++*/ /* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
        {
            //Added for touch behavior (2009/08/14)++
            if(!bIsPenUp)
            {
                //input_report_key(input, BTN_TOUCH, 0);
                //Added for F0XE.B-346++
                input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
                input_mt_sync(input);
                //Added for F0XE.B-346--
                bIsPenUp = 1;
                bi8232_msg(INFO, "[TOUCH-CAP]Send BTN touch - up!\r\n");
                bIsKeyLock = 1;  //Added for new behavior (2009/09/27)
            }
            //Added for touch behavior (2009/08/14)--
            //Added for FST++ 
            if(!bIsKeyLock)  //Added for new behavior (2009/09/27)
            {
                if(bIsFST)
                    input_report_key(input, KEY_SEND, 1);  //FST
                else{
                    input_report_key(input, KEY_KBDILLUMDOWN, 1);
                    //bi8232_msg(INFO, "[TOUCH-CAP]virtual button KEY_KBDILLUMDOWN1 - down!\r\n");
                }
            }
            //Added for FST--
            bIsKeyLock = 0;  //Added for new behavior (2009/09/27)
            bi8232_msg(INFO, "[TOUCH-CAP]virtual button SOFT1 - down!\r\n");
            bSoft1CapKeyPressed = 1;
        }
        //Added for FST++
        else if(virtual_button == 1 && !bCenterKeyPressed && !bIsNeedSkipTouchEvent)  //Added for FST  //Button 5  /* FIH, Henry Juang, 2009/11/20 ++*/ /* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
        {
            //Added for touch behavior (2009/08/14)++
            if(!bIsPenUp)
            {
                //input_report_key(input, BTN_TOUCH, 0);
                //Added for F0XE.B-346++
                input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
                input_mt_sync(input);
                //Added for F0XE.B-346--
                bIsPenUp = 1;
                bi8232_msg(INFO, "[TOUCH-CAP]Send BTN touch - up!\r\n");
                bIsKeyLock = 1;  //Added for new behavior (2009/09/27)
            }
            //Added for touch behavior (2009/08/14)--
            if(!bIsKeyLock)
                input_report_key(input, KEYCODE_SEACHER, 1);
            bIsKeyLock = 0;  //Added for new behavior (2009/09/27)
            bi8232_msg(INFO, "[TOUCH-CAP]virtual button Center - down!\r\n");
            bCenterKeyPressed = 1;  //Added for FST
        }
        //Added for FST--
        //Modified for new CAP sample by Stanley --(2009/05/25)
		input_sync(input);
	} else if (buffer[0] == 0x52) {
		complete(&bi8232->data_ready);
		wait_for_completion(&bi8232->data_complete);
	}
	//Modify the scheme for receive hello packet++
	else if ((buffer[0] == 0x55) && (buffer[1] == 0x55) && (buffer[2] == 0x55) && (buffer[3] == 0x55))
	{
	    bi8232_msg(INFO, "[TOUCH-CAP]Receive the hello packet!\r\n");	    
	}
	gpio_clear_detect_status(bi8232->client->irq);  
	//Modify the scheme for receive hello packet--
	enable_irq(bi8232->client->irq);
}

static irqreturn_t bi8232_isr(int irq, void * dev_id)
{
	//disable_irq(irq);
	schedule_work(&bi8232->wqueue);

	return IRQ_HANDLED;
}

static int input_open(struct input_dev * idev)
{
	struct i2c_client *client = bi8232->client;

    if (request_irq(client->irq, bi8232_isr, 2, TOUCH_NAME, bi8232)) {
        bi8232_msg(ERR, "can not register irq %d", client->irq);
		return -1;
    }

	return 0;
}

static void input_close(struct input_dev *idev)
{
	struct i2c_client *client = bi8232->client;
	
	free_irq(client->irq, bi8232);
}

static int bi8232_misc_open(struct inode *inode, struct file *file)
{
    if ((file->f_flags & O_ACCMODE) == O_WRONLY) {
		bi8232_msg(INFO, "device node is readonly");
        return -1;
    }

	return 0;
}

static int bi8232_misc_release(struct inode *inode, struct file *file)
{
    return 0;
}

static int bi8232_misc_ioctl(struct inode *inode, struct file *file,
									unsigned cmd, unsigned long arg)
{
	int value, ret = 0;
	struct elan_i2c_resolution res;
	struct elan_i2c_sensitivity sen;

	if (_IOC_TYPE(cmd) != BI8232_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > BI8232_IOC_MAXNR) return -ENOTTY;

	switch(cmd) {
	case BI8232_IOC_GFWVERSION:
		if (bi8232_get_fw_version(&value) < 0)
			return -EIO;
		ret = put_user(value, (int __user *)arg);
		break;
	case BI8232_IOC_GPWSTATE:
		if (bi8232_get_pw_state(&value) < 0)
			return -EIO;
		ret = put_user(value, (int __user *)arg);
		break;
	case BI8232_IOC_GORIENTATION:
		if (bi8232_get_orientation(&value) < 0)
			return -EIO;
		ret = put_user(value, (int __user *)arg);
		break;
	case BI8232_IOC_GRESOLUTION:
		if (bi8232_get_resolution(&res) < 0)
			return -EIO;
		if(copy_to_user((int __user*)arg, &res,
			sizeof(struct elan_i2c_resolution)))
			ret = -EFAULT;
		break;
#if 0
	case BI8232_IOC_GDEEPSLEEP:
		if (bi8232_get_deepsleep_mode(&value) < 0)
            return -EIO;
        ret = put_user(value, (int __user *)arg);
        break;
#endif
	case BI8232_IOC_GFWID:
		if (bi8232_get_fw_id(&value) < 0)
			return -EIO;
		ret = put_user(value, (int __user *)arg);
        break;
#if 0
	case BI8232_IOC_GREPORTRATE:
		if (bi8232_get_report_rate(&value) < 0)
			return -EIO;
		ret = put_user(value, (int __user *)arg);
        break;
#endif
	case BI8232_IOC_GSENSITIVITY:
		if (bi8232_get_sensitivity(&sen) < 0)
			return -EIO;
		if(copy_to_user((int __user*)arg, &sen,
			sizeof(struct elan_i2c_sensitivity)))
			ret = -EFAULT;
		break;
	case BI8232_IOC_SPWSTATE:
		ret = get_user(value, (int __user *)arg);
		if (ret == 0)
			ret = bi8232_set_pw_state(value);
		break;
	case BI8232_IOC_SORIENTATION:
		ret = get_user(value, (int __user *)arg);
		if (ret == 0)
			ret = bi8232_set_orientation(value);
		break;
#if 0
	case BI8232_IOC_SRESOLUTION:
		ret = get_user(value, (int __user *)arg);
		if (ret == 0)
            ret = bi8232_set_resolution(value);
		break;
	case BI8232_IOC_SDEEPSLEEP:
		ret = get_user(value, (int __user *)arg);
		if (ret == 0)
			ret = bi8232_set_deepsleep_mode(value);
		break;
	case BI8232_IOC_SREPORTRATE:
		ret = get_user(value, (int __user *)arg);
		if (ret == 0)
			ret = bi8232_set_report_rate(value);
		break;
#endif
	case BI8232_IOC_SSENSITIVITY:
		if (copy_from_user((int __user *)arg, &sen,
				sizeof(struct elan_i2c_sensitivity))) {
			ret = -EFAULT;
			break;
		}
		ret = bi8232_set_sensitivity(&sen);
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static struct file_operations bi8232_misc_fops = {
    .open	= bi8232_misc_open,
    .ioctl	= bi8232_misc_ioctl,
    .release= bi8232_misc_release,
};

static struct miscdevice bi8232_misc_dev = {
    .minor= MISC_DYNAMIC_MINOR,
    .name = TOUCH_NAME,
	.fops = &bi8232_misc_fops,
};

#ifdef CONFIG_PM
static int bi8232_suspend(struct i2c_client *client, pm_message_t state)
{
    struct vreg *vreg_wlan;  //Add for VREG_WLAN power in, 07/08
    struct elan_i2c_platform_data *pdata = client->dev.platform_data;  //Setting the configuration of GPIO 89
    int ret, ret_gpio = 0;  //Add for VREG_WLAN power in, 07/08

	cancel_work_sync(&bi8232->wqueue);
    printk(KERN_INFO "bi8232_suspend() disable IRQ: %d\n", client->irq);
    disable_irq(client->irq);

    //Add for VREG_WLAN power in++
    if((FIH_READ_HWID_FROM_SMEM() != CMCS_7627_PR1) && (FIH_READ_HWID_FROM_SMEM() != CMCS_F913_PR1))  //Don't apply VREG_WLAN power in on PR1++
    {
        vreg_wlan = vreg_get(0, "wlan");

	    if (!vreg_wlan) {
		    printk(KERN_ERR "%s: vreg WLAN get failed\n", __func__);
		return -EIO;
	    }

	    ret = vreg_disable(vreg_wlan);
	    if (ret) {
		    printk(KERN_ERR "%s: vreg WLAN disable failed (%d)\n",
		        __func__, ret);
		return -EIO;
	    }

	    printk(KERN_INFO "%s: vote vreg WLAN to be closed\n", __func__);
	}
	//Add for VREG_WLAN power in--
	//return bi8232_set_pw_state(0);
	//Setting the configuration of GPIO 89++
	ret_gpio = gpio_tlmm_config(GPIO_CFG(pdata->intr_gpio, 0, GPIO_INPUT,
						GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
	if(ret_gpio < 0)
	{
	    msm_cap_touch_gpio_pull_fail = 1;
	    printk(KERN_INFO "bi8232_suspend(): GPIO89_PULL_DOWN failed!\n");
	}
	printk(KERN_INFO "bi8232_suspend(): GPIO89_PULL_DOWN\n");
	//Setting the configuration of GPIO 89--
	return 0; //Remove to set power state by Stanley
}

static int bi8232_resume(struct i2c_client *client)
{
    struct vreg *vreg_wlan;  //Add for VREG_WLAN power in, 07/08
    struct elan_i2c_sensitivity sen;  //Added for modify sensitivity, 0729
    struct elan_i2c_platform_data *pdata = client->dev.platform_data;  //Setting the configuration of GPIO 89
    int ret, i, ret_gpio = 0;  //Added for modify sensitivity, 0729
    //int retry = 20,			/* retry count of device detecting */	    
		//pkt;				/* packet buffer */
    char sensitivity_x = 0xF, sensitivity_y = 0xF;  //Added for modify sensitivity, 0817
//Remove to set power state by Stanley++
#if 0
	int state = 0,
		retry = 10;

	bi8232_set_pw_state(1);
	do {
		bi8232_get_pw_state(&state);
		if (--retry == 0) {
			bi8232_msg(ERR, "can not wake device up");
			return -1;
		}
	} while (!state);
#endif
//Remove to set power state by Stanley--
    //printk(KERN_INFO "bi8232_resume() enable IRQ: %d\n", client->irq);
	//enable_irq(client->irq);

    //Add for VREG_WLAN power in++
    if((FIH_READ_HWID_FROM_SMEM() != CMCS_7627_PR1) && (FIH_READ_HWID_FROM_SMEM() != CMCS_F913_PR1))  //Don't apply VREG_WLAN power in on PR1++
    {
        vreg_wlan = vreg_get(0, "wlan");

	    if (!vreg_wlan) {
		    printk(KERN_ERR "%s: vreg WLAN get failed\n", __func__);
		return -EIO;
	    }

	    ret = vreg_enable(vreg_wlan);
	    if (ret) {
		    printk(KERN_ERR "%s: vreg WLAN enable failed (%d)\n",
		        __func__, ret);
		return -EIO;
	    }

	    //Added for receive hello packet during resuming++
	    #if 0
        if(bIsFxxPR2)
        {
	        msleep(300);
	    	
	        do {
		        bi8232_recv((char *)&pkt, sizeof(int));
		        pkt ^= 0x55555555;
                bi8232_msg(ERR, "[TOUCH-CAP]Try to receive the hello packet!");
		        if (--retry == 0) {
			        bi8232_msg(ERR, "Detect timeout");
			        break;
			        //goto err1;
		        }
	        } while (pkt);
	    }
	    #endif
	    //Added for receive hello packet during resuming--
	}
	//Add for VREG_WLAN power in--
	//Add for modify sensitivity, 0729++
    sen.x = sensitivity_x;
    sen.y = sensitivity_y;
    if(!bIsNewFWVer)  //Add for detect firmware version++
    {
        for(i = 0 ; i < 20 ; i++)
	    {
	        if (bi8232_set_sensitivity(&sen) < 0)
	        {
                bi8232_msg(INFO, "[TOUCH-CAP]bi8232_set_sensitivity failed!\r\n");
                msleep(100);
            }
            else
            {
                bi8232_msg(INFO, "[TOUCH-CAP]bi8232_set_sensitivity successful!\r\n"); 
                break;
            }
	    }
	}
	//Add for modify sensitivity, 0729--
	//Setting the configuration of GPIO 89++
	ret_gpio = gpio_tlmm_config(GPIO_CFG(pdata->intr_gpio, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
	if(ret_gpio < 0)
	{
	    msm_cap_touch_gpio_pull_fail = 1;
	    printk(KERN_INFO "bi8232_suspend(): GPIO89_PULL_UP failed!\n");
	}
	//Setting the configuration of GPIO 89--
	//Modify the scheme for receive hello packet++
	printk(KERN_INFO "bi8232_resume() enable IRQ: %d and GPIO89_PULL_UP\n", client->irq);
	enable_irq(client->irq);
	//Modify the scheme for receive hello packet--
	return 0;
}
#else
#define bi8232_suspend	NULL
#define bi8232_resume	NULL
#endif

//Added for FST by Stanley (F0X_2.B-414)++
#ifdef CONFIG_PM
static int bi8232_fst_suspend(struct i2c_client *client, pm_message_t state)
{
    struct vreg *vreg_wlan;  //Add for VREG_WLAN power in, 07/08
    struct elan_i2c_platform_data *pdata = client->dev.platform_data;  //Setting the configuration of GPIO 89
    int ret = 0, ret_gpio = 0;  //Add for VREG_WLAN power in, 07/08

	cancel_work_sync(&bi8232->wqueue);
    printk(KERN_INFO "bi8232_fst_suspend() disable IRQ: %d\n", client->irq);
    if (!bPhoneCallState || (bPhoneCallState && bIsNeedSkipTouchEvent))
        disable_irq(client->irq);
    else
    {
        disable_irq(client->irq);
	    if (device_may_wakeup(&client->dev)) {
	        enable_irq_wake(client->irq);
	        printk(KERN_INFO "[%s]Enable irq_wake!\n", __func__);
	    }
	}
	
    //Add for VREG_WLAN power in++
    #if 1
    if((FIH_READ_HWID_FROM_SMEM() != CMCS_7627_PR1) && (FIH_READ_HWID_FROM_SMEM() != CMCS_F913_PR1) && (!bPhoneCallState || (bPhoneCallState && bIsNeedSkipTouchEvent)))  //Don't apply VREG_WLAN power in on PR1++
    {
        vreg_wlan = vreg_get(0, "wlan");

	    if (!vreg_wlan) {
		    printk(KERN_ERR "%s: vreg WLAN get failed\n", __func__);
		return -EIO;
	    }

        #if 1 
	    ret = vreg_disable(vreg_wlan);
	    if (ret) {
		    printk(KERN_ERR "%s: vreg WLAN disable failed (%d)\n",
		        __func__, ret);
		return -EIO;
	    }
        #endif
        bIsFSTDisableWLAN = 1;  //Added for VREG sync by Stanley
        vreg_refcnt = vreg_refcnt - 1;  //Added for VREG sync by Stanley
	    printk(KERN_INFO "%s: vote vreg WLAN to be closed. refcnt = %d\n", __func__, vreg_refcnt);  //Added for VREG sync by Stanley
	}
	#endif
	//Add for VREG_WLAN power in--
	//return bi8232_set_pw_state(0);
	//Setting the configuration of GPIO 89++
	if (!bPhoneCallState || (bPhoneCallState && bIsNeedSkipTouchEvent))
	{
	    ret_gpio = gpio_tlmm_config(GPIO_CFG(pdata->intr_gpio, 0, GPIO_INPUT,
						    GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
		if(ret_gpio < 0)
	    {
	        msm_cap_touch_gpio_pull_fail = 1;
	        printk(KERN_INFO "bi8232_fst_suspend(): GPIO89_PULL_DOWN failed!\n");
	    }
    }
    else
    {
        ret_gpio = gpio_tlmm_config(GPIO_CFG(pdata->intr_gpio, 0, GPIO_INPUT,
						    GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
		if(ret_gpio < 0)
	    {
	        msm_cap_touch_gpio_pull_fail = 1;
	        printk(KERN_INFO "bi8232_fst_suspend(): GPIO89_PULL_UP failed (bPhoneCallState is TRUE)!\n");
	    }
    }
    
	printk(KERN_INFO "bi8232_fst_suspend(): GPIO89_PULL_DOWN\n");
	//Setting the configuration of GPIO 89--
	return 0; //Remove to set power state by Stanley
}

static int bi8232_fst_resume(struct i2c_client *client)
{
    struct vreg *vreg_wlan;  //Add for VREG_WLAN power in, 07/08
    //struct elan_i2c_sensitivity sen;  //Added for modify sensitivity, 0729
    struct elan_i2c_platform_data *pdata = client->dev.platform_data;  //Setting the configuration of GPIO 89
    int ret = 0, ret_gpio = 0;  //Added for modify sensitivity, 0729
    //int retry = 20,			/* retry count of device detecting */	    
		//pkt;				/* packet buffer */
    //char sensitivity_x = 0xF, sensitivity_y = 0xF;  //Added for modify sensitivity, 0817

    //printk(KERN_INFO "bi8232_resume() enable IRQ: %d\n", client->irq);
	//enable_irq(client->irq);

    //Add for VREG_WLAN power in++
    #if 1
    if((FIH_READ_HWID_FROM_SMEM() != CMCS_7627_PR1) && (FIH_READ_HWID_FROM_SMEM() != CMCS_F913_PR1) && bIsFSTDisableWLAN)  //Added for VREG sync by Stanley
    {
        vreg_wlan = vreg_get(0, "wlan");

	    if (!vreg_wlan) {
		    printk(KERN_ERR "%s: vreg WLAN get failed\n", __func__);
		return -EIO;
	    }

        #if 1
	    ret = vreg_enable(vreg_wlan);
	    if (ret) {
		    printk(KERN_ERR "%s: vreg WLAN enable failed (%d)\n",
		        __func__, ret);
		return -EIO;
	    }
	    #endif
	    bIsFSTDisableWLAN = 0; //Added for VREG sync by Stanley
	    vreg_refcnt = vreg_refcnt + 1;  //Added for VREG sync by Stanley
	    printk(KERN_INFO "%s: vote vreg WLAN to be enabled. refcnt = %d\n", __func__, vreg_refcnt);  //Added for VREG sync by Stanley
	}
	#endif
	//Add for VREG_WLAN power in--
	//Setting the configuration of GPIO 89++
	ret_gpio = gpio_tlmm_config(GPIO_CFG(pdata->intr_gpio, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
	if(ret_gpio < 0)
	{
	    msm_cap_touch_gpio_pull_fail = 1;
	    printk(KERN_INFO "bi8232_fst_suspend(): GPIO89_PULL_UP failed!\n");
	}
	//Setting the configuration of GPIO 89--
	//Modify the scheme for receive hello packet++
	printk(KERN_INFO "bi8232_fst_resume() enable IRQ: %d and GPIO89_PULL_UP\n", client->irq);
    if (!bPhoneCallState || (bPhoneCallState && bIsNeedSkipTouchEvent))
    	enable_irq(client->irq);
	else
	{
        enable_irq(client->irq);
	    if (device_may_wakeup(&client->dev)) {
	        disable_irq_wake(client->irq);
	        printk(KERN_INFO "[%s]Disable irq_wake!\n", __func__);
	    }
	}
	//Modify the scheme for receive hello packet--
	return 0;
}
#else
#define bi8232_suspend	NULL
#define bi8232_resume	NULL
#endif
//Added for FST by Stanley (F0X_2.B-414)--

/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change CAP touch suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
void bi8232_early_suspend(struct early_suspend *h)
{
    struct bi8232_m32emau *pbi8232;
	pbi8232 = container_of(h, struct bi8232_m32emau, bi8232_early_suspend_desc);

    printk(KERN_INFO "bi8232_early_suspend()\n");
    //bi8232_suspend(pbi8232->client, PMSG_SUSPEND);
    printk(KERN_INFO "bi8232_suspend() disable IRQ: %d\n", pbi8232->client->irq);
    disable_irq(pbi8232->client->irq);
    printk(KERN_INFO "bi8232_early_suspend() exit!\n");
}
void bi8232_late_resume(struct early_suspend *h)
{
    struct bi8232_m32emau *pbi8232;
	pbi8232 = container_of(h, struct bi8232_m32emau, bi8232_early_suspend_desc);

    printk(KERN_INFO "bi8232_late_resume()\n");
    //bi8232_resume(pbi8232->client);
    printk(KERN_INFO "bi8232_resume() enable IRQ: %d\n", pbi8232->client->irq);
	enable_irq(pbi8232->client->irq);
	printk(KERN_INFO "bi8232_late_resume() exit!\n");
}
//Added for FST by Stanley (F0X_2.B-414)++
void bi8232_fst_early_suspend(struct early_suspend *h)
{
    struct bi8232_m32emau *pbi8232;
	pbi8232 = container_of(h, struct bi8232_m32emau, bi8232_early_suspend_desc);

    printk(KERN_INFO "bi8232_fst_early_suspend()\n");
    //bi8232_suspend(pbi8232->client, PMSG_SUSPEND);
    bPhoneCallState = incoming_call_get();
    printk(KERN_INFO "bPhoneCallState: %d\n", bPhoneCallState);
    if (!bPhoneCallState)
    {
        disable_irq(pbi8232->client->irq);
        bIsNeedtoEnableIRQ = 1;  //Added to fix IRQ sync issue for F0X_2.B-414
        printk(KERN_INFO "bi8232_fst_early_suspend(): disable_irq!\n");
    }
    printk(KERN_INFO "bi8232_fst_early_suspend() exit!\n");
}
void bi8232_fst_late_resume(struct early_suspend *h)
{
    struct bi8232_m32emau *pbi8232;
	pbi8232 = container_of(h, struct bi8232_m32emau, bi8232_early_suspend_desc);

    printk(KERN_INFO "bi8232_fst_late_resume()\n");
    //bi8232_resume(pbi8232->client);
    bPhoneCallState = incoming_call_get();
    printk(KERN_INFO "bPhoneCallState: %d\n", bPhoneCallState);
    if (bIsNeedtoEnableIRQ)  //Added to fix IRQ sync issue for F0X_2.B-414
    {
	    enable_irq(pbi8232->client->irq);
	    bIsNeedtoEnableIRQ = 0;  //Added to fix IRQ sync issue for F0X_2.B-414
	    printk(KERN_INFO "bi8232_fst_late_resume(): enable_irq!\n");
    }
	printk(KERN_INFO "bi8232_fst_late_resume() exit!\n");
}
//Added for FST by Stanley (F0X_2.B-414)--
#endif	
#endif
/* } FIH, SimonSSChang, 2009/09/04 */

//Added by Stanley for dump scheme++
static int msm_seq_open(struct inode *inode, struct file *file)
{
	//printk(KERN_INFO "msm_open\n");
  	return single_open(file, NULL, NULL);
}

static ssize_t msm_seq_write(struct file *file, const char *buff, size_t len, loff_t *off)
{
	char str[64];
	int param = -1;
	int param2 = -1;
	int param3 = -1;
	char cmd[32];
	u32 sen_x = 2, sen_y = 3;

	struct elan_i2c_sensitivity sen;

	printk(KERN_INFO "MSM_TOUCH_Write ~~\n");
	if(copy_from_user(str, buff, sizeof(str)))
		return -EFAULT;	

  	if(sscanf(str, "%s %d %d %d", cmd, &param, &param2, &param3) == -1)
	{
	  	printk("parameter format: <type> <value>\n");

 		return -EINVAL;
	}	  	

	if(!strnicmp(cmd, "sen", 3))
	{	
		sen_x = param;
		sen_y = param2;
		printk(KERN_INFO "sen param = %d\n",sen_x);
	}
	else
	{
		printk(KERN_INFO "Parameter error!\n");
	}

    sen.x = sen_x;
    sen.y = sen_y;
	if (bi8232_set_sensitivity(&sen) < 0)
	    {
            bi8232_msg(INFO, "[TOUCH-CAP]bi8232_set_sensitivity failed!\r\n");
            msleep(100);
        }
	
	return len;
}

static ssize_t msm_seq_read(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
    //int bytes, reg;
    int bytes;
    char cmd[4] = { 0x53, 0x20, 0x00, 0x01 };
    char cmd2[4] = { 0x53, 0x30, 0x00, 0x01 };
    char cmd3[4] = { 0x53, 0x40, 0x00, 0x01 };
    char cmd4[4] = { 0x53, 0x60, 0x00, 0x01 };
    char cmd5[4] = { 0x53, 0x63, 0x00, 0x01 };
    char cmd6[4] = { 0x53, 0xB0, 0x00, 0x01 };
    char cmd7[4] = { 0x53, 0xB1, 0x00, 0x01 };
    
    if (*ppos != 0)
        return 0;
    
#if 0
    reg = readl(TSSC_REG(CTL));
    bytes = sprintf(data, "[TOUCH]TSSC_CTL : 0x%x\r\n", reg);
    *ppos += bytes;
    data += bytes;

    reg = readl(TSSC_REG(OPN));
    bytes = sprintf(data, "[TOUCH]TSSC_OPN : 0x%x\r\n", reg);
    *ppos += bytes;
    data += bytes;

    reg = readl(TSSC_REG(SI));
    bytes = sprintf(data, "[TOUCH]TSSC_SAMPLING_INT : 0x%x\r\n", reg);
    *ppos += bytes;
    data += bytes;

    reg = readl(TSSC_REG(STATUS));
    bytes = sprintf(data, "[TOUCH]TSSC_STATUS : 0x%x\r\n", reg);
    *ppos += bytes;
    data += bytes;

    reg = readl(TSSC_REG(AVG12));
    bytes = sprintf(data, "[TOUCH]TSSC_AVG_12 : 0x%x\r\n", reg);
    *ppos += bytes;
    data += bytes;
#endif
    bi8232_send(cmd, ARRAY_SIZE(cmd));
	bi8232_recv(buffer, ARRAY_SIZE(buffer));
	bytes = sprintf(data, "[TOUCH]X-axis absolute : 0x%x, 0x%x, 0x%x, 0x%x\r\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    *ppos += bytes;
    data += bytes;

    //cmd[4] = { 0x53, 0x30, 0x00, 0x01 };
    bi8232_send(cmd2, ARRAY_SIZE(cmd2));
	bi8232_recv(buffer, ARRAY_SIZE(buffer));
	bytes = sprintf(data, "[TOUCH]Y-axis absolute : 0x%x, 0x%x, 0x%x, 0x%x\r\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    *ppos += bytes;
    data += bytes;

    //cmd[4] = { 0x53, 0x40, 0x00, 0x01 };
    bi8232_send(cmd3, ARRAY_SIZE(cmd3));
	bi8232_recv(buffer, ARRAY_SIZE(buffer));
	bytes = sprintf(data, "[TOUCH]Sensitivity value : 0x%x, 0x%x, 0x%x, 0x%x\r\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    *ppos += bytes;
    data += bytes;

    //cmd[4] = { 0x53, 0x60, 0x00, 0x01 };
    bi8232_send(cmd4, ARRAY_SIZE(cmd4));
	bi8232_recv(buffer, ARRAY_SIZE(buffer));
	bytes = sprintf(data, "[TOUCH]X-axis resolution : 0x%x, 0x%x, 0x%x, 0x%x\r\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    *ppos += bytes;
    data += bytes;

    //cmd[4] = { 0x53, 0x63, 0x00, 0x01 };
    bi8232_send(cmd5, ARRAY_SIZE(cmd5));
	bi8232_recv(buffer, ARRAY_SIZE(buffer));
	bytes = sprintf(data, "[TOUCH]Y-axis resolution : 0x%x, 0x%x, 0x%x, 0x%x\r\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    *ppos += bytes;
    data += bytes;

    //cmd[4] = { 0x53, 0xB0, 0x00, 0x01 };
    bi8232_send(cmd6, ARRAY_SIZE(cmd6));
	bi8232_recv(buffer, ARRAY_SIZE(buffer));
	bytes = sprintf(data, "[TOUCH]Normal report state : 0x%x, 0x%x, 0x%x, 0x%x\r\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    *ppos += bytes;
    data += bytes;

    //cmd[4] = { 0x53, 0xB1, 0x00, 0x01 };
    bi8232_send(cmd7, ARRAY_SIZE(cmd7));
	bi8232_recv(buffer, ARRAY_SIZE(buffer));
	bytes = sprintf(data, "[TOUCH]Origin point state : 0x%x, 0x%x, 0x%x, 0x%x\r\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    *ppos += bytes;
    data += bytes;
    
    return *ppos;
}

static struct file_operations msm_touch_seq_fops =
{
  	.owner 		= THIS_MODULE,
	.open  		= msm_seq_open,
	.write 		= msm_seq_write,
	.read		= msm_seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
//Added by Stanley for dump scheme--

static int bi8232_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct input_dev *input;
	struct elan_i2c_platform_data *pdata;
	struct elan_i2c_sensitivity sen;  //Added for modify sensitivity, 0729
	int retry = 50,			/* retry count of device detecting */	    
		pkt;				/* packet buffer */
	int i, iValue = 0;  //Added for software reset
	char reset_cmd[4] = { 0x77, 0x77, 0x77, 0x77 };  //Added for software reset
	char sensitivity_x = 0xF, sensitivity_y = 0xF;  //Added for modify sensitivity, 0817
	
	bi8232 = kzalloc(sizeof(struct bi8232_m32emau), GFP_KERNEL);
	if (bi8232 == NULL) {
		bi8232_msg(ERR, "can not allocate memory for bi8232");
		return -ENOMEM;
	}

/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change CAP touch suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
    if(!bIsFST)
    {
        bi8232->bi8232_early_suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 11;
	    bi8232->bi8232_early_suspend_desc.suspend = bi8232_early_suspend;
	    bi8232->bi8232_early_suspend_desc.resume = bi8232_late_resume;
        printk(KERN_INFO "CAP_Touch register_early_suspend()\n");
	    register_early_suspend(&bi8232->bi8232_early_suspend_desc);
	}
	//Added for FST by Stanley (F0X_2.B-414)++
	else
	{
	    bi8232->bi8232_early_suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 11;
	    bi8232->bi8232_early_suspend_desc.suspend = bi8232_fst_early_suspend;
	    bi8232->bi8232_early_suspend_desc.resume = bi8232_fst_late_resume;
        printk(KERN_INFO "CAP_Touch register_fst_early_suspend()\n");
	    register_early_suspend(&bi8232->bi8232_early_suspend_desc);
	}
	//Added for FST by Stanley (F0X_2.B-414)--
#endif
#endif
/* } FIH, SimonSSChang, 2009/09/04 */
	
	bi8232->client = client;
	dev_set_drvdata(&client->dev, bi8232);

    device_init_wakeup(&client->dev, 1);  //Added for FST by Stanley (F0X_2.B-414)
    
    //Added for software reset++
    for(i = 0 ; i < 10 ; i++)
	{
	    if (bi8232_send(reset_cmd, ARRAY_SIZE(reset_cmd)) < 0)
	    {
            bi8232_msg(INFO, "[TOUCH-CAP]Software reset failed!\r\n");
            msleep(50);
        }
        else
        {
            bi8232_msg(INFO, "[TOUCH-CAP]Software reset successful!\r\n"); 
            break;
        }
	}
	msleep(300);
	//Added for software reset--
	
	do {
		bi8232_recv((char *)&pkt, sizeof(int));
		pkt ^= 0x55555555;

		if (--retry == 0) {
			bi8232_msg(ERR, "detect timeout");
			break;
			//goto err1;
		}
	} while (pkt);

	init_completion(&bi8232->data_ready);
	init_completion(&bi8232->data_complete);
	INIT_WORK(&bi8232->wqueue, bi8232_isr_workqueue);

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		bi8232_msg(ERR, "can not get platform data");
		goto err1;
	}

    input = input_allocate_device();
    if (input == NULL) {
		bi8232_msg(ERR, "can not allocate memory for input device");
        goto err1;
    }

    input->name = "Elan BI1050-M32EMAU Touchscreen";
    input->phys = "bi8232/input0";
    input->open = input_open;
    input->close= input_close;
	
    set_bit(EV_KEY, input->evbit);
    set_bit(EV_ABS, input->evbit);
    set_bit(EV_SYN, input->evbit);
    set_bit(BTN_TOUCH, input->keybit);
    set_bit(BTN_2, input->keybit);  //Added for Multi-touch
    set_bit(KEY_BACK, input->keybit);  //Modified for new CAP sample by Stanley (2009/05/25)
    set_bit(KEY_KBDILLUMDOWN, input->keybit);  //Modified for new CAP sample by Stanley (2009/05/25)
    //Modified for Home and AP key (2009/07/31)++
    if((FIH_READ_HWID_FROM_SMEM() >= CMCS_CTP_PR2) && (FIH_READ_HWID_FROM_SMEM() != CMCS_7627_PR1))
    {
        set_bit(KEY_HOME, input->keybit);
        set_bit(KEYCODE_BROWSER, input->keybit);
        set_bit(KEYCODE_SEACHER, input->keybit);
        set_bit(KEY_SEND, input->keybit);  //Added for FST
        set_bit(KEY_END, input->keybit);  //Added for FST
        bIsFxxPR2 = 1; //Added for PR2
    }
    if((FIH_READ_HWID_FROM_SMEM() >= CMCS_F913_PR1) && (FIH_READ_HWID_FROM_SMEM() <= CMCS_F913_MP1))
        bIsF913 = 1;
    else
        bIsF913 = 0;
    //Modified for Home and AP key (2009/07/31)--

    //Added for FST++
    if((FIH_READ_ORIG_HWID_FROM_SMEM() >= CMCS_125_FST_PR1) && (FIH_READ_ORIG_HWID_FROM_SMEM() <= CMCS_128_FST_MP1))
        bIsFST = 1;
    else
        bIsFST = 0;
    //Added for FST--

    //Added for GRECO++
    if((FIH_READ_ORIG_HWID_FROM_SMEM() >= CMCS_125_CTP_GRE_PR1) && (FIH_READ_ORIG_HWID_FROM_SMEM() <= CMCS_125_CTP_GRE_MP2))
        bIsGRECO = 1;
    else
        bIsGRECO = 0;
    //Added for GRECO--

    input_set_abs_params(input, ABS_X, pdata->abs_x_min,
								pdata->abs_x_max, 0, 0);
    input_set_abs_params(input, ABS_Y, pdata->abs_y_min,
								pdata->abs_y_max, 0, 0);
    input_set_abs_params(input, ABS_HAT0X, pdata->abs_x_min,
								pdata->abs_x_max, 0, 0);
    input_set_abs_params(input, ABS_HAT0Y, pdata->abs_y_min,
								pdata->abs_y_max, 0, 0);
    //Added the MT protocol for Eclair by Stanley (2010/03/23)++
    input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0,
								255, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_X, pdata->abs_x_min,
								pdata->abs_x_max, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, pdata->abs_y_min,
								pdata->abs_y_max, 0, 0);
	//Added the MT protocol for Eclair by Stanley (2010/03/23)--
	bi8232->input = input;
    if (input_register_device(bi8232->input)) {
		bi8232_msg(ERR, "can not register input device");
        goto err2;
	}

	if (misc_register(&bi8232_misc_dev)) {
		bi8232_msg(ERR, "can not add misc device");
		goto err3;
    }

	if (MSM_GPIO_TO_INT(pdata->intr_gpio) != client->irq) {
		bi8232_msg(ERR, "irq not match");
		goto err4;
	}
	gpio_tlmm_config(GPIO_CFG(pdata->intr_gpio, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
    //Added for retry to set orientation++
#if 0 //Add for protect origin point
	for(l = 0 ; l < 10 ; l++)
	{
	    if (bi8232_set_orientation(2) < 0)  //Modified for new CAP sample by Stanley (2009/05/25)
	    {
            bi8232_msg(INFO, "[TOUCH-CAP]bi8232_set_orientation set failed!\r\n");
            msleep(10);
        }
        else
            break;    
	}
#endif //Add for protect origin point
	//Added for retry to set orientation--
	//Add for detect firmware version++	
	if (bi8232_get_fw_version(&iValue) >= 0)
	{
        bi8232_msg(INFO, "[TOUCH-CAP]bi8232 firmware version = %d", iValue);
        msm_cap_touch_fw_version = iValue;  //Added for show FW version on FQC
        //if(iValue >= 103)
        if((iValue >= 103) || bIsFST)  //Added for FST project (2010/03/05)
        {
            bIsNewFWVer = 1;
            bi8232_msg(INFO, "[TOUCH-CAP]bi8232 firmware version >= V1.03");
        }
  	}
	else
    {
        bi8232_msg(INFO, "[TOUCH-CAP]bi8232 firmware version failed!\r\n");
        return -EIO;
    }
    //Add for detect firmware version--
    //Add for modify sensitivity, 0729++
    sen.x = sensitivity_x;
    sen.y = sensitivity_y;
    if(!bIsNewFWVer)  //Add for detect firmware version
    {
        for(i = 0 ; i < 10 ; i++)
	    {
	        if (bi8232_set_sensitivity(&sen) < 0)
	        {
                bi8232_msg(INFO, "[TOUCH-CAP]bi8232_set_sensitivity failed!\r\n");
                msleep(50);
            }
            else
            {
                bi8232_msg(INFO, "[TOUCH-CAP]bi8232_set_sensitivity successful!\r\n"); 
                break;
            }
	    }
	}
	//Add for modify sensitivity, 0729--
	bi8232_msg(INFO, "[TOUCH-CAP]bi8232_probe init ok!\r\n");
	//Added by Stanley for dump scheme++
  	msm_touch_proc_file = create_proc_entry("driver/cap_touch", 0666, NULL);
	
	if(!msm_touch_proc_file){
	  	printk(KERN_INFO "create proc file for Msm_touch failed\n");
		return -ENOMEM;
	}

	printk(KERN_INFO "Msm_touch proc ok\n");
	msm_touch_proc_file->proc_fops = &msm_touch_seq_fops;
	//Added by Stanley for dump scheme--
    return 0;

err4:
	misc_deregister(&bi8232_misc_dev);
err3:
	input_unregister_device(bi8232->input);
err2:
	input_free_device(input);
err1:
	dev_set_drvdata(&client->dev, 0);
	kfree(bi8232);
	return -1;
}

static int bi8232_remove(struct i2c_client * client)
{
/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change CAP touch suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
    printk(KERN_INFO "CAP_Touch unregister_early_suspend()\n");
	unregister_early_suspend(&bi8232->bi8232_early_suspend_desc);
#endif
#endif
/* } FIH, SimonSSChang, 2009/09/04 */

	misc_deregister(&bi8232_misc_dev);
	input_unregister_device(bi8232->input);
    dev_set_drvdata(&client->dev, 0);
    kfree(bi8232);

	return 0;
}

static const struct i2c_device_id bi8232_id[] = {
    { TOUCH_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bi8232);

static struct i2c_driver bi8232_i2c_driver = {
	.driver = {
		.name	= TOUCH_NAME,
		.owner	= THIS_MODULE,
	},
	.id_table   = bi8232_id,
	.probe  	= bi8232_probe,
	.remove 	= bi8232_remove,
/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change CAP touch suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
	.suspend	= bi8232_suspend,
    .resume		= bi8232_resume,
#else
	.suspend	= bi8232_suspend,
    .resume		= bi8232_resume,
#endif	
/* } FIH, SimonSSChang, 2009/09/04 */
};

static int __init bi8232_init( void )
{
    struct vreg *vreg_wlan;  //Add for VREG_WLAN power in, 09/10
    int ret;
	
    //Dynamic to load RES or CAP touch driver++
    if(FIH_READ_HWID_FROM_SMEM() >= CMCS_CTP_PR1)
    {
         //Neo: Add for increasing VREG_WLAN refcnt and let VREG_WLAN can be closed at first suspend +++
         if((FIH_READ_HWID_FROM_SMEM() != CMCS_7627_PR1) && (FIH_READ_HWID_FROM_SMEM() != CMCS_F913_PR1))  //Don't apply VREG_WLAN power in on PR1++
         {
            vreg_wlan = vreg_get(0, "wlan");

            if (!vreg_wlan) {
	         printk(KERN_ERR "%s: init vreg WLAN get failed\n", __func__);
            }

            ret = vreg_enable(vreg_wlan);
            if (ret) {
	          printk(KERN_ERR "%s: init vreg WLAN enable failed (%d)\n", __func__, ret);
            }
            vreg_refcnt = vreg_refcnt + 1;  //Added for VREG sync by Stanley
         }
         //Neo: Add for increasing VREG_WLAN refcnt and let VREG_WLAN can be closed at first suspend ---

         //Added for FST by Stanley (F0X_2.B-414)++
         if((FIH_READ_ORIG_HWID_FROM_SMEM() >= CMCS_125_FST_PR1) && (FIH_READ_ORIG_HWID_FROM_SMEM() <= CMCS_128_FST_MP1))
         {
             bIsFST = 1;
             bi8232_i2c_driver.suspend = bi8232_fst_suspend;
             bi8232_i2c_driver.resume = bi8232_fst_resume;
             bi8232_msg(INFO, "[TOUCH-CAP]bi8232_init: FST device!\r\n"); 
         }
         else
             bIsFST = 0;
         //Added for FST by Stanley (F0X_2.B-414)--
		
	  return i2c_add_driver(&bi8232_i2c_driver);
    }
	else
	    return -ENODEV;
	//Dynamic to load RES or CAP touch driver--
}

static void __exit bi8232_exit( void )
{
	i2c_del_driver(&bi8232_i2c_driver);
}

module_init(bi8232_init);
module_exit(bi8232_exit);

MODULE_DESCRIPTION("Elan BI8232-M32EMAU Touchscreen driver");
MODULE_AUTHOR("Eric Pan <erictcpan@tp.cmcs.com.tw>");
MODULE_VERSION("0:1.4");
MODULE_LICENSE("GPL");
