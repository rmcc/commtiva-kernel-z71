/* drivers/input/touchscreen/msm_touch.c
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>  //[FIH_ADQ.B-296]
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/io.h>
//Added for touch calibration++
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include "../../../arch/arm/mach-msm/proc_comm.h"
//Added for touch calibration--

#include <mach/msm_touch.h>
//Modified for RTP PR1 setting++
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>
//Modified for RTP PR1 setting-- 
#include <mach/gpio.h>  //Modified for Home (Search) and AP key (2009/08/10)

/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change RES touch suspend/resume function
to earlysuspend */
#include <linux/earlysuspend.h>
/* } FIH, SimonSSChang, 2009/09/04 */

/* HW register map */
#define TSSC_CTL_REG      0x100
#define TSSC_SI_REG       0x108
#define TSSC_OPN_REG      0x104
#define TSSC_STATUS_REG   0x10C
#define TSSC_AVG12_REG    0x110

/* status bits */
#define TSSC_STS_OPN_SHIFT 0x6
#define TSSC_STS_OPN_BMSK  0x1C0
#define TSSC_STS_NUMSAMP_SHFT 0x1
#define TSSC_STS_NUMSAMP_BMSK 0x3E

/* CTL bits */
#define TSSC_CTL_EN		(0x1 << 0)
#define TSSC_CTL_DI		(0x0 << 0)  //[FIH_ADQ.B-296]
#define TSSC_CTL_SW_RESET	(0x1 << 2)
#define TSSC_CTL_MASTER_MODE	(0x3 << 3)
#define TSSC_CTL_AVG_EN		(0x1 << 5)
#define TSSC_CTL_DEB_EN		(0x1 << 6)
#define TSSC_CTL_DEB_12_MS	(0x2 << 7)	/* 1.2 ms */
#define TSSC_CTL_DEB_16_MS	(0x3 << 7)	/* 1.6 ms */
#define TSSC_CTL_DEB_2_MS	(0x4 << 7)	/* 2 ms */
#define TSSC_CTL_DEB_3_MS	(0x5 << 7)	/* 3 ms */
#define TSSC_CTL_DEB_4_MS	(0x6 << 7)	/* 4 ms */
#define TSSC_CTL_DEB_6_MS	(0x7 << 7)	/* 6 ms */
#define TSSC_CTL_INTR_FLAG1	(0x1 << 10)
#define TSSC_CTL_DATA		(0x1 << 11)
#define TSSC_CTL_SSBI_CTRL_EN	(0x1 << 13)

/* control reg's default state */
#define TSSC_CTL_STATE	  ( \
		TSSC_CTL_DEB_12_MS | \
		TSSC_CTL_DEB_EN | \
		TSSC_CTL_AVG_EN | \
		TSSC_CTL_MASTER_MODE | \
		TSSC_CTL_EN)

//[FIH_ADQ.B-296]++
/* control reg's suspend state */
#define TSSC_CTL_SUSPEND_STATE	  ( \
		TSSC_CTL_DI)
//[FIH_ADQ.B-296]--

#define TSSC_NUMBER_OF_OPERATIONS 2
#define TS_PENUP_TIMEOUT_MS 15  //[FIH_ADQ] Modified by Stanley

//#define TS_DRIVER_NAME "msm_touchscreen"
#define TS_DRIVER_NAME "msm_touch"

#define X_MAX	1024
#define Y_MAX	1024
#define P_MAX	256
#define VKEY_UP_BOUNDARY 876  //Redefine the virtual key boundary
#define VKEY_UP_BOUNDARY_PR2 756  //Modified for RTP PR2 setting, 2009/07/06
#define VKEY_SOFT1_MIDDLE_BOUNDARY 325  //Redefine the virtual key boundary
#define VKEY_SOFT1_MIDDLE_BOUNDARY_PR2 313  //Modified for Home (Search) and AP key (2009/08/10)++
#define VKEY_SOFT2_MIDDLE_BOUNDARY 610  //Redefine the virtual key boundary
#define VKEY_SOFT2_MIDDLE_BOUNDARY_PR2 618  //Modified for Home (Search) and AP key (2009/08/10)++
//Modified for Home (Search) and AP key (2009/08/10)++
#define VKEY_HOME_KEY_LEFT_BOUNDARY 336
#define VKEY_HOME_KEY_RIGHT_BOUNDARY 454
#define VKEY_AP_KEY_LEFT_BOUNDARY 477
#define VKEY_AP_KEY_RIGHT_BOUNDARY 595
#define GPIO_NEG_Y 93
#define KEYCODE_BROWSER 192
#define KEYCODE_SEACHER 217
//Modified for Home (Search) and AP key (2009/08/10)--
#define NV_RES_TCH_BOUNDARY_I 8031  //Added for new touch calibration

u32 DEFAULT_SI_VALUE = 20;  //[FIH_ADQ] Added by Stanley
bool bSoft1KeyPressed = 0;  //Modified for RTP PR1 setting	
bool bSoft2KeyPressed = 0;  //Modified for RTP PR1 setting
bool bHomeKeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
bool bApKeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
bool bIsPR = 0;  //Modify HW ID to support larger than PR1
bool bIsPR2 = 0;  //Modify HW ID to support larger than PR1
bool bIsEVB = 0;  //Modify HW ID to support larger than PR1
bool bIsPR3 = 0;  //Modified for Home (Search) and AP key (2009/08/10)
bool bIsMP = 0;  //Added for new touch calibration++
bool bIsJBallExist = 0;  //Modified for Home (Search) and AP key (2009/08/10)
//Added for new touch calibration++
u32 uX_Max = 0, uX_Min = 0, uY_Max = 0, uY_Min = 0, uVkeyUpBoundaryMP = 0;
u32 uVkeySoft1MiddleBoundary = 0, uVkeySoft2MiddleBoundary = 0,
    uVkeyHomeLeftBoundary = 0, uVkeyHomeRightBoundary = 0,
    uVkeyApLeftBoundary = 0, uVkeyApRightBoundary = 0;
int iTCHBoundaryItem = NV_RES_TCH_BOUNDARY_I;
bool bIsReadFromNV = 0;
//bool gbIsFactoryTest = 0;
struct input_dev *input_dev;
//Added for new touch calibration--

struct ts {
	struct input_dev *input;
	struct timer_list timer;
	int irq;
	unsigned int x_max;
	unsigned int y_max;
	u32 last_x, last_y;
/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change RES touch suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend ts_early_suspend_desc;
    struct platform_device *pdev;
#endif
#endif
/* } FIH, SimonSSChang, 2009/09/04 */    
};

static void __iomem *virt;
#define TSSC_REG(reg) (virt + TSSC_##reg##_REG)

static void ts_update_pen_state(struct ts *ts, int x, int y, int pressure)
{
	//if (pressure) {
		input_report_abs(ts->input, ABS_X, x);
		input_report_abs(ts->input, ABS_Y, y);
		input_report_abs(ts->input, ABS_PRESSURE, pressure);
		input_report_key(ts->input, BTN_TOUCH, !!pressure);
	//} else {
		//input_report_abs(ts->input, ABS_PRESSURE, 0);
		//input_report_key(ts->input, BTN_TOUCH, 0);
	//}
	input_sync(ts->input);
}

static void ts_timer(unsigned long arg)
{
	struct ts *ts = (struct ts *)arg;

//	ts_update_pen_state(ts, 0, 0, 0);
#ifdef CONFIG_FIH_FXX
	input_report_abs(ts->input, ABS_X, ts->last_x);
	input_report_abs(ts->input, ABS_Y, ts->last_y);
	input_report_abs(ts->input, ABS_PRESSURE, 0);
	input_report_key(ts->input, BTN_TOUCH, 0);

	input_sync(ts->input);

    //Modified for RTP PR1 setting++	
    if (bIsPR || bIsPR2 || bIsPR3 || bIsMP)  //Added for new touch calibration
    {
	    if(bSoft1KeyPressed)
	    {
	        //Modified for Home (Search) and AP key (2009/08/10)++
            if(bIsPR || bIsPR3 || bIsJBallExist || bIsMP){
                input_report_key(ts->input, KEY_KBDILLUMDOWN, 0);
                printk(KERN_INFO "[TOUCH] Soft key 1 (PR3 or later)!!\r\n");
            }
            else{
                input_report_key(ts->input, KEY_HOME, 0);
                printk(KERN_INFO "[TOUCH] Soft key 1 (PR2)!!\r\n");
            }
            //Modified for Home (Search) and AP key (2009/08/10)--
	        //input_report_key(ts->input, KEY_KBDILLUMDOWN, 0);
	        bSoft1KeyPressed = 0;
	        bSoft2KeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        bHomeKeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        bApKeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        printk(KERN_INFO "[TOUCH] Soft key 1 up!!\r\n");
	    }
	    else if(bSoft2KeyPressed)
	    {
	        input_report_key(ts->input, KEY_BACK, 0);
	        bSoft2KeyPressed = 0;
	        bSoft1KeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        bHomeKeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        bApKeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        printk(KERN_INFO "[TOUCH] Soft key 2 up!!\r\n");
	    }
	    //Modified for Home (Search) and AP key (2009/08/10)++
	    else if(bHomeKeyPressed)
	    {
	        if(bIsJBallExist)
                input_report_key(ts->input, KEY_HOME, 0);  //F910
            else
                input_report_key(ts->input, KEYCODE_SEACHER, 0);  //F902  
	        bHomeKeyPressed = 0;
	        bSoft1KeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        bSoft2KeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        bApKeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        printk(KERN_INFO "[TOUCH] Home (Search) key up!!\r\n");
	    }
	    else if(bApKeyPressed)
	    {
	        input_report_key(ts->input, KEYCODE_BROWSER, 0);
	        bApKeyPressed = 0;
	        bSoft1KeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        bSoft2KeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        bHomeKeyPressed = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	        printk(KERN_INFO "[TOUCH] AP key up!!\r\n");
	    }
	    //Modified for Home (Search) and AP key (2009/08/10)--
	}
    //Modified for RTP PR1 setting--
#else
    ts_update_pen_state(ts, 0, 0, 0);
#endif
}

static irqreturn_t ts_interrupt(int irq, void *dev_id)
{
	u32 avgs, x, y, lx, ly, si;  //[FIH_ADQ] Modified by Stanley
	u32 num_op, num_samp;
	u32 status;

	struct ts *ts = dev_id;

	status = readl(TSSC_REG(STATUS));
	avgs = readl(TSSC_REG(AVG12));
	x = avgs & 0xFFFF;
	y = avgs >> 16;

	si = readl(TSSC_REG(SI));  //[FIH_ADQ] Added by Stanley

	/* For pen down make sure that the data just read is still valid.
	 * The DATA bit will still be set if the ARM9 hasn't clobbered
	 * the TSSC. If it's not set, then it doesn't need to be cleared
	 * here, so just return.
	 */
	if (!(readl(TSSC_REG(CTL)) & TSSC_CTL_DATA))
		goto out;

	/* Data has been read, OK to clear the data flag */
	writel(TSSC_CTL_STATE, TSSC_REG(CTL));

    //[FIH_ADQ]++ Added by Stanley++
	/* Set dafault sampling interval */
    if (si != DEFAULT_SI_VALUE)
    {
	    writel(DEFAULT_SI_VALUE, TSSC_REG(SI));
	    //printk(KERN_INFO "[TOUCH] ts_interrupt() : Set TSSC sampling interval!\n");
	}
	//[FIH_ADQ]-- Added by Stanley--

	/* Valid samples are indicated by the sample number in the status
	 * register being the number of expected samples and the number of
	 * samples collected being zero (this check is due to ADC contention).
	 */
	num_op = (status & TSSC_STS_OPN_BMSK) >> TSSC_STS_OPN_SHIFT;
	num_samp = (status & TSSC_STS_NUMSAMP_BMSK) >> TSSC_STS_NUMSAMP_SHFT;

	if ((num_op == TSSC_NUMBER_OF_OPERATIONS) && (num_samp == 0)) {
		/* TSSC can do Z axis measurment, but driver doesn't support
		 * this yet.
		 */

		/*
		 * REMOVE THIS:
		 * These x, y co-ordinates adjustments will be removed once
		 * Android framework adds calibration framework.
		 */
#ifdef CONFIG_ANDROID_TOUCHSCREEN_MSM_HACKS
		lx = ts->x_max + 25 - x;
		ly = ts->y_max + 25 - y;
#else
		lx = x;
		ly = y;
#endif
		ts->last_x = lx;
		ts->last_y = ly;

        //Modified for RTP PR1 setting++ 
        //printk(KERN_INFO "EVB: %d, PR: %d, PR2: %d", bIsEVB, bIsPR, bIsPR2);
		//if((ly <= VKEY_UP_BOUNDARY) || bIsEVB)  //Modified by Stanley
		if(bIsEVB || ((ly <= VKEY_UP_BOUNDARY_PR2) && (bIsPR2 || bIsPR3)) || ((ly <= VKEY_UP_BOUNDARY) && bIsPR) || ((ly <= uVkeyUpBoundaryMP) && bIsMP))  //Added for new touch calibration
        {
		    ts_update_pen_state(ts, lx, ly, 255);
		}
		else if((ly > VKEY_UP_BOUNDARY) && bIsPR)  //Modified by Stanley 
		{
            if ((lx < VKEY_SOFT1_MIDDLE_BOUNDARY) && !bSoft1KeyPressed)  //Redefine the virtual key boundary
            {
                input_report_key(ts->input, KEY_KBDILLUMDOWN, 1);
                bSoft1KeyPressed = 1;
                printk(KERN_INFO "[TOUCH] Soft key 1 down!!\r\n");
            }
            else if ((lx > VKEY_SOFT2_MIDDLE_BOUNDARY) && !bSoft2KeyPressed)  //Redefine the virtual key boundary
            {
                input_report_key(ts->input, KEY_BACK, 1);
                bSoft2KeyPressed = 1;
                printk(KERN_INFO "[TOUCH] Soft key 2 down!!\r\n");
            }
            //printk(KERN_INFO "[TOUCH] Virtual key down!!\r\n");
		}
        //Modified for RTP PR1 setting--
        //Modified for RTP PR2 setting, 2009/07/06++
        else if((ly > VKEY_UP_BOUNDARY_PR2) && (bIsPR2 || bIsPR3))  //Modified by Stanley 
		{
            if ((lx < VKEY_SOFT1_MIDDLE_BOUNDARY_PR2) && !bSoft1KeyPressed)  //Redefine the virtual key boundary
            {
                //Modified for Home (Search) and AP key (2009/08/10)++
                if(bIsPR3 || bIsJBallExist){
                    input_report_key(ts->input, KEY_KBDILLUMDOWN, 1);
                    printk(KERN_INFO "[TOUCH] Soft key 1 (PR3 or later)!!\r\n");
                }
                else{
                    input_report_key(ts->input, KEY_HOME, 1);
                    printk(KERN_INFO "[TOUCH] Soft key 1 (PR2)!!\r\n");
                }
                //Modified for Home (Search) and AP key (2009/08/10)--
                bSoft1KeyPressed = 1;
                printk(KERN_INFO "[TOUCH] Soft key 1 down!!\r\n");
            }
            else if ((lx > VKEY_SOFT2_MIDDLE_BOUNDARY_PR2) && !bSoft2KeyPressed)  //Redefine the virtual key boundary
            {
                input_report_key(ts->input, KEY_BACK, 1);
                bSoft2KeyPressed = 1;
                printk(KERN_INFO "[TOUCH] Soft key 2 down!!\r\n");
            }
            //Modified for Home (Search) and AP key (2009/08/10)++
            else if ((lx >= VKEY_HOME_KEY_LEFT_BOUNDARY) && (lx <= VKEY_HOME_KEY_RIGHT_BOUNDARY) && !bHomeKeyPressed)
            {
                if(bIsJBallExist){
                    input_report_key(ts->input, KEY_HOME, 1);  //F910
                    printk(KERN_INFO "[TOUCH] F910!!\r\n");
                }
                else{
                    input_report_key(ts->input, KEYCODE_SEACHER, 1);  //F902
                    printk(KERN_INFO "[TOUCH] F902!!\r\n");
                }
                bHomeKeyPressed = 1;
                printk(KERN_INFO "[TOUCH] Home (Search) key down!!\r\n");
            }
            else if ((lx >= VKEY_AP_KEY_LEFT_BOUNDARY) && (lx <= VKEY_AP_KEY_RIGHT_BOUNDARY) && !bApKeyPressed)
            {
                input_report_key(ts->input, KEYCODE_BROWSER, 1);
                bApKeyPressed = 1;
                printk(KERN_INFO "[TOUCH] AP key down!!\r\n");
            }
            //Modified for Home (Search) and AP key (2009/08/10)--
            //printk(KERN_INFO "[TOUCH] Virtual key down!!\r\n");
		}
		//Added for new touch calibration++
		else if((ly > uVkeyUpBoundaryMP) && bIsMP)  //Modified by Stanley 
		{
            if ((lx < uVkeySoft1MiddleBoundary) && !bSoft1KeyPressed)  //Redefine the virtual key boundary
            {
                //Modified for Home (Search) and AP key (2009/08/10)++
                if(bIsPR3 || bIsJBallExist || bIsMP){
                    input_report_key(ts->input, KEY_KBDILLUMDOWN, 1);
                    printk(KERN_INFO "[TOUCH] Soft key 1 (PR3 or later)!!\r\n");
                }
                else{
                    input_report_key(ts->input, KEY_HOME, 1);
                    printk(KERN_INFO "[TOUCH] Soft key 1 (PR2)!!\r\n");
                }
                //Modified for Home (Search) and AP key (2009/08/10)--
                bSoft1KeyPressed = 1;
                printk(KERN_INFO "[TOUCH] Soft key 1 down!!\r\n");
            }
            else if ((lx > uVkeySoft2MiddleBoundary) && !bSoft2KeyPressed)  //Redefine the virtual key boundary
            {
                input_report_key(ts->input, KEY_BACK, 1);
                bSoft2KeyPressed = 1;
                printk(KERN_INFO "[TOUCH] Soft key 2 down!!\r\n");
            }
            //Modified for Home (Search) and AP key (2009/08/10)++
            else if ((lx >= uVkeyHomeLeftBoundary) && (lx <= uVkeyHomeRightBoundary) && !bHomeKeyPressed)
            {
                if(bIsJBallExist){
                    input_report_key(ts->input, KEY_HOME, 1);  //F910
                    printk(KERN_INFO "[TOUCH] F910!!\r\n");
                }
                else{
                    input_report_key(ts->input, KEYCODE_SEACHER, 1);  //F902
                    printk(KERN_INFO "[TOUCH] F902!!\r\n");
                }
                bHomeKeyPressed = 1;
                printk(KERN_INFO "[TOUCH] Home (Search) key down!!\r\n");
            }
            else if ((lx >= uVkeyApLeftBoundary) && (lx <= uVkeyApRightBoundary) && !bApKeyPressed)
            {
                input_report_key(ts->input, KEYCODE_BROWSER, 1);
                bApKeyPressed = 1;
                printk(KERN_INFO "[TOUCH] AP key down!!\r\n");
            }
            //Modified for Home (Search) and AP key (2009/08/10)--
            //printk(KERN_INFO "[TOUCH] Virtual key down!!\r\n");
		}
		//Added for new touch calibration--
        //Modified for RTP PR2 setting, 2009/07/06--
#if 0
		//printk(KERN_INFO "[TOUCH] Interrupt : {%3d, %3d},"
				//" op = %3d samp = %3d si = %d\r\n", lx, ly, num_op, num_samp, si);  //[FIH_ADQ] Added by Stanley
        //printk(KERN_INFO "[TOUCH] HWID : %d\r\n", FIH_READ_HWID_FROM_SMEM());
#endif
		/* kick pen up timer - to make sure it expires again(!) */
		mod_timer(&ts->timer,
			jiffies + msecs_to_jiffies(TS_PENUP_TIMEOUT_MS + DEFAULT_SI_VALUE));  //[FIH_ADQ] Modified by Stanley

	} else
		printk(KERN_INFO "Ignored interrupt: {%3d, %3d},"
				" op = %3d samp = %3d\n",
				 x, y, num_op, num_samp);

out:
	return IRQ_HANDLED;
}

//[FIH_ADQ.B-296]++
static int ts_suspend(struct platform_device *pdev, pm_message_t message)
{
    struct ts *ts = platform_get_drvdata(pdev);
    
    //Disable IRQ
    printk(KERN_INFO "ts_suspend() disable IRQ: %d\n", ts->irq);
    disable_irq(ts->irq);
	
	//Disable the TSSC
	writel(TSSC_CTL_EN, TSSC_REG(CTL));
	writel(TSSC_CTL_SUSPEND_STATE, TSSC_REG(CTL));
	printk(KERN_INFO "[TOUCH] ts_suspend() : Disable the TSSC IRQ!\n");
	
	return 0;
}

static int ts_resume(struct platform_device *pdev)
{
    struct ts *ts = platform_get_drvdata(pdev);

    //Enable IRQ
    printk(KERN_INFO "ts_suspend() enable IRQ: %d\n", ts->irq);    
	enable_irq(ts->irq);
	
	//Enable and re-initial the TSSC
	writel(TSSC_CTL_STATE, TSSC_REG(CTL));
	printk(KERN_INFO "[TOUCH] ts_resume() : Enable the TSSC IRQ!\n");
	
	return 0;
}
//[FIH_ADQ.B-296]--

/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change RES touch suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
void ts_early_suspend(struct early_suspend *h)
{
    struct ts *ts;
	ts = container_of(h, struct ts, ts_early_suspend_desc);

    printk(KERN_INFO "ts_early_suspend()\n");
    ts_suspend(ts->pdev, PMSG_SUSPEND);
}
void ts_late_resume(struct early_suspend *h)
{
    struct ts *ts;
	ts = container_of(h, struct ts, ts_early_suspend_desc);

    printk(KERN_INFO "ts_late_resume()\n");
    ts_resume(ts->pdev);
}
#endif	
#endif
/* } FIH, SimonSSChang, 2009/09/04 */

//Added for new touch calibration++
static int msm_misc_open(struct inode *inode, struct file *file)
{
    if ((file->f_flags & O_ACCMODE) == O_WRONLY) {
        printk(KERN_INFO "[TOUCH] msm_misc_open() : device node is readonly!\n");
		return -1;
    }

	return 0;
}

static int msm_misc_release(struct inode *inode, struct file *file)
{
    return 0;
}

static int msm_misc_ioctl(struct inode *inode, struct file *file,
									unsigned cmd, unsigned long arg)
{
    int ret = 0, iInterval = 0, iButtonWidth = 0;
    unsigned int iTCHBoundaryItem[3] = {NV_RES_TCH_BOUNDARY_I, 0x0, 0x0};
    unsigned int iTCHBoundaryItemForRead[3] = {NV_RES_TCH_BOUNDARY_I, 0x0, 0x0};
    struct msm_data_from_ap *data;
    struct msm_data_from_ap data2;
    
	if (_IOC_TYPE(cmd) != MSM_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > MSM_IOC_MAXNR) return -ENOTTY;

    data = (struct msm_data_from_ap*)arg;
	switch(cmd) {
	
	case MSM_IOC_BOUNDARY:
	    printk(KERN_INFO "[TOUCH]msm_misc_ioctl: x_max = %d, x_min = %d, y_max = %d, y_min = %d.\n", data->x_max, data->x_min, data->y_max, data->y_min);
        uX_Max = data->x_max;
        uX_Min = data->x_min;
        uY_Max = data->y_max;
        uY_Min = data->y_min;
        uVkeyUpBoundaryMP = data->y_max;
		//if (copy_from_user((int __user *)data, arg,
				//sizeof(struct msm_data_from_ap))) {
			//ret = -EFAULT;
			//printk(KERN_INFO "[TOUCH]msm_misc_ioctl: Transfer failed!\n);
			//break;
		//}
		if((FIH_READ_HWID_FROM_SMEM() >= CMCS_RTP_MP1) && (FIH_READ_HWID_FROM_SMEM() <= CMCS_RTP_MP3))
        {
	        input_set_abs_params(input_dev, ABS_X, uX_Min, uX_Max, 0, 0);  //Modified by Stanley
	        input_set_abs_params(input_dev, ABS_Y, uY_Min, uY_Max, 0, 0);  //Modified by Stanley
	    
	        iInterval = (uX_Max - uX_Min) / 25;
	        iButtonWidth = ((uX_Max - uX_Min) - (iInterval * 3)) / 4;
	        uVkeySoft1MiddleBoundary = uX_Min + iButtonWidth;
	        uVkeyHomeLeftBoundary = uVkeySoft1MiddleBoundary + iInterval;
	        uVkeyHomeRightBoundary = uVkeyHomeLeftBoundary + iButtonWidth;
	        uVkeyApLeftBoundary = uVkeyHomeRightBoundary + iInterval;
	        uVkeyApRightBoundary = uVkeyApLeftBoundary + iButtonWidth;
	        uVkeySoft2MiddleBoundary = uVkeyApRightBoundary + iInterval;
	        printk(KERN_INFO "[TOUCH]msm_misc_ioctl: iInterval = %d, iButtonWidth = %d," 
	                      "uVkeySoft1MiddleBoundary = %d, uVkeyHomeLeftBoundary = %d,"
	                      "uVkeyHomeRightBoundary = %d, uVkeyApLeftBoundary = %d,"
	                      "uVkeyApRightBoundary = %d, uVkeySoft2MiddleBoundary = %d.\r\n",
	                      iInterval, iButtonWidth, uVkeySoft1MiddleBoundary, uVkeyHomeLeftBoundary,
	                      uVkeyHomeRightBoundary, uVkeyApLeftBoundary, uVkeyApRightBoundary, uVkeySoft2MiddleBoundary);
        }
        //memcpy(&iTCHBoundaryItem[1],(unsigned *)data, 8);
	    //Write boundary value to NV item.
	    if(((uX_Max - uX_Min) > 500) && ((uY_Max - uY_Min) > 500))
        {
	        data2.x_max = uX_Max;
            data2.x_min = uX_Min;
            data2.y_max = uY_Max;
            data2.y_min = uY_Min;
	        memcpy(&iTCHBoundaryItem[1], &data2, 8);
	        proc_comm_write_nv(iTCHBoundaryItem);
	    }
	    else
	        printk(KERN_INFO "[TOUCH-RES]msm_misc_ioctl: NV item value invalid!\r\n");
		
		break;

	case MSM_IOC_ISFACTORYTEST:
	    //printk(KERN_INFO "[TOUCH]msm_misc_ioctl: IsFactoryTest = %d.\n", data->bIsFactoryTest);
        //arg = data->bIsFactoryTest;
        //return data->bIsFactoryTest = 1;
        if(proc_comm_read_nv(iTCHBoundaryItemForRead))  //Read fail
        {
            printk(KERN_INFO "[TOUCH]msm_misc_ioctl: Read NV fail!\n");
            return 1;
        }
        else
        {
            printk(KERN_INFO "[TOUCH]msm_misc_ioctl: Read NV successful!\n");
            return 0;
        }	    

	    break;
	    
	default:
		return -ENOTTY;
	}

	return ret;
}



static struct file_operations msm_misc_fops = {
    .open	= msm_misc_open,
    .ioctl	= msm_misc_ioctl,
    .release= msm_misc_release,
};

static struct miscdevice msm_misc_dev = {
    .minor= MISC_DYNAMIC_MINOR,
    .name = TS_DRIVER_NAME,
	.fops = &msm_misc_fops,
};
//Added for new touch calibration--

static int __devinit ts_probe(struct platform_device *pdev)
{
	int result;
	int iInterval = 0, iButtonWidth = 0;
	//struct input_dev *input_dev;
	struct resource *res, *ioarea;
	struct ts *ts;
	unsigned int x_max, y_max, pressure_max;
	struct msm_ts_platform_data *pdata = pdev->dev.platform_data;
	//Added for new touch calibration++
	struct msm_data_from_ap data;
	unsigned int iTCHBoundaryItem[3] = {NV_RES_TCH_BOUNDARY_I, 0x0, 0x0};
	//Added for new touch calibration--

	/* The primary initialization of the TS Hardware
	 * is taken care of by the ADC code on the modem side
	 */

	ts = kzalloc(sizeof(struct ts), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!input_dev || !ts) {
		result = -ENOMEM;
		goto fail_alloc_mem;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		result = -ENOENT;
		goto fail_alloc_mem;
	}

	ts->irq = platform_get_irq(pdev, 0);
	if (!ts->irq) {
		dev_err(&pdev->dev, "Could not get IORESOURCE_IRQ\n");
		result = -ENODEV;
		goto fail_alloc_mem;
	}

	ioarea = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "Could not allocate io region\n");
		result = -EBUSY;
		goto fail_alloc_mem;
	}

    //Added for touch calibration++
	if (misc_register(&msm_misc_dev)) {
		printk(KERN_INFO "[TOUCH] misc_register : cannot add misc device!\n");
		//goto err3;
    }
    //Added for touch calibration--

	virt = ioremap(res->start, resource_size(res));
	if (!virt) {
		dev_err(&pdev->dev, "Could not ioremap region\n");
		result = -ENOMEM;
		goto fail_ioremap;
	}

	input_dev->name = TS_DRIVER_NAME;
	input_dev->phys = "msm_touch/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	input_dev->absbit[BIT_WORD(ABS_MISC)] = BIT_MASK(ABS_MISC);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	//Modified for RTP PR1 setting++	
    /* a few more misc keys */
    //__set_bit(KEY_BACK, input_dev->keybit);
    //__set_bit(KEY_KBDILLUMDOWN, input_dev->keybit);
    input_dev->keybit[BIT_WORD(KEY_BACK)] = BIT_MASK(KEY_BACK);
    input_dev->keybit[BIT_WORD(KEY_KBDILLUMDOWN)] = BIT_MASK(KEY_KBDILLUMDOWN);
    //Modified for RTP PR1 setting--
    //Modified for Home (Search) and AP key (2009/08/10)++
    if((FIH_READ_HWID_FROM_SMEM() >= CMCS_RTP_PR2) && (FIH_READ_HWID_FROM_SMEM() <= CMCS_RTP_MP3)){
        input_dev->keybit[BIT_WORD(KEY_HOME)] = BIT_MASK(KEY_HOME);
        //input_dev->keybit[BIT_WORD(KEYCODE_BROWSER)] = BIT_MASK(KEYCODE_BROWSER);
        input_dev->keybit[BIT_WORD(KEYCODE_SEACHER)] = BIT_MASK(KEYCODE_SEACHER);
        set_bit(KEYCODE_BROWSER, input_dev->keybit);
    }
    //Modified for Home (Search) and AP key (2009/08/10)--

	if (pdata) {
		x_max = pdata->x_max ? : X_MAX;
		y_max = pdata->y_max ? : Y_MAX;
		pressure_max = pdata->pressure_max ? : P_MAX;
	} else {
		x_max = X_MAX;
		y_max = Y_MAX;
		pressure_max = P_MAX;
	}

	ts->x_max = 960;
	ts->y_max = 960;

/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change RES touch suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->ts_early_suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 11;
	ts->ts_early_suspend_desc.suspend = ts_early_suspend;
	ts->ts_early_suspend_desc.resume = ts_late_resume;
    printk(KERN_INFO "RES_Touch register_early_suspend()\n");
	register_early_suspend(&ts->ts_early_suspend_desc);
    ts->pdev = pdev;
#endif
#endif
/* } FIH, SimonSSChang, 2009/09/04 */

#ifdef CONFIG_FIH_FXX
    if(FIH_READ_HWID_FROM_SMEM() <= CMCS_RTP_PR1)
    {
	    input_set_abs_params(input_dev, ABS_X, 21, 921, 0, 0);  //Modified by Stanley
	}
	else if(FIH_READ_HWID_FROM_SMEM() >= CMCS_RTP_PR2)
	{
	    input_set_abs_params(input_dev, ABS_X, 205, 733, 0, 0);
	}
	//Modified for RTP PR1 setting, 2009/05/20++
	//Modify HW ID to support larger than PR1++
	if(FIH_READ_HWID_FROM_SMEM() == CMCS_HW_VER_EVB1)
	{
	    input_set_abs_params(input_dev, ABS_Y, 15, 960, 0, 0);
	    bIsEVB = 1;
	    bIsPR = 0;
	    bIsPR2 = 0;
	    bIsPR3 = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	    bIsMP = 0;  //Added for new touch calibration
	}
	//else if((FIH_READ_HWID_FROM_SMEM() >= CMCS_RTP_PR1) && (FIH_READ_HWID_FROM_SMEM() <= CMCS_RTP_MP3))
	else if(FIH_READ_HWID_FROM_SMEM() == CMCS_RTP_PR1)  //Modified for RTP PR2 setting, 2009/07/06
	{
	    input_set_abs_params(input_dev, ABS_Y, 5, VKEY_UP_BOUNDARY, 0, 0);  //Modified by Stanley
	    bIsPR = 1;
	    bIsEVB = 0;
	    bIsPR2 = 0;
	    bIsPR3 = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	    bIsMP = 0;  //Added for new touch calibration
	}
	//Modify HW ID to support larger than PR1--
	//Modified for RTP PR1 setting, 2009/05/20--
	//Modified for RTP PR2 setting, 2009/07/06++
	else if(FIH_READ_HWID_FROM_SMEM() == CMCS_RTP_PR2)
	{
	    input_set_abs_params(input_dev, ABS_Y, 148, VKEY_UP_BOUNDARY_PR2, 0, 0);  //Modified by Stanley
        bIsPR2 = 1;
	    bIsPR = 0;
	    bIsEVB = 0;
	    bIsPR3 = 0;  //Modified for Home (Search) and AP key (2009/08/10)
	    bIsMP = 0;  //Added for new touch calibration
	}
	//Modified for RTP PR2 setting, 2009/07/06--
	//Modified for Home (Search) and AP key (2009/08/10)++
	//else if((FIH_READ_HWID_FROM_SMEM() >= CMCS_RTP_PR3) && (FIH_READ_HWID_FROM_SMEM() <= CMCS_RTP_MP3))
	else if(FIH_READ_HWID_FROM_SMEM() == CMCS_RTP_PR3)  //Added for new touch calibration
	{
	    input_set_abs_params(input_dev, ABS_Y, 148, VKEY_UP_BOUNDARY_PR2, 0, 0);  //Modified by Stanley
        bIsPR3 = 1;  
        bIsPR2 = 0;
	    bIsPR = 0;
	    bIsEVB = 0;
	    bIsMP = 0;  //Added for new touch calibration
	}
	//Modified for Home (Search) and AP key (2009/08/10)--
	//Added for new touch calibration++
	else if((FIH_READ_HWID_FROM_SMEM() >= CMCS_RTP_MP1) && (FIH_READ_HWID_FROM_SMEM() <= CMCS_RTP_MP3))
	{
	    //input_set_abs_params(input_dev, ABS_Y, 148, VKEY_UP_BOUNDARY_PR2, 0, 0);  //Modified by Stanley
        bIsMP = 1;  //Added for new touch calibration
        bIsPR3 = 0;  
        bIsPR2 = 0;
	    bIsPR = 0;
	    bIsEVB = 0;
	}
	//Added for new touch calibration--
#else
	input_set_abs_params(input_dev, ABS_X, 0, x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, y_max, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, pressure_max, 0, 0);

	result = input_register_device(input_dev);
	if (result)
		goto fail_ip_reg;

	ts->input = input_dev;

	setup_timer(&ts->timer, ts_timer, (unsigned long)ts);
	ts->last_x =0;
	ts->last_y =0;
	result = request_irq(ts->irq, ts_interrupt, IRQF_TRIGGER_RISING,
				 "touchscreen", ts);
	if (result)
		goto fail_req_irq;

	platform_set_drvdata(pdev, ts);

	/* Set dafault sampling interval */
	writel(DEFAULT_SI_VALUE, TSSC_REG(SI));  //[FIH_ADQ] Added by Stanley

	//Enable the TSSC
	writel(TSSC_CTL_STATE, TSSC_REG(CTL));  //[FIH_ADQ.B-296]
	
    //Modified for Home (Search) and AP key (2009/08/10)++
    if(bIsPR2 || bIsPR3 || bIsMP)  //Modified for PR3 or later
    {
        if(!gpio_get_value(GPIO_NEG_Y))
        {
            bIsJBallExist = 1;  //F910
            printk(KERN_INFO "[TOUCH-RES]ts_probe(): F910!\r\n");
        }
        else
        {
            bIsJBallExist = 0;  //F902 
            printk(KERN_INFO "[TOUCH-RES]ts_probe(): F902!\r\n");
        }
    }
    //Modified for Home (Search) and AP key (2009/08/10)--

    //Added for new touch calibration++
    //Write boundary value to NV item.
    //data.x_max = 0;
    //data.x_min = 0;
    //data.y_max = 0;
    //data.y_min = 0;
	//memcpy(&iTCHBoundaryItem[1], &data, 8);
	//data.x_max = 0;
    //data.x_min = 0;
    //data.y_max = 0;
    //data.y_min = 0;
	//proc_comm_write_nv(iTCHBoundaryItem);
	//iTCHBoundaryItem[1]=0;
	//iTCHBoundaryItem[2]=0;

    if(!proc_comm_read_nv(iTCHBoundaryItem))
	{
	    memcpy(&data, iTCHBoundaryItem, 8);
	    printk(KERN_INFO "[TOUCH-RES]ts_probe(): x_max = %d, x_min = %d, y_max = %d, y_min = %d.\n", data.x_max, data.x_min, data.y_max, data.y_min);
        if(((data.x_max - data.x_min) > 500) && ((data.y_max - data.y_min) > 500))
        {
	        uX_Max = data.x_max;
            uX_Min = data.x_min;
            uY_Max = data.y_max;
            uY_Min = data.y_min;
            uVkeyUpBoundaryMP = data.y_max;
            bIsReadFromNV = 1;
		
	        if((FIH_READ_HWID_FROM_SMEM() >= CMCS_RTP_MP1) && (FIH_READ_HWID_FROM_SMEM() <= CMCS_RTP_MP3))
            {
	            input_set_abs_params(input_dev, ABS_X, uX_Min, uX_Max, 0, 0);  //Modified by Stanley
	            input_set_abs_params(input_dev, ABS_Y, uY_Min, uY_Max, 0, 0);  //Modified by Stanley
	            printk(KERN_INFO "[TOUCH-RES]ts_probe(): input_set_abs_params done!\n");
	        }
	    }
	    else
	    {
	        bIsReadFromNV = 0;
	        printk(KERN_INFO "[TOUCH-RES]ts_probe(): NV item value invalid!\r\n");
	    }
	}
	else
	    printk(KERN_INFO "[TOUCH-RES]ts_probe(): NV item read failed!\r\n");

	if((FIH_READ_HWID_FROM_SMEM() >= CMCS_RTP_MP1) && (FIH_READ_HWID_FROM_SMEM() <= CMCS_RTP_MP3) && !bIsReadFromNV)
    {
	    uX_Max = 1000;
        uX_Min = 80;
        uY_Max = 850;
        uY_Min = 40;
        uVkeyUpBoundaryMP = 850;
        input_set_abs_params(input_dev, ABS_X, uX_Min, uX_Max, 0, 0);  //Modified by Stanley
	    input_set_abs_params(input_dev, ABS_Y, uY_Min, uY_Max, 0, 0);  //Modified by Stanley
	    printk(KERN_INFO "[TOUCH-RES]ts_probe(): Set default boundary value for MP device!\r\n");
	}    
    #if 0
	uX_Max = data.x_max;
    uX_Min = data.x_min;
    uY_Max = data.y_max;
    uY_Min = data.y_min;
    uVkeyUpBoundaryMP = data.y_max;
		
	if((FIH_READ_HWID_FROM_SMEM() >= CMCS_RTP_MP1) && (FIH_READ_HWID_FROM_SMEM() <= CMCS_RTP_MP3))
    {
	    input_set_abs_params(input_dev, ABS_X, uX_Min, uX_Max, 0, 0);  //Modified by Stanley
	    input_set_abs_params(input_dev, ABS_Y, uY_Min, uY_Max, 0, 0);  //Modified by Stanley
	}
	#endif
	iInterval = (uX_Max - uX_Min) / 25;
	iButtonWidth = ((uX_Max - uX_Min) - (iInterval * 3)) / 4;
	uVkeySoft1MiddleBoundary = uX_Min + iButtonWidth;
	uVkeyHomeLeftBoundary = uVkeySoft1MiddleBoundary + iInterval;
	uVkeyHomeRightBoundary = uVkeyHomeLeftBoundary + iButtonWidth;
	uVkeyApLeftBoundary = uVkeyHomeRightBoundary + iInterval;
	uVkeyApRightBoundary = uVkeyApLeftBoundary + iButtonWidth;
	uVkeySoft2MiddleBoundary = uVkeyApRightBoundary + iInterval;
	printk(KERN_INFO "[TOUCH]ts_probe(): iInterval = %d, iButtonWidth = %d," 
	                  "uVkeySoft1MiddleBoundary = %d, uVkeyHomeLeftBoundary = %d,"
	                  "uVkeyHomeRightBoundary = %d, uVkeyApLeftBoundary = %d,"
	                  "uVkeyApRightBoundary = %d, uVkeySoft2MiddleBoundary = %d.\r\n",
	                  iInterval, iButtonWidth, uVkeySoft1MiddleBoundary, uVkeyHomeLeftBoundary,
	                  uVkeyHomeRightBoundary, uVkeyApLeftBoundary, uVkeyApRightBoundary, uVkeySoft2MiddleBoundary);    
	//Added for new touch calibration--
	//Setting the configuration of GPIO 89++
	gpio_tlmm_config(GPIO_CFG(89, 0, GPIO_CFG_INPUT,
						GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	//Setting the configuration of GPIO 89--    
	printk(KERN_INFO "[TOUCH-RES]ts_probe() init ok!\r\n");	

	return 0;

fail_req_irq:
	input_unregister_device(input_dev);
	input_dev = NULL;
fail_ip_reg:
	iounmap(virt);
fail_ioremap:
	release_mem_region(res->start, resource_size(res));
fail_alloc_mem:
	input_free_device(input_dev);
	kfree(ts);
	return result;
}

static int __devexit ts_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ts *ts = platform_get_drvdata(pdev);

/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change RES touch suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
#ifdef CONFIG_HAS_EARLYSUSPEND
    printk(KERN_INFO "RES_Touch unregister_early_suspend()\n");
	unregister_early_suspend(&ts->ts_early_suspend_desc);
#endif
#endif
/* } FIH, SimonSSChang, 2009/09/04 */

	free_irq(ts->irq, ts);
	del_timer_sync(&ts->timer);

	input_unregister_device(ts->input);
	iounmap(virt);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
	platform_set_drvdata(pdev, NULL);
	kfree(ts);

	return 0;
}

static struct platform_driver ts_driver = {
	.probe		= ts_probe,
	.remove		= __devexit_p(ts_remove),
/* FIH, SimonSSChang, 2009/09/04 { */
/* [FXX_CR], change RES touch suspend/resume function
to earlysuspend */
#ifdef CONFIG_FIH_FXX
	//.suspend    = ts_suspend,  //[FIH_ADQ.B-296]
	//.resume     = ts_resume,  //[FIH_ADQ.B-296]
#else
	.suspend    = ts_suspend,  //[FIH_ADQ.B-296]
	.resume     = ts_resume,  //[FIH_ADQ.B-296]
#endif	
/* } FIH, SimonSSChang, 2009/09/04 */
	.driver		= {
		.name = TS_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ts_init(void)
{
    //Dynamic to load RES or CAP touch driver++
    if((FIH_READ_HWID_FROM_SMEM() >= CMCS_HW_VER_EVB1) && (FIH_READ_HWID_FROM_SMEM() <= CMCS_RTP_MP3))  //Modify HW ID to support larger than PR1
    {
	    return platform_driver_register(&ts_driver);
	}
	else
	    return -ENODEV;
	//Dynamic to load RES or CAP touch driver--
}
module_init(ts_init);

static void __exit ts_exit(void)
{
	platform_driver_unregister(&ts_driver);
}
module_exit(ts_exit);

MODULE_DESCRIPTION("MSM Touch Screen driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("QUALCOMM-QCT");
MODULE_ALIAS("platform:msm_touch");
