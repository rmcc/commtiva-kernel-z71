/*
 *     tca6507.c - TCA6507 LED EXPANDER for BACKLIGHT and LEDs CONTROLLER
 *
 *     Copyright (C) 2009 Audi PC Huang <audipchuang@fihtdc.com>
 *     Copyright (C) 2008 FIH CO., Inc.
 *
 *     This program is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <mach/tca6507.h>
#include <mach/gpio.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>

/*FIH_FTM, PinyCHWu, 2009/06/15 {*/
/*Add misc device*/
#ifdef CONFIG_FIH_FXX
#include <linux/miscdevice.h>
#include <asm/ioctl.h>
#include <asm/fcntl.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#endif
/*} FIH_FTM, PinyCHWu, 2009/06/15*/

extern void Battery_power_supply_change(void);
#define	GPIO_CHG_LED_EN	30
struct tca6507_driver_data {
	struct mutex tca6507_lock;
	struct i2c_client *tca6507_i2c_client;
	unsigned int reset_pin;
	unsigned int HWID;
	u16 led_state[7];
	u16 prev_led_state[7];
	bool is_fade_on[7];
	bool is_blinking;
	bool is_charging;
	bool is_suspend;
	bool is_notify_blink;
	int LEDnum;
	/*FIH, MichaelKao, 2009/07/07 {*/
	/*Add a Jogball enable function for Jogball driver*/
	bool Jogball_enable;
	/*FIH, MichaelKao, 2009/07/07 {*/
};

static struct tca6507_driver_data tca6507_drvdata;
static int gCharger_state = CHARGER_STATE_UNKNOWN;
static bool last_led_state[7];

enum {
	GPIO_LOW = 0,
	GPIO_HIGH
};
/* FIH, Michael Kao, 2010/05/14{ */
/* [FXX_CR], add for not to disable charger*/
int charger_on;
/* FIH, Michael Kao, 2010/05/14{ */


/*************I2C functions*******************/
static int tca6507_read( struct i2c_client *client, u8 *rxdata, int length )
{
	struct i2c_msg msgs[] =
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &rxdata[0],
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = &rxdata[1],
		},
	};
	
	int ret;
	
	ret = i2c_transfer( client->adapter, msgs, 2 );
	if ( 0 > ret ) {
		return -EIO;
	}

	return 0;
}

static int tca6507_write( struct i2c_client *client, u8 *txdata, int length )
{
	struct i2c_msg msg[] =
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	
	int ret;
	
	ret = i2c_transfer( client->adapter, msg, 1 );
	if ( 0 > ret ) {
		return -EIO;
	}

	return 0;
}

/*****************gpio function****************/
static int gpio_init_setting(int value)
{
	struct device *tca6507_dev = &tca6507_drvdata.tca6507_i2c_client->dev;
  	int ret = 0;
  	
	gpio_tlmm_config(GPIO_CFG(tca6507_drvdata.reset_pin, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
	
	//request gpio
	ret = gpio_request(tca6507_drvdata.reset_pin, "tca6507_reset_pin");
	if(ret) {
	  	dev_err(tca6507_dev ,"%s: GPIO-%d request failed\n", __func__, tca6507_drvdata.reset_pin);
		return ret;
	}
	
	ret = gpio_direction_output(tca6507_drvdata.reset_pin, value);
	if(ret) {
	  	dev_err(tca6507_dev, "%s: GPIO-%d gpio_direction_output failed\n", __func__, tca6507_drvdata.reset_pin);
		return ret;
	}

	return 0;
}

static void gpio_release(void)
{
	gpio_free( tca6507_drvdata.reset_pin );
}

void tca6507_led_turn_on(int led_selection, uint8_t *cmd)
{		
	if ((tca6507_drvdata.HWID >= CMCS_HW_VER_EVB1) &&(tca6507_drvdata.HWID <= CMCS_CTP_MP3) )
{		
	switch (led_selection) {
	case TCA6507_LED_RFB:
		cmd[1] &= ~0x04;
		cmd[2] |= 0x04;
		cmd[3] &= ~0x04;
		
		break;
	
	case TCA6507_LED_GFB:
			cmd[1] &= ~0x08;
			cmd[2] |= 0x08;
			cmd[3] &= ~0x08;
		
		break;
	
	case TCA6507_LED_KFB:
		cmd[1] &= ~0x10;
		cmd[2] |= 0x10;
		cmd[3] &= ~0x10;
		break;
	case TCA6507_LED_R1:
		cmd[1] &= ~0x20;
		cmd[2] |= 0x20;
		cmd[3] &= ~0x20;
		break;
	case TCA6507_LED_YG2:
		cmd[1] &= ~0x40;
		cmd[2] |= 0x40;
		cmd[3] &= ~0x40;	
		break;
		}
		}else if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
		{
			switch (led_selection) {
			case TCA6507_LED_RFB:
				cmd[1] &= ~0x02;
				cmd[2] |= 0x02;
				cmd[3] &= ~0x02;
				break;
			case TCA6507_LED_KFB3:	
				cmd[1] &= ~0x04;
				cmd[2] |= 0x04;
				cmd[3] &= ~0x04;
				break;
			case TCA6507_LED_KFB2:	
				cmd[1] &= ~0x08;
				cmd[2] |= 0x08;
				cmd[3] &= ~0x08;
				break;
			case TCA6507_LED_KFB:
				cmd[1] &= ~0x10;
				cmd[2] |= 0x10;
				cmd[3] &= ~0x10;
				break;
			case TCA6507_LED_R1:
				cmd[1] &= ~0x20;
				cmd[2] |= 0x20;
				cmd[3] &= ~0x20;
				break;
			case TCA6507_LED_YG2:
				cmd[1] &= ~0x40;
				cmd[2] |= 0x40;
				cmd[3] &= ~0x40;
		}
	}
}
void tca6507_led_turn_off(int led_selection, uint8_t *cmd)
{		
	if ((tca6507_drvdata.HWID >= CMCS_HW_VER_EVB1)&&(tca6507_drvdata.HWID <= CMCS_CTP_MP3) )
{		
	switch (led_selection) {
	case TCA6507_LED_RFB:
		cmd[1] &= ~0x04;
		cmd[2] &= ~0x04;
		cmd[3] &= ~0x04;
		
		break;
	
	case TCA6507_LED_GFB:
		cmd[1] &= ~0x08;
		cmd[2] &= ~0x08;
		cmd[3] &= ~0x08;
		
		break;
	case TCA6507_LED_KFB:
		cmd[1] &= ~0x10;
		cmd[2] &= ~0x10;
		cmd[3] &= ~0x10;
		break;
		
	case TCA6507_LED_R1:
		cmd[1] &= ~0x20;
		cmd[2] &= ~0x20;
		cmd[3] &= ~0x20;
		break;
	
	case TCA6507_LED_YG2:
		cmd[1] &= ~0x40;
		cmd[2] &= ~0x40;
		cmd[3] &= ~0x40;	
		break;
	}
	}else if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
	{
		switch (led_selection) {
		case TCA6507_LED_RFB:
			cmd[1] &= ~0x02;
			cmd[2] &= ~0x02;
			cmd[3] &= ~0x02;
			break;
		case TCA6507_LED_KFB3:	
			cmd[1] &= ~0x04;
			cmd[2] &= ~0x04;
			cmd[3] &= ~0x04;
			break;
		case TCA6507_LED_KFB2:	
			cmd[1] &= ~0x08;
			cmd[2] &= ~0x08;
			cmd[3] &= ~0x08;
			break;
		case TCA6507_LED_KFB:
			cmd[1] &= ~0x10;
			cmd[2] &= ~0x10;
			cmd[3] &= ~0x10;
			break;
		case TCA6507_LED_R1:
			cmd[1] &= ~0x20;
			cmd[2] &= ~0x20;
			cmd[3] &= ~0x20;
			break;
		case TCA6507_LED_YG2:
			cmd[1] &= ~0x40;
			cmd[2] &= ~0x40;
			cmd[3] &= ~0x40;	
		}
	}
}

void tca6507_led_blink(int led_selection, uint8_t *cmd)
{
	if ((tca6507_drvdata.HWID >= CMCS_HW_VER_EVB1)&&(tca6507_drvdata.HWID <= CMCS_CTP_MP3) )
	{
	switch (led_selection) {
	case TCA6507_LED_RFB:
		cmd[1] |= 0x04;
		cmd[2] |= 0x04;
		cmd[3] |= 0x04;
		
		break;
		
	case TCA6507_LED_GFB:
		cmd[1] |= 0x08;
		cmd[2] |= 0x08;
		cmd[3] |= 0x08;
		
		break;
	
	case TCA6507_LED_KFB:
		//if (tca6507_drvdata.HWID < CMCS_HW_VER_DVT1) {
			//cmd[1] |= (0x08|0x10|0x20);
			//cmd[2] |= (0x08|0x10|0x20);
			//cmd[3] |= (0x08|0x10|0x20);
		//} else {
            		//cmd[1] |= 0x08;
            		//cmd[2] |= 0x08;
		cmd[1] |= 0x10;
            	cmd[2] |= 0x10;
            	cmd[3] |= 0x10;
				
		break;
	
	case TCA6507_LED_R1:
		cmd[1] |= 0x20;
		cmd[2] |= 0x20;
		cmd[3] |= 0x20;
		break;
		
	case TCA6507_LED_YG2:
		cmd[1] |= 0x40;
		cmd[2] |= 0x40;
		cmd[3] |= 0x40;	
	}
	}else if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
	{
		switch (led_selection) {
		case TCA6507_LED_RFB:
			cmd[1] |= 0x02;
			cmd[2] |= 0x02;
			cmd[3] |= 0x02;
			break;
		case TCA6507_LED_KFB3:	
			cmd[1] |= 0x04;
			cmd[2] |= 0x04;
			cmd[3] |= 0x04;
			break;
		case TCA6507_LED_KFB2:	
			cmd[1] |= 0x08;
			cmd[2] |= 0x08;
			cmd[3] |= 0x08;
			break;
		case TCA6507_LED_KFB:
			cmd[1] |= 0x10;
			cmd[2] |= 0x10;
			cmd[3] |= 0x10;
			break;
		case TCA6507_LED_R1:
			cmd[1] |= 0x20;
			cmd[2] |= 0x20;
			cmd[3] |= 0x20;
			break;
		case TCA6507_LED_YG2:
			cmd[1] |= 0x40;
			cmd[2] |= 0x40;
			cmd[3] |= 0x40;	
		}
	}
}
void tca6507_led_fade_on(int led_selection, uint8_t *cmd)
{
	if ((tca6507_drvdata.HWID >= CMCS_HW_VER_EVB1) &&(tca6507_drvdata.HWID <= CMCS_CTP_MP3) )
	{
	//tca6507_led_blink(led_selection, cmd);
	switch (led_selection) {
	case TCA6507_LED_RFB:
		cmd[1] &= ~0x04;
		cmd[2] |= 0x04;
		cmd[3] |= 0x04;
		break;
	case TCA6507_LED_GFB:
		cmd[1]&= ~0x08;
		cmd[2] |= 0x08;
		cmd[3] |= 0x08;
		break;
	case TCA6507_LED_KFB:
			cmd[1] &= ~0x10;
            		cmd[2] |= 0x10;
            		cmd[3] |= 0x10;
			break;
		case TCA6507_LED_R1:
			cmd[1] &= ~ 0x20;
			cmd[2] |= 0x20;
			cmd[3] |= 0x20;
			break;
		case TCA6507_LED_YG2:
			cmd[1]&= ~0x40;
			cmd[2] |= 0x40;
			cmd[3] |= 0x40;
		}
	}else if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
	{
		switch (led_selection) {
		case TCA6507_LED_RFB:
			cmd[1] &= ~0x02;
			cmd[2] |= 0x02;
			cmd[3] |= 0x02;
			break;
		case TCA6507_LED_KFB3:	
			cmd[1] &= ~0x04;
			cmd[2] |= 0x04;
			cmd[3] |= 0x04;
			break;
		case TCA6507_LED_KFB2:	
			cmd[1] &= ~0x08;
			cmd[2] |= 0x08;
			cmd[3] |= 0x08;
			break;

		case TCA6507_LED_KFB:
		cmd[1] &= ~0x10;
            	cmd[2] |= 0x10;
            	cmd[3] |= 0x10;
			break;
		case TCA6507_LED_R1:
			cmd[1] &= ~0x20;
			cmd[2] |= 0x20;
			cmd[3] |= 0x20;
			break;
		case TCA6507_LED_YG2:
			cmd[1] &= ~0x40;
			cmd[2] |= 0x40;
			cmd[3] |= 0x40;	
		}
	}
	cmd[11] = 0x88;
}

void tca6507_led_fade_off(int led_selection, uint8_t *cmd)
{
	if ((tca6507_drvdata.HWID >= CMCS_HW_VER_EVB1) &&(tca6507_drvdata.HWID <= CMCS_CTP_MP3) )
	{
	//tca6507_led_blink(led_selection, cmd);
	switch (led_selection) {
	case TCA6507_LED_RFB:
		cmd[1] &= ~0x04;
		cmd[2] |= 0x04;
		cmd[3] |= 0x04;
		break;
	case TCA6507_LED_GFB:
		cmd[1]&= ~0x08;
		cmd[2] |= 0x08;
		cmd[3] |= 0x08;
		break;
	case TCA6507_LED_KFB:
			cmd[1] &= ~0x10;
            		cmd[2] |= 0x10;
            		cmd[3] |= 0x10;
			break;
		case TCA6507_LED_R1:
			cmd[1] &= ~ 0x20;
			cmd[2] |= 0x20;
			cmd[3] |= 0x20;
			break;
		case TCA6507_LED_YG2:
			cmd[1]&= ~0x40;
			cmd[2] |= 0x40;
			cmd[3] |= 0x40;
	}
	}else if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
	{
		switch (led_selection) {
		case TCA6507_LED_RFB:
			cmd[1] &= ~0x02;
			cmd[2] |= 0x02;
			cmd[3] |= 0x02;
			break;
		case TCA6507_LED_KFB3:	
			cmd[1] &= ~0x04;
			cmd[2] |= 0x04;
			cmd[3] |= 0x04;
			break;
		case TCA6507_LED_KFB2:	
			cmd[1] &= ~0x08;
			cmd[2] |= 0x08;
			cmd[3] |= 0x08;
			break;
		case TCA6507_LED_KFB:
		cmd[1] &= ~0x10;
            	cmd[2] |= 0x10;
            	cmd[3] |= 0x10;
			break;
		case TCA6507_LED_R1:
			cmd[1] &= ~0x20;
			cmd[2] |= 0x20;
			cmd[3] |= 0x20;
			break;
		case TCA6507_LED_YG2:
			cmd[1] &= ~0x40;
			cmd[2] |= 0x40;
			cmd[3] |= 0x40;	
		}
	}
	cmd[11] = 0xAA;
}

int tca6507_get_state(int led_selection)
{
	struct device *tca6507_dev = &tca6507_drvdata.tca6507_i2c_client->dev;
	int i;
	
	for (i = JOGBALL_DISABLE; i > TCA6507_ABNORMAL_STATE; i--) {
		if (tca6507_drvdata.led_state[led_selection] & (1 << i)) {
			dev_dbg(tca6507_dev, "%s: LED<%d> in <%d> State!!\n", __func__, led_selection, i);
			return i;
		}
	}
	
	return i;
}

static int tca6507_set_led(void)
{
	struct device *tca6507_dev = &tca6507_drvdata.tca6507_i2c_client->dev;
	u8 cmd[12] = {TCA6507_INITIALIZATION_REGISTER, 0x00, 0x02, 0x00, 0x77, 0x44, 0x77, 0xBB, 0xBB, 0xFF, 0xCF, 0x88};
	u8 cmd_fade_off[12] = {TCA6507_INITIALIZATION_REGISTER, 0x00, 0x02, 0x00, 0x77, 0x44, 0x77, 0xBB, 0xBB, 0xFF, 0xCF, 0xAA};
	int ret = -1;
	int i, state[tca6507_drvdata.LEDnum];
	bool is_blinking = false;
	bool is_fade_off = false;
	bool is_fade_on = false;
	bool is_charging=false;
	//u8 loc_buf[4096];
	if ((tca6507_drvdata.HWID >= CMCS_HW_VER_EVB1) &&(tca6507_drvdata.HWID <= CMCS_CTP_MP3) )
	{
		cmd[2]=0x02;
		cmd_fade_off[2]=0x02;
	}
	else if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
	{
		cmd[2]=0x01;
		cmd_fade_off[2]=0x01;
	}
	dev_dbg(tca6507_dev, "%s\n", __func__);

	for (i = 0; i < tca6507_drvdata.LEDnum; i++) {
		state[i] = tca6507_get_state(i);

		switch (state[i]) {		
		case TCA6507_LED_OFF:
			if (TCA6507_LED_PREV_STATE_ON == tca6507_drvdata.prev_led_state[i]) {
				is_fade_off = true;
			}
			break;
		case TCA6507_LED_BLINK:
		case CHARGER_STATE_LOW_POWER:
			is_blinking = true;
			cmd[10] = 0x4F;

			break;
		case CHARGER_STATE_CHARGING:
			is_charging=true;
		case CHARGER_STATE_DISCHARGING:
		case TCA6507_LED_ON:
		case CHARGER_STATE_UNKNOWN:
		case CHARGER_STATE_NOT_CHARGING:
		case CHARGER_STATE_FULL:
		/*FIH, MichaelKao, 2009/07/07 {*/
		/*Add a Jogball enable function for Jogball driver*/
		case JOGBALL_ENABLE:
		case JOGBALL_DISABLE:
		/*FIH, MichaelKao, 2009/07/07 {*/
			if (TCA6507_LED_PREV_STATE_OFF == tca6507_drvdata.prev_led_state[i]) {
				//cmd[10] = 0xCF;
				is_fade_on = true;			
			}

			break;
		default:
			dev_err(tca6507_dev, "%s: ERROR LED STATE!!\n", __func__);
		}
	}
	
	tca6507_drvdata.is_charging = is_charging;
	tca6507_drvdata.is_blinking = is_blinking;
        
	for (i = 0; i < tca6507_drvdata.LEDnum; i++) {
		switch (state[i]) {
		case TCA6507_LED_OFF:
			if (TCA6507_LED_PREV_STATE_ON == tca6507_drvdata.prev_led_state[i]) {
				if (is_fade_off && is_fade_on) {
					cmd[10] = 0x4F;
					tca6507_led_fade_off(i, cmd_fade_off);
					tca6507_led_turn_off(i, cmd);	
				} else {
					//tca6507_led_fade_off(i, cmd_fade_off);
					tca6507_led_fade_off(i, cmd);
				}
			} else {
				tca6507_led_turn_off(i, cmd);			
			}
			
			tca6507_drvdata.prev_led_state[i] = TCA6507_LED_PREV_STATE_OFF;

			break;
		case TCA6507_LED_BLINK:
		case CHARGER_STATE_LOW_POWER:
			tca6507_led_blink(i, cmd);
			
			tca6507_drvdata.prev_led_state[i] = TCA6507_LED_PREV_STATE_BLINK;
			break;
		case CHARGER_STATE_DISCHARGING:
		case TCA6507_LED_ON:
		case CHARGER_STATE_UNKNOWN:
		case CHARGER_STATE_CHARGING:
		case CHARGER_STATE_NOT_CHARGING:
		case CHARGER_STATE_FULL:
		/*FIH, MichaelKao, 2009/07/07 {*/
		/*Add a Jogball enable function for Jogball driver*/
		case JOGBALL_ENABLE:
		case JOGBALL_DISABLE:
		/*FIH, MichaelKao, 2009/07/07 {*/
			if (TCA6507_LED_PREV_STATE_OFF == tca6507_drvdata.prev_led_state[i]) {
				if (is_fade_off && is_fade_on)
					tca6507_led_turn_off(i, cmd);
					
				tca6507_led_fade_on(i, cmd);		
				
			} else {
				tca6507_led_turn_on(i, cmd);	
				
			}
			
			tca6507_drvdata.prev_led_state[i] = TCA6507_LED_PREV_STATE_ON;

			break;
		default:
			dev_err(tca6507_dev, "%s: ERROR LED STATE!!\n", __func__);
		}
	}
	
	if (is_fade_off && is_fade_on) {
		ret = tca6507_write(tca6507_drvdata.tca6507_i2c_client, cmd_fade_off, sizeof(cmd_fade_off));
		if (ret < 0) {
			dev_err(tca6507_dev, "%s: i2c_write failed<cmd_fade_off>!!\n", __func__);
		}
		mdelay(800);
	}
	ret = tca6507_write(tca6507_drvdata.tca6507_i2c_client, cmd, sizeof(cmd));
	if (ret < 0) {
		dev_err(tca6507_dev, "%s: i2c_write failed!!\n", __func__);
	}
		//bytes_read += sprintf(&loc_buf[bytes_read], "[0x%02x] 0x%02x\n", i, cmd2[i]);
	//}
	return ret;
}

void tca6507_charger_state_report(int state) 
{
	struct device *tca6507_dev = &tca6507_drvdata.tca6507_i2c_client->dev;
	int i;
	
	switch (state) {
	case CHARGER_STATE_UNKNOWN2:		//RED
		gCharger_state = CHARGER_STATE_UNKNOWN;
		break;
	case CHARGER_STATE_CHARGING2:		//RED
		gCharger_state = CHARGER_STATE_CHARGING;
		break;
	case CHARGER_STATE_DISCHARGING2:	//DNP
		gCharger_state = CHARGER_STATE_DISCHARGING;
		break;
	case CHARGER_STATE_NOT_CHARGING2:	//ALL LEDS OFF
		gCharger_state = CHARGER_STATE_NOT_CHARGING;
		break;
	case CHARGER_STATE_FULL2:		//GREEN	
		gCharger_state = CHARGER_STATE_FULL;
		break;
	case CHARGER_STATE_LOW_POWER2:		//RED BLINK
		gCharger_state = CHARGER_STATE_LOW_POWER;
	}

	mutex_lock(&tca6507_drvdata.tca6507_lock);
	for (i = 0; i <tca6507_drvdata.LEDnum; i++) {
		tca6507_drvdata.led_state[i] &= 0x0C07;
	}
	
	dev_dbg(tca6507_dev, "%s: CHARGER STATE <%d>!!\n", __func__, state);
	if ((tca6507_drvdata.HWID >= CMCS_HW_VER_EVB1) &&(tca6507_drvdata.HWID <= CMCS_CTP_MP3) )
	{
	switch (gCharger_state) {
	case CHARGER_STATE_UNKNOWN:		//RED
		tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 1 << CHARGER_STATE_UNKNOWN;
		tca6507_drvdata.led_state[TCA6507_LED_GFB]	|= 0 << CHARGER_STATE_UNKNOWN;
		tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0 << CHARGER_STATE_UNKNOWN;
		break;
	case CHARGER_STATE_CHARGING:		//RED
		tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 1 << CHARGER_STATE_CHARGING;
		tca6507_drvdata.led_state[TCA6507_LED_GFB]	|= 0 << CHARGER_STATE_CHARGING;
		tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0 << CHARGER_STATE_CHARGING;
		break;
	case CHARGER_STATE_DISCHARGING:		//DNP
		tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 0 << CHARGER_STATE_DISCHARGING;
		tca6507_drvdata.led_state[TCA6507_LED_GFB]	|= 0 << CHARGER_STATE_DISCHARGING;
		tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0 << CHARGER_STATE_DISCHARGING;
		break;
	case CHARGER_STATE_NOT_CHARGING:	//RED & GREEN
		tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 0 << CHARGER_STATE_NOT_CHARGING;
		tca6507_drvdata.led_state[TCA6507_LED_GFB]	|= 0 << CHARGER_STATE_NOT_CHARGING;
		tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0 << CHARGER_STATE_NOT_CHARGING;
		break;
	case CHARGER_STATE_FULL:		//GREEN
		tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 0 << CHARGER_STATE_FULL;
		tca6507_drvdata.led_state[TCA6507_LED_GFB]	|= 1 << CHARGER_STATE_FULL;
		tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0 << CHARGER_STATE_FULL;
		break;
	case CHARGER_STATE_LOW_POWER:		//BLINK RED
		tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 1 << CHARGER_STATE_LOW_POWER;
		tca6507_drvdata.led_state[TCA6507_LED_GFB]	|= 0 << CHARGER_STATE_LOW_POWER;
		tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0<< CHARGER_STATE_LOW_POWER;
	}
	}else if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
	{
		switch (gCharger_state) {
		case CHARGER_STATE_UNKNOWN: 	//RED
			tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 1 << CHARGER_STATE_UNKNOWN;
			tca6507_drvdata.led_state[TCA6507_LED_KFB3]	|= 0 << CHARGER_STATE_UNKNOWN;
			tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0 << CHARGER_STATE_UNKNOWN;
			break;
		case CHARGER_STATE_CHARGING:		//RED
			tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 1 << CHARGER_STATE_CHARGING;
			tca6507_drvdata.led_state[TCA6507_LED_KFB3]	|= 0 << CHARGER_STATE_CHARGING;
			tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0 << CHARGER_STATE_CHARGING;
			break;
		case CHARGER_STATE_DISCHARGING: 	//DNP
			tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 0 << CHARGER_STATE_DISCHARGING;
			tca6507_drvdata.led_state[TCA6507_LED_KFB3]	|= 0 << CHARGER_STATE_DISCHARGING;
			tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0 << CHARGER_STATE_DISCHARGING;
			break;
		case CHARGER_STATE_NOT_CHARGING:	//RED & GREEN
			tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 0 << CHARGER_STATE_NOT_CHARGING;
			tca6507_drvdata.led_state[TCA6507_LED_KFB3]	|= 0 << CHARGER_STATE_NOT_CHARGING;
			tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0 << CHARGER_STATE_NOT_CHARGING;
			break;
		case CHARGER_STATE_FULL:		//GREEN
			tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 0 << CHARGER_STATE_FULL;
			tca6507_drvdata.led_state[TCA6507_LED_KFB3]	|= 1 << CHARGER_STATE_FULL;
			tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0 << CHARGER_STATE_FULL;
			break;
		case CHARGER_STATE_LOW_POWER:		//BLINK RED
			tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 1 << CHARGER_STATE_LOW_POWER;
			tca6507_drvdata.led_state[TCA6507_LED_KFB3]	|= 0 << CHARGER_STATE_LOW_POWER;
			tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 0<< CHARGER_STATE_LOW_POWER;

		}
	}
	tca6507_set_led();

	mutex_unlock(&tca6507_drvdata.tca6507_lock);
}
EXPORT_SYMBOL(tca6507_charger_state_report);

void tca6507_led_switch(bool on, int index)
{
	int i;
	
	mutex_lock(&tca6507_drvdata.tca6507_lock);
	
	last_led_state[index] = on;
	for (i = 0; i <tca6507_drvdata.LEDnum; i++) {
		tca6507_drvdata.led_state[i] &= 0xFFFC;
		tca6507_drvdata.led_state[i] |= 1 << (last_led_state[i] ? TCA6507_LED_ON : TCA6507_LED_OFF);
	}
	if ((tca6507_drvdata.HWID >= CMCS_HW_VER_EVB1) &&(tca6507_drvdata.HWID <= CMCS_CTP_MP3) )
	{
		if((index==8)||(index==9))
		{
			tca6507_drvdata.led_state[TCA6507_LED_RFB] |= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);
			tca6507_drvdata.led_state[TCA6507_LED_GFB]	|= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);
			if(!tca6507_drvdata.Jogball_enable)
				tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);		

		}
		if(index==9)
		{
			tca6507_drvdata.led_state[TCA6507_LED_R1]	|= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);
			tca6507_drvdata.led_state[TCA6507_LED_YG2]	|= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);		
		}
	}else if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
	{
		if((index==8)||(index==9))
		{
			tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);
			tca6507_drvdata.led_state[TCA6507_LED_KFB3]  |= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);
			tca6507_drvdata.led_state[TCA6507_LED_KFB2]	 |= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);
			if(!tca6507_drvdata.Jogball_enable)
				tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);		
		}
		if(index==9)
		{
			//tca6507_drvdata.led_state[TCA6507_LED_RFB]	|= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);
			tca6507_drvdata.led_state[TCA6507_LED_R1]	|= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);
			tca6507_drvdata.led_state[TCA6507_LED_YG2]	|= 1 << (on ? TCA6507_LED_ON : TCA6507_LED_OFF);		
		}
	}
	
	tca6507_set_led();
	
	mutex_unlock(&tca6507_drvdata.tca6507_lock);	
	
}
EXPORT_SYMBOL(tca6507_led_switch);
/*FIH, MichaelKao, 2009/08/12 {*/
/*Add a Jogball function for notice LED driver*/
void tca6507_Jogball(bool jogball)
{
	tca6507_drvdata.Jogball_enable=jogball;

}
EXPORT_SYMBOL(tca6507_Jogball);
/*FIH, MichaelKao, 2009/08/12 {*/

/*FIH, MichaelKao, 2009/07/07 {*/
/*Add a Jogball enable function for Jogball driver*/
void tca6507_Jogball_enable(bool enable)
{
	int i;
	mutex_lock(&tca6507_drvdata.tca6507_lock);
	
	for (i = 0; i <tca6507_drvdata.LEDnum; i++) {
		tca6507_drvdata.led_state[i] &= 0x03FF;//Michael modify 10/28
	}
	if(enable)
	{
		tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 1<< JOGBALL_ENABLE;
	}	
	else
	{
		/*FIH, MichaelKao, 2009/11/11 {*/
		/*Solve center key always on bug*/
		tca6507_drvdata.led_state[TCA6507_LED_KFB]	|= 1<<  TCA6507_LED_OFF;
		/*FIH, MichaelKao, 2009/11/11 {*/
	}
	mutex_unlock(&tca6507_drvdata.tca6507_lock);	
	tca6507_set_led();
}
EXPORT_SYMBOL(tca6507_Jogball_enable);
void tca6507_notify_blink(bool blink)
{
	/*FIH, MichaelKao, 2009/06/25 {*/
	/*Add notify blink function for suspend resume*/
	printk("[tca6507_notify_blink]blink = %d, tca6507_drvdata.is_suspend=%d\r\n",blink,tca6507_drvdata.is_suspend);
	tca6507_drvdata.led_state[TCA6507_LED_R1]	&= 0xFFFA;
	tca6507_drvdata.led_state[TCA6507_LED_YG2]	&= 0xFFFA;
	tca6507_drvdata.is_notify_blink=blink;
	mutex_lock(&tca6507_drvdata.tca6507_lock);
	if(blink)
	{		
		tca6507_drvdata.led_state[TCA6507_LED_R1] |= 1 << TCA6507_LED_BLINK;
		tca6507_drvdata.led_state[TCA6507_LED_YG2] |= 1 << TCA6507_LED_OFF;
	}
	#if 1	// +++ For F917, disable the green LED of power key
	else if(tca6507_drvdata.is_suspend)
	{
    	unsigned int orig_hwid = fih_read_orig_hwid_from_smem();
	    if (orig_hwid >= CMCS_CTP_F917_PR1 && orig_hwid <= CMCS_CTP_F917_MP3)
        {
            // do nothing
        }
        else
        {
   	        tca6507_drvdata.led_state[TCA6507_LED_R1] |= 1 << TCA6507_LED_OFF;
            tca6507_drvdata.led_state[TCA6507_LED_YG2] |= 1 << TCA6507_LED_BLINK;
        }
	}
	#endif	// --- For F917, disable the green LED of power key
	else	{		
		tca6507_drvdata.led_state[TCA6507_LED_R1] |= 1 << TCA6507_LED_OFF;
		tca6507_drvdata.led_state[TCA6507_LED_YG2] |= 1 << TCA6507_LED_OFF;
	}
	/*FIH, MichaelKao, 2009/06/25 {*/
	tca6507_set_led();

	mutex_unlock(&tca6507_drvdata.tca6507_lock);	
}
EXPORT_SYMBOL(tca6507_notify_blink);

#if 0
static void tca6507_hw_reset(void)
{
	gpio_set_value( tca6507_drvdata.reset_pin, GPIO_LOW );
	udelay(60);
	gpio_set_value( tca6507_drvdata.reset_pin, GPIO_HIGH );
}
#endif

static int tca6507_led_init(void)
{
	uint8_t cmd[12];
	struct device *tca6507_dev = &tca6507_drvdata.tca6507_i2c_client->dev;
	int ret = 0;
	#if 0 
	if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
	{
		gpio_tlmm_config( GPIO_CFG(GPIO_CHG_LED_EN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA ), GPIO_ENABLE );
		gpio_direction_output(GPIO_CHG_LED_EN,0);
	}
	#endif
	//hw reset was done by max7302 driver
	//tca6507_hw_reset();
	gpio_set_value( tca6507_drvdata.reset_pin, GPIO_HIGH );

	mutex_lock(&tca6507_drvdata.tca6507_lock);
	
	cmd[0] = TCA6507_INITIALIZATION_REGISTER;

	cmd[1] = 0x00;  //Select 0
	if ((tca6507_drvdata.HWID >= CMCS_HW_VER_EVB1) &&(tca6507_drvdata.HWID <= CMCS_CTP_MP3) )
		cmd[2] = 0x02;  //Select 1
	else if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
		cmd[2] = 0x01;  //Select 1
	cmd[3] = 0x00;  //Select 2
   
	//Fade On timer
	cmd[4] = 0x77;

   	 //Fully On timer
	cmd[5] = 0x44;

	//Fade Off timer
	cmd[6] = 0x77;

	//Fully Off timer 1
	cmd[7] = 0xBB;

	//Fully Off timer 2
	cmd[8] = 0xBB;

	cmd[9] = 0xFF;
	cmd[10] = 0xCF;
	cmd[11] = 0x88;
		
	ret = tca6507_write(tca6507_drvdata.tca6507_i2c_client, cmd, sizeof(cmd));
	
	mutex_unlock(&tca6507_drvdata.tca6507_lock);

	if (ret < 0) {
		dev_err(tca6507_dev, "%s: i2c_write failed!!\n", __func__);
		return ret;
	}

	return ret;
}
static ssize_t tca6507_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/*struct i2c_client *client =
		container_of(dev, struct i2c_client, dev);*/
	u8 loc_buf[512];
	u8 cmd[12];
	int bytes_read = 0;
	int i;
	
	cmd[0] = TCA6507_INITIALIZATION_REGISTER;

	mutex_lock(&tca6507_drvdata.tca6507_lock);		
		tca6507_read(tca6507_drvdata.tca6507_i2c_client, cmd, sizeof(cmd) - 1);
	mutex_unlock(&tca6507_drvdata.tca6507_lock);
		
	for (i = 0; i < 12; i++) {
		bytes_read += sprintf(&loc_buf[bytes_read], "[0x%02x] 0x%02x\n", i, cmd[i]);
	}
	
	bytes_read = sprintf(buf, "%s\n RFB: 0x%04x GFB: 0x%04x KFB: 0x%04x R1: 0x%04x YG2: 0x%04x KFB2:0x%04x KFB3:0x%04x\n", 
				__func__,
				tca6507_drvdata.led_state[TCA6507_LED_RFB],
				tca6507_drvdata.led_state[TCA6507_LED_GFB],
				tca6507_drvdata.led_state[TCA6507_LED_KFB],
				tca6507_drvdata.led_state[TCA6507_LED_R1],
				tca6507_drvdata.led_state[TCA6507_LED_YG2],
				tca6507_drvdata.led_state[TCA6507_LED_KFB2],
				tca6507_drvdata.led_state[TCA6507_LED_KFB3]
				);
				
	sprintf(buf, "%s", loc_buf);

	return bytes_read;
}

static ssize_t tca6507_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/*struct i2c_client *client =
		container_of(dev, struct i2c_client, dev);*/
	unsigned cmd_number;
	
	sscanf(buf, "%3d\n", &cmd_number);
	
	dev_dbg(dev, "%s: COMMAND: %d\n", __func__, cmd_number);

	tca6507_charger_state_report(cmd_number);

	return count;
}
static ssize_t btn_brightness_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned brightness;
	unsigned index;
	
	sscanf(buf, "%3d\n", &brightness);
	
	dev_dbg(dev, "%s: %d %d\n", __func__, count, brightness);
	if((brightness==0x808080)||(brightness==0x141414))
	{
		index=9;
		tca6507_led_switch((brightness == 0x808080) ? true : false, index);
	}
	else
	{
		index=8;
		tca6507_led_switch((brightness > 0) ? true : false, index);
		
	}	
	//dev_dbg(dev, "%s: %d %d\n", __func__, msm_set_led_intensity_proc(PM_LCD_LED, 4), msm_set_led_intensity_proc(PM_KBD_LED, 4));
	
	return count;
}

static ssize_t blink_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned enable;
	
	sscanf(buf, "%d\n", &enable);
	
	dev_dbg(dev, "%s: %d\n", __func__, enable);

	tca6507_notify_blink((enable > 0) ? true : false);

	return count;
}
/*FIH, MichaelKao, 2009/07/07 {*/
/*Add a Jogball enable function for Jogball driver*/
static ssize_t Jogball_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned enable;
	
	sscanf(buf, "%d\n", &enable);
	
	dev_dbg(dev, "%s: %d\n", __func__, enable);

	tca6507_Jogball_enable((enable > 0) ? true : false);

	return count;


}
/* FIH, Michael Kao, 2010/05/14{ */
/* [FXX_CR], add for not to disable charger*/
static ssize_t Charging_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned enable;
	
	sscanf(buf, "%d\n", &enable);
	
	dev_dbg(dev, "%s: %d\n", __func__, enable);

	charger_on=enable;
	
	return count;
}
/* FIH, Michael Kao, 2010/05/14{ */

/*FIH, MichaelKao, 2009/07/07 {*/
DEVICE_ATTR(tca6507_debug, 0666, tca6507_debug_show, tca6507_debug_store);
DEVICE_ATTR(btn_brightness, 0666, NULL, btn_brightness_store);//Michael
DEVICE_ATTR(blink, 0666, NULL, blink_store);//Michael
/*FIH, MichaelKao, 2009/07/07 {*/
/*Add a Jogball enable function for Jogball driver*/
DEVICE_ATTR(jben, 0666, NULL, Jogball_enable_store);
/*FIH, MichaelKao, 2009/07/07 {*/
/* FIH, Michael Kao, 2010/05/14{ */
/* [FXX_CR], add for not to disable charger*/
DEVICE_ATTR(chgen, 0666, NULL, Charging_enable_store);
/* FIH, Michael Kao, 2010/05/14{ */

#ifdef CONFIG_PM
static int tca6507_suspend(struct i2c_client *nLeds, pm_message_t mesg)
{

	//int ret;
	u8 cmd[12] = {0x00, 0x00, 0x02, 0x00, 0x77, 0x44, 0x77, 0xBB, 0xBB, 0xFF, 0xCF, 0x88};
	if ((tca6507_drvdata.HWID >= CMCS_HW_VER_EVB1) &&(tca6507_drvdata.HWID <= CMCS_CTP_MP3) )
		cmd[2]=0x02;
	else if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
		cmd[2]=0x01;
	printk("tca6507_suspend");
	//Battery_power_supply_change();
	//tca6507_led_blink(TCA6507_LED_R1, cmd);
	tca6507_drvdata.is_suspend=true;
	/*FIH, MichaelKao, 2009/06/25 {*/
	/*Add notify blink function for suspend resume*/
	tca6507_notify_blink(tca6507_drvdata.is_notify_blink) ;
	/*FIH, MichaelKao, 2009/06/25 {*/
	mutex_lock(&tca6507_drvdata.tca6507_lock);
	if (tca6507_drvdata.is_blinking||tca6507_drvdata.is_charging) {
		cmd[0] = TCA6507_INITIALIZATION_REGISTER;
		tca6507_read(tca6507_drvdata.tca6507_i2c_client, cmd, sizeof(cmd) - 1);
		
		cmd[4] = 0x55;
		cmd[5] = 0x11;
		cmd[6] = 0x55;
		cmd[7] = 0xEE;
		cmd[8] = 0xEE;
		cmd[9] = 0x3F;

		tca6507_write(tca6507_drvdata.tca6507_i2c_client, cmd, sizeof(cmd));
	} else {
		gpio_init_setting(0);
		gpio_release();
	}

	dev_dbg(&nLeds->dev, "%s: ENTER SUSPEND MODE\n", __func__);

	return 0;
}

static int tca6507_resume(struct i2c_client *nLeds)
{
	//int ret;
	tca6507_drvdata.is_suspend=false;
	
	if (tca6507_drvdata.is_blinking) {
	} else {
		gpio_init_setting(1);
		gpio_release();
	}
	
	mutex_unlock(&tca6507_drvdata.tca6507_lock);
	/*FIH, MichaelKao, 2009/09/28 {*/
	/*Modify for correctly update battery information in suspend mode*/
	Battery_power_supply_change();
	/*FIH, MichaelKao, 2009/09/28 {*/
	/*Add notify blink function for suspend resume*/
	tca6507_notify_blink(tca6507_drvdata.is_notify_blink) ;
	/*FIH, MichaelKao, 2009/06/25 {*/

	dev_dbg(&nLeds->dev, "%s: LEAVE SUSPEND MODE\n", __func__);

	return 0;
}
#else
# define tca6507_suspend NULL
# define tca6507_resume  NULL
#endif

/*FIH_FTM, PinyCHWu, 2009/06/15 {*/
/*Add misc device ioctl command functions*/
#ifdef CONFIG_FIH_FXX
//piny add file control function for FTM test.
/*******************File control function******************/
// devfs
static int tca6507_miscdev_open( struct inode * inode, struct file * file )
{
	if( ( file->f_flags & O_ACCMODE ) == O_WRONLY )
	{
		return -1;
	}
	else
		return 0;
}
static int tca6507_miscdev_ioctl( struct inode * inode, struct file * filp, unsigned int cmd2, unsigned long arg )
{
  	int ret = 0;
	int index = 0;
    if(copy_from_user(&index, (int __user*)arg, sizeof(int)))
	{
		return -1;
	}
	if(index < TCA6507_LED_RFB || index > TCA6507_LED_KFB3)
	{
	  	return -1;
	}
	if(cmd2 == TCA6507_IO_ON)
		tca6507_led_switch(TCA6507_LED_ON, index);
	else if(cmd2 == TCA6507_IO_OFF)
	  	tca6507_led_switch(TCA6507_LED_OFF, index);
	else
	{
	  	printk("[%s:%d]Unknow ioctl cmd", __func__, __LINE__);
	  	ret = -1;
	}
	return ret;
}
static int tca6507_miscdev_release( struct inode * inode, struct file * filp )
{
	return 0;
}
static const struct file_operations tca6507_miscdev_fops = {
	.open = tca6507_miscdev_open,
	.ioctl = tca6507_miscdev_ioctl,
	.release = tca6507_miscdev_release,
};
static struct miscdevice tca6507_miscdev = {
 	.minor = MISC_DYNAMIC_MINOR,
	.name = "tca6507_miscdev",
	.fops = &tca6507_miscdev_fops,
};
#endif
/******************Driver Functions*******************/
static int __devinit tca6507_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct tca6507_platform_data *tca6507_pd = client->dev.platform_data;
  	int i, ret = 0;

	i2c_set_clientdata(client, &tca6507_drvdata);
	tca6507_drvdata.reset_pin		= tca6507_pd->tca6507_reset;
	tca6507_drvdata.tca6507_i2c_client	= client;
	//disable this due to there is no need to check version.
	tca6507_drvdata.HWID			= FIH_READ_HWID_FROM_SMEM();
	tca6507_drvdata.is_blinking		= false;
	tca6507_drvdata.is_charging		= false;
	tca6507_drvdata.is_suspend		= false;
	tca6507_drvdata.is_notify_blink	= false;
	/*FIH, MichaelKao, 2009/07/07 {*/
	/*Add a Jogball enable function for Jogball driver*/
	tca6507_drvdata.Jogball_enable     = false;
	/*FIH, MichaelKao, 2009/07/07 {*/
	for (i = 0; i < 7; i++) {
		tca6507_drvdata.led_state[i] = 0x0001;// Initial: TURN_OFF
		tca6507_drvdata.prev_led_state[i] = TCA6507_LED_PREV_STATE_OFF;
		tca6507_drvdata.is_fade_on[i] = true;
	}
	charger_on=0;
	mutex_init(&tca6507_drvdata.tca6507_lock);
	
	ret = tca6507_led_init();
	if(ret < 0) {
		dev_err(&client->dev, "tca6507 LED init failed\n");
		mutex_destroy(&tca6507_drvdata.tca6507_lock);
		
		return ret;
	}
	
	ret = device_create_file(&client->dev, &dev_attr_tca6507_debug);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Create keyboard attribute \"brightness\" failed!! <%d>", __func__, ret);
		mutex_destroy(&tca6507_drvdata.tca6507_lock);

		return ret; 
	}
	ret = device_create_file(&client->dev, &dev_attr_btn_brightness);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Create keyboard attribute \"btn_brightness\" failed!! <%d>", __func__, ret);
		mutex_destroy(&tca6507_drvdata.tca6507_lock);
		return ret; 
	}
	ret = device_create_file(&client->dev, &dev_attr_blink);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Create keyboard attribute \"blink\" failed!! <%d>", __func__, ret);
		mutex_destroy(&tca6507_drvdata.tca6507_lock);
		return ret; 
	}
	/*FIH, MichaelKao, 2009/07/07 {*/
	/*Add a Jogball enable function for Jogball driver*/
	ret = device_create_file(&client->dev, &dev_attr_jben);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Create keyboard attribute \"jben\" failed!! <%d>", __func__, ret);
		mutex_destroy(&tca6507_drvdata.tca6507_lock);
		return ret; 
	}
	/* FIH, Michael Kao, 2010/05/14{ */
	/* [FXX_CR], add for not to disable charger*/
	ret = device_create_file(&client->dev, &dev_attr_chgen);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Create keyboard attribute \"chgen\" failed!! <%d>", __func__, ret);
		mutex_destroy(&tca6507_drvdata.tca6507_lock);
		return ret; 
	}
	/* FIH, Michael Kao, 2010/05/14{ */
	/*FIH, MichaelKao, 2009/07/07 {*/
	if ((tca6507_drvdata.HWID >= CMCS_HW_VER_EVB1) &&(tca6507_drvdata.HWID <= CMCS_CTP_MP3) )
		tca6507_drvdata.LEDnum=5;
	else if(tca6507_drvdata.HWID >= CMCS_7627_EVB1)
		tca6507_drvdata.LEDnum=7;
	
	return ret;
}

static int __devexit tca6507_remove(struct i2c_client *client)
{
	int ret = 0;
	
	mutex_destroy(&tca6507_drvdata.tca6507_lock);
	device_remove_file(&client->dev, &dev_attr_tca6507_debug);

	return ret;
}

static const struct i2c_device_id tca6507_idtable[] = {
       { "tca6507", 0 },
       { }
};

static struct i2c_driver tca6507_driver = {
	.driver = {
		.name	= "tca6507",
	},
	.probe		= tca6507_probe,
	.remove		= __devexit_p(tca6507_remove),
	.suspend  	= tca6507_suspend,
	.resume   	= tca6507_resume,
	.id_table	= tca6507_idtable,
};

static int __init tca6507_init(void)
{
	int ret = 0;
	
	// i2c
	ret = i2c_add_driver(&tca6507_driver);
	if (ret) {
		goto driver_del;
	}

/*FIH_FTM, PinyCHWu, 2009/06/15 {*/
/*Use miscdev*/
#ifdef CONFIG_FIH_FXX
	//register and allocate device, it would create an device node automatically.
	//use misc major number plus random minor number, and init device
	
	ret = misc_register(&tca6507_miscdev);
	if (ret){
		goto register_del;
	}
#endif
/*}FIH_FTM, PinyCHWu, 2009/06/15*/
	//all successfully.
	return ret;
/*FIH_FTM, PinyCHWu, 2009/06/15 {*/
/*new label name for remove misc device*/
#ifdef CONFIG_FIH_FXX
register_del:
	misc_deregister(&tca6507_miscdev);
#endif
/*} FIH_FTM, PinyCHWu, 2009/06/15*/

driver_del:
	i2c_del_driver(&tca6507_driver);
	
	return -1;
}

static void __exit tca6507_exit(void)
{
/*FIH_FTM, PinyCHWu, 2009/06/15 {*/
/*new label name for remove misc device*/
#ifdef CONFIG_FIH_FXX
	misc_deregister(&tca6507_miscdev);
#endif
	i2c_del_driver(&tca6507_driver);
}

module_init(tca6507_init);
module_exit(tca6507_exit);

MODULE_AUTHOR( "Audi PC Huang <audipchuang@fihtdc.com>" );
MODULE_DESCRIPTION( "TCA6507 driver" );
MODULE_LICENSE( "GPL" );
