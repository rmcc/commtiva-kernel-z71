/* drivers/power/goldfish_battery.c
 *
 * Power supply driver for the goldfish emulator
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
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include "../../kernel/power/power.h"
#include <asm/io.h>
#include <linux/delay.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>
/* FIH, Michael Kao, 2009/08/13{ */
/* [FXX_CR], Modify to create a new work queue for BT play MP3 smoothly*/
#include <linux/workqueue.h>
/* FIH, Michael Kao, 2009/08/13{ */

#include "../../arch/arm/mach-msm/proc_comm.h"
#define T_FIH
#ifdef T_FIH	///+T_FIH
//#include <linux/gasgauge_bridge.h>
#include <asm/gpio.h>
/* FIH, Michael Kao, 2009/10/14{ */

#include <linux/wakelock.h>
#include <linux/android_alarm.h>

#ifdef CONFIG_USB_GADGET_MSM_72K 
#include <mach/msm_hsusb.h>
#endif


#define GPIO_CHR_DET 39		// Input power-good (USB port/adapter present indicator) pin
#define GPIO_CHR_FLT 32		// Over-voltage fault flag
#define GPIO_CHR_EN 33		//Charging enable pin
/* FIH, Michael Kao, 2009/08/14{ */
/* [FXX_CR], Add log to check GPIO setting */
#define CHR_1A 26			//Charging current 1A/500mA
#define USBSET 123                       //current 1 or1/5
/* FIH, Michael Kao, 2009/08/14{ */
#define FLAG_CHARGER_DETECT
#endif	// T_FIH	///-T_FIH
/*+++FIH_ADQ+++*/
/* FIH, Michael Kao, 2009/08/18{ */
/* [FXX_CR], Add pmlog*/
//#include "linux/pmlog.h"
/* FIH, Michael Kao, 2009/08/18{ */
/*Add misc device*/
#ifdef CONFIG_FIH_FXX
#include <linux/miscdevice.h>
#include <asm/ioctl.h>
#include <asm/fcntl.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>


/* +++ New charging temperature protection for Greco hardware */
#define GET_TEMPERATURE_FROM_BATTERY_THERMAL    1
#if GET_TEMPERATURE_FROM_BATTERY_THERMAL
#define BATTERY_TEMP_LOW_LIMIT  0   //Low temperature limit for charging
#define BATTERY_TEMP_HIGH_LIMIT 45  //High temperature limit for charging
#define BATTERY_TEMP_COOL_DOWN_FROM_EMGCY   50  //Battery temp. lower than 50 degree leaves cool down mode.
#define BATTERY_TEMP_SHUTDOWN_AP    55  //Battery temperature threshod to shut down AP to cool down device.
#define BATTERY_TEMP_EMGCY_CALL_ONLY    58  //Battery temperature threshod for emergency call only.

#define MSM_TEMP_COOL_DOWN_FROM_EMGCY   60  //Main board temperature lower than 60 degree leaves cool down mode.
#define MSM_TEMP_SHUTDOWN_AP    70  //Main board temperature threshod to shut down AP to cool down device.
#define MSM_TEMP_EMGCY_CALL_ONLY    75  //Main board temperature threshod for emergency call only.
#define RECHARGE_TEMP_OFFSET    1   //Start to recharge if the temperature is > (0+1) or < (40-1)
#define BATTERY_THERMAL_TEMP_OFFSET 1   //When charging, the temp. read from thermistor is higher then actual battery temp. 
#define OLD_BATTERY_RESISTOR_TEMP   (-21)   //Old battery has 100k resistor => -21 degree
#define OLD_BATTERY_RESISTOR_TEMP_TOL   (5) //Tolerance for temperature read from adc.

/* We may misjudge the new battery as old battery due to the resistor and adc tolerance. */
/* If the temperature of battery thermistor becomes -11 degree, it means the battery is new battery. */
/* Then we should use the battery thermistor to do temperature protection. */
#define MAY_BE_BATTERY_THERMAL_TEMP     (-11)

#endif
/* --- New charging temperature protection for Greco hardware */

/*Add misc device ioctl command*/
#define FTMBATTERY_MAGIC 'f'
#define FTMBATTERY_CAP		_IO(FTMBATTERY_MAGIC, 0)
#define FTMBATTERY_VOL		_IO(FTMBATTERY_MAGIC, 1)
#define FTMBATTERY_TEP		_IO(FTMBATTERY_MAGIC, 2)
#define FTMBATTERY_CUR		_IO(FTMBATTERY_MAGIC, 3)
#define FTMBATTERY_AVC		_IO(FTMBATTERY_MAGIC, 4)
#define FTMBATTERY_STA		_IO(FTMBATTERY_MAGIC, 5)
#define FTMBATTERY_BID		_IO(FTMBATTERY_MAGIC, 6)
#define FTMBATTERY_PRE_VOL		_IO(FTMBATTERY_MAGIC, 7)
#define FTMBATTERY_VBAT		_IO(FTMBATTERY_MAGIC, 8)

/* FIH; Tiger; 2010/1/20 { */
/* support ecompass */
#ifdef CONFIG_FIH_FXX
#define FEATURE_ECOMPASS
#endif
/* } FIH; Tiger; 2010/1/20 */

#endif

enum {
	CHARGER_STATE_UNKNOWN,		
	CHARGER_STATE_CHARGING,		
	CHARGER_STATE_DISCHARGING,	
	CHARGER_STATE_NOT_CHARGING,	
	CHARGER_STATE_FULL,
	CHARGER_STATE_LOW_POWER,
};
typedef struct _VOLT_TO_PERCENT
{
    u16 dwVolt;
    u16 dwPercent;
} VOLT_TO_PERCENT;
/* FIH, Michael Kao, 2009/08/26{ */
/* [FXX_CR], Modify for charging too fast in low power issue*/
#if 0
static VOLT_TO_PERCENT g_Volt2PercentMode[6] =
{
    { 3400, 0},		// empty,    Rx Table
    { 3700, 20},    // level 1
    { 3785, 40},    // level 2
    { 3860, 60},    // level 3
    { 3985, 80},    // level 4
    /* FIH, Michaelkao, 2009/09/30 { */
    /* Requirement to change battery full voltage to 4.10V*/
    { 4100, 100},   // full
    /* FIH, Michaelkao, 2009/09/30 { */
};
#endif
/* FIH, Michael Kao, 2009/10/14{ */
/* [FXX_CR], Modofy for using different profile for different battery*/
static VOLT_TO_PERCENT g_Volt2PercentMode[10] =
{
    /* FIH, Michael Kao, 2009/12/21{ */
    /* [FXX_CR], Modofy for using 300mA discharging table*/
    /* FIH, Michael Kao, 2010/01/11{ */
    /* [FXX_CR], Adjust power off voltage to 3.5V*/
    { 3500, 0},	   // empty,    Rx Table
    /* FIH, Michael Kao, 2010/01/11{ */
    { 3610, 15},    // level 1
    { 3675, 25},    // level 2
    { 3695, 35},    // level 3
    { 3720, 45},    // level 4
    { 3760, 55},    // level 5
    { 3825, 65},    // level 6
    { 3900, 75},    // level 7
    { 3990, 85},    // level 8
    { 4100, 100},   // full
};
static VOLT_TO_PERCENT g_Volt2PercentMode2[10] =
{
    /* FIH, Michael Kao, 2010/01/11{ */
    /* [FXX_CR], Adjust power off voltage to 3.5V*/
    { 3500, 0},	   // empty,    Rx Table
    /* FIH, Michael Kao, 2010/01/11{ */
    { 3610, 15},    // level 1
    { 3675, 25},    // level 2
    { 3695, 35},    // level 3
    { 3720, 45},    // level 4
    { 3760, 55},    // level 5
    { 3825, 65},    // level 6
    { 3900, 75},    // level 7
    { 3990, 85},    // level 8
    { 4100, 100},   // full
};
/* FIH, Michael Kao, 2009/12/21{ */
/* FIH, Michael Kao, 2009/10/14{ */

/* FIH, Michael Kao, 2009/08/26{ */
extern void tca6507_charger_state_report(int state);
//static int g_charging_state_last = CHARGER_STATE_LOW_POWER;
static int g_charging_state_last = CHARGER_STATE_NOT_CHARGING;

/* FIH, Michael Kao, 2009/08/13{ */
/* [FXX_CR], Modify to create a new work queue for BT play MP3 smoothly*/
/* FIH, Michael Kao, 2009/08/13{ */

struct zeus_battery_data {
	u8 capacity;
	int voltage;
} __attribute__((packed));

struct zeus_battery_update {
	struct power_supply bat;

	struct zeus_battery_data data;

	struct alarm alarm;
	struct wake_lock wakelock;
	struct work_struct zeus_batt_work;
	struct workqueue_struct *wqueue;
	ktime_t last_poll;
	u8 slow_poll;
};

static struct zeus_battery_update *batt_update;

unsigned int Battery_HWID;
/* FIH, Michael Kao, 2009/07/16{ */
/* [FXX_CR], Modify for different smem channel with different modem image*/
unsigned int Modem_mode;
/* FIH, Michael Kao, 2009/07/16{ */
#define GOLDFISH_BATTERY_READ(data, addr)   (readl(data->reg_base + addr))
#define GOLDFISH_BATTERY_WRITE(data, addr, x)   (writel(x, data->reg_base + addr))
///extern int check_USB_type;

/* temporary variable used between goldfish_battery_probe() and goldfish_battery_open() */
#ifdef T_FIH    ///+T_FIH
static int g_charging_state = CHARGER_STATE_NOT_CHARGING;
#endif	// T_FIH	///-T_FIH

enum {
	/* status register */
	BATTERY_INT_STATUS	    = 0x00,
	/* set this to enable IRQ */
	BATTERY_INT_ENABLE	    = 0x04,

	BATTERY_AC_ONLINE       = 0x08,
	BATTERY_STATUS          = 0x0C,
	BATTERY_HEALTH          = 0x10,
	BATTERY_PRESENT         = 0x14,
	BATTERY_CAPACITY        = 0x18,

	BATTERY_STATUS_CHANGED	= 1U << 0,
	AC_STATUS_CHANGED   	= 1U << 1,
	BATTERY_INT_MASK        = BATTERY_STATUS_CHANGED | AC_STATUS_CHANGED,
};

static enum power_supply_property zeus_battery_props[] = {
       POWER_SUPPLY_PROP_STATUS,
       POWER_SUPPLY_PROP_HEALTH,
       POWER_SUPPLY_PROP_PRESENT,
       POWER_SUPPLY_PROP_TECHNOLOGY,
       POWER_SUPPLY_PROP_CAPACITY,
       POWER_SUPPLY_PROP_TEMP,
       POWER_SUPPLY_PROP_VOLTAGE_NOW,

};

static enum power_supply_property zeus_power_properties[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
    "battery",
};

typedef enum {
    CHARGER_BATTERY = 0,
    CHARGER_USB,
    CHARGER_AC
} charger_type_t;

charger_type_t current_charger = CHARGER_BATTERY;
enum{
	battery_charger_type=0,
	wifi_state,
	GPS_state,
	phone_state,
};

struct F9_device_state{
	int F9_battery_charger_type;
	int F9_wifi_state;
	int F9_GPS_state;
	int F9_phone_state;
};
struct battery_info{
	unsigned pre_batt_val;
	unsigned new_batt_val;
};

#if GET_TEMPERATURE_FROM_BATTERY_THERMAL
#define TRUE    1
#define FALSE   0
static int g_use_battery_thermal = FALSE;
static int g_orig_hw_id;
static int g_cool_down_mode = FALSE;
#endif
/* FIH, Michael Kao, 2010/05/26{ */
/* [FXX_CR], for save the correct format of thermal value with negative value*/
signed	msm_termal;
/* FIH, Michael Kao, 2010/05/26{ */

bool over_temper;
bool slight_over_temper;
static int g_data_number;
static unsigned g_batt_data[10];
static struct battery_info	PMIC_batt;
static struct F9_device_state batt_state;

static int zeus_power_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{
    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        if (psy->type == POWER_SUPPLY_TYPE_MAINS)
            val->intval = (current_charger ==  CHARGER_AC ? 1 : 0);
        else if (psy->type == POWER_SUPPLY_TYPE_USB)
            val->intval = (current_charger ==  CHARGER_USB ? 1 : 0);
        else
            val->intval = 0;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int zeus_battery_get_property(struct power_supply *psy,
               enum power_supply_property psp,
               union power_supply_propval *val);

static struct power_supply zeus_power_supplies[] = {
    {
        .name = "battery",
        .type = POWER_SUPPLY_TYPE_BATTERY,
        .properties = zeus_battery_props,
        .num_properties = ARRAY_SIZE(zeus_battery_props),
        .get_property = zeus_battery_get_property,
    },
    {
        .name = "usb",
        .type = POWER_SUPPLY_TYPE_USB,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
        .properties = zeus_power_properties,
        .num_properties = ARRAY_SIZE(zeus_power_properties),
        .get_property = zeus_power_get_property,
    },
    {
        .name = "ac",
        .type = POWER_SUPPLY_TYPE_MAINS,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
        .properties = zeus_power_properties,
        .num_properties = ARRAY_SIZE(zeus_power_properties),
        .get_property = zeus_power_get_property,
    },
};

/* FIH, Michael Kao, 2009/08/14{ */
/* [FXX_CR], Not update battery information when charger state changes*/
bool charger_state_change;//flag for chargung state change
int system_time_second;//get the system time while charger plug in/out
int time_duration; //the time duration while charger plug in/out
/* FIH, Michael Kao, 2009/08/24{ */
/* [FXX_CR], Avoid weird value*/
int weird_count;//count for value goes down while charging
/* FIH, Michael Kao, 2009/08/24{ */

/* FIH, Michael Kao, 2009/09/10{ */
/* [FXX_CR], charging full protection*/
/* FIH, Michael Kao, 2009/12/01{ */
/* [FXX_CR], Modify for battery_full_falg abnormal changed*/
int battery_full_flag;//battery full flag
/* FIH, Michael Kao, 2009/12/01{ */

bool battery_full_flag2;//for show 100%
int battery_full_count;//for first time reach 100% to update time start point 
//int battery_full_count2;//for first time plug in charger at 100% to update time start point
int battery_full_start_time;//battery full start time
int battery_full_time_duration;//battery full time duration
/* FIH, Michael Kao, 2009/09/10{ */

/* FIH, Michael Kao, 2009/09/10{ */
/* [FXX_CR], add for avoid too frequently update battery*/
int pre_time_cycle;
int pre_val;
/* FIH, Michael Kao, 2009/08/14{ */
/* FIH, Michael Kao, 2009/09/30{ */
/* [FXX_CR], add for update battery information more accuracy in suspend mode*/
bool suspend_update_flag;
/* FIH, Michael Kao, 2009/09/30{ */
int VBAT_from_SMEM;
int battery_low;
/* FIH, Michael Kao, 2009/11/25{ */
/* [FXX_CR], add a retry mechanism to prevent the sudden high value*/
bool weird_retry_flag;
/* FIH, Michael Kao, 2009/11/25{ */

/* FIH, Michael Kao, 2009/12/01{ */
/* [FXX_CR], add for update battery information more accuracy only for battery update in suspend mode*/
int pre_suspend_time;
int suspend_time_duration;
/* FIH, Michael Kao, 2009/12/01{ */
int suspend_update_sample_gate;
/* FIH, Michael Kao, 2009/12/25{ */
//New charging temperature protection scenario
int high_vol;
int charging_bat_vol;
bool over_temper2;

/* FIH, Michael Kao, 2009/12/25{ */

/* [FXX_CR], add for not to disable charger*/
extern int charger_on;
int g_pre_use_battery_thermal;



static int Average_Voltage(void)
{
	int i, j;
	unsigned temp_sum=0;
	g_batt_data[g_data_number%10]=PMIC_batt.new_batt_val;
	g_data_number++;
	if(suspend_update_flag&&(suspend_update_sample_gate==1))
	{
		//eventlog("update 3");
		for(i=0;i<=1;i++)
		{
			g_batt_data[g_data_number%10]=PMIC_batt.new_batt_val;
			g_data_number++;
		}

	}
	else if((suspend_update_flag&&(suspend_update_sample_gate==2))||battery_low)/* FIH, Michael Kao, 2009/11/16{ */
	{	
  	   	/* FIH, Michael Kao, 2009/09/30{ */
		/* [FXX_CR], add for update battery information more accuracy in suspend mode*/
		//eventlog("update 5");
		for(i=0;i<=3;i++)
		{
			g_batt_data[g_data_number%10]=PMIC_batt.new_batt_val;
			g_data_number++;
		}
  	   	/* FIH, Michael Kao, 2009/09/30{ */
	}
	for(j=0;j<10;j++)
	{
		temp_sum+=g_batt_data[j];
	}
	PMIC_batt.new_batt_val=temp_sum/10;
	return	PMIC_batt.new_batt_val;
}
static int	GetPMIC_MSM_TERMAL(void)
{
	int adc_read_id;
	/* FIH, Michael Kao, 2010/05/17{ */
	/* [FXX_CR], Add Greco battery protection scenario*/
	#if GET_TEMPERATURE_FROM_BATTERY_THERMAL    //Enable it after the algorithm for two thermistor is ready
   	int t;
	#endif
	adc_read_id=22;
	#if GET_TEMPERATURE_FROM_BATTERY_THERMAL    //Enable it after the algorithm for two thermistor is ready
   	if ((g_orig_hw_id >= CMCS_125_CTP_GRE_PR1 && g_orig_hw_id <= CMCS_125_CTP_GRE_MP2)||
	(g_orig_hw_id >= CMCS_CTP_F917_PR2 && g_orig_hw_id <= CMCS_CTP_F917_MP3))
   	//if (g_orig_hw_id >= CMCS_HW_EVB1 && g_orig_hw_id < CMCS_7627_ORIG_EVB1)
    	{
        		if (g_use_battery_thermal)
        		{
            		adc_read_id = 0x10; //Use the ADC_BATT_THERM_DEGC channel of 7227 modem
        		}
        		else
        		{
            		adc_read_id = 0x10; //Use the ADC_BATT_THERM_DEGC channel of 7227 modem
            		t = proc_comm_read_adc(&adc_read_id);
            		if ((t >= MAY_BE_BATTERY_THERMAL_TEMP)&&(t!=90))
            		{
                			g_use_battery_thermal = TRUE;
                			return t;
            		}
            		else
            		{
                			adc_read_id = 22;
            		}
        		}
    	}else
	#endif  //Enable it after the algorithm for two thermistor is ready 
	/* FIH, Michael Kao, 2010/05/17{ */
	if(Battery_HWID>=CMCS_7627_EVB1)
	{
		if(Modem_mode==FIH_WCDMA)
			adc_read_id=22;
		else if(Modem_mode==FIH_CDMA1X)
			adc_read_id=23;
		else if(Modem_mode==FIH_WCDMA_CDMA1X)
			adc_read_id=24;
	}
	return proc_comm_read_adc(&adc_read_id);
}

/* FIH; Tiger; 2009/8/15 { */
/* ecompass needs this value to control charger behavior */
#ifdef FEATURE_ECOMPASS
extern unsigned fihsensor_battery_voltage;
extern unsigned fihsensor_battery_level;
extern int fihsensor_magnet_guard1;
#endif
/* } FIH; Tiger; 2009/8/15 */


static int	GetPMICBatteryInfo(void)
{
	int adc_read_id, i;
	unsigned battery_sum=0;
	int smpnum=3;
	int countB;
	int weird_count_number;
	/* FIH, Michael Kao, 2009/11/16{ */
	/* [FXX_CR], Create different thresholds in diffrerent voltage range to prevent voltage drop issue*/
	int iC, Vbat_threshold;
	/* FIH, Michael Kao, 2009/11/16{ */
	
	if(suspend_update_flag)
		weird_count_number=3;
	else
		weird_count_number=7;
	//int batt_offset=80;
	/* FIH, Michael Kao, 2009/08/14{ */
	/* [FXX_CR], Not update battery information when charger state changes*/
	time_duration=get_seconds()-system_time_second;
	if(charger_state_change&&(time_duration<3)&&(PMIC_batt.pre_batt_val!=0))
	{
		return PMIC_batt.pre_batt_val;
	}
	else
	{
		charger_state_change=false;
	}
	/* FIH, Michael Kao, 2009/08/14{ */
	adc_read_id=11;
	if(Battery_HWID >= CMCS_7627_EVB1)
	{
		/* FIH, Michael Kao, 2009/07/16{ */
		/* [FXX_CR], Modify for different smem channel with different modem image*/
		if(Modem_mode==FIH_WCDMA)
			adc_read_id=11;
		else if(Modem_mode==FIH_CDMA1X)
			adc_read_id=12;
		else if(Modem_mode==FIH_WCDMA_CDMA1X)
			adc_read_id=13;
		/* FIH, Michael Kao, 2009/07/16{ */
	}

	/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
	if(fihsensor_magnet_guard1 != 1) {
		fihsensor_magnet_guard1 = -1;
	}
#endif
	/* } FIH; Tiger; 2009/8/15 */
/* FIH, Michael Kao, 2010/05/14{ */
/* [FXX_CR], add for not to disable charger*/

if(charger_on==0)
{
	if(!gpio_get_value(GPIO_CHR_DET)&&!over_temper&&!battery_full_flag&&!over_temper2)
	{
		charging_bat_vol=proc_comm_read_adc(&adc_read_id);
		gpio_set_value(GPIO_CHR_EN,1);//Disable chager before read voltage
  	   	/* FIH, Michael Kao, 2009/09/30{ */
		/* [FXX_CR], add for update battery information more accuracy in suspend mode*/
		//if(suspend_update_flag)
			//msleep(1000);
		//else
			msleep(2000);
  	   	/* FIH, Michael Kao, 2009/09/30{ */
	}
}
	//Read VBat from PMIC
	for(countB=0;countB<smpnum;countB++)
	{
		battery_sum+=proc_comm_read_adc(&adc_read_id);
	}
if(charger_on==0)
{
	if(!gpio_get_value(GPIO_CHR_DET)&&!over_temper&&!battery_full_flag&&!over_temper2)
	{
		/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
		if(fihsensor_magnet_guard1 != 0) {
			fihsensor_magnet_guard1 = -1;
		}
#endif
		/* } FIH; Tiger; 2009/8/15 */
		gpio_set_value(GPIO_CHR_EN,0);//Enable chager after read voltage
	}
}
/* FIH, Michael Kao, 2010/05/14{ */

	PMIC_batt.new_batt_val=battery_sum/smpnum;
	VBAT_from_SMEM=PMIC_batt.new_batt_val;
	/* FIH, Michael Kao, 2009/11/16{ */
	/* [FXX_CR],update battery sample data when battery capacity is low*/
	if(PMIC_batt.new_batt_val<=3600&&gpio_get_value(GPIO_CHR_DET))
		battery_low=true;
	else
		battery_low=false;
	/* FIH, Michael Kao, 2009/11/16{ */
	//PMIC_batt.new_batt_val+=batt_offset;//Temporarily add offset

	/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
	fihsensor_battery_voltage = PMIC_batt.new_batt_val;
#endif
	/* } FIH; Tiger; 2009/8/15 */

	//Set default voltage when first boot
	if(PMIC_batt.pre_batt_val==0)
	{
		for(i=0;i<10;i++)
		{
			g_batt_data[i]=PMIC_batt.new_batt_val;
		}
		PMIC_batt.pre_batt_val=PMIC_batt.new_batt_val;
	}
	//When charging plug in
	if(!gpio_get_value(GPIO_CHR_DET)) 
	{
		/* [FXX_CR], add a retry mechanism to prevent the sudden high value*/
		//Avoid sudden high value  value
		if((PMIC_batt.pre_batt_val>3600)&&!weird_retry_flag&&(PMIC_batt.new_batt_val>(PMIC_batt.pre_batt_val+150)))
		{
			weird_retry_flag=true;
			return PMIC_batt.pre_batt_val;
		}
		else
			weird_retry_flag=false;
		/* FIH, Michael Kao, 2009/11/25{ */
		
		//Voltage only can go up
		/* FIH, Michael Kao, 2009/08/24{ */
		/* [FXX_CR], Avoid weird value*/
		if(PMIC_batt.new_batt_val<PMIC_batt.pre_batt_val)
		{
			weird_count++;
			if(weird_count<weird_count_number)
			{
				return PMIC_batt.pre_batt_val;
			}
		}
		else
		{
			weird_count=0;
		}
		/* FIH, Michael Kao, 2009/08/24{ */
		//Filter weird value
		//if((PMIC_batt.new_batt_val>=3600)&&((PMIC_batt.new_batt_val -PMIC_batt.pre_batt_val)>300))
			//return PMIC_batt.pre_batt_val;

		//Get average voltage
		PMIC_batt.new_batt_val=Average_Voltage();
		PMIC_batt.pre_batt_val=PMIC_batt.new_batt_val;
	}
	else//Charging unplug
	{
		if(PMIC_batt.pre_batt_val>3900)
			suspend_update_sample_gate=0;
		else if((PMIC_batt.pre_batt_val<=3900)&&(PMIC_batt.pre_batt_val>3700))
			suspend_update_sample_gate=1;
		else
			suspend_update_sample_gate=2;
		/* FIH, Michael Kao, 2009/11/16{ */
		/* [FXX_CR], Create different thresholds in diffrerent voltage range to prevent voltage drop issue*/
		//get threshold value
		for(iC=0;iC<10;iC++)
		{
			if(Battery_HWID >= CMCS_7627_EVB1)
			{
				if(PMIC_batt.pre_batt_val <= g_Volt2PercentMode2[iC].dwVolt)
           				break;
			}else
			{
				if(PMIC_batt.pre_batt_val <= g_Volt2PercentMode[iC].dwVolt)
           				break;
			}
		}
		if(Battery_HWID >= CMCS_7627_EVB1)
			Vbat_threshold=(g_Volt2PercentMode2[iC].dwVolt -g_Volt2PercentMode2[iC-1].dwVolt);
		else
			Vbat_threshold=(g_Volt2PercentMode[iC].dwVolt -g_Volt2PercentMode[iC-1].dwVolt);
		if(iC==2)
			Vbat_threshold=Vbat_threshold*6/5;
		else if(iC==1)
			Vbat_threshold=Vbat_threshold*3/2;
		else if(iC==0)
			Vbat_threshold=0;
		/* FIH, Michael Kao, 2009/11/16{ */

		//Voltage only can go down
		if(PMIC_batt.new_batt_val>PMIC_batt.pre_batt_val)
		{
			return PMIC_batt.pre_batt_val;
		}
		//Filter weird value
		else if((PMIC_batt.pre_batt_val -PMIC_batt.new_batt_val)>300)
		{
			return PMIC_batt.pre_batt_val;
		}
		/* FIH, Michael Kao, 2009/10/14{ */
		/* [FXX_CR], Modofy for avoid voltage drop cause capacity drop too much*/
		/* FIH, Michael Kao, 2009/11/16{ */
		/* [FXX_CR], use different thresholds in diffrerent voltage range to prevent voltage drop issue*/
		
		/* FIH, Michael Kao, 2009/12/01{ */
		/* [FXX_CR], use different thresholds in diffrerent voltage range to prevent voltage drop issue in suspend mode*/
		else 	if((PMIC_batt.pre_batt_val -PMIC_batt.new_batt_val)>Vbat_threshold)
		{
		/* FIH, Michael Kao, 2009/12/01{ */
			PMIC_batt.new_batt_val=PMIC_batt.pre_batt_val-Vbat_threshold;
		}
		/* FIH, Michael Kao, 2009/11/16{ */

		/* FIH, Michael Kao, 2009/10/14{ */
		//Get average voltage
		PMIC_batt.new_batt_val=Average_Voltage();
		PMIC_batt.pre_batt_val=PMIC_batt.new_batt_val;
	}
	return PMIC_batt.new_batt_val;			
}

static int	ChangeToVoltPercentage(unsigned Vbat)
{
	int Volt_pec=0;
	int iC;
	
	/* FIH, Michael Kao, 2009/10/14{ */
	/* [FXX_CR], Modofy for using different profile for different battery*/
	for(iC=0;iC<10;iC++)//Michael modify, 2009/10/21
	{
		if(Battery_HWID >= CMCS_7627_EVB1)
		{
			if(Vbat <= g_Volt2PercentMode2[iC].dwVolt)
           			break;
		}else
		{
			if(Vbat <= g_Volt2PercentMode[iC].dwVolt)
           			break;
		}
	}
	if(iC==0)
		Volt_pec=0;
	else if(iC==10)//Michael modify, 2009/10/21
		Volt_pec=100;
	else if((iC>=0)&&(iC<10))//Michael modify, 2009/10/21
	{
		if(Battery_HWID >= CMCS_7627_EVB1)
		{
			Volt_pec=g_Volt2PercentMode2[iC-1].dwPercent + 
				( Vbat -g_Volt2PercentMode2[iC-1].dwVolt) * ( g_Volt2PercentMode2[iC].dwPercent -g_Volt2PercentMode2[iC-1].dwPercent)/( g_Volt2PercentMode2[iC].dwVolt -g_Volt2PercentMode2[iC-1].dwVolt);
		}else
		{
			Volt_pec=g_Volt2PercentMode[iC-1].dwPercent + 
				( Vbat -g_Volt2PercentMode[iC-1].dwVolt) * ( g_Volt2PercentMode[iC].dwPercent -g_Volt2PercentMode[iC-1].dwPercent)/( g_Volt2PercentMode[iC].dwVolt -g_Volt2PercentMode[iC-1].dwVolt);
		}
	}
	/* FIH, Michael Kao, 2009/10/14{ */
	/* FIH, Michael Kao, 2009/09/10{ */
	/* [FXX_CR], charging full protection*/
	//get the battery full start time
	
	/* FIH, Michael Kao, 2009/11/11{ */
	/* [FXX_CR], Improve charging full protection scenario*/
	#if 0
	if(Volt_pec==100)
	{
		if(battery_full_count==0)
			battery_full_start_time=get_seconds();
		battery_full_count++;
	}
	else
		battery_full_count=0;
	#endif
	
	/* FIH, Michael Kao, 2009/11/11{ */
	/* FIH, Michael Kao, 2009/09/10{ */
	if(charger_on)
		Volt_pec=	80;

	/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
	fihsensor_battery_level = (unsigned)Volt_pec;
#endif
	/* } FIH; Tiger; 2009/8/15 */
	/* FIH, Michael Kao, 2010/05/06{ */
	/* [FXX_CR], Divide battery level (0~100) into 10 pieces*/
#if 0
	if (Volt_pec == 0)      
    		return 0;
    	else if (Volt_pec <= 5)
    		return 5;
    	else if (Volt_pec <= 11)
    		return 10;
    	else if (Volt_pec <= 16)
    		return 15;
    	else if (Volt_pec <= 22)
    		return 20;
    	else if (Volt_pec <= 33)
    		return 30;
    	else if (Volt_pec <= 44)
    		return 40;
    	else if (Volt_pec <= 54)
    		return 50;
    	else if (Volt_pec <= 64)
    		return 60;
    	else if (Volt_pec <= 74)
    		return 70;
    	else if (Volt_pec <= 84)
    		return 80;
    	else if (Volt_pec <= 94)
    		return 90;
    	else if (Volt_pec <= 100)
    		return 100;
    	else
    		return -1;
#endif
		
	
	return Volt_pec;
}


static void zeus_program_alarm(struct zeus_battery_update *zbu, int seconds)
{
    ktime_t low_interval = ktime_set(seconds - 10, 0);
    ktime_t slack = ktime_set(20, 0);
    ktime_t next;

    next = ktime_add(zbu->last_poll, low_interval);

    alarm_start_range(&zbu->alarm, next, ktime_add(next, slack));
}

/* FIH, Michael Kao, 2009/06/08{ */
/* [FXX_CR], Add For Blink RED LED when battery low in suspend mode */
void Battery_power_supply_change(void)
{
	power_supply_changed(&zeus_power_supplies[CHARGER_BATTERY]);
}
EXPORT_SYMBOL(Battery_power_supply_change);
/* } FIH, Michael Kao, 2009/06/08 */

void Battery_update_state(int _device, int device_state)
{
	if(_device==battery_charger_type)
		batt_state.F9_battery_charger_type=device_state;
	else if(_device==wifi_state)
		batt_state.F9_wifi_state=device_state;
	else if(_device==GPS_state)
		batt_state.F9_GPS_state=device_state;
	else if(_device==phone_state)
		batt_state.F9_phone_state=device_state;
}
EXPORT_SYMBOL(Battery_update_state);

static void zeus_battery_refresh_values(struct zeus_battery_update *zbu) {

	unsigned vbatt;
	u8 percent_now;

	/* [FXX_CR], add a retry mechanism to prevent the sudden high value*/
	vbatt=GetPMICBatteryInfo();
	if(weird_retry_flag)
	{
		weird_retry_flag=false;
		vbatt=GetPMICBatteryInfo();
		if(weird_retry_flag)
		{
			vbatt=GetPMICBatteryInfo();
		}
	}

	/* [FXX_CR], add event log*/
	percent_now=ChangeToVoltPercentage(vbatt);
	zbu->data.voltage=vbatt;
	msm_termal=GetPMIC_MSM_TERMAL();
	if(!gpio_get_value(GPIO_CHR_DET))
	{
		if(charging_bat_vol>4000)
			high_vol=1;
		else
			high_vol=0;
		/* [FXX_CR], charging full protection*/
		//Battery full protection
		//if(!battery_full_flag&&(val->intval==100))

		/* [FXX_CR], Improve charging full protection scenario*/
		if(vbatt>=4120)
		{
			if(battery_full_count==0)
				battery_full_start_time=get_seconds();
			battery_full_count++;
		}
		else
			battery_full_count=0;

		/* [FXX_CR], Modify for battery_full_falg abnormal changed*/
		if((battery_full_flag==0)&&(vbatt>=4120))
		{
			battery_full_time_duration=get_seconds()-battery_full_start_time;
			//pmlog("time_du=%d\n",battery_full_time_duration);
			if(charger_on==0)
			{
				if(battery_full_time_duration>=5400)
				{
					gpio_set_value(GPIO_CHR_EN,1);//Disable chager
					battery_full_flag=1;
					//pmlog("BAT_F\n");
					//eventlog("BAT_F\n") ;
				}
			}
		}
		//else if(battery_full_flag&&(val->intval<100)&&!over_temper)
		else if((battery_full_flag==1)&&(vbatt<=4120)&&!over_temper)
		{
			gpio_set_value(GPIO_CHR_EN,0);//Enable charger
			battery_full_flag=0;
			battery_full_flag2=true;
			//val->intval=100;
			//pmlog("BAT_NOF\n");
			//eventlog("BAT_NOF\n") ;
		}
		if(battery_full_flag2&&(percent_now<100)&&(percent_now>90))
			percent_now=100;
		//Temperature protection
		//New charging temperature protection scenario
#if 0
		if(!slight_over_temper&&(msm_termal>55))
		{
			//eventlog("sOVT\n") ;
			//pmlog("sOVT\n");
			if(batt_state.F9_battery_charger_type==2)
			{
				gpio_set_value(CHR_1A,0);//Set to 500mA
				gpio_set_value(USBSET,1);
			}
			slight_over_temper=true;
		}
		if(slight_over_temper&&(msm_termal>=5)&&(msm_termal<=52))
		{
			//eventlog("Recover\n") ;
			//pmlog("Recover\n");
			if(batt_state.F9_battery_charger_type==2)
			{
				gpio_set_value(CHR_1A,1);//Set to 1A
				gpio_set_value(USBSET,1);
			}
			slight_over_temper=false;
		}
#endif

		/* [FXX_CR], Add Greco battery protection scenario*/
#if GET_TEMPERATURE_FROM_BATTERY_THERMAL

		if (((g_orig_hw_id >= CMCS_125_CTP_GRE_PR1 && g_orig_hw_id <= CMCS_125_CTP_GRE_MP2)||
					(g_orig_hw_id >= CMCS_CTP_F917_PR2 && g_orig_hw_id <= CMCS_CTP_F917_MP3))&&g_use_battery_thermal)
			//if (g_orig_hw_id >= CMCS_HW_EVB1 && g_orig_hw_id < CMCS_7627_ORIG_EVB1&& g_use_battery_thermal)
		{
			printk("Battery Thermistor Case\n");
			if (!over_temper
					&& !over_temper2
					&& (msm_termal <= BATTERY_TEMP_LOW_LIMIT || msm_termal >= (BATTERY_TEMP_HIGH_LIMIT + BATTERY_THERMAL_TEMP_OFFSET))
			   )
			{
				//eventlog("OVT\n") ;	

				/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
				if(fihsensor_magnet_guard1 != 1) {
					fihsensor_magnet_guard1 = -1;
				}
#endif

				/* } FIH; Tiger; 2009/8/15 */
				gpio_set_value(GPIO_CHR_EN,1);//Disable chager
				over_temper=true;
				//pmlog("OVT\n");
			}

			if (over_temper
					&&(msm_termal >= (BATTERY_TEMP_LOW_LIMIT + RECHARGE_TEMP_OFFSET))
					&&(msm_termal <= (BATTERY_TEMP_HIGH_LIMIT - RECHARGE_TEMP_OFFSET))
			   )
			{
				/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
				if(fihsensor_magnet_guard1 != 1) {
					fihsensor_magnet_guard1 = -1;
				}
#endif

				/* } FIH; Tiger; 2009/8/15 */
				gpio_set_value(GPIO_CHR_EN,0);//Enable chager
				over_temper=false;
				//eventlog("Recharge\n") ;	
			}
		}
		else
		{
#endif  //end of GET_TEMPERATURE_FROM_BATTERY_THERMAL
			if(!over_temper&&!over_temper2&&(((msm_termal<=10)||(msm_termal>=60))&&(high_vol==1)))
			{
				//eventlog("OVT\n") ;	

				/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
				if(fihsensor_magnet_guard1 != 1) {
					fihsensor_magnet_guard1 = -1;
				}
#endif
				/* } FIH; Tiger; 2009/8/15 */
				/* [FXX_CR], add for not to disable charger*/
				if(charger_on==0)
					gpio_set_value(GPIO_CHR_EN,1);//Disable chager
				over_temper=true;
				//pmlog("OVT\n");
			}
			if(!over_temper&&!over_temper2&&(((msm_termal<=10)||(msm_termal>=70))&&(high_vol==0)))
			{
				//eventlog("OVT2\n") ;	

				/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
				if(fihsensor_magnet_guard1 != 1) {
					fihsensor_magnet_guard1 = -1;
				}
#endif
				/* } FIH; Tiger; 2009/8/15 */
				/* [FXX_CR], add for not to disable charger*/
				if(charger_on==0)
					gpio_set_value(GPIO_CHR_EN,1);//Disable chager
				over_temper2=true;
				//pmlog("OVT\n");
			}

			if(over_temper&&((msm_termal>=15)&&(msm_termal<=55)))
			{
				/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
				if(fihsensor_magnet_guard1 != 0) {
					fihsensor_magnet_guard1 = -1;
				}
#endif
				/* } FIH; Tiger; 2009/8/15 */
				gpio_set_value(GPIO_CHR_EN,0);//Enable chager
				over_temper=false;
				//eventlog("Recharge\n") ;	
			}
			if(over_temper2&&((msm_termal>=15)&&(msm_termal<=65)))
			{
				/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
				if(fihsensor_magnet_guard1 != 0) {
					fihsensor_magnet_guard1 = -1;
				}
#endif
				/* } FIH; Tiger; 2009/8/15 */
				gpio_set_value(GPIO_CHR_EN,0);//Enable chager
				over_temper2=false;
				//eventlog("Recharge2\n") ;	
			}

#if GET_TEMPERATURE_FROM_BATTERY_THERMAL
		}   //End of (Battery_HWID >= CMCS_125_CTP_GRE_PR1 && Battery_HWID <=CMCS_125_CTP_GRE_MP2 && g_use_battery_thermal)
#endif
	}
	else
		battery_full_flag2=false;
	//pmlog("Temp= %d\n", msm_termal);

	// +++ADQ_FIH+++
	if ((percent_now == 100)&&(g_charging_state == CHARGER_STATE_CHARGING)){
		g_charging_state = CHARGER_STATE_FULL;
	}
	else if (g_charging_state == CHARGER_STATE_FULL) {
		if (percent_now < 98) {
			g_charging_state = CHARGER_STATE_CHARGING;
		}
	}
	else if (g_charging_state == CHARGER_STATE_NOT_CHARGING){
		if (percent_now < 15) 
		{
			g_charging_state = CHARGER_STATE_LOW_POWER;
		}
	}

	printk(KERN_DEBUG "batt : %d - %d, EN = %d, 1A = %d, SET = %d\n", percent_now, g_charging_state, gpio_get_value(GPIO_CHR_EN), gpio_get_value(CHR_1A), gpio_get_value(USBSET));
	if ((g_charging_state_last != g_charging_state))
		tca6507_charger_state_report(g_charging_state);   
	zbu->data.capacity=percent_now;
	g_charging_state_last = g_charging_state;

}

static int zeus_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		// "Unknown", "Charging", "Discharging", "Not charging", "Full"
                	if (g_charging_state != CHARGER_STATE_LOW_POWER)
			val->intval = g_charging_state;
                	else 
                		val->intval = CHARGER_STATE_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		//GetBatteryInfo(BATT_AVCURRENT_INFO, &buf);
		// "Unknown", "Good", "Overheat", "Dead", "Over voltage", "Unspecified failure"
		#if GET_TEMPERATURE_FROM_BATTERY_THERMAL
		//if ((g_orig_hw_id >= CMCS_125_CTP_GRE_PR1 && g_orig_hw_id <= CMCS_125_CTP_GRE_MP2)||
		//(g_orig_hw_id >= CMCS_CTP_F917_PR2 && g_orig_hw_id <= CMCS_CTP_F917_MP3))
		if (g_orig_hw_id >= CMCS_HW_EVB1 && g_orig_hw_id < CMCS_7627_ORIG_EVB1)
		{
            		if (g_use_battery_thermal)
            		{
               	 		if (g_cool_down_mode)
                			{
                    			if (msm_termal < BATTERY_TEMP_COOL_DOWN_FROM_EMGCY)
                    			{
                        				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
                        				g_cool_down_mode = FALSE;
                    			}
                    			else
                        				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT_EMGCY_CALL_ONLY;
                			}
                			else
                			{
                    			if (msm_termal > BATTERY_TEMP_EMGCY_CALL_ONLY)
                    			{
                        				g_cool_down_mode = TRUE;    //Enter cool-down mode by moto's request
                        				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT_EMGCY_CALL_ONLY;
                    			}
                    			else if (msm_termal > BATTERY_TEMP_SHUTDOWN_AP && msm_termal <= BATTERY_TEMP_EMGCY_CALL_ONLY)
                        				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT_SHUTDOWN_AP;
                    			else if (over_temper)   // (BATTERY_TEMP_HIGH_LIMIT + BATTERY_THERMAL_TEMP_OFFSET) <= msm_termal <= BATTERY_TEMP_LOW_LIMIT
                        				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
                    			else
                        				val->intval = POWER_SUPPLY_HEALTH_GOOD;
                			}
           		 }
            		else
            		{   //  MSM thermistor case
               			if (g_cool_down_mode)
                			{
                    			if (msm_termal < MSM_TEMP_COOL_DOWN_FROM_EMGCY)
                    			{
                        				g_cool_down_mode = FALSE;
                        				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
                    			}
                    			else
                        				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT_EMGCY_CALL_ONLY;
               			}
                			else
                			{
                		
                   			if (msm_termal > MSM_TEMP_EMGCY_CALL_ONLY)
                    			{
                        				g_cool_down_mode = TRUE;
                        				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT_EMGCY_CALL_ONLY;
                    			}
                    			else if (msm_termal > MSM_TEMP_SHUTDOWN_AP && msm_termal <= MSM_TEMP_EMGCY_CALL_ONLY)
                        				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT_SHUTDOWN_AP;
                    			else if (over_temper||over_temper2)
            					val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
            				else
            					val->intval = POWER_SUPPLY_HEALTH_GOOD;
               			}
            		}
            		printk("battery_thermal=%d health=%d cool-down=%d msm_termal=%d\n", g_use_battery_thermal, val->intval, g_cool_down_mode,msm_termal);
        		}
        		else
        		#endif
        		{
    			if(over_temper||over_temper2)
    				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
    			else
    				val->intval = POWER_SUPPLY_HEALTH_GOOD;
        		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		//GetBatteryInfo(BATT_CURRENT_INFO, &buf);
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		// "Unknown", "NiMH", "Li-ion", "Li-poly", "LiFe", "NiCd", "LiMn"
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = msm_termal*10;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = batt_update->data.voltage*1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = batt_update->data.capacity;
                break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#define BATTERY_POLLING_TIMER  60//300000 10000

static void zeus_battery_alarm(struct alarm *alarm)
{
	struct zeus_battery_update *zbu =
		container_of(alarm, struct zeus_battery_update, alarm);

	wake_lock(&zbu->wakelock);

	queue_work(zbu->wqueue, &zbu->zeus_batt_work);
}

static void zeus_battery_work(struct work_struct *work)
{
	struct zeus_battery_update *zbu =
		container_of(work, struct zeus_battery_update, zeus_batt_work);
	struct timespec ts;
	unsigned long flags;
	unsigned capacity;

	if(charger_on==1)
		g_use_battery_thermal=FALSE;
	else
		g_use_battery_thermal=g_pre_use_battery_thermal;

	zbu->last_poll = alarm_get_elapsed_realtime();


	ts = ktime_to_timespec(zbu->last_poll);

	capacity = zbu->data.capacity;
	zeus_battery_refresh_values(zbu);
	/* Notify userspace */
	if (capacity != zbu->data.capacity)
		power_supply_changed(&zeus_power_supplies[CHARGER_BATTERY]);

	/* prevent suspend before starting the alarm */
	local_irq_save(flags);
	wake_unlock(&zbu->wakelock);

	zeus_program_alarm(zbu, get_suspend_state() == PM_SUSPEND_MEM ? 
			(BATTERY_POLLING_TIMER*10) : BATTERY_POLLING_TIMER);
	local_irq_restore(flags);

}

#ifdef T_FIH	///+T_FIH

#ifdef FLAG_CHARGER_DETECT
#include <asm/gpio.h>

#define GPIO_CHR_DET 39		// Input power-good (USB port/adapter present indicator) pin
#define GPIO_CHR_FLT 32		// Over-voltage fault flag

static irqreturn_t chgdet_irqhandler(int irq, void *dev_id)
{
	g_charging_state = (gpio_get_value(GPIO_CHR_DET)) ? CHARGER_STATE_NOT_CHARGING : CHARGER_STATE_CHARGING;
	/* FIH, Michael Kao, 2009/08/14{ */
	/* [FXX_CR], Not update battery information when charger state changes*/
	charger_state_change=true;
	system_time_second=get_seconds();
	//power_supply_changed(g_ps_battery);
	power_supply_changed(&zeus_power_supplies[CHARGER_BATTERY]);
	
	/* FIH, Michael Kao, 2009/08/13{ */
	return IRQ_HANDLED;
}
#endif	// FLAG_CHARGER_DETECT
#endif	// T_FIH	///-T_FIH
#ifdef CONFIG_FIH_FXX
/*Add misc device ioctl command functions*/
/*******************File control function******************/
// devfs
static int Zeus_battery_miscdev_open( struct inode * inode, struct file * file )
{
	if( ( file->f_flags & O_ACCMODE ) == O_WRONLY )
	{
		return -1;
	}
	else
		return 0;
}

static int Zeus_battery_miscdev_ioctl( struct inode * inode, struct file * filp, unsigned int cmd2, unsigned long arg )
{
    	int ret = 0;
	int data=0;
    	//uint8_t BatteryID;
    	#if 1
    	if(copy_from_user(&data, (int __user*)arg, sizeof(int)))
    	{
        		return -1;
    	}
    	#endif
    
    	if(cmd2 == FTMBATTERY_VOL)
    	{
		ret = 3800;
    	}
	else if(cmd2 ==FTMBATTERY_PRE_VOL)
	{
		ret =PMIC_batt.pre_batt_val;
	}
	else if(cmd2 == FTMBATTERY_VBAT)
	{
		ret=VBAT_from_SMEM;
	}
	else if(cmd2 == FTMBATTERY_BID || cmd2==FTMBATTERY_STA)
	{
		ret = 0;
	}
    	else
    	{
        	printk(KERN_DEBUG  "[%s:%d]Unknow ioctl cmd", __func__, __LINE__);
        	ret = -1;
    	}
    	return ret;
}


static int Zeus_battery_miscdev_release( struct inode * inode, struct file * filp )
{
	return 0;
}


static const struct file_operations Zeus_battery_miscdev_fops = {
	.open = Zeus_battery_miscdev_open,
	.ioctl = Zeus_battery_miscdev_ioctl,
	.release = Zeus_battery_miscdev_release,
};


static struct miscdevice Zeus_battery_miscdev = {
 	.minor = MISC_DYNAMIC_MINOR,
	.name = "ftmbattery",
	.fops = &Zeus_battery_miscdev_fops,
};
#endif


static int goldfish_battery_probe(struct platform_device *pdev)
{
       int ret, i;
	struct zeus_battery_update *zbu;

	zbu = kzalloc(sizeof(*zbu), GFP_KERNEL);
	if (!zbu)
		return -ENOMEM;

	platform_set_drvdata(pdev, zbu);

	msm_termal=10;
	over_temper=false;
	slight_over_temper=false;
	/* FIH, Michael Kao, 2009/08/14{ */
	/* [FXX_CR], Not update battery information when charger state changes*/
	charger_state_change=false;
	time_duration=0;
	/* FIH, Michael Kao, 2009/08/14{ */
	/* FIH, Michael Kao, 2009/09/10{ */
	/* [FXX_CR], charging full protection*/
	
	/* FIH, Michael Kao, 2009/12/01{ */
	/* [FXX_CR], add for update battery information more accuracy only for battery update in suspend mode*/
	battery_full_flag=0;
	/* FIH, Michael Kao, 2009/12/01{ */
	
	battery_full_count=0;
	battery_full_flag2=false;
	pre_val=0;
	pre_time_cycle=0;
	
	/* FIH, Michael Kao, 2009/09/30{ */
	/* [FXX_CR], add for update battery information more accuracy in suspend mode*/
	suspend_update_flag=false;
	/* FIH, Michael Kao, 2009/09/30{ */
	
	/* FIH, Michael Kao, 2009/10/14{ */
	/* [FXX_CR], Add wake lock to avoid incompleted update battery information*/
	/* FIH, Michael Kao, 2009/10/14{ */
	battery_low=false;
	VBAT_from_SMEM=0;
	
	/* FIH, Michael Kao, 2009/11/25{ */
	/* [FXX_CR], add a retry mechanism to prevent the sudden high value*/
	weird_retry_flag=false;
	/* FIH, Michael Kao, 2009/11/25{ */
	
	/* FIH, Michael Kao, 2009/12/01{ */
	/* [FXX_CR], add for update battery information more accuracy only for battery update in suspend mode*/
	pre_suspend_time=0;
	/* FIH, Michael Kao, 2009/12/01{ */
	
	/* FIH, Michael Kao, 2009/12/25{ */
	/* [FXX_CR], add for new charging temperature protection scenario*/
	high_vol=0;
	charging_bat_vol=0;
	over_temper2=false;
	suspend_update_sample_gate=0;
	charger_on=0;
	/* FIH, Michael Kao, 2009/12/25{ */

	/* FIH, Michael Kao, 2009/09/10{ */
	/* FIH, Michael Kao, 2009/07/16{ */
	/* [FXX_CR], Modify for different smem channel with different modem image*/
	if(Battery_HWID >= CMCS_7627_EVB1)
	{
		Modem_mode = FIH_READ_NETWORK_MODE_FROM_SMEM();
	}
    #if GET_TEMPERATURE_FROM_BATTERY_THERMAL
    g_orig_hw_id = FIH_READ_ORIG_HWID_FROM_SMEM();

	if ((g_orig_hw_id >= CMCS_125_CTP_GRE_PR1 && g_orig_hw_id <= CMCS_125_CTP_GRE_MP2)||
	(g_orig_hw_id >= CMCS_CTP_F917_PR2 && g_orig_hw_id <= CMCS_CTP_F917_MP3))
	//if (g_orig_hw_id >= CMCS_HW_EVB1 && g_orig_hw_id < CMCS_7627_ORIG_EVB1)
    {
        int adc_read_id = 0x10; //Use the ADC_BATT_THERM_DEGC channel of 7227 modem
        int battery_thermal = 0;

        printk("Battery adc_read_id %d\n", adc_read_id);
        battery_thermal = proc_comm_read_adc(&adc_read_id);
        printk("Battery battery_thermal %d\n", battery_thermal);
	/* FIH, Michael Kao, 2010/05/14{ */
	/* [FXX_CR], add for not to disable charger*/
        if (battery_thermal >= (OLD_BATTERY_RESISTOR_TEMP - OLD_BATTERY_RESISTOR_TEMP_TOL) 
            && battery_thermal <= (OLD_BATTERY_RESISTOR_TEMP + OLD_BATTERY_RESISTOR_TEMP_TOL))
        {
            g_pre_use_battery_thermal = FALSE;
            g_use_battery_thermal = FALSE;
            printk("Warning! Maybe no battery thermistor!\n");
        }
        else
        {
        	   g_pre_use_battery_thermal = TRUE;
            g_use_battery_thermal = TRUE;
            printk("Battery thermistor exists\n");
        }
	/* FIH, Michael Kao, 2010/05/14{ */
    }
    #endif

	/* FIH, Michael Kao, 2009/07/16{ */
	if(!gpio_get_value(GPIO_CHR_DET)) {
		/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
		if(fihsensor_magnet_guard1 != 0) {
			fihsensor_magnet_guard1 = -1;
		}
#endif
		/* } FIH; Tiger; 2009/8/15 */
		gpio_set_value(GPIO_CHR_EN,0);//Enable chager
	}
	else { 
		/* FIH; Tiger; 2009/8/15 { */
#ifdef FEATURE_ECOMPASS
		if(fihsensor_magnet_guard1 != 1) {
			fihsensor_magnet_guard1 = -1;
		}
#endif
		/* } FIH; Tiger; 2009/8/15 */
		gpio_set_value(GPIO_CHR_EN,1);//Disable chager
	}
		
    /* init power supplier framework */
    for (i = 0; i < ARRAY_SIZE(zeus_power_supplies); i++) {
        ret = power_supply_register(&pdev->dev, &zeus_power_supplies[i]);
        if (ret) {
            printk(KERN_ERR "Failed to register power supply (%d)\n", ret);
                   return ret;
               }
    }

	INIT_WORK(&zbu->zeus_batt_work, zeus_battery_work);
	zbu->wqueue = create_freezeable_workqueue(dev_name(&pdev->dev));
	zbu->last_poll = alarm_get_elapsed_realtime();

	if (!zbu->wqueue)
		return -ENOMEM;

		
	wake_lock_init(&zbu->wakelock, WAKE_LOCK_SUSPEND, "zeus_battery");

	batt_update = zbu;

	/* Init the values */
	zeus_battery_refresh_values(zbu);

	alarm_init(&zbu->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
		zeus_battery_alarm);
	queue_work(zbu->wqueue, &zbu->zeus_batt_work);

#ifdef T_FIH	///+T_FIH

#ifdef FLAG_CHARGER_DETECT
	gpio_tlmm_config( GPIO_CFG(GPIO_CHR_DET, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE );
	ret = gpio_request(GPIO_CHR_DET, "gpio_keybd_irq");
	ret = gpio_direction_input(GPIO_CHR_DET);
	ret = request_irq(MSM_GPIO_TO_INT(GPIO_CHR_DET), &chgdet_irqhandler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, pdev->name, NULL);
	/* FIH, Michael Kao, 2009/10/14{ */
	/* [FXX_CR], Add charger detect pin to wake up source*/
	if (!ret)
		enable_irq_wake(MSM_GPIO_TO_INT(GPIO_CHR_DET));
	/* FIH, Michael Kao, 2009/10/14{ */

#endif	// FLAG_CHARGER_DETECT
#endif	// T_FIH	///-T_FIH

	return 0;

}

static int goldfish_battery_remove(struct platform_device *pdev)
{

	int i;
	struct zeus_battery_update *zbu = platform_get_drvdata(pdev);

#ifdef T_FIH	///+T_FIH
#ifdef FLAG_CHARGER_DETECT
	free_irq(MSM_GPIO_TO_INT(GPIO_CHR_DET), NULL);
	gpio_free(GPIO_CHR_DET);
#endif	// FLAG_CHARGER_DETECT

#endif	// T_FIH	///-T_FIH

	for (i = 0; i < ARRAY_SIZE(zeus_power_supplies); i++) {
		power_supply_unregister(&zeus_power_supplies[i]);
	}


	kfree(zbu);
	batt_update = NULL;

	return 0;
}

#ifdef CONFIG_USB_GADGET_MSM_72K 
void zeus_update_usb_status(enum chg_type chgtype) {

       /* Prepare enabler/USB/1A GPIOs */
       /* Prepare enabler/USB/1A GPIOs */
       int rc = gpio_request(GPIO_CHR_EN, "CHR_EN");
       if (rc) printk(KERN_ERR "%s: GPIO_CHR_EN setting failed! rc = %d\n", __func__, rc);
       rc = gpio_request(USBSET, "USBSET");
       if (rc) printk(KERN_ERR "%s: USBSET setting failed! rc = %d\n", __func__, rc);
       rc = gpio_request(CHR_1A, "CHR_1A");
       if (rc) printk(KERN_ERR "%s: CHR_1A setting failed! rc = %d\n", __func__, rc);


       switch (chgtype) {
               case USB_CHG_TYPE__WALLCHARGER:
                       /* Turn on charger IC */
                       /* Set the charging current to 1A */
                       gpio_set_value(GPIO_CHR_EN,0);
                       gpio_set_value(CHR_1A,1);
                       gpio_set_value(USBSET,1);
                       current_charger = CHARGER_AC;
                       break;
               case USB_CHG_TYPE__CARKIT:
                       /* Turn on charger IC */
                       /* Set the charging current to 100mA */
                       gpio_set_value(GPIO_CHR_EN,0);
                       gpio_set_value(CHR_1A,0);
                       gpio_set_value(USBSET,1);
                       current_charger = CHARGER_AC;
                       break;
               case USB_CHG_TYPE__SDP:
                       /* Turn on charger IC */
                       /* Set the charging current to 100mA */
                       gpio_set_value(GPIO_CHR_EN,0);
                       gpio_set_value(CHR_1A,0);
                       gpio_set_value(USBSET,1);
                       current_charger = CHARGER_USB;
                       break;
               case USB_CHG_TYPE__INVALID:
               default:
                       /* Turn off charger IC */
                       gpio_set_value(GPIO_CHR_EN,1);
                       gpio_set_value(USBSET,0);
                       current_charger = CHARGER_BATTERY;
                       break;
       }
       gpio_free(GPIO_CHR_EN);
       gpio_free(USBSET);
       gpio_free(CHR_1A);

       power_supply_changed(&zeus_power_supplies[current_charger]);
}
EXPORT_SYMBOL(zeus_update_usb_status);
#endif


static struct platform_driver goldfish_battery_device = {
	.probe		= goldfish_battery_probe,
	.remove		= goldfish_battery_remove,
	.driver = {
		.name = "goldfish-battery",
	}
};

static int __init goldfish_battery_init(void)
{
    int ret;

     gpio_tlmm_config(GPIO_CFG(26, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
     gpio_tlmm_config(GPIO_CFG(33, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
     gpio_tlmm_config(GPIO_CFG(123, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

    ret = platform_driver_register(&goldfish_battery_device);
    if(ret)
    {
        goto ERROR;
    }
	
    #ifdef CONFIG_FIH_FXX
        /*Use miscdev*/
        //register and allocate device, it would create an device node automatically.
        //use misc major number plus random minor number, and init device
        ret = misc_register(&Zeus_battery_miscdev);
        if (ret)
        {
            misc_deregister(&Zeus_battery_miscdev);
        }        
    #endif
    
    ERROR:    
	return ret;
    
}

static void __exit goldfish_battery_exit(void)
{
	platform_driver_unregister(&goldfish_battery_device);

	#ifdef CONFIG_FIH_FXX
    	/*new label name for remove misc device*/
		misc_deregister(&Zeus_battery_miscdev);	
	#endif
}

module_init(goldfish_battery_init);
module_exit(goldfish_battery_exit);

MODULE_AUTHOR("Mike Lockwood lockwood@android.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Battery driver for the Goldfish emulator");
