/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>

#include <mach/msm_rpcrouter.h>
#include <mach/msm_battery.h>

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VERS	0x00010001

#define CHG_RPC_PROG		0x3000001a
#define CHG_RPC_VERS		0x00010001


#define BATTERY_REGISTER_PROC                          	2
#define BATTERY_MODIFY_CLIENT_PROC                     	4
#define BATTERY_DEREGISTER_CLIENT_PROC			5
#define BATTERY_READ_MV_PROC 				12
#define BATTERY_ENABLE_DISABLE_FILTER_PROC 		14

#define VBATT_FILTER			2

#define BATTERY_CB_TYPE_PROC 		1
#define BATTERY_CB_ID_ALL_ACTIV       	1
#define BATTERY_CB_ID_LOW_VOL		2

#define BATTERY_LOW            	2800
#define BATTERY_HIGH           	4300

#define ONCRPC_CHG_GET_GENERAL_STATUS_PROC 	12
#define ONCRPC_CHARGER_API_VERSIONS_PROC 	0xffffffff

#define CHARGER_API_VERSION  			0x00010003
#define DEFAULT_CHARGER_API_VERSION		0x00010001

#define BATT_RPC_TIMEOUT    10000	/* 10 sec */

#define INVALID_BATT_HANDLE    -1

#define RPC_TYPE_REQ     0
#define RPC_TYPE_REPLY   1
#define RPC_REQ_REPLY_COMMON_HEADER_SIZE   (3 * sizeof(uint32_t))


#define DEBUG  0

#if DEBUG
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do {} while (0)
#endif


enum {
	BATTERY_REGISTRATION_SUCCESSFUL = 0,
	BATTERY_DEREGISTRATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_MODIFICATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_INTERROGATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_CLIENT_TABLE_FULL = 1,
	BATTERY_REG_PARAMS_WRONG = 2,
	BATTERY_DEREGISTRATION_FAILED = 4,
	BATTERY_MODIFICATION_FAILED = 8,
	BATTERY_INTERROGATION_FAILED = 16,
	/* Client's filter could not be set because perhaps it does not exist */
	BATTERY_SET_FILTER_FAILED         = 32,
	/* Client's could not be found for enabling or disabling the individual
	 * client */
	BATTERY_ENABLE_DISABLE_INDIVIDUAL_CLIENT_FAILED  = 64,
	BATTERY_LAST_ERROR = 128,
};

enum {
	BATTERY_VOLTAGE_UP = 0,
	BATTERY_VOLTAGE_DOWN,
	BATTERY_VOLTAGE_ABOVE_THIS_LEVEL,
	BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
	BATTERY_VOLTAGE_LEVEL,
	BATTERY_ALL_ACTIVITY,
	VBATT_CHG_EVENTS,
	BATTERY_VOLTAGE_UNKNOWN,
};

/*
 * This enum contains defintions of the charger hardware status
 */
enum chg_charger_status_type {
    /* The charger is good      */
    CHARGER_STATUS_GOOD,
    /* The charger is bad       */
    CHARGER_STATUS_BAD,
    /* The charger is weak      */
    CHARGER_STATUS_WEAK,
    /* Invalid charger status.  */
    CHARGER_STATUS_INVALID
};

static char *charger_status[] = {
	"good", "bad", "weak", "invalid"
};

/*
 *This enum contains defintions of the charger hardware type
 */
enum chg_charger_hardware_type {
    /* The charger is removed                 */
    CHARGER_TYPE_NONE,
    /* The charger is a regular wall charger   */
    CHARGER_TYPE_WALL,
    /* The charger is a PC USB                 */
    CHARGER_TYPE_USB_PC,
    /* The charger is a wall USB charger       */
    CHARGER_TYPE_USB_WALL,
    /* The charger is a USB carkit             */
    CHARGER_TYPE_USB_CARKIT,
    /* Invalid charger hardware status.        */
    CHARGER_TYPE_INVALID
};

static char *charger_type[] = {
	"No charger", "wall", "USB PC", "USB wall", "USB car kit",
	"invalid charger"
};

/*
 *  This enum contains defintions of the battery status
 */
enum chg_battery_status_type {
    /* The battery is good        */
    BATTERY_STATUS_GOOD,
    /* The battery is cold/hot    */
    BATTERY_STATUS_BAD_TEMP,
    /* The battery is bad         */
    BATTERY_STATUS_BAD,
    /* Invalid battery status.    */
    BATTERY_STATUS_INVALID
};

static char *battery_status[] = {
	"good ", "bad temperature", "bad", "invalid"
};


/*
 *This enum contains defintions of the battery voltage level
 */
enum chg_battery_level_type {
    /* The battery voltage is dead/very low (less than 3.2V)        */
    BATTERY_LEVEL_DEAD,
    /* The battery voltage is weak/low (between 3.2V and 3.4V)      */
    BATTERY_LEVEL_WEAK,
    /* The battery voltage is good/normal(between 3.4V and 4.2V)  */
    BATTERY_LEVEL_GOOD,
    /* The battery voltage is up to full (close to 4.2V)            */
    BATTERY_LEVEL_FULL,
    /* Invalid battery voltage level.                               */
    BATTERY_LEVEL_INVALID
};

static char *battery_level[] = {
	"dead", "weak", "good", "full", "invalid"
};


struct rpc_reply_batt_chg {
	struct rpc_reply_hdr hdr;
	u32 	more_data;

	u32	charger_status;
	u32	charger_type;
	u32	battery_status;
	u32	battery_level;
	u32     battery_voltage;
	u32	battery_temp;
};

static struct rpc_reply_batt_chg rep_batt_chg;

struct msm_battery_info {

	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 chg_api_version;
	u32 batt_technology;

	u32 avail_chg_sources;
	u32 current_chg_source;

	u32 batt_status;
	u32 batt_health;
	u32 charger_valid;
	u32 batt_valid;
	u32 batt_capacity; /* in percentage */

	s32 charger_status;
	s32 charger_type;
	s32 battery_status;
	s32 battery_level;
	s32 battery_voltage; /* in millie volts */
	s32 battery_temp;  /* in celsius */

	u32(*calculate_capacity) (u32 voltage);

	s32 batt_handle;

	struct power_supply *msm_psy_ac;
	struct power_supply *msm_psy_usb;
	struct power_supply *msm_psy_batt;

	struct msm_rpc_client *batt_client;
	struct msm_rpc_endpoint *chg_ep;

	wait_queue_head_t wait_q;

	u32 vbatt_modify_reply_avail;

	struct early_suspend early_suspend;
};

static struct msm_battery_info msm_batt_info = {
	.batt_handle = INVALID_BATT_HANDLE,
	.charger_status = CHARGER_STATUS_INVALID,
	.charger_type = CHARGER_TYPE_INVALID,
	.battery_status = BATTERY_STATUS_GOOD,
	.battery_level = BATTERY_LEVEL_FULL,
	.battery_voltage = BATTERY_HIGH,
	.batt_capacity = 100,
	.batt_status = POWER_SUPPLY_STATUS_DISCHARGING,
	.batt_health = POWER_SUPPLY_HEALTH_GOOD,
	.batt_valid  = 1,
	.battery_temp = 23,
	.vbatt_modify_reply_avail = 0,
};

static enum power_supply_property msm_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] = {
	"battery",
};

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:

		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {

			val->intval = msm_batt_info.current_chg_source & AC_CHG
			    ? 1 : 0;
			DBG("%s(): power supply = %s online = %d\n"
					, __func__, psy->name, val->intval);

		}

		if (psy->type == POWER_SUPPLY_TYPE_USB) {

			val->intval = msm_batt_info.current_chg_source & USB_CHG
			    ? 1 : 0;

			DBG("%s(): power supply = %s online = %d\n"
					, __func__, psy->name, val->intval);
		}

		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static struct power_supply msm_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static void msm_batt_external_power_changed(struct power_supply *psy)
{
	printk(KERN_NOTICE "%s() : external power supply changed for %s\n",
			__func__, psy->name);
	power_supply_changed(psy);
}

static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	switch (psp) {

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = msm_batt_info.batt_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = msm_batt_info.batt_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = msm_batt_info.batt_valid;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = msm_batt_info.batt_technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_batt_info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_batt_info.voltage_min_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = msm_batt_info.battery_voltage;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = msm_batt_info.batt_capacity;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
	.external_power_changed = msm_batt_external_power_changed,
};

struct msm_batt_get_volt_ret_data {
	u32 battery_voltage;
};

static int msm_batt_get_volt_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct msm_batt_get_volt_ret_data *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_get_volt_ret_data *)data;
	buf_ptr = (struct msm_batt_get_volt_ret_data *)buf;

	data_ptr->battery_voltage = be32_to_cpu(buf_ptr->battery_voltage);

	return 0;
}

static u32 msm_batt_get_vbatt_voltage(void)
{
	int rc;

	struct msm_batt_get_volt_ret_data rep;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_READ_MV_PROC,
			NULL, NULL,
			msm_batt_get_volt_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s : ERROR. vbatt get volt returned error = %d\n",
				__func__, rc);
		return rc ;
	}

	if (msm_batt_info.battery_voltage == rep.battery_voltage) {

		printk(KERN_NOTICE" No charger. Batt Volt = %u."
				" No change in voltage.\n",
				msm_batt_info.battery_voltage);
		return 0;
	}

	msm_batt_info.battery_voltage = rep.battery_voltage;

	msm_batt_info.batt_capacity = msm_batt_info.calculate_capacity(
					msm_batt_info.battery_voltage);

	msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
	msm_batt_info.batt_valid  = 1 ;
	msm_batt_info.battery_temp = 23;

	printk(KERN_INFO "%s() : No charger. Batt Volt = %u.\n",
			 __func__, msm_batt_info.battery_voltage);

	power_supply_changed(&msm_psy_batt);

	return 0;
}

static int msm_batt_get_batt_chg_status(void)
{
	int rc ;
	struct rpc_req_batt_chg {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_batt_chg;

	req_batt_chg.more_data = cpu_to_be32(1);

	memset(&rep_batt_chg, 0, sizeof(rep_batt_chg));

	rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
				ONCRPC_CHG_GET_GENERAL_STATUS_PROC,
				&req_batt_chg, sizeof(req_batt_chg),
				&rep_batt_chg, sizeof(rep_batt_chg),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		pr_err("%s: ERROR. msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, ONCRPC_CHG_GET_GENERAL_STATUS_PROC, rc);
		return rc;
	} else if (be32_to_cpu(rep_batt_chg.more_data)) {

		rep_batt_chg.charger_status =
			be32_to_cpu(rep_batt_chg.charger_status);

		rep_batt_chg.charger_type =
			be32_to_cpu(rep_batt_chg.charger_type);

		rep_batt_chg.battery_status =
			be32_to_cpu(rep_batt_chg.battery_status);

		rep_batt_chg.battery_level =
			be32_to_cpu(rep_batt_chg.battery_level);

		rep_batt_chg.battery_voltage =
			be32_to_cpu(rep_batt_chg.battery_voltage);

		rep_batt_chg.battery_temp =
			be32_to_cpu(rep_batt_chg.battery_temp);

	} else {
		printk(KERN_INFO "%s():No more data in batt_chg rpc reply\n",
				__func__);
		return -EIO;
	}

	return 0;
}

static void msm_batt_update_psy_status(void)
{
	static u32 unnecessary_event_count;

	msm_batt_get_batt_chg_status();

	if (rep_batt_chg.charger_type == CHARGER_TYPE_INVALID  ||
		rep_batt_chg.charger_type == CHARGER_TYPE_NONE ||
		rep_batt_chg.charger_status == CHARGER_STATUS_INVALID ||
		rep_batt_chg.battery_status == BATTERY_STATUS_INVALID ||
		rep_batt_chg.battery_level == BATTERY_LEVEL_INVALID) {

		printk(KERN_INFO "charger_type = %s, get Vbatt voltage \n",
				charger_type[rep_batt_chg.charger_type]);

		if (rep_batt_chg.charger_type == CHARGER_TYPE_INVALID  ||
			rep_batt_chg.charger_type == CHARGER_TYPE_NONE) {

			msm_batt_info.charger_type = rep_batt_chg.charger_type;
			msm_batt_info.current_chg_source &= ~(AC_CHG | USB_CHG);
		}

		msm_batt_get_vbatt_voltage();
		return;
	}

	if (msm_batt_info.charger_status == rep_batt_chg.charger_status &&
		msm_batt_info.charger_type == rep_batt_chg.charger_type &&
		msm_batt_info.battery_status ==  rep_batt_chg.battery_status &&
		msm_batt_info.battery_level ==  rep_batt_chg.battery_level &&
		msm_batt_info.battery_voltage  == rep_batt_chg.battery_voltage
		&& msm_batt_info.battery_temp ==  rep_batt_chg.battery_temp) {

		/* Got unnecessary event from Modem PMIC VBATT driver.
		 * Nothing changed in Battery or charger status.
		 */

		unnecessary_event_count++;

		printk(KERN_NOTICE "%s() : unnecessary_event_count = %u\n",
				__func__, unnecessary_event_count);

		return;
	}

	unnecessary_event_count = 0;

	printk(KERN_INFO "charger_status = %s, charger_type = %s,"
				" batt_status = %s, batt_level = %s,"
				" batt_volt = %u, batt_temp = %u,\n",
				charger_status[rep_batt_chg.charger_status],
				charger_type[rep_batt_chg.charger_type],
				battery_status[rep_batt_chg.battery_status],
				battery_level[rep_batt_chg.battery_level],
				rep_batt_chg.battery_voltage,
				rep_batt_chg.battery_temp);


	msm_batt_info.battery_voltage  	= 	rep_batt_chg.battery_voltage;
	msm_batt_info.battery_temp 	=	rep_batt_chg.battery_temp;


	if (rep_batt_chg.battery_status != BATTERY_STATUS_INVALID) {

		msm_batt_info.batt_valid = 1;

		if (msm_batt_info.battery_voltage >
		    msm_batt_info.voltage_max_design) {

			msm_batt_info.batt_health =
			    POWER_SUPPLY_HEALTH_OVERVOLTAGE;

			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;

		} else if (msm_batt_info.battery_voltage
			 < msm_batt_info.voltage_min_design) {

			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_DEAD;
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_UNKNOWN;

		} else if (rep_batt_chg.battery_status == BATTERY_STATUS_BAD) {

			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_DEAD;
			msm_batt_info.batt_status = POWER_SUPPLY_STATUS_UNKNOWN;

		} else if (rep_batt_chg.battery_status ==
				BATTERY_STATUS_BAD_TEMP) {

			msm_batt_info.batt_health =
				POWER_SUPPLY_HEALTH_OVERHEAT;

			if (rep_batt_chg.charger_status == CHARGER_STATUS_BAD
				|| rep_batt_chg.charger_status ==
				CHARGER_STATUS_INVALID)
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_UNKNOWN;
			else
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;

		} else if ((rep_batt_chg.charger_status == CHARGER_STATUS_GOOD
			|| rep_batt_chg.charger_status == CHARGER_STATUS_WEAK)
			&& (rep_batt_chg.battery_status ==
				BATTERY_STATUS_GOOD)) {

			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;

			if (rep_batt_chg.battery_level == BATTERY_LEVEL_FULL)
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_FULL;
			else
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_CHARGING;

		} else if ((rep_batt_chg.charger_status == CHARGER_STATUS_BAD
				|| rep_batt_chg.charger_status ==
				CHARGER_STATUS_INVALID) &&
				(rep_batt_chg.battery_status ==
					BATTERY_STATUS_GOOD)) {

			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_DISCHARGING;
		}

		msm_batt_info.batt_capacity =
			msm_batt_info.calculate_capacity(
					msm_batt_info.battery_voltage);

	} else {
		msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
		msm_batt_info.batt_status = POWER_SUPPLY_STATUS_UNKNOWN;
		msm_batt_info.batt_capacity = 0;
	}


	if (msm_batt_info.charger_type != rep_batt_chg.charger_type) {

		msm_batt_info.charger_type = rep_batt_chg.charger_type ;

		if (msm_batt_info.charger_type == CHARGER_TYPE_WALL) {

			msm_batt_info.current_chg_source &= ~USB_CHG;
			msm_batt_info.current_chg_source |= AC_CHG;

			printk(KERN_INFO "%s() : charger_type = WALL\n",
					__func__);

			power_supply_changed(&msm_psy_ac);

		} else if (msm_batt_info.charger_type ==
				CHARGER_TYPE_USB_WALL ||
				msm_batt_info.charger_type ==
				CHARGER_TYPE_USB_PC) {

			msm_batt_info.current_chg_source &= ~AC_CHG;
			msm_batt_info.current_chg_source |= USB_CHG;

			printk(KERN_INFO "%s() : charger_type = %s\n",
			__func__,
			charger_type[msm_batt_info.charger_type]);

			power_supply_changed(&msm_psy_usb);

		} else {

			printk(KERN_INFO "%s() : charger_type = %s\n",
			__func__,
			charger_type[msm_batt_info.charger_type]);

			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_DISCHARGING;

			if (msm_batt_info.current_chg_source & AC_CHG) {

				msm_batt_info.current_chg_source &= ~AC_CHG;

				printk(KERN_INFO "%s() : AC WALL charger"
					" removed\n", __func__);

				power_supply_changed(&msm_psy_ac);

			} else if (msm_batt_info.current_chg_source & USB_CHG) {

				msm_batt_info.current_chg_source &= ~USB_CHG;
				printk(KERN_INFO "%s() : USB charger removed\n",
						__func__);

				power_supply_changed(&msm_psy_usb);
			} else
				power_supply_changed(&msm_psy_batt);
		}
	} else
		power_supply_changed(&msm_psy_batt);

	msm_batt_info.charger_status 	= 	rep_batt_chg.charger_status;
	msm_batt_info.charger_type 	= 	rep_batt_chg.charger_type;
	msm_batt_info.battery_status 	=  	rep_batt_chg.battery_status;
	msm_batt_info.battery_level 	=  	rep_batt_chg.battery_level;
	msm_batt_info.battery_voltage  	= 	rep_batt_chg.battery_voltage;
	msm_batt_info.battery_temp 	=	rep_batt_chg.battery_temp;
}


#ifdef CONFIG_HAS_EARLYSUSPEND

struct batt_modify_client_req {

	u32 client_handle;

	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
};

struct batt_modify_client_rep {
	u32 result;
};


static int msm_batt_modify_client_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_modify_client_req *batt_modify_client_req =
		(struct batt_modify_client_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;


	*req = cpu_to_be32(batt_modify_client_req->client_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->cb_data);
	size += sizeof(u32);

	return size;
}

static int msm_batt_modify_client_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct  batt_modify_client_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_modify_client_rep *)data;
	buf_ptr = (struct batt_modify_client_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);

	return 0;
}

static int msm_batt_modify_client(u32 client_handle, u32 desired_batt_voltage,
	     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	int rc;

	struct batt_modify_client_req  req;
	struct batt_modify_client_rep rep;

	req.client_handle = client_handle;
	req.desired_batt_voltage = desired_batt_voltage;
	req.voltage_direction = voltage_direction;
	req.batt_cb_id = batt_cb_id;
	req.cb_data = cb_data;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_MODIFY_CLIENT_PROC,
			msm_batt_modify_client_arg_func, &req,
			msm_batt_modify_client_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s(): ERROR. failed to modify  Vbatt client\n",
				__func__);
		return rc;
	}

	if (rep.result != BATTERY_MODIFICATION_SUCCESSFUL) {

		pr_err("%s(): ERROR. modify client failed._result  = %u\n",
				__func__, rep.result);
		return -EIO;
	}

	printk(KERN_INFO "%s() : modify client successful.\n", __func__);
	return 0;
}

void msm_batt_early_suspend(struct early_suspend *h)
{
	int rc;

	printk(KERN_INFO "%s(): going to early suspend\n", __func__);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {

		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
				BATTERY_CB_ID_LOW_VOL, BATTERY_LOW);

		if (rc < 0) {
			pr_err("%s(): ERROR. failed to complete early "
					"suspend\n", __func__);
			return;
		}
	} else {
		pr_err("%s(): ERROR. Batt handle is invalid \n", __func__);
		return;
	}

	printk(KERN_INFO "%s(): Handled early suspend event.\n", __func__);
}

void msm_batt_late_resume(struct early_suspend *h)
{
	int rc;
	printk(KERN_INFO "%s(): going to resume\n", __func__);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {

		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
		if (rc < 0) {
			pr_err("%s(): ERROR. failed to complete late resume\n",
					__func__);
			return;
		}
	} else {
		pr_err("%s(): ERROR. Batt handle is invalid \n", __func__);
		return;
	}

	printk(KERN_INFO "%s(): Handled Late resume event event.\n", __func__);
}
#endif

struct msm_batt_vbatt_filter_req {
	u32 batt_handle;
	u32 enable_filter;
	u32 vbatt_filter;
};

struct msm_batt_vbatt_filter_rep {
	u32 result;
};

static int msm_batt_filter_arg_func(struct msm_rpc_client *batt_client,

		void *buf, void *data)
{
	struct msm_batt_vbatt_filter_req *vbatt_filter_req =
		(struct msm_batt_vbatt_filter_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(vbatt_filter_req->batt_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->enable_filter);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->vbatt_filter);
	size += sizeof(u32);
	return size;
}

static int msm_batt_filter_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{

	struct msm_batt_vbatt_filter_rep *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_vbatt_filter_rep *)data;
	buf_ptr = (struct msm_batt_vbatt_filter_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);
	return 0;
}

static int msm_batt_enable_filter(u32 vbatt_filter)
{
	int rc;
	struct  msm_batt_vbatt_filter_req  vbatt_filter_req;
	struct  msm_batt_vbatt_filter_rep  vbatt_filter_rep;

	vbatt_filter_req.batt_handle = msm_batt_info.batt_handle;
	vbatt_filter_req.enable_filter = 1;
	vbatt_filter_req.vbatt_filter = vbatt_filter;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_ENABLE_DISABLE_FILTER_PROC,
			msm_batt_filter_arg_func, &vbatt_filter_req,
			msm_batt_filter_ret_func, &vbatt_filter_rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s(): ERROR. vbatt Filter enable failed.\n", __func__);
		return rc;
	}

	if (vbatt_filter_rep.result != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s(): ERROR. vbatt Filter enable result not "
				"successul\n", __func__);
		return -EIO;
	}

	printk(KERN_INFO "%s: vbatt Filter enabled successfully.\n", __func__);
	return rc;
}

struct batt_client_registration_req {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 more_data;
	u32 batt_error;
};

struct batt_client_registration_rep {
	u32 batt_handle;
};

static int msm_batt_register_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_req *batt_reg_req =
		(struct batt_client_registration_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;


	*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->cb_data);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->more_data);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->batt_error);
	size += sizeof(u32);

	return size;
}

static int msm_batt_register_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_client_registration_rep *)data;
	buf_ptr = (struct batt_client_registration_rep *)buf;

	data_ptr->batt_handle = be32_to_cpu(buf_ptr->batt_handle);

	return 0;
}

static int msm_batt_register(u32 desired_batt_voltage,
			     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	struct batt_client_registration_req batt_reg_req;
	struct batt_client_registration_rep batt_reg_rep;
	int rc;

	batt_reg_req.desired_batt_voltage = desired_batt_voltage;
	batt_reg_req.voltage_direction = voltage_direction;
	batt_reg_req.batt_cb_id = batt_cb_id;
	batt_reg_req.cb_data = cb_data;
	batt_reg_req.more_data = 1;
	batt_reg_req.batt_error = 0;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_REGISTER_PROC,
			msm_batt_register_arg_func, &batt_reg_req,
			msm_batt_register_ret_func, &batt_reg_rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s(): ERROR. vbatt Filter enable failed.\n", __func__);
		return rc;
	}

	msm_batt_info.batt_handle = batt_reg_rep.batt_handle;

	printk(KERN_INFO "batt_clnt_handle = %d\n", msm_batt_info.batt_handle);
	return 0;
}


struct batt_client_deregister_req {
	u32 batt_handle;
};

struct batt_client_deregister_rep {
	u32 batt_error;
};

static int msm_batt_deregister_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_req *deregister_req =
		(struct  batt_client_deregister_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(deregister_req->batt_handle);
	size += sizeof(u32);

	return size;
}

static int msm_batt_deregister_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_client_deregister_rep *)data;
	buf_ptr = (struct batt_client_deregister_rep *)buf;

	data_ptr->batt_error = be32_to_cpu(buf_ptr->batt_error);

	return 0;
}

static int msm_batt_deregister(u32 batt_handle)
{
	int rc;
	struct batt_client_deregister_req req;
	struct batt_client_deregister_rep rep;

	req.batt_handle = batt_handle;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_DEREGISTER_CLIENT_PROC,
			msm_batt_deregister_arg_func, &req,
			msm_batt_deregister_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s(): ERROR. vbatt client deregister failed. \n",
				__func__);
		return rc;
	}

	if (rep.batt_error != BATTERY_DEREGISTRATION_SUCCESSFUL) {

		printk(KERN_ERR "%s: vBatt deregistration Failed. "
		       "batt_clnt_handle = %d\n", __func__, batt_handle);
		return -EIO;
	}
	return 0;
}


static int msm_batt_cleanup(void)
{
	int rc = 0;

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {

		rc = msm_batt_deregister(msm_batt_info.batt_handle);
		if (rc < 0)
			pr_err("%s(): ERROR. msm_batt_deregister failed "
					"rc=%d\n", __func__, rc);
	}

	msm_batt_info.batt_handle = INVALID_BATT_HANDLE;

	if (msm_batt_info.batt_client)
		msm_rpc_unregister_client(msm_batt_info.batt_client);

	if (msm_batt_info.msm_psy_ac)
		power_supply_unregister(msm_batt_info.msm_psy_ac);

	if (msm_batt_info.msm_psy_usb)
		power_supply_unregister(msm_batt_info.msm_psy_usb);
	if (msm_batt_info.msm_psy_batt)
		power_supply_unregister(msm_batt_info.msm_psy_batt);


	if (msm_batt_info.chg_ep) {
		rc = msm_rpc_close(msm_batt_info.chg_ep);
		if (rc < 0) {
			pr_err("%s(): ERROR. msm_rpc_close failed for chg_ep."
					"rc=%d\n", __func__, rc);
		}
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (msm_batt_info.early_suspend.suspend == msm_batt_early_suspend)
		unregister_early_suspend(&msm_batt_info.early_suspend);
#endif
	return rc;
}

static u32 msm_batt_capacity(u32 current_voltage)
{
	u32 low_voltage = msm_batt_info.voltage_min_design;
	u32 high_voltage = msm_batt_info.voltage_max_design;

	return (current_voltage - low_voltage) * 100
	    / (high_voltage - low_voltage);
}


int msm_batt_get_charger_api_version(void)
{
	int rc ;
	struct rpc_reply_hdr *reply;

	struct rpc_req_chg_api_ver {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_chg_api_ver;

	struct rpc_rep_chg_api_ver {
		struct rpc_reply_hdr hdr;
		u32 num_of_chg_api_versions;
		u32 *chg_api_versions;
	};

	u32 num_of_versions;

	struct rpc_rep_chg_api_ver *rep_chg_api_ver;


	req_chg_api_ver.more_data = cpu_to_be32(1);

	msm_rpc_setup_req(&req_chg_api_ver.hdr, CHG_RPC_PROG, CHG_RPC_VERS,
			 ONCRPC_CHARGER_API_VERSIONS_PROC);

	rc = msm_rpc_write(msm_batt_info.chg_ep, &req_chg_api_ver,
			sizeof(req_chg_api_ver));
	if (rc < 0) {
		pr_err(
		       "%s(): msm_rpc_write failed.  proc = 0x%08x rc = %d\n",
		       __func__, ONCRPC_CHARGER_API_VERSIONS_PROC, rc);
		return rc;
	}

	for (;;) {
		rc = msm_rpc_read(msm_batt_info.chg_ep, (void *) &reply, -1,
				BATT_RPC_TIMEOUT);
		if (rc < 0)
			return rc;
		if (rc < RPC_REQ_REPLY_COMMON_HEADER_SIZE) {
			pr_err("%s(): ERROR. msm_rpc_read failed. read"
					" returned invalid packet which is"
					" neither rpc req nor rpc reply. "
					"legnth of packet = %d\n",
					__func__, rc);

			rc = -EIO;
			break;
		}
		/* we should not get RPC REQ or call packets -- ignore them */
		if (reply->type == RPC_TYPE_REQ) {

			pr_err("%s(): ERROR. returned RPC REQ packets while"
				" waiting for RPC REPLY replay read \n",
				__func__);
			kfree(reply);
			continue;
		}

		/* If an earlier call timed out, we could get the (no
		 * longer wanted) reply for it.	 Ignore replies that
		 * we don't expect
		 */
		if (reply->xid != req_chg_api_ver.hdr.xid) {

			pr_err("%s(): ERROR. returned RPC REPLY XID is not"
					" equall to RPC REQ XID \n", __func__);
			kfree(reply);
			continue;
		}
		if (reply->reply_stat != RPCMSG_REPLYSTAT_ACCEPTED) {
			rc = -EPERM;
			break;
		}
		if (reply->data.acc_hdr.accept_stat !=
				RPC_ACCEPTSTAT_SUCCESS) {
			rc = -EINVAL;
			break;
		}

		rep_chg_api_ver = (struct rpc_rep_chg_api_ver *)reply;

		num_of_versions =
			be32_to_cpu(rep_chg_api_ver->num_of_chg_api_versions);

		rep_chg_api_ver->chg_api_versions =  (u32 *)
			((u8 *) reply + sizeof(struct rpc_reply_hdr) +
			sizeof(rep_chg_api_ver->num_of_chg_api_versions));

		rc = be32_to_cpu(
			rep_chg_api_ver->chg_api_versions[num_of_versions - 1]);

		printk(KERN_INFO "%s(): num_of_chg_api_versions = %u"
			"  The chg api version = 0x%08x\n", __func__,
			num_of_versions, rc);
		break;
	}
	kfree(reply);
	return rc;
}

static int msm_batt_cb_func(struct msm_rpc_client *client,
			    void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;
	u32 procedure;
	u32 accept_status;

	req = (struct rpc_request_hdr *)buffer;
	procedure = be32_to_cpu(req->procedure);

	printk(KERN_INFO "%s: Got call back from modem Vbatt server."
			" Procedure number = %u\n", __func__, procedure);

	switch (procedure) {
	case BATTERY_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("%s(): ERROR. procedure not supported %d\n", __func__,
				procedure);
		break;
	}

	msm_rpc_start_accepted_reply(msm_batt_info.batt_client,
			be32_to_cpu(req->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(msm_batt_info.batt_client, 0);
	if (rc)
		pr_err("%s(): ERROR. sending reply failed: %d\n", __func__, rc);

	if (accept_status == RPC_ACCEPTSTAT_SUCCESS)
		msm_batt_update_psy_status();

	return rc;
}

static int __devinit msm_batt_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_psy_batt_pdata *pdata = pdev->dev.platform_data;

	if (pdev->id != -1) {
		dev_err(&pdev->dev,
			"%s: MSM chipsets Can only support one"
			" battery ", __func__);
		return -EINVAL;
	}

	if (pdata->avail_chg_sources & AC_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_ac);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_ac = &msm_psy_ac;
		msm_batt_info.avail_chg_sources |= AC_CHG;
	}

	if (pdata->avail_chg_sources & USB_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_usb);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_usb = &msm_psy_usb;
		msm_batt_info.avail_chg_sources |= USB_CHG;
	}

	if (!msm_batt_info.msm_psy_ac && !msm_batt_info.msm_psy_usb) {

		dev_err(&pdev->dev,
			"%s: No external Power supply(AC or USB)"
			"is avilable\n", __func__);
		msm_batt_cleanup();
		return -ENODEV;
	}

	msm_batt_info.voltage_max_design = pdata->voltage_max_design;
	msm_batt_info.voltage_min_design = pdata->voltage_min_design;
	msm_batt_info.batt_technology = pdata->batt_technology;
	msm_batt_info.calculate_capacity = pdata->calculate_capacity;

	if (!msm_batt_info.voltage_min_design)
		msm_batt_info.voltage_min_design = BATTERY_LOW;
	if (!msm_batt_info.voltage_max_design)
		msm_batt_info.voltage_max_design = BATTERY_HIGH;

	if (msm_batt_info.batt_technology == POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
		msm_batt_info.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION;

	if (!msm_batt_info.calculate_capacity)
		msm_batt_info.calculate_capacity = msm_batt_capacity;

	rc = power_supply_register(&pdev->dev, &msm_psy_batt);
	if (rc < 0) {
		dev_err(&pdev->dev, "%s: power_supply_register failed"
			" rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}
	msm_batt_info.msm_psy_batt = &msm_psy_batt;


	rc = msm_batt_register(BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_register failed rc = %d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	rc =  msm_batt_enable_filter(VBATT_FILTER);

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_enable_filter failed rc = %d\n",
			__func__, rc);
		msm_batt_cleanup();
		return rc;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	msm_batt_info.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	msm_batt_info.early_suspend.suspend = msm_batt_early_suspend;
	msm_batt_info.early_suspend.resume = msm_batt_late_resume;
	register_early_suspend(&msm_batt_info.early_suspend);
#endif
	msm_batt_update_psy_status();

	return 0;
}

static int __devexit msm_batt_remove(struct platform_device *pdev)
{
	int rc;
	rc = msm_batt_cleanup();

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_cleanup  failed rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static struct platform_driver msm_batt_driver = {
	.probe = msm_batt_probe,
	.remove = __devexit_p(msm_batt_remove),
	.driver = {
		   .name = "msm-battery",
		   .owner = THIS_MODULE,
		   },
};

static int __devinit msm_batt_init_rpc(void)
{
	int rc;

	msm_batt_info.chg_ep =
	    msm_rpc_connect_compatible(CHG_RPC_PROG, CHG_RPC_VERS, 0);

	if (msm_batt_info.chg_ep == NULL) {
		return -ENODEV;
	} else if (IS_ERR(msm_batt_info.chg_ep)) {
		int rc = PTR_ERR(msm_batt_info.chg_ep);
		pr_err("%s: ERROR. rpc connect failed for CHG_RPC_PROG."
				"rc = %d\n", __func__, rc);
		msm_batt_info.chg_ep = NULL;
		return rc;
	}

	msm_batt_info.chg_api_version =  msm_batt_get_charger_api_version();

	if (msm_batt_info.chg_api_version < CHARGER_API_VERSION) {

		pr_err("%s(): ERROR. unsupported charger API version on modem."
		       " chg_api_version = %x\n ", __func__,
		       msm_batt_info.chg_api_version);
		return -ENODEV;
	}

	msm_batt_info.batt_client =
		msm_rpc_register_client("battery", BATTERY_RPC_PROG,
				BATTERY_RPC_VERS, 1, msm_batt_cb_func);

	if (msm_batt_info.batt_client == NULL) {

		pr_err("%s: ERROR. rpc_register_client failed for Battery"
			"server. batt_client = NULL\n ", __func__);
		return -ENODEV;

	} else if (IS_ERR(msm_batt_info.batt_client)) {
		int rc = PTR_ERR(msm_batt_info.batt_client);
		pr_err("%s: ERROR. rpc_register_client failed for Battery"
			"server. rc = %d\n ", __func__, rc);
		msm_batt_info.batt_client = NULL;
		return rc;
	}

	rc = platform_driver_register(&msm_batt_driver);

	if (rc < 0) {
		pr_err("%s(): ERROR. platform_driver_register failed for "
			"batt driver. rc = %d\n", __func__, rc);
		return rc;
	}

	return 0;
}

static int __init msm_batt_init(void)
{
	int rc;

	rc = msm_batt_init_rpc();

	if (rc < 0) {
		pr_err("%s(): ERROR. msm_batt_init_rpc Failed  rc=%d\n",
		       __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	return 0;
}

static void __exit msm_batt_exit(void)
{
	platform_driver_unregister(&msm_batt_driver);
}

module_init(msm_batt_init);
module_exit(msm_batt_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Kiran Kandi, Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:msm_battery");
