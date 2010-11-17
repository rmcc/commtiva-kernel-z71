/*
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>

#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/switch.h>

#include <mach/msm_hsusb.h>

static char *supply_list[] = {
	"battery",
};

static int usb_status = -1;


static int power_get_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
		val->intval = (usb_status == USB_CHG_TYPE__WALLCHARGER);
	} else {
		val->intval = (usb_status == USB_CHG_TYPE__SDP);
	}
	return 0;
}

static enum power_supply_property power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply ac_supply = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = supply_list,
	.num_supplicants = ARRAY_SIZE(supply_list),
	.properties = power_properties,
	.num_properties = ARRAY_SIZE(power_properties),
	.get_property = power_get_property,
};

static struct power_supply usb_supply = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = supply_list,
	.num_supplicants = ARRAY_SIZE(supply_list),
	.properties = power_properties,
	.num_properties = ARRAY_SIZE(power_properties),
	.get_property = power_get_property,
};

static int zeus_power_probe(struct platform_device *pdev)
{

	power_supply_register(&pdev->dev, &ac_supply);
	power_supply_register(&pdev->dev, &usb_supply);

	return 0;
}

static struct platform_driver zeus_power_driver = {
	.probe	= zeus_power_probe,
	.driver	= {
		.name	= "zeus-power",
		.owner	= THIS_MODULE,
	},
};


#ifdef CONFIG_BATTERY_FIH_ZEUS
extern void zeus_update_usb_status(enum chg_type chgtype);
#endif

void notify_usb_connected(int status)
{
	printk("### notify_usb_connected(%d) ###\n", status);
	usb_status = status;
#ifdef CONFIG_BATTERY_FIH_ZEUS
	zeus_update_usb_status(status);
#endif
	power_supply_changed(&ac_supply);
	power_supply_changed(&usb_supply);
}

int is_ac_power_supplied(void)
{
	return (usb_status == USB_CHG_TYPE__WALLCHARGER);
}


static int __init zeus_power_init(void)
{
	platform_driver_register(&zeus_power_driver);
	power_supply_changed(&ac_supply);
	power_supply_changed(&usb_supply);
	return 0;
}

module_init(zeus_power_init);
MODULE_DESCRIPTION("ADQ Power Driver");
MODULE_LICENSE("GPL");

