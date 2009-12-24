/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#include <linux/usb/android.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#ifdef CONFIG_USB_ANDROID_CDC_ECM
#include "u_ether.h"
#endif

#include "f_mass_storage.h"
#include "f_adb.h"
#include "u_serial.h"
#ifdef CONFIG_USB_ANDROID_DIAG
#include "f_diag.h"
#endif

#include "gadget_chips.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";
#ifdef CONFIG_USB_ANDROID_CDC_ECM
static u8 hostaddr[ETH_ALEN];
#endif

/* Default vendor and product IDs, overridden by platform data */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001
#define ADB_PRODUCT_ID	0x0002

struct android_dev {
	struct usb_gadget *gadget;
	struct usb_composite_dev *cdev;

	int product_id;
	int adb_product_id;
	int version;

	int adb_enabled;
	int nluns;
	struct android_usb_platform_data *pdata;
	unsigned long functions;
};

static int acm_func_cnt;
static int gser_func_cnt;
static atomic_t adb_enable_excl;
static struct android_dev *_android_dev;
static void android_switch_composition(unsigned short pid);

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "Android",
	[STRING_PRODUCT_IDX].s = "Android",
	[STRING_SERIAL_IDX].s = "0123456789ABCDEF",
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static void enable_adb(struct android_dev *dev, int enable);

#define android_func_attr(function, index)				\
static ssize_t  show_##function(struct device *dev,			\
		struct device_attribute *attr, char *buf)		\
{									\
									\
	unsigned long n = _android_dev->functions;			\
	int val = 0;							\
									\
	while (n) {							\
		if ((n & 0x0F) == index)				\
			val = 1;					\
		n = n >> 4;						\
	}								\
	return sprintf(buf, "%d", val);					\
									\
}									\
									\
static DEVICE_ATTR(function, S_IRUGO, show_##function, NULL);

android_func_attr(adb, ANDROID_ADB);
android_func_attr(mass_storage, ANDROID_MSC);
android_func_attr(acm_modem, ANDROID_ACM_MODEM);
android_func_attr(acm_nmea, ANDROID_ACM_NMEA);
android_func_attr(diag, ANDROID_DIAG);
android_func_attr(modem, ANDROID_GENERIC_MODEM);
android_func_attr(nmea, ANDROID_GENERIC_NMEA);
android_func_attr(cdc_ecm, ANDROID_CDC_ECM);

static struct attribute *android_func_attrs[] = {
	&dev_attr_adb.attr,
	&dev_attr_mass_storage.attr,
	&dev_attr_acm_modem.attr,
	&dev_attr_acm_nmea.attr,
	&dev_attr_diag.attr,
	&dev_attr_modem.attr,
	&dev_attr_nmea.attr,
	&dev_attr_cdc_ecm.attr,
	NULL,
};

static struct attribute_group android_func_attr_grp = {
	.name  = "functions",
	.attrs = android_func_attrs,
};

static int  android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = -EINVAL;
	unsigned long n;
	acm_func_cnt = 0;
	gser_func_cnt = 0;
	printk(KERN_DEBUG "android_bind_config\n");
	n = dev->functions;
	while (n) {
		switch (n & 0x0F) {
		case ANDROID_ADB:
			ret = adb_function_add(dev->cdev, c);
			if (ret)
				return ret;
			dev->adb_enabled = 1;
			break;
		case ANDROID_MSC:
			ret = mass_storage_function_add(dev->cdev, c,
								dev->nluns);
			if (ret)
				return ret;
			break;
		case ANDROID_ACM_MODEM:
			ret = acm_bind_config(c, 0);
			if (ret)
				return ret;
			acm_func_cnt++;
			break;
		case ANDROID_ACM_NMEA:
			ret = acm_bind_config(c, 1);
			if (ret)
				return ret;
			acm_func_cnt++;
			break;
#ifdef CONFIG_USB_ANDROID_DIAG
		case ANDROID_DIAG:
			ret = diag_function_add(c, dev->pdata->serial_number);
			if (ret)
				return ret;
			break;
#endif
#ifdef CONFIG_USB_F_SERIAL
		case ANDROID_GENERIC_MODEM:
			ret = gser_bind_config(c, 0);
			if (ret)
				return ret;
			gser_func_cnt++;
			break;
		case ANDROID_GENERIC_NMEA:
			ret = gser_bind_config(c, 1);
			if (ret)
				return ret;
			gser_func_cnt++;
			break;
#endif
#ifdef CONFIG_USB_ANDROID_CDC_ECM
		case ANDROID_CDC_ECM:
			ret = ecm_bind_config(c, hostaddr);
			if (ret)
				return ret;
			break;
#endif
		default:
			ret = -EINVAL;
			return ret;
		}
		n = n >> 4;
	}
	return ret;

}

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.bind		= android_bind_config,
	.bConfigurationValue = 1,
	.bmAttributes	= (USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER |
				USB_CONFIG_ATT_WAKEUP),
	.bMaxPower	= 0xFA, /* 500ma */
};

static int android_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	if (dev->adb_enabled)
		dev->adb_enabled = 0;
	if (acm_func_cnt || gser_func_cnt)
		gserial_cleanup();
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	gether_cleanup();
#endif

	return 0;
}
static int  android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum;
	int			id;
	int			ret;
	int                     num_ports;

	printk(KERN_INFO "android_bind\n");

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	if (gadget->ops->wakeup)
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;

	dev->cdev = cdev;
	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		printk(KERN_ERR "usb_add_config failed\n");
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	device_desc.idProduct = dev->product_id;
	num_ports = acm_func_cnt + gser_func_cnt;
	if (acm_func_cnt || gser_func_cnt) {
		ret = gserial_setup(cdev->gadget, num_ports);
		if (ret < 0)
			return ret;
	}
	if (acm_func_cnt) {
		device_desc.bDeviceClass         = USB_CLASS_MISC;
		device_desc.bDeviceSubClass      = USB_SUB_CLASS_COMMON;
		device_desc.bDeviceProtocol      = USB_PROTOCOL_IAD;
	} else {
		device_desc.bDeviceClass         = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceSubClass      = 0;
		device_desc.bDeviceProtocol      = 0;
	}
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	/* set up network link layer */
	ret = gether_setup(cdev->gadget, hostaddr);
	if (ret < 0)
		return ret;
#endif

	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.unbind		= android_unbind,
};
static ssize_t android_show_compswitch(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct android_dev *device = _android_dev;
	int i;

	i = scnprintf(buf, PAGE_SIZE,
			"composition product id = %x\n",
				device->product_id);
	return i;
}

static ssize_t android_store_compswitch(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	unsigned long pid;
	if (!strict_strtoul(buf, 16, &pid)) {
		pr_info("%s: Requested New Product id = %lx\n", __func__, pid);
		android_switch_composition((unsigned short)pid);
	} else
		pr_info("%s: strict_strtoul conversion failed\n", __func__);

	return size;
}

static unsigned int android_validate_product_id(unsigned short pid)
{
	struct android_dev *dev = _android_dev;
	int i;

	if (!dev->pdata)
		return 0;

	for (i = 0; i < dev->pdata->num_compositions; i++) {
		if (dev->pdata->compositions[i].product_id == pid) {
			dev->product_id = pid;
			dev->functions = dev->pdata->compositions[i].functions;
			return dev->functions;
		}
	}
	return 0;
}
static unsigned short android_validate_function_map(unsigned int n)
{
	struct android_dev *dev = _android_dev;
	int i;

	if (!dev->pdata)
		return 0;

	for (i = 0; i < dev->pdata->num_compositions; i++) {
		if (dev->pdata->compositions[i].functions == n) {
			dev->functions = dev->pdata->compositions[i].functions;
			return dev->pdata->compositions[i].product_id;
		}
	}
	return 0;
}
static void android_switch_composition(unsigned short pid)
{
	if (!android_validate_product_id(pid))
		return;
	usb_composite_unregister(&android_usb_driver);
	usb_composite_register(&android_usb_driver);
}
static DEVICE_ATTR(composition, 0664,
		android_show_compswitch, android_store_compswitch);

static struct attribute *android_attrs[] = {
	&dev_attr_composition.attr,
	NULL,
};

static struct attribute_group android_attr_grp = {
	.attrs = android_attrs,
};

static void enable_adb(struct android_dev *dev, int enable)
{
	unsigned int n;
	unsigned short pid;

	n = dev->functions;

	if (enable) {

		if (dev->adb_enabled) {
			printk(KERN_ERR "adb already enabled\n");
			return;
		}
		/* inserting adb function at second nibble*/
		n = ((((((n >> 4) << 4) | ANDROID_ADB)) << 4) | (n & 0x0F));
		pid = android_validate_function_map(n);
		if (pid)
			android_switch_composition(pid);
		else {
			printk(KERN_ERR "can't add adb to"
				"the existing composition\n");
			return;
		}
	} else {
		if (!dev->adb_enabled) {
			printk(KERN_ERR "adb already disabled\n");
			return;
		}
		/* remove adb function from second nibble*/
		n = (((n >> 8) << 4) | (n & 0x0F));
		pid = android_validate_function_map(n);
		if (pid)
			android_switch_composition(pid);
		else {
			printk(KERN_ERR "can't remove adb from"
				"the existing composition\n");
			return;
		}
	}
}

static int adb_enable_open(struct inode *ip, struct file *fp)
{
	if (atomic_inc_return(&adb_enable_excl) != 1) {
		atomic_dec(&adb_enable_excl);
		return -EBUSY;
	}

	printk(KERN_INFO "enabling adb\n");
	enable_adb(_android_dev, 1);

	return 0;
}

static int adb_enable_release(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "disabling adb\n");
	enable_adb(_android_dev, 0);
	atomic_dec(&adb_enable_excl);
	return 0;
}

static struct file_operations adb_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    adb_enable_open,
	.release = adb_enable_release,
};

static struct miscdevice adb_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_adb_enable",
	.fops = &adb_enable_fops,
};

static int __init android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "android_probe pdata: %p\n", pdata);

	if (pdata) {
		if (pdata->vendor_id)
			device_desc.idVendor =
				__constant_cpu_to_le16(pdata->vendor_id);
		if (pdata->product_id) {
			dev->product_id = pdata->product_id;
			dev->functions  = pdata->functions;
			device_desc.idProduct =
				__constant_cpu_to_le16(pdata->product_id);
		}
		if (pdata->adb_product_id)
			dev->adb_product_id = pdata->adb_product_id;
		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
					pdata->manufacturer_name;
		if (pdata->serial_number)
			strings_dev[STRING_SERIAL_IDX].s = pdata->serial_number;
		dev->nluns = pdata->nluns;
		dev->pdata     = pdata;
	}
	if (sysfs_create_group(&pdev->dev.kobj, &android_attr_grp))
		pr_err("%s: Failed to create the sysfs entry \n", __func__);
	if (sysfs_create_group(&pdev->dev.kobj, &android_func_attr_grp))
		pr_err("%s: Failed to create the functions sysfs entry \n",
				__func__);

	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", },
	.probe = android_probe,
};

static int __init init(void)
{
	struct android_dev *dev;
	int ret;

	printk(KERN_INFO "android init\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* set default values, which should be overridden by platform data */
	dev->product_id = PRODUCT_ID;
	dev->adb_product_id = ADB_PRODUCT_ID;
	_android_dev = dev;

	ret = adb_function_init();
	if (ret)
		return ret;
	ret = platform_driver_register(&android_platform_driver);
	if (ret)
		return ret;
	ret = misc_register(&adb_enable_device);
	if (ret) {
		platform_driver_unregister(&android_platform_driver);
		return ret;
	}
	ret = usb_composite_register(&android_usb_driver);
	if (ret) {
		misc_deregister(&adb_enable_device);
		platform_driver_unregister(&android_platform_driver);
	}
	return ret;
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	misc_deregister(&adb_enable_device);
	platform_driver_unregister(&android_platform_driver);
	adb_function_exit();
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
