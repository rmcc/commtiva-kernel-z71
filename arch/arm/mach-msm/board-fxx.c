/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/usb/mass_storage_function.h>
#include <linux/power_supply.h>


#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_serial_hs.h>
#include <mach/memory.h>
#include <mach/msm_battery.h>


#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <mach/camera.h>

#include <linux/elan_i2c.h>  //Added for capacitive touch panel, by Stanley++ 2009/05/20
#include <linux/switch.h> 
#include "devices.h"
#include "socinfo.h"
#include "clock.h"
#include "msm-keypad-devices.h"
#include <mach/tca6507.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#endif
#include "pm.h"
//WilsonWHLee smem command ++
#include "proc_comm.h"
//WilsonWHLee smem command --
/* FIH, Chandler Kang, 2009/05/18 { */
#ifdef CONFIG_SPI_GPIO  

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/spi_gpio.h>

#endif  //CONFIG_SPI_GPIO
/* FIH, Chandler Kang, 2009/05/18 } */
/* FIH, PeterKCTseng, @20090521 { */
#include <mach/7x27_kybd.h>
/* } FIH, PeterKCTseng, @20090521 */
#ifdef CONFIG_ARCH_MSM7X27
#include <linux/msm_kgsl.h>
#endif

/* FIH, Debbie Sun, 2009/06/18 { */
/* get share memory command address dynamically */
#include "smd_private.h"
#include <mach/msm_smd.h>
/* FIH, Debbie Sun, 2009/06/18 }*/


#ifdef CONFIG_ARCH_MSM7X25
#define MSM_PMEM_MDP_SIZE	0xb21000
#define MSM_PMEM_ADSP_SIZE	0x97b000
#define MSM_FB_SIZE		0x200000
#define PMEM_KERNEL_EBI1_SIZE	0x80000
#endif

#ifdef CONFIG_ARCH_MSM7X27
    //For HVGA surface flinger gralloc, from 23 (20 + 3) MB -> (8MB + 1200KB)
    //and VGA video encode fit into pmem_adsp (gain 5700 KB)
    //Need to modify vendor\qcom-proprietary\mm-video\7k\venc-omx\driver\src\venc_drv.c 
    //venc_drv_malloc need to use pmem_adsp instead of pmem
    #define MSM_PMEM_MDP_SIZE	0x92C000
    //For HVGA camera preview & SOC sensor (gain ~2 MB)
    //HVGA (420 x 320) preview + 5M (2592 x 1944) raw data + 512 x 384 thumbnail data
    #define MSM_PMEM_ADSP_SIZE	0x8DB000 // 0x156000 + 0x73D000 + 0x48000
#ifndef CONFIG_MSM_KGSL_MMU
    //Need to enable GPU MMU feature
    //Need to use 5320 gles library
    //CONFIG_MSM_KGSL_MMU=y in kernel/arch/arm/configs/msm7627_defconfig
    //GPU RAM from 22 MB -> 16MB
    #define MSM_PMEM_GPU1_SIZE	0x1000000 
#endif //CONFIG_MSM_KGSL_MMU  

/* FIH, ChandlerKang, 2009/12/8{ */
#define MSM_FB_SIZE		0xA0000
/* FIH, ChandlerKang, 2009/12/8 } */
#define MSM_GPU_PHYS_SIZE	SZ_2M
#define PMEM_KERNEL_EBI1_SIZE	0x200000
#endif

#ifdef CONFIG_AR6K
#define WIFI_CONTROL_MASK   0x10000000
#define WIFI_SUSPEND_CONTROL_MASK   0x01000000
#endif

char *board_serial;

#ifdef CONFIG_AR6K
#define MODULE_TURN_ON      0x01
#define MODULE_TURN_OFF     0x02
static int wifi_status = 0;
static int bt_status = 0;
static DEFINE_SPINLOCK(wif_bt_lock);
#endif


#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = 0x02,
	.buf_size       = 16384,
	.vendor         = "GOOGLE",
	.product        = "Mass storage",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif
#ifdef CONFIG_USB_ANDROID
/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		/* MSC */
		.product_id         = 0xC000,
		.functions	    = 0x02,
		.adb_product_id     = 0xC002,
		.adb_functions	    = 0x12
	},
#ifdef CONFIG_USB_F_SERIAL
	{
		/* MODEM */
		.product_id         = 0xF00B,
		.functions	    = 0x06,
		.adb_product_id     = 0x901E,
		.adb_functions	    = 0x16,
	},
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		/* DIAG */
		.product_id         = 0x900E,
		.functions	    = 0x04,
		.adb_product_id     = 0x901D,
		.adb_functions	    = 0x14,
	},
#endif
#if defined(CONFIG_USB_ANDROID_DIAG) && defined(CONFIG_USB_F_SERIAL)
	{
		/* DIAG + MODEM */
		.product_id         = 0x9004,
		.functions	    = 0x64,
		.adb_product_id     = 0x901F,
		.adb_functions	    = 0x0614,
	},
	{
		/* DIAG + MODEM + NMEA*/
		.product_id         = 0x9016,
		.functions	    = 0x764,
		.adb_product_id     = 0x9020,
		.adb_functions	    = 0x7614,
	},
	{
		/* DIAG + MODEM + NMEA + MSC */
		.product_id         = 0x9017,
		.functions	    = 0x2764,
		.adb_product_id     = 0x9018,
		.adb_functions	    = 0x27614,
	},
#endif
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	{
		/* MSC + CDC-ECM */
		.product_id         = 0x9014,
		.functions	    = 0x82,
		.adb_product_id     = 0x9023,
		.adb_functions	    = 0x812,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	{
		/* DIAG + RMNET */
		.product_id         = 0x9021,
		.functions	    = 0x94,
		.adb_product_id     = 0x9022,
		.adb_functions	    = 0x914,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		/* RNDIS */
		.product_id         = 0xC004,
		.functions	    = 0xA,
		.adb_product_id     = 0xC001,
		.adb_functions	    = 0x1A,
	},
#endif
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id  = 0x489,
	.version	= 0x0100,
	.compositions   = usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.product_name	= "Qualcomm HSUSB Device",
	.manufacturer_name = "Qualcomm Incorporated",
	.nluns = 2,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
	{"rmnet", 6},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
		.product_id         = 0x9018,
		.functions	    = 0x1F, /* 011111 */
	},
#ifdef CONFIG_USB_FUNCTION_RMNET
	{
		.product_id         = 0x9021,
		/* DIAG + RMNET */
		.functions	    = 0x41,
	},
	{
		.product_id         = 0x9022,
		/* DIAG + ADB + RMNET */
		.functions	    = 0x43,
	},
#endif

};
#endif

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_FUNCTION
	.version	= 0x0100,
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_65NM),
	.vendor_id          = 0x5c6,
	.product_name       = "Qualcomm HSUSB Device",
	.serial_number      = "1234567890ABCDEF",
	.manufacturer_name  = "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.config_gpio    = NULL,
#endif
};

static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static int msm_hsusb_rpc_phy_reset(void __iomem *addr)
{
	return msm_hsusb_phy_reset();
}

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,
	.phy_reset	= msm_hsusb_rpc_phy_reset,
	.pmic_notif_init         = msm_pm_app_rpc_init,
	.pmic_notif_deinit       = msm_pm_app_rpc_deinit,
	.pmic_register_vbus_sn   = msm_pm_app_register_vbus_sn,
	.pmic_unregister_vbus_sn = msm_pm_app_unregister_vbus_sn,
	.pmic_enable_ldo         = msm_pm_app_enable_usb_ldo,
};

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
/* FIH, Debbie , 2009/07/01 { */
/* just work around */
	SND(HANDSET, 0),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
//+++ FIH, KarenLiao, @20091002: FA3.B-352: Add for TTY devices.
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
//--- FIH, KarenLiao, @20091002: FA3.B-352: Add for TTY devices.
	SND(BT, 12),
//+++ FIH, KarenLiao @20100305: 5310 Porting	
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
//--- FIH, KarenLiao @20100305: 5310 Porting
	SND(HEADSET_AND_SPEAKER, 26), //FIH, KarenLiao, @20090623: Implement ringtone output from speaker and headset.
	SND(HEADSET_WITH_INNER_MIC, 27),  //FIH, KarenLiao, @20091023: [F0X.FC-663]: Allow user to use the normal headset.
	SND(CURRENT, 29), //FIH, KarenLiao, @20091023: [F0X.FC-663]: Allow user to use the normal headset.
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP)| \
	(1<<MSM_ADSP_CODEC_MP3))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP)| \
	(1<<MSM_ADSP_CODEC_MP3))

#ifdef CONFIG_ARCH_MSM7X25
#define DEC3_FORMAT 0
#define DEC4_FORMAT 0
#else
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)
#endif

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 5),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 5),  /* AudPlay2BitStreamCtrlQueue */
#ifdef CONFIG_ARCH_MSM7X25
	DEC_INFO("AUDPLAY3TASK", 16, 3, 0),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 0),  /* AudPlay4BitStreamCtrlQueue */
#else
	DEC_INFO("AUDPLAY3TASK", 16, 3, 4),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
#endif
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

#ifdef CONFIG_ARCH_MSM7X27
#ifndef CONFIG_MSM_KGSL_MMU
static struct android_pmem_platform_data android_pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};
#endif //CONFIG_MSM_KGSL_MMU
#endif

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

#ifdef CONFIG_ARCH_MSM7X27
#ifndef CONFIG_MSM_KGSL_MMU
static struct platform_device android_pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_gpu1_pdata },
};
#endif //CONFIG_MSM_KGSL_MMU
#endif

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = "7k_handset",
	},
};

/* FIH; Tiger; 2009/6/22 { */
/* implement suspend/resume for jogball */
static struct platform_device mtb_platform_device = {
	.name = "mtb",
};
/* } FIH; Tiger; 2009/6/22 */

// +++ FIH, KarenLiao, 20090518: Add for headset detection.
static struct gpio_switch_platform_data headset_sensor_device_data = {
	.name = "h2w",
	.gpio = 40,
	.name_on = "",
	.name_off = "",
	//+++ FIH, KarenLiao, @20090625: Modify kernel layer for headset low active detection mechanism.	
	//.state_on = "1",
	//.state_off = "0",
	.state_on = "0",
	.state_off = "1",
	//--- FIH, KarenLiao, @20090625: Modify kernel layer for headset low active detection mechanism.
};
	
static struct platform_device headset_sensor_device = {
	.name = "switch_gpio",
	.id	= -1,
	.dev = { .platform_data = &headset_sensor_device_data },
};

// --- FIH, KarenLiao, 20090518: Add for headset detection.
//FIH, WillChen, 2009/8/24++
/* [FXX_CR], Add ALS & PS driver into platform driver*/
static struct platform_device ALSPS_sensor_device = {
	.name = "cm3602_alsps",

};

//FIH, WillChen, 2009/8/24--
#define LCDC_CONFIG_PROC          21
#define LCDC_UN_CONFIG_PROC       22
#define LCDC_API_PROG             0x30000066
#define LCDC_API_VERS             0x00010001

#define GPIO_OUT_132    132
#define GPIO_OUT_131    131
#define GPIO_OUT_103    103
#define GPIO_OUT_102    102
#define GPIO_OUT_88     88
/* { FIH, Chandler, 2009/7/1*/

static int msm_fb_lcdc_config(int on){
    acpuclk_set_lcdcoff_wait_for_irq(on);
    return 0;
}


static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_gpio_config = msm_fb_lcdc_config
};


static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;

	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
		if (!strcmp(name, "lcdc_gordon_vga"))
			ret = 0;
		else
			ret = -ENODEV;
	}

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource[] = {
        {
            .flags  = IORESOURCE_MEM,
        }
};

static struct platform_device ram_console_device = {
        .name = "ram_console",
        .id = -1,
        .num_resources  = ARRAY_SIZE(ram_console_resource),
        .resource       = ram_console_resource,
};
#endif

/* FIH, JamesKCTung, 2009/05/11 { */
static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd);
static uint32_t msm_ar6k_sdcc_setup_power(struct device *dv, unsigned int vdd);
int static ar6k_wifi_suspend(int dev_id);
int static ar6k_wifi_resume(int dev_id);
static void (*ar6k_wifi_status_cb)(int card_present, void *dev_id);
static void *ar6k_wifi_status_cb_devid;
static unsigned int  wifi_power_on = 0;
//FIH, WilsonWHLee, 2009/11/26++
/* [FXX_CR], read product id as serial number*/
static int msm_read_serial_number_from_nvitem(void);
//FIH, WilsonWHLee, 2009/11/26--

static int ar6k_wifi_status_register(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
	if (ar6k_wifi_status_cb)
		return -EAGAIN;
	ar6k_wifi_status_cb = callback;
	ar6k_wifi_status_cb_devid = dev_id;
	return 0;
}

extern int android_set_sn(const char *kmessage, struct kernel_param *kp);

//FIH, WilsonWHLee, 2009/11/26++
/* [FXX_CR], read product id as serial number*/
static int msm_read_serial_number_from_nvitem()
{
	uint32_t smem_proc_comm_oem_cmd1 = PCOM_CUSTOMER_CMD1;
	uint32_t smem_proc_comm_oem_data1 = SMEM_PROC_COMM_OEM_PRODUCT_ID_READ;
  	uint32_t smem_proc_comm_oem_data2 = 0;
  	uint32_t product_id[40];	
	board_serial = kzalloc(64, GFP_KERNEL);
  	
    if(msm_proc_comm_oem(smem_proc_comm_oem_cmd1, &smem_proc_comm_oem_data1, product_id, &smem_proc_comm_oem_data2) == 0)
    {      
      printk(KERN_INFO"%s: [wilson product_id=%s]\r\n",__func__,(char *)product_id);
      //memcpy(msm_hsusb_pdata.serial_number, product_id, 16);
	  //memcpy(android_usb_pdata.serial_number, product_id, 16);
	  memcpy(board_serial, product_id, 16);
	  android_set_sn(board_serial,NULL);
    } 	
	return 1;
  	
}
//FIH, WilsonWHLee, 2009/11/26--
static unsigned int ar6k_wifi_status(struct device *dev)
{
	return wifi_power_on;
}
static struct mmc_platform_data ar6k_wifi_data = {
	.ocr_mask	    = MMC_VDD_28_29,
	.translate_vdd	= msm_ar6k_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.status			= ar6k_wifi_status,
	.register_status_notify	= ar6k_wifi_status_register,
	.sdio_suspend = ar6k_wifi_suspend,
	.sdio_resume = ar6k_wifi_resume,
	.dummy52_required = 1,	
};
/* } FIH, JamesKCTung, 2009/05/11 */
#ifdef CONFIG_BT
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

enum {
	BT_WAKE,
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
	BT_PCM_DOUT,
	BT_PCM_DIN,
	BT_PCM_SYNC,
	BT_PCM_CLK,
	BT_HOST_WAKE,
};

static unsigned bt_config_init[] = {
	GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* RFR */
	GPIO_CFG(44, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	        /* CTS */
	GPIO_CFG(45, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	        /* Rx */
	GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* Tx */
	//GPIO_CFG(43, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),     /* RFR */ 
	//GPIO_CFG(44, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),       /* CTS */ 
	//GPIO_CFG(45, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),       /* Rx */ 
	//GPIO_CFG(46, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),     /* Tx */ 

	GPIO_CFG(76, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* 3.3V */
	GPIO_CFG(77, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* 1.5V */
	GPIO_CFG(34, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* 1.2V */
	GPIO_CFG(27, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* BT_RST */
	GPIO_CFG(37, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* HOST_WAKE_BT */
	GPIO_CFG(42, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),           /* BT_WAKE_HOST */
	GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	        /* PCM_DIN */
	GPIO_CFG(70, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_CLK */
	GPIO_CFG(77, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* 1.5V */
	GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA),   /* sd2 */
	GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),        /* sd2 */
	GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),        /* sd2 */
	GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),        /* sd2 */
	GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),        /* sd2 */
	GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),        /* sd2 */
	GPIO_CFG(49, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),       /* WLAN_INT_HOST */
	GPIO_CFG(96, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* WLAN_PWD */
	GPIO_CFG(35, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* WLAN_RESET */
	GPIO_CFG(35, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* WLAN_RESET */
};
static void init_Bluetooth_gpio_table(void)
{
	int pin,rc;

	printk(KERN_INFO "Config Bluetooth GPIO\n");
	
		for (pin = 0; pin < ARRAY_SIZE(bt_config_init); pin++) {
			//printk(KERN_INFO " set gpio table entry %d\n",pin);
			rc = gpio_tlmm_config(bt_config_init[pin],GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_init[pin], rc);
/* FIH, JamesKCTung, 2009/06/30 { */
#ifndef CONFIG_FIH_FXX
				return -EIO;
#endif
/* } FIH, JamesKCTung, 2009/06/30 */
			}
/* FIH, JamesKCTung, 2009/06/30 { */
#ifndef CONFIG_FIH_FXX
			mdelay(200);
#endif
/* } FIH, JamesKCTung, 2009/06/30 */
		}
		
/* FIH, JamesKCTung, 2009/06/30 { */
#ifdef CONFIG_FIH_FXX
    rc = gpio_request(96, "WIFI_PWD");
    if (rc)	printk(KERN_ERR "%s: WIFI_PWD 96 setting failed! rc = %d\n", __func__, rc);
    rc = gpio_request(76, "3.3V");
    if (rc)	printk(KERN_ERR "%s: 3.3V 76 setting failed! rc = %d\n", __func__, rc);
    rc = gpio_request(77, "1.8V");
    if (rc)	printk(KERN_ERR "%s: 1.8V 77 setting failed! rc = %d\n", __func__, rc);
    rc = gpio_request(34, "1.2V");
    if (rc)	printk(KERN_ERR "%s: 1.2V 34 setting failed! rc = %d\n", __func__, rc);
    rc = gpio_request(35, "WIFI_RST");
    if (rc)	printk(KERN_ERR "%s: WIFI_RST 35 setting failed! rc = %d\n", __func__, rc);
    rc = gpio_request(27, "BT_RST");
    if (rc)	printk(KERN_ERR "%s: BT_RST 27 setting failed! rc = %d\n", __func__, rc);
#endif
/* } FIH, JamesKCTung, 2009/06/30 */

}

/* FIH, JamesKCTung, 2009/06/30 { */
static int bluetooth_power(int on)
{
    int module_status=0,prev_status=0;
    bool bConfigWIFI;
/* FIH, SimonSSChang, 2010/02/26 { */
/* let ar6000 driver to turn on/off power when enter suspend/resume */
#ifdef CONFIG_FIH_FXX
    bool bConfigWIFI_suspend;
#endif
/* } FIH, SimonSSChang, 2010/02/26 */
    int value = 0;
/* FIH, WilsonWHLee, 2009/07/30 { */
/* [FXX_CR], re-configure GPIO when BT turn on/off */
#if	CONFIG_FIH_FXX 
	gpio_tlmm_config(GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);	/* RFR */
	gpio_tlmm_config(GPIO_CFG(44, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);	        /* CTS */
	gpio_tlmm_config(GPIO_CFG(45, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);	        /* Rx */
	gpio_tlmm_config(GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);	/* Tx */
	gpio_tlmm_config(GPIO_CFG(37, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);  /* HOST_WAKE_BT */
	gpio_tlmm_config(GPIO_CFG(42, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);          /* BT_WAKE_HOST */
	gpio_tlmm_config(GPIO_CFG(27, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);   /* BT_RST */
	gpio_tlmm_config(GPIO_CFG(76, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);   /* 3.3V */
	gpio_tlmm_config(GPIO_CFG(77, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);   /* 1.5V */
	gpio_tlmm_config(GPIO_CFG(34, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);   /* 1.2V */
#endif
	spin_lock(&wif_bt_lock);

    bConfigWIFI = (on & WIFI_CONTROL_MASK);
/* FIH, SimonSSChang, 2010/02/26 { */
/* let ar6000 driver to turn on/off power when enter suspend/resume */
#ifdef CONFIG_FIH_FXX
    bConfigWIFI_suspend = (on & WIFI_SUSPEND_CONTROL_MASK);
    printk(KERN_INFO "on = 0x%08x", on);
#endif
/* } FIH, SimonSSChang, 2010/02/26 */

    if(bConfigWIFI)
    {
        prev_status = wifi_status;
/* FIH, SimonSSChang, 2010/02/26 { */
/* let ar6000 driver to turn on/off power when enter suspend/resume */
#ifdef CONFIG_FIH_FXX
        wifi_status = on & ~(WIFI_CONTROL_MASK | WIFI_SUSPEND_CONTROL_MASK); 
#else
        wifi_status = on & ~(WIFI_CONTROL_MASK); 
#endif
/* } FIH, SimonSSChang, 2010/02/26 */

        if( wifi_status == prev_status )
        {
            printk(KERN_ERR "%s: WIFI already turn %s\n", __func__,  (wifi_status?"ON":"OFF") );
            spin_unlock(&wif_bt_lock);
            return 0;
        }
        if(wifi_status && !bt_status)
            module_status = MODULE_TURN_ON;
        else if(!wifi_status && !bt_status)
            module_status = MODULE_TURN_OFF;

    }else {
        prev_status = bt_status;
        bt_status = on;
        if( bt_status == prev_status )
        {
            printk(KERN_ERR "%s: BT already turn %s\n", __func__,  (bt_status?"ON":"OFF") );
            spin_unlock(&wif_bt_lock);
            return 0;
        }
        if(bt_status && !wifi_status)
            module_status = MODULE_TURN_ON;
        else if(!wifi_status && !bt_status)
            module_status = MODULE_TURN_OFF;
    }

    //power control before module on/off
    if(!bConfigWIFI &&  !bt_status) {     //Turn BT off
        printk(KERN_DEBUG "%s : Turn BT off.\n", __func__);
		gpio_direction_output(27,0);    
    }else if(!bConfigWIFI &&  bt_status){     //Turn BT on        
        printk(KERN_DEBUG "%s : Turn BT on.\n", __func__);
    }else if(bConfigWIFI && wifi_status) {  //Turn WIFI on
        printk(KERN_DEBUG "%s : Turn WIFI on.\n", __func__);
        gpio_direction_output(96,0);
        gpio_direction_output(35,0);
    }else if(bConfigWIFI && !wifi_status) {  //Turn WIFI OFF
        printk(KERN_DEBUG "%s : Turn WIFI off.\n", __func__);

/* FIH, SimonSSChang, 2010/02/26 { */
/* let ar6000 driver to turn on/off power when enter suspend/resume */
#ifdef CONFIG_FIH_FXX
        if(!bConfigWIFI_suspend) {
        if(ar6k_wifi_status_cb) {
            wifi_power_on=0;
            ar6k_wifi_status_cb(0,ar6k_wifi_status_cb_devid);
        }else
            printk(KERN_ERR "!!!wifi_power Fail:  ar6k_wifi_status_cb_devid is NULL \n");

        gpio_direction_output(96,0);
        gpio_direction_output(35,0);
    }
    }
#else
        if(ar6k_wifi_status_cb) {
            wifi_power_on=0;
            ar6k_wifi_status_cb(0,ar6k_wifi_status_cb_devid);
        }else
            printk(KERN_ERR "!!!wifi_power Fail:  ar6k_wifi_status_cb_devid is NULL \n");

        gpio_direction_output(96,0);
        gpio_direction_output(35,0);
    }
#endif
/* } FIH, SimonSSChang, 2010/02/26 */

    //Turn module on/off
    if(module_status == MODULE_TURN_ON) {   //turn module on
        printk(KERN_DEBUG "%s : Turn module(A22) on.\n", __func__);
        //FIH_ADQ.B.1741 turn on BT is too bad
        gpio_direction_output(76,1);
        value = 0;
        value = gpio_get_value(76);
        printk(KERN_DEBUG "%s : GPIO 76 is %d.\n", __func__, value);
        //mdelay(10);
        gpio_direction_output(77,1);
        value = 0;
        value = gpio_get_value(77);
        printk(KERN_DEBUG "%s : GPIO 77 is %d.\n", __func__, value);
        //mdelay(10);
        gpio_direction_output(34,1);
        value = 0;
        value = gpio_get_value(34);
        printk(KERN_DEBUG "%s : GPIO 34 is %d.\n", __func__, value);
        //mdelay(10);
    }else if(module_status == MODULE_TURN_OFF) { //turn module off
        printk(KERN_DEBUG "%s : Turn module(A22) off.\n", __func__);
        gpio_direction_output(34,0);
        gpio_direction_output(77,0);
        gpio_direction_output(76,0);
    }

    if(!bConfigWIFI &&  !bt_status) {  //Turn BT off
/* FIH, WilsonWHLee, 2009/07/30 { */
/* [FXX_CR], re-configure GPIO when BT turn on/off */
#if	CONFIG_FIH_FXX  
       gpio_tlmm_config(GPIO_CFG(43, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);	/* RFR */
	   gpio_tlmm_config(GPIO_CFG(44, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);	/* CTS */
	   gpio_tlmm_config(GPIO_CFG(45, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);	/* Rx */
	   gpio_tlmm_config(GPIO_CFG(46, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);	/* Tx */
	   gpio_tlmm_config(GPIO_CFG(37, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE); /* HOST_WAKE_BT */
	   gpio_tlmm_config(GPIO_CFG(42, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE); /* BT_WAKE_HOST */
#endif
    }else if(!bConfigWIFI &&  bt_status){    //Turn BT on
    	//FIH_ADQ.B.1741 turn on BT is too bad
        //gpio_direction_output(27,1);
        //mdelay(200);
        gpio_direction_output(27,0);
        mdelay(10);
        gpio_direction_output(27,1);
        value = 0;
        value = gpio_get_value(27);
        printk(KERN_DEBUG "%s : GPIO 27 is %d.\n", __func__, value);
        mdelay(10);
    }else if(bConfigWIFI && wifi_status) { //Turn WIFI on
        gpio_direction_output(96,1);
        value = 0;
        value = gpio_get_value(96);
        printk(KERN_DEBUG "%s : GPIO 96 is %d.\n", __func__, value);
        mdelay(10);
        gpio_direction_output(35,1);
        value = 0;
        value = gpio_get_value(35);
        printk(KERN_DEBUG "%s : GPIO 35 is %d.\n", __func__, value);

/* FIH, SimonSSChang, 2010/02/26 { */
/* let ar6000 driver to turn on/off power when enter suspend/resume */
#ifdef CONFIG_FIH_FXX
        if(!bConfigWIFI_suspend) {
        if(ar6k_wifi_status_cb) {
            wifi_power_on=1;
            ar6k_wifi_status_cb(1,ar6k_wifi_status_cb_devid);
        }else
            printk(KERN_ERR "!!!wifi_power Fail:  ar6k_wifi_status_cb_devid is NULL \n");
        }
#else
        if(ar6k_wifi_status_cb) {
            wifi_power_on=1;
            ar6k_wifi_status_cb(1,ar6k_wifi_status_cb_devid);
        }else
            printk(KERN_ERR "!!!wifi_power Fail:  ar6k_wifi_status_cb_devid is NULL \n");
#endif
/* } FIH, SimonSSChang, 2010/02/26 */
    }else if(bConfigWIFI && !wifi_status) {  //Turn WIFI OFF        
    }

    spin_unlock(&wif_bt_lock);

	return 0;
}
/* FIH, SimonSSChang, 2010/02/26 { */
/* let ar6000 driver to turn on/off power when enter suspend/resume */
#ifdef CONFIG_FIH_FXX
int wifi_power(int on)
{
    int ret;
    ret = bluetooth_power(on);
    printk(KERN_INFO "wifi_power ret = %d\n", ret);
    return ret;
}
EXPORT_SYMBOL(wifi_power);
#endif
/* } FIH, SimonSSChang, 2010/02/26 */

/* } FIH, JamesKCTung, 2009/06/30 */

static void __init bt_power_init(void)
{
	msm_bt_power_device.dev.platform_data = &bluetooth_power;
}
#else
#define bt_power_init(x) do {} while (0)
#endif

static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= 42,
		.end	= 42,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= 37,
		.end	= 37,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(42),
		.end	= MSM_GPIO_TO_INT(42),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};


#ifdef CONFIG_ARCH_MSM7X27
static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "kgsl_phys_memory",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "kgsl_yamato_irq",
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_platform_data kgsl_pdata;

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};
#endif

static struct platform_device msm_device_pmic_leds = {
	.name   = "pmic-leds",
	.id = -1,
};



/* FIH, Michael Kao, 2010/01/21{ */
/* [FXX_CR], Add For TC6507 LED Expander */
static struct tca6507_platform_data tca6507_data = {
	.tca6507_reset = 84,
};
static struct elan_i2c_platform_data elan8232_device = {
    .abs_x_min= 0,
    .abs_x_max= 1792,
    .abs_y_min= 0,
    .abs_y_max= 2816,
    .intr_gpio= 89,
};

//Added for capacitive touch panel, by Stanley-- 2009/06/03
static struct i2c_board_info i2c_devices[] = {
/* FIH, Neo Chen, 2009/07/02 { */
/* [FXX_CR], change backlight driver from max7302 to max8831 */
    {
	    I2C_BOARD_INFO("max8831", 0x9a>>1),
    },
/* } FIH, Neo Chen, 2009/07/02 */

/* FIH; Tiger; 2009/4/14 { */
        {
                I2C_BOARD_INFO("MS3C", 0x2e),
        },
        {
                I2C_BOARD_INFO("SMB380", 0x38),
        },
/* } FIH; Tiger; 2009/4/14 */

/* FIH, Charles Huang, 2009/05/11 { */
/* [FXX_CR], camera sensor ov5642/ov3642 */
#ifdef CONFIG_OV3642
	{
		/* Fake address for multi sensors with the same adress */
		I2C_BOARD_INFO("ov3642", 0xFE),
	},
#endif
#ifdef CONFIG_OV5642AF
	{
		I2C_BOARD_INFO("ov5642af", 0xFD),
	},
#endif
#ifdef CONFIG_OV5642
	{
		I2C_BOARD_INFO("ov5642", 0x3C),
	},
#endif
/* } FIH, Charles Huang, 2009/05/11 */

/* FIH, Charles Huang, 2009/05/11 { */
/* [FXX_CR], flash driver aat1272 */
/* Because read and write slave address are different, we define slave address in driver */
#ifdef CONFIG_FIH_FXX
	{
		I2C_BOARD_INFO("aat1272", 0x6F >> 1),	
	},
#endif
/* } FIH, Charles Huang, 2009/05/11 */

	 /* FIH, Michael Kao, 2010/01/21/{ */
	/* [FXX_CR], Add For TC6507 LED Expander */
	#ifdef CONFIG_FIH_FXX
	{
	I2C_BOARD_INFO("tca6507", 0x8A >> 1),
	.platform_data = &tca6507_data,
	},
	#endif
	 /* FIH, Michael Kao, 2010/01/21{ */
//Added for capacitive touch panel, by Stanley++ 2009/06/03
/* Add support for elan8232 i2c touchpanel */
{
    I2C_BOARD_INFO("bi8232", 0x10),
    .irq            = MSM_GPIO_TO_INT(89),
    .platform_data  = &elan8232_device,
},
//Added for capacitive touch panel, by Stanley-- 2009/06/03
};

#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
/* FIH, Charles Huang, 2009/05/19 { */
/* [FXX_CR], disable unused gpio */
#if 0
	GPIO_CFG(0,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
#endif
/* } FIH, Charles Huang, 2009/05/19 */
	GPIO_CFG(2,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
/* FIH, Charles Huang, 2009/05/19 { */
/* [FXX_CR], disable unused gpio */
#if 0
	GPIO_CFG(0,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
#endif
/* } FIH, Charles Huang, 2009/05/19 */
	GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
	};
/* FIH, PeterKCTseng, @20090521 { */
/* Config GPIO for keypad         */
#ifdef CONFIG_FIH_F9xx_GPIO_KEYPAD
static uint32_t keypad_gpio_table[] = {
	/* parallel CAMERA interfaces */
//	GPIO_CFG(41,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* Volume Up Key    */
//	GPIO_CFG(36,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* Volume Down Key  */
//	GPIO_CFG(28,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* Send key         */
//	GPIO_CFG(19,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* End Key          */
/* FIH, PeterKCTseng, @20090525 { */
/* Config GPIO for keypad         */
	GPIO_CFG(41,  0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA), /* Volume Up Key    */
	GPIO_CFG(36,  0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA), /* Volume Down Key  */
	GPIO_CFG(28,  0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA), /* Send key         */
	GPIO_CFG(19,  0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA), /* End Key          */
/* } FIH, PeterKCTseng, @20090525 */

//	GPIO_CFG(20,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* Camera Key1      */
//	GPIO_CFG(29,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* Camera Key2      */
//	GPIO_CFG(94,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* Hook Key         */
};
#endif /* CONFIG_FIH_F9xx_GPIO_KEYPAD */
/* } FIH, PeterKCTseng, @20090521 */

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static struct vreg *vreg_gp2;
static struct vreg *vreg_gp3;

static void msm_camera_vreg_config(int vreg_en)
{
	int rc;

	if (vreg_gp2 == NULL) {
		vreg_gp2 = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_gp2)) {
			printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp2", PTR_ERR(vreg_gp2));
			return;
		}

		rc = vreg_set_level(vreg_gp2, 1800);
		if (rc) {
			printk(KERN_ERR "%s: GP2 set_level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_gp3 == NULL) {
		vreg_gp3 = vreg_get(NULL, "gp3");
		if (IS_ERR(vreg_gp3)) {
			printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp3", PTR_ERR(vreg_gp3));
			return;
		}

		rc = vreg_set_level(vreg_gp3, 2850);
		if (rc) {
			printk(KERN_ERR "%s: GP3 set level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_en) {
		rc = vreg_enable(vreg_gp2);
		if (rc) {
			printk(KERN_ERR "%s: GP2 enable failed (%d)\n",
				 __func__, rc);
		}

		rc = vreg_enable(vreg_gp3);
		if (rc) {
			printk(KERN_ERR "%s: GP3 enable failed (%d)\n",
				__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_gp2);
		if (rc) {
			printk(KERN_ERR "%s: GP2 disable failed (%d)\n",
				 __func__, rc);
		}

		rc = vreg_disable(vreg_gp3);
		if (rc) {
			printk(KERN_ERR "%s: GP3 disable failed (%d)\n",
				__func__, rc);
		}
	}
}

static void config_camera_on_gpios(void)
{
	int vreg_en = 1;

	if (machine_is_msm7x25_ffa() ||
	    machine_is_msm7x27_ffa())
		msm_camera_vreg_config(vreg_en);

	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
	int vreg_en = 0;

	if (machine_is_msm7x25_ffa() ||
	    machine_is_msm7x27_ffa())
		msm_camera_vreg_config(vreg_en);

	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}
/* FIH, PeterKCTseng, @20090521 { */
/* Config GPIO for keypad         */
#ifdef CONFIG_FIH_F9xx_GPIO_KEYPAD
static void config_keypad_gpios(void)
{
	config_gpio_table(keypad_gpio_table,
		ARRAY_SIZE(keypad_gpio_table));
}
#endif /* CONFIG_FIH_F9xx_GPIO_KEYPAD */
/* } FIH, PeterKCTseng, @20090521 */

/* FIH, PeterKCTseng, @20090604 { */
/* Config GPIO for keypad         */
#ifdef CONFIG_FIH_F9xx_GPIO_KEYPAD
static struct Q7x27_kybd_platform_data q7x27_kybd_data = {
	.keypad_gpio = config_keypad_gpios,
	.volup_pin = 28,
	.voldn_pin = 19,
	.key_1_pin = 41,
	.key_2_pin = 36,	
	.cam_sw_t_pin = 20,
	.cam_sw_f_pin = 29,
	.hook_sw_pin = 94,
};

static struct platform_device q7x27_kybd_device = {
        .name = "7x27_kybd",
        .dev = {
                .platform_data = &q7x27_kybd_data,
        },
};
#endif /* CONFIG_FIH_F9xx_GPIO_KEYPAD */
/* } FIH, PeterKCTseng, @20090521 */
/* FIH, PeterKCTseng, @20090521 { */
/* Config GPIO for keypad         */
#ifdef CONFIG_FIH_F9xx_GPIO_KEYPAD
static void __init keypad_gpio_init(void)
{
	config_keypad_gpios();
}
#endif /* CONFIG_FIH_F9xx_GPIO_KEYPAD */
/* } FIH, PeterKCTseng, @20090521 */
/* FIH, Kenny Chu, 2009/06/04 {*/
// add vibrator
static struct platform_device pmic_rpc_device = {
        .name	= "pmic_rpc",
        .id		= -1,
};
/* } FIH, Kenny Chu, 2009/06/04 */

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};


static struct msm_camera_sensor_flash_src msm_flash_src = {
        .flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
        ._fsrc.pmic_src.low_current  = 30,
        ._fsrc.pmic_src.high_current = 100,
};

#ifdef CONFIG_OV5642

static struct msm_camera_sensor_flash_data flash_ov5642 = {
        .flash_type = MSM_CAMERA_FLASH_LED,
        .flash_src  = &msm_flash_src
};
 
static struct msm_camera_sensor_info msm_camera_sensor_ov5642_data = {
	.sensor_name    = "ov5642",
	.sensor_reset   = 0,
	.sensor_pwd     = 31,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_ov5642,
	//.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_ov5642 = {
	.name      = "msm_camera_ov5642",
	.dev       = {
		.platform_data = &msm_camera_sensor_ov5642_data,
	},
};
#endif
/* } FIH, Charles Huang, 2009/05/11 */

/* FIH, Charles Huang, 2009/05/11 { */
/* [FXX_CR], camera sensor ov5642af */
#ifdef CONFIG_OV5642AF
static struct msm_camera_sensor_info msm_camera_sensor_ov5642af_data = {
	.sensor_name    = "ov5642af",
	.sensor_reset   = 0,
	.sensor_pwd     = 31,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	//.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_ov5642af = {
	.name      = "msm_camera_ov5642af",
	.dev       = {
		.platform_data = &msm_camera_sensor_ov5642af_data,
	},
};
#endif
/* } FIH, Charles Huang, 2009/05/11 */

/* FIH, Charles Huang, 2009/05/11 { */
/* [FXX_CR], camera sensor ov3642 */
#ifdef CONFIG_OV3642
static struct msm_camera_sensor_info msm_camera_sensor_ov3642_data = {
	.sensor_name    = "ov3642",
	.sensor_reset   = 0,
	.sensor_pwd     = 31,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	//.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_ov3642 = {
	.name      = "msm_camera_ov3642",
	.dev       = {
		.platform_data = &msm_camera_sensor_ov3642_data,
	},
};
#endif
/* } FIH, Charles Huang, 2009/05/11 */

#ifdef CONFIG_MT9D112
static struct msm_camera_sensor_info msm_camera_sensor_mt9d112_data = {
	.sensor_name    = "mt9d112",
	.sensor_reset   = 89,
	.sensor_pwd     = 85,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	//.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_mt9d112 = {
	.name      = "msm_camera_mt9d112",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9d112_data,
	},
};
#endif

#ifdef CONFIG_S5K3E2FX
static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name    = "s5k3e2fx",
	.sensor_reset   = 89,
	.sensor_pwd     = 85,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	//.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name      = "msm_camera_s5k3e2fx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};
#endif

#ifdef CONFIG_MT9P012
static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
	.sensor_name    = "mt9p012",
	.sensor_reset   = 89,
	.sensor_pwd     = 85,
	.vcm_pwd        = 88,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	//.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_mt9p012 = {
	.name      = "msm_camera_mt9p012",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p012_data,
	},
};
#endif

#ifdef CONFIG_MT9P012_KM
static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_km_data = {
	.sensor_name    = "mt9p012_km",
	.sensor_reset   = 89,
	.sensor_pwd     = 85,
	.vcm_pwd        = 88,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	//.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_mt9p012_km = {
	.name      = "msm_camera_mt9p012_km",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p012_km_data,
	},
};
#endif

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name    = "mt9t013",
	.sensor_reset   = 89,
	.sensor_pwd     = 85,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	//.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name      = "msm_camera_mt9t013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};
#endif

#ifdef CONFIG_VB6801
static struct msm_camera_sensor_info msm_camera_sensor_vb6801_data = {
	.sensor_name    = "vb6801",
	.sensor_reset   = 89,
	.sensor_pwd     = 88,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	//.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_vb6801 = {
	.name      = "msm_camera_vb6801",
	.dev       = {
		.platform_data = &msm_camera_sensor_vb6801_data,
	},
};
#endif
#endif

/* FIH, Chandler Kang, 2009/05/18 { */
/* lcm_innolux gpio */
#ifdef CONFIG_SPI_GPIO

static unsigned spi_gpio_config_input[] = {
	    GPIO_CFG(101, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),  /* clk */
        GPIO_CFG(102, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),  /* cs */
        GPIO_CFG(132, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  // no use
        GPIO_CFG(131, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),  /* mosi */
};


static int __init spi_gpio_init_fake(void)
{
	int rc = 0, pin;

    return 0; 
    
    for (pin = 0; pin < ARRAY_SIZE(spi_gpio_config_input); pin++) {
        rc = gpio_tlmm_config(spi_gpio_config_input[pin], GPIO_ENABLE);
        if (rc) {
            printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",__func__, spi_gpio_config_input[pin], rc);
            return -EIO;
        }
    }

    rc = gpio_tlmm_config(GPIO_CFG(85, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
    rc = gpio_tlmm_config(GPIO_CFG(103, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
        
    rc = gpio_request(85, "cam_pwr");

    rc = gpio_request(103, "lcd_reset");

    gpio_direction_output(85,1);
    gpio_direction_output(103,1);
	mdelay(500);
	printk(KERN_INFO "%s: (103) gpio_read = %d\n", __func__, gpio_get_value(103));

	gpio_direction_output(103,0);
	mdelay(500);
	printk(KERN_INFO "%s: (103) gpio_read = %d\n", __func__, gpio_get_value(103));

	gpio_direction_output(103,1);
	mdelay(50);
	printk(KERN_INFO "%s: (85) gpio_read = %d\n", __func__, gpio_get_value(85));
	printk(KERN_INFO "%s: (103) gpio_read = %d\n", __func__, gpio_get_value(103));

    gpio_free(85);
    gpio_free(103);

	rc = gpio_request(101, "gpio_spi");
    rc = gpio_request(102, "gpio_spi");

    rc = gpio_request(131, "gpio_spi");
    rc = gpio_request(132, "gpio_spi");

	gpio_direction_output(101,1);
    printk(KERN_INFO "%s: (101) gpio_read = %d\n", __func__, gpio_get_value(101));
	gpio_direction_output(102,1);
    printk(KERN_INFO "%s: (102) gpio_read = %d\n", __func__, gpio_get_value(102));

	gpio_direction_input(132);
	gpio_direction_output(131,1);	

	
	gpio_free(101);
    gpio_free(102);
    gpio_free(131);
    gpio_free(132);               
	
	return rc;
}


static struct spi_board_info lcdc_spi_devices[] = {
        {
                .modalias = "lcdc_spi",
              	.max_speed_hz = 10000000,
                .chip_select = 0,
                .controller_data = (void *) 102,
        		.mode           = SPI_MODE_3,
        },
};


struct spi_gpio_platform_data lcdc_spigpio_platform_data = { 
        .sck = 101,
        .mosi = 131,
        .miso = 132,
        .num_chipselect = 1,
};

static struct platform_device lcdc_spigpio_device = {

        .name = "spi_gpio",
        .dev = {
                .platform_data = &lcdc_spigpio_platform_data,
        },
};
#endif  //CONFIG_SPI_GPIO
/* FIH, Chandler Kang, 2009/05/18 } */

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design 	= 2800,
	.voltage_max_design	= 4300,
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity	= &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage   = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage  = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
		/ (high_voltage - low_voltage);
}

static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

/* FIH, SimonSSChang, 2009/02/26 { */
/* ATHENV */
static struct platform_device msm_wlan_ar6000_pm_device = {
	.name		= "wlan_ar6000_pm_dev",
	.id		= 1,
	.num_resources	= 0,
	.resource	= NULL,
};
/* ATHENV */
/* } FIH, SimonSSChang, 2009/02/26 */

static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_ANDROID_RAM_CONSOLE
    &ram_console_device,
#endif
/* FIH, SimonSSChang, 2009/02/26 { */
/* ATHENV */
	&msm_wlan_ar6000_pm_device,
/* ATHENV */
/* } FIH, SimonSSChang, 2009/02/26 */
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart3,
#endif
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,
	&msm_device_otg,
	&msm_device_hsusb_otg,
	&msm_device_hsusb_host,
	&msm_device_hsusb_peripheral,
	&msm_device_gadget_peripheral,
#ifdef CONFIG_USB_FUNCTION
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
#endif
	&msm_device_i2c,
	&msm_device_tssc,
	&android_pmem_kernel_ebi1_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
#ifdef CONFIG_ARCH_MSM7X27
#ifndef CONFIG_MSM_KGSL_MMU
	&android_pmem_gpu1_device,
#endif //CONFIG_MSM_KGSL_MMU
#endif
/* FIH, Chandler Kang, 2009/05/18 { */
#ifdef CONFIG_SPI_GPIO
	&lcdc_spigpio_device,
#endif	 //CONFIG_SPI_GPIO
/* FIH, Chandler Kang, 2009/05/18 } */
	&msm_fb_device,
	&msm_device_uart_dm1,
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
	&msm_device_pmic_leds,
	&msm_device_snd,
	&msm_device_adspdec,
/* FIH, PeterKCTseng, @20090521 { */
/* Config GPIO for keypad         */
#ifdef CONFIG_FIH_F9xx_GPIO_KEYPAD
	&q7x27_kybd_device,
#endif /* CONFIG_FIH_F9xx_GPIO_KEYPAD */
/* } FIH, PeterKCTseng, @20090521 */
/* FIH, Kenny Chu, 2009/06/04 {*/
// add vibrator
    &pmic_rpc_device,
/* } FIH, Kenny Chu, 2009/06/04 */
/* FIH, Charles Huang, 2009/05/11 { */
/* [FXX_CR], camera sensor ov5642 */
#ifdef CONFIG_OV5642
	&msm_camera_sensor_ov5642,
#endif
#ifdef CONFIG_OV5642AF
	&msm_camera_sensor_ov5642af,
#endif
#ifdef CONFIG_OV3642
	&msm_camera_sensor_ov3642,
#endif
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
#ifdef CONFIG_MT9D112
	&msm_camera_sensor_mt9d112,
#endif
#ifdef CONFIG_S5K3E2FX
	&msm_camera_sensor_s5k3e2fx,
#endif
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
#endif
#ifdef CONFIG_MT9P012_KM
	&msm_camera_sensor_mt9p012_km,
#endif
#ifdef CONFIG_VB6801
	&msm_camera_sensor_vb6801,
#endif
	&msm_bluesleep_device,

	&headset_sensor_device, 
// --- FIH, KarenLiao, 20090518: Add for headset detection.
#ifdef CONFIG_ARCH_MSM7X27
	&msm_device_kgsl,
#endif
/* [FXX_CR], Add ALS & PS driver into platform driver*/
	&ALSPS_sensor_device, 
/* implement suspend/resume for jogball */
	&mtb_platform_device,
/* } FIH; Tiger; 2009/6/22 */
	&hs_device,
	&msm_batt_device,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", 0);
	msm_fb_register_device("lcdc", &lcdc_pdata);
	/* [FXX_CR], Add For Battery Driver*/
	msm_fb_register_device("batt", 0);
	/* } FIH, Michael Kao, 2010/01/21 */
}

extern struct sys_timer msm_timer;

static void __init msm7x2x_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data msm7x2x_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.max_axi_khz = 160000,
};

void msm_serial_debug_init(unsigned int base, int irq,
			   struct device *clk_device, int signal_irq);

#ifdef CONFIG_MMC
static void sdcc_gpio_init(void)
{
	/* SDC1 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (gpio_request(51, "sdc1_data_3"))
		pr_err("failed to request gpio sdc1_data_3\n");
	if (gpio_request(52, "sdc1_data_2"))
		pr_err("failed to request gpio sdc1_data_2\n");
	if (gpio_request(53, "sdc1_data_1"))
		pr_err("failed to request gpio sdc1_data_1\n");
	if (gpio_request(54, "sdc1_data_0"))
		pr_err("failed to request gpio sdc1_data_0\n");
	if (gpio_request(55, "sdc1_cmd"))
		pr_err("failed to request gpio sdc1_cmd\n");
	if (gpio_request(56, "sdc1_clk"))
		pr_err("failed to request gpio sdc1_clk\n");
#endif
	if (machine_is_msm7x25_ffa())
		return;

	/* SDC2 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (gpio_request(62, "sdc2_clk"))
		pr_err("failed to request gpio sdc2_clk\n");
	if (gpio_request(63, "sdc2_cmd"))
		pr_err("failed to request gpio sdc2_cmd\n");
	if (gpio_request(64, "sdc2_data_3"))
		pr_err("failed to request gpio sdc2_data_3\n");
	if (gpio_request(65, "sdc2_data_2"))
		pr_err("failed to request gpio sdc2_data_2\n");
	if (gpio_request(66, "sdc2_data_1"))
		pr_err("failed to request gpio sdc2_data_1\n");
	if (gpio_request(67, "sdc2_data_0"))
		pr_err("failed to request gpio sdc2_data_0\n");
#endif
	if (machine_is_msm7x27_ffa())
		return;

	/* SDC3 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	if (gpio_request(88, "sdc3_clk"))
		pr_err("failed to request gpio sdc3_clk\n");
	if (gpio_request(89, "sdc3_cmd"))
		pr_err("failed to request gpio sdc3_cmd\n");
	if (gpio_request(90, "sdc3_data_3"))
		pr_err("failed to request gpio sdc3_data_3\n");
	if (gpio_request(91, "sdc3_data_2"))
		pr_err("failed to request gpio sdc3_data_2\n");
	if (gpio_request(92, "sdc3_data_1"))
		pr_err("failed to request gpio sdc3_data_1\n");
	if (gpio_request(93, "sdc3_data_0"))
		pr_err("failed to request gpio sdc3_data_0\n");
#endif

	/* SDC4 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	if (gpio_request(19, "sdc4_data_3"))
		pr_err("failed to request gpio sdc4_data_3\n");
	if (gpio_request(20, "sdc4_data_2"))
		pr_err("failed to request gpio sdc4_data_2\n");
	if (gpio_request(21, "sdc4_data_1"))
		pr_err("failed to request gpio sdc4_data_1\n");
	if (gpio_request(107, "sdc4_cmd"))
		pr_err("failed to request gpio sdc4_cmd\n");
	if (gpio_request(108, "sdc4_data_0"))
		pr_err("failed to request gpio sdc4_data_0\n");
	if (gpio_request(109, "sdc4_clk"))
		pr_err("failed to request gpio sdc4_clk\n");
#endif
}

static unsigned sdcc_cfg_data[][6] = {
	/* SDC1 configs */
	{
	GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	},
	/* SDC2 configs */
	{
	GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	},
	/* SDC3 configs */
	{
	GPIO_CFG(88, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(89, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(90, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(91, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(92, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(93, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	},
	/* SDC4 configs */
	{
	GPIO_CFG(19, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(20, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(21, 4, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(107, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(108, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(109, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	}
};

/* FIH, BillHJChang, 2009/07/20 { */
/* [FXX_CR], Set power state of SD bus when card remove */
static unsigned sdcc_cfg_pulldown[][6] = {
	/* SDC1 configs */
	{
	GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA),
	GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA),
	GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA),
	GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA),
	GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA),
	GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA),
	}
};

static unsigned long vreg_sts, gpio_sts;
static unsigned mpp_mmc = 2;
static struct vreg *vreg_mmc;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int i, rc;

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable)
		set_bit(dev_id, &gpio_sts);
	else
		clear_bit(dev_id, &gpio_sts);

/* [FXX_CR], Set power state of SD bus when card remove */
// Modified Code
	if (enable)
	{
		for (i = 0; i < ARRAY_SIZE(sdcc_cfg_data[dev_id - 1]); i++) {
			rc = gpio_tlmm_config(sdcc_cfg_data[dev_id - 1][i],
				enable ? GPIO_ENABLE : GPIO_DISABLE);
			if (rc )
				printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, sdcc_cfg_data[dev_id - 1][i], rc);
		}	
		printk(KERN_INFO"%s: (pull up)\n",__func__);
	}
	else
	{
		for (i = 0; i < ARRAY_SIZE(sdcc_cfg_pulldown[dev_id - 1]); i++) {
			rc = gpio_tlmm_config(sdcc_cfg_pulldown[dev_id - 1][i],
				!enable ? GPIO_ENABLE : GPIO_DISABLE);
			if (rc)
				printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, sdcc_cfg_pulldown[dev_id - 1][i], rc);
		}	
		printk(KERN_INFO"%s: (pull down)\n",__func__);
	}


	// Setup SD card detection interrupt pin
	rc = gpio_tlmm_config(GPIO_CFG(18, 0, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
					  			
	if (rc)
			printk(KERN_ERR "%s: ERROR!! config SD detect pin =%d\n",__func__, rc);
    /* FIH, MonkyChen, 2010/02/10 } */
}

/* FIH, JamesKCTung, 2009/07/09 { */
#ifdef CONFIG_AR6K
static int  ar6k_wifi_suspend(int dev_id)
{
    bluetooth_power(WIFI_CONTROL_MASK | 0);  
    return 1;
}

static int  ar6k_wifi_resume(int dev_id)
{
    bluetooth_power(WIFI_CONTROL_MASK | 1);    
    return 1;
}

static uint32_t msm_ar6k_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
    return 0;
}
#endif
/* } FIH, JamesKCTung, 2009/07/09 */
static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	/* FIH, BillHJChang, 2009/07/22 { */
	/* [FXX_CR], 7627 SD card power supply source changed*/
	if (vreg_mmc == NULL)
	{
		int iHwid = 0;
		iHwid = FIH_READ_HWID_FROM_SMEM();

		if(iHwid >= CMCS_7627_EVB1)
		{
			vreg_mmc = vreg_get(NULL, "gp5");
			printk(KERN_INFO"%s: vreg_get from VREG_GP5 !!!!!!!\n", __func__);			
		}
		else
		{
			vreg_mmc = vreg_get(NULL, "mmc");
			printk(KERN_INFO"%s: vreg_get from VREG_MMC !!!!!!!\n", __func__);					
		}

		if (IS_ERR(vreg_mmc))
			printk(KERN_ERR "%s: vreg get failed (%ld)\n",__func__, PTR_ERR(vreg_mmc));			
	}
	/* } FIH, BillHJChang, 2009/07/22 */	
	
	printk(KERN_ERR "%s: vdd: %d \n", __func__, vdd);

	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);

		if (!vreg_sts) {
			if (machine_is_msm7x25_ffa() ||
					machine_is_msm7x27_ffa()) {
				rc = mpp_config_digital_out(mpp_mmc,
				     MPP_CFG(MPP_DLOGIC_LVL_MSMP,
				     MPP_DLOGIC_OUT_CTRL_LOW));
			} else {
				rc = vreg_disable(vreg_mmc);
            }
			if (rc)
				printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		}
		return 0;
	}

	if (!vreg_sts) {
		if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
			rc = mpp_config_digital_out(mpp_mmc,
			     MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			     MPP_DLOGIC_OUT_CTRL_HIGH));
		} else {
			printk(KERN_INFO"%s: [SD card power: on]\r\n",__func__);	
			
			rc = vreg_set_level(vreg_mmc, 2850);
			if (!rc)
				rc = vreg_enable(vreg_mmc);
		}
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	set_bit(pdev->id, &vreg_sts);
	return 0;
}

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data msm7x2x_sdc1_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
};
#endif

#if 0
//not used mark it#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data msm7x2x_sdc2_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.sdiowakeup_irq = MSM_GPIO_TO_INT(66),
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm7x2x_sdc3_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm7x2x_sdc4_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
};
#endif

static void __init msm7x2x_init_mmc(void)
{
	sdcc_gpio_init();
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	msm_add_sdcc(1, &msm7x2x_sdc1_data);
#endif

	if (machine_is_msm7x25_surf() || machine_is_msm7x27_surf() ||
		machine_is_msm7x27_ffa()) {
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
/* FIH, JamesKCTung, 2009/06/03 { */
#ifdef CONFIG_AR6K
		msm_add_sdcc(2, &ar6k_wifi_data);
#else
		msm_add_sdcc(2, &msm7x2x_sdc2_data);
#endif
#endif
	}

	if (machine_is_msm7x25_surf() || machine_is_msm7x27_surf()) {
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
		msm_add_sdcc(3, &msm7x2x_sdc3_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
		msm_add_sdcc(4, &msm7x2x_sdc4_data);
#endif
	}
}
#else
#define msm7x2x_init_mmc() do {} while (0)
#endif

static struct msm_pm_platform_data msm7x25_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
};

static struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 20000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 20000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 0,
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	gpio_scl = 60;
	gpio_sda = 61;
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 400000, //FIH, Henryhyjuang raise up I2C clock from 100 Khz to 400 Khz.
	.rmutex  = 0,
	.pri_clk = 60,
	.pri_dat = 61,
	.aux_clk = 95,
	.aux_dat = 96,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(60, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(61, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");
	if (cpu_is_msm7x27())
		msm_i2c_pdata.pm_lat =
		msm7x27_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	else
		msm_i2c_pdata.pm_lat =
		msm7x25_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

void msm_init_pmic_vibrator(void);

/* FIH, Chandler Kang, 2009/05/18 { */
/* FIH_ADQ, lcm_innolux */
extern int  spi_gpio_init(void); //lcm_innolux
/* FIH, Chandler Kang, 2009/05/18 } */

// +++ FIH, KarenLiao, 20090518: Add for headset detection.
static void __init init_headset_sensor(void)
{	
	gpio_direction_input(40);
}
// --- FIH, KarenLiao, 20090518: Add for headset detection.
// +++ FIH, WillChen, 20090712: Add for device info
extern void adq_info_init(void);
// --- FIH, WillChen, 20090712: Add for device info
static void __init msm7x2x_init(void)
{
/* FIH, Debbie, 2009/09/11 { */
/* get share memory command address dynamically */
       fih_smem_alloc_for_host_used();
/* FIH, Debbie, 2009/09/11 } */

/* FIH, KennyChu, 2010/05/04, read hwid here to ensure more drivers can get it {*/
    fih_smem_alloc();
/* } FIH, KennyChu, 2010/05/04*/

	if (socinfo_init() < 0)
		BUG();
/* FIH, JamesKCTung, 2009/06/03 { */
#ifdef CONFIG_AR6K
	ar6k_wifi_status_cb=NULL; 
	ar6k_wifi_status_cb_devid=NULL;
#endif
/* } FIH, JamesKCTung, 2009/06/03 */

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1);
#endif

	if (cpu_is_msm7x27())
		msm7x2x_clock_data.max_axi_khz = 200000;

	msm_acpu_clock_init(&msm7x2x_clock_data);

#ifdef CONFIG_ARCH_MSM7X27

	/* 7x27 doesn't allow graphics clocks to be run asynchronously to */
	/* the AXI bus */
	kgsl_pdata.max_grp2d_freq = 0;
	kgsl_pdata.set_grp2d_async = NULL;
	kgsl_pdata.max_grp3d_freq = 0;
	kgsl_pdata.set_grp3d_async = NULL;
#endif

	msm_hsusb_pdata.swfi_latency =
		msm7x27_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;

	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;

	msm_gadget_pdata.swfi_latency =
		msm7x27_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
	msm_device_hsusb_host.dev.platform_data = &msm_hsusb_pdata;
    msm_read_serial_number_from_nvitem();

/* FIH, Chandler Kang, 2009-05-18 { */
#ifdef CONFIG_SPI_GPIO
	spi_gpio_init_fake();
#endif    //CONFIG_SPI_GPIO
/* FIH, Chandler Kang, 2009/05/18 } */

/* FIH, PeterKCTseng, @20090521 { */
/* Config GPIO for keypad         */
#ifdef CONFIG_FIH_F9xx_GPIO_KEYPAD
	keypad_gpio_init();
#endif /* CONFIG_FIH_F9xx_GPIO_KEYPAD */
/* } FIH, PeterKCTseng, @20090521 */

	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); /* might not be necessary */
#endif
/* FIH, Chandler Kang, 2009/05/18 { */
#ifdef CONFIG_SPI_GPIO
	spi_register_board_info(lcdc_spi_devices,ARRAY_SIZE(lcdc_spi_devices));
#endif    //CONFIG_SPI_GPIO	
/* FIH, Chandler Kang, 2009/05/18 } */
	msm_device_i2c_init();
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	msm_fb_add_devices();
	rmt_storage_add_ramfs();
	msm7x2x_init_mmc();

// +++ FIH, KarenLiao, 20090518: Add for headset detection.	
	init_headset_sensor(); 
// --- FIH, KarenLiao, 20090518: Add for headset detection.
/* FIH, Kenny Chu, 2009/06/04 {*/
// add vibrator
    msm_init_pmic_vibrator();
/* } FIH, Kenny Chu, 2009/06/04 */
/* FIH, JamesKCTung, 2009/06/03 { */
	init_Bluetooth_gpio_table();
/* } FIH, JamesKCTung, 2009/06/03 */
	bt_power_init();

	if (cpu_is_msm7x27())
		msm_pm_set_platform_data(msm7x27_pm_data);
	else
		msm_pm_set_platform_data(msm7x25_pm_data);
// +++ FIH, WillChen, 20090712: Add for device info
	adq_info_init();
// --- FIH, WillChen, 20090712: Add for device info

}

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
	pmem_kernel_ebi1_size = memparse(*p, p);
}
__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static void __init pmem_mdp_size_setup(char **p)
{
	pmem_mdp_size = memparse(*p, p);
}
__early_param("pmem_mdp_size=", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
	pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);

#ifdef CONFIG_ARCH_MSM7X27
#ifndef CONFIG_MSM_KGSL_MMU
static unsigned pmem_gpu1_size = MSM_PMEM_GPU1_SIZE;
static void __init pmem_gpu1_size_setup(char **p)
{
	pmem_gpu1_size = memparse(*p, p);
}
__early_param("pmem_gpu1_size=", pmem_gpu1_size_setup);
#endif //CONFIG_MSM_KGSL_MMU
#endif
static unsigned fb_size = MSM_FB_SIZE;
static void __init fb_size_setup(char **p)
{
	fb_size = memparse(*p, p);
}
__early_param("fb_size=", fb_size_setup);

#ifdef CONFIG_ARCH_MSM7X27
static unsigned gpu_phys_size = MSM_GPU_PHYS_SIZE;
static void __init gpu_phys_size_setup(char **p)
{
	gpu_phys_size = memparse(*p, p);
}
__early_param("gpu_phys_size=", gpu_phys_size_setup);
#endif

static void __init msm_msm7x2x_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_mdp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for mdp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}


	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

	/* RAM Console can't use alloc_bootmem(), since that zeroes the
         * region */
	size = 128 * SZ_1K;
	ram_console_resource[0].start = msm_fb_resources[0].end+1;
	ram_console_resource[0].end = ram_console_resource[0].start + size - 1;
	pr_info("allocating %lu bytes at (%lx physical) for ram console\n",
		size, (unsigned long)ram_console_resource[0].start);
	/* We still have to reserve it, though */
	reserve_bootmem(ram_console_resource[0].start,size,0);

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}
#ifdef CONFIG_ARCH_MSM7X27
	size = gpu_phys_size ? : MSM_GPU_PHYS_SIZE;
	addr = alloc_bootmem(size);
	kgsl_resources[1].start = __pa(addr);
	kgsl_resources[1].end = kgsl_resources[1].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for KGSL\n",
		size, addr, __pa(addr));

#ifndef CONFIG_MSM_KGSL_MMU
	size = pmem_gpu1_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_gpu1_pdata.start = __pa(addr);
		android_pmem_gpu1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for gpu1 "
			"pmem arena\n", size, addr, __pa(addr));
	}
#endif //CONFIG_MSM_KGSL_MMU
#endif

}

static void __init msm7x2x_map_io(void)
{
	msm_map_common_io();
	/* Technically dependent on the SoC but using machine_is
	 * macros since socinfo is not available this early and there
	 * are plans to restructure the code which will eliminate the
	 * need for socinfo.
	 */
	if (machine_is_msm7x27_surf() || machine_is_msm7x27_ffa())
		msm_clock_init(msm_clocks_7x27, msm_num_clocks_7x27);

	if (machine_is_msm7x25_surf() || machine_is_msm7x25_ffa())
		msm_clock_init(msm_clocks_7x25, msm_num_clocks_7x25);

	msm_msm7x2x_allocate_memory_regions();

#ifdef CONFIG_CACHE_L2X0
	if (machine_is_msm7x27_surf() || machine_is_msm7x27_ffa()) {
		/* 7x27 has 256KB L2 cache:
			64Kb/Way and 4-Way Associativity;
			R/W latency: 3 cycles;
			evmon/parity/share disabled. */
		l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
	}
#endif
}

MACHINE_START(MSM7X27_SURF, "QCT MSM7x27 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7X27_FFA, "QCT MSM7x27 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7X25_SURF, "QCT MSM7x25 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7X25_FFA, "QCT MSM7x25 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END
