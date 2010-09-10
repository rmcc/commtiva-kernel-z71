/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include <mach/irqs.h>
#include <mach/sirc.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include <mach/board.h>

#include "devices.h"
#include "smd_private.h"

#define NOCLOCK 1

#ifndef NOCLOCK
#include "clock-fsm9xxx.h"
#endif

#include <asm/mach/flash.h>

#include <asm/mach/mmc.h>
#ifdef CONFIG_PMIC8058
#include <linux/mfd/pmic8058.h>
#endif

#include "tlmm-fsm9xxx.h"

static struct resource resources_uart1[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

#ifdef NOTNOW
static struct resource resources_uart2[] = {
	{
		.start	= INT_UART2,
		.end	= INT_UART2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART2_PHYS,
		.end	= MSM_UART2_PHYS + MSM_UART2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif

struct platform_device msm_device_uart1 = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart1),
	.resource	= resources_uart1,
};

#ifdef NOTNOW
struct platform_device msm_device_uart2 = {
	.name	= "msm_serial",
	.id	= 1,
	.num_resources	= ARRAY_SIZE(resources_uart2),
	.resource	= resources_uart2,
};
#endif

#ifdef NOTNOW
#define MSM_I2C_SIZE          SZ_4K
#define MSM_I2C_PHYS        0x81200000

static struct resource resources_i2c[] = {
	{
		.start	= MSM_I2C_PHYS,
		.end	= MSM_I2C_PHYS + MSM_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C,
		.end	= INT_PWB_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_i2c = {
	.name		= "msm_i2c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_i2c),
	.resource	= resources_i2c,
};

#endif /* NOTNOW */

#ifdef CONFIG_I2C_SSBI
#define MSM_SSBI0_PHYS	0x94080000
static struct resource msm_ssbi0_resources[] = {
	{
		.name   = "ssbi_base",
		.start	= MSM_SSBI0_PHYS,
		.end	= MSM_SSBI0_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi0 = {
	.name		= "i2c_ssbi",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(msm_ssbi0_resources),
	.resource	= msm_ssbi0_resources,
};

#define MSM_SSBI1_PHYS  0x94090000
static struct resource msm_ssbi1_resources[] = {
	{
		.name   = "ssbi_base",
		.start  = MSM_SSBI1_PHYS,
		.end    = MSM_SSBI1_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi1 = {
	.name		= "i2c_ssbi",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(msm_ssbi1_resources),
	.resource	= msm_ssbi1_resources,
};
#endif /* CONFIG_I2C_SSBI */

#define MSM_NAND_PHYS		0x81600000
#define EBI2_REG_BASE		0x81400000
static struct resource resources_nand[] = {
	[0] = {
		.name   = "msm_nand_dmac",
		.start	= DMOV_NAND_CHAN,
		.end	= DMOV_NAND_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	[1] = {
		.name   = "msm_nand_phys",
		.start  = MSM_NAND_PHYS,
		.end    = MSM_NAND_PHYS + 0x7FF,
		.flags  = IORESOURCE_MEM,
	},
	[3] = {
		.name   = "ebi2_reg_base",
		.start  = EBI2_REG_BASE,
		.end    = EBI2_REG_BASE + 0x60,
		.flags  = IORESOURCE_MEM,
	},
};

struct flash_platform_data msm_nand_data = {
	.parts		= NULL,
	.nr_parts	= 0,
	.interleave     = 0,
};

struct platform_device msm_device_nand = {
	.name		= "msm_nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_nand),
	.resource	= resources_nand,
	.dev		= {
		.platform_data	= &msm_nand_data,
	},
};

struct platform_device msm_device_smd = {
	.name	= "msm_smd",
	.id	= -1,
};

struct platform_device msm_device_dmov = {
	.name	= "msm_dmov",
	.id	= -1,
};

#ifdef NOTNOW

#define MSM_SDC1_BASE         0x80A00000
static struct resource resources_sdc1[] = {
	{
		.start	= MSM_SDC1_BASE,
		.end	= MSM_SDC1_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC1_0,
		.end	= INT_SDC1_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_sdc1 = {
	.name		= "msm_sdcc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(resources_sdc1),
	.resource	= resources_sdc1,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};


static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_device_sdc1,
};

int __init msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (controller != 1)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}

#endif /* NOTNOW */

#define RAMFS_INFO_MAGICNUMBER		0x654D4D43
#define RAMFS_INFO_VERSION		0x00000001
#define RAMFS_MODEMSTORAGE_ID		0x4D454653

static struct resource rmt_storage_resources[] = {
       {
		.flags  = IORESOURCE_MEM,
       },
};

static struct platform_device rmt_storage_device = {
       .name           = "rmt_storage",
       .id             = -1,
       .num_resources  = ARRAY_SIZE(rmt_storage_resources),
       .resource       = rmt_storage_resources,
};

struct shared_ramfs_entry {
	uint32_t client_id;	/* Client id to uniquely identify a client */
	uint32_t base_addr;	/* Base address of shared RAMFS memory */
	uint32_t size;		/* Size of the shared RAMFS memory */
	uint32_t reserved;	/* Reserved attribute for future use */
};
struct shared_ramfs_table {
	uint32_t magic_id;	/* Identify RAMFS details in SMEM */
	uint32_t version;	/* Version of shared_ramfs_table */
	uint32_t entries;	/* Total number of valid entries   */
	struct shared_ramfs_entry ramfs_entry[3];	/* List all entries */
};

int __init rmt_storage_add_ramfs(void)
{
	struct shared_ramfs_table *ramfs_table;
	struct shared_ramfs_entry *ramfs_entry;
	int index;

	ramfs_table = smem_alloc(SMEM_SEFS_INFO,
			sizeof(struct shared_ramfs_table));

	if (!ramfs_table) {
		printk(KERN_WARNING "%s: No RAMFS table in SMEM\n", __func__);
		return -ENOENT;
	}

	if ((ramfs_table->magic_id != (u32) RAMFS_INFO_MAGICNUMBER) ||
		(ramfs_table->version != (u32) RAMFS_INFO_VERSION)) {
		printk(KERN_WARNING "%s: Magic / Version mismatch:, "
		       "magic_id=%#x, format_version=%#x\n", __func__,
		       ramfs_table->magic_id, ramfs_table->version);
		return -ENOENT;
	}

	for (index = 0; index < ramfs_table->entries; index++) {
		ramfs_entry = &ramfs_table->ramfs_entry[index];

		/* Find a match for the Modem Storage RAMFS area */
		if (ramfs_entry->client_id == (u32) RAMFS_MODEMSTORAGE_ID) {
			printk(KERN_INFO "%s: RAMFS Info (from SMEM): "
				"Baseaddr = 0x%08x, Size = 0x%08x\n", __func__,
				ramfs_entry->base_addr, ramfs_entry->size);

			rmt_storage_resources[0].start = ramfs_entry->base_addr;
			rmt_storage_resources[0].end = ramfs_entry->base_addr +
							ramfs_entry->size - 1;
			platform_device_register(&rmt_storage_device);
			return 0;
		}
	}
	return -ENOENT;
}


#ifdef NOTNOW

static void __init msm_register_device(struct platform_device *pdev, void *data)
{
	int ret;

	pdev->dev.platform_data = data;

	ret = platform_device_register(pdev);
	if (ret)
		dev_err(&pdev->dev,
			  "%s: platform_device_register() failed = %d\n",
			  __func__, ret);
}

#endif

#ifndef NOCLOCK
struct clk msm_clocks_fsm9xxx[] = {
	CLK_FSM9XXX("adm_clk",	ADM_CLK,	NULL, 0),
#ifdef NOTNOW
	CLK_FSM9XXX("i2c_clk",	I2C_CLK,	&msm_device_i2c.dev, 0),
#endif
	CLK_FSM9XXX("imem_clk",	IMEM_CLK,	NULL, OFF),
	CLK_FSM9XXX("uart_clk",	UART1_CLK,	&msm_device_uart1.dev, OFF),
#ifdef NOTNOW
	CLK_FSM9XXX("uart_clk",	UART2_CLK,	&msm_device_uart2.dev, 0),
#endif
};

unsigned msm_num_clocks_fsm9xxx = ARRAY_SIZE(msm_clocks_fsm9xxx);
#endif

#ifdef CONFIG_GPIOLIB

#define FSM9XXX_GPIO_PLATFORM_DATA(ix, begin, end, irq)			\
	[ix] = {							\
		.gpio_base	= begin,				\
		.ngpio		= end - begin + 1,			\
		.irq_base	= MSM_GPIO_TO_INT(begin),		\
		.irq_summary	= irq,					\
		.regs = {						\
			.in		= GPIO_IN_ ## ix,		\
			.out		= GPIO_OUT_ ## ix,		\
			.oe		= GPIO_OE_ ## ix,		\
		},							\
	}


#define FSM9XXX_GPIO_DEVICE(ix, pdata)			\
	{						\
		.name		= "fsm9xxx-gpio",	\
		.id		= ix,			\
		.num_resources	= 0,			\
		.dev = {				\
			.platform_data = &pdata[ix],	\
		},					\
	}

static struct fsm9xxx_gpio_platform_data gpio_platform_data[] = {
	FSM9XXX_GPIO_PLATFORM_DATA(0,   0,  31, INT_SIRC_0),
	FSM9XXX_GPIO_PLATFORM_DATA(1,  32,  63, INT_SIRC_0),
	FSM9XXX_GPIO_PLATFORM_DATA(2,  64,  95, INT_SIRC_0),
	FSM9XXX_GPIO_PLATFORM_DATA(3,  96, 127, INT_SIRC_0),
	FSM9XXX_GPIO_PLATFORM_DATA(4, 128, 159, INT_SIRC_0),
	FSM9XXX_GPIO_PLATFORM_DATA(5, 160, 167, INT_SIRC_0),
};

struct platform_device msm_gpio_devices[] = {
	FSM9XXX_GPIO_DEVICE(0, gpio_platform_data),
	FSM9XXX_GPIO_DEVICE(1, gpio_platform_data),
	FSM9XXX_GPIO_DEVICE(2, gpio_platform_data),
	FSM9XXX_GPIO_DEVICE(3, gpio_platform_data),
	FSM9XXX_GPIO_DEVICE(4, gpio_platform_data),
	FSM9XXX_GPIO_DEVICE(5, gpio_platform_data),
};
#endif
