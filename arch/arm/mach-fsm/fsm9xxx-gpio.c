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
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include "tlmm-fsm9xxx.h"

enum {
	IRQ_MASK_NORMAL = 0,
	IRQ_MASK_WAKE_ON,
	NUM_IRQ_MASKS
};

struct msm_gpio_dev {
	struct gpio_chip		gpio_chip;
	spinlock_t			lock;
	unsigned			irq_base;
	unsigned			irq_summary;
	struct fsm9xxx_gpio_regs	regs;
	u32				irq_masks[NUM_IRQ_MASKS];
	int				nsuspend;
};

#define TO_MSM_GPIO_DEV(c) container_of(c, struct msm_gpio_dev, gpio_chip)

static inline unsigned bit(unsigned offset)
{
	BUG_ON(offset >= sizeof(unsigned) * 8);
	return 1U << offset;
}

/*
 * This function assumes that msm_gpio_dev::lock is held.
 */
static inline void set_gpio_bit(unsigned n, void __iomem *reg)
{
	writel(readl(reg) | bit(n), reg);
}

/*
 * This function assumes that msm_gpio_dev::lock is held.
 */
static inline void clr_gpio_bit(unsigned n, void __iomem *reg)
{
	writel(readl(reg) & ~bit(n), reg);
}

/*
 * This function assumes that msm_gpio_dev::lock is held.
 */
static inline void
msm_gpio_write(struct msm_gpio_dev *dev, unsigned n, unsigned on)
{
	if (on)
		set_gpio_bit(n, dev->regs.out);
	else
		clr_gpio_bit(n, dev->regs.out);
}

static int gpio_chip_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct msm_gpio_dev *msm_gpio = TO_MSM_GPIO_DEV(chip);
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	clr_gpio_bit(offset, msm_gpio->regs.oe);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return 0;
}

static int
gpio_chip_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct msm_gpio_dev *msm_gpio = TO_MSM_GPIO_DEV(chip);
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);

	msm_gpio_write(msm_gpio, offset, value);
	set_gpio_bit(offset, msm_gpio->regs.oe);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return 0;
}

static int gpio_chip_get(struct gpio_chip *chip, unsigned offset)
{
	struct msm_gpio_dev *msm_gpio = TO_MSM_GPIO_DEV(chip);
	unsigned long irq_flags;
	int ret;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	ret = readl(msm_gpio->regs.in) & bit(offset) ? 1 : 0;
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return ret;
}

static void gpio_chip_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct msm_gpio_dev *msm_gpio = TO_MSM_GPIO_DEV(chip);
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	msm_gpio_write(msm_gpio, offset, value);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);
}

static int msm_gpio_probe(struct platform_device *dev)
{
	struct msm_gpio_dev *msm_gpio;
	struct fsm9xxx_gpio_platform_data *pdata =
		(struct fsm9xxx_gpio_platform_data *)dev->dev.platform_data;
	int ret;

	if (!pdata)
		return -EINVAL;

	msm_gpio = kzalloc(sizeof(struct msm_gpio_dev), GFP_KERNEL);
	if (!msm_gpio)
		return -ENOMEM;

	spin_lock_init(&msm_gpio->lock);
	platform_set_drvdata(dev, msm_gpio);
	memcpy(&msm_gpio->regs,
	       &pdata->regs,
	       sizeof(struct fsm9xxx_gpio_regs));

	msm_gpio->gpio_chip.label            = dev->name;
	msm_gpio->gpio_chip.base             = pdata->gpio_base;
	msm_gpio->gpio_chip.ngpio            = pdata->ngpio;
	msm_gpio->gpio_chip.direction_input  = gpio_chip_direction_input;
	msm_gpio->gpio_chip.direction_output = gpio_chip_direction_output;
	msm_gpio->gpio_chip.get              = gpio_chip_get;
	msm_gpio->gpio_chip.set              = gpio_chip_set;

	ret = gpiochip_add(&msm_gpio->gpio_chip);
	if (ret < 0)
		goto err_post_malloc;

	return ret;

err_post_malloc:
	kfree(msm_gpio);
	return ret;
}

static int msm_gpio_remove(struct platform_device *dev)
{
	struct msm_gpio_dev *msm_gpio = platform_get_drvdata(dev);
	int ret;

	ret = gpiochip_remove(&msm_gpio->gpio_chip);
	if (ret < 0)
		return ret;
	kfree(msm_gpio);

	return 0;
}

static int msm_gpio_suspend(struct msm_gpio_dev *msm_gpio)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	/* TODO call SIRC */
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return 0;
}

static int msm_gpio_resume(struct msm_gpio_dev *msm_gpio)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	/* TODO call SIRC */
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return 0;
}

static int msm_gpio_legacy_suspend(struct platform_device *dev,
				   pm_message_t state)
{
	struct msm_gpio_dev *msm_gpio = platform_get_drvdata(dev);

	return msm_gpio_suspend(msm_gpio);
}

static int msm_gpio_legacy_resume(struct platform_device *dev)
{
	struct msm_gpio_dev *msm_gpio = platform_get_drvdata(dev);

	return msm_gpio_resume(msm_gpio);
}

static int msm_gpio_dev_pm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_gpio_dev *msm_gpio = platform_get_drvdata(pdev);

	return msm_gpio_suspend(msm_gpio);
}

static int msm_gpio_dev_pm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_gpio_dev *msm_gpio = platform_get_drvdata(pdev);

	return msm_gpio_resume(msm_gpio);
}

static SIMPLE_DEV_PM_OPS(msm_gpio_pm_ops,
			 msm_gpio_dev_pm_suspend,
			 msm_gpio_dev_pm_resume);

static struct platform_driver msm_gpio_driver = {
	.probe   = msm_gpio_probe,
	.remove  = msm_gpio_remove,
	.suspend = msm_gpio_legacy_suspend,
	.resume  = msm_gpio_legacy_resume,
	.driver  = {
		.name  = "fsm9xxx-gpio",
		.owner = THIS_MODULE,
		.pm    = &msm_gpio_pm_ops,
	},
};

static int __init msm_gpio_init(void)
{
	return platform_driver_register(&msm_gpio_driver);
}

static void __exit msm_gpio_exit(void)
{
	platform_driver_unregister(&msm_gpio_driver);
}

postcore_initcall(msm_gpio_init);
module_exit(msm_gpio_exit);
MODULE_DESCRIPTION("FSM GPIO Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rick Adams <rgadams@codeaurora.org>");
MODULE_VERSION("1.0");
