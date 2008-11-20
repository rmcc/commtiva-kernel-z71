/* drivers/video/msm_fb/mddi_client_toshiba.c
 *
 * Support for Toshiba TC358720XBG mddi client devices which require no
 * special initialization code.
 *
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <mach/msm_fb.h>

static DECLARE_WAIT_QUEUE_HEAD(toshiba_vsync_wait);
static volatile int toshiba_got_int;

#define LCD_CONTROL_BLOCK_BASE 0x110000
#define CMN         (LCD_CONTROL_BLOCK_BASE|0x10)
#define INTFLG      (LCD_CONTROL_BLOCK_BASE|0x18)
#define HCYCLE      (LCD_CONTROL_BLOCK_BASE|0x34)
#define HDE_START   (LCD_CONTROL_BLOCK_BASE|0x3C)
#define VPOS        (LCD_CONTROL_BLOCK_BASE|0xC0)
#define MPLFBUF     (LCD_CONTROL_BLOCK_BASE|0x20)
#define WAKEUP      (LCD_CONTROL_BLOCK_BASE|0x54)
#define WSYN_DLY    (LCD_CONTROL_BLOCK_BASE|0x58)
#define REGENB      (LCD_CONTROL_BLOCK_BASE|0x5C)

#define BASE5 0x150000
#define BASE6 0x160000
#define BASE7 0x170000

#define GPIOIEV     (BASE5 + 0x10)
#define GPIOIE      (BASE5 + 0x14)
#define GPIORIS     (BASE5 + 0x18)
#define GPIOMIS     (BASE5 + 0x1C)
#define GPIOIC      (BASE5 + 0x20)

#define INTMASK     (BASE6 + 0x0C)
#define INTMASK_VWAKEOUT (1U << 0)
#define INTMASK_VWAKEOUT_ACTIVE_LOW (1U << 8)
#define GPIOSEL     (BASE7 + 0x00)
#define GPIOSEL_VWAKEINT (1U << 0)

#define get_panel_info(data) \
	container_of(data, struct msm_mddi_panel_info, panel_data)

#define get_client_data(data) \
	(get_panel_info(data)->client_data)

#define get_toshiba_client_data(data) \
	(get_client_data(data)->private_client_data)

static void toshiba_request_vsync(struct msm_panel_data *panel_data,
				  struct msmfb_callback *callback)
{
	struct msm_mddi_panel_info *panel = get_panel_info(panel_data);
	struct msm_mddi_client_data *cdata = get_client_data(panel_data);

	panel->toshiba_callback = callback;
	if (toshiba_got_int) {
		toshiba_got_int = 0;
		cdata->activate_link(cdata);
	}
}

static void toshiba_wait_vsync(struct msm_panel_data *panel_data)
{
	struct msm_mddi_client_data *cdata = get_client_data(panel_data);

	if (toshiba_got_int) {
		toshiba_got_int = 0;
		cdata->activate_link(cdata); /* clears interrupt */
	}
	if (wait_event_timeout(toshiba_vsync_wait, toshiba_got_int, HZ/2) == 0)
		printk(KERN_ERR "timeout waiting for VSYNC\n");
	toshiba_got_int = 0;
	/* interrupt clears when screen dma starts */
}

static int toshiba_suspend(struct msm_panel_data *panel_data)
{
	struct msm_mddi_client_data *cdata = get_client_data(panel_data);
	struct msm_mddi_toshiba_client_data *toshiba_data =
		get_toshiba_client_data(panel_data);

	if (toshiba_data->uninit(cdata))
		return -1;
	cdata->suspend(cdata);
	return 0;
}

static int toshiba_resume(struct msm_panel_data *panel_data)
{
	struct msm_mddi_client_data *cdata = get_client_data(panel_data);
	struct msm_mddi_toshiba_client_data *toshiba_data =
		get_toshiba_client_data(panel_data);

	cdata->resume(cdata);
	if (toshiba_data->init(cdata))
		return -1;
	return 0;
}

irqreturn_t toshiba_vsync_interrupt(int irq, void *data)
{
	struct msm_mddi_panel_info *panel = data;

	toshiba_got_int = 1;
	if (panel->toshiba_callback) {
		panel->toshiba_callback->func(panel->toshiba_callback);
		panel->toshiba_callback = 0;
	}
	wake_up(&toshiba_vsync_wait);
	return IRQ_HANDLED;
}

static int setup_vsync(struct msm_mddi_panel_info *panel,
				    int init)
{
	int ret;
	int gpio = 97;
	unsigned int irq;

	if (!init) {
		ret = 0;
		goto uninit;
	}
	ret = gpio_request(gpio, "vsync");
	if (ret)
		goto err_request_gpio_failed;

	ret = gpio_direction_input(gpio);
	if (ret)
		goto err_gpio_direction_input_failed;

	ret = irq = gpio_to_irq(gpio);
	if (ret < 0)
		goto err_get_irq_num_failed;

	ret = request_irq(irq, toshiba_vsync_interrupt, IRQF_TRIGGER_RISING,
			  "vsync", panel);
	if (ret)
		goto err_request_irq_failed;
	printk(KERN_INFO "vsync on gpio %d now %d\n",
	       gpio, gpio_get_value(gpio));
	return 0;

uninit:
	free_irq(gpio_to_irq(gpio), panel->client_data);
err_request_irq_failed:
err_get_irq_num_failed:
err_gpio_direction_input_failed:
	gpio_free(gpio);
err_request_gpio_failed:
	return ret;
}

static int mddi_toshiba_probe(struct platform_device *pdev)
{
	int ret;
	struct msm_mddi_client_data *pdata = pdev->dev.platform_data;
	struct msm_mddi_toshiba_client_data *toshiba_data =
		pdata->private_client_data;
	struct msm_mddi_panel_info *panel =
		kzalloc(sizeof(struct msm_mddi_panel_info), GFP_KERNEL);
	struct platform_device *panel_pdev;

	/* mddi_remote_write(mddi, 0, WAKEUP); */
	pdata->remote_write(pdata, GPIOSEL_VWAKEINT, GPIOSEL);
	pdata->remote_write(pdata, INTMASK_VWAKEOUT, INTMASK);

	ret = setup_vsync(panel, 1);
	if (ret) {
		dev_err(&pdev->dev, "mddi_toshiba_setup_vsync failed\n");
		return ret;
	}

	panel->client_data = pdata;
	panel->client_data->panel = panel;
	panel->panel_data.suspend = toshiba_suspend;
	panel->panel_data.resume = toshiba_resume;
	panel->panel_data.wait_vsync = toshiba_wait_vsync;
	panel->panel_data.request_vsync = toshiba_request_vsync;
	panel->panel_data.blank = toshiba_data->blank;
	panel->panel_data.unblank = toshiba_data->unblank;
	panel->panel_data.fb_data =  &toshiba_data->fb_data;

	panel_pdev = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	if (!panel_pdev) {
		printk(KERN_ERR "mddi_toshiba_panel: could not allocate"
			"device for mddi_toshiba_panel\n");
		return -ENOMEM;
	}
	panel_pdev->name = "msm_panel";
	panel_pdev->id = pdev->id;
	panel_pdev->resource = pdata->fb_resource;
	panel_pdev->num_resources = 1;
	panel_pdev->dev.platform_data = &panel->panel_data;
	toshiba_data->init(panel->client_data);
	platform_device_register(panel_pdev);

	return 0;
}

static int mddi_toshiba_remove(struct platform_device *pdev)
{
	struct msm_mddi_client_data *pdata = pdev->dev.platform_data;
	setup_vsync(pdata->panel, 0);
	kfree(pdata->panel);
	return 0;
}

static struct platform_driver mddi_client_d263_0000 = {
	.probe = mddi_toshiba_probe,
	.remove = mddi_toshiba_remove,
	.driver = { .name = "mddi_c_d263_0000" },
};

static int __init mddi_client_toshiba_init(void)
{
	platform_driver_register(&mddi_client_d263_0000);
	return 0;
}

module_init(mddi_client_toshiba_init);

