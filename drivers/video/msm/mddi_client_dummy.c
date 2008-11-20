/* drivers/video/msm_fb/mddi_client_dummy.c
 *
 * Support for "dummy" mddi client devices which require no
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

#include <mach/msm_fb.h>


static int mddi_dummy_suspend(struct msm_panel_data *panel_data)
{
	return 0;
}

static int mddi_dummy_resume(struct msm_panel_data *panel_data)
{
	return 0;
}

static int mddi_dummy_probe(struct platform_device *pdev)
{
	struct msm_mddi_panel_info *panel =
		kzalloc(sizeof(struct msm_mddi_panel_info), GFP_KERNEL);
	struct platform_device *panel_pdev;

	panel->client_data = pdev->dev.platform_data;
	panel->client_data->panel = panel;
	panel->panel_data.suspend = mddi_dummy_suspend;
	panel->panel_data.resume = mddi_dummy_resume;
	panel_pdev = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	panel_pdev->name = "msm_panel";
	panel_pdev->id = pdev->id;
	platform_device_add_resources(panel_pdev,
				      panel->client_data->fb_resource, 1);
	panel_pdev->dev.platform_data = &panel->panel_data;
	platform_device_register(panel_pdev);
	return 0;
}

static int mddi_dummy_remove(struct platform_device *pdev)
{
	struct msm_mddi_client_data *pdata = pdev->dev.platform_data;
	kfree(pdata->panel);
	return 0;
}

static struct platform_driver mddi_client_dummy = {
	.probe = mddi_dummy_probe,
	.remove = mddi_dummy_remove,
	.driver = { .name = "mddi_c_dummy" },
};

static int __init mddi_client_dummy_init(void)
{
	platform_driver_register(&mddi_client_dummy);
	return 0;
}

module_init(mddi_client_dummy_init);

