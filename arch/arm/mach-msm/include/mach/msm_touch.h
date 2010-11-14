/* arch/arm/mach-msm/include/mach/msm_touch.h
 *
 * Platform data for MSM touchscreen driver.
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#ifndef _MACH_MSM_TOUCH_H_
#define _MACH_MSM_TOUCH_H_

struct msm_ts_platform_data {
	unsigned int x_max;
	unsigned int y_max;
	unsigned int pressure_max;
};

//Added for touch calibration++
struct msm_data_from_ap {
	unsigned int short x_max;
	unsigned int short x_min;
	unsigned int short y_max;
	unsigned int short y_min;
	//bool bIsFactoryTest; 
};

#define MSM_IOC_MAGIC    'M'
#define MSM_IOC_BOUNDARY    _IOW(MSM_IOC_MAGIC, 0, int)  /* Send boundary value */
#define MSM_IOC_ISFACTORYTEST    _IO(MSM_IOC_MAGIC, 1)  /* Pass factory test flag */
#define MSM_IOC_MAXNR    2
//Added for touch calibration--

#endif
