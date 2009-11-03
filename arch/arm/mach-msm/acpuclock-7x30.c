/*
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, Code Aurora Forum. All rights reserved.
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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/sort.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <asm/mach-types.h>

#include "proc_comm.h"
#include "smd_private.h"
#include "clock.h"
#include "acpuclock.h"

#define SCSS_CLK_CTL_ADDR	(MSM_ACC_BASE + 0x04)
#define SCSS_CLK_SEL_ADDR	(MSM_ACC_BASE + 0x08)

#define dprintk(msg...) \
	cpufreq_debug_printk(CPUFREQ_DEBUG_DRIVER, "cpufreq-msm", msg)

struct clock_state {
	struct clkctl_acpu_speed	*current_speed;
	struct mutex			lock;
	uint32_t			acpu_switch_time_us;
};

struct clkctl_acpu_speed {
	unsigned int	acpu_clk_khz;
	int		src;
	unsigned int	acpu_src_sel;
	unsigned int	acpu_src_div;
	unsigned int 	axi_clk_khz;
	unsigned int	vdd_mv;
	unsigned long	lpj; /* loops_per_jiffy */
};

static struct clock_state drv_state = { 0 };

#ifdef CONFIG_CPU_FREQ_MSM
static struct cpufreq_frequency_table freq_table[] = {
	{ 0, 122880 },
	{ 1, 245760 },
	{ 2, 368640 },
	{ 3, 768000 },
	{ 4, 806400 },
	{ 5, CPUFREQ_TABLE_END },
};
#endif

/* Use negative numbers for sources that can't be enabled/disabled */
#define SRC_LPXO (-2)
#define SRC_AXI  (-1)
static struct clkctl_acpu_speed acpu_freq_tbl[] = {
	{ 24576,  SRC_LPXO, 0, 0,  30720,  1200 },
	{ 61440,  PLL_3,    5, 11, 61440,  1200 },
	{ 122880, PLL_3,    5, 5,  61440,  1200 },
	{ 128000, SRC_AXI,  1, 0,  128000, 1200 },
	{ 184320, PLL_3,    5, 4,  61440,  1200 },
	{ 245760, PLL_3,    5, 2,  61440,  1200 },
	{ 368640, PLL_3,    5, 1,  128000, 1200 },
	{ 768000, PLL_1,    2, 0,  128000, 1200 },
	{ 806400, PLL_2,    3, 0,  128000, 1200 },
	{ 0, 0, 0, 0, 0, 0, 0 }
};

#define POWER_COLLAPSE_HZ 128000000
unsigned long acpuclk_power_collapse(void)
{
	int ret = acpuclk_get_rate();
	acpuclk_set_rate(POWER_COLLAPSE_HZ, SETRATE_PC);
	return ret * 1000;
}

#define WAIT_FOR_IRQ_HZ 128000000
unsigned long acpuclk_wait_for_irq(void)
{
	int ret = acpuclk_get_rate();
	acpuclk_set_rate(WAIT_FOR_IRQ_HZ, SETRATE_SWFI);
	return ret * 1000;
}

static int pc_pll_request(unsigned id, unsigned on)
{
	int rc = 0;
	on = !!on;

	if (on)
		dprintk("Enabling PLL %d\n", id);
	else
		dprintk("Disabling PLL %d\n", id);

	if (id >= NUM_PLL)
		return -EINVAL;

	rc = msm_proc_comm(PCOM_CLKCTL_RPC_PLL_REQUEST, &id, &on);
	if (rc < 0)
		return rc;
	else if ((int) id < 0)
		return -EINVAL;

	if (on)
		dprintk("PLL enabled\n");
	else
		dprintk("PLL disabled\n");

	return rc;
}

static int acpuclk_set_acpu_vdd(struct clkctl_acpu_speed *s)
{
	/* TODO: Add local VDD control.
	 * Assuming VDD is set to max by bootloader. */

	return 0;
}

/* Set clock source and divider given a clock speed */
static void acpuclk_set_src(const struct clkctl_acpu_speed *s)
{
	uint32_t reg_clksel, reg_clkctl, src_sel;

	reg_clksel = readl(SCSS_CLK_SEL_ADDR);

	/* CLK_SEL_SRC1NO */
	src_sel = reg_clksel & 1;

	/* Program clock source and divider. */
	reg_clkctl = readl(SCSS_CLK_CTL_ADDR);
	reg_clkctl &= ~(0xFF << (8 * src_sel));
	reg_clkctl |= s->acpu_src_sel << (4 + 8 * src_sel);
	reg_clkctl |= s->acpu_src_div << (0 + 8 * src_sel);
	writel(reg_clkctl, SCSS_CLK_CTL_ADDR);

	/* Toggle clock source. */
	reg_clksel ^= 1;

	/* Program clock source selection. */
	writel(reg_clksel, SCSS_CLK_SEL_ADDR);
}

int acpuclk_set_rate(unsigned long rate, enum setrate_reason reason)
{
	struct clkctl_acpu_speed *tgt_s, *strt_s;
	int res, rc = 0;

	if (reason == SETRATE_CPUFREQ)
		mutex_lock(&drv_state.lock);

	strt_s = drv_state.current_speed;

	if (rate == (strt_s->acpu_clk_khz * 1000))
		goto out;

	for (tgt_s = acpu_freq_tbl; tgt_s->acpu_clk_khz != 0; tgt_s++) {
		if (tgt_s->acpu_clk_khz == (rate / 1000))
			break;
	}
	if (tgt_s->acpu_clk_khz == 0) {
		rc = -EINVAL;
		goto out;
	}

	if (reason == SETRATE_CPUFREQ) {
		/* Increase VDD if needed. */
		if (tgt_s->vdd_mv > strt_s->vdd_mv) {
			rc = acpuclk_set_acpu_vdd(tgt_s);
			if (rc < 0) {
				pr_err("ACPU VDD increase to %d mV failed "
					"(%d)\n", tgt_s->vdd_mv, rc);
				goto out;
			}
		}
	}

	dprintk("Switching from ACPU rate %u KHz -> %u KHz\n",
	       strt_s->acpu_clk_khz, tgt_s->acpu_clk_khz);

	/* Turn on target PLL if needed. */
	if (strt_s->src != tgt_s->src && tgt_s->src >= 0) {
		rc = pc_pll_request(tgt_s->src, 1);
		if (rc < 0) {
			pr_err("PLL%d enable failed (%d)\n", tgt_s->src, rc);
			goto out;
		}
	}

	/* Perform the frequency switch */
	acpuclk_set_src(tgt_s);
	drv_state.current_speed = tgt_s;
	loops_per_jiffy = tgt_s->lpj;

	/* Nothing else to do for SWFI. */
	if (reason == SETRATE_SWFI)
		goto out;

	/* Change the AXI bus frequency if we can. */
	if (strt_s->axi_clk_khz != tgt_s->axi_clk_khz) {
		res = ebi1_clk_set_min_rate(CLKVOTE_ACPUCLK,
						tgt_s->axi_clk_khz * 1000);
		if (res < 0)
			pr_warning("Setting AXI min rate failed (%d)\n", res);
	}

	/* Turn off previous PLL if not needed. */
	if (strt_s->src != tgt_s->src && strt_s->src >= 0) {
		res = pc_pll_request(strt_s->src, 0);
		if (res < 0) {
			pr_warning("PLL%d disable failed (%d)\n",
					strt_s->src, res);
		}
	}

	/* Nothing else to do for power collapse. */
	if (reason == SETRATE_PC)
		goto out;

	/* Drop VDD level if we can. */
	if (tgt_s->vdd_mv < strt_s->vdd_mv) {
		res = acpuclk_set_acpu_vdd(tgt_s);
		if (res < 0) {
			pr_warning("ACPU VDD decrease to %d mV failed (%d)\n",
					tgt_s->vdd_mv, res);
		}
	}

	dprintk("ACPU speed change complete\n");
out:
	if (reason == SETRATE_CPUFREQ)
		mutex_unlock(&drv_state.lock);

	return rc;
}

unsigned long acpuclk_get_rate(void)
{
	WARN_ONCE(drv_state.current_speed == NULL,
		  "acpuclk_get_rate: not initialized\n");
	if (drv_state.current_speed)
		return drv_state.current_speed->acpu_clk_khz;
	else
		return 0;
}

uint32_t acpuclk_get_switch_time(void)
{
	return drv_state.acpu_switch_time_us;
}

#define MAX_AXI_KHZ 128000
unsigned long clk_get_max_axi_khz(void)
{
	return MAX_AXI_KHZ;
}
EXPORT_SYMBOL(clk_get_max_axi_khz);


/*----------------------------------------------------------------------------
 * Clock driver initialization
 *---------------------------------------------------------------------------*/

static void __init acpuclk_init(void)
{
	struct clkctl_acpu_speed *s;
	uint32_t div, sel, src_num;
	uint32_t reg_clksel, reg_clkctl;
	int res;

	reg_clksel = readl(SCSS_CLK_SEL_ADDR);

	/* Determine the ACPU clock rate. */
	switch ((reg_clksel >> 1) & 0x3) {
	case 0:	/* Running off the output of the raw clock source mux. */
		reg_clkctl = readl(SCSS_CLK_CTL_ADDR);
		src_num = reg_clksel & 0x1;
		sel = (reg_clkctl >> (12 - (8 * src_num))) & 0x7;
		div = (reg_clkctl >> (8 -  (8 * src_num))) & 0xF;

		/* Check frequency table for matching sel/div pair. */
		for (s = acpu_freq_tbl; s->acpu_clk_khz != 0; s++) {
			if (s->acpu_src_sel == sel && s->acpu_src_div == div)
				break;
		}
		if (s->acpu_clk_khz == 0) {
			pr_err("Error - ACPU clock reports invalid speed\n");
			return;
		}
		break;
	case 2:	/* Running off of the SCPLL selected through the core mux. */
		/* Switch to run off of the SCPLL selected through the raw
		 * clock source mux. */
		for (s = acpu_freq_tbl; s->acpu_clk_khz != 0
			&& s->src != PLL_2 && s->acpu_src_div == 0; s++)
			;
		if (s->acpu_clk_khz != 0) {
			/* Program raw clock source mux. */
			acpuclk_set_src(s);

			/* Switch to raw clock source input of the core mux. */
			reg_clksel = readl(SCSS_CLK_SEL_ADDR);
			reg_clksel &= ~(0x3 << 1);
			writel(reg_clksel, SCSS_CLK_SEL_ADDR);
			break;
		}
		/* else fall through */
	default:
		pr_err("Error - ACPU clock reports invalid source\n");
		return;
	}

	drv_state.current_speed = s;

	res = ebi1_clk_set_min_rate(CLKVOTE_ACPUCLK, s->axi_clk_khz * 1000);
	if (res < 0)
		pr_warning("Setting AXI min rate failed!\n");

	pr_info("ACPU running at %d KHz\n", s->acpu_clk_khz);

	return;
}

/* Initalize the lpj field in the acpu_freq_tbl. */
static void __init lpj_init(void)
{
	int i;
	const struct clkctl_acpu_speed *base_clk = drv_state.current_speed;

	for (i = 0; acpu_freq_tbl[i].acpu_clk_khz; i++) {
		acpu_freq_tbl[i].lpj = cpufreq_scale(loops_per_jiffy,
						base_clk->acpu_clk_khz,
						acpu_freq_tbl[i].acpu_clk_khz);
	}
}

void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *clkdata)
{
	pr_info("acpu_clock_init()\n");

	mutex_init(&drv_state.lock);
	drv_state.acpu_switch_time_us = clkdata->acpu_switch_time_us;
	acpuclk_init();
	lpj_init();
#ifdef CONFIG_CPU_FREQ_MSM
	cpufreq_frequency_table_get_attr(freq_table, smp_processor_id());
#endif
}
