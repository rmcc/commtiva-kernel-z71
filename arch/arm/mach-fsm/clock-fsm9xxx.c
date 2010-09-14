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
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include <mach/msm_iomap.h>
#include <mach/clk.h>
#include <mach/internal_power_rail.h>

#include "clock.h"
#include "clock-fsm9xxx.h"

enum {
	NOMINAL,
	HIGH,
	MSMC1_END
};

struct clk_freq_tbl {
	uint32_t	freq_hz;
	uint32_t	src;
	uint32_t	md_val;
	uint32_t	ns_val;
	uint32_t	mode;
	unsigned	msmc1;
};

struct clk_local {
	uint32_t	count;
	uint32_t	type;
	uint32_t	md_reg;
	uint32_t	ns_reg;
	uint32_t	freq_mask;
	uint32_t	br_en_mask;
	uint32_t	root_en_mask;
	int		parent;
	uint32_t	*children;
	struct clk_freq_tbl	*freq_tbl;
	struct clk_freq_tbl	*current_freq;
	uint32_t	halt_reg;
	uint32_t	halt_mask;
};


enum {
	SRC_TCXO = 0, /* 19.2 MHz */
	SRC_PLL_GLOBAL = 1, /* 320 MHz */
	SRC_PLL_ARM = 2, /* 768 MHz */
	SRC_PLL_QDSP6 = 3, /* 595.2 MHz */
	SRC_PLL_DDR = 4, /* 266 - 332 MHz */
	SRC_EXT_CLK_1 = 5, /* 39.2 MHz */
	SRC_EXT_CLK_2 = 6,
	SRC_CORE_BI_PLL_TEST_SE = 7, /* 400 MHz */

	SRC_PLL_ETH = 0x14, /* Specific to ETH_NS_REG */

	SRC_AXI  = 100, /* Used for rates that sync to AXI */
	SRC_MAX,        /* Used for sources that can't be turned on/off. */
};

static uint32_t src_pll_tbl[SRC_MAX] = {
	[SRC_PLL_GLOBAL] = PLL_0,
	[SRC_PLL_ARM] = PLL_4,
	[SRC_PLL_QDSP6] = PLL_2,
	[SRC_PLL_DDR] = PLL_3,
	[SRC_PLL_ETH] = PLL_7,
};

enum {
	PRE_DIV_BYPASS,
	PRE_DIV_2,
	PRE_DIV_3,
	PRE_DIV_4,
};

#define B(x)	BIT(x)
#define BM(msb, lsb)	(((((uint32_t)-1) << (31-msb)) >> (31-msb+lsb)) << lsb)
#define BVAL(msb, lsb, val)	(((val) << lsb) & BM(msb, lsb))

#define MD8(m, n)		(BVAL(15, 8, m) | BVAL(7, 0, ~(n)))
#define N8(msb, lsb, m, n)	(BVAL(msb, lsb, ~(n-m)))
#define MD16(m, n)		(BVAL(31, 16, m) | BVAL(15, 0, ~(n)))
#define N16(m, n)		(BVAL(31, 16, ~(n-m)))
#define SPDIV(s, d)		(BVAL(4, 3, d-1) | BVAL(2, 0, s))
#define SDIV(s, d)		(BVAL(6, 3, d-1) | BVAL(2, 0, s))
#define F_MASK_BASIC		(BM(6, 3)|BM(2, 0))
#define F_MASK_MND16		(BM(31, 16)|BM(4, 3)|BM(2, 0))
#define F_MASK_MND8(m, l)	(BM(m, l)|BM(4, 3)|BM(2, 0))

#define F_RAW(f, s, m_v, n_v, mde, v) { \
	.freq_hz = f, \
	.src = s, \
	.md_val = m_v, \
	.ns_val = n_v, \
	.mode = mde, \
	.msmc1 = v \
	}

#define FREQ_END	0
#define F_BASIC(f, s, div, v) F_RAW(f, s, 0, SDIV(s, div), 0, v)
#define F_MND16(f, s, div, m, n, v) \
	F_RAW(f, s, MD16(m, n), N16(m, n)|SPDIV(s, div), !!(n), v)
#define F_MND8(f, nmsb, nlsb, s, div, m, n, v) \
	F_RAW(f, s, MD8(m, n), N8(nmsb, nlsb, m, n)|SPDIV(s, div), !!(n), v)
#define F_END	F_RAW(FREQ_END, SRC_MAX, 0, 0, 0, MSMC1_END)

static struct clk_freq_tbl clk_tbl_tcxo[] = {
	F_RAW(19200000, SRC_TCXO, 0, 0, 0, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_axi[] = {
	F_RAW(1, SRC_AXI, 0, 0, 0, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_uartdm[] = {
	/* 320 MHz * 1 / 5 = 64 MHz */
	F_MND16(64000000, SRC_PLL_GLOBAL, PRE_DIV_BYPASS, 1, 5, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_sdcc[] = {
	/* 768 MHz * 13 / 192 = 52 MHz */
	F_MND16(52000000, SRC_PLL_ARM, PRE_DIV_BYPASS, 13, 192, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_eth[] = {
	/* 250 MHz / 2 = 125 MHz */
	F_MND16(250000000, SRC_PLL_ETH, PRE_DIV_2, 1, 1, NOMINAL),
	F_END,
};

static struct clk_freq_tbl dummy_freq = F_END;

#define MND	1 /* Integer predivider and fractional MN:D divider. */
#define BASIC	2 /* Integer divider. */
#define NORATE	3 /* Just on/off. */

#define C(x) L_FSM9XXX_##x##_CLK

#define CLK_LOCAL(id, t, md, ns, f_msk, br, root, tbl, par, chld_lst, h, hm) \
	[C(id)] = { \
	.type = t, \
	.md_reg = md, \
	.ns_reg = ns, \
	.freq_mask = f_msk, \
	.br_en_mask = br, \
	.root_en_mask = root, \
	.parent = C(par), \
	.children = chld_lst, \
	.freq_tbl = tbl, \
	.current_freq = &dummy_freq, \
	.halt_reg = h, \
	.halt_mask = hm, \
	}

#define CLK_BASIC(id, ns, br, root, tbl, par, h, hm) \
		CLK_LOCAL(id, BASIC, 0, ns, F_MASK_BASIC, br, root, tbl, \
							par, NULL, h, hm)
#define CLK_MND8_P(id, ns, m, l, br, root, tbl, par, chld_lst, h, hm) \
		CLK_LOCAL(id, MND, (ns-4), ns, F_MASK_MND8(m, l), br, root, \
						tbl, par, chld_lst, h, hm)
#define CLK_MND8(id, ns, m, l, br, root, tbl, chld_lst, h, hm) \
		CLK_MND8_P(id, ns, m, l, br, root, tbl, NONE, chld_lst, h, hm)
#define CLK_MND16(id, ns, br, root, tbl, par, chld_lst, h, hm) \
		CLK_LOCAL(id, MND, (ns-4), ns, F_MASK_MND16, br, root, tbl, \
							par, chld_lst, h, hm)
#define CLK_1RATE(id, ns, br, root, tbl, h, hm) \
		CLK_LOCAL(id, BASIC, 0, ns, 0, br, root, tbl, NONE, NULL, h, hm)
#define CLK_SLAVE(id, ns, br, par, h, hm) \
		CLK_LOCAL(id, NORATE, 0, ns, 0, br, 0, NULL, par, NULL, h, hm)
#define CLK_NORATE(id, ns, br, root, h, hm) \
		CLK_LOCAL(id, NORATE, 0, ns, 0, br, root, NULL, NONE, NULL, \
				h, hm)
#define CLK_GLBL(id, glbl, br, h, hm) \
		CLK_LOCAL(id, NORATE, 0, glbl, 0, br, 0, NULL, GLBL_ROOT, \
				NULL, h, hm)
#define CLK_BRIDGE(id, glbl, br, par, h, hm) \
		CLK_LOCAL(id, NORATE, 0, glbl, 0, br, 0, NULL, par, NULL, \
				h, hm)
#define REG(off) (MSM_CLK_CTL_BASE + off)
#define MNCNTR_EN_MASK		B(8)
#define MNCNTR_RST_MASK		B(7)
#define MNCNTR_MODE_MASK	BM(6, 5)
#define MNCNTR_MODE		BVAL(6, 5, 0x2) /* Dual-edge mode. */

/* Register offsets used more than once. */
#define GLBL_CLK_ENA			0x0000
#define GLBL_CLK_ENA_2			0x0004
#define GLBL_CLK_ENA_3			0x0008
#define GLBL_CLK_STATE			0x0010 /* halt statue */
#define GLBL_CLK_STATE_2		0x0014 /* halt statue */
#define GLBL_CLK_STATE_3		0x0018 /* halt statue */
#define SDCC_NS					0x0060
#define UARTDM_NS				0x0068
#define UART1_NS				0x0078
#define UART2_NS				0x0077
#define UART3_NS				0x0080
#define I2C_NS					0x0084
#define SSBI_NS					0x008c
#define ETH_NS					0x02a8

/*
 * PLL 0 global PLL
 * PLL 1 HF scorpion clock
 * PLL 2 QDSP6 clock
 * PLL 3 HF HSDDR clk
 * PLL 4 CSM seawolf
 * PLL 5 MSM and CSM
 * PLL 6 CSM C2K and MSM
 * PLL 7 Ethernet MAC
 */

static uint32_t pll_mode_addr[NUM_PLL] = {
	[PLL_0] = (uint32_t) REG(0x300),
	[PLL_1] = (uint32_t) REG(0), /* N/A */
	[PLL_2] = (uint32_t) REG(0x31c),
	[PLL_3] = (uint32_t) REG(0x710),
	[PLL_4] = (uint32_t) REG(0x338),
	[PLL_5] = (uint32_t) REG(0x354),
	[PLL_6] = (uint32_t) REG(0x370),
	[PLL_7] = (uint32_t) REG(0x38c),
};

static uint32_t pll_count[NUM_PLL];

static struct clk_local clk_local_tbl[] = {
	CLK_MND16(SDCC, SDCC_NS, B(9), B(11), clk_tbl_sdcc, NONE, NULL,
		0, 0),
	CLK_MND16(UARTDM, UARTDM_NS, B(9), B(11), clk_tbl_uartdm, NONE, NULL,
		0, 0),
	CLK_1RATE(UART1, UART1_NS, B(2), B(4), clk_tbl_tcxo,
		0, 0),
	CLK_1RATE(UART2, UART2_NS, B(2), B(4), clk_tbl_tcxo,
		0, 0),
	CLK_1RATE(I2C, I2C_NS, B(2), B(4), clk_tbl_tcxo,
		0, 0),
	CLK_1RATE(SSBI1, SSBI_NS, B(25), B(7), clk_tbl_tcxo,
		0, 0),
	CLK_1RATE(SSBI2, SSBI_NS, B(26), B(11), clk_tbl_tcxo,
		0, 0),
	CLK_1RATE(SSBI3, SSBI_NS, B(27), B(15), clk_tbl_tcxo,
		0, 0),
	CLK_MND16(SDCC, ETH_NS, B(9), B(11), clk_tbl_eth, NONE, NULL,
		0, 0),

	/* For global clocks to be on we must have GLBL_ROOT_ENA set */
	CLK_1RATE(GLBL_ROOT, GLBL_CLK_ENA, 0, B(29), clk_tbl_axi,
		0, 0),

	/* Peripheral bus clocks. */
	CLK_GLBL(ADM, GLBL_CLK_ENA, B(4),
		GLBL_CLK_STATE, B(4)),
	CLK_GLBL(IMEM, GLBL_CLK_ENA_2, B(29),
		GLBL_CLK_STATE_2, B(29)),
	CLK_GLBL(SDCC_H, GLBL_CLK_ENA_2, B(20),
		GLBL_CLK_STATE_2, B(20)),
	CLK_GLBL(UARTDM_P, GLBL_CLK_ENA_2, B(21),
		GLBL_CLK_STATE_2, B(21)),

	/* AXI bridge clocks. */
	CLK_BRIDGE(AXI_HSDDR, GLBL_CLK_ENA, B(8), GLBL_ROOT,
		GLBL_CLK_STATE, B(8)),
	CLK_BRIDGE(AXI_FAB_M0, GLBL_CLK_ENA, B(9), GLBL_ROOT,
		GLBL_CLK_STATE, B(9)),
	CLK_BRIDGE(AXI_FAB_M1, GLBL_CLK_ENA, B(10), GLBL_ROOT,
		GLBL_CLK_STATE, B(10)),
	CLK_BRIDGE(AXI_FAB_M2, GLBL_CLK_ENA, B(11), GLBL_ROOT,
		GLBL_CLK_STATE, B(11)),
	CLK_BRIDGE(AXI_FAB_M3, GLBL_CLK_ENA, B(12), GLBL_ROOT,
		GLBL_CLK_STATE, B(12)),
	CLK_BRIDGE(AXI_FAB_M4, GLBL_CLK_ENA, B(13), GLBL_ROOT,
		GLBL_CLK_STATE, B(13)),
	CLK_BRIDGE(AXI_FAB_M5, GLBL_CLK_ENA, B(14), GLBL_ROOT,
		GLBL_CLK_STATE, B(14)),
	CLK_BRIDGE(AXI_FAB_M6, GLBL_CLK_ENA, B(15), GLBL_ROOT,
		GLBL_CLK_STATE, B(15)),
	CLK_BRIDGE(AXI_FAB_M7, GLBL_CLK_ENA, B(16), GLBL_ROOT,
		GLBL_CLK_STATE, B(16)),
	CLK_BRIDGE(AXI_FAB_S0, GLBL_CLK_ENA, B(17), GLBL_ROOT,
		GLBL_CLK_STATE, B(17)),
	CLK_BRIDGE(AXI_FAB_S1, GLBL_CLK_ENA, B(18), GLBL_ROOT,
		GLBL_CLK_STATE, B(18)),
	CLK_BRIDGE(AXI_FAB_S2, GLBL_CLK_ENA, B(19), GLBL_ROOT,
		GLBL_CLK_STATE, B(19)),
	CLK_BRIDGE(AXI_FAB_S3, GLBL_CLK_ENA, B(20), GLBL_ROOT,
		GLBL_CLK_STATE, B(20)),
	CLK_BRIDGE(AXI_FAB_S4, GLBL_CLK_ENA, B(21), GLBL_ROOT,
		GLBL_CLK_STATE, B(21)),
	CLK_BRIDGE(AXI_FAB_S5, GLBL_CLK_ENA, B(22), GLBL_ROOT,
		GLBL_CLK_STATE, B(22)),
	CLK_BRIDGE(AXI_FAB_S6, GLBL_CLK_ENA, B(23), GLBL_ROOT,
		GLBL_CLK_STATE, B(23)),
	CLK_BRIDGE(AXI_FAB_S7, GLBL_CLK_ENA, B(24), GLBL_ROOT,
		GLBL_CLK_STATE, B(24)),
	CLK_BRIDGE(AXI_Q6SS_M, GLBL_CLK_ENA, B(25), GLBL_ROOT,
		GLBL_CLK_STATE, B(25)),
	CLK_BRIDGE(AXI_Q6SS_S, GLBL_CLK_ENA, B(26), GLBL_ROOT,
		GLBL_CLK_STATE, B(26)),
	CLK_BRIDGE(AXI_SC, GLBL_CLK_ENA, B(27), GLBL_ROOT,
		GLBL_CLK_STATE, B(27)),
};

static DEFINE_SPINLOCK(clock_reg_lock);
static DEFINE_SPINLOCK(pll_vote_lock);

enum {
	TCXO,
	NUM_XO
};
static unsigned xo_votes[NUM_XO]; /* Tracks the number of users for each XO */

static void vote_for_xo(unsigned xo)
{
	BUG_ON(xo >= NUM_XO);

	xo_votes[xo]++;
}

static void unvote_for_xo(unsigned xo)
{
	BUG_ON(xo >= NUM_XO);

	if (xo_votes[xo]) {
		xo_votes[xo]--;
	} else {
		pr_warning("%s: Reference count mismatch!\n", __func__);
		return;
	}
}

void pll_enable(uint32_t pll)
{
	uint32_t reg_val;
	unsigned long flags;

	BUG_ON(pll >= NUM_PLL);

	spin_lock_irqsave(&pll_vote_lock, flags);
	if (pll_count[pll] == 0) {
		if (pll == PLL_1) {
			/* PLL1 is for Scorpion, and can't be turned
			 * on or off
			 */
		} else {
			/* bit 0 of mode register is PLL_OUTCTRL. 0 = disable,
			 * 1 = enable
			 */
			reg_val = readl(pll_mode_addr[pll]);
			reg_val |= 0x01u;
			writel(reg_val, pll_mode_addr[pll]);
		}
	}
	pll_count[pll]++;
	spin_unlock_irqrestore(&pll_vote_lock, flags);
}

void pll_disable(uint32_t pll)
{
	uint32_t reg_val;
	unsigned long flags;

	BUG_ON(pll >= NUM_PLL);

	spin_lock_irqsave(&pll_vote_lock, flags);
	if (pll_count[pll]) {
		pll_count[pll]--;
	} else {
		pr_warning("Reference count mismatch in PLL disable!\n");
		goto out;
	}

	if (pll_count[pll] == 0) {
		if (pll == PLL_1) {
			;
		} else {
			reg_val = readl(pll_mode_addr[pll]);
			reg_val &= ~0x01u;
			writel(reg_val, pll_mode_addr[pll]);
		}
	}
out:
	spin_unlock_irqrestore(&pll_vote_lock, flags);
}

static void src_enable(uint32_t src)
{
	switch (src) {
	case SRC_MAX:
		/*
		 * SRC_MAX is used as a placeholder for some freqencies that
		 * don't have any direct PLL dependency. Instead they source
		 * off an external/internal clock which takes care of any
		 * PLL or XO dependency.
		 */
		break;
	case SRC_TCXO:
		vote_for_xo(TCXO);
		break;
	case SRC_AXI:
		/* AXI clock relies on PLL0 and PLL4 */
		break;
	case SRC_PLL_GLOBAL:
	case SRC_PLL_ARM:
	case SRC_PLL_QDSP6:
	case SRC_PLL_DDR:
		pll_enable(src_pll_tbl[src]);
		break;
	default:
		break;
	}
}

static void src_disable(uint32_t src)
{
	switch (src) {
	case SRC_MAX:
		/*
		 * SRC_MAX is used as a placeholder for some freqencies that
		 * don't have any direct PLL dependency. Instead they source
		 * off an external/internal clock which takes care of any
		 * PLL or XO dependency.
		 */
		break;
	case SRC_TCXO:
		unvote_for_xo(TCXO);
		break;
	case SRC_AXI:
		/* AXI clock relies on PLL0 and PLL4 */
		break;
	case SRC_PLL_GLOBAL:
	case SRC_PLL_ARM:
	case SRC_PLL_QDSP6:
	case SRC_PLL_DDR:
		pll_disable(src_pll_tbl[src]);
		break;
	default:
		break;
	}
}

static unsigned msmc1_votes[MSMC1_END];

static int update_msmc1(void)
{
	return 0;
}

static void unvote_msmc1(unsigned level)
{
	if (level >= ARRAY_SIZE(msmc1_votes))
		return;

	if (msmc1_votes[level]) {
		msmc1_votes[level]--;
	} else {
		pr_warning("%s: Reference counts are incorrect\n", __func__);
		return;
	}

	update_msmc1();
}

static int vote_msmc1(unsigned level)
{
	int ret;

	if (level >= ARRAY_SIZE(msmc1_votes))
		return 0;

	msmc1_votes[level]++;
	ret = update_msmc1();
	if (ret)
		msmc1_votes[level]--;

	return ret;
}

/*
 * SoC specific register-based control of clocks.
 */
static int _soc_clk_enable(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	void *ns_reg = REG(t->ns_reg);
	uint32_t reg_val = 0;

	reg_val = readl(ns_reg);
	if (t->type == MND) {
		/* mode can be either 0 or 1. So the R-value of the
		 * expression will evaluate to MNCNTR_EN_MASK or 0. This
		 * avoids the need for a "if(mode == 1)". A "&" will not work
		 * here. */
		reg_val |= (MNCNTR_EN_MASK * t->current_freq->mode);
		writel(reg_val, ns_reg);
	}
	if (t->root_en_mask) {
		reg_val |= t->root_en_mask;
		writel(reg_val, ns_reg);
	}
	if (t->br_en_mask) {
		reg_val |= t->br_en_mask;
		writel(reg_val, ns_reg);
	}
	if (t->halt_reg) {
		uint32_t halted, count = 0;

		/* Wait for the halt bit to clear, but timeout after 100usecs
		 * since the halt bit may be buggy. */
		mb();
		while ((halted = readl(REG(t->halt_reg)) & t->halt_mask)
			&& count++ < 100)
			udelay(1);
		if (halted)
			pr_warning("%s: clock %d never turned on\n", __func__,
					id);
	}
	return 0;
}

static void _soc_clk_disable(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	void *ns_reg = REG(t->ns_reg);
	uint32_t reg_val = 0;

	reg_val = readl(ns_reg);

	if (t->br_en_mask) {
		reg_val &= ~(t->br_en_mask);
		writel(reg_val, ns_reg);
	}
	if (t->halt_reg) {
		uint32_t halted, count = 0;

		/* Wait for the halt bit to be set, but timeout after 100usecs
		 * since the halt bit may be buggy. */
		mb();
		while (!(halted = readl(REG(t->halt_reg)) & t->halt_mask)
			&& count++ < 100)
			udelay(1);
		if (!halted)
			pr_warning("%s: clock %d never turned off\n", __func__,
					id);
	}
	if (t->root_en_mask) {
		reg_val &= ~(t->root_en_mask);
		writel(reg_val, ns_reg);
	}
	if (t->type == MND) {
		reg_val &= ~MNCNTR_EN_MASK;
		writel(reg_val, ns_reg);
	}
}

static int soc_clk_enable_nolock(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	int ret = 0;

	if (!t->count) {
		ret = vote_msmc1(t->current_freq->msmc1);
		if (ret)
			return ret;
		if (t->parent != C(NONE)) {
			ret = soc_clk_enable_nolock(t->parent);
			if (ret)
				return ret;
		}
		src_enable(t->current_freq->src);
		ret = _soc_clk_enable(id);
	}
	t->count++;

	return ret;
}

static void soc_clk_disable_nolock(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];

	if (!t->count) {
		pr_warning("Reference count mismatch in clock disable!\n");
		return;
	}
	if (t->count)
		t->count--;
	if (t->count == 0) {
		_soc_clk_disable(id);
		src_disable(t->current_freq->src);
		unvote_msmc1(t->current_freq->msmc1);
		if (t->parent != C(NONE))
			soc_clk_disable_nolock(t->parent);
	}

	return;
}

static int update_pwr_rail(unsigned id, int enable)
{
	return 0;
}

static int soc_clk_enable(unsigned id)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&clock_reg_lock, flags);
	ret = soc_clk_enable_nolock(id);
	if (ret)
		goto unlock;
	/*
	 * The modem might modify the register bits for the clock branch when
	 * the rail is enabled/disabled, so enable the rail inside the lock
	 * instead of outside it.
	 */
	ret = update_pwr_rail(id, 1);
	if (ret)
		soc_clk_disable_nolock(id);
unlock:
	spin_unlock_irqrestore(&clock_reg_lock, flags);

	return ret;
}

static void soc_clk_disable(unsigned id)
{
	unsigned long flags;

	spin_lock_irqsave(&clock_reg_lock, flags);
	update_pwr_rail(id, 0);
	soc_clk_disable_nolock(id);
	spin_unlock_irqrestore(&clock_reg_lock, flags);
}

static void soc_clk_auto_off(unsigned id)
{
	unsigned long flags;

	spin_lock_irqsave(&clock_reg_lock, flags);
	_soc_clk_disable(id);
	spin_unlock_irqrestore(&clock_reg_lock, flags);
}

static long soc_clk_round_rate(unsigned id, unsigned rate)
{
	struct clk_local *t = &clk_local_tbl[id];
	struct clk_freq_tbl *f;

	if (t->type != MND && t->type != BASIC)
		return -EINVAL;

	for (f = t->freq_tbl; f->freq_hz != FREQ_END; f++)
		if (f->freq_hz >= rate)
			return f->freq_hz;

	return -EPERM;
}

static int soc_clk_set_rate(unsigned id, unsigned rate)
{
	struct clk_local *t = &clk_local_tbl[id];
	struct clk_freq_tbl *cf = t->current_freq;
	struct clk_freq_tbl *nf;
	uint32_t *chld = t->children;
	void *ns_reg = REG(t->ns_reg);
	void *md_reg = REG(t->md_reg);
	uint32_t reg_val = 0;
	int i, ret = 0;
	unsigned long flags;
	long rounded;

	rounded = soc_clk_round_rate(id, rate);
	if (rounded != rate)
		pr_warning("Use clk_round_rate() before clk_set_rate() with "
			   "clock %u\n", id);
	rate = rounded;

	if (t->type != MND && t->type != BASIC)
		return -EPERM;

	spin_lock_irqsave(&clock_reg_lock, flags);

	if (rate == cf->freq_hz)
		goto release_lock;

	for (nf = t->freq_tbl; nf->freq_hz != FREQ_END; nf++)
		if (nf->freq_hz == rate)
			break;

	if (nf->freq_hz == FREQ_END) {
		ret = -EINVAL;
		goto release_lock;
	}

	if (t->freq_mask == 0) {
		t->current_freq = nf;
		goto release_lock;
	}

	/* Disable all branches before changing rate to prevent jitter. */
	for (i = 0; chld && chld[i] != C(NONE); i++) {
		struct clk_local *ch = &clk_local_tbl[chld[i]];
		/* Don't bother turning off if it is already off.
		 * Checking ch->count is cheaper (cache) than reading and
		 * writing to a register (uncached/unbuffered). */
		if (ch->count) {
			reg_val = readl(REG(ch->ns_reg));
			reg_val &= ~(ch->br_en_mask);
			writel(reg_val, REG(ch->ns_reg));
		}
	}

	if (t->count) {
		_soc_clk_disable(id);

		ret = vote_msmc1(nf->msmc1);
		if (ret)
			goto msmc1_err;
		/* Turn on PLL of the new freq. */
		src_enable(nf->src);
	}

	/* Some clocks share the same register, so must be careful when
	 * assuming a register doesn't need to be re-read. */
	reg_val = readl(ns_reg);
	if (t->type == MND) {
		reg_val |= MNCNTR_RST_MASK;
		writel(reg_val, ns_reg);
		/* TODO: Currently writing 0's into reserved bits for 8-bit
		 * MND. Can be avoided by adding md_mask. */
		if (nf->mode)
			writel(nf->md_val, md_reg);
		reg_val &= ~MNCNTR_MODE_MASK;
		reg_val |= (MNCNTR_MODE * nf->mode);
	}
	reg_val &= ~(t->freq_mask);
	reg_val |= nf->ns_val;
	writel(reg_val, ns_reg);

	if (t->type == MND) {
		reg_val &= ~MNCNTR_RST_MASK;
		writel(reg_val, ns_reg);
	}

	if (t->count) {
		/* Turn off PLL of the old freq. */
		src_disable(cf->src);
		unvote_msmc1(cf->msmc1);
	}

	/* Current freq must be updated before _soc_clk_enable() is called to
	 * make sure the MNCNTR_E bit is set correctly. */
	t->current_freq = nf;

msmc1_err:
	if (t->count)
		_soc_clk_enable(id);
	/* Enable only branches that were ON before. */
	for (i = 0; chld && chld[i] != C(NONE); i++) {
		struct clk_local *ch = &clk_local_tbl[chld[i]];
		if (ch->count) {
			reg_val = readl(REG(ch->ns_reg));
			reg_val |= ch->br_en_mask;
			writel(reg_val, REG(ch->ns_reg));
		}
	}

release_lock:
	spin_unlock_irqrestore(&clock_reg_lock, flags);
	return ret;
}

static int soc_clk_set_min_rate(unsigned id, unsigned rate)
{
	long rounded = soc_clk_round_rate(id, rate);
	return soc_clk_set_rate(id, rounded);
}

static int soc_clk_set_max_rate(unsigned id, unsigned rate)
{
	return -EPERM;
}

static int soc_clk_set_flags(unsigned id, unsigned clk_flags)
{
	uint32_t ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&clock_reg_lock, flags);
	switch (id) {
	default:
		ret = -EPERM;
	}
	spin_unlock_irqrestore(&clock_reg_lock, flags);

	return ret;
}

static unsigned soc_clk_get_rate(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	unsigned long flags;
	unsigned ret = 0;

	if (t->type == NORATE)
		return 0;

	spin_lock_irqsave(&clock_reg_lock, flags);
	ret = t->current_freq->freq_hz;
	spin_unlock_irqrestore(&clock_reg_lock, flags);

	/* Return 0 if the rate has never been set. Might not be correct,
	 * but it's good enough. */
	if (ret == FREQ_END)
		ret = 0;

	return ret;
}

static unsigned soc_clk_is_enabled(unsigned id)
{
	return !!(clk_local_tbl[id].count);
}

struct clk_ops clk_ops_fsm9xxx = {
	.enable = soc_clk_enable,
	.disable = soc_clk_disable,
	.auto_off = soc_clk_auto_off,
	.reset = pc_clk_reset,
	.set_rate = soc_clk_set_rate,
	.set_min_rate = soc_clk_set_min_rate,
	.set_max_rate = soc_clk_set_max_rate,
	.set_flags = soc_clk_set_flags,
	.get_rate = soc_clk_get_rate,
	.is_enabled = soc_clk_is_enabled,
	.round_rate = soc_clk_round_rate,
};

void __init msm_clk_soc_set_ops(struct clk *clk)
{
	if (!clk->ops)
		clk->ops = &clk_ops_fsm9xxx;
}

#define set_1rate(clk) \
	soc_clk_set_rate(C(clk), clk_local_tbl[C(clk)].freq_tbl->freq_hz)
void __init msm_clk_soc_init(void)
{
	/* This is just to update the driver data structures. The actual
	 * register set up is taken care of in the register init loop
	 * or is the default value out of reset. */
	set_1rate(I2C);
	set_1rate(UART1);
	set_1rate(UART2);
	set_1rate(GLBL_ROOT);
}
