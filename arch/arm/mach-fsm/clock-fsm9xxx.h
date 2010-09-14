/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __ARCH_ARM_MACH_MSM_CLOCK_9XXX_H
#define __ARCH_ARM_MACH_MSM_CLOCK_9XXX_H

enum {
	PLL_0 = 0,
	PLL_1,
	PLL_2,
	PLL_3,
	PLL_4,
	PLL_5,
	PLL_6,
	PLL_7,
	PLL_8,
	NUM_PLL
};

enum {
	L_FSM9XXX_NONE_CLK = -1,

	L_FSM9XXX_SDCC_CLK,
	L_FSM9XXX_UARTDM_CLK,
	L_FSM9XXX_UART1_CLK,
	L_FSM9XXX_UART2_CLK,
	L_FSM9XXX_I2C_CLK,
	L_FSM9XXX_SSBI1_CLK,
	L_FSM9XXX_SSBI2_CLK,
	L_FSM9XXX_SSBI3_CLK,
	L_FSM9XXX_ETH_CLK,

	L_FSM9XXX_GLBL_ROOT_CLK,

	L_FSM9XXX_ADM_CLK,
	L_FSM9XXX_IMEM_CLK,
	L_FSM9XXX_SDCC_H_CLK, /* SDCC AHB */
	L_FSM9XXX_UARTDM_P_CLK, /* UART1 DM pbus */

	/* bridge clocks */
	L_FSM9XXX_AXI_HSDDR_CLK, /* HSDDR AXI */
	L_FSM9XXX_AXI_FAB_M0_CLK, /* System FABRIC AXI master 0 */
	L_FSM9XXX_AXI_FAB_M1_CLK, /* System FABRIC AXI master 1 */
	L_FSM9XXX_AXI_FAB_M2_CLK, /* System FABRIC AXI master 2 */
	L_FSM9XXX_AXI_FAB_M3_CLK, /* System FABRIC AXI master 3 */
	L_FSM9XXX_AXI_FAB_M4_CLK, /* System FABRIC AXI master 4 */
	L_FSM9XXX_AXI_FAB_M5_CLK, /* System FABRIC AXI master 5 */
	L_FSM9XXX_AXI_FAB_M6_CLK, /* System FABRIC AXI master 6 */
	L_FSM9XXX_AXI_FAB_M7_CLK, /* System FABRIC AXI master 7 */
	L_FSM9XXX_AXI_FAB_S0_CLK, /* System FABRIC AXI slave 0 */
	L_FSM9XXX_AXI_FAB_S1_CLK, /* System FABRIC AXI slave 1 */
	L_FSM9XXX_AXI_FAB_S2_CLK, /* System FABRIC AXI slave 2 */
	L_FSM9XXX_AXI_FAB_S3_CLK, /* System FABRIC AXI slave 3 */
	L_FSM9XXX_AXI_FAB_S4_CLK, /* System FABRIC AXI slave 4 */
	L_FSM9XXX_AXI_FAB_S5_CLK, /* System FABRIC AXI slave 5 */
	L_FSM9XXX_AXI_FAB_S6_CLK, /* System FABRIC AXI slave 6 */
	L_FSM9XXX_AXI_FAB_S7_CLK, /* System FABRIC AXI slave 7 */
	L_FSM9XXX_AXI_Q6SS_M_CLK, /* Q6SS AXI master */
	L_FSM9XXX_AXI_Q6SS_S_CLK, /* Q6SS AXI slave */
	L_FSM9XXX_AXI_SC_CLK, /* Scorpion */

	L_FSM9XXX_NR_CLKS
};

void pll_enable(uint32_t pll);
void pll_disable(uint32_t pll);

extern int internal_pwr_rail_ctl_auto(unsigned rail_id, bool enable);

#define CLK_FSM9XXX(clk_name, clk_id, clk_dev, clk_flags) {	\
	.name = clk_name, \
	.id = L_FSM9XXX_##clk_id, \
	.remote_id = P_##clk_id, \
	.flags = clk_flags, \
	.dev = clk_dev, \
	.dbg_name = #clk_id, \
	}

#endif

