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
#ifndef __ASM_ARCH_MSM_IOMAP_FSM9XXX_H
#define __ASM_ARCH_MSM_IOMAP_FSM9XXX_H

/* Physical base address and size of peripherals.
 * Ordered by the virtual base addresses they will be mapped at.
 *
 * MSM_VIC_BASE must be an value that can be loaded via a "mov"
 * instruction, otherwise entry-macro.S will not compile.
 *
 * If you add or remove entries here, you'll want to edit the
 * msm_io_desc array in arch/arm/mach-fsm/io.c to reflect your
 * changes.
 *
 */

#define MSM_VIC_BASE          IOMEM(0xE0000000)
#define MSM_VIC_PHYS          0x9C080000
#define MSM_VIC_SIZE          SZ_4K

/* INT_CTL2_BASE */
#define MSM_SIRC_BASE         IOMEM(0xE0001000)
#define MSM_SIRC_PHYS         0x94190000
#define MSM_SIRC_SIZE         SZ_4K

#define MSM_CSR_BASE          IOMEM(0xE0002000)
#define MSM_CSR_PHYS          0x9C000000
#define MSM_CSR_SIZE          SZ_4K

#define MSM_TMR_PHYS          MSM_CSR_PHYS
#define MSM_TMR_BASE          MSM_CSR_BASE
#define MSM_TMR_SIZE          SZ_4K

#define MSM_DMOV_BASE         IOMEM(0xE0003000)
#define MSM_DMOV_PHYS         0x94310000
#define MSM_DMOV_SIZE         SZ_4K

#define MSM_GPIO1_BASE        IOMEM(0xE0004000)
#define MSM_GPIO1_PHYS        0x94040000
#define MSM_GPIO1_SIZE        SZ_4K

#define MSM_CLK_CTL_BASE      IOMEM(0xE0005000)
#define MSM_CLK_CTL_PHYS      0x94020000
#define MSM_CLK_CTL_SIZE      SZ_4K


#define MSM_ACC_BASE          IOMEM(0xE0006000)
#define MSM_ACC_PHYS          0x9C001000
#define MSM_ACC_SIZE          SZ_4K

#define MSM_SAW_BASE          IOMEM(0xE0007000)
#define MSM_SAW_PHYS          0x9C002000
#define MSM_SAW_SIZE          SZ_4K

#define MSM_GCC_BASE	      IOMEM(0xE0008000)
#define MSM_GCC_PHYS	      0x9C082000
#define MSM_GCC_SIZE	      SZ_4K

#define MSM_TCSR_BASE	      IOMEM(0xE0009000)
#define MSM_TCSR_PHYS	      0x94030000
#define MSM_TCSR_SIZE	      SZ_4K


#define MSM_TLMM_BASE         IOMEM(0xE000A000)
#define MSM_TLMM_PHYS         0x94040000
#define MSM_TLMM_SIZE         SZ_4K

#define MSM_SHARED_RAM_BASE   IOMEM(0xE0100000)
#define MSM_SHARED_RAM_SIZE   SZ_1M

#define MSM_UART1_PHYS        0x94000000
#define MSM_UART1_SIZE        SZ_4K

#define MSM_UART2_PHYS        0x94100000
#define MSM_UART2_SIZE        SZ_4K


#ifdef CONFIG_MSM_DEBUG_UART
#define MSM_DEBUG_UART_BASE   0xE1000000
#if CONFIG_MSM_DEBUG_UART == 1
#define MSM_DEBUG_UART_PHYS   MSM_UART1_PHYS
#elif CONFIG_MSM_DEBUG_UART == 2
#define MSM_DEBUG_UART_PHYS   MSM_UART2_PHYS
#endif
#define MSM_DEBUG_UART_SIZE   SZ_4K
#endif

#endif
