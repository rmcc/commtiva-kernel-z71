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
#ifndef _ARCH_ARM_MACH_MSM_TLMM_FSM9XXX_H
#define _ARCH_ARM_MACH_MSM_TLMM_FSM9XXX_H

#define  NR_MSM_GPIOS 168

struct fsm9xxx_gpio_regs {
	void __iomem *in;
	void __iomem *out;
	void __iomem *oe;
};

struct fsm9xxx_gpio_platform_data {
	unsigned gpio_base;
	unsigned ngpio;
	unsigned irq_base;
	unsigned irq_summary;
	struct fsm9xxx_gpio_regs regs;
};

#define GPIO_OUT(gpio)    (MSM_TLMM_BASE + (0x4 * (gpio)/32))
#define GPIO_IN(gpio)     (MSM_TLMM_BASE + 0x48 + (0x4 * (gpio)/32))
#define GPIO_PAGE         (MSM_TLMM_BASE + 0x40)
#define GPIO_CONFIG       (MSM_TLMM_BASE + 0x44)
#define GPIO_REG(off)     (MSM_TLMM_BASE + (off))


/* output value */
#define GPIO_OUT_0         GPIO_REG(0x00)   /* gpio  31-0   */
#define GPIO_OUT_1         GPIO_REG(0x04)   /* gpio  63-32  */
#define GPIO_OUT_2         GPIO_REG(0x08)   /* gpio  95-64  */
#define GPIO_OUT_3         GPIO_REG(0x0C)   /* gpio 127-96  */
#define GPIO_OUT_4         GPIO_REG(0x10)   /* gpio 159-128 */
#define GPIO_OUT_5         GPIO_REG(0x14)   /* gpio 167-160 */

/* same pin map as above, output enable */
#define GPIO_OE_0          GPIO_REG(0x20)
#define GPIO_OE_1          GPIO_REG(0x24)
#define GPIO_OE_2          GPIO_REG(0x28)
#define GPIO_OE_3          GPIO_REG(0x2C)
#define GPIO_OE_4          GPIO_REG(0x30)
#define GPIO_OE_5          GPIO_REG(0x34)

/* same pin map as above, input read */
#define GPIO_IN_0          GPIO_REG(0x48)
#define GPIO_IN_1          GPIO_REG(0x4C)
#define GPIO_IN_2          GPIO_REG(0x50)
#define GPIO_IN_3          GPIO_REG(0x54)
#define GPIO_IN_4          GPIO_REG(0x58)
#define GPIO_IN_5          GPIO_REG(0x5C)

#endif /* _ARCH_ARM_MACH_MSM_TLMM_FSM9XXX_H */
