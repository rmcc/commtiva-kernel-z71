/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef VID_C_720P_H
#define VID_C_720P_H
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/system.h>

#define VIDC_720P_IN(reg)                       VIDC_##reg##_IN
#define VIDC_720P_INM(reg,  mask)                VIDC_##reg##_INM(mask)
#define VIDC_720P_OUT(reg,  val)                 VIDC_##reg##_OUT(val)
#define VIDC_720P_OUTI(reg,  index,  val)         VIDC_##reg##_OUTI(index, val)
#define VIDC_720P_OUTM(reg,  mask,  val)          VIDC_##reg##_OUTM(mask,  val)
#define VIDC_720P_SHFT(reg,  field)              VIDC_##reg##_##field##_SHFT
#define VIDC_720P_FMSK(reg,  field)              VIDC_##reg##_##field##_BMSK

#define VIDC_720P_INF(io, field) (VIDC_720P_INM(io, VIDC_720P_FMSK(io, field)) \
		>> VIDC_720P_SHFT(io,  field))
#define VIDC_720P_OUTF(io, field, val) \
		VIDC_720P_OUTM(io, VIDC_720P_FMSK(io, field), \
		val << VIDC_720P_SHFT(io,  field))

#define __inpdw(port)	ioread32(port)
#define __outpdw(port,  val) iowrite32(val, port)

#define in_dword_masked(addr,  mask) (__inpdw(addr) & (mask))

#define out_dword(addr,  val)        __outpdw(addr, val)

#define out_dword_masked(io,  mask,  val,  shadow)  \
do { \
	shadow = (shadow & (u32)(~(mask))) | ((u32)((val) & (mask))); \
	(void) out_dword(io,  shadow); \
} while (0)

#define out_dword_masked_ns(io,  mask,  val,  current_reg_content) \
	(void) out_dword(io,  ((current_reg_content & (u32)(~(mask))) | \
				((u32)((val) & (mask)))))

extern u32 VID_C_REG_817468_SHADOW;
extern u32 VID_C_REG_767703_SHADOW;
extern u32 VID_C_REG_703273_SHADOW;
extern u32 VID_C_REG_688165_SHADOW;
extern u32 VID_C_REG_842480_SHADOW;
extern u32 VID_C_REG_830933_SHADOW;
extern u32 VID_C_REG_396763_SHADOW[32];
extern u32 VID_C_REG_492691_SHADOW;
extern u32 VID_C_REG_586901_SHADOW;
extern u32 VID_C_REG_37680_SHADOW;
extern u32 VID_C_REG_797110_SHADOW;
extern u32 VID_C_REG_928824_SHADOW;
extern u32 VID_C_REG_4084_SHADOW;
extern u32 VID_C_REG_623236_SHADOW;
extern u32 VID_C_REG_967981_SHADOW;
extern u32 VID_C_REG_754370_SHADOW;
extern u32 VID_C_REG_934882_SHADOW;
extern u32 VID_C_REG_354189_SHADOW;
extern u32 VID_C_REG_360816_SHADOW;
extern u32 VID_C_REG_812777_SHADOW;
extern u32 VID_C_REG_990962_SHADOW;
extern u32 VID_C_REG_684964_SHADOW;

extern u8 *vid_c_base_addr;

#define VIDC720P_BASE  vid_c_base_addr
#define VIDC_720P_WRAPPER_REG_BASE               (VIDC720P_BASE + \
		0x00000000)
#define VIDC_720P_WRAPPER_REG_BASE_PHYS          VIDC_720P_BASE_PHYS

#define VIDC_REG_898679_ADDR                     \
	(VIDC_720P_WRAPPER_REG_BASE      + 00000000)
#define VIDC_REG_898679_PHYS                     \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 00000000)
#define VIDC_REG_898679_RMSK                            0x1
#define VIDC_REG_898679_SHFT                              0
#define VIDC_REG_898679_IN                       \
	in_dword_masked(VIDC_REG_898679_ADDR,  VIDC_REG_898679_RMSK)
#define VIDC_REG_898679_INM(m)                   \
	in_dword_masked(VIDC_REG_898679_ADDR,  m)
#define VIDC_REG_898679_OUT(v)                   \
	out_dword(VIDC_REG_898679_ADDR, v)
#define VIDC_REG_898679_OUTM(m, v)                \
do { \
	out_dword_masked_ns(VIDC_REG_898679_ADDR, m, v, \
			VIDC_REG_898679_IN); \
} while (0)
#define VIDC_REG_898679_DMA_START_BMSK                  0x1
#define VIDC_REG_898679_DMA_START_SHFT                    0

#define VIDC_REG_639306_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000000c)
#define VIDC_REG_639306_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000000c)
#define VIDC_REG_639306_RMSK                 0xffffffff
#define VIDC_REG_639306_SHFT                          0
#define VIDC_REG_639306_IN                   \
	in_dword_masked(VIDC_REG_639306_ADDR,  \
			VIDC_REG_639306_RMSK)
#define VIDC_REG_639306_INM(m)               \
	in_dword_masked(VIDC_REG_639306_ADDR,  m)
#define VIDC_REG_639306_OUT(v)               \
	out_dword(VIDC_REG_639306_ADDR, v)
#define VIDC_REG_639306_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_639306_ADDR, m, v, \
			VIDC_REG_639306_IN); \
} while (0)
#define VIDC_REG_639306_BOOTCODE_SIZE_BMSK   0xffffffff
#define VIDC_REG_639306_BOOTCODE_SIZE_SHFT            0

#define VIDC_REG_341300_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000014)
#define VIDC_REG_341300_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000014)
#define VIDC_REG_341300_RMSK                   0xffffffff
#define VIDC_REG_341300_SHFT                            0
#define VIDC_REG_341300_IN                     \
	in_dword_masked(VIDC_REG_341300_ADDR,  \
			VIDC_REG_341300_RMSK)
#define VIDC_REG_341300_INM(m)                 \
	in_dword_masked(VIDC_REG_341300_ADDR,  m)
#define VIDC_REG_341300_OUT(v)                 \
	out_dword(VIDC_REG_341300_ADDR, v)
#define VIDC_REG_341300_OUTM(m, v)              \
do { \
	out_dword_masked_ns(VIDC_REG_341300_ADDR, m, v, \
			VIDC_REG_341300_IN); \
} while (0)
#define VIDC_REG_341300_DMA_EXTADDR_BMSK       0xffffffff
#define VIDC_REG_341300_DMA_EXTADDR_SHFT                0

#define VIDC_REG_766291_ADDR_ADDR            \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000018)
#define VIDC_REG_766291_ADDR_PHYS            \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000018)
#define VIDC_REG_766291_ADDR_RMSK            0xffffffff
#define VIDC_REG_766291_ADDR_SHFT                     0
#define VIDC_REG_766291_ADDR_IN              \
	in_dword_masked(VIDC_REG_766291_ADDR_ADDR,  \
			VIDC_REG_766291_ADDR_RMSK)
#define VIDC_REG_766291_ADDR_INM(m)          \
	in_dword_masked(VIDC_REG_766291_ADDR_ADDR,  m)
#define VIDC_REG_766291_ADDR_OUT(v)          \
	out_dword(VIDC_REG_766291_ADDR_ADDR, v)
#define VIDC_REG_766291_ADDR_OUTM(m, v)       \
do { \
	out_dword_masked_ns(VIDC_REG_766291_ADDR_ADDR, m, v, \
			VIDC_REG_766291_ADDR_IN); \
} while (0)
#define VIDC_REG_715796_ADDR_BMSK 0xffffffff
#define VIDC_REG_715796_ADDR_SHFT          0

#define VIDC_REG_592511_ADDR_ADDR              \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000001c)
#define VIDC_REG_592511_ADDR_PHYS              \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000001c)
#define VIDC_REG_592511_ADDR_RMSK              0xffffffff
#define VIDC_REG_592511_ADDR_SHFT                       0
#define VIDC_REG_592511_ADDR_IN                \
	in_dword_masked(VIDC_REG_592511_ADDR_ADDR,  \
			VIDC_REG_592511_ADDR_RMSK)
#define VIDC_REG_592511_ADDR_INM(m)            \
	in_dword_masked(VIDC_REG_592511_ADDR_ADDR,  m)
#define VIDC_REG_592511_ADDR_OUT(v)            \
	out_dword(VIDC_REG_592511_ADDR_ADDR, v)
#define VIDC_REG_592511_ADDR_OUTM(m, v)         \
do { \
	out_dword_masked_ns(VIDC_REG_592511_ADDR_ADDR, m, v, \
			VIDC_REG_592511_ADDR_IN); \
} while (0)
#define VIDC_REG_592511_ADDR_EXT_BUF_END_ADDR_BMSK 0xffffffff
#define VIDC_REG_592511_ADDR_EXT_BUF_END_ADDR_SHFT          0

#define VIDC_REG_133233_ADDR_ADDR                  \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000020)
#define VIDC_REG_133233_ADDR_PHYS                  \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000020)
#define VIDC_REG_133233_ADDR_RMSK                  0xffffffff
#define VIDC_REG_133233_ADDR_SHFT                           0
#define VIDC_REG_133233_ADDR_IN                    \
	in_dword_masked(VIDC_REG_133233_ADDR_ADDR,  \
			VIDC_REG_133233_ADDR_RMSK)
#define VIDC_REG_133233_ADDR_INM(m)                \
	in_dword_masked(VIDC_REG_133233_ADDR_ADDR,  m)
#define VIDC_REG_133233_ADDR_OUT(v)                \
	out_dword(VIDC_REG_133233_ADDR_ADDR, v)
#define VIDC_REG_133233_ADDR_OUTM(m, v)             \
do { \
	out_dword_masked_ns(VIDC_REG_133233_ADDR_ADDR, m, v, \
			VIDC_REG_133233_ADDR_IN); \
} while (0)
#define VIDC_REG_133233_ADDR_DMA_INT_ADDR_BMSK     0xffffffff
#define VIDC_REG_133233_ADDR_DMA_INT_ADDR_SHFT              0

#define VIDC_REG_713038_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000024)
#define VIDC_REG_713038_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000024)
#define VIDC_REG_713038_RMSK                 0xffffffff
#define VIDC_REG_713038_SHFT                          0
#define VIDC_REG_713038_IN                   \
	in_dword_masked(VIDC_REG_713038_ADDR,  \
			VIDC_REG_713038_RMSK)
#define VIDC_REG_713038_INM(m)               \
	in_dword_masked(VIDC_REG_713038_ADDR,  m)
#define VIDC_REG_713038_OUT(v)               \
	out_dword(VIDC_REG_713038_ADDR, v)
#define VIDC_REG_713038_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_713038_ADDR, m, v, \
			VIDC_REG_713038_IN); \
} while (0)
#define VIDC_REG_713038_HOST_PTR_ADDR_BMSK   0xffffffff
#define VIDC_REG_713038_HOST_PTR_ADDR_SHFT            0

#define VIDC_REG_405933_ADDR                      \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000028)
#define VIDC_REG_405933_PHYS                      \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000028)
#define VIDC_REG_405933_RMSK                             0x1
#define VIDC_REG_405933_SHFT                               0
#define VIDC_REG_405933_IN                        \
	in_dword_masked(VIDC_REG_405933_ADDR,  VIDC_REG_405933_RMSK)
#define VIDC_REG_405933_INM(m)                    \
	in_dword_masked(VIDC_REG_405933_ADDR,  m)
#define VIDC_REG_405933_OUT(v)                    \
	out_dword(VIDC_REG_405933_ADDR, v)
#define VIDC_REG_405933_OUTM(m, v)                 \
do { \
	out_dword_masked_ns(VIDC_REG_405933_ADDR, m, v, \
			VIDC_REG_405933_IN); \
} while (0)
#define VIDC_REG_405933_LAST_DEC_BMSK                    0x1
#define VIDC_REG_405933_LAST_DEC_SHFT                      0

#define VIDC_REG_496251_ADDR                        \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000002c)
#define VIDC_REG_496251_PHYS                        \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000002c)
#define VIDC_REG_496251_RMSK                               0x1
#define VIDC_REG_496251_SHFT                                 0
#define VIDC_REG_496251_IN                          \
	in_dword_masked(VIDC_REG_496251_ADDR,  VIDC_REG_496251_RMSK)
#define VIDC_REG_496251_INM(m)                      \
	in_dword_masked(VIDC_REG_496251_ADDR,  m)
#define VIDC_REG_496251_DONE_M_BMSK                        0x1
#define VIDC_REG_496251_DONE_M_SHFT                          0

#define VIDC_REG_456374_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000044)
#define VIDC_REG_456374_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000044)
#define VIDC_REG_456374_RMSK                          0x1
#define VIDC_REG_456374_SHFT                            0
#define VIDC_REG_456374_IN                     \
	in_dword_masked(VIDC_REG_456374_ADDR,  \
			VIDC_REG_456374_RMSK)
#define VIDC_REG_456374_INM(m)                 \
	in_dword_masked(VIDC_REG_456374_ADDR,  m)
#define VIDC_REG_456374_OUT(v)                 \
	out_dword(VIDC_REG_456374_ADDR, v)
#define VIDC_REG_456374_OUTM(m, v)              \
do { \
	out_dword_masked_ns(VIDC_REG_456374_ADDR, m, v, \
			VIDC_REG_456374_IN); \
} while (0)
#define VIDC_REG_456374_BITS_ENDIAN_BMSK              0x1
#define VIDC_REG_456374_BITS_ENDIAN_SHFT                0

#define VIDC_REG_822291_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000054)
#define VIDC_REG_822291_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000054)
#define VIDC_REG_822291_RMSK                 0xffffffff
#define VIDC_REG_822291_SHFT                          0
#define VIDC_REG_822291_IN                   \
	in_dword_masked(VIDC_REG_822291_ADDR,  \
			VIDC_REG_822291_RMSK)
#define VIDC_REG_822291_INM(m)               \
	in_dword_masked(VIDC_REG_822291_ADDR,  m)
#define VIDC_REG_822291_OUT(v)               \
	out_dword(VIDC_REG_822291_ADDR, v)
#define VIDC_REG_822291_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_822291_ADDR, m, v, \
			VIDC_REG_822291_IN); \
} while (0)
#define VIDC_REG_822291_DEC_UNIT_SIZE_BMSK   0xffffffff
#define VIDC_REG_822291_DEC_UNIT_SIZE_SHFT            0

#define VIDC_REG_103784_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000058)
#define VIDC_REG_103784_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000058)
#define VIDC_REG_103784_RMSK                 0xffffffff
#define VIDC_REG_103784_SHFT                          0
#define VIDC_REG_103784_IN                   \
	in_dword_masked(VIDC_REG_103784_ADDR,  \
			VIDC_REG_103784_RMSK)
#define VIDC_REG_103784_INM(m)               \
	in_dword_masked(VIDC_REG_103784_ADDR,  m)
#define VIDC_REG_103784_ENC_UNIT_SIZE_BMSK   0xffffffff
#define VIDC_REG_103784_ENC_UNIT_SIZE_SHFT            0

#define VIDC_REG_249975_ADDR                \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000005c)
#define VIDC_REG_249975_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000005c)
#define VIDC_REG_249975_RMSK                       0xf
#define VIDC_REG_249975_SHFT                         0
#define VIDC_REG_249975_IN                  \
	in_dword_masked(VIDC_REG_249975_ADDR,  \
			VIDC_REG_249975_RMSK)
#define VIDC_REG_249975_INM(m)              \
	in_dword_masked(VIDC_REG_249975_ADDR,  m)
#define VIDC_REG_249975_OUT(v)              \
	out_dword(VIDC_REG_249975_ADDR, v)
#define VIDC_REG_249975_OUTM(m, v)           \
do { \
	out_dword_masked_ns(VIDC_REG_249975_ADDR, m, v, \
			VIDC_REG_249975_IN); \
} while (0)
#define VIDC_REG_249975_START_BYTE_NUM_BMSK        0xf
#define VIDC_REG_249975_START_BYTE_NUM_SHFT          0

#define VIDC_REG_257551_ADDR               \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000060)
#define VIDC_REG_257551_PHYS               \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000060)
#define VIDC_REG_257551_RMSK               0xffffffff
#define VIDC_REG_257551_SHFT                        0
#define VIDC_REG_257551_IN                 \
	in_dword_masked(VIDC_REG_257551_ADDR,  \
			VIDC_REG_257551_RMSK)
#define VIDC_REG_257551_INM(m)             \
	in_dword_masked(VIDC_REG_257551_ADDR,  m)
#define VIDC_REG_257551_ENC_HEADER_SIZE_BMSK 0xffffffff
#define VIDC_REG_257551_ENC_HEADER_SIZE_SHFT          0

#define VIDC_REG_30435_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000100)
#define VIDC_REG_30435_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000100)
#define VIDC_REG_30435_RMSK                         0x1f
#define VIDC_REG_30435_SHFT                            0
#define VIDC_REG_30435_IN                     \
	in_dword_masked(VIDC_REG_30435_ADDR,  \
			VIDC_REG_30435_RMSK)
#define VIDC_REG_30435_INM(m)                 \
	in_dword_masked(VIDC_REG_30435_ADDR,  m)
#define VIDC_REG_30435_OUT(v)                 \
	out_dword(VIDC_REG_30435_ADDR, v)
#define VIDC_REG_30435_OUTM(m, v)              \
do { \
	out_dword_masked_ns(VIDC_REG_30435_ADDR, m, v, \
			VIDC_REG_30435_IN); \
} while (0)
#define VIDC_REG_30435_ENC_ON_BMSK                  0x10
#define VIDC_REG_30435_ENC_ON_SHFT                   0x4
#define VIDC_REG_30435_STANDARD_SEL_BMSK             0xf
#define VIDC_REG_30435_STANDARD_SEL_SHFT               0

#define VIDC_REG_255774_ADDR                         \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000104)
#define VIDC_REG_255774_PHYS                         \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000104)
#define VIDC_REG_255774_RMSK                               0x1f
#define VIDC_REG_255774_SHFT                                  0
#define VIDC_REG_255774_IN                           \
	in_dword_masked(VIDC_REG_255774_ADDR,  VIDC_REG_255774_RMSK)
#define VIDC_REG_255774_INM(m)                       \
	in_dword_masked(VIDC_REG_255774_ADDR,  m)
#define VIDC_REG_255774_OUT(v)                       \
	out_dword(VIDC_REG_255774_ADDR, v)
#define VIDC_REG_255774_OUTM(m, v)                    \
do { \
	out_dword_masked_ns(VIDC_REG_255774_ADDR, m, v, \
			VIDC_REG_255774_IN); \
} while (0)
#define VIDC_REG_255774_CH_ID_BMSK                         0x1f
#define VIDC_REG_255774_CH_ID_SHFT                            0

#define VIDC_REG_342124_ADDR                     \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000108)
#define VIDC_REG_342124_PHYS                     \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000108)
#define VIDC_REG_342124_RMSK                            0x1
#define VIDC_REG_342124_SHFT                              0
#define VIDC_REG_342124_IN                       \
	in_dword_masked(VIDC_REG_342124_ADDR,  VIDC_REG_342124_RMSK)
#define VIDC_REG_342124_INM(m)                   \
	in_dword_masked(VIDC_REG_342124_ADDR,  m)
#define VIDC_REG_342124_OUT(v)                   \
	out_dword(VIDC_REG_342124_ADDR, v)
#define VIDC_REG_342124_OUTM(m, v)                \
do { \
	out_dword_masked_ns(VIDC_REG_342124_ADDR, m, v, \
			VIDC_REG_342124_IN); \
} while (0)
#define VIDC_REG_342124_CPU_RESET_BMSK                  0x1
#define VIDC_REG_342124_CPU_RESET_SHFT                    0

#define VIDC_REG_616562_ADDR                        \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000010c)
#define VIDC_REG_616562_PHYS                        \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000010c)
#define VIDC_REG_616562_RMSK                               0x1
#define VIDC_REG_616562_SHFT                                 0
#define VIDC_REG_616562_IN                          \
	in_dword_masked(VIDC_REG_616562_ADDR,  VIDC_REG_616562_RMSK)
#define VIDC_REG_616562_INM(m)                      \
	in_dword_masked(VIDC_REG_616562_ADDR,  m)
#define VIDC_REG_616562_OUT(v)                      \
	out_dword(VIDC_REG_616562_ADDR, v)
#define VIDC_REG_616562_OUTM(m, v)                   \
do { \
	out_dword_masked_ns(VIDC_REG_616562_ADDR, m, v, \
			VIDC_REG_616562_IN); \
} while (0)
#define VIDC_REG_616562_FW_END_BMSK                        0x1
#define VIDC_REG_616562_FW_END_SHFT                          0

#define VIDC_REG_261304_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000110)
#define VIDC_REG_261304_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000110)
#define VIDC_REG_261304_RMSK                           0x1
#define VIDC_REG_261304_SHFT                             0
#define VIDC_REG_261304_IN                      \
	in_dword_masked(VIDC_REG_261304_ADDR,  \
			VIDC_REG_261304_RMSK)
#define VIDC_REG_261304_INM(m)                  \
	in_dword_masked(VIDC_REG_261304_ADDR,  m)
#define VIDC_REG_261304_OUT(v)                  \
	out_dword(VIDC_REG_261304_ADDR, v)
#define VIDC_REG_261304_OUTM(m, v)               \
do { \
	out_dword_masked_ns(VIDC_REG_261304_ADDR, m, v, \
			VIDC_REG_261304_IN); \
} while (0)
#define VIDC_REG_261304_BUS_MASTER_BMSK                0x1
#define VIDC_REG_261304_BUS_MASTER_SHFT                  0

#define VIDC_REG_7977_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000114)
#define VIDC_REG_7977_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000114)
#define VIDC_REG_7977_RMSK                          0x1
#define VIDC_REG_7977_SHFT                            0
#define VIDC_REG_7977_IN                     \
	in_dword_masked(VIDC_REG_7977_ADDR,  \
			VIDC_REG_7977_RMSK)
#define VIDC_REG_7977_INM(m)                 \
	in_dword_masked(VIDC_REG_7977_ADDR,  m)
#define VIDC_REG_7977_OUT(v)                 \
	out_dword(VIDC_REG_7977_ADDR, v)
#define VIDC_REG_7977_OUTM(m, v)              \
do { \
	out_dword_masked_ns(VIDC_REG_7977_ADDR, m, v, \
			VIDC_REG_7977_IN); \
} while (0)
#define VIDC_REG_7977_FRAME_START_BMSK              0x1
#define VIDC_REG_7977_FRAME_START_SHFT                0

#define VIDC_REG_652102_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000118)
#define VIDC_REG_652102_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000118)
#define VIDC_REG_652102_RMSK                        0xffff
#define VIDC_REG_652102_SHFT                             0
#define VIDC_REG_652102_IN                      \
	in_dword_masked(VIDC_REG_652102_ADDR,  \
			VIDC_REG_652102_RMSK)
#define VIDC_REG_652102_INM(m)                  \
	in_dword_masked(VIDC_REG_652102_ADDR,  m)
#define VIDC_REG_652102_OUT(v)                  \
	out_dword(VIDC_REG_652102_ADDR, v)
#define VIDC_REG_652102_OUTM(m, v)               \
do { \
	out_dword_masked_ns(VIDC_REG_652102_ADDR, m, v, \
			VIDC_REG_652102_IN); \
} while (0)
#define VIDC_REG_652102_IMG_SIZE_X_BMSK             0xffff
#define VIDC_REG_652102_IMG_SIZE_X_SHFT                  0

#define VIDC_REG_122456_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000011c)
#define VIDC_REG_122456_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000011c)
#define VIDC_REG_122456_RMSK                        0xffff
#define VIDC_REG_122456_SHFT                             0
#define VIDC_REG_122456_IN                      \
	in_dword_masked(VIDC_REG_122456_ADDR,  \
			VIDC_REG_122456_RMSK)
#define VIDC_REG_122456_INM(m)                  \
	in_dword_masked(VIDC_REG_122456_ADDR,  m)
#define VIDC_REG_122456_OUT(v)                  \
	out_dword(VIDC_REG_122456_ADDR, v)
#define VIDC_REG_122456_OUTM(m, v)               \
do { \
	out_dword_masked_ns(VIDC_REG_122456_ADDR, m, v, \
			VIDC_REG_122456_IN); \
} while (0)
#define VIDC_REG_122456_IMG_SIZE_Y_BMSK             0xffff
#define VIDC_REG_122456_IMG_SIZE_Y_SHFT                  0

#define VIDC_REG_669330_ADDR                       \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000124)
#define VIDC_REG_669330_PHYS                       \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000124)
#define VIDC_REG_669330_RMSK                              0x1
#define VIDC_REG_669330_SHFT                                0
#define VIDC_REG_669330_IN                         \
	in_dword_masked(VIDC_REG_669330_ADDR,  VIDC_REG_669330_RMSK)
#define VIDC_REG_669330_INM(m)                     \
	in_dword_masked(VIDC_REG_669330_ADDR,  m)
#define VIDC_REG_669330_OUT(v)                     \
	out_dword(VIDC_REG_669330_ADDR, v)
#define VIDC_REG_669330_OUTM(m, v)                  \
do { \
	out_dword_masked_ns(VIDC_REG_669330_ADDR, m, v, \
			VIDC_REG_669330_IN); \
} while (0)
#define VIDC_REG_669330_POST_ON_BMSK                      0x1
#define VIDC_REG_669330_POST_ON_SHFT                        0

#define VIDC_REG_530137_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000128)
#define VIDC_REG_530137_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000128)
#define VIDC_REG_530137_RMSK                    0xffffffff
#define VIDC_REG_530137_SHFT                             0
#define VIDC_REG_530137_IN                      \
	in_dword_masked(VIDC_REG_530137_ADDR,  \
			VIDC_REG_530137_RMSK)
#define VIDC_REG_530137_INM(m)                  \
	in_dword_masked(VIDC_REG_530137_ADDR,  m)
#define VIDC_REG_530137_OUT(v)                  \
	out_dword(VIDC_REG_530137_ADDR, v)
#define VIDC_REG_530137_OUTM(m, v)               \
do { \
	out_dword_masked_ns(VIDC_REG_530137_ADDR, m, v, \
			VIDC_REG_530137_IN); \
} while (0)
#define VIDC_REG_530137_QUOTIENT_VAL_BMSK       0xffff0000
#define VIDC_REG_530137_QUOTIENT_VAL_SHFT             0x10
#define VIDC_REG_530137_REMAINDER_VAL_BMSK          0xffff
#define VIDC_REG_530137_REMAINDER_VAL_SHFT               0

#define VIDC_REG_460639_ADDR                \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000012c)
#define VIDC_REG_460639_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000012c)
#define VIDC_REG_460639_RMSK                       0x1
#define VIDC_REG_460639_SHFT                         0
#define VIDC_REG_460639_IN                  \
	in_dword_masked(VIDC_REG_460639_ADDR,  \
			VIDC_REG_460639_RMSK)
#define VIDC_REG_460639_INM(m)              \
	in_dword_masked(VIDC_REG_460639_ADDR,  m)
#define VIDC_REG_460639_OUT(v)              \
	out_dword(VIDC_REG_460639_ADDR, v)
#define VIDC_REG_460639_OUTM(m, v)           \
do { \
	out_dword_masked_ns(VIDC_REG_460639_ADDR, m, v, \
			VIDC_REG_460639_IN); \
} while (0)
#define VIDC_REG_460639_SEQUENCE_START_BMSK        0x1
#define VIDC_REG_460639_SEQUENCE_START_SHFT          0

#define VIDC_REG_893174_ADDR                      \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000130)
#define VIDC_REG_893174_PHYS                      \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000130)
#define VIDC_REG_893174_RMSK                             0x1
#define VIDC_REG_893174_SHFT                               0
#define VIDC_REG_893174_IN                        \
	in_dword_masked(VIDC_REG_893174_ADDR,  VIDC_REG_893174_RMSK)
#define VIDC_REG_893174_INM(m)                    \
	in_dword_masked(VIDC_REG_893174_ADDR,  m)
#define VIDC_REG_893174_OUT(v)                    \
	out_dword(VIDC_REG_893174_ADDR, v)
#define VIDC_REG_893174_OUTM(m, v)                 \
do { \
	out_dword_masked_ns(VIDC_REG_893174_ADDR, m, v, \
			VIDC_REG_893174_IN); \
} while (0)
#define VIDC_REG_893174_SW_RESET_BMSK                    0x1
#define VIDC_REG_893174_SW_RESET_SHFT                      0

#define VIDC_REG_481035_ADDR                      \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000134)
#define VIDC_REG_481035_PHYS                      \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000134)
#define VIDC_REG_481035_RMSK                             0x1
#define VIDC_REG_481035_SHFT                               0
#define VIDC_REG_481035_IN                        \
	in_dword_masked(VIDC_REG_481035_ADDR,  VIDC_REG_481035_RMSK)
#define VIDC_REG_481035_INM(m)                    \
	in_dword_masked(VIDC_REG_481035_ADDR,  m)
#define VIDC_REG_481035_OUT(v)                    \
	out_dword(VIDC_REG_481035_ADDR, v)
#define VIDC_REG_481035_OUTM(m, v)                 \
do { \
	out_dword_masked_ns(VIDC_REG_481035_ADDR, m, v, \
			VIDC_REG_481035_IN); \
} while (0)
#define VIDC_REG_481035_FW_START_BMSK                    0x1
#define VIDC_REG_481035_FW_START_SHFT                      0

#define VIDC_REG_627510_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000138)
#define VIDC_REG_627510_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000138)
#define VIDC_REG_627510_RMSK                           0x1
#define VIDC_REG_627510_SHFT                             0
#define VIDC_REG_627510_IN                      \
	in_dword_masked(VIDC_REG_627510_ADDR,  \
			VIDC_REG_627510_RMSK)
#define VIDC_REG_627510_INM(m)                  \
	in_dword_masked(VIDC_REG_627510_ADDR,  m)
#define VIDC_REG_627510_OUT(v)                  \
	out_dword(VIDC_REG_627510_ADDR, v)
#define VIDC_REG_627510_OUTM(m, v)               \
do { \
	out_dword_masked_ns(VIDC_REG_627510_ADDR, m, v, \
			VIDC_REG_627510_IN); \
} while (0)
#define VIDC_REG_627510_ARM_ENDIAN_BMSK                0x1
#define VIDC_REG_627510_ARM_ENDIAN_SHFT                  0

#define VIDC_REG_49209_ADDR                      \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000013c)
#define VIDC_REG_49209_PHYS                      \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000013c)
#define VIDC_REG_49209_RMSK                             0x1
#define VIDC_REG_49209_SHFT                               0
#define VIDC_REG_49209_IN                        \
	in_dword_masked(VIDC_REG_49209_ADDR,  VIDC_REG_49209_RMSK)
#define VIDC_REG_49209_INM(m)                    \
	in_dword_masked(VIDC_REG_49209_ADDR,  m)
#define VIDC_REG_49209_OUT(v)                    \
	out_dword(VIDC_REG_49209_ADDR, v)
#define VIDC_REG_49209_OUTM(m, v)                 \
do { \
	out_dword_masked_ns(VIDC_REG_49209_ADDR, m, v, \
			VIDC_REG_49209_IN); \
} while (0)
#define VIDC_REG_49209_ERR_CTRL_BMSK                    0x1
#define VIDC_REG_49209_ERR_CTRL_SHFT                      0

#define VIDC_REG_396197_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000200)
#define VIDC_REG_396197_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000200)
#define VIDC_REG_396197_RMSK                 0xffffffff
#define VIDC_REG_396197_SHFT                          0
#define VIDC_REG_396197_IN                   \
	in_dword_masked(VIDC_REG_396197_ADDR,  \
			VIDC_REG_396197_RMSK)
#define VIDC_REG_396197_INM(m)               \
	in_dword_masked(VIDC_REG_396197_ADDR,  m)
#define VIDC_REG_396197_OUT(v)               \
	out_dword(VIDC_REG_396197_ADDR, v)
#define VIDC_REG_396197_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_396197_ADDR, m, v, \
			VIDC_REG_396197_IN); \
} while (0)
#define VIDC_REG_396197_FW_STT_ADDR_0_BMSK   0xffffffff
#define VIDC_REG_396197_FW_STT_ADDR_0_SHFT            0

#define VIDC_REG_368592_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000204)
#define VIDC_REG_368592_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000204)
#define VIDC_REG_368592_RMSK                 0xffffffff
#define VIDC_REG_368592_SHFT                          0
#define VIDC_REG_368592_IN                   \
	in_dword_masked(VIDC_REG_368592_ADDR,  \
			VIDC_REG_368592_RMSK)
#define VIDC_REG_368592_INM(m)               \
	in_dword_masked(VIDC_REG_368592_ADDR,  m)
#define VIDC_REG_368592_OUT(v)               \
	out_dword(VIDC_REG_368592_ADDR, v)
#define VIDC_REG_368592_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_368592_ADDR, m, v, \
			VIDC_REG_368592_IN); \
} while (0)
#define VIDC_REG_368592_FW_STT_ADDR_1_BMSK   0xffffffff
#define VIDC_REG_368592_FW_STT_ADDR_1_SHFT            0

#define VIDC_REG_60733_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000208)
#define VIDC_REG_60733_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000208)
#define VIDC_REG_60733_RMSK                 0xffffffff
#define VIDC_REG_60733_SHFT                          0
#define VIDC_REG_60733_IN                   \
	in_dword_masked(VIDC_REG_60733_ADDR,  \
			VIDC_REG_60733_RMSK)
#define VIDC_REG_60733_INM(m)               \
	in_dword_masked(VIDC_REG_60733_ADDR,  m)
#define VIDC_REG_60733_OUT(v)               \
	out_dword(VIDC_REG_60733_ADDR, v)
#define VIDC_REG_60733_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_60733_ADDR, m, v, \
			VIDC_REG_60733_IN); \
} while (0)
#define VIDC_REG_60733_FW_STT_ADDR_2_BMSK   0xffffffff
#define VIDC_REG_60733_FW_STT_ADDR_2_SHFT            0

#define VIDC_REG_36891_ADDR_3_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000020c)
#define VIDC_REG_36891_ADDR_3_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000020c)
#define VIDC_REG_36891_ADDR_3_RMSK                 0xffffffff
#define VIDC_REG_36891_ADDR_3_SHFT                          0
#define VIDC_REG_36891_ADDR_3_IN                   \
	in_dword_masked(VIDC_REG_36891_ADDR_3_ADDR,  \
			VIDC_REG_36891_ADDR_3_RMSK)
#define VIDC_REG_36891_ADDR_3_INM(m)               \
	in_dword_masked(VIDC_REG_36891_ADDR_3_ADDR,  m)
#define VIDC_REG_36891_ADDR_3_OUT(v)               \
	out_dword(VIDC_REG_36891_ADDR_3_ADDR, v)
#define VIDC_REG_36891_ADDR_3_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_36891_ADDR_3_ADDR, m, v, \
			VIDC_REG_36891_ADDR_3_IN); \
} while (0)
#define VIDC_REG_36891_ADDR_3_FW_STT_ADDR_3_BMSK   0xffffffff
#define VIDC_REG_36891_ADDR_3_FW_STT_ADDR_3_SHFT            0

#define VIDC_REG_683268_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000210)
#define VIDC_REG_683268_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000210)
#define VIDC_REG_683268_RMSK                 0xffffffff
#define VIDC_REG_683268_SHFT                          0
#define VIDC_REG_683268_IN                   \
	in_dword_masked(VIDC_REG_683268_ADDR,  \
			VIDC_REG_683268_RMSK)
#define VIDC_REG_683268_INM(m)               \
	in_dword_masked(VIDC_REG_683268_ADDR,  m)
#define VIDC_REG_683268_OUT(v)               \
	out_dword(VIDC_REG_683268_ADDR, v)
#define VIDC_REG_683268_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_683268_ADDR, m, v, \
			VIDC_REG_683268_IN); \
} while (0)
#define VIDC_REG_683268_FW_STT_ADDR_4_BMSK   0xffffffff
#define VIDC_REG_683268_FW_STT_ADDR_4_SHFT            0

#define VIDC_REG_382641_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000214)
#define VIDC_REG_382641_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000214)
#define VIDC_REG_382641_RMSK                 0xffffffff
#define VIDC_REG_382641_SHFT                          0
#define VIDC_REG_382641_IN                   \
	in_dword_masked(VIDC_REG_382641_ADDR,  \
			VIDC_REG_382641_RMSK)
#define VIDC_REG_382641_INM(m)               \
	in_dword_masked(VIDC_REG_382641_ADDR,  m)
#define VIDC_REG_382641_OUT(v)               \
	out_dword(VIDC_REG_382641_ADDR, v)
#define VIDC_REG_382641_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_382641_ADDR, m, v, \
			VIDC_REG_382641_IN); \
} while (0)
#define VIDC_REG_382641_FW_STT_ADDR_5_BMSK   0xffffffff
#define VIDC_REG_382641_FW_STT_ADDR_5_SHFT            0

#define VIDC_REG_36891_ADDR_6_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000218)
#define VIDC_REG_36891_ADDR_6_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000218)
#define VIDC_REG_36891_ADDR_6_RMSK                 0xffffffff
#define VIDC_REG_36891_ADDR_6_SHFT                          0
#define VIDC_REG_36891_ADDR_6_IN                   \
	in_dword_masked(VIDC_REG_36891_ADDR_6_ADDR,  \
			VIDC_REG_36891_ADDR_6_RMSK)
#define VIDC_REG_36891_ADDR_6_INM(m)               \
	in_dword_masked(VIDC_REG_36891_ADDR_6_ADDR,  m)
#define VIDC_REG_36891_ADDR_6_OUT(v)               \
	out_dword(VIDC_REG_36891_ADDR_6_ADDR, v)
#define VIDC_REG_36891_ADDR_6_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_36891_ADDR_6_ADDR, m, v, \
			VIDC_REG_36891_ADDR_6_IN); \
} while (0)
#define VIDC_REG_36891_ADDR_6_FW_STT_ADDR_6_BMSK   0xffffffff
#define VIDC_REG_36891_ADDR_6_FW_STT_ADDR_6_SHFT            0

#define VIDC_REG_385703_ADDR                  \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000230)
#define VIDC_REG_385703_PHYS                  \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000230)
#define VIDC_REG_385703_RMSK                  0xffffffff
#define VIDC_REG_385703_SHFT                           0
#define VIDC_REG_385703_IN                    \
	in_dword_masked(VIDC_REG_385703_ADDR,  \
			VIDC_REG_385703_RMSK)
#define VIDC_REG_385703_INM(m)                \
	in_dword_masked(VIDC_REG_385703_ADDR,  m)
#define VIDC_REG_385703_OUT(v)                \
	out_dword(VIDC_REG_385703_ADDR, v)
#define VIDC_REG_385703_OUTM(m, v)             \
do { \
	out_dword_masked_ns(VIDC_REG_385703_ADDR, m, v, \
			VIDC_REG_385703_IN); \
} while (0)
#define VIDC_REG_541626_ADDR_BMSK     0xffffffff
#define VIDC_REG_541626_ADDR_SHFT              0

#define VIDC_REG_270065_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000234)
#define VIDC_REG_270065_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000234)
#define VIDC_REG_270065_RMSK                   0xffffffff
#define VIDC_REG_270065_SHFT                            0
#define VIDC_REG_270065_IN                     \
	in_dword_masked(VIDC_REG_270065_ADDR,  \
			VIDC_REG_270065_RMSK)
#define VIDC_REG_270065_INM(m)                 \
	in_dword_masked(VIDC_REG_270065_ADDR,  m)
#define VIDC_REG_270065_OUT(v)                 \
	out_dword(VIDC_REG_270065_ADDR, v)
#define VIDC_REG_270065_OUTM(m, v)              \
do { \
	out_dword_masked_ns(VIDC_REG_270065_ADDR, m, v, \
			VIDC_REG_270065_IN); \
} while (0)
#define VIDC_REG_270065_DB_STT_ADDR_BMSK       0xffffffff
#define VIDC_REG_270065_DB_STT_ADDR_SHFT                0

#define VIDC_REG_582300_ADDR                       \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000300)
#define VIDC_REG_582300_PHYS                       \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000300)
#define VIDC_REG_582300_RMSK                           0xff1f
#define VIDC_REG_582300_SHFT                                0
#define VIDC_REG_582300_IN                         \
	in_dword_masked(VIDC_REG_582300_ADDR,  VIDC_REG_582300_RMSK)
#define VIDC_REG_582300_INM(m)                     \
	in_dword_masked(VIDC_REG_582300_ADDR,  m)
#define VIDC_REG_582300_OUT(v)                     \
	out_dword(VIDC_REG_582300_ADDR, v)
#define VIDC_REG_582300_OUTM(m, v)                  \
do { \
	out_dword_masked_ns(VIDC_REG_582300_ADDR, m, v, \
			VIDC_REG_582300_IN); \
} while (0)
#define VIDC_REG_582300_LEVEL_BMSK                     0xff00
#define VIDC_REG_582300_LEVEL_SHFT                        0x8
#define VIDC_REG_582300_PROFILE_BMSK                     0x1f
#define VIDC_REG_582300_PROFILE_SHFT                        0

#define VIDC_REG_106897_ADDR                      \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000308)
#define VIDC_REG_106897_PHYS                      \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000308)
#define VIDC_REG_106897_RMSK                          0xffff
#define VIDC_REG_106897_SHFT                               0
#define VIDC_REG_106897_IN                        \
	in_dword_masked(VIDC_REG_106897_ADDR,  VIDC_REG_106897_RMSK)
#define VIDC_REG_106897_INM(m)                    \
	in_dword_masked(VIDC_REG_106897_ADDR,  m)
#define VIDC_REG_106897_OUT(v)                    \
	out_dword(VIDC_REG_106897_ADDR, v)
#define VIDC_REG_106897_OUTM(m, v)                 \
do { \
	out_dword_masked_ns(VIDC_REG_106897_ADDR, m, v, \
			VIDC_REG_106897_IN); \
} while (0)
#define VIDC_REG_106897_I_PERIOD_BMSK                 0xffff
#define VIDC_REG_106897_I_PERIOD_SHFT                      0

#define VIDC_REG_923745_ADDR               \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000310)
#define VIDC_REG_923745_PHYS               \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000310)
#define VIDC_REG_923745_RMSK                      0xd
#define VIDC_REG_923745_SHFT                        0
#define VIDC_REG_923745_IN                 \
	in_dword_masked(VIDC_REG_923745_ADDR,  \
			VIDC_REG_923745_RMSK)
#define VIDC_REG_923745_INM(m)             \
	in_dword_masked(VIDC_REG_923745_ADDR,  m)
#define VIDC_REG_923745_OUT(v)             \
	out_dword(VIDC_REG_923745_ADDR, v)
#define VIDC_REG_923745_OUTM(m, v)          \
do { \
	out_dword_masked_ns(VIDC_REG_923745_ADDR, m, v, \
			VIDC_REG_923745_IN); \
} while (0)
#define VIDC_REG_923745_FIXED_NUMBER_BMSK         0xc
#define VIDC_REG_923745_FIXED_NUMBER_SHFT         0x2
#define VIDC_REG_923745_ENTROPY_SEL_BMSK          0x1
#define VIDC_REG_923745_ENTROPY_SEL_SHFT            0

#define VIDC_REG_19004_ADDR            \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000314)
#define VIDC_REG_19004_PHYS            \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000314)
#define VIDC_REG_19004_RMSK                 0xfff
#define VIDC_REG_19004_SHFT                     0
#define VIDC_REG_19004_IN              \
	in_dword_masked(VIDC_REG_19004_ADDR,  \
			VIDC_REG_19004_RMSK)
#define VIDC_REG_19004_INM(m)          \
	in_dword_masked(VIDC_REG_19004_ADDR,  m)
#define VIDC_REG_19004_OUT(v)          \
	out_dword(VIDC_REG_19004_ADDR, v)
#define VIDC_REG_19004_OUTM(m, v)       \
do { \
	out_dword_masked_ns(VIDC_REG_19004_ADDR, m, v, \
			VIDC_REG_19004_IN); \
} while (0)
#define VIDC_REG_19004_SLICE_ALPHA_C0_OFFSET_DIV2_BMSK      \
	0xf80
#define VIDC_REG_19004_SLICE_ALPHA_C0_OFFSET_DIV2_SHFT        \
	0x7
#define VIDC_REG_19004_SLICE_BETA_OFFSET_DIV2_BMSK       0x7c
#define VIDC_REG_19004_SLICE_BETA_OFFSET_DIV2_SHFT        0x2
#define \
	\
VIDC_REG_19004_DISABLE_DEBLOCKING_FILTER_IDC_BMSK        0x3
#define \
	\
VIDC_REG_19004_DISABLE_DEBLOCKING_FILTER_IDC_SHFT          0

#define VIDC_REG_201982_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000318)
#define VIDC_REG_201982_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000318)
#define VIDC_REG_201982_RMSK                          0x1
#define VIDC_REG_201982_SHFT                            0
#define VIDC_REG_201982_IN                     \
	in_dword_masked(VIDC_REG_201982_ADDR,  \
			VIDC_REG_201982_RMSK)
#define VIDC_REG_201982_INM(m)                 \
	in_dword_masked(VIDC_REG_201982_ADDR,  m)
#define VIDC_REG_201982_OUT(v)                 \
	out_dword(VIDC_REG_201982_ADDR, v)
#define VIDC_REG_201982_OUTM(m, v)              \
do { \
	out_dword_masked_ns(VIDC_REG_201982_ADDR, m, v, \
			VIDC_REG_201982_IN); \
} while (0)
#define VIDC_REG_201982_SHORT_HD_ON_BMSK              0x1
#define VIDC_REG_201982_SHORT_HD_ON_SHFT                0

#define VIDC_REG_724140_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000031c)
#define VIDC_REG_724140_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000031c)
#define VIDC_REG_724140_RMSK                           0x1
#define VIDC_REG_724140_SHFT                             0
#define VIDC_REG_724140_IN                      \
	in_dword_masked(VIDC_REG_724140_ADDR,  \
			VIDC_REG_724140_RMSK)
#define VIDC_REG_724140_INM(m)                  \
	in_dword_masked(VIDC_REG_724140_ADDR,  m)
#define VIDC_REG_724140_OUT(v)                  \
	out_dword(VIDC_REG_724140_ADDR, v)
#define VIDC_REG_724140_OUTM(m, v)               \
do { \
	out_dword_masked_ns(VIDC_REG_724140_ADDR, m, v, \
			VIDC_REG_724140_IN); \
} while (0)
#define VIDC_REG_724140_MSLICE_ENA_BMSK                0x1
#define VIDC_REG_724140_MSLICE_ENA_SHFT                  0

#define VIDC_REG_176761_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000320)
#define VIDC_REG_176761_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000320)
#define VIDC_REG_176761_RMSK                           0x3
#define VIDC_REG_176761_SHFT                             0
#define VIDC_REG_176761_IN                      \
	in_dword_masked(VIDC_REG_176761_ADDR,  \
			VIDC_REG_176761_RMSK)
#define VIDC_REG_176761_INM(m)                  \
	in_dword_masked(VIDC_REG_176761_ADDR,  m)
#define VIDC_REG_176761_OUT(v)                  \
	out_dword(VIDC_REG_176761_ADDR, v)
#define VIDC_REG_176761_OUTM(m, v)               \
do { \
	out_dword_masked_ns(VIDC_REG_176761_ADDR, m, v, \
			VIDC_REG_176761_IN); \
} while (0)
#define VIDC_REG_176761_MSLICE_SEL_BMSK                0x3
#define VIDC_REG_176761_MSLICE_SEL_SHFT                  0

#define VIDC_REG_471408_ADDR                     \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000324)
#define VIDC_REG_471408_PHYS                     \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000324)
#define VIDC_REG_471408_RMSK                     0xffffffff
#define VIDC_REG_471408_SHFT                              0
#define VIDC_REG_471408_IN                       \
	in_dword_masked(VIDC_REG_471408_ADDR,  VIDC_REG_471408_RMSK)
#define VIDC_REG_471408_INM(m)                   \
	in_dword_masked(VIDC_REG_471408_ADDR,  m)
#define VIDC_REG_471408_OUT(v)                   \
	out_dword(VIDC_REG_471408_ADDR, v)
#define VIDC_REG_471408_OUTM(m, v)                \
do { \
	out_dword_masked_ns(VIDC_REG_471408_ADDR, m, v, \
			VIDC_REG_471408_IN); \
} while (0)
#define VIDC_REG_471408_MSLICE_MB_BMSK           0xffffffff
#define VIDC_REG_471408_MSLICE_MB_SHFT                    0

#define VIDC_REG_530212_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000328)
#define VIDC_REG_530212_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000328)
#define VIDC_REG_530212_RMSK                   0xffffffff
#define VIDC_REG_530212_SHFT                            0
#define VIDC_REG_530212_IN                     \
	in_dword_masked(VIDC_REG_530212_ADDR,  \
			VIDC_REG_530212_RMSK)
#define VIDC_REG_530212_INM(m)                 \
	in_dword_masked(VIDC_REG_530212_ADDR,  m)
#define VIDC_REG_530212_OUT(v)                 \
	out_dword(VIDC_REG_530212_ADDR, v)
#define VIDC_REG_530212_OUTM(m, v)              \
do { \
	out_dword_masked_ns(VIDC_REG_530212_ADDR, m, v, \
			VIDC_REG_530212_IN); \
} while (0)
#define VIDC_REG_530212_MSLICE_BYTE_BMSK       0xffffffff
#define VIDC_REG_530212_MSLICE_BYTE_SHFT                0

#define VIDC_REG_759828_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000400)
#define VIDC_REG_759828_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000400)
#define VIDC_REG_759828_RMSK                 0xffffffff
#define VIDC_REG_759828_SHFT                          0
#define VIDC_REG_759828_IN                   \
	in_dword_masked(VIDC_REG_759828_ADDR,  \
			VIDC_REG_759828_RMSK)
#define VIDC_REG_759828_INM(m)               \
	in_dword_masked(VIDC_REG_759828_ADDR,  m)
#define VIDC_REG_759828_DISPLAY_Y_ADR_BMSK   0xffffffff
#define VIDC_REG_759828_DISPLAY_Y_ADR_SHFT            0

#define VIDC_REG_186653_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000404)
#define VIDC_REG_186653_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000404)
#define VIDC_REG_186653_RMSK                 0xffffffff
#define VIDC_REG_186653_SHFT                          0
#define VIDC_REG_186653_IN                   \
	in_dword_masked(VIDC_REG_186653_ADDR,  \
			VIDC_REG_186653_RMSK)
#define VIDC_REG_186653_INM(m)               \
	in_dword_masked(VIDC_REG_186653_ADDR,  m)
#define VIDC_REG_186653_DISPLAY_C_ADR_BMSK   0xffffffff
#define VIDC_REG_186653_DISPLAY_C_ADR_SHFT            0

#define VIDC_REG_380424_ADDR                \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000408)
#define VIDC_REG_380424_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000408)
#define VIDC_REG_380424_RMSK                      0x3f
#define VIDC_REG_380424_SHFT                         0
#define VIDC_REG_380424_IN                  \
	in_dword_masked(VIDC_REG_380424_ADDR,  \
			VIDC_REG_380424_RMSK)
#define VIDC_REG_380424_INM(m)              \
	in_dword_masked(VIDC_REG_380424_ADDR,  m)
#define VIDC_REG_380424_DISPLAY_STATUS_BMSK       0x3f
#define VIDC_REG_380424_DISPLAY_STATUS_SHFT          0

#define VIDC_REG_828425_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000040c)
#define VIDC_REG_828425_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000040c)
#define VIDC_REG_828425_RMSK                          0x1
#define VIDC_REG_828425_SHFT                            0
#define VIDC_REG_828425_IN                     \
	in_dword_masked(VIDC_REG_828425_ADDR,  \
			VIDC_REG_828425_RMSK)
#define VIDC_REG_828425_INM(m)                 \
	in_dword_masked(VIDC_REG_828425_ADDR,  m)
#define VIDC_REG_828425_HEADER_DONE_BMSK              0x1
#define VIDC_REG_828425_HEADER_DONE_SHFT                0

#define VIDC_REG_218030_ADDR                     \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000410)
#define VIDC_REG_218030_PHYS                     \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000410)
#define VIDC_REG_218030_RMSK                     0xffffffff
#define VIDC_REG_218030_SHFT                              0
#define VIDC_REG_218030_IN                       \
	in_dword_masked(VIDC_REG_218030_ADDR,  VIDC_REG_218030_RMSK)
#define VIDC_REG_218030_INM(m)                   \
	in_dword_masked(VIDC_REG_218030_ADDR,  m)
#define VIDC_REG_218030_FRAME_NUM_BMSK           0xffffffff
#define VIDC_REG_218030_FRAME_NUM_SHFT                    0

#define VIDC_REG_551104_ADDR              \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000414)
#define VIDC_REG_551104_PHYS              \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000414)
#define VIDC_REG_551104_RMSK              0xffffffff
#define VIDC_REG_551104_SHFT                       0
#define VIDC_REG_551104_IN                \
	in_dword_masked(VIDC_REG_551104_ADDR,  \
			VIDC_REG_551104_RMSK)
#define VIDC_REG_551104_INM(m)            \
	in_dword_masked(VIDC_REG_551104_ADDR,  m)
#define VIDC_REG_551104_DBG_INFO_OUTPUT0_BMSK 0xffffffff
#define VIDC_REG_551104_DBG_INFO_OUTPUT0_SHFT          0

#define VIDC_REG_929721_ADDR              \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000418)
#define VIDC_REG_929721_PHYS              \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000418)
#define VIDC_REG_929721_RMSK              0xffffffff
#define VIDC_REG_929721_SHFT                       0
#define VIDC_REG_929721_IN                \
	in_dword_masked(VIDC_REG_929721_ADDR,  \
			VIDC_REG_929721_RMSK)
#define VIDC_REG_929721_INM(m)            \
	in_dword_masked(VIDC_REG_929721_ADDR,  m)
#define VIDC_REG_929721_DBG_INFO_OUTPUT1_BMSK 0xffffffff
#define VIDC_REG_929721_DBG_INFO_OUTPUT1_SHFT          0

#define VIDC_REG_749215_ADDR                       \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000500)
#define VIDC_REG_749215_PHYS                       \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000500)
#define VIDC_REG_749215_RMSK                              0x1
#define VIDC_REG_749215_SHFT                                0
#define VIDC_REG_749215_IN                         \
	in_dword_masked(VIDC_REG_749215_ADDR,  VIDC_REG_749215_RMSK)
#define VIDC_REG_749215_INM(m)                     \
	in_dword_masked(VIDC_REG_749215_ADDR,  m)
#define VIDC_REG_749215_OUT(v)                     \
	out_dword(VIDC_REG_749215_ADDR, v)
#define VIDC_REG_749215_OUTM(m, v)                  \
do { \
	out_dword_masked_ns(VIDC_REG_749215_ADDR, m, v, \
			VIDC_REG_749215_IN); \
} while (0)
#define VIDC_REG_749215_INT_OFF_BMSK                      0x1
#define VIDC_REG_749215_INT_OFF_SHFT                        0

#define VIDC_REG_170047_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000504)
#define VIDC_REG_170047_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000504)
#define VIDC_REG_170047_RMSK                        0x1
#define VIDC_REG_170047_SHFT                          0
#define VIDC_REG_170047_IN                   \
	in_dword_masked(VIDC_REG_170047_ADDR,  \
			VIDC_REG_170047_RMSK)
#define VIDC_REG_170047_INM(m)               \
	in_dword_masked(VIDC_REG_170047_ADDR,  m)
#define VIDC_REG_170047_OUT(v)               \
	out_dword(VIDC_REG_170047_ADDR, v)
#define VIDC_REG_170047_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_170047_ADDR, m, v, \
			VIDC_REG_170047_IN); \
} while (0)
#define VIDC_REG_170047_INT_PULSE_SEL_BMSK          0x1
#define VIDC_REG_170047_INT_PULSE_SEL_SHFT            0

#define VIDC_REG_908806_ADDR                \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000508)
#define VIDC_REG_908806_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000508)
#define VIDC_REG_908806_RMSK                       0x1
#define VIDC_REG_908806_SHFT                         0
#define VIDC_REG_908806_IN                  \
	in_dword_masked(VIDC_REG_908806_ADDR,  \
			VIDC_REG_908806_RMSK)
#define VIDC_REG_908806_INM(m)              \
	in_dword_masked(VIDC_REG_908806_ADDR,  m)
#define VIDC_REG_908806_OUT(v)              \
	out_dword(VIDC_REG_908806_ADDR, v)
#define VIDC_REG_908806_OUTM(m, v)           \
do { \
	out_dword_masked_ns(VIDC_REG_908806_ADDR, m, v, \
			VIDC_REG_908806_IN); \
} while (0)
#define VIDC_REG_908806_INT_DONE_CLEAR_BMSK        0x1
#define VIDC_REG_908806_INT_DONE_CLEAR_SHFT          0

#define VIDC_REG_818037_ADDR                \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000050c)
#define VIDC_REG_818037_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000050c)
#define VIDC_REG_818037_RMSK                       0x1
#define VIDC_REG_818037_SHFT                         0
#define VIDC_REG_818037_IN                  \
	in_dword_masked(VIDC_REG_818037_ADDR,  \
			VIDC_REG_818037_RMSK)
#define VIDC_REG_818037_INM(m)              \
	in_dword_masked(VIDC_REG_818037_ADDR,  m)
#define VIDC_REG_818037_OPERATION_DONE_BMSK        0x1
#define VIDC_REG_818037_OPERATION_DONE_SHFT          0

#define VIDC_REG_50416_ADDR                       \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000510)
#define VIDC_REG_50416_PHYS                       \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000510)
#define VIDC_REG_50416_RMSK                              0x1
#define VIDC_REG_50416_SHFT                                0
#define VIDC_REG_50416_IN                         \
	in_dword_masked(VIDC_REG_50416_ADDR,  VIDC_REG_50416_RMSK)
#define VIDC_REG_50416_INM(m)                     \
	in_dword_masked(VIDC_REG_50416_ADDR,  m)
#define VIDC_REG_50416_FW_DONE_BMSK                      0x1
#define VIDC_REG_50416_FW_DONE_SHFT                        0

#define VIDC_REG_202377_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000514)
#define VIDC_REG_202377_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000514)
#define VIDC_REG_202377_RMSK                         0x1f8
#define VIDC_REG_202377_SHFT                             0
#define VIDC_REG_202377_IN                      \
	in_dword_masked(VIDC_REG_202377_ADDR,  \
			VIDC_REG_202377_RMSK)
#define VIDC_REG_202377_INM(m)                  \
	in_dword_masked(VIDC_REG_202377_ADDR,  m)
#define VIDC_REG_202377_FRAME_DONE_STAT_BMSK         0x100
#define VIDC_REG_202377_FRAME_DONE_STAT_SHFT           0x8
#define VIDC_REG_202377_DMA_DONE_STAT_BMSK            0x80
#define VIDC_REG_202377_DMA_DONE_STAT_SHFT             0x7
#define VIDC_REG_202377_HEADER_DONE_STAT_BMSK         0x40
#define VIDC_REG_202377_HEADER_DONE_STAT_SHFT          0x6
#define VIDC_REG_202377_FW_DONE_STAT_BMSK             0x20
#define VIDC_REG_202377_FW_DONE_STAT_SHFT              0x5
#define VIDC_REG_202377_OPERATION_FAILED_BMSK         0x10
#define VIDC_REG_202377_OPERATION_FAILED_SHFT          0x4
#define VIDC_REG_202377_STREAM_HDR_CHANGED_BMSK        0x8
#define VIDC_REG_202377_STREAM_HDR_CHANGED_SHFT        0x3

#define VIDC_REG_787282_ADDR                \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000518)
#define VIDC_REG_787282_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000518)
#define VIDC_REG_787282_RMSK                     0x1fa
#define VIDC_REG_787282_SHFT                         0
#define VIDC_REG_787282_IN                  \
	in_dword_masked(VIDC_REG_787282_ADDR,  \
			VIDC_REG_787282_RMSK)
#define VIDC_REG_787282_INM(m)              \
	in_dword_masked(VIDC_REG_787282_ADDR,  m)
#define VIDC_REG_787282_OUT(v)              \
	out_dword(VIDC_REG_787282_ADDR, v)
#define VIDC_REG_787282_OUTM(m, v)           \
do { \
	out_dword_masked_ns(VIDC_REG_787282_ADDR, m, v, \
			VIDC_REG_787282_IN); \
} while (0)
#define VIDC_REG_787282_FRAME_DONE_ENABLE_BMSK      0x100
#define VIDC_REG_787282_FRAME_DONE_ENABLE_SHFT        0x8
#define VIDC_REG_787282_DMA_DONE_ENABLE_BMSK       0x80
#define VIDC_REG_787282_DMA_DONE_ENABLE_SHFT        0x7
#define VIDC_REG_787282_HEADER_DONE_ENABLE_BMSK       0x40
#define VIDC_REG_787282_HEADER_DONE_ENABLE_SHFT        0x6
#define VIDC_REG_787282_FW_DONE_ENABLE_BMSK       0x20
#define VIDC_REG_787282_FW_DONE_ENABLE_SHFT        0x5
#define VIDC_REG_787282_OPERATION_FAILED_ENABLE_BMSK       0x10
#define VIDC_REG_787282_OPERATION_FAILED_ENABLE_SHFT        0x4
#define VIDC_REG_787282_STREAM_HDR_CHANGED_ENABLE_BMSK        0x8
#define VIDC_REG_787282_STREAM_HDR_CHANGED_ENABLE_SHFT        0x3
#define VIDC_REG_787282_BUFFER_FULL_ENABLE_BMSK        0x2
#define VIDC_REG_787282_BUFFER_FULL_ENABLE_SHFT        0x1

#define VIDC_REG_137224_ADDR                \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000600)
#define VIDC_REG_137224_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000600)
#define VIDC_REG_137224_RMSK                       0x3
#define VIDC_REG_137224_SHFT                         0
#define VIDC_REG_137224_IN                  \
	in_dword_masked(VIDC_REG_137224_ADDR,  \
			VIDC_REG_137224_RMSK)
#define VIDC_REG_137224_INM(m)              \
	in_dword_masked(VIDC_REG_137224_ADDR,  m)
#define VIDC_REG_137224_OUT(v)              \
	out_dword(VIDC_REG_137224_ADDR, v)
#define VIDC_REG_137224_OUTM(m, v)           \
do { \
	out_dword_masked_ns(VIDC_REG_137224_ADDR, m, v, \
			VIDC_REG_137224_IN); \
} while (0)
#define VIDC_REG_137224_TILE_MODE_BMSK             0x3
#define VIDC_REG_137224_TILE_MODE_SHFT               0

#define VIDC_REG_813721_ADDR                \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000800)
#define VIDC_REG_813721_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000800)
#define VIDC_REG_813721_RMSK                0xffffffff
#define VIDC_REG_813721_SHFT                         0
#define VIDC_REG_813721_IN                  \
	in_dword_masked(VIDC_REG_813721_ADDR,  \
			VIDC_REG_813721_RMSK)
#define VIDC_REG_813721_INM(m)              \
	in_dword_masked(VIDC_REG_813721_ADDR,  m)
#define VIDC_REG_813721_OUT(v)              \
	out_dword(VIDC_REG_813721_ADDR, v)
#define VIDC_REG_813721_OUTM(m, v)           \
do { \
	out_dword_masked_ns(VIDC_REG_813721_ADDR, m, v, \
			VIDC_REG_813721_IN); \
} while (0)
#define VIDC_REG_813721_ENC_CUR_Y_ADDR_BMSK 0xffffffff
#define VIDC_REG_813721_ENC_CUR_Y_ADDR_SHFT          0

#define VIDC_REG_327840_ADDR_ADDR                \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000804)
#define VIDC_REG_327840_ADDR_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000804)
#define VIDC_REG_327840_ADDR_RMSK                0xffffffff
#define VIDC_REG_327840_ADDR_SHFT                         0
#define VIDC_REG_327840_ADDR_IN                  \
	in_dword_masked(VIDC_REG_327840_ADDR_ADDR,  \
			VIDC_REG_327840_ADDR_RMSK)
#define VIDC_REG_327840_ADDR_INM(m)              \
	in_dword_masked(VIDC_REG_327840_ADDR_ADDR,  m)
#define VIDC_REG_327840_ADDR_OUT(v)              \
	out_dword(VIDC_REG_327840_ADDR_ADDR, v)
#define VIDC_REG_327840_ADDR_OUTM(m, v)           \
do { \
	out_dword_masked_ns(VIDC_REG_327840_ADDR_ADDR, m, v, \
			VIDC_REG_327840_ADDR_IN); \
} while (0)
#define VIDC_REG_327840_ADDR_ENC_CUR_C_ADDR_BMSK 0xffffffff
#define VIDC_REG_327840_ADDR_ENC_CUR_C_ADDR_SHFT          0

#define VIDC_REG_578009_ADDR_ADDR                  \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000080c)
#define VIDC_REG_578009_ADDR_PHYS                  \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000080c)
#define VIDC_REG_578009_ADDR_RMSK                  0xffffffff
#define VIDC_REG_578009_ADDR_SHFT                           0
#define VIDC_REG_578009_ADDR_IN                    \
	in_dword_masked(VIDC_REG_578009_ADDR_ADDR,  \
			VIDC_REG_578009_ADDR_RMSK)
#define VIDC_REG_578009_ADDR_INM(m)                \
	in_dword_masked(VIDC_REG_578009_ADDR_ADDR,  m)
#define VIDC_REG_578009_ADDR_OUT(v)                \
	out_dword(VIDC_REG_578009_ADDR_ADDR, v)
#define VIDC_REG_578009_ADDR_OUTM(m, v)             \
do { \
	out_dword_masked_ns(VIDC_REG_578009_ADDR_ADDR, m, v, \
			VIDC_REG_578009_ADDR_IN); \
} while (0)
#define VIDC_REG_578009_ADDR_ENC_DPB_ADR_BMSK      0xffffffff
#define VIDC_REG_578009_ADDR_ENC_DPB_ADR_SHFT               0

#define VIDC_REG_658259_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000810)
#define VIDC_REG_658259_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000810)
#define VIDC_REG_658259_RMSK                         0xfff
#define VIDC_REG_658259_SHFT                             0
#define VIDC_REG_658259_IN                      \
	in_dword_masked(VIDC_REG_658259_ADDR,  \
			VIDC_REG_658259_RMSK)
#define VIDC_REG_658259_INM(m)                  \
	in_dword_masked(VIDC_REG_658259_ADDR,  m)
#define VIDC_REG_658259_OUT(v)                  \
	out_dword(VIDC_REG_658259_ADDR, v)
#define VIDC_REG_658259_OUTM(m, v)               \
do { \
	out_dword_masked_ns(VIDC_REG_658259_ADDR, m, v, \
			VIDC_REG_658259_IN); \
} while (0)
#define VIDC_REG_658259_CIR_MB_NUM_BMSK              0xfff
#define VIDC_REG_658259_CIR_MB_NUM_SHFT                  0

#define VIDC_REG_774590_ADDR                  \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000900)
#define VIDC_REG_774590_PHYS                  \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000900)
#define VIDC_REG_774590_RMSK                  0xffffffff
#define VIDC_REG_774590_SHFT                           0
#define VIDC_REG_774590_IN                    \
	in_dword_masked(VIDC_REG_774590_ADDR,  \
			VIDC_REG_774590_RMSK)
#define VIDC_REG_774590_INM(m)                \
	in_dword_masked(VIDC_REG_774590_ADDR,  m)
#define VIDC_REG_774590_OUT(v)                \
	out_dword(VIDC_REG_774590_ADDR, v)
#define VIDC_REG_774590_OUTM(m, v)             \
do { \
	out_dword_masked_ns(VIDC_REG_774590_ADDR, m, v, \
			VIDC_REG_774590_IN); \
} while (0)
#define VIDC_REG_774590_DEC_DPB_ADDR_BMSK     0xffffffff
#define VIDC_REG_774590_DEC_DPB_ADDR_SHFT              0

#define VIDC_REG_223538_ADDR_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000904)
#define VIDC_REG_223538_ADDR_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000904)
#define VIDC_REG_223538_ADDR_RMSK                 0xffffffff
#define VIDC_REG_223538_ADDR_SHFT                          0
#define VIDC_REG_223538_ADDR_IN                   \
	in_dword_masked(VIDC_REG_223538_ADDR_ADDR,  \
			VIDC_REG_223538_ADDR_RMSK)
#define VIDC_REG_223538_ADDR_INM(m)               \
	in_dword_masked(VIDC_REG_223538_ADDR_ADDR,  m)
#define VIDC_REG_223538_ADDR_OUT(v)               \
	out_dword(VIDC_REG_223538_ADDR_ADDR, v)
#define VIDC_REG_223538_ADDR_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_223538_ADDR_ADDR, m, v, \
			VIDC_REG_223538_ADDR_IN); \
} while (0)
#define VIDC_REG_223538_ADDR_DPB_COMV_ADDR_BMSK   0xffffffff
#define VIDC_REG_223538_ADDR_DPB_COMV_ADDR_SHFT            0

#define VIDC_REG_46962_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000908)
#define VIDC_REG_46962_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000908)
#define VIDC_REG_46962_RMSK                 0xffffffff
#define VIDC_REG_46962_SHFT                          0
#define VIDC_REG_46962_IN                   \
	in_dword_masked(VIDC_REG_46962_ADDR,  \
			VIDC_REG_46962_RMSK)
#define VIDC_REG_46962_INM(m)               \
	in_dword_masked(VIDC_REG_46962_ADDR,  m)
#define VIDC_REG_46962_OUT(v)               \
	out_dword(VIDC_REG_46962_ADDR, v)
#define VIDC_REG_46962_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_46962_ADDR, m, v, \
			VIDC_REG_46962_IN); \
} while (0)
#define VIDC_REG_344745_ADDR_BMSK   0xffffffff
#define VIDC_REG_344745_ADDR_SHFT            0

#define VIDC_REG_104358_ADDR                      \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x0000090c)
#define VIDC_REG_104358_PHYS                      \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x0000090c)
#define VIDC_REG_104358_RMSK                            0xff
#define VIDC_REG_104358_SHFT                               0
#define VIDC_REG_104358_IN                        \
	in_dword_masked(VIDC_REG_104358_ADDR,  VIDC_REG_104358_RMSK)
#define VIDC_REG_104358_INM(m)                    \
	in_dword_masked(VIDC_REG_104358_ADDR,  m)
#define VIDC_REG_104358_DPB_SIZE_BMSK                   0xff
#define VIDC_REG_104358_DPB_SIZE_SHFT                      0

#define VIDC_REG_934552_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000a00)
#define VIDC_REG_934552_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000a00)
#define VIDC_REG_934552_RMSK                      0x33f
#define VIDC_REG_934552_SHFT                          0
#define VIDC_REG_934552_IN                   \
	in_dword_masked(VIDC_REG_934552_ADDR,  \
			VIDC_REG_934552_RMSK)
#define VIDC_REG_934552_INM(m)               \
	in_dword_masked(VIDC_REG_934552_ADDR,  m)
#define VIDC_REG_934552_OUT(v)               \
	out_dword(VIDC_REG_934552_ADDR, v)
#define VIDC_REG_934552_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_934552_ADDR, m, v, \
			VIDC_REG_934552_IN); \
} while (0)
#define VIDC_REG_934552_FR_RC_EN_BMSK             0x200
#define VIDC_REG_934552_FR_RC_EN_SHFT               0x9
#define VIDC_REG_934552_MB_RC_EN_BMSK             0x100
#define VIDC_REG_934552_MB_RC_EN_SHFT               0x8
#define VIDC_REG_934552_FRAME_QP_BMSK              0x3f
#define VIDC_REG_934552_FRAME_QP_SHFT                 0

#define VIDC_REG_627555_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000a04)
#define VIDC_REG_627555_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000a04)
#define VIDC_REG_627555_RMSK                          0x3f
#define VIDC_REG_627555_SHFT                             0
#define VIDC_REG_627555_IN                      \
	in_dword_masked(VIDC_REG_627555_ADDR,  \
			VIDC_REG_627555_RMSK)
#define VIDC_REG_627555_INM(m)                  \
	in_dword_masked(VIDC_REG_627555_ADDR,  m)
#define VIDC_REG_627555_OUT(v)                  \
	out_dword(VIDC_REG_627555_ADDR, v)
#define VIDC_REG_627555_OUTM(m, v)               \
do { \
	out_dword_masked_ns(VIDC_REG_627555_ADDR, m, v, \
			VIDC_REG_627555_IN); \
} while (0)
#define VIDC_REG_627555_P_FRAME_QP_BMSK               0x3f
#define VIDC_REG_627555_P_FRAME_QP_SHFT                  0

#define VIDC_REG_576121_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000a08)
#define VIDC_REG_576121_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000a08)
#define VIDC_REG_576121_RMSK                   0xffffffff
#define VIDC_REG_576121_SHFT                            0
#define VIDC_REG_576121_IN                     \
	in_dword_masked(VIDC_REG_576121_ADDR,  \
			VIDC_REG_576121_RMSK)
#define VIDC_REG_576121_INM(m)                 \
	in_dword_masked(VIDC_REG_576121_ADDR,  m)
#define VIDC_REG_576121_OUT(v)                 \
	out_dword(VIDC_REG_576121_ADDR, v)
#define VIDC_REG_576121_OUTM(m, v)              \
do { \
	out_dword_masked_ns(VIDC_REG_576121_ADDR, m, v, \
			VIDC_REG_576121_IN); \
} while (0)
#define VIDC_REG_576121_BIT_RATE_BMSK          0xffffffff
#define VIDC_REG_576121_BIT_RATE_SHFT                   0

#define VIDC_REG_880537_ADDR                     \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000a0c)
#define VIDC_REG_880537_PHYS                     \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000a0c)
#define VIDC_REG_880537_RMSK                         0x3f3f
#define VIDC_REG_880537_SHFT                              0
#define VIDC_REG_880537_IN                       \
	in_dword_masked(VIDC_REG_880537_ADDR,  VIDC_REG_880537_RMSK)
#define VIDC_REG_880537_INM(m)                   \
	in_dword_masked(VIDC_REG_880537_ADDR,  m)
#define VIDC_REG_880537_OUT(v)                   \
	out_dword(VIDC_REG_880537_ADDR, v)
#define VIDC_REG_880537_OUTM(m, v)                \
do { \
	out_dword_masked_ns(VIDC_REG_880537_ADDR, m, v, \
			VIDC_REG_880537_IN); \
} while (0)
#define VIDC_REG_880537_MAX_QP_BMSK                  0x3f00
#define VIDC_REG_880537_MAX_QP_SHFT                     0x8
#define VIDC_REG_880537_MIN_QP_BMSK                    0x3f
#define VIDC_REG_880537_MIN_QP_SHFT                       0

#define VIDC_REG_708001_ADDR                      \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000a10)
#define VIDC_REG_708001_PHYS                      \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000a10)
#define VIDC_REG_708001_RMSK                          0xffff
#define VIDC_REG_708001_SHFT                               0
#define VIDC_REG_708001_IN                        \
	in_dword_masked(VIDC_REG_708001_ADDR,  VIDC_REG_708001_RMSK)
#define VIDC_REG_708001_INM(m)                    \
	in_dword_masked(VIDC_REG_708001_ADDR,  m)
#define VIDC_REG_708001_OUT(v)                    \
	out_dword(VIDC_REG_708001_ADDR, v)
#define VIDC_REG_708001_OUTM(m, v)                 \
do { \
	out_dword_masked_ns(VIDC_REG_708001_ADDR, m, v, \
			VIDC_REG_708001_IN); \
} while (0)
#define VIDC_REG_708001_REACT_PARA_BMSK               0xffff
#define VIDC_REG_708001_REACT_PARA_SHFT                    0

#define VIDC_REG_594627_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000a14)
#define VIDC_REG_594627_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000a14)
#define VIDC_REG_594627_RMSK                           0xf
#define VIDC_REG_594627_SHFT                             0
#define VIDC_REG_594627_IN                      \
	in_dword_masked(VIDC_REG_594627_ADDR,  \
			VIDC_REG_594627_RMSK)
#define VIDC_REG_594627_INM(m)                  \
	in_dword_masked(VIDC_REG_594627_ADDR,  m)
#define VIDC_REG_594627_OUT(v)                  \
	out_dword(VIDC_REG_594627_ADDR, v)
#define VIDC_REG_594627_OUTM(m, v)               \
do { \
	out_dword_masked_ns(VIDC_REG_594627_ADDR, m, v, \
			VIDC_REG_594627_IN); \
} while (0)
#define VIDC_REG_594627_DARK_DISABLE_BMSK              0x8
#define VIDC_REG_594627_DARK_DISABLE_SHFT              0x3
#define VIDC_REG_594627_SMOOTH_DISABLE_BMSK            0x4
#define VIDC_REG_594627_SMOOTH_DISABLE_SHFT            0x2
#define VIDC_REG_594627_STATIC_DISABLE_BMSK            0x2
#define VIDC_REG_594627_STATIC_DISABLE_SHFT            0x1
#define VIDC_REG_594627_ACT_DISABLE_BMSK               0x1
#define VIDC_REG_594627_ACT_DISABLE_SHFT                 0

#define VIDC_REG_121939_ADDR                       \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000a18)
#define VIDC_REG_121939_PHYS                       \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000a18)
#define VIDC_REG_121939_RMSK                             0x3f
#define VIDC_REG_121939_SHFT                                0
#define VIDC_REG_121939_IN                         \
	in_dword_masked(VIDC_REG_121939_ADDR,  VIDC_REG_121939_RMSK)
#define VIDC_REG_121939_INM(m)                     \
	in_dword_masked(VIDC_REG_121939_ADDR,  m)
#define VIDC_REG_121939_QP_OUT_BMSK                      0x3f
#define VIDC_REG_121939_QP_OUT_SHFT                         0

#define VIDC_REG_558246_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000b00)
#define VIDC_REG_558246_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000b00)
#define VIDC_REG_558246_RMSK                   0xffffffff
#define VIDC_REG_558246_SHFT                            0
#define VIDC_REG_558246_IN                     \
	in_dword_masked(VIDC_REG_558246_ADDR,  \
			VIDC_REG_558246_RMSK)
#define VIDC_REG_558246_INM(m)                 \
	in_dword_masked(VIDC_REG_558246_ADDR,  m)
#define VIDC_REG_558246_720P_VERSION_BMSK       0xffffffff
#define VIDC_REG_558246_720P_VERSION_SHFT                0

#define VIDC_REG_904540_ADDR                     \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000c00)
#define VIDC_REG_904540_PHYS                     \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000c00)
#define VIDC_REG_904540_RMSK                     0xffffffff
#define VIDC_REG_904540_SHFT                              0
#define VIDC_REG_904540_IN                       \
	in_dword_masked(VIDC_REG_904540_ADDR,  VIDC_REG_904540_RMSK)
#define VIDC_REG_904540_INM(m)                   \
	in_dword_masked(VIDC_REG_904540_ADDR,  m)
#define VIDC_REG_904540_CROP_RIGHT_OFFSET_BMSK   0xffff0000
#define VIDC_REG_904540_CROP_RIGHT_OFFSET_SHFT         0x10
#define VIDC_REG_904540_CROP_LEFT_OFFSET_BMSK        0xffff
#define VIDC_REG_904540_CROP_LEFT_OFFSET_SHFT             0

#define VIDC_REG_663308_ADDR                     \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000c04)
#define VIDC_REG_663308_PHYS                     \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000c04)
#define VIDC_REG_663308_RMSK                     0xffffffff
#define VIDC_REG_663308_SHFT                              0
#define VIDC_REG_663308_IN                       \
	in_dword_masked(VIDC_REG_663308_ADDR,  VIDC_REG_663308_RMSK)
#define VIDC_REG_663308_INM(m)                   \
	in_dword_masked(VIDC_REG_663308_ADDR,  m)
#define VIDC_REG_663308_CROP_BOTTOM_OFFSET_BMSK  0xffff0000
#define VIDC_REG_663308_CROP_BOTTOM_OFFSET_SHFT        0x10
#define VIDC_REG_663308_CROP_TOP_OFFSET_BMSK         0xffff
#define VIDC_REG_663308_CROP_TOP_OFFSET_SHFT              0

#define VIDC_REG_177025_ADDR              \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000c08)
#define VIDC_REG_177025_PHYS              \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000c08)
#define VIDC_REG_177025_RMSK              0xffffffff
#define VIDC_REG_177025_SHFT                       0
#define VIDC_REG_177025_IN                \
	in_dword_masked(VIDC_REG_177025_ADDR,  \
			VIDC_REG_177025_RMSK)
#define VIDC_REG_177025_INM(m)            \
	in_dword_masked(VIDC_REG_177025_ADDR,  m)
#define VIDC_REG_177025_720P_DEC_FRM_SIZE_BMSK 0xffffffff
#define VIDC_REG_177025_720P_DEC_FRM_SIZE_SHFT          0


#define VIDC_REG_469881_ADDR \
		(VIDC_720P_WRAPPER_REG_BASE + 0x00000c0c)
#define VIDC_REG_469881_PHYS \
		(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000c0c)
#define VIDC_REG_469881_RMSK  0xff1f
#define VIDC_REG_469881_SHFT  0
#define VIDC_REG_469881_IN                         \
		in_dword_masked(VIDC_REG_469881_ADDR, \
		VIDC_REG_469881_RMSK)
#define VIDC_REG_469881_INM(m)                     \
		in_dword_masked(VIDC_REG_469881_ADDR, m)
#define VIDC_REG_469881_OUT(v)                     \
		out_dword(VIDC_REG_469881_ADDR, v)
#define VIDC_REG_469881_OUTM(m, v)                  \
		out_dword_masked_ns(VIDC_REG_469881_ADDR,\
		m, v, VIDC_REG_469881_IN); \

#define VIDC_REG_469881_DIS_PIC_LEVEL_BMSK 0xff00
#define VIDC_REG_469881_DIS_PIC_LEVEL_SHFT 0x8
#define VIDC_REG_469881_DISP_PIC_PROFILE_BMSK 0x1f
#define VIDC_REG_469881_DISP_PIC_PROFILE_SHFT 0

#define VIDC_REG_81455_ADDR \
		(VIDC_720P_WRAPPER_REG_BASE      + 0x00000c10)
#define VIDC_REG_81455_PHYS \
		(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000c10)
#define VIDC_REG_81455_RMSK 0xffffffff
#define VIDC_REG_81455_SHFT 0
#define VIDC_REG_81455_IN \
		in_dword_masked(VIDC_REG_81455_ADDR,\
		VIDC_REG_81455_RMSK)
#define VIDC_REG_81455_INM(m) \
		in_dword_masked(VIDC_REG_81455_ADDR, m)
#define VIDC_REG_81455_MIN_DPB_SIZE_BMSK 0xffffffff
#define VIDC_REG_81455_MIN_DPB_SIZE_SHFT 0


#define VIDC_REG_659013_ADDR \
		(VIDC_720P_WRAPPER_REG_BASE + 0x00000c14)
#define VIDC_REG_659013_PHYS \
		(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000c14)
#define VIDC_REG_659013_RMSK 0xffffffff
#define VIDC_REG_659013_SHFT 0
#define VIDC_REG_659013_IN \
		in_dword_masked(VIDC_REG_659013_ADDR,\
		VIDC_REG_659013_RMSK)
#define VIDC_REG_659013_INM(m) \
		in_dword_masked(VIDC_REG_659013_ADDR, m)
#define VIDC_REG_659013_720P_FW_STATUS_BMSK 0xffffffff
#define VIDC_REG_659013_720P_FW_STATUS_SHFT 0


#define VIDC_REG_444141_ADDR \
		(VIDC_720P_WRAPPER_REG_BASE + 0x00000c18)
#define VIDC_REG_444141_PHYS \
		(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000c18)
#define VIDC_REG_444141_RMSK 0xffffffff
#define VIDC_REG_444141_SHFT 0
#define VIDC_REG_444141_IN \
		in_dword_masked(VIDC_REG_444141_ADDR,\
		VIDC_REG_444141_RMSK)
#define VIDC_REG_444141_INM(m) \
		in_dword_masked(VIDC_REG_444141_ADDR, m)
#define VIDC_REG_444141_FREE_LUMA_DPB_BMSK 0xffffffff
#define VIDC_REG_444141_FREE_LUMA_DPB_SHFT 0


#define VIDC_REG_826219_ADDR              \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000d00)
#define VIDC_REG_826219_PHYS              \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000d00)
#define VIDC_REG_826219_RMSK                     0xf
#define VIDC_REG_826219_SHFT                       0
#define VIDC_REG_826219_IN                \
		in_dword_masked(VIDC_REG_826219_ADDR,  \
		VIDC_REG_826219_RMSK)
#define VIDC_REG_826219_INM(m)            \
	in_dword_masked(VIDC_REG_826219_ADDR,  m)
#define VIDC_REG_826219_OUT(v)            \
	out_dword(VIDC_REG_826219_ADDR, v)
#define VIDC_REG_826219_OUTM(m, v)         \
do { \
	out_dword_masked_ns(VIDC_REG_826219_ADDR, m, v, \
			VIDC_REG_826219_IN); \
} while (0)
#define VIDC_REG_826219_COMMAND_TYPE_BMSK        0xf
#define VIDC_REG_826219_COMMAND_TYPE_SHFT          0

#define VIDC_REG_767703_ADDR  \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000d04)
#define VIDC_REG_767703_PHYS  \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000d04)
#define VIDC_REG_767703_RMSK                       0xffffffff
#define VIDC_REG_767703_SHFT                                0
#define VIDC_REG_767703_OUT(v)                     \
	out_dword(VIDC_REG_767703_ADDR, v)
#define VIDC_REG_767703_OUTM(m, v)                  \
	out_dword_masked(VIDC_REG_767703_ADDR, m, v, \
			VID_C_REG_767703_SHADOW)
#define VIDC_REG_767703_ALLOCATED_DB_BUF_SIZE_BMSK 0xffffffff
#define VIDC_REG_767703_ALLOCATED_DB_BUF_SIZE_SHFT          0

#define VIDC_REG_703273_ADDR  \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000d08)
#define VIDC_REG_703273_PHYS  \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000d08)
#define VIDC_REG_703273_RMSK                       \
	0xffffffff
#define \
	\
VIDC_REG_703273_SHFT                                0
#define VIDC_REG_703273_OUT(v)                     \
	out_dword(VIDC_REG_703273_ADDR, v)
#define VIDC_REG_703273_OUTM(m, v)                  \
	out_dword_masked(VIDC_REG_703273_ADDR, m, v, \
			VID_C_REG_703273_SHADOW)
#define VIDC_REG_703273_ALLOCATED_COMV_BUF_SIZE_BMSK \
	0xffffffff
#define \
	\
VIDC_REG_703273_ALLOCATED_COMV_BUF_SIZE_SHFT          0

#define VIDC_REG_571213_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000d14)
#define VIDC_REG_571213_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000d14)
#define VIDC_REG_571213_RMSK                 0xffffffff
#define VIDC_REG_571213_SHFT                          0
#define VIDC_REG_571213_IN                   \
	in_dword_masked(VIDC_REG_571213_ADDR,  \
			VIDC_REG_571213_RMSK)
#define VIDC_REG_571213_INM(m)               \
	in_dword_masked(VIDC_REG_571213_ADDR,  m)
#define VIDC_REG_571213_OUT(v)               \
	out_dword(VIDC_REG_571213_ADDR, v)
#define VIDC_REG_571213_OUTM(m, v)            \
do { \
	out_dword_masked_ns(VIDC_REG_571213_ADDR, m, v, \
			VIDC_REG_571213_IN); \
} while (0)
#define VIDC_REG_571213_FRAME_RATE_BMSK      0xffffffff
#define VIDC_REG_571213_FRAME_RATE_SHFT               0

#define VIDC_REG_688165_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000e00)
#define VIDC_REG_688165_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000e00)
#define VIDC_REG_688165_RMSK                    0xffffffff
#define VIDC_REG_688165_SHFT                             0
#define VIDC_REG_688165_OUT(v)                  \
	out_dword(VIDC_REG_688165_ADDR, v)
#define VIDC_REG_688165_OUTM(m, v)               \
	out_dword_masked(VIDC_REG_688165_ADDR, m, v, \
			VID_C_REG_688165_SHADOW)
#define VIDC_REG_688165_VOP_TIMING_ENABLE_BMSK  0x80000000
#define VIDC_REG_688165_VOP_TIMING_ENABLE_SHFT        0x1f
#define VIDC_REG_688165_VOP_TIME_RESOLUTION_BMSK 0x7fff0000
#define VIDC_REG_688165_VOP_TIME_RESOLUTION_SHFT       0x10
#define VIDC_REG_688165_FRAME_DELTA_BMSK            0xffff
#define VIDC_REG_688165_FRAME_DELTA_SHFT                 0

#define VIDC_REG_684964_ADDR \
		(VIDC_720P_WRAPPER_REG_BASE + 0x00000e04)
#define VIDC_REG_684964_PHYS \
		(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000e04)
#define VIDC_REG_684964_RMSK 0x7f
#define VIDC_REG_684964_SHFT 0
#define VIDC_REG_684964_OUT(v) \
		out_dword(VIDC_REG_684964_ADDR, v)
#define VIDC_REG_684964_OUTM(m, v) \
		out_dword_masked(VIDC_REG_684964_ADDR, m, v, \
		VID_C_REG_684964_SHADOW)
#define VIDC_REG_684964_OUTPUT_ORDER_SETTING_BMSK 0x1
#define VIDC_REG_684964_OUTPUT_ORDER_SETTING_SHFT 0


#define VIDC_REG_842480_ADDR              \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000e08)
#define VIDC_REG_842480_PHYS              \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000e08)
#define VIDC_REG_842480_RMSK                     0x7
#define VIDC_REG_842480_SHFT                       0
#define VIDC_REG_842480_OUT(v)            \
	out_dword(VIDC_REG_842480_ADDR, v)
#define VIDC_REG_842480_OUTM(m, v)         \
	out_dword_masked(VIDC_REG_842480_ADDR, m, v, \
			VID_C_REG_842480_SHADOW)
#define VIDC_REG_842480_RC_BIT_RATE_CHANGE_BMSK        0x4
#define VIDC_REG_842480_RC_BIT_RATE_CHANGE_SHFT        0x2
#define VIDC_REG_842480_RC_FRAME_RATE_CHANGE_BMSK        0x2
#define VIDC_REG_842480_RC_FRAME_RATE_CHANGE_SHFT        0x1
#define VIDC_REG_842480_I_PERIOD_CHANGE_BMSK        0x1
#define VIDC_REG_842480_I_PERIOD_CHANGE_SHFT          0

#define VIDC_REG_221996_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000e10)
#define VIDC_REG_221996_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000e10)
#define VIDC_REG_221996_RMSK                   0xffffffff
#define VIDC_REG_221996_SHFT                            0
#define VIDC_REG_221996_IN                     \
	in_dword_masked(VIDC_REG_221996_ADDR,  \
			VIDC_REG_221996_RMSK)
#define VIDC_REG_221996_INM(m)                 \
	in_dword_masked(VIDC_REG_221996_ADDR,  m)
#define VIDC_REG_221996_MIN_NUM_DPB_BMSK       0xffffffff
#define VIDC_REG_221996_MIN_NUM_DPB_SHFT                0

#define VIDC_REG_830933_ADDR                       \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000e14)
#define VIDC_REG_830933_PHYS                       \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000e14)
#define VIDC_REG_830933_RMSK                       0xffffffff
#define VIDC_REG_830933_SHFT                                0
#define VIDC_REG_830933_OUT(v)                     \
	out_dword(VIDC_REG_830933_ADDR, v)
#define VIDC_REG_830933_OUTM(m, v)                  \
	out_dword_masked(VIDC_REG_830933_ADDR, m, v, \
			VID_C_REG_830933_SHADOW)
#define VIDC_REG_830933_NUM_DPB_BMSK               0xffffffff
#define VIDC_REG_830933_NUM_DPB_SHFT                        0

#define VIDC_REG_396763_ADDR(n)               \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000e18 + 4 * (n))
#define VIDC_REG_396763_PHYS(n)               \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000e18 + 4 * (n))
#define VIDC_REG_396763_RMSK                  0xffffffff
#define VIDC_REG_396763_SHFT                           0
#define VIDC_REG_396763_OUTI(n, v) \
	out_dword(VIDC_REG_396763_ADDR(n), v)
#define VIDC_REG_396763_OUTMI(n, mask, v) \
	out_dword_masked(VIDC_REG_396763_ADDR(n), mask, v, \
			VID_C_REG_396763_SHADOW[n])
#define VIDC_REG_396763_DEC_DPB_BUF_BMSK      0xffffffff
#define VIDC_REG_396763_DEC_DPB_BUF_SHFT               0

#define VIDC_REG_492691_ADDR                \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000e98)
#define VIDC_REG_492691_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000e98)
#define VIDC_REG_492691_RMSK                0xffffffff
#define VIDC_REG_492691_SHFT                         0
#define VIDC_REG_492691_OUT(v)              \
	out_dword(VIDC_REG_492691_ADDR, v)
#define VIDC_REG_492691_OUTM(m, v)           \
	out_dword_masked(VIDC_REG_492691_ADDR, m, v, \
			VID_C_REG_492691_SHADOW)
#define VIDC_REG_492691_RELEASE_BUFFER_BMSK 0xffffffff
#define VIDC_REG_492691_RELEASE_BUFFER_SHFT          0

#define VIDC_REG_71403_ADDR                  \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000e9c)
#define VIDC_REG_71403_PHYS                  \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000e9c)
#define VIDC_REG_71403_RMSK                  0xffffffff
#define VIDC_REG_71403_SHFT                           0
#define VIDC_REG_71403_IN                    \
	in_dword_masked(VIDC_REG_71403_ADDR,  \
			VIDC_REG_71403_RMSK)
#define VIDC_REG_71403_INM(m)                \
	in_dword_masked(VIDC_REG_71403_ADDR,  m)
#define VIDC_REG_71403_ERROR_STATUS_BMSK     0xffffffff
#define VIDC_REG_71403_ERROR_STATUS_SHFT              0

#define VIDC_REG_586901_ADDR        \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ea0)
#define VIDC_REG_586901_PHYS        \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ea0)
#define VIDC_REG_586901_RMSK        0xffffffff
#define VIDC_REG_586901_SHFT                 0
#define VIDC_REG_586901_OUT(v)      \
	out_dword(VIDC_REG_586901_ADDR, v)
#define VIDC_REG_586901_OUTM(m, v)   \
	out_dword_masked(VIDC_REG_586901_ADDR, m, v, \
			VID_C_REG_586901_SHADOW)
#define VIDC_REG_586901_UPPER_UNALIGNED_BACKUP_BMSK \
	0xffffffff
#define \
	\
VIDC_REG_586901_UPPER_UNALIGNED_BACKUP_SHFT          \
0

#define VIDC_REG_37680_ADDR        \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ea4)
#define VIDC_REG_37680_PHYS        \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ea4)
#define VIDC_REG_37680_RMSK        0xffffffff
#define VIDC_REG_37680_SHFT                 0
#define VIDC_REG_37680_OUT(v)      \
	out_dword(VIDC_REG_37680_ADDR, v)
#define VIDC_REG_37680_OUTM(m, v)   \
	out_dword_masked(VIDC_REG_37680_ADDR, m, v, \
			VID_C_REG_37680_SHADOW)
#define VIDC_REG_37680_LOWER_UNALIGNED_BACKUP_BMSK \
	0xffffffff
#define \
	\
VIDC_REG_37680_LOWER_UNALIGNED_BACKUP_SHFT          0

#define VIDC_REG_30219_ADDR            \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ea8)
#define VIDC_REG_30219_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ea8)
#define VIDC_REG_30219_RMSK                0xffffffff
#define VIDC_REG_30219_SHFT                         0
#define VIDC_REG_30219_IN                  \
	in_dword_masked(VIDC_REG_30219_ADDR,  \
			VIDC_REG_30219_RMSK)
#define VIDC_REG_30219_INM(m)              \
	in_dword_masked(VIDC_REG_30219_ADDR,  m)
#define VIDC_REG_30219_GET_FRAME_TAG_TOP_BMSK 0xffffffff
#define VIDC_REG_30219_GET_FRAME_TAG_TOP_SHFT          0

#define VIDC_REG_797110_ADDR               \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000eac)
#define VIDC_REG_797110_PHYS               \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000eac)
#define VIDC_REG_797110_RMSK                        0x1
#define VIDC_REG_797110_SHFT                        0
#define VIDC_REG_797110_OUT(v)             \
	out_dword(VIDC_REG_797110_ADDR, v)
#define VIDC_REG_797110_OUTM(m, v)          \
	out_dword_masked(VIDC_REG_797110_ADDR, m, v, \
			VID_C_REG_797110_SHADOW)
#define VIDC_REG_797110_PUT_EXTRADATA_BMSK         0x1
#define VIDC_REG_797110_PUT_EXTRADATA_SHFT            0

#define VIDC_REG_928824_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000eb0)
#define VIDC_REG_928824_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000eb0)
#define VIDC_REG_928824_RMSK                    0xffffffff
#define VIDC_REG_928824_SHFT                             0
#define VIDC_REG_928824_OUT(v)                  \
	out_dword(VIDC_REG_928824_ADDR, v)
#define VIDC_REG_928824_OUTM(m, v)               \
	out_dword_masked(VIDC_REG_928824_ADDR, m, v, \
			VID_C_REG_928824_SHADOW)
#define VIDC_REG_928824_HEC_PERIOD_BMSK         \
	0xffffffff
#define VIDC_REG_928824_HEC_PERIOD_SHFT                  0

#define VIDC_REG_457489_ADDR \
		(VIDC_720P_WRAPPER_REG_BASE + 0x00000eb4)
#define VIDC_REG_457489_PHYS \
		(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000eb4)
#define VIDC_REG_457489_RMSK 0x3
#define VIDC_REG_457489_SHFT 0
#define VIDC_REG_457489_IN \
		in_dword_masked(VIDC_REG_457489_ADDR,\
		VIDC_REG_457489_RMSK)
#define VIDC_REG_457489_INM(m) \
		in_dword_masked(VIDC_REG_457489_ADDR, m)
#define VIDC_REG_457489_FRAME_TYPE_BMSK 0x3
#define VIDC_REG_457489_FRAME_TYPE_SHFT 0


#define VIDC_REG_389650_ADDR               \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000eb8)
#define VIDC_REG_389650_PHYS               \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000eb8)
#define VIDC_REG_389650_RMSK                      0x1
#define VIDC_REG_389650_SHFT                        0
#define VIDC_REG_389650_IN                 \
	in_dword_masked(VIDC_REG_389650_ADDR,  \
			VIDC_REG_389650_RMSK)
#define VIDC_REG_389650_INM(m)             \
	in_dword_masked(VIDC_REG_389650_ADDR,  m)
#define VIDC_REG_389650_METADATA_STATUS_BMSK        0x1
#define VIDC_REG_389650_METADATA_STATUS_SHFT          0
#define VIDC_REG_974642_ADDR                    \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ebc)
#define VIDC_REG_974642_PHYS                    \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ebc)
#define VIDC_REG_974642_RMSK                           0x7
#define VIDC_REG_974642_SHFT                             0
#define VIDC_REG_974642_IN                      \
	in_dword_masked(VIDC_REG_974642_ADDR,  \
			VIDC_REG_974642_RMSK)
#define VIDC_REG_974642_INM(m)                  \
	in_dword_masked(VIDC_REG_974642_ADDR,  m)
#define VIDC_REG_974642_FRAME_TYPE_BMSK                0x7
#define VIDC_REG_974642_FRAME_TYPE_SHFT                  0

#define VIDC_REG_595068_ADDR        \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ec0)
#define VIDC_REG_595068_PHYS        \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ec0)
#define VIDC_REG_595068_RMSK               0x3
#define VIDC_REG_595068_SHFT                 0
#define VIDC_REG_595068_IN          \
	in_dword_masked(VIDC_REG_595068_ADDR,  \
			VIDC_REG_595068_RMSK)
#define VIDC_REG_595068_INM(m)      \
	in_dword_masked(VIDC_REG_595068_ADDR,  m)
#define VIDC_REG_595068_OPERATION_FAILED_BMSK        0x2
#define VIDC_REG_595068_OPERATION_FAILED_SHFT        0x1
#define VIDC_REG_595068_RESOLUTION_CHANGE_BMSK        0x1
#define VIDC_REG_595068_RESOLUTION_CHANGE_SHFT          0

#define VIDC_REG_4084_ADDR               \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ec4)
#define VIDC_REG_4084_PHYS               \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ec4)
#define VIDC_REG_4084_RMSK                     0x7f
#define VIDC_REG_4084_SHFT                        0
#define VIDC_REG_4084_OUT(v)             \
	out_dword(VIDC_REG_4084_ADDR, v)
#define VIDC_REG_4084_OUTM(m, v)          \
	out_dword_masked(VIDC_REG_4084_ADDR, m, v, \
			VID_C_REG_4084_SHADOW)
#define VIDC_REG_4084_EXTRADATA_ENABLE_BMSK       0x40
#define VIDC_REG_4084_EXTRADATA_ENABLE_SHFT        0x6
#define VIDC_REG_4084_ENC_SLICE_SIZE_ENABLE_BMSK       0x20
#define VIDC_REG_4084_ENC_SLICE_SIZE_ENABLE_SHFT        0x5
#define VIDC_REG_4084_VUI_ENABLE_BMSK          0x10
#define VIDC_REG_4084_VUI_ENABLE_SHFT           0x4
#define VIDC_REG_4084_SEI_NAL_ENABLE_BMSK        0x8
#define VIDC_REG_4084_SEI_NAL_ENABLE_SHFT        0x3
#define VIDC_REG_4084_VC1_PARAM_ENABLE_BMSK        0x4
#define VIDC_REG_4084_VC1_PARAM_ENABLE_SHFT        0x2
#define VIDC_REG_4084_CONCEALED_MB_ENABLE_BMSK        0x2
#define VIDC_REG_4084_CONCEALED_MB_ENABLE_SHFT        0x1
#define VIDC_REG_4084_QP_ENABLE_BMSK            0x1
#define VIDC_REG_4084_QP_ENABLE_SHFT              0
#define VIDC_REG_623236_ADDR               \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ec8)
#define VIDC_REG_623236_PHYS               \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ec8)
#define VIDC_REG_623236_RMSK               0xffff000f
#define VIDC_REG_623236_SHFT                        0
#define VIDC_REG_623236_OUT(v)             \
	out_dword(VIDC_REG_623236_ADDR, v)
#define VIDC_REG_623236_OUTM(m, v)          \
	out_dword_masked(VIDC_REG_623236_ADDR, m, v, \
			VID_C_REG_623236_SHADOW)
#define VIDC_REG_623236_VBV_BUFFER_SIZE_BMSK 0xffff0000
#define VIDC_REG_623236_VBV_BUFFER_SIZE_SHFT       0x10
#define VIDC_REG_623236_FRAME_SKIP_ENABLE_BMSK        0xc
#define VIDC_REG_623236_FRAME_SKIP_ENABLE_SHFT        0x2
#define VIDC_REG_623236_HEC_ENABLE_BMSK           0x2
#define VIDC_REG_623236_HEC_ENABLE_SHFT           0x1
#define VIDC_REG_623236_INSERT_I_FRAME_BMSK        0x1
#define VIDC_REG_623236_INSERT_I_FRAME_SHFT          0

#define VIDC_REG_917177_ADDR        \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ecc)
#define VIDC_REG_917177_PHYS        \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ecc)
#define VIDC_REG_917177_RMSK        0xffffffff
#define VIDC_REG_917177_SHFT                 0
#define VIDC_REG_917177_IN          \
	in_dword_masked(VIDC_REG_917177_ADDR,  \
			VIDC_REG_917177_RMSK)
#define VIDC_REG_917177_INM(m)      \
	in_dword_masked(VIDC_REG_917177_ADDR,  m)
#define VIDC_REG_917177_METADATA_DISPLAY_INDEX_BMSK \
	0xffffffff
#define \
	\
VIDC_REG_917177_METADATA_DISPLAY_INDEX_SHFT          0

#define VIDC_REG_967981_ADDR       \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ed0)
#define VIDC_REG_967981_PHYS       \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ed0)
#define VIDC_REG_967981_RMSK       0xffffffff
#define VIDC_REG_967981_SHFT                0
#define VIDC_REG_967981_OUT(v)     \
	out_dword(VIDC_REG_967981_ADDR, v)
#define VIDC_REG_967981_OUTM(m, v)  \
	out_dword_masked(VIDC_REG_967981_ADDR, m, v, \
			VID_C_REG_967981_SHADOW)
#define VIDC_REG_967981_EXT_METADATA_START_ADDR_BMSK \
	0xffffffff
#define \
	\
VIDC_REG_967981_EXT_METADATA_START_ADDR_SHFT          0

#define VIDC_REG_817468_ADDR  \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ed4)
#define VIDC_REG_817468_PHYS  \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ed4)
#define VIDC_REG_817468_RMSK                       0xffffffff
#define VIDC_REG_817468_SHFT                                0
#define VIDC_REG_817468_OUT(v)                     \
	out_dword(VIDC_REG_817468_ADDR, v)
#define VIDC_REG_817468_OUTM(m, v)                  \
	out_dword_masked(VIDC_REG_817468_ADDR, m, v, \
			VID_C_REG_817468_SHADOW)
#define VIDC_REG_817468_ALLOCATED_DPB_SIZE_BMSK               \
	0xffffffff
#define \
	\
VIDC_REG_817468_ALLOCATED_DPB_SIZE_SHFT                        0

#define VIDC_REG_528414_ADDR                   \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ed8)
#define VIDC_REG_528414_PHYS                   \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ed8)
#define VIDC_REG_528414_RMSK                   0xffffffff
#define VIDC_REG_528414_SHFT                            0
#define VIDC_REG_528414_IN                     \
	in_dword_masked(VIDC_REG_528414_ADDR,  \
			VIDC_REG_528414_RMSK)
#define VIDC_REG_528414_INM(m)                 \
	in_dword_masked(VIDC_REG_528414_ADDR,  m)
#define VIDC_REG_528414_PIC_TIME_TOP_BMSK       0xffffffff
#define VIDC_REG_528414_PIC_TIME_TOP_SHFT                0

#define VIDC_REG_1642_ADDR                     \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000edc)
#define VIDC_REG_1642_PHYS                     \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000edc)
#define VIDC_REG_1642_RMSK                     0xffffffff
#define VIDC_REG_1642_SHFT                              0
#define VIDC_REG_1642_IN                       \
	in_dword_masked(VIDC_REG_1642_ADDR,  \
			VIDC_REG_1642_RMSK)
#define VIDC_REG_1642_INM(m)                   \
	in_dword_masked(VIDC_REG_1642_ADDR,  m)
#define VIDC_REG_1642_PIC_TIME_BOTTOM_BMSK           0xffffffff
#define VIDC_REG_1642_PIC_TIME_BOTTOM_SHFT                    0

#define VIDC_REG_754370_ADDR                 \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ee0)
#define VIDC_REG_754370_PHYS                 \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ee0)
#define VIDC_REG_754370_RMSK                 0xffffffff
#define VIDC_REG_754370_SHFT                          0
#define VIDC_REG_754370_OUT(v)               \
	out_dword(VIDC_REG_754370_ADDR, v)
#define VIDC_REG_754370_OUTM(m, v)            \
	out_dword_masked(VIDC_REG_754370_ADDR, m, v, \
			VID_C_REG_754370_SHADOW)
#define VIDC_REG_754370_SET_FRAME_TAG_BMSK   0xffffffff
#define VIDC_REG_754370_SET_FRAME_TAG_SHFT            0

#define VIDC_REG_822104_ADDR          \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ee4)
#define VIDC_REG_822104_PHYS                \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ee4)
#define VIDC_REG_822104_RMSK                0xffffffff
#define VIDC_REG_822104_SHFT                         0
#define VIDC_REG_822104_IN                  \
	in_dword_masked(VIDC_REG_822104_ADDR,  \
			VIDC_REG_822104_RMSK)
#define VIDC_REG_822104_INM(m)              \
	in_dword_masked(VIDC_REG_822104_ADDR,  m)
#define VIDC_REG_822104_GET_FRAME_TAG_BOTTOM_BMSK 0xffffffff
#define VIDC_REG_822104_GET_FRAME_TAG_BOTTOM_SHFT          0

#define VIDC_REG_934882_ADDR               \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00000ee8)
#define VIDC_REG_934882_PHYS               \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000ee8)
#define VIDC_REG_934882_RMSK                      0x1
#define VIDC_REG_934882_SHFT                        0
#define VIDC_REG_934882_OUT(v)             \
	out_dword(VIDC_REG_934882_ADDR, v)
#define VIDC_REG_934882_OUTM(m, v)          \
	out_dword_masked(VIDC_REG_934882_ADDR, m, v, \
			VID_C_REG_934882_SHADOW)
#define VIDC_REG_934882_DPB_FLUSH_BMSK            0x1
#define VIDC_REG_934882_DPB_FLUSH_SHFT              0

#define VIDC_REG_645833_ADDR                       \
	(VIDC_720P_WRAPPER_REG_BASE      + 0x00001000)
#define VIDC_REG_645833_PHYS                       \
	(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00001000)
#define VIDC_REG_645833_RMSK                       0xffff0007
#define VIDC_REG_645833_SHFT                                0
#define VIDC_REG_645833_IN                         \
	in_dword_masked(VIDC_REG_645833_ADDR,  VIDC_REG_645833_RMSK)
#define VIDC_REG_645833_INM(m)                     \
	in_dword_masked(VIDC_REG_645833_ADDR,  m)
#define VIDC_REG_645833_720PV_720P_WRAPPER_VERSION_BMSK 0xffff0000
#define VIDC_REG_645833_720PV_720P_WRAPPER_VERSION_SHFT       0x10
#define VIDC_REG_645833_TEST_MUX_SEL_BMSK                 0x7
#define VIDC_REG_645833_TEST_MUX_SEL_SHFT                   0


#define VIDC_REG_360816_ADDR \
       (VIDC_720P_WRAPPER_REG_BASE + 0x00000d0c)
#define VIDC_REG_360816_PHYS \
       (VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000d0c)
#define VIDC_REG_360816_RMSK 0xffffffff
#define VIDC_REG_360816_SHFT 0
#define VIDC_REG_360816_OUT(v)                  \
		out_dword(VIDC_REG_360816_ADDR, v)

#define VIDC_REG_360816_OUTM(m, v)               \
		out_dword_masked(VIDC_REG_360816_ADDR, m, v, \
		VID_C_REG_360816_SHADOW)
#define VIDC_REG_360816_ALLOCATED_MEM_SIZE_BMSK  0xffff0000
#define VIDC_REG_360816_ALLOCATED_MEM_SIZE_SHFT      0x10
#define VIDC_REG_360816_SET_MEMORY_DUMP_TYPE_BMSK     0x6
#define VIDC_REG_360816_SET_MEMORY_DUMP_TYPE_SHFT     0x1
#define VIDC_REG_360816_ENABLE_FW_DEBUG_INFO_BMSK     0x1
#define VIDC_REG_360816_ENABLE_FW_DEBUG_INFO_SHFT     0x0

#define VIDC_REG_812777_ADDR \
       (VIDC_720P_WRAPPER_REG_BASE + 0x00000d10)
#define VIDC_REG_812777_PHYS \
       (VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000d10)
#define VIDC_REG_812777_RMSK 0xffffffff
#define VIDC_REG_812777_SHFT 0
#define VIDC_REG_812777_OUT(v)               \
		out_dword(VIDC_REG_812777_ADDR, v)
#define VIDC_REG_812777_OUTM(m, v)            \
		out_dword_masked(VIDC_REG_812777_ADDR,\
		m, v, VID_C_REG_812777_SHADOW)
#define VIDC_REG_812777_DBG_INFO_INPUT1_BMSK 0xffffffff
#define VIDC_REG_812777_DBG_INFO_INPUT1_SHFT 0

#define VIDC_REG_354189_ADDR \
		(VIDC_720P_WRAPPER_REG_BASE + 0x00000d18)
#define VIDC_REG_354189_PHYS \
		(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000d18)
#define VIDC_REG_354189_RMSK 0xffffffff
#define VIDC_REG_354189_SHFT 0
#define VIDC_REG_354189_OUT(v) \
		out_dword(VIDC_REG_354189_ADDR, v)
#define VIDC_REG_354189_OUTM(m, v)\
		out_dword_masked(VIDC_REG_354189_ADDR,\
		m, v, VID_C_REG_354189_SHADOW)
#define VIDC_REG_354189_EXTRADATA_ADDR_BMSK   0xffffffff
#define VIDC_REG_354189_EXTRADATA_ADDR_SHFT            0


#define VIDC_REG_990962_ADDR \
		(VIDC_720P_WRAPPER_REG_BASE + 0x00000d1c)
#define VIDC_REG_990962_PHYS \
		(VIDC_720P_WRAPPER_REG_BASE_PHYS + 0x00000d1c)
#define VIDC_REG_990962_RMSK 0xffffffff
#define VIDC_REG_990962_SHFT 0
#define VIDC_REG_990962_OUT(v) \
		out_dword(VIDC_REG_990962_ADDR, v)
#define VIDC_REG_990962_OUTM(m, v) \
		out_dword_masked(VIDC_REG_990962_ADDR, m, v,\
		VID_C_REG_990962_SHADOW)
#define VIDC_REG_990962_CMD_START_BMSK   0xffffffff
#define VIDC_REG_990962_CMD_START_SHFT            0



/** List all the levels and their register valus */

#define VIDC_720p_PROFILE_MPEG4_SP      0
#define VIDC_720p_PROFILE_MPEG4_ASP     1
#define VIDC_720p_PROFILE_H264_BASELINE 0
#define VIDC_720p_PROFILE_H264_MAIN     1
#define VIDC_720p_PROFILE_H264_HIGH     2
#define VIDC_720p_PROFILE_H263_BASELINE 0

#define VIDC_720p_PROFILE_VC1_SP        0
#define VIDC_720p_PROFILE_VC1_MAIN      1
#define VIDC_720p_PROFILE_VC1_ADV       2
#define VIDC_720p_PROFILE_MPEG2_MAIN    4
#define VIDC_720p_PROFILE_MPEG2_SP      5

#define VIDC_720P_MPEG4_LEVEL0  0
#define VIDC_720P_MPEG4_LEVEL0b 9
#define VIDC_720P_MPEG4_LEVEL1  1
#define VIDC_720P_MPEG4_LEVEL2  2
#define VIDC_720P_MPEG4_LEVEL3  3
#define VIDC_720P_MPEG4_LEVEL3b 7
#define VIDC_720P_MPEG4_LEVEL4a 4
#define VIDC_720P_MPEG4_LEVEL5  5
#define VIDC_720P_MPEG4_LEVEL6  6

#define VIDC_720P_H264_LEVEL1     10
#define VIDC_720P_H264_LEVEL1b    9
#define VIDC_720P_H264_LEVEL1p1   11
#define VIDC_720P_H264_LEVEL1p2   12
#define VIDC_720P_H264_LEVEL1p3   13
#define VIDC_720P_H264_LEVEL2     20
#define VIDC_720P_H264_LEVEL2p1   21
#define VIDC_720P_H264_LEVEL2p2   22
#define VIDC_720P_H264_LEVEL3     30
#define VIDC_720P_H264_LEVEL3p1   31

#define VIDC_720P_H263_LEVEL10    10
#define VIDC_720P_H263_LEVEL20    20
#define VIDC_720P_H263_LEVEL30    30
#define VIDC_720P_H263_LEVEL40    40
#define VIDC_720P_H263_LEVEL45    45
#define VIDC_720P_H263_LEVEL50    50
#define VIDC_720P_H263_LEVEL60    60
#define VIDC_720P_H263_LEVEL70    70

#define VIDC_720P_CMD_CHSET               0x0
#define VIDC_720P_CMD_CHEND               0x2
#define VIDC_720P_CMD_INITCODEC           0x3
#define VIDC_720P_CMD_FRAMERUN            0x4
#define VIDC_720P_CMD_INITBUFFERS         0x5
#define VIDC_720P_CMD_FRAMERUN_REALLOCATE 0x6

enum vidc_720p_endian_type {
	VIDC_720P_BIG_ENDIAN = 0x0,
	VIDC_720P_LITTLE_ENDIAN = 0x1
};

enum vidc_720p_memory_access_method_type {
	VIDC_720P_TILE_LINEAR = 0,
	VIDC_720P_TILE_16x16 = 2,
	VIDC_720P_TILE_64x32 = 3
};

enum vidc_720p_interrupt_control_mode_type {
	VIDC_720P_INTERRUPT_MODE = 0,
	VIDC_720P_POLL_MODE = 1
};

enum vidc_720p_interrupt_level_selection_type {
	VIDC_720P_INTERRUPT_LEVEL_SEL = 0,
	VIDC_720P_INTERRUPT_PULSE_SEL = 1
};

#define VIDC_720P_INTR_BUFFER_FULL             0x002
#define VIDC_720P_INTR_FW_DONE                 0x020
#define VIDC_720P_INTR_HEADER_DONE             0x040
#define VIDC_720P_INTR_DMA_DONE                0x080
#define VIDC_720P_INTR_FRAME_DONE              0x100

enum vidc_720p_enc_dec_selection_type {
	VIDC_720p_DECODER = 0,
	VIDC_720p_ENCODER = 1
};

enum vidc_720p_codec_type {
	VIDC_720p_MPEG4 = 0,
	VIDC_720p_H264 = 1,
	VIDC_720p_DIVX = 2,
	VIDC_720p_XVID = 3,
	VIDC_720p_H263 = 4,
	VIDC_720p_MPEG2 = 5,
	VIDC_720p_VC1 = 6
};

enum vidc_720p_frame_type {
	VIDC_720p_NOTCODED = 0,
	VIDC_720p_IFRAME = 1,
	VIDC_720p_PFRAME = 2,
	VIDC_720p_BFRAME = 3
};

enum vidc_720p_entropy_sel_type {
	VIDC_720p_ENTROPY_SEL_CAVLC = 0,
	VIDC_720p_ENTROPY_SEL_CABAC = 1
};

enum vidc_720p_cabac_model_type {
	VIDC_720p_CABAC_MODEL_NUMBER_0 = 0,
	VIDC_720p_CABAC_MODEL_NUMBER_1 = 1,
	VIDC_720p_CABAC_MODEL_NUMBER_2 = 2
};

enum vidc_720p_DBConfig_type {
	VIDC_720p_DB_ALL_BLOCKING_BOUNDARY = 0,
	VIDC_720p_DB_DISABLE = 1,
	VIDC_720p_DB_SKIP_SLICE_BOUNDARY = 2
};

enum vidc_720p_MSlice_selection_type {
	VIDC_720P_MSLICE_BY_MB_COUNT = 0,
	VIDC_720P_MSLICE_BY_BYTE_COUNT = 1,
	VIDC_720P_MSLICE_BY_GOB = 2,
	VIDC_720P_MSLICE_OFF = 3
};

enum vidc_720p_display_status_type {
	VIDC_720p_DECODE_ONLY = 0,
	VIDC_720p_DECODE_AND_DISPLAY = 1,
	VIDC_720p_DISPLAY_ONLY = 2,
  VIDC_720p_EMPTY_BUFFER = 3
};

#define VIDC_720p_ENC_IFRAME_REQ       0x1
#define VIDC_720p_ENC_IPERIOD_CHANGE   0x2
#define VIDC_720p_ENC_FRAMERATE_CHANGE 0x3
#define VIDC_720p_ENC_BITRATE_CHANGE   0x4

#define VIDC_720p_FLUSH_REQ     0x1
#define VIDC_720p_EXTRADATA     0x2

#define VIDC_720p_METADATA_ENABLE_QP           0x01
#define VIDC_720p_METADATA_ENABLE_CONCEALMB    0x02
#define VIDC_720p_METADATA_ENABLE_VC1          0x04
#define VIDC_720p_METADATA_ENABLE_SEI          0x08
#define VIDC_720p_METADATA_ENABLE_VUI          0x10
#define VIDC_720p_METADATA_ENABLE_ENCSLICE     0x20
#define VIDC_720p_METADATA_ENABLE_PASSTHROUGH  0x40

struct vidc_720p_dec_disp_info_type {
	enum vidc_720p_display_status_type e_disp_status;
	u32 n_resl_change;
	u32 n_reconfig_flush_done;
	u32 n_img_size_x;
	u32 n_img_size_y;
	u32 n_y_addr;
	u32 n_c_addr;
	u32 n_tag_top;
	u32 n_pic_time_top;
	u32 n_disp_is_interlace;
	u32 n_tag_bottom;
	u32 n_pic_time_bottom;
	u32 n_metadata_exists;
	u32 n_crop_exists;
	u32 n_crop_right_offset;
	u32 n_crop_left_offset;
	u32 n_crop_bottom_offset;
	u32 n_crop_top_offset;
	u32 n_input_frame_type;
	u32 n_input_bytes_consumed;
	u32 n_input_is_interlace;
	u32 n_input_frame_num;
};

struct vidc_720p_seq_hdr_info_type {
	u32 n_img_size_x;
	u32 n_img_size_y;
	u32 n_dec_frm_size;
	u32 n_min_num_dpb;
	u32 n_min_dpb_size;
	u32 n_profile;
	u32 n_level;
	u32 n_progressive;
};

struct vidc_720p_enc_frame_info_type {
	u32 n_enc_size;
	u32 n_frame_type;
	u32 n_metadata_exists;
};

void vidc_720p_set_device_virtual_base(u8 *p_core_virtual_base_addr);

void vidc_720p_init(char **ppsz_version, u32 i_firmware_size,
	u32 *pi_firmware_address, enum vidc_720p_endian_type e_dma_endian,
	u32 b_interrupt_off,
	enum vidc_720p_interrupt_level_selection_type	e_interrupt_sel,
	u32 Interrupt_mask);

u32 vidc_720p_do_sw_reset(void);

u32 vidc_720p_reset_is_success(void);

void vidc_720p_start_cpu(enum vidc_720p_endian_type e_dma_endian,
		u32 *p_icontext_bufferstart, u32 *p_debug_core_dump_addr,
		u32  debug_buffer_size);

void vidc_720p_fw_status(u32 *fw_status);

void vidc_720p_stop_fw(void);

void vidc_720p_get_interrupt_status(u32 *p_interrupt_status,
		u32 *p_cmd_err_status, u32 *p_disp_pic_err_status,
		u32 *p_op_failed);

void vidc_720p_interrupt_done_clear(void);

void vidc_720p_submit_command(u32 ch_id, u32 n_cmd_id);


void vidc_720p_set_channel(u32 i_ch_id,
	enum vidc_720p_enc_dec_selection_type e_enc_dec_sel,
	enum vidc_720p_codec_type e_codec, u32 *pi_fw, u32 i_firmware_size);

void vidc_720p_encode_set_profile(u32 i_profile, u32 i_level);

void vidc_720p_set_frame_size(u32 i_size_x, u32 i_size_y);

void vidc_720p_encode_set_fps(u32 i_rc_frame_rate);

void vidc_720p_encode_set_vop_time(u32 n_vop_time_resolution,
		u32 n_vop_time_increment);

void vidc_720p_encode_hdr_ext_control(u32 n_header_extension);

void vidc_720p_encode_set_short_header(u32 i_short_header);

void vidc_720p_encode_set_qp_params(u32 i_max_qp, u32 i_min_qp);

void vidc_720p_encode_set_rc_config(u32 b_enable_frame_level_rc,
		u32 b_enable_mb_level_rc_flag, u32 i_frame_qp, u32 n_pframe_qp);

void vidc_720p_encode_set_bit_rate(u32 i_target_bitrate);

void vidc_720p_encode_dynamic_req_reset(void);

void vidc_720p_encode_dynamic_req_set(u32 n_param_type, u32 n_param_val);

void vidc_720p_encode_set_frame_level_rc_params(u32 i_reaction_coeff);

void vidc_720p_encode_set_mb_level_rc_params(u32 b_dark_region_as_flag,
	u32 b_smooth_region_as_flag, u32 b_static_region_as_flag,
	u32 b_activity_region_flag);

void vidc_720p_encode_set_entropy_control(enum vidc_720p_entropy_sel_type \
		e_entropy_sel,
		enum vidc_720p_cabac_model_type e_cabac_model_number);

void vidc_720p_encode_set_db_filter_control(enum vidc_720p_DBConfig_type
		e_db_config, u32 i_slice_alpha_offset, u32 i_slice_beta_offset);

void vidc_720p_encode_set_intra_refresh_mb_number(u32 i_cir_mb_number);

void vidc_720p_encode_set_multi_slice_info(
		enum vidc_720p_MSlice_selection_type e_m_slice_sel,
		u32 n_multi_slice_size);

void vidc_720p_encode_set_dpb_buffer(u32 *pi_enc_dpb_addr, u32 alloc_len);

void vidc_720p_set_deblock_line_buffer(u32 *pi_deblock_line_buffer_start,
		u32 n_alloc_len);

void vidc_720p_encode_set_i_period(u32 i_i_period);

void vidc_720p_encode_init_codec(u32 i_ch_id,
	enum vidc_720p_memory_access_method_type e_memory_access_model);

void vidc_720p_encode_unalign_bitstream(u32 n_upper_unalign_word,
	u32 n_lower_unalign_word);

void vidc_720p_encode_set_seq_header_buffer(u32 n_ext_buffer_start,
	u32 n_ext_buffer_end, u32 n_start_byte_num);

void vidc_720p_encode_frame(u32 n_ch_id, u32 n_ext_buffer_start,
	u32 n_ext_buffer_end, u32 n_start_byte_number,
	u32 n_y_addr, u32 n_c_addr);

void vidc_720p_encode_get_header(u32 *pi_enc_header_size);

void vidc_720p_enc_frame_info
	(struct vidc_720p_enc_frame_info_type *p_enc_frame_info);

void vidc_720p_decode_bitstream_header(u32 n_ch_id, u32 n_dec_unit_size,
	u32 n_start_byte_num, u32 n_ext_buffer_start, u32 n_ext_buffer_end,
	enum vidc_720p_memory_access_method_type e_memory_access_model);

void vidc_720p_decode_get_seq_hdr_info
    (struct vidc_720p_seq_hdr_info_type *p_seq_hdr_info);

void vidc_720p_decode_set_dpb_release_buffer_mask
    (u32 i_dpb_release_buffer_mask);

void vidc_720p_decode_set_dpb_buffers(u32 i_buf_index, u32 *pi_dpb_buffer);

void vidc_720p_decode_set_comv_buffer
    (u32 *pi_dpb_comv_buffer, u32 n_alloc_len);

void vidc_720p_decode_set_dpb_details
    (u32 n_num_dpb, u32 n_alloc_len, u32 *p_ref_buffer);

void vidc_720p_decode_set_mpeg4Post_filter(u32 b_enable_post_filter);

void vidc_720p_decode_set_error_control(u32 b_enable_error_control);

void vidc_720p_decode_set_mpeg4_data_partitionbuffer(u32 *p_vsp_buf_start);

void vidc_720p_decode_setH264VSPBuffer(u32 *pi_vsp_temp_buffer_start);

void vidc_720p_decode_frame(u32 n_ch_id, u32 n_ext_buffer_start,
		u32 n_ext_buffer_end, u32 n_dec_unit_size,
		u32 n_start_byte_num, u32 n_input_frame_tag);

void vidc_720p_issue_eos(u32 i_ch_id);
void vidc_720p_eos_info(u32 *p_disp_status);

void vidc_720p_decode_display_info
    (struct vidc_720p_dec_disp_info_type *p_disp_info);

void vidc_720p_decode_skip_frm_details(u32 *p_free_luma_dpb);

void vidc_720p_metadata_enable(u32 n_flag, u32 *p_input_buffer);

void vidc_720p_decode_dynamic_req_reset(void);

void vidc_720p_decode_dynamic_req_set(u32 n_property);

void vidc_720p_decode_setpassthrough_start(u32 n_pass_startaddr);

void vidc_720p_RCFrame_skip(u32 n_op, u32 n_vbv_size);


#define DDL_720P_REG_BASE VIDC_720P_WRAPPER_REG_BASE
#define VIDC_BUSY_WAIT(n) udelay(n)

#undef VIDC_REGISTER_LOG_MSG

#define  VIDC_REGISTER_LOG_INTO_BUFFER

#ifdef VIDC_REGISTER_LOG_MSG
#define VIDC_MSG1(msg_format, a) printk(KERN_INFO msg_format, a)
#define VIDC_MSG2(msg_format, a, b) printk(KERN_INFO msg_format, a, b)
#define VIDC_MSG3(msg_format, a, b, c) printk(KERN_INFO msg_format, a, b, c)
#else
#define VIDC_MSG1(msg_format, a)
#define VIDC_MSG2(msg_format, a, b)
#define VIDC_MSG3(msg_format, a, b, c)
#endif

#ifdef VIDC_REGISTER_LOG_INTO_BUFFER

#define VIDC_REGLOG_BUFSIZE 200000
#define VIDC_REGLOG_MAX_PRINT_SIZE 100
extern char vidclog[VIDC_REGLOG_BUFSIZE];
extern unsigned int vidclog_index;

#define VIDC_LOG_BUFFER_INIT \
{if (vidclog_index) \
  memset(vidclog, 0, vidclog_index+1); \
  vidclog_index = 0; }

#define VIDC_REGLOG_CHECK_BUFINDEX(req_size) \
  vidclog_index = \
  (vidclog_index+(req_size) < VIDC_REGLOG_BUFSIZE) ? vidclog_index : 0;

#define VIDC_LOG_WRITE(reg, val) \
{unsigned int len; \
	VIDC_REGLOG_CHECK_BUFINDEX(VIDC_REGLOG_MAX_PRINT_SIZE); \
	len = snprintf(&vidclog[vidclog_index], VIDC_REGLOG_MAX_PRINT_SIZE, \
	"(0x%x:"#reg"=0x%x)" , VIDC_##reg##_ADDR - DDL_720P_REG_BASE, val);\
	vidclog_index += len; }

#define VIDC_LOG_WRITEI(reg, index, val) \
{unsigned int len; \
	VIDC_REGLOG_CHECK_BUFINDEX(VIDC_REGLOG_MAX_PRINT_SIZE); \
	len = snprintf(&vidclog[vidclog_index], VIDC_REGLOG_MAX_PRINT_SIZE, \
	"(0x%x:"#reg"=0x%x)" , VIDC_##reg##_ADDR(index)-DDL_720P_REG_BASE,  \
	val); vidclog_index += len; }

#define VIDC_LOG_WRITEF(reg, field, val) \
{unsigned int len; \
	VIDC_REGLOG_CHECK_BUFINDEX(VIDC_REGLOG_MAX_PRINT_SIZE); \
	len = snprintf(&vidclog[vidclog_index], VIDC_REGLOG_MAX_PRINT_SIZE, \
	"(0x%x:"#reg":0x%x:=0x%x)" , VIDC_##reg##_ADDR - DDL_720P_REG_BASE,  \
	VIDC_##reg##_##field##_BMSK,  val);\
	vidclog_index += len; }

#define VIDC_LOG_READ(reg, pval) \
{ unsigned int len; \
	VIDC_REGLOG_CHECK_BUFINDEX(VIDC_REGLOG_MAX_PRINT_SIZE); \
	len = snprintf(&vidclog[vidclog_index], VIDC_REGLOG_MAX_PRINT_SIZE, \
	"(0x%x:"#reg"==0x%x)" , VIDC_##reg##_ADDR - DDL_720P_REG_BASE,  \
	(u32)*pval); \
	vidclog_index += len; }

#define VIDC_STR_LOGBUFFER(str) \
{ unsigned int len; \
	VIDC_REGLOG_CHECK_BUFINDEX(VIDC_REGLOG_MAX_PRINT_SIZE); \
	len = snprintf(&vidclog[vidclog_index], VIDC_REGLOG_MAX_PRINT_SIZE, \
	"<%s>" , str); vidclog_index += len; }

#define VIDC_LONG_LOGBUFFER(str, arg1) \
{ unsigned int len; \
	VIDC_REGLOG_CHECK_BUFINDEX(VIDC_REGLOG_MAX_PRINT_SIZE); \
	len = snprintf(&vidclog[vidclog_index], VIDC_REGLOG_MAX_PRINT_SIZE, \
	"<%s=0x%x>" , str, arg1); vidclog_index += len; }

#define VIDC_DEBUG_REGISTER_LOG \
{ u32 val; unsigned int len; \
	val = VIDC_720P_IN(REG_904540); \
	VIDC_REGLOG_CHECK_BUFINDEX(VIDC_REGLOG_MAX_PRINT_SIZE); \
	len = snprintf(&vidclog[vidclog_index], 50,  "[dbg1=%x]" , val); \
	vidclog_index += len; \
	val = VIDC_720P_IN(REG_663308); \
	VIDC_REGLOG_CHECK_BUFINDEX(VIDC_REGLOG_MAX_PRINT_SIZE); \
	len = snprintf(&vidclog[vidclog_index], 50,  "[dbg2=%x]" , val); \
	vidclog_index += len; }

#else
#define VIDC_LOG_WRITE(reg, val)
#define VIDC_LOG_WRITEI(reg, index, val)
#define VIDC_LOG_WRITEF(reg, field, val)
#define VIDC_LOG_READ(reg, pval)
#define VIDC_LOG_BUFFER_INIT
#define VIDC_STR_LOGBUFFER(str)
#define VIDC_LONG_LOGBUFFER(str, arg1)
#define VIDC_DEBUG_REGISTER_LOG
#endif

void vidcputlog(char *str);
void vidcput_debug_reglog(void);

#define VIDC_LOGERR_STRING(str) \
do { \
	VIDC_STR_LOGBUFFER(str); \
	VIDC_MSG1("\n<%s>", str); \
} while (0)

#define VIDC_LOG_STRING(str) \
do { \
	VIDC_STR_LOGBUFFER(str); \
	VIDC_MSG1("\n<%s>", str); \
} while (0)

#define VIDC_LOG1(str, arg1) \
do { \
	VIDC_LONG_LOGBUFFER(str, arg1); \
	VIDC_MSG2("\n<%s=0x%08x>", str, arg1); \
} while (0)

#define VIDC_IO_OUT(reg,  val) \
do { \
	VIDC_LOG_WRITE(reg, (u32)val);  \
	VIDC_MSG2("\n(0x%08x:"#reg"=0x%08x)",  \
	(u32)(VIDC_##reg##_ADDR - DDL_720P_REG_BASE),  (u32)val); \
	mb(); \
	VIDC_720P_OUT(reg, val);  \
} while (0)

#define VIDC_IO_OUTI(reg,  index,  val) \
do { \
	VIDC_LOG_WRITEI(reg, index, (u32)val); \
	VIDC_MSG2("\n(0x%08x:"#reg"=0x%08x)",  \
	(u32)(VIDC_##reg##_ADDR(index)-DDL_720P_REG_BASE),  (u32)val); \
	mb(); \
	VIDC_720P_OUTI(reg, index, val);  \
} while (0)

#define VIDC_IO_OUTF(reg,  field,  val) \
do { \
	VIDC_LOG_WRITEF(reg, field, val); \
	VIDC_MSG3("\n(0x%08x:"#reg":0x%x:=0x%08x)",  \
	(u32)(VIDC_##reg##_ADDR - DDL_720P_REG_BASE),  \
	VIDC_##reg##_##field##_BMSK,  (u32)val); \
	mb(); \
	VIDC_720P_OUTF(reg, field, val);  \
} while (0)

#define VIDC_IO_IN(reg, pval) \
do { \
	mb(); \
	*pval = (u32) VIDC_720P_IN(reg); \
	VIDC_LOG_READ(reg, pval); \
	VIDC_MSG2("\n(0x%08x:"#reg"==0x%08x)",  \
	(u32)(VIDC_##reg##_ADDR - DDL_720P_REG_BASE), (u32) *pval);  \
} while (0)

#define VIDC_IO_INF(reg, mask, pval) \
do { \
	mb(); \
	*pval = VIDC_720P_INF(reg, mask); \
	VIDC_LOG_READ(reg, pval); \
	VIDC_MSG2("\n(0x%08x:"#reg"==0x%08x)",  \
	(u32)(VIDC_##reg##_ADDR - DDL_720P_REG_BASE),  *pval); \
} while (0)

#endif
