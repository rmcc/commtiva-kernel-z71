/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
/*
 * Qualcomm Tavarua FM core Driver header file
 */

#ifndef __LINUX_TAVARUA_H
#define __LINUX_TAVARUA_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <asm/sizes.h>
#else
#include <stdint.h>
#endif
#include <linux/ioctl.h>


#define FM_DEBUG

#undef FMDBG
#ifdef FM_DEBUG
  #define FMDBG(fmt, args...) printk(KERN_INFO "msm_radio: " fmt, ##args)
#else
  #define FMDBG(fmt, args...)
#endif


#define FM_PROTO        0x01
#define PROC_DIR        "fm"

/*
 * The frequency is set in units of 62.5 Hz when using V4L2_TUNER_CAP_LOW,
 * 62.5 kHz otherwise.
 * The tuner is able to have a channel spacing of 50, 100 or 200 kHz.
 * tuner->capability is therefore set to V4L2_TUNER_CAP_LOW
 * The FREQ_MUL is then: 1 MHz / 62.5 Hz = 16000
 */
#define FREQ_MUL (1000000 / 62.5)

#define V4L2_CID_PRIVATE_TAVARUA_SRHCMODE	(V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_PRIVATE_TAVARUA_SCANDWELL	(V4L2_CID_PRIVATE_BASE + 2)
#define V4L2_CID_PRIVATE_TAVARUA_SRCHON		(V4L2_CID_PRIVATE_BASE + 3)
#define V4L2_CID_PRIVATE_TAVARUA_STATE		(V4L2_CID_PRIVATE_BASE + 4)
#define V4L2_CID_PRIVATE_TAVARUA_TRANSMIT_MODE	(V4L2_CID_PRIVATE_BASE + 5)
#define V4L2_CID_PRIVATE_TAVARUA_RDSGROUP_MASK	(V4L2_CID_PRIVATE_BASE + 6)
#define V4L2_CID_PRIVATE_TAVARUA_ZONE		(V4L2_CID_PRIVATE_BASE + 7)

enum tavarua_buf_t {
  TAVARUA_BUF_RT_RDS,
  TAVARUA_BUF_PS_RDS,
  TAVARUA_BUF_EVENTS,
  TAVARUA_BUF_RAW_RDS,
  TAVARUA_BUF_MAX
};

enum tavarua_xfr_t {
  TAVARUA_XFR_RT_RDS,
  TAVARUA_XFR_PS_RDS,
  TAVARUA_XFR_ERROR,
  TAVARUA_XFR_MAX

};

/* offsets */
#define RAW_RDS		0x0F
#define RDS_BLOCK 	3

/* registers*/
#define MARIMBA_XO_BUFF_CNTRL 0x04
#define RADIO_REGISTERS 0x2F

#define STATUS_REG_NUM 	3
#define STATUS_REG1	0x00
#define STATUS_REG2	0x01
#define STATUS_REG3	0x02

#define RDCTRL		0x03
#define FM_RECV_ON	0x01

#define XFR_REG_NUM	16
#define XFRCTRL		0x1F
#define XFRCTRL_WRITE	0x80
#define LEAKAGE_CNTRL	0xFE

#define FREQ		0x04
#define TUNECTRL	0x05
#define TUNE_STATION	0x01

#define IOCTRL		0x09
#define RDSCTRL		0x0A
#define RDS_ON		0x01

#define ADVCTRL		0x0B
#define RDSRTEN		0x08
#define RDSPSEN		0x10

/* search options */
#define SRCHRDS1	0x06
#define SRCHRDS2	0x07
#define SRCHCTRL	0x08
#define SEEK		0x00
#define SCAN		0x01
#define SCAN_FOR_STRONG	0x02
#define SCAN_FOR_WEAK 	0x03
#define RDS_SEEK_PTY	0x04
#define RDS_SCAN_PTY	0x05
#define RDS_SEEK_PI	0x06
#define RDS_AF_JUMP	0x07

#define SRCH_MODE	0x07
#define SRCH_DIR	0x08 /* 0-up 1-down */
#define SCAN_DWELL	0x70
#define SRCH_ON		0x80

#define IOC_HRD_MUTE	0x03
#define IOC_SFT_MUTE    0x04
#define IOC_MON_STR     0x08
#define IOC_SIG_BLND    0x10
#define IOC_INTF_BLND   0x20
#define IOC_ANTENNA     0x40

#define FM_ENABLE	0x0C


/* Interrupt status */

/* interrupt register 1 */
#define	READY		0x01	/* Radio ready after powerup or reset */
#define	TUNE		0x02	/* Tune completed */
#define	SEARCH		0x04	/* Search completed (read FREQ) */
#define	SCANNEXT	0x08	/* Scanning for next station */
#define	SIGNAL		0x10	/* Signal indicator change (read SIGSTATE) */
#define	INTF		0x20	/* Interference cnt has fallen outside range */
#define	SYNC		0x40	/* RDS sync state change (read RDSSYNC) */
#define	AUDIO		0x80	/* Audio Control indicator (read AUDIOIND) */

/* interrupt register 2 */
#define	RDSDAT		0x01	/* New unread RDS data group available */
#define	BLOCKB		0x02	/* Block-B match condition exists */
#define	PROGID		0x04	/* Block-A or Block-C matched stored PI value */
#define	RDSPS		0x08	/* New RDS Program Service Table available */
#define	RDSRT		0x10	/* New RDS Radio Text available */
#define	RDSAF		0x20	/* New RDS AF List available */
#define	TXRDSDAT	0x40	/* Transmitted an RDS group */
#define	TXRDSDONE	0x80	/* RDS raw group one-shot transmit completed */

/* interrupt register 3 */
#define	TRANSFER	0x01	/* Data transfer (XFR) completed */
#define	RDSPROC		0x02	/* Dynamic RDS Processing complete */
#define	ERROR		0x80	/* Err occurred. Read code to determine cause */

/* Transfer */
enum tavarua_xfr_ctrl_t {
  RDS_PS_0 = 0x01,
  RDS_PS_1,
  RDS_PS_2,
  RDS_PS_3,
  RDS_PS_4,
  RDS_PS_5,
  RDS_PS_6,
  RDS_RT_0,
  RDS_RT_1,
  RDS_RT_2,
  RDS_RT_3,
  RDS_RT_4,
  RDS_AF_0,
  RDS_AF_1,
  RDS_CONFIG,
  RDS_TX_GROUPS,
  RDS_COUNT_0,
  RDS_COUNT_1,
  RDS_COUNT_2,
  RADIO_CONFIG,
  RX_CONFIG,
  RX_TIMERS,
  RX_STATIONS_0,
  RX_STATIONS_1,
  INT_CTRL,
  ERROR_CODE,
  CHIPID,
  CAL_DAT_0 = 0x20,
  CAL_DAT_1,
  CAL_DAT_2,
  CAL_DAT_3,
  CAL_CFG_0,
  CAL_CFG_1,
  DIG_INTF_0,
  DIG_INTF_1,
  DIG_AGC_0,
  DIG_AGC_1,
  DIG_AGC_2,
  DIG_AUDIO_0,
  DIG_AUDIO_1,
  DIG_AUDIO_2,
  DIG_AUDIO_3,
  DIG_AUDIO_4,
  DIG_RXRDS,
  DIG_DCC,
  DIG_SPUR,
  DIG_MPXDCC,
  DIG_PILOT,
  DIG_DEMOD,
  DIG_MOST,
  DIG_TX_0,
  DIG_TX_1,
  PHY_CONFIG = 0x3C,
  PHY_TXBLOCK,
  PHY_TCB,
  TAVARUA_XFR_CTRL_MAX
};

enum tavarua_evt_t {
  TAVARUA_EVT_RADIO_READY,
  TAVARUA_EVT_TUNE_SUCC,
  TAVARUA_EVT_SEEK_COMPLETE,
  TAVARUA_EVT_SCAN_NEXT,
  TAVARUA_EVT_SYNC,
  TAVARUA_EVT_SIGNAL,
  TAVARUA_EVT_AUDIO,
  TAVARUA_EVT_NEW_RAW_RDS,
  TAVARUA_EVT_NEW_RT_RDS,
  TAVARUA_EVT_NEW_PS_RDS,
  TAVARUA_EVT_ERROR
};

enum tavarua_zone {
  TAVARUA_ZONE_0, /* 0: 200 kHz (USA, Australia) */
  TAVARUA_ZONE_1, /* 1: 100 kHz (Europe, Japan) */
  TAVARUA_ZONE_2  /* 2:  50 kHz */
};


#endif /* __LINUX_TAVARUA_H */
