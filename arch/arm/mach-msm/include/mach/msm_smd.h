/* linux/include/asm-arm/arch-msm/msm_smd.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_SMD_H
#define __ASM_ARCH_MSM_SMD_H

/* FIH, Debbie Sun, 2009/06/18 { */
/* get share memory command address dynamically */
#ifdef CONFIG_FIH_FXX
#include "smd_private_fih.h"
#endif
/* FIH, Debbie Sun, 2009/06/18 }*/

typedef struct smd_channel smd_channel_t;

/* FIH, Debbie Sun, 2009/06/18 { */
/* get share memory command address dynamically */
#ifdef CONFIG_FIH_FXX
struct smem_oem_info
{
        unsigned int hw_id;
        unsigned int keypad_info;
        unsigned int power_on_cause;
	/* FIH, MichaelKao, 2009/07/16 { */
	/* add for Read modem mode from smem */
        unsigned int network_mode;
	/* FIH, MichaelKao, 2009/07/16 { */
	/* FIH, Debbie modify, 2010/01/06 { */
        /* add for display info about download and sd ram dump */
        unsigned int oemsbl_mode;
        char   flash_name[32];
        char   oem_mod_rev[16];
        unsigned int progress;
        unsigned int msg_counter;
        /* FIH, Debbie modify, 2010/01/06 } */
	/* FIH, Debbie Sun, 2010/05/24 { */
	/* get ram info form share memory */
        unsigned int dram_info;
	/* FIH, Debbie Sun, 2010/05/24 { */
};

/* FIH, Debbie, 2009/09/11 { */
/* switch UART for printk log */
struct smem_host_oem_info
{
        unsigned int host_usb_id;
        unsigned int host_log_from_uart;
        unsigned int host_enable_kpd;
        unsigned int host_used4;
};
/* FIH, Debbie, 2009/09/11 } */
#endif
/* FIH, Debbie Sun, 2009/06/18 }*/
/* FIH, MichaelKao, 2009/07/16 { */
/* add for Read modem mode from smem */
typedef enum
{
    FIH_ERROR           = 0,
    FIH_GSM             = 0x0001,
    FIH_WCDMA           = 0x0002, // WCDMA only, e.g. in release 4315, the build id is TSNCJOLY
    FIH_CDMA1X          = 0x0004, // CDMA2000 only
    FIH_WCDMA_CDMA1X    = (FIH_WCDMA | FIH_CDMA1X), // WORLD MODE
    FIH_NETWORK_MODE_MAX
} fih_network_mode_type;
	/* FIH, MichaelKao, 2009/07/16 } */

/* warning: notify() may be called before open returns */
int smd_open(const char *name, smd_channel_t **ch, void *priv,
	     void (*notify)(void *priv, unsigned event));

#define SMD_EVENT_DATA 1
#define SMD_EVENT_OPEN 2
#define SMD_EVENT_CLOSE 3

int smd_close(smd_channel_t *ch);

/* passing a null pointer for data reads and discards */
int smd_read(smd_channel_t *ch, void *data, int len);
int smd_read_from_cb(smd_channel_t *ch, void *data, int len);

/* Write to stream channels may do a partial write and return
** the length actually written.
** Write to packet channels will never do a partial write --
** it will return the requested length written or an error.
*/
int smd_write(smd_channel_t *ch, const void *data, int len);

int smd_write_avail(smd_channel_t *ch);
int smd_read_avail(smd_channel_t *ch);

/* Returns the total size of the current packet being read.
** Returns 0 if no packets available or a stream channel.
*/
int smd_cur_packet_size(smd_channel_t *ch);

/* FIH, Debbie Sun, 2009/06/18 { */
/* get share memory command address dynamically */
#ifdef CONFIG_FIH_FXX
void fih_smem_alloc(void);

/* switch UART for printk log */
void fih_smem_alloc_for_host_used(void);

unsigned int fih_read_hwid_from_smem(void);

/* modify new hardware id */
unsigned int fih_read_orig_hwid_from_smem(void);

unsigned int fih_read_mode_from_smem(void);

/* add for Read modem mode from smem */
unsigned int fih_read_network_mode_from_smem(void);
#define POWER_ON_CAUSE_PROC_READ_ENTRY 1
#ifdef POWER_ON_CAUSE_PROC_READ_ENTRY
unsigned int fih_read_power_on_cuase_from_smem(void);
#endif
/* switch UART for printk log */
unsigned int fih_read_uart_switch_from_smem(void);

unsigned int fih_read_usb_id_from_smem(void);
unsigned int fih_read_kpd_from_smem(void);
/* FIH, Debbie Sun, 2010/05/24 { */
/* get ram info and device name form share memory */
unsigned int fih_read_dram_info_from_smem(void);
char* fih_read_flash_name_from_smem(void);
/* FIH, Debbie Sun, 2010/05/24 } */

/* FIH, Paul Huang, 2009/08/04 { */
/* add for QXDM LOG to SD card */
#define SAVE_QXDM_LOG_TO_SD_CARD    1
/* FIH, Paul Huang, 2009/08/04 } */
#endif
/* FIH, Debbie Sun, 2009/06/18 }*/

#if 0
/* these are interruptable waits which will block you until the specified
** number of bytes are readable or writable.
*/
int smd_wait_until_readable(smd_channel_t *ch, int bytes);
int smd_wait_until_writable(smd_channel_t *ch, int bytes);
#endif

/* these are used to get and set the IF sigs of a channel.
 * DTR and RTS can be set; DSR, CTS, CD and RI can be read.
 */
int smd_tiocmget(smd_channel_t *ch);
int smd_tiocmset(smd_channel_t *ch, unsigned int set, unsigned int clear);

enum {
	SMD_APPS_MODEM = 0,
	SMD_APPS_QDSP,
	SMD_MODEM_QDSP,
	SMD_APPS_DSPS,
	SMD_MODEM_DSPS,
	SMD_QDSP_DSPS,
	SMD_LOOPBACK_TYPE = 100,

};

int smd_named_open_on_edge(const char *name, uint32_t edge, smd_channel_t **_ch,
			   void *priv, void (*notify)(void *, unsigned));

/* FIH, Charles Huang, 2009/05/18 { */
/* [FXX_CR], HW ID */
#ifdef CONFIG_FIH_FXX
typedef enum 
{
    CMCS_HW_VER_EVB1=0,  //40k resister  
    CMCS_HW_VER_EVB2,    
    CMCS_HW_VER_EVB3,
    CMCS_HW_VER_EVB4,
    CMCS_HW_VER_EVB5,
    CMCS_RTP_PR1,        //10k resister
    CMCS_RTP_PR2,        //30k resister
    CMCS_RTP_PR3,
    CMCS_RTP_PR4,
    CMCS_RTP_PR5,     
    CMCS_RTP_MP1,
    CMCS_RTP_MP2,
    CMCS_RTP_MP3,
    CMCS_CTP_PR1,       //20k resister
    CMCS_CTP_PR2,       //68.2k resister
    CMCS_CTP_PR3,
    CMCS_CTP_PR4,
    CMCS_CTP_PR5,
    CMCS_CTP_MP1,
    CMCS_CTP_MP2,
    CMCS_CTP_MP3,

    /* add for 7627 { */
    /* FIH, Debbie, 2010/05/04 { */
    /* modify 7627 start index because 7227 hwid is not enough */
    CMCS_7627_EVB1 = 0x500,
    /* FIH, Debbie, 2010/05/04 } */
    CMCS_7627_PR1,      //100k resister
    CMCS_7627_PR2,
    CMCS_7627_PR3,
    CMCS_7627_PR4,
    CMCS_7627_PR5,
 
    /* add for F913 { */
    CMCS_F913_PR1,
    CMCS_F913_PR2,    
    CMCS_F913_PR3,
    CMCS_F913_PR4,
    CMCS_F913_PR5,
    CMCS_F913_MP1,

    CMCS_HW_VER_MAX
}cmcs_hw_version_type;

/* FIH, Debbie Sun, 2009/07/15 { */
/* 7227_CR_XX start, modify for new HWID list*/
typedef enum 
{
    CMCS_HW_EVB1=0,        //40k resister  
    CMCS_HW_EVB2,    
    CMCS_HW_EVB3,
    CMCS_HW_EVB4,
    CMCS_HW_EVB5,
    CMCS_ORIG_RTP_PR1,             //10k resister
    CMCS_ORIG_CTP_PR1 = 0xd,       //20k resister

    /* 850 family */
    CMCS_850_RTP_PR2 = 0x10,  // 30k resister
    CMCS_850_RTP_PR3,
    CMCS_850_RTP_PR4,
    CMCS_850_RTP_PR5,
    CMCS_850_RTP_MP1,        //4.7k resister
    CMCS_850_RTP_MP2,
    CMCS_850_RTP_MP3,
    CMCS_850_CTP_PR2 = 0x17, //68.1k resister
    CMCS_850_CTP_PR3,
    CMCS_850_CTP_PR4,
    CMCS_850_CTP_PR5,
    CMCS_850_CTP_MP1,        //82K resister
    CMCS_850_CTP_MP2,
    CMCS_850_CTP_MP3,

    /* 900 family */
    CMCS_900_RTP_PR2 = 0x20, //51k resister
    CMCS_900_RTP_PR3,
    CMCS_900_RTP_PR4,
    CMCS_900_RTP_PR5,
    CMCS_900_RTP_MP1,        //60.4k resister
    CMCS_900_RTP_MP2,
    CMCS_900_RTP_MP3,
    CMCS_900_CTP_PR2 = 0x27, //90.9k resister
    CMCS_900_CTP_PR3,
    CMCS_900_CTP_PR4,
    CMCS_900_CTP_PR5,
    CMCS_900_CTP_MP1,        //100k resister
    CMCS_900_CTP_MP2,
    CMCS_900_CTP_MP3,

    /* AWS family */
    CMCS_145_CTP_PR1 = 0x2E,  //750k resister
	
    /* FST family */
    CMCS_125_FST_PR1 = 0x30, //220k resister
    CMCS_125_FST_PR2,        //270k resister
    CMCS_125_FST_MP1,        //390k resister
    CMCS_128_FST_PR1 = 0x40, //240k resister
    CMCS_128_FST_PR2,        //300k resister
    CMCS_128_FST_MP1,        //430k resister

    /* F917 family */
    CMCS_CTP_F917_PR1 = 0x50,
    CMCS_CTP_F917_PR2,
    CMCS_CTP_F917_PR3,
    CMCS_CTP_F917_PR4,
    CMCS_CTP_F917_PR5,
    CMCS_CTP_F917_MP1,
    CMCS_CTP_F917_MP2,
    CMCS_CTP_F917_MP3,
	
    /* FIH, Paul Huang, 2010/03/03 { */
    /* Greco family */
    CMCS_125_CTP_GRE_PR1 = 0x60,   //160k resister 
    CMCS_125_CTP_GRE_PR2,  
    CMCS_125_CTP_GRE_MP1,  
    CMCS_125_CTP_GRE_MP2, 
    /* FIH, Paul Huang, 2010/03/03 } */

    /* FIH, Debbie, 2010/05/04 { */
    /* FA9 family*/
    /* without real key */
    CMCS_125_FA9_PR1 = 0x70, //MPP3 4.7K, MPP4 High
    CMCS_125_FA9_PR2,
    CMCS_125_FA9_PR3,
    CMCS_125_FA9_MP1,

    /* FIH, Debbie, 2010/05/24 { */
    //FAA family
    CMCS_125_4G4G_FAA_PR1 = 0x80,
    CMCS_125_4G4G_FAA_PR2,
    CMCS_125_4G4G_FAA_PR3,
    CMCS_125_4G4G_FAA_MP1,

    CMCS_128_4G4G_FAA_PR1 = 0x87,
    CMCS_128_4G4G_FAA_PR2,
    CMCS_128_4G4G_FAA_PR3,
    CMCS_128_4G4G_FAA_MP1,
    /* FIH, Debbie, 2010/05/24 { */

    /* FM6 family */
    /*CMCS_FM6_PR1 =0x100,*/
    /* FIH, Debbie, 2010/05/04 } */

    /* add for 7627 { */
    /* FIH, Debbie, 2010/05/04 { */
    /* modify 7627 start index because 7227 hwid is not enough */
    /*** 7627 definition start from 0x500 ***/
    CMCS_7627_ORIG_EVB1     = 0x500,
    /* FIH, Debbie, 2010/05/04 } */
    CMCS_7627_F905_PR1,      // 100 k
    CMCS_7627_F905_PR2,      // 130 k
    CMCS_7627_F905_PR3,
    CMCS_7627_F905_PR4,
    CMCS_7627_F905_PR5,
    CMCS_7627_F913_PR1,      // 160 k
    CMCS_7627_F913_PR2,      // 180 k
    CMCS_7627_F913_PR3,
    CMCS_7627_F913_PR4,
    CMCS_7627_F913_PR5,
    CMCS_7627_F913_MP1_W,
    CMCS_7627_F913_MP1_C_G,
    /* FIH, Debbie, 2010/05/24 { */
    CMCS_7627_F913_MP1_W_4G4G,
    CMCS_7627_F913_MP1_C_G_4G4G,
    /* FIH, Debbie, 2010/05/24 } */

    /* FIH, Debbie, 2010/05/04 { */
    /* F20 family */		
    CMCS_7627_F20_PR1          =0x520,
    CMCS_7627_F20_PR2,
    CMCS_7627_F20_PR3,
    CMCS_7627_F20_MP1,

    /* FN6 family */
    /*
    CMCS_FN6_PR1               =0x600,
    CMCS_FN6_PR2,
    CMCS_FN6_PR3,
    CMCS_FN6_MP1,
    */
    /* FIH, Debbie, 2010/05/04 } */

    CMCS_HW_VERSION_MAX
}cmcs_hw_orig_version_type;
/* 7227_CR_XX end*/
/* FIH, Debbie Sun, 2009/07/15 } */

#ifdef MSM_SHARED_RAM_BASE
/* get share memory command address dynamically */
#define FIH_READ_HWID_FROM_SMEM()  fih_read_hwid_from_smem()

/* modify new hardware id */
#define FIH_READ_ORIG_HWID_FROM_SMEM()  fih_read_orig_hwid_from_smem()

/* add for Read modem mode from smem */
#define FIH_READ_NETWORK_MODE_FROM_SMEM()  fih_read_network_mode_from_smem()
    #ifdef POWER_ON_CAUSE_PROC_READ_ENTRY
    #define FIH_READ_POWER_ON_CAUSE()  fih_read_power_on_cuase_from_smem()
    #endif
#endif
#endif
/* } FIH, Charles Huang, 2009/05/18 */

/* Tells the other end of the smd channel that this end wants to recieve
 * interrupts when the written data is read.  Read interrupts should only
 * enabled when there is no space left in the buffer to write to, thus the
 * interrupt acts as notification that space may be avaliable.  If the
 * other side does not support enabling/disabling interrupts on demand,
 * then this function has no effect if called.
 */
void smd_enable_read_intr(smd_channel_t *ch);

/* Tells the other end of the smd channel that this end does not want
 * interrupts when written data is read.  The interrupts should be
 * disabled by default.  If the other side does not support enabling/
 * disabling interrupts on demand, then this function has no effect if
 * called.
 */
void smd_disable_read_intr(smd_channel_t *ch);

#endif
