/* drivers/video/msm/mddi_wintek.c
 *
 * Copyright (C) 2008 QUALCOMM Incorporated.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * except where indicated.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <asm/gpio.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>

/* FIH, ChandlerKang ,09.12.14 { */
//chandler_failon
static int mddi_write_ret=0;
extern u32 mddi_msg_level;
extern void mddi_host_reinit(void);
extern int mddi_host_register_write_non_block(uint32 reg_addr,uint32 reg_val,boolean wait, 
                                    mddi_llist_done_cb_type done_cb, mddi_host_type host);

#undef mddi_queue_register_write
#define mddi_queue_register_write(reg, val, wait, sig) \
	mddi_write_ret=mddi_host_register_write_non_block(reg, val, wait, NULL, MDDI_HOST_PRIM)

/* FIH, ChandlerKang ,09.12.14 } */

static int wintek_lcd_on(struct platform_device *pdev);
static int wintek_lcd_off(struct platform_device *pdev);

/* FIH, ChandlerKang, 10.2.10, for innolux mddi panel { */
#define FLAG_INL 
#define Register_access(x,y) mddi_queue_register_write(x,y,FALSE,0)
#define Delayms(n) mdelay(n)
enum mddi_id {
    MDDI_ID_WINTEK=0,
    MDDI_ID_INL
};

static int mddi_panel_id;
/* FIH, ChandlerKang, 10.2.10, for innolux mddi panel } */

/* for power init */
#define REG_8357_POWER_CTRL1 0x1A
#define REG_8357_POWER_CTRL2 0x1B

#define REG_8357_POWER_VCOM1 0x23
#define REG_8357_POWER_VCOM2 0x24
#define REG_8357_POWER_VCOM3 0x25

/* for display */
#define REG_8357_DISPLAY_MODE 0x01
#define REG_8357_MEMORY_ACCESS 0x16
#define REG_8357_COL_ACCESS 0x17

void panel_on_wintek_24pins(void);
void panel_on_wintek_30pins(void);
void panel_off_wintek(void);

#ifdef FLAG_INL
void panel_on_inl_30pins(void);
void panel_off_inl(void);
#endif

static int lcm_is_mddi_type=0;

int fih_lcm_is_mddi_type(void)
{
    return lcm_is_mddi_type;
}
EXPORT_SYMBOL(fih_lcm_is_mddi_type);


/*  Add this flag to skip 1st lcd on, 
    because bootload has init before. 
*/
static int panel_first_on=1; 

static int wintek_lcd_on(struct platform_device *pdev)
{
    // lcm_wintek
	/* Set the MDP pixel data attributes for Primary Display */
    
#if 1  //workaround for baseline release
	if(panel_first_on==1) {
        goto ignore;
    }
#endif 

    if( (FIH_READ_HWID_FROM_SMEM() >= CMCS_CTP_PR1 && FIH_READ_HWID_FROM_SMEM() <= CMCS_CTP_MP3 ) ||
        (FIH_READ_HWID_FROM_SMEM() >= CMCS_7627_PR1 && FIH_READ_HWID_FROM_SMEM() <= CMCS_HW_VER_MAX ))
    {
/* FIH, ChandlerKang ,09.12.14 { */
        //chandler_failon
        int retry=0;
RETRY:    

/* FIH, ChandlerKang, 10.2.10, for innolux mddi panel { */    
        if(mddi_panel_id == MDDI_ID_INL)
            panel_on_inl_30pins();
        else
            panel_on_wintek_30pins();
/* FIH, ChandlerKang, 10.2.10, for innolux mddi panel } */

        if(mddi_write_ret==-1 && !retry){
            mddi_write_ret=0;
    	    mddi_host_reinit();
    	    retry=1;
            goto RETRY;
    	}
/* FIH, ChandlerKang ,09.12.14 } */
    }else{
        panel_on_wintek_24pins();
    }
ignore:    
    panel_first_on=0;
    
	return 0;
}

static int wintek_lcd_off(struct platform_device *pdev)
{
    
/* FIH, ChandlerKang, 10.2.10, for innolux mddi panel { */        
    if(mddi_panel_id == MDDI_ID_INL)
        panel_off_inl();    
    else
        panel_off_wintek();
/* FIH, ChandlerKang, 10.2.10, for innolux mddi panel } */    

	return 0;
}

void panel_off_wintek(void)
{

	//chandler_fix
    mddi_queue_register_write(0xFF,0x02,FALSE,0);	
    mddi_queue_register_write(0x17,0x03,FALSE,0);		
    mddi_queue_register_write(0xFF,0x00,FALSE,0);		
    //printk(KERN_ERR "lcm_wintek: clear GRAM!!\n");
	
    /* Perform Display Off Sequence  */
    
    mddi_queue_register_write(0x28,0x38,FALSE,0);
    mddi_wait(10);
    mddi_queue_register_write(0x28,0x24,FALSE,0);
    mddi_wait(10);
    mddi_queue_register_write(0x28,0x04,FALSE,0);
    mddi_wait(10);
    
    /* Perform Power Off Sequence  */
    mddi_queue_register_write(0x1F,0x90,FALSE,0);
    mddi_wait(2);

    mddi_queue_register_write(0x1F,0x88,FALSE,0);
    mddi_queue_register_write(0x1C,0x00,FALSE,0);
    mddi_queue_register_write(0x1F,0x89,FALSE,0);

    /* lcm_wintek MDDI link shutdown */
    mddi_queue_shutdown_write(FALSE,0);

}

void panel_off_inl(void)
{
 	//chandler_fix
    Register_access(0xFF,0x02);	
    Register_access(0x17,0x03);		
    //printk(KERN_ERR "lcm_wintek: clear GRAM!!\n");

    Register_access(0x00FF,0x0000);//
    Register_access(0x0028,0x0038);//
    Delayms(40);
    Register_access(0x0028,0x0004);//
    Register_access(0x001F,0x0090);//
    Delayms(5);
    Register_access(0x001F,0x0088);//
    Register_access(0x001C,0x0000);//
    Register_access(0x001F,0x0089);//
    Register_access(0x0019,0x0000);//
    
    /* lcm_wintek MDDI link shutdown */
    mddi_queue_shutdown_write(FALSE,0);
}

static void reset_mddi_lcm(void)
{

    int ret;
    int gpio_lcd_reset=103;
    int gpio_lcd_reset_a=78;   //chandler, 2009/06/08

    /*
        For 7627 mddi, 
    */
    int gpio_lcd_7627_power=112;   //LCM_VCI_EN

    if( FIH_READ_HWID_FROM_SMEM() >= CMCS_7627_PR1 ){
        ret = gpio_direction_output(gpio_lcd_7627_power, 1);
        mdelay(10);

    }

    /* LCD reset pin: pull high */
    gpio_request(gpio_lcd_reset,"mddi_reset");
    ret = gpio_direction_output(gpio_lcd_reset, 0);
    mdelay(10);

    ret = gpio_direction_output(gpio_lcd_reset, 1);
    mdelay(10);
    gpio_free(gpio_lcd_reset);
    
    /* Chandler, 2009/06/08 { */
    /* { for mddi rework version: use gpio 78 as LCM_RST*/

    gpio_request(gpio_lcd_reset_a,"mddi_reset");
    if( FIH_READ_HWID_FROM_SMEM() == CMCS_CTP_PR1 ){
        ret = gpio_direction_output(gpio_lcd_reset_a, 0);
        mdelay(10);
        
        ret = gpio_direction_output(gpio_lcd_reset_a, 1);
        mdelay(10);
    }
    gpio_free(gpio_lcd_reset_a);

    /* } for mddi rework version: use gpio 78 as LCM_RST*/
    /* Chandler, 2009/06/08 } */

}

void panel_on_inl_30pins(void)
{

    reset_mddi_lcm();

    printk(KERN_INFO "panel_on_inl_30-10.01.21: %s()\n",__func__);        


    Register_access(0x00ff,0x0000);
    Delayms(20);
    Register_access(0x001a,0x0004);
    Register_access(0x001b,0x001c);//VRH[3:0] 22-VREG1 = 5.0V;1E=4.8v
                               //VREG 1c 4.75v 
    Register_access(0x0023,0x009f);
    Register_access(0x0024,0x0069);//VCOMH 64=4.0V   89=4.63v   93=4.8 
                               // original 69
    Register_access(0x0025,0x0063);//VCOML 3C=-1.6V  92=0.32v   7f=-0.6
                               // original 63
    Register_access(0x0019,0x0001);
    Delayms(10);
    Register_access(0x001f,0x008a);
    Register_access(0x0001,0x0000);
    Register_access(0x001c,0x0005);
    Register_access(0x001f,0x0082);
    Delayms(10);
    Register_access(0x001f,0x0092);
    Delayms(10);
    Register_access(0x001f,0x00d4);//D2 DDVDH = 6.1, D4 DDVDH=5.1;

    Register_access(0x0002,0x0000);
    Register_access(0x0003,0x0000);
    Register_access(0x0004,0x0001);
    Register_access(0x0005,0x003f);
    Register_access(0x0006,0x0000);
    Register_access(0x0007,0x0000);
    Register_access(0x0008,0x0001);
    Register_access(0x0009,0x00df);
    Register_access(0x0016,0x0009);//
    Register_access(0x0017,0x0066);
    Register_access(0x0018,0x0000);//OSC control 44-60Hz;22-70HZ;00-90HZ;
    Register_access(0x001d,0x0000);
    Register_access(0x001e,0x0000);
    Register_access(0x0026,0x0033);
    Register_access(0x0027,0x0001);
    Register_access(0x0029,0x0000);
    Register_access(0x002a,0x0000);
    Register_access(0x002b,0x000a);
    Register_access(0x002c,0x000a);
    Register_access(0x002d,0x0020);
    Register_access(0x002e,0x00a3);
    Register_access(0x002f,0x0000);
    Register_access(0x0031,0x0000);
    Register_access(0x0032,0x0000);
    Register_access(0x0033,0x0008);
    Register_access(0x0034,0x0002);
    Register_access(0x0036,0x000a);



    Register_access(0x0040,0x0001); //vrp0
    Register_access(0x0041,0x0012); //vrp1
    Register_access(0x0042,0x0019); //vrp2
    Register_access(0x0043,0x0023); //vrp3
    Register_access(0x0044,0x0028); //vrp4


    Register_access(0x0045,0x003d); //original 3a
                                  //limit 2a  
                                  //vrp5
    Register_access(0x0046,0x000b); //prp0
    Register_access(0x0047,0x005f);//5e
                                  //prp1 
    Register_access(0x0048,0x0000); //pkp0 
    Register_access(0x0049,0x000b);//08 pkp1
    Register_access(0x004A,0x000d);//09 pkp2
    Register_access(0x004B,0x000d);//0a pkp3
    Register_access(0x004C,0x001b);//15 pkp4
    //////////////////////////////////////


    Register_access(0x0050,0x0004); //original 01
                                  //limit 1f
                                  //totalsum with 45h => 3b
                                  //vrn0 
    Register_access(0x0051,0x0021); //vrn1
    Register_access(0x0052,0x0024); //vrn2
    Register_access(0x0053,0x0034); //vrn3
    Register_access(0x0054,0x0037); //vrn4
    Register_access(0x0055,0x003f); //vrn5

    Register_access(0x0056,0x0029);//2a prn0 
    Register_access(0x0057,0x007f);  // prn1

    Register_access(0x0058,0x0004);//02 pkn0
    Register_access(0x0059,0x0012);//18 pkn1
    Register_access(0x005A,0x0019);
    Register_access(0x005B,0x001b);
    Register_access(0x005C,0x001a);//   pkn4 
    Register_access(0x005D,0x0055);



    Register_access(0x0060,0x0008);
    Register_access(0x00f2,0x0000);
    Register_access(0x00e4,0x001f);
    Register_access(0x00e5,0x001f);
    Register_access(0x00e6,0x0020);
    Register_access(0x00e7,0x0000);
    Register_access(0x00e8,0x00d1);
    Register_access(0x00e9,0x00c0);
    Register_access(0x0028,0x0038);

    Delayms(50);

    Register_access(0x0028,0x003c);
    Register_access(0x0080,0x0000);
    Register_access(0x0081,0x0000);
    Register_access(0x0082,0x0000);
    Register_access(0x0083,0x0000);


    Register_access(0x0022,0x0000);
}

void panel_on_wintek_30pins(void)
{
    // init code v.104

    /*  -------------------------
        Perform Power On Sequence 
        -------------------------
    */
    reset_mddi_lcm();
    
    
    mddi_queue_register_write(0xFF,0x00,FALSE,0);	
	mddi_queue_register_write(0x16,0x08,FALSE,0);
	mddi_queue_register_write(0xE2,0x00,FALSE,0);
	mddi_queue_register_write(0xE3,0x00,FALSE,0);	  
	mddi_queue_register_write(0xF2,0x00,FALSE,0);	  
	
	mddi_queue_register_write(0xE4,0x1c,FALSE,0);	  
	mddi_queue_register_write(0xE5,0x1c,FALSE,0);	  
	mddi_queue_register_write(0xE6,0x00,FALSE,0);	 
	mddi_queue_register_write(0xE7,0x1c,FALSE,0);
	mddi_queue_register_write(0x19,0x01,FALSE,0);
	
    mddi_wait(10);   	
    
    mddi_queue_register_write(0x29,0x01,FALSE,0);	
	mddi_queue_register_write(0x18,0x22,FALSE,0);	// for internal clock 
	mddi_queue_register_write(0x2A,0x00,FALSE,0);	
    mddi_queue_register_write(0x2B,0x13,FALSE,0);	
	mddi_queue_register_write(0x02,0x00,FALSE,0);	
	mddi_queue_register_write(0x03,0x00,FALSE,0);	
	mddi_queue_register_write(0x04,0x01,FALSE,0);	
	mddi_queue_register_write(0x05,0x3F,FALSE,0);	
	mddi_queue_register_write(0x06,0x00,FALSE,0);	
	mddi_queue_register_write(0x07,0x00,FALSE,0);	
	mddi_queue_register_write(0x08,0x01,FALSE,0);	
 	mddi_queue_register_write(0x09,0xDF,FALSE,0);
 
	
	mddi_queue_register_write(0x24,0x91,FALSE,0);	
	mddi_queue_register_write(0x25,0x8a,FALSE,0);	
	mddi_queue_register_write(0x1B,0x30,FALSE,0);
	mddi_wait(10);	
	mddi_queue_register_write(0x1D,0x22,FALSE,0);
	mddi_wait(10);	
	
	//gamma setting 	
	
	mddi_queue_register_write(0x40,0x00,FALSE,0);	
	mddi_queue_register_write(0x41,0x3c,FALSE,0);	
	mddi_queue_register_write(0x42,0x38,FALSE,0);	
	mddi_queue_register_write(0x43,0x34,FALSE,0);//VRP3	
	mddi_queue_register_write(0x44,0x2e,FALSE,0);//VRP4	
	mddi_queue_register_write(0x45,0x2f,FALSE,0);//VRP5	
	mddi_queue_register_write(0x46,0x41,FALSE,0);//VRN0
    mddi_queue_register_write(0x47,0x7d,FALSE,0);
    mddi_queue_register_write(0x48,0x0b,FALSE,0);
    mddi_queue_register_write(0x49,0x05,FALSE,0);
    mddi_queue_register_write(0x4a,0x06,FALSE,0);
    mddi_queue_register_write(0x4b,0x12,FALSE,0);
    mddi_queue_register_write(0x4c,0x16,FALSE,0);
    mddi_queue_register_write(0x50,0x10,FALSE,0);
    
    mddi_queue_register_write(0x51,0x11,FALSE,0);//VRN1       
	mddi_queue_register_write(0x52,0x0b,FALSE,0);//VRN2	
	mddi_queue_register_write(0x53,0x07,FALSE,0);//VRN3	
	mddi_queue_register_write(0x54,0x03,FALSE,0);//VRN4	
	
	mddi_queue_register_write(0x55,0x3f,FALSE,0);//VRN5	
	mddi_queue_register_write(0x56,0x02,FALSE,0);//PRN0	
	mddi_queue_register_write(0x57,0x3e,FALSE,0);//PRN1	
	
	mddi_queue_register_write(0x58,0x09,FALSE,0);//PKP0      
	mddi_queue_register_write(0x59,0x0d,FALSE,0);//PKP1	
	mddi_queue_register_write(0x5a,0x19,FALSE,0);//PKP1	
	mddi_queue_register_write(0x5b,0x1a,FALSE,0);//PKP1	
	mddi_queue_register_write(0x5c,0x14,FALSE,0);//PKP1	
	mddi_queue_register_write(0x5d,0xc0,FALSE,0);//PKP1	

	mddi_queue_register_write(0x01,0x02,FALSE,0);//PKP1	
	mddi_queue_register_write(0x1a,0x05,FALSE,0);//PKP1	
	mddi_queue_register_write(0x1c,0x03,FALSE,0);//PKP1	
	
	mddi_wait(10);	
	mddi_queue_register_write(0x1F,0x88,FALSE,0);
	mddi_wait(10);			
	mddi_queue_register_write(0x1F,0x80,FALSE,0);
	mddi_wait(10);	
	mddi_queue_register_write(0x1F,0x90,FALSE,0);
	mddi_wait(10);	
	mddi_queue_register_write(0x1F,0xD2,FALSE,0);
	mddi_wait(10);	

/* FIH, Chandler, add for abnormal color during init. { */ 
//chandler_color
#if 0
	mddi_queue_register_write(0x28,0x08,FALSE,0);
	mddi_wait(40);	
#endif    
/* FIH, Chandler, add for abnormal color during init. } */ 

	mddi_queue_register_write(0x28,0x38,FALSE,0);
	mddi_wait(40);	
	mddi_queue_register_write(0x28,0x3C,FALSE,0);
	mddi_wait(40);	

	
	mddi_queue_register_write(0x80,0x00,FALSE,0);	
	mddi_queue_register_write(0x81,0x00,FALSE,0);	
	mddi_queue_register_write(0x82,0x00,FALSE,0);	
	mddi_queue_register_write(0x83,0x00,FALSE,0);

	
	mddi_queue_register_write(0x17,0x06,FALSE,0);//color mode
	mddi_queue_register_write(0x2d,0x1f,FALSE,0);	
	mddi_queue_register_write(0x60,0x08,FALSE,0);	
	mddi_queue_register_write(0xE8,0x90,FALSE,0);
}

// From Wintek kevin_fu.
void panel_on_wintek_24pins(void)
{

    /*  -------------------------
        Perform Power On Sequence 
        -------------------------
    */
    reset_mddi_lcm();
    
        
    mddi_queue_register_write(0xFF,0x00,FALSE,0);	
	mddi_queue_register_write(0xE2,0x0B,FALSE,0);
	mddi_queue_register_write(0xE3,0x03,FALSE,0);
	mddi_queue_register_write(0xF2,0x00,FALSE,0);	  
	mddi_queue_register_write(0xE4,0x00,FALSE,0);	  
	mddi_queue_register_write(0xE5,0x1c,FALSE,0);	  
	mddi_queue_register_write(0xE6,0x00,FALSE,0);	 
	mddi_queue_register_write(0xE7,0x1c,FALSE,0);

	  	
	mddi_queue_register_write(0x19,0x01,FALSE,0);
        mddi_wait(10);   	
	mddi_queue_register_write(0x18,0x22,FALSE,0);	
//	mddi_queue_register_write(0x18,0x44,FALSE,0);	//chandler_test
	mddi_queue_register_write(0x2A,0x00,FALSE,0);	
	mddi_queue_register_write(0x02,0x00,FALSE,0);	
	mddi_queue_register_write(0x03,0x00,FALSE,0);	
	mddi_queue_register_write(0x04,0x01,FALSE,0);	
	mddi_queue_register_write(0x05,0x3F,FALSE,0);	
	mddi_queue_register_write(0x06,0x00,FALSE,0);	
	mddi_queue_register_write(0x07,0x00,FALSE,0);	
	mddi_queue_register_write(0x08,0x00,FALSE,0);	
 	mddi_queue_register_write(0x09,0xEF,FALSE,0);
 
	
	mddi_queue_register_write(0x24,0x75,FALSE,0);	
	mddi_queue_register_write(0x25,0x55,FALSE,0);	
	mddi_queue_register_write(0x23,0x85,FALSE,0);	
	mddi_queue_register_write(0x1B,0x26,FALSE,0);
	mddi_wait(10);	
	mddi_queue_register_write(0x1D,0x22,FALSE,0);
	mddi_wait(10);	
	
	//gamma setting 	
	
	mddi_queue_register_write(0x40,0x00,FALSE,0);	
	mddi_queue_register_write(0x41,0x2D,FALSE,0);	
	mddi_queue_register_write(0x42,0x29,FALSE,0);	
	mddi_queue_register_write(0x43,0x2f,FALSE,0);//VRP3	
	mddi_queue_register_write(0x44,0x2c,FALSE,0);//VRP4	
	mddi_queue_register_write(0x45,0x30,FALSE,0);//VRP5	
	mddi_queue_register_write(0x50,0x0F,FALSE,0);//VRN0
        mddi_queue_register_write(0x51,0x13,FALSE,0);//VRN1       
	mddi_queue_register_write(0x52,0x10,FALSE,0);//VRN2	
	mddi_queue_register_write(0x53,0x16,FALSE,0);//VRN3	
	mddi_queue_register_write(0x54,0x12,FALSE,0);//VRN4	
	mddi_queue_register_write(0x55,0x3f,FALSE,0);//VRN5	
	mddi_queue_register_write(0x46,0x24,FALSE,0);//PRP0	
	mddi_queue_register_write(0x47,0x76,FALSE,0);//PRP1	
	mddi_queue_register_write(0x56,0x09,FALSE,0);//PRN0	
	mddi_queue_register_write(0x57,0x5b,FALSE,0);//PRN1	
	mddi_queue_register_write(0x48,0x00,FALSE,0);//PKP0      
	mddi_queue_register_write(0x49,0x03,FALSE,0);//PKP1	
	mddi_queue_register_write(0x4A,0x07,FALSE,0);//PKP2	
	mddi_queue_register_write(0x4B,0x11,FALSE,0);//PKP3	
	mddi_queue_register_write(0x4C,0x11,FALSE,0);//PKP4	
        mddi_queue_register_write(0x58,0x0e,FALSE,0);//PKN0        
	mddi_queue_register_write(0x59,0x0e,FALSE,0);//PKN1
	mddi_queue_register_write(0x5A,0x18,FALSE,0);//PKN2	
	mddi_queue_register_write(0x5B,0x1b,FALSE,0);//PKN3	
	mddi_queue_register_write(0x5C,0x1f,FALSE,0);//PKN4	
	mddi_queue_register_write(0x5D,0xC0,FALSE,0);

	// roate 180
       /* Chandler mark, 2009/06/08 { */
	//if (FIH_READ_HWID_FROM_SMEM() >= CMCS_RTP_PR1)
    	//mddi_queue_register_write(0x16,0xC0,FALSE,0);	 
       /* Chandler mark, 2009/06/08 } */
	
	mddi_queue_register_write(0x01,0x02,FALSE,0);	
	mddi_queue_register_write(0x1C,0x03,FALSE,0);	
	mddi_wait(10);	
	mddi_queue_register_write(0x1F,0x88,FALSE,0);
	mddi_wait(10);			
	mddi_queue_register_write(0x1F,0x80,FALSE,0);
	mddi_wait(10);	
	mddi_queue_register_write(0x1F,0x90,FALSE,0);
	mddi_wait(10);	
	mddi_queue_register_write(0x1F,0xD2,FALSE,0);
	mddi_wait(10);	
	mddi_queue_register_write(0x28,0x04,FALSE,0);
	mddi_wait(40);	
	mddi_queue_register_write(0x28,0x38,FALSE,0);
	mddi_wait(40);	
	mddi_queue_register_write(0x28,0x3C,FALSE,0);
	mddi_wait(40);	
	
	
	mddi_queue_register_write(0x80,0x00,FALSE,0);	
	mddi_queue_register_write(0x81,0x00,FALSE,0);	
	mddi_queue_register_write(0x82,0x00,FALSE,0);	
	mddi_queue_register_write(0x83,0x00,FALSE,0);

	
	mddi_queue_register_write(0x17,0x06,FALSE,0);	
	mddi_queue_register_write(0x2d,0x1f,FALSE,0);	
	mddi_queue_register_write(0x60,0x08,FALSE,0);	
	mddi_queue_register_write(0xE8,0x90,FALSE,0);
}


static int __init wintek_probe(struct platform_device *pdev)
{

    if(!fih_lcm_is_mddi_type()){
        return -ENODEV;
    }
    
	msm_fb_add_device(pdev);
    
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = wintek_probe,
	.driver = {
		.name   = "mddi_wintek_hvga",
	},
};

static struct msm_fb_panel_data wintek_panel_data = {
	.on = wintek_lcd_on,
	.off = wintek_lcd_off,
};

static struct platform_device this_device = {
	.name   = "mddi_wintek_hvga",
	.id	= 0,
	.dev	= {
		.platform_data = &wintek_panel_data,
	}
};

/* FIH, ChandlerKang, 10.2.10, for innolux mddi panel { */
void get_otp_id(uint32 id_addr,uint32 *ptr_value)
{
#if 0
    mddi_queue_register_write(0x39,id_addr,FALSE,0);  
    mddi_queue_register_write(0x3a,0x20,FALSE,0);  
    mddi_queue_register_write(0x3a,0x00,FALSE,0);  
    mddi_queue_register_read(0x3b,ptr_value,TRUE,0);  
#else
    mddi_queue_register_read(0x60+id_addr,ptr_value,TRUE,0);  
#endif
}

void check_panel_id(void)
{
#if 0
    mddi_panel_id=MDDI_ID_WINTEK;
    return;
#else

    unsigned otp_id1=0, otp_id2=0, otp_id3=0;

    get_otp_id(1,&otp_id1);
    get_otp_id(2,&otp_id2);
    get_otp_id(3,&otp_id3);    	

    if(otp_id1 == 0x01 && otp_id2 == 0x01 && otp_id3 == 0x01 ){
        mddi_panel_id=MDDI_ID_INL;
    }else{
        mddi_panel_id=MDDI_ID_WINTEK;
    }

#endif
}
/* FIH, ChandlerKang, 10.2.10, for innolux mddi panel } */

static int __init wintek_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

//#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT

	u32 id, hwid;
	hwid=FIH_READ_HWID_FROM_SMEM();

    //reset_mddi_lcm();//chandler_porting
	
    /* Chandler, 2009/06/08 { */
    /* using HW-ID to decide the lcm mddi type */
    if( (hwid >= CMCS_CTP_PR1 && hwid < CMCS_7627_EVB1 ) || (hwid >= CMCS_7627_PR1 ) ) {
    
    	lcm_is_mddi_type = 1;
    	
    }else {
            
	    id = mddi_get_client_id();
        if(id) {
            lcm_is_mddi_type=1;
        } else {
            lcm_is_mddi_type=0;
        }
    }
    /* Chandler, 2009/06/08 } */

    /* FIH, ChandlerKang, 10.2.10, for innolux mddi panel { */
    if(lcm_is_mddi_type) check_panel_id();
    /* FIH, ChandlerKang, 10.2.10, for innolux mddi panel } */

    
	ret = platform_driver_register(&this_driver);
	if (!ret) {
		pinfo = &wintek_panel_data.panel_info;
		pinfo->xres = 320;
		pinfo->yres = 480;
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		pinfo->wait_cycle = 0;
		pinfo->bpp = 18;
		pinfo->fb_num = 2;
#if 0		
		pinfo->clk_rate = 122880000;
		pinfo->clk_min = 122880000;
		pinfo->clk_max = 122880000;
		pinfo->lcd.refx100 = 6050; 
		pinfo->lcd.v_back_porch = 23;
		pinfo->lcd.v_front_porch = 20;
		pinfo->lcd.v_pulse_width = 105;
#else
		pinfo->clk_rate = 96000000;
		pinfo->clk_min =  96000000;
		pinfo->clk_max = 96000000;
		
		if(mddi_panel_id==MDDI_ID_INL){
    		pinfo->lcd.refx100 = 9000; 
			pinfo->lcd.v_back_porch = 0;
			pinfo->lcd.v_front_porch = 0;
			pinfo->lcd.v_pulse_width = 20;
    	}else{
    	    pinfo->lcd.refx100 = 7100; 
			pinfo->lcd.v_back_porch = 0;
			pinfo->lcd.v_front_porch = 30;
			pinfo->lcd.v_pulse_width = 30;
    	}
		printk(KERN_INFO "lcm_wintek: wintek_init(): refx100(%d)\n",pinfo->lcd.refx100 );					
#endif 
		pinfo->lcd.vsync_enable = TRUE;
		pinfo->lcd.hw_vsync_mode = TRUE;
		pinfo->lcd.vsync_notifier_period = 0;

		ret = platform_device_register(&this_device);
		if (ret)
			platform_driver_unregister(&this_driver);
	}
	

	return ret;
}

module_init(wintek_init);
