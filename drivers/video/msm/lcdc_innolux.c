
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/delay.h>

#include <linux/proc_fs.h>
#include "msm_fb.h"
#include <asm/gpio.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>

/*
    Macro for easily apply LCM vendor's init code
*/
#define WRITE9BITS(data,c) \
do { \
    u16 buf=data; \
    buf|=(c<<8); \
    ret=spi_write_9bits(spi, &buf, 2); \
} while(0)

#define Delayms(n) mdelay(n)
#define WriteI(n) WRITE9BITS(n,0)
#define WriteD(n) WRITE9BITS(n,1)
#define LCD_ILI9481_CMD(n) WRITE9BITS(n,0)
#define LCD_ILI9481_INDEX(n) WRITE9BITS(n,1)

#define LCDSPI_PROC_FILE        "driver/lcd_spi"
#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

static struct mutex               innolux_dd_lock;
static struct list_head           dd_list;
static struct proc_dir_entry	  *lcdspi_proc_file;
static struct spi_device	  *g_spi;

struct driver_data {
        struct input_dev         *ip_dev;
        struct spi_device        *spi;
        char                      bits_per_transfer;
        struct work_struct        work_data;
        bool                      config;
        struct list_head          next_dd;
};


extern int spi_write_9bits_then_read_8bits(struct spi_device *spi,
		const u16 *txbuf, unsigned n_tx,
		u8 *rxbuf, unsigned n_rx);

/**
 * spi_write - SPI synchronous write with 9 bits per word
 * @spi: device to which data will be written
 * @buf: data buffer
 * @len: data buffer size
 */
static inline int
spi_write_9bits(struct spi_device *spi, const u16 *buf, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= buf,
			.len		= len,
			.bits_per_word =9,
		};
		
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

void reset_lcmic(void)
{
    int gpio_lcd_reset=103;
    int ret=0;
    
    /* LCD reset pin: pull high */
    ret = gpio_direction_output(gpio_lcd_reset, 1);
    mdelay(1);

    ret = gpio_direction_output(gpio_lcd_reset, 0);
    mdelay(10);
    
    ret = gpio_direction_output(gpio_lcd_reset, 1);
    mdelay(10);

}


int ilitek_config(struct driver_data *dd, struct spi_device *spi)
{
    int ret=0;
    reset_lcmic();
    //************* Start Initial Sequence **********//
    LCD_ILI9481_CMD(0x11);    
    Delayms(20);
    
    //lcm_innolux: for polarity of den, hsync, vsync, pclk
    LCD_ILI9481_CMD(0xC6);    
    //LCD_ILI9481_INDEX(0x1B);    
    LCD_ILI9481_INDEX(0x03); //chandler_boot_lcm: hsync, vsync active low  
    
    LCD_ILI9481_CMD(0xD0);   
    LCD_ILI9481_INDEX(0x07);
    LCD_ILI9481_INDEX(0x42);
    LCD_ILI9481_INDEX(0x1C);

    LCD_ILI9481_CMD(0xD1);   
    LCD_ILI9481_INDEX(0x00);
    LCD_ILI9481_INDEX(0x02);
    LCD_ILI9481_INDEX(0x0F);

    /* Power_Setting for Normal Mode */
    LCD_ILI9481_CMD(0xD2);   
    LCD_ILI9481_INDEX(0x01);
    //lcm_innolux: 
//    LCD_ILI9481_INDEX(0x02); 
    LCD_ILI9481_INDEX(0x22); 

    LCD_ILI9481_CMD(0xC0);   
    LCD_ILI9481_INDEX(0x10);
    LCD_ILI9481_INDEX(0x3b);
    LCD_ILI9481_INDEX(0x00);

    LCD_ILI9481_INDEX(0x02);
    LCD_ILI9481_INDEX(0x11);

    /* 
    // lcm_innolux: from innolux
    LCD_ILI9481_INDEX(0x00);
    LCD_ILI9481_INDEX(0x01);
    LCD_ILI9481_CMD(0x28);   
    */
    // for internal clock
    LCD_ILI9481_CMD(0xC5);   
	WriteD(0x03);  //72Mhz
	//WriteD(0x02);  //85Mhz
	//WriteD(0x01);  //100Mhz
	//WriteD(0x00);  //125Mhz

    //lcm_innolux: Display timing (form innolux) 
    WriteI(0xC1);
    WriteD(0x10);  
    WriteD(0x10); 
    WriteD(0x88);
/*
    LCD_ILI9481_CMD(0xC8);   
    LCD_ILI9481_INDEX(0x00);
    LCD_ILI9481_INDEX(0x30);
    LCD_ILI9481_INDEX(0x36);
    LCD_ILI9481_INDEX(0x45);
    LCD_ILI9481_INDEX(0x04);
    LCD_ILI9481_INDEX(0x16);
    LCD_ILI9481_INDEX(0x37);
    LCD_ILI9481_INDEX(0x75);
    LCD_ILI9481_INDEX(0x77);
    LCD_ILI9481_INDEX(0x54);
    LCD_ILI9481_INDEX(0x0F);
    LCD_ILI9481_INDEX(0x00);
*/
 //===Gamma setting =====(form innolux)
 
     WriteI(0xC8);
     WriteD(0x00);  
     WriteD(0x77);  
     WriteD(0x06);  
     WriteD(0x42);  
     WriteD(0x0F);  
     WriteD(0x00);  
     WriteD(0x17);  
     WriteD(0x00);  
     WriteD(0x77);  
     WriteD(0x24);  
     WriteD(0x00);  
     WriteD(0x03);       

    LCD_ILI9481_CMD(0xF0);   
    LCD_ILI9481_INDEX(0x00);

    LCD_ILI9481_CMD(0xF6);   
    LCD_ILI9481_INDEX(0x80);

    LCD_ILI9481_CMD(0xF3);   
    LCD_ILI9481_INDEX(0x02);
    LCD_ILI9481_INDEX(0x1A);

    LCD_ILI9481_CMD(0xF4);   //For AC RGB pre-buffer
    LCD_ILI9481_INDEX(0x4D);
    LCD_ILI9481_INDEX(0x2C);

    LCD_ILI9481_CMD(0x3A);   //
    LCD_ILI9481_INDEX(0x55);  //16bit

    LCD_ILI9481_CMD(0xF1);   //External clock division
    LCD_ILI9481_INDEX(0x0B);


    LCD_ILI9481_CMD(0xF7);   
    LCD_ILI9481_INDEX(0x80);

    LCD_ILI9481_CMD(0x36);   
    LCD_ILI9481_INDEX(0xca); //rotate degree 180

    LCD_ILI9481_CMD(0x2A);   
    LCD_ILI9481_INDEX(0x00);
    LCD_ILI9481_INDEX(0x00);
    LCD_ILI9481_INDEX(0x01);
    LCD_ILI9481_INDEX(0x3F);

    LCD_ILI9481_CMD(0x2B);
    LCD_ILI9481_INDEX(0x00);
    LCD_ILI9481_INDEX(0x00);
    LCD_ILI9481_INDEX(0x01);
    LCD_ILI9481_INDEX(0xDF);


    Delayms(120);
    LCD_ILI9481_CMD(0x29);

    LCD_ILI9481_CMD(0xB4);   //Switch to RGB I/F
    //LCD_ILI9481_INDEX(0x11);  // select PCLK
	LCD_ILI9481_INDEX(0x10);  // internal clock

    Delayms(120);
    LCD_ILI9481_CMD(0x2C); //send 0x2c before write data

    return 0;
}


int innolux_config_v2(struct driver_data *dd, struct spi_device *spi)
{
    int ret=0;
    //reset_lcmic();
    
    //VCI=2.8V;
    WriteI(0x11);   
    Delayms(20);

    //lcm_innolux: for polarity of den, hsync, vsync, pclk
    LCD_ILI9481_CMD(0xC6);    
    //LCD_ILI9481_INDEX(0x1B);    
    LCD_ILI9481_INDEX(0x03); //chandler_boot_lcm: hsync, vsync active low  

    
    //=====POWER Setting==//
    WriteI(0xD0);     
    WriteD(0x07);   
    WriteD(0x40);                                        
    WriteD(0x1A);    
         
  //===VCOM Setting=======//
     WriteI(0xD1);
     WriteD(0x00);  
     WriteD(0x0A);    
     WriteD(0x12);    

  
  //====POWER Setting For normal mode ===//
     WriteI(0xD2);
     WriteD(0x01); 
     WriteD(0x00); 
    
       
  // ==== Panel driver setting =====//
  
     WriteI(0xC0);
     WriteD(0x10);
     WriteD(0x3B); 
     WriteD(0x00); 
     WriteD(0x02);
     WriteD(0x11);
     
    //==== display off  =====//
     WriteI(0x28);  
    
  
  //====Frame rate and invertion control===//

     WriteI(0xC5);
     //WriteD(0x03);  //72Mhz
	 WriteD(0x02);  //85Mhz
	 //WriteD(0x01);  //100Mhz
	 //WriteD(0x00);  //125Mhz
    
     WriteI(0xC1);
     WriteD(0x10);  
     WriteD(0x10); 
     WriteD(0x88);
 

  //===Interface Pixel mode=====//?
  
    WriteI(0x0C);
    WriteD(0x55); //16 bits mode //chandler
    Delayms(50);

    //lcm_innolux
    WriteI(0x3A);
    WriteD(0x55);
 

 //===Gamma setting =====// 
     WriteI(0xC8);
     WriteD(0x00);  
     WriteD(0x26);  
     WriteD(0x03);  
     WriteD(0x34);  
     WriteD(0x06);  
     WriteD(0x06);  
     WriteD(0x47);  
     WriteD(0x15);  
     WriteD(0x77);  
     WriteD(0x43);  
     WriteD(0x07);  
     WriteD(0x04);    
     //====chandler mark===//
//     WriteI(0xC6);  
//     WriteD(0x82);  

     WriteI(0xB4);
     WriteD(0x11);  //use PCLK


     WriteI(0xF3);
     WriteD(0x20);
     WriteD(0x1F);

     WriteI(0x36);
     WriteD(0x09);
     
     WriteI(0xB3);
     WriteD(0x02);
     WriteD(0x00);
     WriteD(0x00);
     WriteD(0x21);
          
     WriteI(0x29);
     Delayms(20);
     WriteI(0x2C);
	
    return ret;	
}

int innolux_config_v3(struct driver_data *dd, struct spi_device *spi)
{

    int ret=0;
    
    //VCI=2.8V;
    WriteI(0x11);   
    Delayms(20);
    
    //=====POWER Setting==//
    WriteI(0xD0);     
    WriteD(0x07);   
    WriteD(0x40);                                        
    WriteD(0x1A);    
         
  //===VCOM Setting=======//
     WriteI(0xD1);
     WriteD(0x00);  
     WriteD(0x05);    
     WriteD(0x10);    

  
  //====POWER Setting For normal mode ===//
     WriteI(0xD2);
     WriteD(0x01); 
     WriteD(0x00); 
    
       
  // ==== Panel driver setting =====//
  
     WriteI(0xC0);
     WriteD(0x10);
     WriteD(0x3B); 
     WriteD(0x00); 
     WriteD(0x02);
     WriteD(0x11);
     
    //==== display off  =====//
     //WriteI(0x28);  
    
  
  //====Frame rate and invertion control===//

     WriteI(0xC5);
     //WriteD(0x03);  //72Mhz
	 WriteD(0x02);  //85Mhz
	 //WriteD(0x01);  //100Mhz
	 //WriteD(0x00);  //125Mhz
    
     WriteI(0xC1);
     WriteD(0x10);  
     WriteD(0x10); 
     WriteD(0x88);
 

  //===Interface Pixel mode=====//?
  
    //lcm_innolux
    WriteI(0x3A);
    WriteD(0x55);
 

 //===Gamma setting =====// 
     WriteI(0xC8);
     WriteD(0x00);  
     WriteD(0x37);  
     WriteD(0x11);  
     WriteD(0x21);  
     WriteD(0x04);  
     WriteD(0x02);  
     WriteD(0x66);  
     WriteD(0x04);  
     WriteD(0x77);  
     WriteD(0x12);  
     WriteD(0x05);  
     WriteD(0x00);    

     /* lcm_innolux: for polarity of den, hsync, vsync, pclk */
     WriteI(0xC6);    
     //LCD_ILI9481_INDEX(0x82);    
     //WriteD(0x03); //chandler_boot_lcm: hsync, vsync active low  
     WriteD(0x01); //chandler_boot_lcdc


     WriteI(0xB4);
     WriteD(0x11);  //use PCLK


     WriteI(0xF3);
     WriteD(0x20);
     WriteD(0x07);

     WriteI(0x36);
     WriteD(0x09);
     
     WriteI(0xB3);
     WriteD(0x02);
     WriteD(0x00);
     WriteD(0x00);
     WriteD(0x21);
          
     WriteI(0x29);
     Delayms(20);
     WriteI(0x2C);
	
    return ret;	

}

inline int innolux_config(struct driver_data *dd, struct spi_device *spi)
{
    innolux_config_v3(dd,spi);
    return 0;
}

int simple_config(struct driver_data *dd, struct spi_device *spi)
{
    int ret=0;
    reset_lcmic();
    LCD_ILI9481_CMD(0x11);  //exit sleep mode    
    Delayms(20);
//    LCD_ILI9481_CMD(0x3A);   // 16bits
//    LCD_ILI9481_INDEX(0x55); // 16bits
    Delayms(120);
    LCD_ILI9481_CMD(0x29);  // display on
    
//    LCD_ILI9481_CMD(0xc0);  // panel driving
//    LCD_ILI9481_INDEX(0x10);  // color inversion    
    return 0;    
}

int get_power_mode(struct driver_data *dd, struct spi_device *spi)
{

    int ret=0;
    reset_lcmic();
    LCD_ILI9481_CMD(0x11);    
    Delayms(20);
    LCD_ILI9481_CMD(0x0a);  //get power mode  
    LCD_ILI9481_CMD(0x00);  //nop  
    LCD_ILI9481_CMD(0x00);  //nop  

    return 0;    
}

int get_pixel_mode(struct driver_data *dd, struct spi_device *spi)
{

    int ret=0;    
    u16 txbuf; // just use 9 bits of LSB
    u8 rxbuf[2]={0};
/*
    reset_lcmic();
    printk(KERN_INFO "lcm_innolux: get_pixel_mode()\n");

    LCD_ILI9481_CMD(0x11);    // exit sleep mode
    Delayms(20);

    LCD_ILI9481_CMD(0x3A);   // 15bits
    LCD_ILI9481_INDEX(0x55); // 15bits
*/
    txbuf=0x0c;
    spi_write_9bits_then_read_8bits(spi, &txbuf, sizeof(txbuf), rxbuf, 2);

    return ret;    
}

int sleep_config(struct driver_data *dd, struct spi_device *spi)
{

    int ret=0;    
    u16 txbuf; // just use 9 bits of LSB
    u8 rxbuf[2]={0};

    reset_lcmic();

    LCD_ILI9481_CMD(0x11);    // exit sleep mode
    Delayms(20);

    LCD_ILI9481_CMD(0x3A);   // 15bits
    LCD_ILI9481_INDEX(0x55); // 15bits

    txbuf=0x0c;
    spi_write_9bits_then_read_8bits(spi, &txbuf, sizeof(txbuf), rxbuf, 2);

    return 0;    
}

inline int fih_lcdc_config(struct driver_data *dd, struct spi_device *spi)
{

    int ret=0;

    //chandler_porting {
    //reset_lcmic();
    //innolux_config(0,spi);
    //chandler_porting }
    return ret;
}

void fih_lcdc_enter_sleep(void)
{
    int ret=0;
    struct spi_device *spi=g_spi;
    
    if (!spi){
        return;
    }

    
	WriteI(0x28);	// display off
	WriteI(0x10); // enter sleep

}

/*  Add this flag to skip 1st lcd on, 
    because bootload has init before. 
*/
//chandler_boot_lcdc
static int panel_first_on=1; 

void fih_lcdc_exit_sleep(void)
{

    struct spi_device *spi=g_spi;

    //chandler_boot_lcdc
	if(panel_first_on==1) {
    	panel_first_on=0;
        return;
    }
    
    if (!spi){
        return;
    }

    
    //chandler_boot_lcm
    
    reset_lcmic();
    innolux_config(0,spi);
    
    //Delayms(100);
}

EXPORT_SYMBOL(fih_lcdc_enter_sleep);
EXPORT_SYMBOL(fih_lcdc_exit_sleep);


static int lcdc_spi_probe(struct spi_device *spi)
{
	struct driver_data *dd;
    int                 rc=0;

	dd = kzalloc(sizeof(struct driver_data), GFP_KERNEL);
    if (!dd) {
        rc = -ENOMEM;
        goto probe_exit;
    }
	
	mutex_lock(&innolux_dd_lock);
    list_add_tail(&dd->next_dd, &dd_list);
    mutex_unlock(&innolux_dd_lock);
    dd->spi = spi;
	g_spi= spi;
	
	rc = fih_lcdc_config(dd,spi);
	if (rc)
        goto probe_err_cfg;

	return 0;
	
probe_err_cfg:
    mutex_lock(&innolux_dd_lock);
    list_del(&dd->next_dd);
    mutex_unlock(&innolux_dd_lock);
probe_exit:
    return rc;	
}

static struct spi_driver lcdc_spi_drv = {
    .driver = {
            .name  = "lcdc_spi",
    },
    .probe         = lcdc_spi_probe,
};

static ssize_t lcdspi_proc_read(struct file *filp,
                char *buffer, size_t length, loff_t *offset)
{
        
    printk( KERN_INFO "do nothing\n");

    return 0;
}

static ssize_t lcdspi_proc_write(struct file *filp,
                const char *buff, size_t len, loff_t *off)
{
    char msg[ 512];

    if( len > 512)
            len = 512;

    if( copy_from_user( msg, buff, len))
            return -EFAULT;

	if( strncmp("innolux", msg, strlen("innolux")) == 0)
	{
		innolux_config( NULL, g_spi);
	}else if(strncmp("ilitek", msg, strlen("ilitek")) == 0){
        ilitek_config( NULL, g_spi);
	}else if(strncmp("simple", msg, strlen("simple")) == 0){
        simple_config( NULL, g_spi);
	}else if(strncmp("reset", msg, strlen("reset")) == 0){
        reset_lcmic();
	}else if(strncmp("power", msg, strlen("power")) == 0){
        get_power_mode( NULL, g_spi);
	}else if(strncmp("pixel", msg, strlen("pixel")) == 0){
        get_pixel_mode( NULL, g_spi);
	}else if(strncmp("sleep", msg, strlen("sleep")) == 0){
        sleep_config( NULL, g_spi);
	}else if(strstr( msg, "pixel") ){

	}

    return len;
}


static struct file_operations lcdspi_proc_fops = {
    .owner          = THIS_MODULE,
    .read           = lcdspi_proc_read,
    .write          = lcdspi_proc_write,
};

static int create_lcdspi_proc_file(void)
{
    lcdspi_proc_file = create_proc_entry( LCDSPI_PROC_FILE, 0644, NULL);
    if (!lcdspi_proc_file) {
        printk(KERN_INFO "Create proc file for LCD SPI failed\n");
        return -ENOMEM;
    }

    printk( KERN_INFO "LCD SPI proc OK\n" );
    lcdspi_proc_file->proc_fops = &lcdspi_proc_fops;
    return 0;
}
extern int fih_lcm_is_mddi_type(void);
extern void lcdc_clk_off(void);
static int __init lcdc_innolux_init(void)
{
	int ret;
	struct msm_panel_info pinfo;
	
    
    if(fih_lcm_is_mddi_type()){
        lcdc_clk_off();        
        ret=-ENODEV;
        goto err;
    }
    
	pinfo.xres = 320;
	pinfo.yres = 480;
	pinfo.type = LCDC_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 16;
	pinfo.fb_num = 2;
    
    if( (FIH_READ_HWID_FROM_SMEM() >= CMCS_RTP_PR2) ) {
        pinfo.clk_rate = 11360000;
    }else{
        pinfo.clk_rate = 8192000;
    }

    
/* FIH_ADQ added by Guorui*/
	pinfo.bl_max=100;
	pinfo.bl_min=0;
/* FIH_ADQ */

	pinfo.lcdc.h_back_porch = 20;
	pinfo.lcdc.h_front_porch = 40;
	pinfo.lcdc.h_pulse_width = 10;
	
	pinfo.lcdc.v_back_porch = 2;
	pinfo.lcdc.v_front_porch = 4;
	pinfo.lcdc.v_pulse_width = 2;

	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;	

	ret = lcdc_device_register(&pinfo);
	if (ret)
		printk(KERN_ERR "%s: failed to register device!\n", __func__);

    INIT_LIST_HEAD(&dd_list);
    mutex_init(&innolux_dd_lock);

    create_lcdspi_proc_file();

    ret = spi_register_driver(&lcdc_spi_drv);
err:    return ret;
}

static void __exit lcdc_innolux_exit(void)
{
    spi_unregister_driver(&lcdc_spi_drv);
}

module_init(lcdc_innolux_init);
module_exit(lcdc_innolux_exit);

