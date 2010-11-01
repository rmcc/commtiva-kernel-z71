/*
 * **Function command codes for io_ctl.
 * */
#define CM3602_PS_OFF			0
#define CM3602_PS_ON			1
#define CM3602_ALS_ON				2
#define CM3602_ALS_OFF				3
#define CM3602_PS_ON_SIGNAL		4
#define CM3602_ALS_READ			5
#define CM3602_PS_READ			6
#define CM3602_ACTIVATE			7
#define CM3602_Proximity_Status	8
/* FIH, Michael Kao, 2009/11/20 ++*/
/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
#define CM3602_FQC_Testing		9
/* FIH, Michael Kao, 2009/11/20 --*/
#define CM3602_OFF		10
/*
 * **CM3602 ALS & PS bit
 * */ 
#define CM3602_EN_GPIO                82 
#define CM3602_PS_GPIO           	  1 
#define CM3602_EVB1_PS_GPIO_OUT           124
#define CM3602_PR1_PS_GPIO_OUT            92

/*
 * **CM3602 ALS & PS  group
 * */
#define CM3602_PS_G			1
#define CM3602_ALS_G		2

#define get_cm3602_ps_out(x)  ((gpio_get_value(x))?"High":"Low")
#define get_cm3602_ps_status(x)  ((gpio_get_value(x))?"off":"on")
#define get_cm3602_als_status(x) ((gpio_get_value(x))?"off":"on")

static int cm3602_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

static ssize_t cm3602_read_ps(struct file *file, char *buf, size_t count, loff_t *ofs);
static int cm3602_dev_open(struct inode *inode, struct file *file);
static int cm3602_proc_open(struct inode *inode, struct file *file);
static ssize_t cm3602_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off);
static int sensor_probe(struct platform_device *pdev);
static int sensor_remove(struct platform_device *pdev);
static int sensor_suspend(struct platform_device *pdev, pm_message_t state);
static int sensor_resume(struct platform_device *pdev);
static int ALSPS_panic_handler(struct notifier_block *this, unsigned long event, void *unused);
static struct notifier_block trace_panic_notifier = {
	.notifier_call  = ALSPS_panic_handler,

};
static struct file_operations cm3602_fops = {
    .open    = cm3602_dev_open,
    .read    = cm3602_read_ps,
	.ioctl   = cm3602_ioctl,
};

static struct miscdevice cm3602_alsps_dev = {
        MISC_DYNAMIC_MINOR,
        "lightsensor",
        &cm3602_fops
};

static struct file_operations cm3602_proc_ops = {
    .owner   = THIS_MODULE,
	.open    = cm3602_proc_open,
	.read    = cm3602_read_ps,
	.write   = cm3602_proc_write,
	.llseek  = seq_lseek,
	.release = single_release,
};
static struct platform_driver ALSPS_driver = {
	.probe		= sensor_probe,
	.remove		= sensor_remove,
	.suspend    = sensor_suspend,
	.resume     = sensor_resume,
	.driver		= {
		.name = "cm3602_alsps",
	},
};

/* FIH_ADQ, Hanson Lin */
typedef enum
{
	ADIE_AD_MUX_NONE,    /* None           */
	ADIE_AD_AIN0,    /* Analog input 0 */
	ADIE_AD_MUX0 = ADIE_AD_AIN0,
	ADIE_AD_AIN1,    /* Analog input 1 */
	ADIE_AD_MUX1 = ADIE_AD_AIN1,
	ADIE_AD_AIN2,    /* Analog input 2 */
	ADIE_AD_MUX2 = ADIE_AD_AIN2,
	ADIE_AD_WIPER,   /* Analog input 7 */
	ADIE_AD_MUX7 = ADIE_AD_WIPER,
	ADIE_AD_XP_UL,   /* Analog input 3 */
	ADIE_AD_MUX3 = ADIE_AD_XP_UL,
	ADIE_AD_YP_UR,   /* Analog input 4 */
	ADIE_AD_MUX4 = ADIE_AD_YP_UR,
	ADIE_AD_YM_LR,   /* Analog input 6 */
	ADIE_AD_MUX6 = ADIE_AD_YM_LR,
	ADIE_AD_XM_LL,   /* Analog input 5 */
	ADIE_AD_MUX5 = ADIE_AD_XM_LL,
	ADIE_AD_AIN_INT, /* Analog input 8 */
	ADIE_AD_MUX8 = ADIE_AD_AIN_INT,
	ADIE_AD_MUX_INVALID
}adie_hkadc_muxsel_type;

typedef enum
{
	ADIE_ADC_RES_8_BITS,
	ADIE_ADC_RES_10_BITS,
	ADIE_ADC_RES_12_BITS,
	ADIE_ADC_RES_INVALID
}adie_adc_res_type;

typedef struct
{
	adie_hkadc_muxsel_type muxsel;
	adie_adc_res_type	res;
}adie_adc_config;

