#ifndef ELAN_I2C_H
#define ELAN_I2C_H

#include <linux/ioctl.h>

struct elan_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int (*power)(int on);
};

struct elan_i2c_sensitivity {
	int x;
	int y;
};

struct elan_i2c_resolution {
	int x;
	int y;
};

#define BI8232_IOC_MAGIC    'E'
#define BI8232_IOC_GFWVERSION   _IOR(BI8232_IOC_MAGIC, 0, int)  /* get firmware version */
#define BI8232_IOC_GPWSTATE     _IOR(BI8232_IOC_MAGIC, 1, int)  /* get power state */
#define BI8232_IOC_GORIENTATION _IOR(BI8232_IOC_MAGIC, 2, int)  /* get orientation */
#define BI8232_IOC_GRESOLUTION  _IOR(BI8232_IOC_MAGIC, 3, int)  /* get resolution */
#define BI8232_IOC_GDEEPSLEEP   _IOR(BI8232_IOC_MAGIC, 4, int)  /* get deep sleep function status */
#define BI8232_IOC_GFWID        _IOR(BI8232_IOC_MAGIC, 5, int)  /* get firmware id */
#define BI8232_IOC_GREPORTRATE  _IOR(BI8232_IOC_MAGIC, 6, int)  /* get report rate */
#define BI8232_IOC_GSENSITIVITY _IOR(BI8232_IOC_MAGIC, 7, int)  /* get sensitivity setting */
#define BI8232_IOC_SPWSTATE     _IOW(BI8232_IOC_MAGIC, 8, int)  /* change power state */
#define BI8232_IOC_SORIENTATION _IOW(BI8232_IOC_MAGIC, 9, int)  /* change orientation */
#define BI8232_IOC_SRESOLUTION  _IOW(BI8232_IOC_MAGIC,10, int)  /* change resolution */
#define BI8232_IOC_SDEEPSLEEP   _IOW(BI8232_IOC_MAGIC,11, int)  /* enable or disable deep sleep function */
#define BI8232_IOC_SREPORTRATE  _IOW(BI8232_IOC_MAGIC,12, int)  /* change device report frequency */
#define BI8232_IOC_SSENSITIVITY _IOR(BI8232_IOC_MAGIC,13, int)  /* change sensitivity setting */
#define BI8232_IOC_MAXNR    15
/* FIH, Henry Juang, 2009/11/20 ++*/
/* [FXX_CR], Add for proximity driver to turn on/off BL and TP. */
int notify_from_proximity(bool bFlag);  //Added for test
/* FIH, Henry Juang, 2009/11/20 --*/
#endif

