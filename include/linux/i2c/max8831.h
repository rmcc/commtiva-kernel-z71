#ifndef _MAX8831_H_
#define _MAX8831_H_

//MAX8831 I2C Registers address
#define PORT_P0_REG		0x00 // On/Off Control Register
#define PORT_P3_REG		0x03 // LED1 Ramp Control Register
#define PORT_P4_REG		0x04 // LED2 Ramp Control Register
#define PORT_P11_REG	0x0B // LED1 Current Control Register
#define PORT_P12_REG	0x0C // LED2 Current Control Register


//PWM/Blink settings value for output port
#define PWM_STATIC_LOW		0x00
#define PWM_LEVEL(x)		(x)			//Duty cycle: x/32 (1~31)
#define PWM_STATIC_HIGH		0x70
#define BLK_STATIC_LOW		(PWM_STATIC_LOW+0x20)
#define BLK_LEVEL(x)		((x)+0x20)	//Duty cycle: x/16 (1~15)
#define BLK_STATIC_HIGH		PWM_STATIC_HIGH

/*FIH_FTM, PinyCHWu, 2009/06/15 {*/
/*Add IOCTL command*/
#ifdef CONFIG_FIH_FXX
//IO conrol definition.
#define MAX7302_MAGIC 'm'
#define MAX7302_S_LCD		_IOW(MAX7302_MAGIC, 2, int)
#endif
/*} FIH_FTM, PinyCHWu, 2009/06/15*/

struct max8831_i2c_data
{
  	char reg;
	char data;
};


int max8831_port_set_level(int port, int level);
int lcd_bl_set_intensity(int level);
#endif
