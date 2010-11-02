#ifndef _7X27_KYBD_H_
#define _7x27_KYBD_H_

struct Q7x27_kybd_platform_data {
	void (*keypad_gpio) (void);
	int volup_pin;
	int voldn_pin;
	int key_1_pin;
	int key_2_pin;
	int cam_sw_t_pin;
	int cam_sw_f_pin;
	int hook_sw_pin;
	int center_pin;
};

/* FIH, SimonSSChang, 2009/07/28 { */
/* [FXX_CR], F0X.FC-116 Add option for wake up source*/
#ifdef CONFIG_FIH_FXX
bool key_wakeup_get(void);
int key_wakeup_set(int on);
#endif
/* } FIH, SimonSSChang, 2009/07/28 */

/* FIH, SimonSSChang, 2009/09/10 { */
/* [FXX_CR], To enable Send & End key wakeup when incoming call*/
#ifdef CONFIG_FIH_FXX
bool incoming_call_get(void);
int incoming_call_set(int on);
#endif
/* } FIH, SimonSSChang, 2009/09/10 */

#endif
