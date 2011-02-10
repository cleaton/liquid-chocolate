/*
 * Acer Headset device button driver.
 *
 *
 * Copyright (C) 2008 acer Corporation.
 *
 * Authors:
 *    Shawn Tu <Shawn_Tu@acer.com.tw>
 */

struct hs_butt_gpio {
	int gpio_hs_butt;
	int gpio_hs_dett;
	int gpio_hs_mic;
};
extern void set_hs_state(bool state);
extern void set_hs_type_state(bool state);
extern bool get_hs_type_state(void);