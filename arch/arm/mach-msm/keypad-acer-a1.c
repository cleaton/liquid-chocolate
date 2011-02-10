/*
 * Copyright (c) 2009 ACER, INC.
 * Author: Robert CH Chou <Robert_CH_Chou@acer.com.tw>
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

#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/gpio_event.h>

#define A1_CAM_AUTOFCS	34	/* Not using */
#define A1_POWER_KEY	35
#define A1_CAM_CAPTURE	38
#define A1_VOLUME_DOWN	41
#define A1_VOLUME_UP	42

static struct gpio_event_direct_entry a1_keypad_map[] = {
	{ A1_POWER_KEY,              KEY_POWER      },
	{ A1_VOLUME_UP,              KEY_VOLUMEUP   },
	{ A1_VOLUME_DOWN,            KEY_VOLUMEDOWN },
	{ A1_CAM_CAPTURE,            KEY_CAMERA     },
	{ A1_CAM_AUTOFCS,            211            },
};

#if defined(CONFIG_MSM_RPCSERVER_HANDSET)
static int keypad_gpio_event_input_func(struct input_dev *input_dev,
		struct gpio_event_info *info,
		void **data, int func);
#endif

static struct gpio_event_input_info a1_direct_keypad_info = {
#if defined(CONFIG_MSM_RPCSERVER_HANDSET)
	.info.func	= keypad_gpio_event_input_func,
#else
	.info.func	= gpio_event_input_func,
#endif
	.flags		= 0,
	.type		= EV_KEY,
	.keymap		= a1_keypad_map,
        .debounce_time.tv.nsec = 20 * NSEC_PER_MSEC,
        .keymap_size = ARRAY_SIZE(a1_keypad_map)
};

static struct gpio_event_info *a1_keypad_info[] = {
	&a1_direct_keypad_info.info,
};

static struct gpio_event_platform_data a1_keypad_data = {
	.name		= "a1-keypad",
	.info		= a1_keypad_info,
	.info_count	= ARRAY_SIZE(a1_keypad_info)
};

struct platform_device a1_keypad_device = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &a1_keypad_data,
	},
};

#if defined(CONFIG_MSM_RPCSERVER_HANDSET)
/* This is used by rpc_server_handset.c for sending keycodes */
static struct input_dev *keypad_dev;

static int keypad_gpio_event_input_func(struct input_dev *input_dev,
		struct gpio_event_info *info,
		void **data, int func)
{
	int err;

	err = gpio_event_input_func(input_dev, info, data, func);

	if (func == GPIO_EVENT_FUNC_INIT && !err) {
		keypad_dev = input_dev;
	} else if (func == GPIO_EVENT_FUNC_UNINIT) {
		keypad_dev = NULL;
	}

	return err;
}

struct input_dev *msm_keypad_get_input_dev(void)
{
	return keypad_dev;
}
#endif /* defined(CONFIG_MSM_RPCSERVER_HANDSET) */

static int __init a1_init_keypad(void)
{
	return platform_device_register(&a1_keypad_device);
}

device_initcall(a1_init_keypad);
