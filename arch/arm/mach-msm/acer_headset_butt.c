/*
 * Acer Headset device button driver.
 *
 *
 * Copyright (C) 2008 acer Corporation.
 *
 * Authors:
 *    Shawn Tu <Shawn_Tu@acer.com.tw>
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

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <mach/acer_headset_butt.h>
#include <mach/acer_headset.h>

#define HS_BUTT_DRIVER_NAME           "acer-hs-butt"
#define DEBOUNCE_TIME                 200000000 /* 200 ms */

static int __init hs_butt_init(void);
static int hs_butt_probe(struct platform_device *pdev);
static int hs_butt_remove(struct platform_device *pdev);
static irqreturn_t hs_butt_interrupt(int irq, void *dev_id);
static int __init hs_butt_register_input(struct input_dev *input);
/* Add a work queue !! */
static struct work_struct hsmic_wq;
static bool hs_state;
static bool hs_type_state;

struct hs_butt_data {
	struct switch_dev sdev;
	struct input_dev *input;
	struct hrtimer btn_timer;
	ktime_t btn_debounce_time;
	unsigned int butt;
	unsigned int dett;
	unsigned int mic;
	unsigned int irq;
};

static struct hs_butt_data *hr;

static struct platform_driver hs_butt_driver = {
	.probe     = hs_butt_probe,
	.remove    = hs_butt_remove,
	.driver    = {
		.name    = HS_BUTT_DRIVER_NAME,
		.owner   = THIS_MODULE,
	},
};

static enum hrtimer_restart button_event_timer_func(struct hrtimer *data)
{
	bool state;

	/* Prevent to trigger the Music_AP after hanging up*/
	if(!(gpio_get_value(hr->dett)==0 && gpio_get_value(hr->mic)==1))
		return HRTIMER_NORESTART;

	state = gpio_get_value(hr->butt);
	if(state) {
		input_report_key(hr->input, KEY_MEDIA, 1);
		input_sync(hr->input);
	}
	else {
		input_report_key(hr->input, KEY_MEDIA, 0);
		input_sync(hr->input);
	}

	return HRTIMER_NORESTART;
}

/* For Headset type detect  +++*/
void set_hs_type_state(bool state)
{
	hs_type_state = state;
}
bool get_hs_type_state(void)
{
	return hs_type_state;
}

void set_hs_state(bool state)
{
	hs_state = state;
}

static void acer_update_mic_work(struct work_struct *work)
{
	pr_debug("#### acer_update_mic_work \n");
	hs_type_state = true;
}
/* For Headset type detect ---*/

static int __init hs_butt_init(void)
{
	int ret;

	ret = platform_driver_register(&hs_butt_driver);
	if (ret){
		pr_err("[HS-BUTT] hs_butt_init failed! \n");
	}

	return ret;
}

static int hs_butt_probe(struct platform_device *pdev)
{
	int err = 0;
	int ret;

	struct hs_butt_gpio *pdata = pdev->dev.platform_data;

	hr = kzalloc(sizeof(struct hs_butt_data), GFP_KERNEL);
	if (!hr)
		return -ENOMEM;

	hr->btn_debounce_time = ktime_set(0, DEBOUNCE_TIME);

	/* init work queue*/
	INIT_WORK(&hsmic_wq, acer_update_mic_work);
	hr->sdev.name = HS_BUTT_DRIVER_NAME;
	ret = switch_dev_register(&hr->sdev);
	if (ret < 0)
	{
		pr_err("switch_dev fail!\n");
		goto err_switch_dev_register;
	}
	else
		pr_debug("### hs_butt_switch_dev success register ###\n");

	hr->butt = pdata->gpio_hs_butt;
	hr->dett = pdata->gpio_hs_dett;
	hr->mic = pdata->gpio_hs_mic;

	ret = gpio_request(hr->butt, "hs_butt");
	if (ret < 0)
	{
		pr_err("err_request_butt_gpio fail!\n");
		goto err_request_butt_gpio;
	}

	hr->irq = gpio_to_irq(hr->butt);
	if (hr->irq < 0) {
		ret = hr->irq;
		pr_err("err_get_hs_butt_irq_num fail!\n");
		goto err_get_hs_butt_irq_num_failed;
	}

	hrtimer_init(&hr->btn_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hr->btn_timer.function = button_event_timer_func;

	ret = request_irq(hr->irq, hs_butt_interrupt,
			  IRQF_TRIGGER_RISING, "ht_butt", NULL);
	if (ret < 0)
	{
		pr_err("err_request_butt_irq fail!\n");
		goto err_request_butt_irq;
	}else{
		pr_debug("[HS-BUTT] IRQ_%d already request_butt_irq in use\n", hr->irq);
	}

	ret = set_irq_wake(hr->irq, 1);
	if (ret < 0)
	{
		pr_err("err_request_butt_irq fail!\n");
		goto err_request_butt_irq;
	}

	/* input register */
	hr->input = input_allocate_device();
	if (hr->input == NULL) {
		pr_err("[HS-BUTT] input_allocate_device error!\n");
		return -ENOMEM;
	}

	err = hs_butt_register_input(hr->input);
	if (err < 0) {
		pr_err("[HS-BUTT] register_input error\n");
		goto err_register_input_dev;
	}

	hs_state = false;
	hs_type_state = false;

	pr_info("[HS-BUTT] Probe done\n");

	return 0;

err_register_input_dev:
	input_free_device(hr->input);
err_request_butt_irq:
	free_irq(hr->irq, 0);
err_get_hs_butt_irq_num_failed:
err_request_butt_gpio:
	gpio_free(hr->butt);
err_switch_dev_register:
	pr_err("[HS-BUTT] Probe error\n");

	return ret;
}

static int hs_butt_remove(struct platform_device *pdev)
{
	input_unregister_device(hr->input);
	free_irq(hr->irq, 0);
	gpio_free(hr->butt);
	switch_dev_unregister(&hr->sdev);

	return 0;
}

static irqreturn_t hs_butt_interrupt(int irq, void *dev_id)
{
	schedule_work(&hsmic_wq);
	if (hs_state) {
		hrtimer_start(&hr->btn_timer, hr->btn_debounce_time, HRTIMER_MODE_REL);
	}

	return IRQ_HANDLED;
}

static int __init hs_butt_register_input(struct input_dev *input)
{
	input->name = HS_BUTT_DRIVER_NAME;
	input->evbit[0] = BIT_MASK(EV_SYN)|BIT_MASK(EV_KEY);
	input->keybit[BIT_WORD(KEY_MEDIA)] |= BIT_MASK(KEY_MEDIA);

	return input_register_device(input);
}


static void __exit hs_butt_exit(void)
{
	platform_driver_unregister(&hs_butt_driver);
}

module_init(hs_butt_init);
module_exit(hs_butt_exit);

MODULE_AUTHOR("Shawn Tu <Shawn_Tu@acer.com.tw>");
MODULE_DESCRIPTION("Acer Headset Button");
MODULE_LICENSE("GPL");
