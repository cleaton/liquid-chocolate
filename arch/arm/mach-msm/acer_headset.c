/*
 * Acer Headset device detection driver.
 *
 *
 * Copyright (C) 2008 acer Corporation.
 *
 * Authors:
 *    Lawrence Hou <Lawrence_Hou@acer.com.tw>
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

/*
    For detecting acer headset .

    Headset insertion/removal causes UEvent's to be sent, and
    /sys/class/switch/acer-hs/state to be updated.
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
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <mach/acer_headset.h>
#include <mach/acer_headset_butt.h>

#if 0
#define ACER_HS_DBG(fmt, arg...) printk(KERN_INFO "[ACER_HS]: %s: " fmt "\n", __FUNCTION__, ## arg)
#else
#define ACER_HS_DBG(fmt, arg...) do {} while (0)
#endif

#define ACER_HS_DRIVER_NAME "acer-hs"
#define HS_DET		151
#define MIC_BIAS_EN	152

bool control;
static int curstate;

static int acer_hs_probe(struct platform_device *pdev);
static int acer_hs_remove(struct platform_device *pdev);
static int acer_hs_open(struct inode *inode, struct file *file);
static int acer_hs_close(struct inode *inode, struct file *file);
static int acer_hs_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

static void enable_amp_work(struct work_struct *work);
static void acer_headset_delay_set_work(struct work_struct *work);
static void acer_headset_delay_butt_work(struct work_struct *work);

static struct work_struct short_wq;
static DECLARE_DELAYED_WORK(en_amp_wq, enable_amp_work);
static DECLARE_DELAYED_WORK(set_hs_state_wq, acer_headset_delay_set_work);
static DECLARE_DELAYED_WORK(set_hs_butt_wq, acer_headset_delay_butt_work);

static struct platform_device acer_hs_device = {
	.name		= ACER_HS_DRIVER_NAME,
};

static const struct file_operations acer_hs_fops = {
	.owner		= THIS_MODULE,
	.open		= acer_hs_open,
	.release	= acer_hs_close,
	.ioctl		= acer_hs_ioctl,
};

static struct platform_driver acer_hs_driver = {
	.probe		= acer_hs_probe,
	.remove		= acer_hs_remove,
	.driver		= {
		.name		= ACER_HS_DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
};

static struct miscdevice acer_hs_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = ACER_HS_DRIVER_NAME,
	.fops = &acer_hs_fops,
};

enum {
	NO_DEVICE	= 0,
	ACER_HEADSET	= 1,
	ACER_HEADSET_NO_MIC   = 2,
};

static struct hs_res *hr;

static ssize_t acer_hs_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hr->sdev)) {
		case NO_DEVICE:
			return sprintf(buf, "No Device\n");
		case ACER_HEADSET:
			return sprintf(buf, "Headset\n");
		case ACER_HEADSET_NO_MIC:
			return sprintf(buf, "Headset no mic\n");
	}
	return -EINVAL;
}

static ssize_t acer_hs_print_state(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hr->sdev)) {
		case NO_DEVICE:
			return sprintf(buf, "%s\n", "0");
		case ACER_HEADSET:
			return sprintf(buf, "%s\n", "1");
		case ACER_HEADSET_NO_MIC:
			return sprintf(buf, "%s\n", "2");
	}
	return -EINVAL;
}

static void remove_headset(void)
{
	ACER_HS_DBG(" Remove Headset.\n");
	switch_set_state(&hr->sdev, NO_DEVICE);
}

static void acer_update_headset_switch_state(int state)
{
	switch_set_state(&hr->sdev, state);
	ACER_HS_DBG(" headset_switch_state = %d \n", switch_get_state(&hr->sdev));
}

static void acer_headset_delay_butt_work(struct work_struct *work)
{
	ACER_HS_DBG(" ==> Enable headset button !!!\n");
	set_hs_state(true);
}

static void acer_headset_delay_set_work(struct work_struct *work)
{
	bool state;

	state = gpio_get_value(hr->det);

	if (!state) {
		ACER_HS_DBG("delay work: get_hs_type_state() = %d \n",get_hs_type_state());
		if (get_hs_type_state()) {
			curstate = ACER_HEADSET;
			ACER_HS_DBG(" ACER_HEADSET!! \n");
		} else {
			curstate = ACER_HEADSET_NO_MIC;
			ACER_HS_DBG(" ACER_HEADSET_NO_MIC!! \n");
		}
		schedule_delayed_work(&set_hs_butt_wq, 500);
	} else {
		gpio_set_value(hr->mic_bias_en,0);
		curstate = NO_DEVICE;
		set_hs_type_state(false);
		set_hs_state(false);
		ACER_HS_DBG(" NO_DEVICE!! \n");
	}
	ACER_HS_DBG("mic_bias_en = %d \n",gpio_get_value(hr->mic_bias_en));
	acer_update_headset_switch_state(curstate);
}

static void acer_update_state_work(struct work_struct *work)
{
	set_hs_type_state(false);
	set_hs_state(false);
	ACER_HS_DBG(" Pull up the mic bias enable!! \n");
	gpio_set_value(hr->mic_bias_en,1);
}

static void enable_amp_work(struct work_struct *work)
{
	gpio_set_value(hr->hph_amp_en,1);
}

void hs_amp(bool enable)
{
	if(enable) {
		hr->headsetOn = true;
		schedule_delayed_work(&en_amp_wq, EXPIRES);
	} else {
		hr->headsetOn = false;
		gpio_set_value(hr->hph_amp_en, enable);
	}
}

static enum hrtimer_restart detect_event_timer_func(struct hrtimer *data)
{
	ACER_HS_DBG("");

	schedule_work(&short_wq);
	schedule_delayed_work(&set_hs_state_wq, 20);
	return HRTIMER_NORESTART;
}

static irqreturn_t hs_det_irq(int irq, void *dev_id)
{
	ACER_HS_DBG("Update Headset state by scheduling a work.\n");
	gpio_set_value(hr->mic_bias_en,0);
	hrtimer_start(&hr->timer, hr->debounce_time, HRTIMER_MODE_REL);

	return IRQ_HANDLED;
}

static int acer_hs_probe(struct platform_device *pdev)
{
	int ret;

	printk(KERN_INFO "[ACER-HS]: Registering ACER headset driver\n");
	hr = kzalloc(sizeof(struct hs_res), GFP_KERNEL);
	if (!hr)
		return -ENOMEM;

	hr->debounce_time = ktime_set(0, 500000000);  /* 500 ms */

	INIT_WORK(&short_wq, acer_update_state_work);
	//hr->sdev.name = "acer-hs";
	hr->sdev.name = "h2w";
	hr->sdev.print_name = acer_hs_print_name;
	hr->sdev.print_state = acer_hs_print_state;
	hr->headsetOn = false;

	ret = switch_dev_register(&hr->sdev);
	if (ret < 0)
	{
		pr_err("switch_dev fail!\n");
		goto err_switch_dev_register;
	}

	hr->det = HS_DET;
	ret = gpio_request(hr->det, "hs_detect");
	if (ret < 0)
	{
		pr_err("request detect gpio fail!\n");
		goto err_request_detect_gpio;
	}

	/* mic_bias_en - mic bias enable*/
	hr->mic_bias_en = MIC_BIAS_EN;
	ret = gpio_request(hr->mic_bias_en, "MIC BIAS EN");
	if (ret) {
		pr_err("GPIO request for MIC BIAS EN failed\n");
		goto err_request_mic_bias_gpio;
	}

	/* hph_en_amp - head phone amplifier enable*/
	hr->hph_amp_en = HPH_AMP_EN;
	ret = gpio_request(hr->hph_amp_en, "HPH AMP EN");
	if (ret) {
		pr_err("GPIO request for HPH AMP EN failed!\n");
		return ret;
	}

	hr->irq = gpio_to_irq(hr->det);
	if (hr->irq < 0) {
		ret = hr->irq;
		pr_err("get hs detect irq num fail!\n");
		goto err_get_hs_detect_irq_num_failed;
	}

	hrtimer_init(&hr->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hr->timer.function = detect_event_timer_func;

	ret = request_irq(hr->irq, hs_det_irq,
			  IRQF_TRIGGER_FALLING, "hs_detect", NULL);
	if (ret < 0)
	{
		pr_err("request detect irq fail!\n");
		goto err_request_detect_irq;
	}

	ret = misc_register(&acer_hs_dev);
	if(ret) {
		pr_err("acer_hs_probe: acer_hs_dev register failed!\n");
		goto err_acer_hs_dev;
	}

#if 0
	/* open this section for debug usage. */
	ret = set_irq_wake(hr->irq, 1);
	if (ret < 0)
	{
		pr_info("err_request_detect_irq fail!\n");
		goto err_request_detect_irq;
	}
#endif
	curstate = switch_get_state(&hr->sdev);

	ACER_HS_DBG("probe done.\n");

	return 0;

err_request_detect_irq:
	free_irq(hr->irq, 0);
err_get_hs_detect_irq_num_failed:
	gpio_free(hr->det);
err_request_detect_gpio:
	gpio_free(hr->det);
err_request_mic_bias_gpio:
	gpio_free(hr->mic_bias_en);
err_switch_dev_register:
	pr_err("ACER-HS: Failed to register driver\n");
err_acer_hs_dev:
	pr_err("ACER-HS: Failed to register MISC acer-hs driver");

	return ret;
}

static int acer_hs_remove(struct platform_device *pdev)
{
	ACER_HS_DBG("");
	if (switch_get_state(&hr->sdev))
		remove_headset();
	gpio_free(hr->det);
	free_irq(hr->irq, 0);
	switch_dev_unregister(&hr->sdev);

	return 0;
}

static int acer_hs_open(struct inode *inode, struct file *file)
{
	ACER_HS_DBG("acer hs opened");
	return 0;
}

static int acer_hs_close(struct inode *inode, struct file *file)
{
	ACER_HS_DBG("acer hs closed");
	return 0;
}

static int acer_hs_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	u32 uparam;

	ACER_HS_DBG("IOCTL\n");
	if(_IOC_TYPE(cmd) != ACER_HS_IOCTL_MAGIC){
		pr_err("[ACER_HS] IOCTL: cmd magic type error\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > ACER_HS_IOC_MAXNR){
		pr_err("[ACER_HS] IOCTL: cmd number error\n");
		return -ENOTTY;
	}
	if(_IOC_DIR(cmd) & _IOC_NONE)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	if(err){
		pr_err("[ACER_HS] IOCTL: cmd access right error\n");
		return -EFAULT;
	}

	switch(cmd){
		case ACER_HS_ENABLE_AMP:
			pr_info("[ACER_HS]: Enable hs amp by ioctl\n");
			if(!hr->headsetOn)
				hs_amp(true);
			return 0;
		case ACER_HS_CHANGE_CONTROL:
			if (copy_from_user(&uparam, (void *)arg,
					sizeof(uparam))) {
				pr_err("Headset control copy failed.\n");
				return -1;
			}
			if(uparam) {
				control = true;
				ACER_HS_DBG("headset control change to driver.\n");
			} else {
				control = false;
				ACER_HS_DBG("headset control change to framework.\n");
			}
			return 0;
		default:
			pr_err("[ACER_HS] IOCTL: Command not found!\n");
			return -1;
	}
}

static int __init acer_hs_init(void)
{
	int ret;
	ACER_HS_DBG("");
	ret = platform_driver_register(&acer_hs_driver);
	if (ret)
		return ret;
	return platform_device_register(&acer_hs_device);
}

static void __exit acer_hs_exit(void)
{
	platform_device_unregister(&acer_hs_device);
	platform_driver_unregister(&acer_hs_driver);
}

module_init(acer_hs_init);
module_exit(acer_hs_exit);

MODULE_AUTHOR("Lawrence Hou <Lawrence_Hou@acer.com.tw>");
MODULE_DESCRIPTION("Acer Headset detection driver");
MODULE_LICENSE("GPL");
