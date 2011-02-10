/* linux/drivers/wifi/wifi-power.c
 *
 * wifi Power Switch Module
 * controls power to external wifi device
 * with interface to power management device
 *

 * All source code in this file is licensed under the following license
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
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

static int wifi_power_state;
static int (*power_control)(int enable);

static char firmware_path[64];
static char nvram_path[64];

/* load firmware and/or nvram values from the filesystem */
module_param_string(firmware_path, firmware_path, 64, 0);
module_param_string(nvram_path, nvram_path, 64, 0);


static int wifi_power_param_set(const char *val, struct kernel_param *kp)
{
	int ret;

	pr_debug(
		"%s: previous wifi_power_state=%d\n",
		__func__, wifi_power_state);

	/* lock change of state and reference */
	ret = param_set_bool(val, kp);
	if (power_control) {
		if (!ret)
			ret = (*power_control)(wifi_power_state);
		else
			pr_err("%s param set bool failed (%d)\n",
					__func__, ret);
	} else {
		pr_info(
			"%s: deferring power switch until probe\n",
			__func__);
	}
	pr_info(
		"%s: current wifi_power_state=%d\n",
		__func__, wifi_power_state);
	return ret;
}

module_param_call(power, wifi_power_param_set, param_get_bool,
		  &wifi_power_state, S_IWUSR | S_IRUGO);

static int __init_or_module wifi_power_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info( "%s\n", __func__);

	if (!pdev->dev.platform_data) {
		pr_err("%s: platform data not initialized\n",
				__func__);
		return -ENOSYS;
	}

	power_control = pdev->dev.platform_data;

	wifi_power_state=1;
	if (wifi_power_state) {
		pr_info(
			"%s: handling deferred power switch\n",
			__func__);
	}
	ret = (*power_control)(wifi_power_state);
	return ret;
}

static int wifi_power_remove(struct platform_device *pdev)
{
	int ret;

	pr_debug( "%s\n", __func__);
	if (!power_control) {
		pr_err("%s: power_control function not initialized\n",
				__func__);
		return -ENOSYS;
	}
	wifi_power_state = 0;
	ret = (*power_control)(wifi_power_state);
	power_control = NULL;

	return ret;
}

static struct platform_driver wifi_power_driver = {
	.probe = wifi_power_probe,
	.remove = wifi_power_remove,
	.driver = {
		.name = "wifi_power",
		.owner = THIS_MODULE,
	},
};

static int __init_or_module wifi_power_init(void)
{
	int ret;

	pr_debug( "%s\n", __func__);
	ret = platform_driver_register(&wifi_power_driver);
	return ret;
}

static void __exit wifi_power_exit(void)
{
	pr_debug( "%s\n", __func__);
	platform_driver_unregister(&wifi_power_driver);
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Craig A1");
MODULE_DESCRIPTION("wifi power control driver");
MODULE_VERSION("1.00");
MODULE_PARM_DESC(power, "A1 wifi power switch (bool): 0,1=off,on");

module_init(wifi_power_init);
module_exit(wifi_power_exit);
