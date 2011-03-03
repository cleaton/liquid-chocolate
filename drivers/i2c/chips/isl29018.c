#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include "isl29018.h"

#define  ISL29018_GPIO   142

#define  BIT_16_RESOLUTION  65536
#define  BIT_12_RESOLUTION  4096
#define  BIT_08_RESOLUTION  256
#define  BIT_04_RESOLUTION  16

#define  RANGE1  1000
#define  RANGE2  4000
#define  RANGE3  16000
#define  RANGE4  64000

#define  CALCULATING_LUX(range, resolution, adc) \
			((range) * (adc) / (resolution))

static uint16_t lsensor_adc_table[10] = {
	0, 0x1c71, 0x38e2, 0x5553, 0x71c4, 0x8e35, 0xaaa6, 0xc717, 0xe388, 0xffff
};

static struct i2c_client *private_isl29018_client;

static uint16_t g_distance;

struct isl29018_infomation {
	uint32_t resolution;
	uint32_t range;
	int operation_mode;
};

/* Data for I2C driver */
struct isl_data {
	struct i2c_client *client;
	struct work_struct work;
	wait_queue_head_t wait;
	struct input_dev *ps_input_dev;
	struct input_dev *ls_input_dev;
	struct isl29018_infomation info;
	int ps_enabled;
	int ls_enabled;
	int object_distance;   // 0 is close, and 1 is far

	struct work_struct sensor_work;
	wait_queue_head_t sensor_wait;
	bool timer_enabled;
};
static struct isl_data *isl_data;

#define READ_ADC_POLLING_TIME                    100 /* milliseconds */
struct timer_list sensor_timer;

/*
 * client: target client
 * buf: target register
 * count: length of response
 */
static int i2c_read(struct i2c_client *client, char *buf, int count)
{
	if (1 != i2c_master_send(client, buf, 1)) {
		pr_err("[ISL] i2c_read --> Send reg. info error\n");
		return -1;
	}

	if (count != i2c_master_recv(client, buf, count)) {
		pr_err("[ISL] i2c_read --> get response error\n");
		return -1;
	}
	return 0;
}

/*
 * client: target client
 * buf: target register with command
 * count: length of transmitting
 */
static int i2c_write(struct i2c_client *client, char *buf, int count)
{
	if (count != i2c_master_send(client, buf, count)) {
		pr_err("[ISL] i2c_write --> Send reg. info error\n");
		return -1;
	}
	return 0;
}

static int isl_adc(uint16_t *value)
{
	struct i2c_client *client;
	int rs = 0;
	unsigned char reg_buf[2] = {0};

	client = private_isl29018_client;

	reg_buf[0] = 0x02;  // Addr: 0x02, 0x03
	rs = i2c_read(client, reg_buf, 2);  // Read adc data
	if (rs == -1) {
		pr_err("[ISL]i2c_read fail in isl_adc()\n");
	}
	else {
		*value = (reg_buf[1] * 256) + reg_buf[0];
	}
	return rs;
}

static int isl_ls_adc(uint16_t *adc_value, uint8_t *adc_level)
{
	int ret;
	int i;

	ret = isl_adc(adc_value);

	if (*adc_value > 0xFFFF) {
		pr_warning("%s: get wrong value: 0x%X\n",
				__func__, *adc_value);
			return -1;
	}

	*adc_level = ARRAY_SIZE(lsensor_adc_table) - 1;
	for (i = 0; i < ARRAY_SIZE(lsensor_adc_table); i++) {
		if (*adc_value <= lsensor_adc_table[i]) {
			*adc_level = i;
			break;
		}
	}

	return ret;
}

static void sensor_timer_func(unsigned long unused)
{
	schedule_work(&isl_data->sensor_work);
	mod_timer(&sensor_timer, jiffies + msecs_to_jiffies(READ_ADC_POLLING_TIME));
}

static int isl_power_down(struct i2c_client *client)
{
	int rs = 0;
	unsigned char reg_buf[2] = {0};

	reg_buf[0] = 0x00;  //Addr: 0x00
	reg_buf[1] = 0x00;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[ISL] i2c_write reg 0x00 fail in %s()\n", __func__);
		goto err_i2c_write;
	}

err_i2c_write:
	return rs;
}

static int isl_enable(void)
{
	struct i2c_client *client;
	struct isl_data *cdata;
	int rc=0;
	pr_debug("%s\n", __func__);

	client = private_isl29018_client;
	cdata = i2c_get_clientdata(client);

	if (!cdata->timer_enabled){
		setup_timer(&sensor_timer, sensor_timer_func, 0);
		mod_timer(&sensor_timer, jiffies + msecs_to_jiffies(5));
		cdata->timer_enabled=1;
	}
	return rc;
}

static int isl_disable(void)
{
	struct i2c_client *client;
	struct isl_data *cdata;
	int rc = 0;
	pr_debug("%s\n", __func__);

	client = private_isl29018_client;
	cdata = i2c_get_clientdata(client);
	if (cdata->ps_enabled == 0 && cdata->ls_enabled == 0){
		cdata->timer_enabled=0;
		if (del_timer(&sensor_timer)){
			pr_debug("del sensor timer OK\n");
		}
		isl_power_down(client);
	}
	return rc;
}

static int isl_ls_open(struct inode *inode, struct file *file)
{
	struct i2c_client *client = private_isl29018_client;
	if (client == NULL){
		pr_err("[ISL] I2C driver not install (isl_open)\n");
		return -1;
	}
	pr_debug("[ISL] has been opened\n");
	return 0;
}

static int isl_ls_close(struct inode *inode, struct file *file)
{
	pr_debug("[ISL] has been closed\n");
	return 0;
}

static int isl_ls_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client;
	struct isl_data *cdata;
	int err = 0;
	int isl_cmd = 0;
	int val = 0;

	client = private_isl29018_client;
	cdata = i2c_get_clientdata(client);

	/* check cmd */
	if (_IOC_TYPE(cmd) != ISL29018_IOC_MAGIC) {
		pr_err("[ISL] cmd magic type error\n");
		return -ENOTTY;
	}
	isl_cmd = _IOC_NR(cmd);
	if (isl_cmd >= ISL29018_IOC_MAXNR) {
		pr_err("[ISL] cmd number error\n");
		return -ENOTTY;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	}
	else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	}
	if (err) {
		pr_err("[ISL] cmd access_ok error\n");
		return -EFAULT;
	}
	if (client == NULL) {
		pr_err("[ISL] I2C driver not install (isl_ioctl)\n");
		return -EFAULT;
	}

	/* cmd mapping */
	switch (cmd) {
		case LIGHT_SENSOR_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg))
				return -EFAULT;
			if (val) {
				cdata->ls_enabled = 1;
				cdata->info.resolution = BIT_12_RESOLUTION;
				cdata->info.range = RANGE2;
				pr_debug("LIGHT_SENSOR_IOCTL_ENABLE\n");
				return isl_enable();
			}
			else {
				cdata->ls_enabled = 0;
				pr_debug("LIGHT_SENSOR_IOCTL_DISABLE\n");
				return isl_disable();
			}
			break;
		case LIGHT_SENSOR_IOCTL_GET_ENABLE:
			return put_user(cdata->ls_enabled, (unsigned long __user *)arg);
			break;
		default:
			pr_err("[ISL] ioctl cmd not found\n");
			return -EFAULT;
	}
	return 0;
}

static int isl_ps_open(struct inode *inode, struct file *file)
{
	struct i2c_client *client = private_isl29018_client;
	if (client == NULL){
		pr_err("[ISL] I2C driver not install (isl_open)\n");
		return -1;
	}
	pr_debug("[ISL] has been opened\n");
	return 0;
}

static int isl_ps_close(struct inode *inode, struct file *file)
{
	pr_debug("[ISL] has been closed\n");
	return 0;
}

static int isl_ps_ioctl(struct inode *inode, struct file *file, 
				unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client;
	struct isl_data *cdata;
	int err = 0;
	int isl_cmd = 0;
	int val = 0;

	client = private_isl29018_client;
	cdata = i2c_get_clientdata(client);

	/* check cmd */
	if (_IOC_TYPE(cmd) != ISL29018_IOC_MAGIC) {
		pr_err("[ISL] cmd magic type error\n");
		return -ENOTTY;
	}
	isl_cmd = _IOC_NR(cmd);
	if (isl_cmd >= ISL29018_IOC_MAXNR) {
		pr_err("[ISL] cmd number error\n");
		return -ENOTTY;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	}
	else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	}
	if (err) {
		pr_err("[ISL] cmd access_ok error\n");
		return -EFAULT;
	}
	if (client == NULL) {
		pr_err("[ISL] I2C driver not install (isl_ioctl)\n");
		return -EFAULT;
	}

	/* cmd mapping */
	switch (cmd) {
		case PROXIMITY_SENSOR_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg))
				return -EFAULT;
			if (val) {
				cdata->ps_enabled=1;
				pr_debug("PROXIMITY_SENSOR_IOCTL_ENABLE\n");
				return isl_enable();
			}
			else {
				cdata->ps_enabled=0;
				pr_debug("PROXIMITY_SENSOR_IOCTL_DISABLE\n");
				return isl_disable();
			}
			break;
		case PROXIMITY_SENSOR_IOCTL_GET_ENABLE:
			return put_user(cdata->ps_enabled, (unsigned long __user *)arg);
			break;
		default:
			pr_err("[ISL] ioctl cmd not found\n");
			return -EFAULT;
	}
	return 0;
}

/* File operation of ISL device file for proximity sensor */
static const struct file_operations isl_ps_fops = {
	.owner		= THIS_MODULE,
	.open		= isl_ps_open,
	.release	= isl_ps_close,
	.ioctl		= isl_ps_ioctl,
};

static struct miscdevice isl_ps_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "proximity",
	.fops = &isl_ps_fops,
};

/* File operation of ISL device file for light sensor */
static const struct file_operations isl_ls_fops = {
	.owner		= THIS_MODULE,
	.open		= isl_ls_open,
	.release	= isl_ls_close,
	.ioctl		= isl_ls_ioctl,
};

static struct miscdevice isl_ls_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &isl_ls_fops,
};

static ssize_t isl_ls_adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	uint8_t adc_level = 0;
	uint16_t adc_value = 0;
	int ret;

	ret = isl_ls_adc(&adc_value, &adc_level);
	ret = sprintf(buf, "ADC[0x%03X] => level %d\n", adc_value, adc_level);

	return ret;
}
static DEVICE_ATTR(ls_adc, 0644, isl_ls_adc_show, NULL);

int read_ps_adc(uint16_t *adc_value)
{
	struct i2c_client *client;
	struct isl_data *cdata;
	unsigned char reg_buf[2] = {0};
	int rs=0;
	static uint16_t old_adc = 0;

	client = private_isl29018_client;
	cdata = i2c_get_clientdata(client);

	reg_buf[0] = 0x01;  //Addr: 0x01
	reg_buf[1] = 0xB5; //bit 12 , range2:4000
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[ISL]i2c_write reg 0x08 fail in isl_complete_reset()\n");
		return 0;
	}
	reg_buf[0] = 0x00;  //Addr: 0x00
	reg_buf[1] = 0x60;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[ISL]i2c_write reg 0x08 fail in isl_complete_reset()\n");
	}
	msleep(10);
	reg_buf[0] = 0x02;  // Addr: 0x02, 0x03
	rs = i2c_read(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[ISL]i2c_read fail in isl_adc()\n");
		return 0;
	} else {
		if ((reg_buf[1] >> 7) == 0) {
			*adc_value = (reg_buf[1] * 256) + reg_buf[0];
			if (old_adc == *adc_value) {
				(*adc_value)++;
			}
			old_adc = *adc_value;
		}
		return 1 ;
	}
}

static ssize_t isl_ps_adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	uint16_t adc_value = 0;
	int ret;

	read_ps_adc(&adc_value);
	ret = sprintf(buf, "Raw ADC[%d] == ADC[%d]\n", adc_value , adc_value*16);
	return ret;
}
static DEVICE_ATTR(ps_adc, 0644, isl_ps_adc_show, NULL);

static int isl_complete_reset(void)
{
	struct i2c_client *client;
	int rs = 0;
	unsigned char reg_buf[2] = {0};

	client = private_isl29018_client;

	reg_buf[0] = 0x08;  //Addr: 0x08
	reg_buf[1] = 0x00;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[ISL]i2c_write reg 0x08 fail in isl_complete_reset()\n");
		goto err_i2c_write;
	}

	reg_buf[0] = 0x00;  //Addr: 0x00
	reg_buf[1] = 0x00;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[ISL]i2c_write reg 0x00 fail in isl_complete_reset()\n");
		goto err_i2c_write;
	}

err_i2c_write:
	return rs;
}

void report_ps_adc(void)
{
	struct i2c_client *client;
	struct isl_data *cdata;
	int ret=0;
	uint16_t adc_value = 0;
	uint16_t report_value = 0;

	client = private_isl29018_client;
	cdata = i2c_get_clientdata(client);
	ret = read_ps_adc(&adc_value);
	if (ret) {
		report_value = 10 * (2048-adc_value) / (2048-g_distance);
		input_report_abs(cdata->ps_input_dev, ABS_DISTANCE, report_value);
		input_sync(cdata->ps_input_dev);
	}
}

void report_ls_adc(void)
{
	struct i2c_client *client;
	struct isl_data *cdata;
	unsigned char reg_buf[2] = {0};
	int rs=0;
	uint16_t adc_value = 0;
	static uint16_t old_adc = 0;

	client = private_isl29018_client;
	cdata = i2c_get_clientdata(client);

	reg_buf[0] = 0x01;  //Addr: 0x01
	reg_buf[1] = 0xB7; //because time, we use 12-bit rs
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[ISL]i2c_write reg 0x08 fail in isl_complete_reset()\n");
	}

	reg_buf[0] = 0x00;  //Addr: 0x00
	reg_buf[1] = 0x20;
	rs = i2c_write(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[ISL]i2c_write reg 0x08 fail in isl_complete_reset()\n");
	}
	msleep(10);
	
	reg_buf[0] = 0x02;  // Addr: 0x02, 0x03
	rs = i2c_read(client, reg_buf, 2);
	if (rs == -1) {
		pr_err("[ISL]i2c_read fail in isl_adc()\n");
	} else {
		adc_value = (reg_buf[1] * 256) + reg_buf[0];
	}
	if(old_adc == adc_value){
		return;
	}
	old_adc = adc_value;
	input_report_abs(cdata->ls_input_dev, ABS_MISC, adc_value);
	input_sync(cdata->ls_input_dev);
}

static void sensor_work_func(struct work_struct *work)
{
	struct i2c_client *client;
	struct isl_data *cdata;
	static int count = 0;

	client = private_isl29018_client;
	cdata = i2c_get_clientdata(client);

	if (cdata->ps_enabled) {
		report_ps_adc();
	}
	if (cdata->ls_enabled) {
		if (count % 2 == 0) {
			count = 0;
			msleep(10);// this is needed for more correct light sensor adc
			report_ls_adc();
		}
		count++;
	}
}

static int isl_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;
	pr_debug("%s ++ entering\n", __FUNCTION__);

	isl_data = kzalloc(sizeof(struct isl_data),GFP_KERNEL);
	if (NULL==isl_data) {
		res = -ENOMEM;
		goto out;
	}
	isl_data->client = client;
	isl_data->info.operation_mode = POWER_DOWN;
	private_isl29018_client = client;

	/* check i2c functionality is workable */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[ISL] i2c_check_functionality error!\n");
		res = -ENOTSUPP;
		goto out_free_mem;
	}
	strlcpy(client->name, ISL29018_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, isl_data);

	/* reset isl29011 chip */
	res = isl_complete_reset();
	if (res == -1) {
		pr_err("[ISL] reset isl29011 chip error!\n");
		goto out_free_mem;
	}
	/* Proximity sensor related */
	res = device_create_file(&client->dev, &dev_attr_ps_adc);
	/* allocate input device */
	isl_data -> ps_input_dev = input_allocate_device();
	if (!isl_data -> ps_input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		res = -ENOMEM;
		goto out;
	}
	isl_data -> ps_input_dev -> name = "proximity";
	set_bit(EV_ABS, isl_data -> ps_input_dev -> evbit);
	input_set_abs_params(isl_data -> ps_input_dev, ABS_DISTANCE , 0, 65536 , 0, 0);

	/* register input device */
	res = input_register_device(isl_data -> ps_input_dev);
	if (res < 0) {
		pr_err("%s: could not register input device\n", __func__);
		goto out;
	}

	/* register misc device */
	res = misc_register(&isl_ps_dev);
	if (res < 0) {
		pr_err("[ISL]isl_dev register failed! error code:[%d]\n", res);
		goto out_unreg_irq;
	}
	/* Proximity sensor related - end */

	/* Light sensor related */
	res = device_create_file(&client->dev, &dev_attr_ls_adc);
	/* allocate input device */
	isl_data -> ls_input_dev = input_allocate_device();
	if (!isl_data -> ls_input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		res = -ENOMEM;
		goto out;
	}
	isl_data -> ls_input_dev -> name = "lightsensor-level";
	set_bit(EV_ABS, isl_data -> ls_input_dev -> evbit);
	input_set_abs_params(isl_data -> ls_input_dev, ABS_MISC, 0, 65535, 0, 0);

	/* register input device */
	res = input_register_device(isl_data -> ls_input_dev);
	if (res < 0) {
		pr_err("%s: could not register input device\n", __func__);
		goto out;
	}

	/* register misc device */
	res = misc_register(&isl_ls_dev);
	if (res < 0) {
		pr_err("[ISL]isl_dev register failed! error code:[%d]\n", res);
		goto out_unreg_irq;
	}
	/* Light sensor related - end */

	INIT_WORK(&isl_data->sensor_work, sensor_work_func);
	init_waitqueue_head(&isl_data->sensor_wait);

	read_ps_adc(&g_distance);

	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;

out_unreg_irq:
	free_irq(client->irq, isl_data);
out_free_mem:
	kfree(isl_data);
out:
	pr_err("[ISL] probe error\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return res;
}

static int isl_remove(struct i2c_client *client)
{
	struct isl_data *tp = i2c_get_clientdata(client);

	misc_deregister(&isl_ps_dev);
	misc_deregister(&isl_ls_dev);
	gpio_free(ISL29018_GPIO);
	free_irq(client->irq, tp);
	kfree(tp);

	return 0;
}

static const struct i2c_device_id isl_id[] = {
	{ISL29018_DRIVER_NAME, 0},
	{ }
};

static int isl_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pr_debug("%s ++ entering\n", __FUNCTION__);
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;
}

static int isl_resume(struct i2c_client *client)
{
	pr_debug("%s ++ entering\n", __FUNCTION__);
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;
}

/* new style I2C driver struct */
static struct i2c_driver isl_driver = {
	.probe = isl_probe,
	.remove = __devexit_p(isl_remove),
	.id_table = isl_id,
	.suspend = isl_suspend,
	.resume = isl_resume,
	.driver = {
		.name = ISL29018_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init isl_init(void)
{
	int res;
	pr_debug("%s ++ entering\n", __FUNCTION__);
	res = i2c_add_driver(&isl_driver);
	if (res) {
		pr_err("[ISL]%s: Driver Initialisation failed\n", __FILE__);
	}
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return res;
}

static void __exit isl_exit(void)
{
	pr_debug("%s ++ entering\n", __FUNCTION__);
	i2c_del_driver(&isl_driver);
	pr_debug("%s -- leaving\n", __FUNCTION__);
}

module_init(isl_init);
module_exit(isl_exit);

MODULE_AUTHOR("Wei Liu <Wei_Liu@acer.com.tw>");
MODULE_DESCRIPTION("Intersil isl29011/18 P/L sensor driver");
