#if defined(CONFIG_ACER_DEBUG)
#define DEBUG
#endif
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include "isl29018.h"

#define ISL29018_GPIO   153

/*
*
*	register definitions
*
*/
#define ISL29018_COMMAND_I_REG      0x00
#define ISL29018_COMMAND_II_REG     0x01
#define ISL29018_DATA_LSB_REG       0x02
#define ISL29018_DATA_MSB_REG       0x03
#define ISL29018_INT_LT_LSB_REG     0x04
#define ISL29018_INT_LT_MSB_REG     0x05
#define ISL29018_INT_HT_LSB_REG     0x06
#define ISL29018_INT_HT_MSB_REG     0x07
#define ISL29018_INTERNAL_STATE_REG	0x08	/* un-specification */

#define ISL29018_GET_BITSLICE(regvar, bitname)\
	(regvar & bitname##__MSK) >> bitname##__POS


#define ISL29018_SET_BITSLICE(regvar, bitname, val)\
	(regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)

/*
*
*	bit slice positions in registers
*
*/

/* DATA REGISTERS */

#define ISL29018_DATA_LSB__POS      0
#define ISL29018_DATA_LSB__LEN      8
#define ISL29018_DATA_LSB__MSK      0xFF
#define ISL29018_DATA_LSB__REG      ISL29018_DATA_LSB_REG

#define ISL29018_DATA_MSB__POS      0
#define ISL29018_DATA_MSB__LEN      8
#define ISL29018_DATA_MSB__MSK      0xFF
#define ISL29018_DATA_MSB__REG      ISL29018_DATA_MSB_REG

/* CONTROL BITS */

#define ISL29018_MODE__POS      5
#define ISL29018_MODE__LEN      3
#define ISL29018_MODE__MSK      0xE0
#define ISL29018_MODE__REG      ISL29018_COMMAND_I_REG

#define ISL29018_INT_PERSIST_CYCLE__POS     0
#define ISL29018_INT_PERSIST_CYCLE__LEN     2
#define ISL29018_INT_PERSIST_CYCLE__MSK     0x03
#define ISL29018_INT_PERSIST_CYCLE__REG     ISL29018_COMMAND_I_REG

#define ISL29018_PROX_SCHEME__POS       7
#define ISL29018_PROX_SCHEME__LEN       1
#define ISL29018_PROX_SCHEME__MSK       0x80
#define ISL29018_PROX_SCHEME__REG       ISL29018_COMMAND_II_REG

#define ISL29018_MODULATION_FREQ__POS       6
#define ISL29018_MODULATION_FREQ__LEN       1
#define ISL29018_MODULATION_FREQ__MSK       0x40
#define ISL29018_MODULATION_FREQ__REG       ISL29018_COMMAND_II_REG

#define ISL29018_IRDR_CURRENT__POS      4
#define ISL29018_IRDR_CURRENT__LEN      2
#define ISL29018_IRDR_CURRENT__MSK      0x30
#define ISL29018_IRDR_CURRENT__REG      ISL29018_COMMAND_II_REG

#define ISL29018_RESOLUTION__POS        2
#define ISL29018_RESOLUTION__LEN        2
#define ISL29018_RESOLUTION__MSK        0x0C
#define ISL29018_RESOLUTION__REG        ISL29018_COMMAND_II_REG

#define ISL29018_RANGE__POS     0
#define ISL29018_RANGE__LEN     2
#define ISL29018_RANGE__MSK     0x03
#define ISL29018_RANGE__REG     ISL29018_COMMAND_II_REG


/* STATUS BITS */

#define ISL29018_INT_FLAG__POS      2
#define ISL29018_INT_FLAG__LEN      1
#define ISL29018_INT_FLAG__MSK      0x04
#define ISL29018_INT_FLAG__REG      ISL29018_COMMAND_I_REG

/* interrupt low/high threshold */

#define ISL29018_INT_LOW_THRES_LSB__POS     0
#define ISL29018_INT_LOW_THRES_LSB__LEN     8
#define ISL29018_INT_LOW_THRES_LSB__MSK     0xFF
#define ISL29018_INT_LOW_THRES_LSB__REG     ISL29018_INT_LT_LSB_REG

#define ISL29018_INT_LOW_THRES_MSB__POS     0
#define ISL29018_INT_LOW_THRES_MSB__LEN     8
#define ISL29018_INT_LOW_THRES_MSB__MSK     0xFF
#define ISL29018_INT_LOW_THRES_MSB__REG     ISL29018_INT_LT_MSB_REG

#define ISL29018_INT_HIGH_THRES_LSB__POS    0
#define ISL29018_INT_HIGH_THRES_LSB__LEN    8
#define ISL29018_INT_HIGH_THRES_LSB__MSK    0xFF
#define ISL29018_INT_HIGH_THRES_LSB__REG    ISL29018_INT_HT_LSB_REG

#define ISL29018_INT_HIGH_THRES_MSB__POS    0
#define ISL29018_INT_HIGH_THRES_MSB__LEN    8
#define ISL29018_INT_HIGH_THRES_MSB__MSK    0xFF
#define ISL29018_INT_HIGH_THRES_MSB__REG    ISL29018_INT_HT_MSB_REG

/* INTERAL STATE BITS (un-specification) */

#define ISL29018_INTERNAL_STATE__POS    0
#define ISL29018_INTERNAL_STATE__LEN    8
#define ISL29018_INTERNAL_STATE__MSK    0xFF
#define ISL29018_INTERNAL_STATE__REG    ISL29018_INTERNAL_STATE_REG

#define _I2C_READ_BIT(bitname, czdata)\
{\
	unsigned char reg_buf[2] = {0};\
	reg_buf[0] = bitname##__REG;\
	i2c_read(client, reg_buf, 1);\
	czdata[0] = ISL29018_GET_BITSLICE(reg_buf[0], bitname);\
}

#define _I2C_WRITE_BIT(bitname, czdata)\
{\
	unsigned char reg_buf[2] = {0};\
	reg_buf[0] = bitname##__REG;\
	i2c_read(client, reg_buf, 1);\
	reg_buf[1] = ISL29018_SET_BITSLICE(reg_buf[0], bitname, czdata);\
	reg_buf[0] = bitname##__REG;\
	i2c_write(client, reg_buf, 2);\
}

#define _I2C_READ_SHORT(bitname, czdata)\
{\
	unsigned char reg_buf[2] = {0};\
	reg_buf[0] = bitname##_LSB__REG;\
	i2c_read(client, reg_buf, 2);\
	memcpy(czdata, reg_buf, 2);\
}

#define _I2C_WRITE_SHORT(bitname, czdata)\
{\
	unsigned char reg_buf[3] = {0};\
	reg_buf[0] = bitname##_LSB__REG;\
	memcpy(&(reg_buf[1]), czdata, 2);\
	i2c_write(client, reg_buf, 3);\
}

static int __init isl_init(void);
static int isl_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int isl_remove(struct i2c_client *client);
static int isl_suspend(struct i2c_client *client, pm_message_t mesg);
static int isl_resume(struct i2c_client *client);
static irqreturn_t isl_interrupt(int irq, void *dev_id);
static void isl_work_func(struct work_struct *work);
static int isl_open(struct inode *inode, struct file *file);
static int isl_close(struct inode *inode, struct file *file);
static int isl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static void __exit isl_exit(void);
static int i2c_write(struct i2c_client *client, char *buf, int count);
static int i2c_read(struct i2c_client *client, char *buf, int count);


static const struct i2c_device_id isl_id[] = {
	{ ISL29018_DRIVER_NAME, 0 },
	{ }
};

/* Data for I2C driver */
struct isl_data {
	struct i2c_client *client;
	struct work_struct work;
	wait_queue_head_t wait;
};
static struct isl_data *isl_data;

/*File operation of ISL device file */
static const struct file_operations isl_fops = {
	.owner		= THIS_MODULE,
	.open		= isl_open,
	.release	= isl_close,
	.ioctl		= isl_ioctl,
};

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

static struct miscdevice isl_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = ISL29018_DRIVER_NAME,
	.fops = &isl_fops,
};

static int __init isl_init(void)
{
	int res;
	pr_info("[ISL] ISL29018 init\n");
	res = i2c_add_driver(&isl_driver);
	if (res) {
		pr_err("[ISL] %s: Driver Initialisation failed\n", __FILE__);
	}
	return res;
}

/* reset to initial value and confirm that is correct */
static int reset_and_check_value(struct i2c_client *client)
{
	unsigned char reg_buf[2] = {0};
	int i = 0;
	/* ISL29018_COMMAND_I_REG ~ ISL29018_INT_LT_MSB_REG: set value to zero and check it whether correct */
	for (i = 0x00; i <= 0x05; i++) {
		if (i==0x02 || i==0x03) {
			continue; /* output data register: read only, can't set */
		}
		reg_buf[0] = i;
		reg_buf[1] = 0x00;
		if (0!=i2c_write(client, reg_buf, 2)) {
			pr_err("[ISL] Can't write [%d] registry!\n", i);
			return -1;
		}
		if (0!=i2c_read(client, reg_buf, 1)) {
			pr_err("[ISL] Can't read [%d] registry!\n", i);
			return -1;
		}
		if (0x00!=reg_buf[0]) {
			pr_err("[ISL] Wrong init value[%X] in [%d] registry!\n", reg_buf[0], i);
			return -1;
		}
	}
	/* ISL29018_INT_HT_LSB_REG ~ ISL29018_INT_HT_MSB_REG: set value to 0xFF and check it whether correct */
	for (i = 0x06; i <= 0x07; i++) {
		reg_buf[0] = i;
		reg_buf[1] = 0xFF;
		if (0!=i2c_write(client, reg_buf, 2)) {
			pr_err("[ISL] Can't write [%d] registry!\n", i);
			return -1;
		}
		if (0!=i2c_read(client, reg_buf, 1)) {
			pr_err("[ISL] Can't read [%d] registry!\n", i);
			return -1;
		}
		if (0xFF!=reg_buf[0]) {
			pr_err("[ISL] Wrong init value[%X] in [%d] registry!\n", reg_buf[0], i);
			return -1;
		}
	}

	return 0;
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

	/* check i2c functionality is workable */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[ISL] i2c_check_functionality error!\n");
		res = -ENOTSUPP;
		goto out_free_mem;
	}
	strlcpy(client->name, ISL29018_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, isl_data);
	/* check isl29018 chip is workable */
	if (-1==reset_and_check_value(client)) {
		pr_err("[ISL] ISL29018 is not workable on i2c bus!\n");
		res = -ENXIO;
		goto out_free_mem;
	}

	/* request the gpio for interrupt */
	res = gpio_request(ISL29018_GPIO, "[ISL]");
	if (res < 0) {
		pr_err("[ISL] gpio_request error! error code: [%d]\n", res);
		goto out_free_mem;
	}
	/* initial wait queue and work function for interrupt triggered */
	INIT_WORK(&isl_data->work, isl_work_func);
	init_waitqueue_head(&isl_data->wait);
	/*  link interrupt routine with the irq */
	if (!client->irq) {
		pr_err("[ISL] client->irq => NULL!\n");
		res = -EFAULT;
		goto out_unreg_gpio;
	}
	res = request_irq(client->irq, isl_interrupt, IRQF_TRIGGER_FALLING,
			  ISL29018_DRIVER_NAME, isl_data);
	if (res < 0) {
		pr_err("[ISL] request_irq error! error code: [%d]\n", res);
		goto out_unreg_gpio;;
	}

	/* register misc device */
	res = misc_register(&isl_dev);
	if (res < 0) {
		pr_err("[BMA150]: bma150_dev register failed! error code:[%d]\n", res);
		goto out_unreg_irq;
	}

	pr_info("[ISL] probe done\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;

out_unreg_irq:
	free_irq(client->irq, isl_data);
out_unreg_gpio:
	gpio_free(ISL29018_GPIO);
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
	misc_deregister(&isl_dev);
	gpio_free(ISL29018_GPIO);
	free_irq(client->irq, tp);
	kfree(isl_data);
	pr_info("[ISL] remove done\n");
	return 0;
}

static int isl_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pr_debug("%s ++ entering\n", __FUNCTION__);
	disable_irq(client->irq);
	pr_debug("[BMA150] low power suspend init done.\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;
}

static int isl_resume(struct i2c_client *client)
{
	pr_debug("%s ++ entering\n", __FUNCTION__);
	enable_irq(client->irq);
	pr_debug("[BMA150] normal resume init done.\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;
}

static void isl_work_func(struct work_struct *work)
{
	/* TODO: make a notify to user */
}

static irqreturn_t isl_interrupt(int irq, void *dev_id)
{
	disable_irq(irq);
	schedule_work(&isl_data->work);
	enable_irq(irq);
	return IRQ_HANDLED;
}

/*	open command for ISL device file	*/
static int isl_open(struct inode *inode, struct file *file)
{
	struct i2c_client *client = isl_data->client;
	if( client == NULL ){
		pr_err("[ISL] I2C driver not install (isl_open)\n");
		return -1;
	}
	pr_debug("[ISL] has been opened\n");
	return 0;
}

static int isl_close(struct inode *inode, struct file *file)
{
	pr_debug("[ISL] has been closed\n");
	return 0;
}

static int isl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int isl_cmd = 0;
	unsigned char data[2] = {0};
	struct i2c_client *client = isl_data->client;

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
		case ISL29018_SET_MODE:
			if (0!=copy_from_user(data,(unsigned char*)arg,1)) {
				pr_err("[ISL] ISL29018_SET_MODE: copy_from_user error\n");
				return -EFAULT;
			}
			if (data[0]>ISL29018_PROX_CONTINUE_MODE) {
				pr_err("[ISL] ISL29018_SET_MODE: input value out of range: [%u]\n", data[0]);
				return -EFAULT;
			}
			_I2C_WRITE_BIT(ISL29018_MODE, data[0]);
			/* When setting off mode, reset IC interal state to prevent the wrong situation that output vaule equals zero */
			/* Note: This register is un-specifitcation, but we confirm this feature with Intersil */
			if (ISL29018_OFF_MODE==data[0]) {
				_I2C_READ_BIT(ISL29018_INTERNAL_STATE, data);
				if (0!=data[0]) {
					pr_info("[ISL] Correct IC internal state(reg#8) from [%d] to [0]\n", data[0]);
					data[1]=0;
					_I2C_WRITE_BIT(ISL29018_INTERNAL_STATE, data[1]); /* reset ic interal state */
				}
			}
			break;
		case ISL29018_GET_MODE:
			_I2C_READ_BIT(ISL29018_MODE, data);
			if (0!=copy_to_user((unsigned char*)arg,data,1)) {
				pr_err("[ISL] ISL29018_GET_MODE: copy_to_user error\n");
				return -EFAULT;
			}
			break;
		case ISL29018_GET_INT_FLAG:
			_I2C_READ_BIT(ISL29018_INT_FLAG, data);
			if (0!=copy_to_user((unsigned char*)arg,data,1)) {
				pr_err("[ISL] ISL29018_GET_INT_FLAG: copy_to_user error\n");
				return -EFAULT;
			}
			break;
		case ISL29018_SET_INT_PERSIST_CYCLE:
			if (0!=copy_from_user(data,(unsigned char*)arg,1)) {
				pr_err("[ISL] ISL29018_SET_INT_PERSIST_CYCLE: copy_from_user error\n");
				return -EFAULT;
			}
			if (data[0]>ISL29018_INT_PERSIST_CYCLE_16) {
				pr_err("[ISL] ISL29018_SET_INT_PERSIST_CYCLE: input value out of range: [%u]\n", data[0]);
				return -EFAULT;
			}
			_I2C_WRITE_BIT(ISL29018_INT_PERSIST_CYCLE, data[0]);
			break;
		case ISL29018_GET_INT_PERSIST_CYCLE:
			_I2C_READ_BIT(ISL29018_INT_PERSIST_CYCLE, data);
			if (0!=copy_to_user((unsigned char*)arg,data,1)) {
				pr_err("[ISL] ISL29018_GET_INT_PERSIST_CYCLE: copy_to_user error\n");
				return -EFAULT;
			}
			break;
		case ISL29018_SET_PROX_SCHEME:
			if (0!=copy_from_user(data,(unsigned char*)arg,1)) {
				pr_err("[ISL] ISL29018_SET_PROX_SCHEME: copy_from_user error\n");
				return -EFAULT;
			}
			if (data[0]>ISL29018_Scheme_1) {
				pr_err("[ISL] ISL29018_SET_PROX_SCHEME: input value out of range: [%u]\n", data[0]);
				return -EFAULT;
			}
			_I2C_WRITE_BIT(ISL29018_PROX_SCHEME, data[0]);
			break;
		case ISL29018_GET_PROX_SCHEME:
			_I2C_READ_BIT(ISL29018_PROX_SCHEME, data);
			if (0!=copy_to_user((unsigned char*)arg,data,1)) {
				pr_err("[ISL] ISL29018_GET_PROX_SCHEME: copy_to_user error\n");
				return -EFAULT;
			}
			break;
		case ISL29018_SET_MODULATION_FREQ:
			if (0!=copy_from_user(data,(unsigned char*)arg,1)) {
				pr_err("[ISL] ISL29018_SET_MODULATION_FREQ: copy_from_user error\n");
				return -EFAULT;
			}
			if (data[0]>ISL29018_Modu_Freq_360) {
				pr_err("[ISL] ISL29018_SET_MODULATION_FREQ: input value out of range: [%u]\n", data[0]);
				return -EFAULT;
			}
			_I2C_WRITE_BIT(ISL29018_MODULATION_FREQ, data[0]);
			break;
		case ISL29018_GET_MODULATION_FREQ:
			_I2C_READ_BIT(ISL29018_MODULATION_FREQ, data);
			if (0!=copy_to_user((unsigned char*)arg,data,1)) {
				pr_err("[ISL] ISL29018_GET_MODULATION_FREQ: copy_to_user error\n");
				return -EFAULT;
			}
			break;
		case ISL29018_SET_IRDR_CURRENT:
			if (0!=copy_from_user(data,(unsigned char*)arg,1)) {
				pr_err("[ISL] ISL29018_SET_IRDR_CURRENT: copy_from_user error\n");
				return -EFAULT;
			}
			if (data[0]>ISL29018_IRDR_CURRENT_100) {
				pr_err("[ISL] ISL29018_SET_IRDR_CURRENT input value out of range: [%u]\n", data[0]);
				return -EFAULT;
			}
			_I2C_WRITE_BIT(ISL29018_IRDR_CURRENT, data[0]);
			break;
		case ISL29018_GET_IRDR_CURRENT:
			_I2C_READ_BIT(ISL29018_IRDR_CURRENT, data);
			if (0!=copy_to_user((unsigned char*)arg,data,1)) {
				pr_err("[ISL] ISL29018_GET_IRDR_CURRENT: copy_to_user error\n");
				return -EFAULT;
			}
			break;
		case ISL29018_SET_RESOLUTION:
			if (0!=copy_from_user(data,(unsigned char*)arg,1)) {
				pr_err("[ISL] ISL29018_SET_RESOLUTION: copy_from_user error\n");
				return -EFAULT;
			}
			if (data[0]>ISL29018_RESOLUTION_4) {
				pr_err("[ISL] ISL29018_SET_RESOLUTION: input value out of range: [%u]\n", data[0]);
				return -EFAULT;
			}
			_I2C_WRITE_BIT(ISL29018_RESOLUTION, data[0]);
			break;
		case ISL29018_GET_RESOLUTION:
			_I2C_READ_BIT(ISL29018_RESOLUTION, data);
			if (0!=copy_to_user((unsigned char*)arg,data,1)) {
				pr_err("[ISL] ISL29018_GET_RESOLUTION: copy_to_user error\n");
				return -EFAULT;
			}
			break;
		case ISL29018_SET_RANGE:
			if (0!=copy_from_user(data,(unsigned char*)arg,1)) {
				pr_err("[ISL] ISL29018_SET_RANGE: copy_from_user error\n");
				return -EFAULT;
			}
			if (data[0]>ISL29018_RANGE_4) {
				pr_err("[ISL] ISL29018_SET_RANGE: input value out of range: [%u]\n", data[0]);
				return -EFAULT;
			}
			_I2C_WRITE_BIT(ISL29018_RANGE, data[0]);
			break;
		case ISL29018_GET_RANGE:
			_I2C_READ_BIT(ISL29018_RANGE, data);
			if (0!=copy_to_user((unsigned char*)arg,data,1)) {
				pr_err("[ISL] ISL29018_GET_RANGE: copy_to_user error\n");
				return -EFAULT;
			}
			break;
		case ISL29018_GET_DATA:
			/* hw limit */
			msleep(10);
			_I2C_READ_SHORT(ISL29018_DATA, data);
			if (0!=copy_to_user((short*)arg,(short*)data,2)) {
				pr_err("[ISL] ISL29018_GET_DATA: copy_to_user error\n");
				return -EFAULT;
			}
			break;
		case ISL29018_SET_INT_LOW_THRES:
			if (0!=copy_from_user((short*)data,(short*)arg,2)) {
				pr_err("[ISL] ISL29018_SET_INT_LOW_THRES: copy_from_user error\n");
				return -EFAULT;
			}
			_I2C_WRITE_SHORT(ISL29018_INT_LOW_THRES, data);
			break;
		case ISL29018_GET_INT_LOW_THRES:
			_I2C_READ_SHORT(ISL29018_INT_LOW_THRES, data);
			if (0!=copy_to_user((short*)arg,(short*)data,2)) {
				pr_err("[ISL] ISL29018_GET_INT_LOW_THRES: copy_to_user error\n");
				return -EFAULT;
			}
			break;
		case ISL29018_SET_INT_HIGH_THRES:
			if (0!=copy_from_user((short*)data,(short*)arg,2)) {
				pr_err("[ISL] ISL29018_SET_INT_HIGH_THRES: copy_from_user error\n");
				return -EFAULT;
			}
			_I2C_WRITE_SHORT(ISL29018_INT_HIGH_THRES, data);
			break;
		case ISL29018_GET_INT_HIGH_THRES:
			_I2C_READ_SHORT(ISL29018_INT_HIGH_THRES, data);
			if (0!=copy_to_user((short*)arg,(short*)data,2)) {
				pr_err("[ISL] ISL29018_GET_INT_HIGH_THRES: copy_to_user error\n");
				return -EFAULT;
			}
			break;
		default:
			pr_err("[ISL] ioctl cmd not found\n");
			return -EFAULT;
	}
	return 0;
}

static void __exit isl_exit(void)
{
	i2c_del_driver(&isl_driver);
	pr_info("[ISL] ISL29018 exit\n");
}

/*
 * client: target client
 * buf: target register
 * count: length of response
 */
static int i2c_read(struct i2c_client *client, char *buf, int count)
{
	//Send target reg. info.
	if(1 != i2c_master_send(client, buf, 1)){
		pr_err("[ISL] i2c_read --> Send reg. info error\n");
		return -1;
	}
	//Get response data and set to buf
	if(count != i2c_master_recv(client, buf, count)){
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
	//buf[0] -> target reg. info.
	//buf[1] -> cmd1
	//buf[2] -> cmd2
	//buf...
	//printk(KERN_ERR "[ISL] Write to reg: 0x%02X, cmd=0x%02X. \n", buf[0],  buf[1]);
	if(count != i2c_master_send(client, buf, count)){
		pr_err("[ISL] i2c_write --> Send reg. info error\n");
		return -1;
	}
	return 0;
}

module_init(isl_init);
module_exit(isl_exit);

MODULE_AUTHOR("Andrew <Andrew_Chen@acer.com.tw>");
MODULE_DESCRIPTION("i2c isl29018 driver");

