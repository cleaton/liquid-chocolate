/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2008 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#if defined(CONFIG_ACER_DEBUG)
#define DEBUG
#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>

#include "smb380.h"
#include "smb380calib.h"

#define BMA150_GPIO	22

#define BMA150_IOC_MAGIC 'B'

#define BMA150_SOFT_RESET			_IO(BMA150_IOC_MAGIC,0)
#define BMA150_GET_OFFSET			_IOWR(BMA150_IOC_MAGIC,1, short)
#define BMA150_SET_OFFSET			_IOWR(BMA150_IOC_MAGIC,2, short)
#define BMA150_SELFTEST				_IOWR(BMA150_IOC_MAGIC,3, unsigned char)
#define BMA150_SET_RANGE			_IOWR(BMA150_IOC_MAGIC,4, unsigned char)
#define BMA150_GET_RANGE			_IOWR(BMA150_IOC_MAGIC,5, unsigned char)
#define BMA150_SET_MODE				_IOWR(BMA150_IOC_MAGIC,6, unsigned char)
#define BMA150_GET_MODE				_IOWR(BMA150_IOC_MAGIC,7, unsigned char)
#define BMA150_SET_BANDWIDTH			_IOWR(BMA150_IOC_MAGIC,8, unsigned char)
#define BMA150_GET_BANDWIDTH			_IOWR(BMA150_IOC_MAGIC,9, unsigned char)
#define BMA150_SET_WAKE_UP_PAUSE		_IOWR(BMA150_IOC_MAGIC,10,unsigned char)
#define BMA150_GET_WAKE_UP_PAUSE		_IOWR(BMA150_IOC_MAGIC,11,unsigned char)
#define BMA150_SET_LOW_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,12,unsigned char)
#define BMA150_GET_LOW_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,13,unsigned char)
#define BMA150_SET_LOW_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,14,unsigned char)
#define BMA150_GET_LOW_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,15,unsigned char)
#define BMA150_SET_HIGH_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,16,unsigned char)
#define BMA150_GET_HIGH_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,17,unsigned char)
#define BMA150_SET_LOW_G_DURATION		_IOWR(BMA150_IOC_MAGIC,18,unsigned char)
#define BMA150_GET_LOW_G_DURATION		_IOWR(BMA150_IOC_MAGIC,19,unsigned char)
#define BMA150_SET_HIGH_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,20,unsigned char)
#define BMA150_GET_HIGH_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,21,unsigned char)
#define BMA150_SET_HIGH_G_DURATION		_IOWR(BMA150_IOC_MAGIC,22,unsigned char)
#define BMA150_GET_HIGH_G_DURATION		_IOWR(BMA150_IOC_MAGIC,23,unsigned char)
#define BMA150_SET_ANY_MOTION_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,24,unsigned char)
#define BMA150_GET_ANY_MOTION_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,25,unsigned char)
#define BMA150_SET_ANY_MOTION_COUNT		_IOWR(BMA150_IOC_MAGIC,26,unsigned char)
#define BMA150_GET_ANY_MOTION_COUNT		_IOWR(BMA150_IOC_MAGIC,27,unsigned char)
#define BMA150_SET_INTERRUPT_MASK		_IOWR(BMA150_IOC_MAGIC,28,unsigned char)
#define BMA150_GET_INTERRUPT_MASK		_IOWR(BMA150_IOC_MAGIC,29,unsigned char)
#define BMA150_RESET_INTERRUPT			_IO(BMA150_IOC_MAGIC,30)
#define BMA150_READ_ACCEL_X			_IOWR(BMA150_IOC_MAGIC,31,short)
#define BMA150_READ_ACCEL_Y			_IOWR(BMA150_IOC_MAGIC,32,short)
#define BMA150_READ_ACCEL_Z			_IOWR(BMA150_IOC_MAGIC,33,short)
#define BMA150_GET_INTERRUPT_STATUS		_IOWR(BMA150_IOC_MAGIC,34,unsigned char)
#define BMA150_SET_LOW_G_INT			_IOWR(BMA150_IOC_MAGIC,35,unsigned char)
#define BMA150_SET_HIGH_G_INT			_IOWR(BMA150_IOC_MAGIC,36,unsigned char)
#define BMA150_SET_ANY_MOTION_INT		_IOWR(BMA150_IOC_MAGIC,37,unsigned char)
#define BMA150_SET_ALERT_INT			_IOWR(BMA150_IOC_MAGIC,38,unsigned char)
#define BMA150_SET_ADVANCED_INT			_IOWR(BMA150_IOC_MAGIC,39,unsigned char)
#define BMA150_LATCH_INT			_IOWR(BMA150_IOC_MAGIC,40,unsigned char)
#define BMA150_SET_NEW_DATA_INT			_IOWR(BMA150_IOC_MAGIC,41,unsigned char)
#define BMA150_GET_LOW_G_HYST			_IOWR(BMA150_IOC_MAGIC,42,unsigned char)
#define BMA150_SET_LOW_G_HYST			_IOWR(BMA150_IOC_MAGIC,43,unsigned char)
#define BMA150_GET_HIGH_G_HYST			_IOWR(BMA150_IOC_MAGIC,44,unsigned char)
#define BMA150_SET_HIGH_G_HYST			_IOWR(BMA150_IOC_MAGIC,45,unsigned char)
#define BMA150_READ_ACCEL_XYZ			_IOWR(BMA150_IOC_MAGIC,46,short)
#define BMA150_READ_TEMPERATURE			_IOWR(BMA150_IOC_MAGIC,47,short)
#define BMA150_CALIBRATION			_IOWR(BMA150_IOC_MAGIC,48,short)

#define BMA150_IOC_MAXNR			49

#define BMA150_I2C_NAME "smb380"
#define BMA150_DEVICE_NAME "smb380"

static const struct i2c_device_id bma150_id[] = {
	{ BMA150_I2C_NAME, 0 },
	{ }
};

/* Data for I2C driver */
struct bma150_data {
	struct i2c_client *client;
	struct work_struct work;
	wait_queue_head_t wait;
};

static struct bma150_data* bma150_data;
static smb380_t smb380;

static int bma150_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int bma150_remove(struct i2c_client *client);
static int bma150_suspend(struct i2c_client *client, pm_message_t mesg);
static int bma150_resume(struct i2c_client *client);
static irqreturn_t bma150_interrupt(int irq, void *dev_id);
static void bma150_work_func(struct work_struct *work);

static char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len);
static char bma150_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len);
static void bma150_i2c_delay(unsigned int msec);

/* new style I2C driver struct */
static struct i2c_driver bma150_driver = {
	.probe		= bma150_probe,
	.remove		= __devexit_p(bma150_remove),
	.id_table	= bma150_id,
	.suspend	= bma150_suspend,
	.resume		= bma150_resume,
	.driver		= {
		.name = BMA150_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

/*	i2c delay routine for eeprom	*/
static inline void bma150_i2c_delay(unsigned int msec)
{
	mdelay(msec);
}

/*	i2c write routine for bma150	*/
static inline char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	//buf[0] -> target reg. info.
	//buf[1] -> cmd
	int nSetRealLen=0;
#define I2C_WRITE_LEN 2
	unsigned char buf[I2C_WRITE_LEN] = {0};
	if (NULL == bma150_data->client) {
		return -1;
	}
	buf[0] = reg_addr;
	buf[1] = data[0];
	nSetRealLen = i2c_master_send(bma150_data->client, buf, I2C_WRITE_LEN);
	if(I2C_WRITE_LEN != nSetRealLen){
		pr_err("[BMA150] i2c_write --> Send reg. info error\n");
	}
	return nSetRealLen - 1;	// ignore first byte(reg_addr) number
}

/*	i2c read routine for bma150	*/
static inline char bma150_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	int nGetRealLen=0;
	unsigned char buf[1] = {0};
	if (NULL == bma150_data->client) {
		return -1;
	}
	//Send target reg. info.
	buf[0] = reg_addr;
	if(1 != i2c_master_send(bma150_data->client, buf, 1)){
		pr_err("[BMA150] i2c_read --> Send reg. info error\n");
		return -1;
	}
	//Get response data and set to buf
	nGetRealLen = i2c_master_recv(bma150_data->client, data, len);
	if(len != nGetRealLen){
		pr_err("[BMA150] i2c_read --> get response error\n");
		return nGetRealLen;
	}
	return len;
}

/*	open command for BMA150 device file	*/
static int bma150_open(struct inode *inode, struct file *file)
{
	if( bma150_data->client == NULL)
	{
		pr_err("[BMA150] I2C driver not install (bma150_open)\n");
		return -1;
	}

	if (smb380.chip_id>0)
	{
		pr_debug("[BMA150] ChipId: 0x%x\n" , smb380.chip_id);
		pr_debug("[BMA150] ALVer: 0x%x MLVer: 0x%x\n", smb380.al_version, smb380.ml_version);
	}
	else
	{
		pr_err("[BMA150] BMA150: open error\n");
		return -1;
	}

#ifdef DEBUG
	smb380_set_bandwidth(0);
#endif
	pr_debug("[BMA150] BMA150 has been opened\n");
	return 0;
}

/*	release command for BMA150 device file	*/
static int bma150_close(struct inode *inode, struct file *file)
{
	pr_debug("[BMA150] BMA150 has been closed\n");
	return 0;
}


/*	ioctl command for BMA150 device file	*/
static int bma150_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	int temp;

	/* check cmd */
	if(_IOC_TYPE(cmd) != BMA150_IOC_MAGIC)
	{
		pr_err("[BMA150] cmd magic type error\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) >= BMA150_IOC_MAXNR)
	{
		pr_err("[BMA150] cmd number error\n");
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err)
	{
		pr_err("[BMA150] cmd access_ok error\n");
		return -EFAULT;
	}
	/* check bam150_client */
	if( bma150_data->client == NULL)
	{
		pr_err("[BMA150] I2C driver not install (bma150_ioctl)\n");
		return -EFAULT;
	}

	/* cmd mapping */

	switch(cmd)
	{
	case BMA150_SOFT_RESET:
		err = smb380_soft_reset();
		return err;

	case BMA150_GET_OFFSET:
		if(copy_from_user((unsigned short*)data,(unsigned short*)arg,4)!=0)
		{
			pr_err("[BMA150] BMA150_GET_OFFSET: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_get_offset(*((unsigned short*)data),(unsigned short*)(data+2));
		if(copy_to_user((unsigned short*)arg,(unsigned short*)data,4)!=0)
		{
			pr_err("[BMA150] BMA150_GET_OFFSET: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_OFFSET:
		if(copy_from_user((unsigned short*)data,(unsigned short*)arg,4)!=0)
		{
			pr_err("[BMA150] BMA150_SET_OFFSET: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_offset(*((unsigned short*)data),*(unsigned short*)(data+2));
		return err;

	case BMA150_SELFTEST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SELFTEST: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_selftest(*data);
		return err;

	case BMA150_SET_RANGE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_RANGE: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_range(*data);
		return err;

	case BMA150_GET_RANGE:
		err = smb380_get_range(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_RANGE: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_MODE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_MODE: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_mode(*data);
		return err;

	case BMA150_GET_MODE:
		err = smb380_get_mode(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_MODE: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_BANDWIDTH:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_BANDWIDTH: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_bandwidth(*data);
		return err;

	case BMA150_GET_BANDWIDTH:
		err = smb380_get_bandwidth(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_BANDWIDTH: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_WAKE_UP_PAUSE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_WAKE_UP_PAUSE: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_wake_up_pause(*data);
		return err;

	case BMA150_GET_WAKE_UP_PAUSE:
		err = smb380_get_wake_up_pause(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_WAKE_UP_PAUSE: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_LOW_G_THRESHOLD: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_low_g_threshold(*data);
		return err;

	case BMA150_GET_LOW_G_THRESHOLD:
		err = smb380_get_low_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_LOW_G_THRESHOLD: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_COUNTDOWN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_LOW_G_COUNTDOWN: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_low_g_countdown(*data);
		return err;

	case BMA150_GET_LOW_G_COUNTDOWN:
		err = smb380_get_low_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_LOW_G_COUNTDOWN: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_COUNTDOWN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_HIGH_G_COUNTDOWN: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_high_g_countdown(*data);
		return err;

	case BMA150_GET_HIGH_G_COUNTDOWN:
		err = smb380_get_high_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_HIGH_G_COUNTDOWN: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_LOW_G_DURATION: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_low_g_duration(*data);
		return err;

	case BMA150_GET_LOW_G_DURATION:
		err = smb380_get_low_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_LOW_G_DURATION: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_HIGH_G_THRESHOLD: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_high_g_threshold(*data);
		return err;

	case BMA150_GET_HIGH_G_THRESHOLD:
		err = smb380_get_high_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_HIGH_G_THRESHOLD: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_HIGH_G_DURATION: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_high_g_duration(*data);
		return err;

	case BMA150_GET_HIGH_G_DURATION:
		err = smb380_get_high_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_HIGH_G_DURATION: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_ANY_MOTION_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_ANY_MOTION_THRESHOLD: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_any_motion_threshold(*data);
		return err;

	case BMA150_GET_ANY_MOTION_THRESHOLD:
		err = smb380_get_any_motion_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_ANY_MOTION_THRESHOLD: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_ANY_MOTION_COUNT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_ANY_MOTION_COUNT: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_any_motion_count(*data);
		return err;

	case BMA150_GET_ANY_MOTION_COUNT:
		err = smb380_get_any_motion_count(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_ANY_MOTION_COUNT: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_INTERRUPT_MASK:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_INTERRUPT_MASK: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_interrupt_mask(*data);
		return err;

	case BMA150_GET_INTERRUPT_MASK:
		err = smb380_get_interrupt_mask(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_INTERRUPT_MASK: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_RESET_INTERRUPT:
		err = smb380_reset_interrupt();
		return err;

	case BMA150_READ_ACCEL_X:
		err = smb380_read_accel_x((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
			pr_err("[BMA150] BMA150_READ_ACCEL_X: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_READ_ACCEL_Y:
		err = smb380_read_accel_y((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
			pr_err("[BMA150] BMA150_READ_ACCEL_Y: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_READ_ACCEL_Z:
		err = smb380_read_accel_z((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
			pr_err("[BMA150] BMA150_READ_ACCEL_Z: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_GET_INTERRUPT_STATUS:
		err = smb380_get_interrupt_status(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_INTERRUPT_STATUS: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_LOW_G_INT: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_low_g_int(*data);
		return err;

	case BMA150_SET_HIGH_G_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_HIGH_G_INT: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_high_g_int(*data);
		return err;

	case BMA150_SET_ANY_MOTION_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_ANY_MOTION_INT: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_any_motion_int(*data);
		return err;

	case BMA150_SET_ALERT_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_ALERT_INT: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_alert_int(*data);
		return err;

	case BMA150_SET_ADVANCED_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_ADVANCED_INT: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_advanced_int(*data);
		return err;

	case BMA150_LATCH_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_LATCH_INT: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_latch_int(*data);
		return err;

	case BMA150_SET_NEW_DATA_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_NEW_DATA_INT: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_new_data_int(*data);
		return err;

	case BMA150_GET_LOW_G_HYST:
		err = smb380_get_low_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_LOW_G_HYST: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_HYST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_LOW_G_HYST: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_low_g_hysteresis(*data);
		return err;

	case BMA150_GET_HIGH_G_HYST:
		err = smb380_get_high_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_GET_HIGH_G_HYST: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_HYST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			pr_err("[BMA150] BMA150_SET_HIGH_G_HYST: copy_from_user error\n");
			return -EFAULT;
		}
		err = smb380_set_high_g_hysteresis(*data);
		return err;

	case BMA150_READ_ACCEL_XYZ:
		err = smb380_read_accel_xyz((smb380acc_t*)data);
		if(copy_to_user((smb380acc_t*)arg,(smb380acc_t*)data,6)!=0)
		{
			pr_err("[BMA150] BMA150_READ_ACCEL_XYZ: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	case BMA150_READ_TEMPERATURE:
		err = smb380_read_temperature(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			pr_err("[BMA150] BMA150_READ_TEMPERATURE: copy_to_user error\n");
			return -EFAULT;
		}
		return err;

	/* offset calibration routine */
	case BMA150_CALIBRATION:
		if(copy_from_user((smb380acc_t*)data,(smb380acc_t*)arg,6)!=0)
		{
			pr_err("[BMA150] BMA150_CALIBRATION: copy_from_user error\n");
			return -EFAULT;
		}
		/* iteration time 10 */
		temp = 10;
		err = smb380_calibrate(*(smb380acc_t*)data, &temp);
		return err;

	default:
		pr_err("[BMA150] ioctl cmd not found\n");
		return -EFAULT;
	}
}

static const struct file_operations bma150_fops = {
	.owner = THIS_MODULE,
	.open = bma150_open,
	.release = bma150_close,
	.ioctl = bma150_ioctl,
};

static struct miscdevice bma150_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = BMA150_DEVICE_NAME,
	.fops = &bma150_fops,
};

static void bma150_work_func(struct work_struct *work)
{
	/* TODO: make a notify to user */
}

static irqreturn_t bma150_interrupt(int irq, void *dev_id)
{
	disable_irq(irq);
	schedule_work(&bma150_data->work);
	enable_irq(irq);
	return IRQ_HANDLED;
}

static int bma150_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res;
	int comres;

	pr_debug("%s ++ entering\n", __FUNCTION__);
	bma150_data = kzalloc(sizeof(struct bma150_data),GFP_KERNEL);
	if (NULL==bma150_data) {
		res = -ENOMEM;
		goto out;
	}
	bma150_data->client = client;

	/* check i2c functionality is workable */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[BMA150] i2c_check_functionality error!\n");
		res = -ENOTSUPP;
		goto out_free_mem;
	}
	strlcpy(client->name, BMA150_I2C_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, bma150_data);
	/* check bma150 chip is workable */
	res = i2c_smbus_read_word_data(bma150_data->client, 0x00);
	if (res < 0) {
		pr_err("[BMA150] i2c_smbus_read_word_data(0x00) is failure! error code[%d]\n", res);
		goto out_free_mem;
	}
	else if ((res&0xFF) != 0x02) {
		pr_err("[BMA150] BMA150 is not registered 0x%x!\n", res);
		res = -ENXIO;
		goto out_free_mem;
	}
	/* set the i2c read/write/dealy function for bma150 subroutine */
	smb380.bus_write = bma150_i2c_write;
	smb380.bus_read = bma150_i2c_read;
	smb380.delay_msec = bma150_i2c_delay;
	comres = smb380_init(&smb380);
	if (10 != comres) {
		pr_err("[BMA150] smb380_init() is failure, comres:[%d]!\n", comres);
		res = -EIO;
		goto out_free_mem;
	}

	/* request the gpio for interrupt */
	res = gpio_request(BMA150_GPIO, "[BMA150]");
	if (res < 0) {
		pr_err("[BMA150] gpio_request error! error code: [%d]\n", res);
		goto out_free_mem;
	}
	/* initial wait queue and work function for interrupt triggered */
	INIT_WORK(&bma150_data->work, bma150_work_func);
	init_waitqueue_head(&bma150_data->wait);
	/*  link interrupt routine with the irq */
	if (!client->irq) {
		pr_err("[BMA150] client->irq => NULL!\n");
		res = -EFAULT;
		goto out_unreg_gpio;
	}
	res = request_irq(client->irq, bma150_interrupt, IRQF_TRIGGER_FALLING,
				  BMA150_DEVICE_NAME, bma150_data);
	if (res < 0) {
		pr_err("[BMA150] request_irq error! error code[%d]\n", res);
		goto out_unreg_gpio;
	}

	/* register misc device */
	res = misc_register(&bma150_dev);
	if (res < 0) {
		pr_err("[BMA150]: bma150_dev register failed! error code:[%d]\n", res);
		goto out_unreg_irq;
	}

	smb380_set_mode(SMB380_MODE_NORMAL);

	pr_info("[BMA150] probe done\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;

out_unreg_irq:
	free_irq(client->irq, bma150_data);
out_unreg_gpio:
	gpio_free(BMA150_GPIO);
out_free_mem:
	kfree(bma150_data);
out:
	pr_err("[BMA150] probe error\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return res;
}

static int bma150_remove(struct i2c_client *client)
{
	misc_deregister(&bma150_dev);
	free_irq(client->irq, bma150_data);
	gpio_free(BMA150_GPIO);
	kfree(bma150_data);
	pr_info("[BMA150] remove done\n");
	return 0;
}

static int bma150_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pr_debug("%s ++ entering\n", __FUNCTION__);
	disable_irq(client->irq);
	smb380_set_mode(SMB380_MODE_SLEEP);
	pr_debug("[BMA150] low power suspend init done.\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;
}

static int bma150_resume(struct i2c_client *client)
{
	pr_debug("%s ++ entering\n", __FUNCTION__);
	enable_irq(client->irq);
	smb380_set_mode(SMB380_MODE_NORMAL);
	/* for first read data integrity after power become normal(specification, page. 21) */
	smb380_pause(10);
	gpio_set_value(23, 1);
	pr_debug("[BMA150] normal resume init done.\n");
	pr_debug("%s -- leaving\n", __FUNCTION__);
	return 0;
}

static int __init bma150_init(void)
{
	int res;
	pr_info("[BMA150] BMA150 init\n");
	res = i2c_add_driver(&bma150_driver);
	if (res) {
		pr_err("[BMA150] %s: Driver Initialisation failed\n", __FILE__);
	}
	return res;
}

static void __exit bma150_exit(void)
{
	i2c_del_driver(&bma150_driver);
	pr_info("[BMA150] BMA150 exit\n");
}


MODULE_AUTHOR("Bin Du <bin.du@cn.bosch.com>");
MODULE_DESCRIPTION("BMA150 driver");
MODULE_LICENSE("GPL");

module_init(bma150_init);
module_exit(bma150_exit);

