/* First release: 2009.07.08
 * TI's TPA2018 is an amp.s
 */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
#include <mach/tpa2018.h>
#include <mach/board.h>

#if 0
#define ACER_DBG(fmt, arg...) printk(KERN_INFO "[TPA2018]: %s " fmt "\n", __FUNCTION__, ## arg)
#else
#define ACER_DBG(fmt, arg...) do {} while (0)
#endif

#define TPA2018_DRIVER_NAME "tpa2018"

/* Stream Type definition */
#define STREAM_VOICE_CALL	0
#define STREAM_SYSTEM	1
#define STREAM_RING	2
#define STREAM_MUSIC	3
#define STREAM_ALARM	4
#define STREAM_NOTIFICATION	5
#define STREAM_BLUETOOTH_SCO	6

static int __init tpa2018_init(void);
static int tpa2018_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpa2018_remove(struct i2c_client *client);
static int tpa2018_open(struct inode *inode, struct file *file);
static int tpa2018_close(struct inode *inode, struct file *file);
static int i2c_write(struct i2c_client *client, char *buf, int count);
static int i2c_read(struct i2c_client *client, char *buf, int count);
static void tpa2018_arg_init(void);
static int tpa2018_check_gpio_and_regvalue(void);
static int tpa2018_set_limitor(int type);
static int tpa2018_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int tpa2018_suspend(struct i2c_client *client, pm_message_t mesg);
static int tpa2018_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void tpa2018_early_suspend(struct early_suspend *h);
static void tpa2018_early_resume(struct early_suspend *h);
#endif

static int adie_act_flag;
static bool tpa_act_flag;

static const struct i2c_device_id tpa2018_id[] = {
	{ TPA2018_DRIVER_NAME, 0 },
	{ }
};

static struct tpa2018_data {
	struct i2c_client *client;
	wait_queue_head_t wait;
	//int def_vol;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
} tpa2018_data;

static const struct file_operations tpa2018_fops = {
	.owner      = THIS_MODULE,
	.open       = tpa2018_open,
	.release    = tpa2018_close,
	.ioctl      = tpa2018_ioctl,
};

static struct i2c_driver tpa2018_driver = {
	.probe		= tpa2018_probe,
	.remove		= tpa2018_remove,
	.id_table	= tpa2018_id,
	.suspend	= tpa2018_suspend,
	.resume		= tpa2018_resume,
	.driver		= {
	.name = TPA2018_DRIVER_NAME,
	},
};

static struct miscdevice tpa2018_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = TPA2018_DRIVER_NAME,
	.fops = &tpa2018_fops,
};

static int i2c_read(struct i2c_client *client, char *buf, int count){
	if(1 != i2c_master_send(client, buf, 1)){
		pr_err("[TPA2018] i2c_read --> Send reg. info error\n");
		return -1;
	}

	if(count != i2c_master_recv(client, buf, count)){
		pr_err("[TPA2018] i2c_read --> get response error\n");
		return -1;
	}
	return 0;
}

static int i2c_write(struct i2c_client *client, char *buf, int count){
	if(count != i2c_master_send(client, buf, count)){
		pr_err("[TPA2018] i2c_write --> Send reg. info error\n");
		return -1;
	}
	return 0;
}

static void tpa2018_arg_init(void)
{
	int count;
	uint8_t tpa_wBuf_v3[8]={1,195,1,1,0,24,24,80}; /* 195,1,1,0,26, limit = 4.5, max = 23 ratio = 1:1 */
	uint8_t tpa_wBuf[8]={1,195,1,1,0,26,24,80}; /* 195,1,1,0,30, limit = 4.5, max = 23 ratio = 1:1 */
	uint8_t tpa_rBuf[7]={0};
	struct i2c_client *client = tpa2018_data.client;

	msleep(10);
	if (hw_version <= 3)
		i2c_write(client, tpa_wBuf_v3, 8);
	else
		i2c_write(client, tpa_wBuf, 8);

	tpa_rBuf[0]=1;
	i2c_read(client, tpa_rBuf, 7);
	for(count=0;count<7;count++)
		ACER_DBG("init - reg[%d] = %d\n", count, tpa_rBuf[count]);

	tpa2018_check_gpio_and_regvalue();
}

static int tpa2018_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res;

	tpa2018_data.client = client;

	/* spk_amp_en - speaker amplifier enable*/
	res = gpio_request(SPK_AMP_EN, "SPK AMP EN");
	if (res) {
		pr_err("GPIO request for SPK AMP EN failed!\n");
		goto gpio_err;
	}
	gpio_set_value(SPK_AMP_EN, 1);

	pr_debug("[TPA2018] Probe!!\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[TPA2018] i2c_check_functionality error!\n");
		return -ENOTSUPP;
	}
	strlcpy(client->name, TPA2018_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, &tpa2018_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	tpa2018_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	tpa2018_data.early_suspend.suspend = tpa2018_early_suspend;
	tpa2018_data.early_suspend.resume = tpa2018_early_resume;
	register_early_suspend(&tpa2018_data.early_suspend);
#endif

	res = misc_register(&tpa2018_dev);
	if (res) {
		pr_err("tpa2018_probe: tpa2018_dev register failed\n");
		goto error_tpa2018_dev;
	}

	tpa2018_arg_init();

	pr_info("[TPA2018] probe done\n");
	return 0;

gpio_err:
	gpio_free(SPK_AMP_EN);
error_tpa2018_dev:
	pr_err("[TPA2018] probe: tpa2018_dev error\n");
	return res;
}

static int tpa2018_remove(struct i2c_client *client)
{
	ACER_DBG("remove tpa2018\n");
	gpio_free(SPK_AMP_EN);
	return 0;
}

static int tpa2018_open(struct inode *inode, struct file *file)
{
	pr_debug("[TPA2018] has been opened\n");
	return 0;
}

static int tpa2018_close(struct inode *inode, struct file *file)
{
	pr_debug("[TPA2018] has been closed\n");
	return 0;
}

static int tpa2018_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pr_debug("[TPA2018] low power suspend init done.\n");

	return 0;
}

static int tpa2018_resume(struct i2c_client *client)
{
	pr_debug("[TPA2018] normal resume init done.\n");

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tpa2018_early_suspend(struct early_suspend *h)
{
	pr_debug("[TPA2018] %s ++ entering\n", __FUNCTION__);
	if (adie_act_flag == 0) {
		tpa2018_software_shutdown(1);
		ACER_DBG("tpa2018_software_shutdown(1)!!\n");
	}
	pr_debug("[TPA2018] %s -- leaving\n", __FUNCTION__);
}

static void tpa2018_early_resume(struct early_suspend *h)
{
	pr_debug("[TPA2018] %s ++ entering\n", __FUNCTION__);
	if (adie_act_flag == 0) {
		if (!gpio_get_value(SPK_AMP_EN)) {
			gpio_set_value(SPK_AMP_EN, 1);
			ACER_DBG("SPK_AMP_EN pull high!!\n");
			tpa2018_arg_init();
		}
		tpa2018_software_shutdown(1);
	}
	pr_debug("[TPA2018] %s -- leaving\n", __FUNCTION__);
}
#endif

static int tpa2018_check_gpio_and_regvalue(void)
{
	int count;
	uint8_t tpa_rBuf[7]={0};
	struct i2c_client *client = tpa2018_data.client;

	tpa_rBuf[0]=1;
	i2c_read(client, tpa_rBuf, 7);

	for(count=0;count<7;count++)
		ACER_DBG("check - reg[%d] = %d\n", count, tpa_rBuf[count]);

	if (gpio_get_value(SPK_AMP_EN) == 0) {
		gpio_set_value(SPK_AMP_EN, 1);
		tpa2018_arg_init();
		return 0;
	}

	//if ( tpa_rBuf[0] != 195 )
		//tpa2018_set_control(1, 1, 195);
	if ( tpa_rBuf[1] != 1 )
		tpa2018_set_control(1, 2, 1);
	if ( tpa_rBuf[2] != 1 )
		tpa2018_set_control(1, 3, 1);
	if ( tpa_rBuf[3] != 0 )
		tpa2018_set_control(1, 4, 0);
	//if ( tpa_rBuf[5] != 28 )
		//tpa2018_set_control(1, 6, 28); /* 28 */
	//if ( tpa_rBuf[6] != 192 )
		//tpa2018_set_control(1, 7, 192); /* 192 */

	return 0;
}

static int tpa2018_set_limitor(int type)
{
	switch(type) {
		case STREAM_VOICE_CALL:
			tpa2018_set_control(1, 7, 128);
			if (hw_version <= 3) {
				tpa2018_set_control(1, 6, 22);
				ACER_DBG(" Set TPA2018 limiter which is 4.5dBv!! \n");
			} else {
				tpa2018_set_control(1, 6, 24);
				ACER_DBG(" Set TPA2018 limiter which is 5.5dBv!! \n");
			}
			return 0;

		case STREAM_SYSTEM:
			tpa2018_set_control(1, 7, 128);
			if (hw_version <= 3) {
				tpa2018_set_control(1, 6, 22);
				ACER_DBG(" Set TPA2018 limiter which is 4.5dBv!! \n");
			} else {
				tpa2018_set_control(1, 6, 24);
				ACER_DBG(" Set TPA2018 limiter which is 5.5dBv!! \n");
			}
			return 0;

		case STREAM_RING:
			tpa2018_set_control(1, 7, 128);

			if (hw_version <= 3) {
				tpa2018_set_control(1, 6, 22);
				ACER_DBG(" Set TPA2018 limiter which is 4.5dBv!! \n");
			} else {
				tpa2018_set_control(1, 6, 24);
				ACER_DBG(" Set TPA2018 limiter which is 5.5dBv!! \n");
			}
			return 0;

		case STREAM_MUSIC:
			tpa2018_set_control(1, 7, 80);
			if (hw_version <= 3) {
				tpa2018_set_control(1, 6, 18);
				ACER_DBG(" Set TPA2018 limiter which is 4.0dBv!! \n");
			} else {
				tpa2018_set_control(1, 6, 21);
				ACER_DBG(" Set TPA2018 limiter which is 5.0dBv!! \n");
			}
			return 0;

		case STREAM_ALARM:
			tpa2018_set_control(1, 7, 128);
			if (hw_version <= 3) {
				tpa2018_set_control(1, 6, 24);
				ACER_DBG(" Set TPA2018 limiter which is 6.0dBv!! \n");
			} else {
				tpa2018_set_control(1, 6, 26);
				ACER_DBG(" Set TPA2018 limiter which is 7.5dBv!! \n");
			}
			return 0;

		case STREAM_NOTIFICATION:
			tpa2018_set_control(1, 7, 128);
			if (hw_version <= 3) {
				tpa2018_set_control(1, 6, 22);
				ACER_DBG(" Set TPA2018 limiter which is 4.5dBv!! \n");
			} else {
				tpa2018_set_control(1, 6, 24);
				ACER_DBG(" Set TPA2018 limiter which is 5.5dBv!! \n");
			}
			return 0;

		default:
			tpa2018_set_control(1, 7, 128);
			if (hw_version <= 3) {
				tpa2018_set_control(1, 6, 24);
				ACER_DBG(" Set TPA2018 limiter which is 6.0dBv!! \n");
			} else {
				tpa2018_set_control(1, 6, 26);
				ACER_DBG(" Set TPA2018 limiter which is 7.5dBv!! \n");
			}
			return 0;
	}
}

void set_adie_flag(int flag)
{
	ACER_DBG("adie flag = %d \n", flag);
	adie_act_flag = flag;
}

int get_adie_flag(void)
{
	int flag;
	flag = adie_act_flag;
	ACER_DBG("adie flag = %d \n", flag);
	return flag;
}

int tpa2018_software_shutdown(int command)
{
#if 1
	if (tpa_act_flag)
		return 0;
#endif

	tpa2018_check_gpio_and_regvalue();

	if (command == 1) {
		ACER_DBG("software shutdown = true \n");
		tpa2018_set_control(1, 1, 227);
	} else {
		ACER_DBG("software shutdown = false \n");
		tpa2018_set_control(1, 1, 195);
	}

	return 0;
}

int tpa2018_set_control(int commad, int regiter, int value)
{
	uint8_t tpa_wBuf[2];
	uint8_t tpa_rBuf[2];

	struct i2c_client *client = tpa2018_data.client;

	switch(commad){
		case 1:
			tpa_wBuf[0] = regiter;
			tpa_wBuf[1] = value;
			i2c_write(client, tpa_wBuf, 2);
			ACER_DBG("[TPA2018] WRITE GAIN CONTROL \n");
			msleep(1);
			return 0;

		case 2:
			tpa_wBuf[0] = regiter;
			tpa_wBuf[1] = value;
			i2c_read(client, tpa_rBuf, 1);
			ACER_DBG("[TPA2018] READ GAIN CONTROL \n");
			msleep(1);
			return tpa_rBuf[0];

		default:
			pr_err("[TPA2018]: Command not found!\n");
			return -1;
	}
}

static int tpa2018_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	u32 uparam;

	struct i2c_client *client = tpa2018_data.client;

	pr_debug("[TPA2018] tpa2018 ioctl \n");
	if(_IOC_TYPE(cmd) != TPA2018_IOCTL_MAGIC){
		pr_err("[TPA2018] IOCTL: cmd magic type error\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > IOC_MAXNR){
		pr_err("[TPA2018] IOCTL: cmd number error\n");
		return -ENOTTY;
	}
	if(_IOC_DIR(cmd) & _IOC_NONE)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	if(err){
		pr_err("[TPA2018] IOCTL: cmd access right error\n");
		return -EFAULT;
	}
	if( client == NULL){
		pr_err("[TPA2018] IOCTL: I2C driver not install (tpa2018_ioctl)\n");
		return -EFAULT;
	}

	switch(cmd){
		case TPA2018_SET_FIXED_GAIN:
			if (copy_from_user(&uparam, (void *)arg,
					sizeof(uparam)))
				return -1;
			ACER_DBG(" uparam = %d.\n", uparam);
			if (hw_version <= 3) {
				uparam*=20;
				ACER_DBG(" uparam*=20 %d.\n", uparam);
			} else {
				uparam*=26;
				ACER_DBG(" uparam*=26 %d.\n", uparam);
			}
			uparam/=100;
			ACER_DBG(" uparam/=100 = %d.\n", uparam);
			tpa2018_set_control(1, 5, uparam);
			return 0;

		case TPA2018_SET_STREAM_TYPE:
			if (copy_from_user(&uparam, (void *)arg,
					sizeof(uparam)))
				return -1;
			ACER_DBG(" Stream Type = %d.\n", uparam);
			tpa2018_set_limitor(uparam);
			return 0;

		case TPA2018_OPEN:
			tpa2018_software_shutdown(0);
			tpa_act_flag = true;
			return 0;

		case TPA2018_CLOSE:
			if (tpa_act_flag)
				tpa2018_software_shutdown(1);
			tpa_act_flag = false;
			return 0;

		default:
			pr_err("[TPA2018] IOCTL: Command not found!\n");
			return -1;
	}
}

static void __exit tpa2018_exit(void)
{
	i2c_del_driver(&tpa2018_driver);
}

static int __init tpa2018_init(void)
{
	int res=0;

	res = i2c_add_driver(&tpa2018_driver);
	if (res){
		pr_err("[TPA2018]i2c_add_driver failed! \n");
		return res;
	}

	pr_info("[TPA2018] tpa2018 device init ok!\n");
	return 0;
}

module_init(tpa2018_init);
module_exit(tpa2018_exit);

MODULE_AUTHOR("Andyl Liu <Andyl_Liu@acer.com.tw>");
MODULE_DESCRIPTION("TPA2018-380 driver");
