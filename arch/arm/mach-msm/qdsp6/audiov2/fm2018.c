/* First release: 2009.05.10
 * Foremedia's FM2018-380 is an echo canceller and noise suppressor.
 */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <mach/fm2018.h>

#define FM2018_DEV_IOCTLID	0x15
#define IOC_MAXNR	2

#define IOCTL_GET_FM_PD_STATUS	_IO(FM2018_DEV_IOCTLID, 1)
#define IOCTL_SET_FM_PD_DEFAULT	_IO(FM2018_DEV_IOCTLID, 2)

#define FM2018_DRIVER_NAME "fm2018"

static int __init fm2018_init(void);
static int fm2018_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int fm2018_remove(struct i2c_client *client);
static int fm2018_open(struct inode *inode, struct file *file);
static int fm2018_close(struct inode *inode, struct file *file);
static int fm2018_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int i2c_write(struct i2c_client *client, char *buf, int count);
static int i2c_read(struct i2c_client *client, char *buf, int count);
static int fm2018_write(int regH, int regL, int dataH, int dataL);
static int fm2018_read(int regH, int regL);
static void fm2018_power_seq(void);
static s32 fm2018_get_dm_gpios(void);
static void show_dn_gpio(void);

struct ard_denoise_mic_gpios dm_gpios;

static const struct i2c_device_id fm2018_id[] = {
	{ FM2018_DRIVER_NAME, 0 },
	{ }
};

static struct fm2018_data {
	struct i2c_client *client;
	wait_queue_head_t wait;
} fm2018_data;

static const struct file_operations fm2018_fops = {
	.owner      = THIS_MODULE,
	.open       = fm2018_open,
	.release    = fm2018_close,
	.ioctl      = fm2018_ioctl,
};

static struct i2c_driver fm2018_driver = {
	.probe		= fm2018_probe,
	.remove		= fm2018_remove,
	.id_table	= fm2018_id,
	.driver		= {
	.name = FM2018_DRIVER_NAME,
	},
};

static struct miscdevice fm2018_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FM2018_DRIVER_NAME,
	.fops = &fm2018_fops,
};

static int i2c_read(struct i2c_client *client, char *buf, int count){
	if(count != i2c_master_send(client, buf, count)){
		pr_err("[FM2018] i2c_read --> Send reg. info error\n");
		return -1;
	}

	if(1 != i2c_master_recv(client, buf, 1)){
		pr_err("[FM2018] i2c_read --> get response error\n");
		return -1;
	}
	return 0;
}

static int i2c_write(struct i2c_client *client, char *buf, int count){
	if(count != i2c_master_send(client, buf, count)){
		pr_err("[FM2018] i2c_write --> Send reg. info error\n");
		return -1;
	}
	return 0;
}

static int fm2018_read(int regH, int regL)
{
	uint8_t fm_wBuf[5];
	uint8_t fm_rBuf[4];
	int dataH, dataL, dataA;
	struct i2c_client *client = fm2018_data.client;

	fm_wBuf[0]=0xFC;
	fm_wBuf[1]=0xF3;
	fm_wBuf[2]=0x37;
	fm_wBuf[3]=regH;
	fm_wBuf[4]=regL;
	i2c_write(client, fm_wBuf, 5);
	msleep(1);

	/* Get high byte */
	fm_rBuf[0]=0xfc;
	fm_rBuf[1]=0xf3;
	fm_rBuf[2]=0x60;
	fm_rBuf[3]=0x26;
	i2c_read(client, fm_rBuf, 4);
	dataH = fm_rBuf[0];

	/* Get low byte */
	fm_rBuf[0]=0xfc;
	fm_rBuf[1]=0xf3;
	fm_rBuf[2]=0x60;
	fm_rBuf[3]=0x25;
	i2c_read(client, fm_rBuf, 4);
	dataL = fm_rBuf[0];

	dataA = dataH;
	dataA = dataA << 8;
	dataA = dataA | dataL;

	return dataA;
}

static int fm2018_write(int regH, int regL, int dataH, int dataL)
{
	uint8_t fm_wBuf[7];
	struct i2c_client *client = fm2018_data.client;

	fm_wBuf[0]=0xFC;
	fm_wBuf[1]=0xF3;
	fm_wBuf[2]=0x3B;
	fm_wBuf[3]=regH;
	fm_wBuf[4]=regL;
	fm_wBuf[5]=dataH;
	fm_wBuf[6]=dataL;
	i2c_write(client, fm_wBuf, 7);
	msleep(1);

	return 0;
}

static int fm2018_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	fm2018_data.client = client;

	pr_info("[FM2018] Probe!!\n");
	if (hw_version >= 4)
		goto error_fm2018_nodev;

	err = fm2018_get_dm_gpios();
	if (err<0) {
		pr_err("[FM2018] De-noise MIC GPIO configuration failed\n");
		show_dn_gpio();
		goto error_fm2018_dev;
	}

	fm2018_power_seq();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[FM2018] i2c_check_functionality error!\n");
		return -ENOTSUPP;
	}
	strlcpy(client->name, FM2018_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, &fm2018_data);

	init_waitqueue_head(&fm2018_data.wait);

	err = misc_register(&fm2018_dev);
	if (err) {
		pr_err("fm2018_probe: fm2018_dev register failed\n");
		goto error_fm2018_dev;
	}

	fm2018_set_procedure(SET_FM_PD_DEFAULT);
	mdelay(115);

	gpio_set_value(dm_gpios.bypass, 1);
	mdelay(10);

	gpio_set_value(dm_gpios.pwd, 0);
	mdelay(10);

	pr_info("[FM2018] probe done\n");
	return 0;

error_fm2018_dev:
	pr_err("[FM2018] probe error\n");
	return err;
error_fm2018_nodev:
	pr_err("[FM2018] probe error, v1.0 without fm2018!\n");
	return -ENOTSUPP;

}

static void show_dn_gpio(void)
{
	pr_info("FM2018 EN     = %d\n", gpio_get_value(dm_gpios.en));
	pr_info("FM2018 PWD    = %d\n", gpio_get_value(dm_gpios.pwd));
	pr_info("FM2018 CLK    = %d\n", gpio_get_value(dm_gpios.clk));
	pr_info("FM2018 RESET  = %d\n", gpio_get_value(dm_gpios.reset));
	pr_info("FM2018 BYPASS = %d\n", gpio_get_value(dm_gpios.bypass));
}

static void fm2018_power_seq(void)
{
	gpio_set_value(dm_gpios.en, 1);
	mdelay(10);
	gpio_set_value(dm_gpios.pwd, 1);
	gpio_set_value(dm_gpios.clk, 1);
	mdelay(5);
	gpio_set_value(dm_gpios.reset, 1);
	mdelay(30);
}

static s32 fm2018_get_dm_gpios(void)
{
	s32			rc;

	/* clk - de-noise mic clock enable */
	if(hw_version >= 3)
		dm_gpios.clk = DM_CLK_V03;
	else
		dm_gpios.clk = DM_CLK;
	rc = gpio_request(dm_gpios.clk, "De-noise MIC");
	if (rc) {
		pr_err("GPIO request for De-noise MIC clock failed!\n");
		return rc;
	}

	/* en - de-noise mic enable */
	if(hw_version >= 3)
		dm_gpios.en = DM_EN_V03;
	else
		dm_gpios.en = DM_EN;
	rc = gpio_request(dm_gpios.en, "De-noise MIC");
	if (rc) {
		pr_err("GPIO request for De-noise MIC enable failed!\n");
		return rc;
	}

	/* bypass - de-noise mic bypass */
	if(hw_version >= 3)
		dm_gpios.bypass = DM_BYPASS_V03;
	else
		dm_gpios.bypass = DM_BYPASS;
	rc = gpio_request(dm_gpios.bypass, "De-noise MIC");
	if (rc) {
		pr_err("GPIO request for De-noise MIC bypass failed!\n");
		return rc;
	}

	/* clk - de-noise mic reset */
	if(hw_version >= 3)
		dm_gpios.reset = DM_RESET_V03;
	else
		dm_gpios.reset = DM_RESET;
	rc = gpio_request(dm_gpios.reset, "De-noise MIC");
	if (rc) {
		pr_err("GPIO request for De-noise MIC reset failed!\n");
		return rc;
	}

	/* pwd - de-noise mic power */
	if(hw_version >= 3)
		dm_gpios.pwd = DM_PWD_V03;
	else
		dm_gpios.pwd = DM_PWD;
	rc = gpio_request(dm_gpios.pwd, "De-noise MIC");
	if (rc) {
		pr_err("GPIO request for De-noise MIC power failed!\n");
		return rc;
	}

	return rc;
}

static int fm2018_remove(struct i2c_client *client)
{
	misc_deregister(&fm2018_dev);

	gpio_free(dm_gpios.clk);
	gpio_free(dm_gpios.en);
	gpio_free(dm_gpios.bypass);
	gpio_free(dm_gpios.reset);
	gpio_free(dm_gpios.pwd);

	return 0;
}

/*	open command for fm2018 device file	*/
static int fm2018_open(struct inode *inode, struct file *file)
{
	pr_debug("[FM2018] has been opened\n");
	return 0;
}

static int fm2018_close(struct inode *inode, struct file *file)
{
	pr_debug("[FM2018] has been closed\n");
	return 0;
}

int fm2018_set_pwd(int com)
{
	gpio_set_value(dm_gpios.pwd, com);
	return 0;
}

int fm2018_set_procedure(int commad)
{
	uint16_t fmdata;

	switch(commad){
		case GET_FM_PD_STATUS:
			pr_debug("[FM2018] function_GET_LOW&HIGH_BYTE\n");
			fmdata = fm2018_read(0x1E, 0x3A);

			return fmdata;

		case SET_FM_PD_DEFAULT:
			pr_debug("[FM2018] IOCTL_SET_FM_2018 procedure!!!!!\n");
			fm2018_write(0x1E, 0x51, 0xC0, 0x00); /* no need to reload parameter */
			fm2018_write(0x1E, 0x3A, 0x00, 0x00);
			return 0;

		case SET_FM_PD_SW_BYPASS:
			pr_debug("[FM2018] SET_FM_PD_SW_BYPASS procedure!!!!!\n");
			fm2018_write(0x1E, 0x70, 0x05, 0xC0); //1E70 05C0
			fm2018_write(0x1E, 0x34, 0x00, 0x22); //1E34 0022
			fm2018_write(0x1E, 0x3D, 0x01, 0x00); //1E3D 0100
			fm2018_write(0x1E, 0x45, 0x00, 0x2D); //1E45 002D
			fm2018_write(0x1E, 0xDA, 0x40, 0x00); //1EDA 4000
			fm2018_write(0x1E, 0xF9, 0x03, 0x00); //1EF9 0300
			fm2018_write(0x1F, 0x00, 0x2D, 0xC8); //1F00 2DC8
			fm2018_write(0x1F, 0x01, 0x2F, 0x00); //1F01 2F00
			fm2018_write(0x1F, 0x0C, 0x03, 0x80); //1F0C 0380
			fm2018_write(0x1E, 0xFF, 0x28, 0x00); //1EFF 2800
			fm2018_write(0x1E, 0xA0, 0x20, 0x00); //1EA1 2000
			fm2018_write(0x1E, 0xA2, 0x20, 0x00); //1EA2 2000
			fm2018_write(0x1E, 0x44, 0x28, 0x89); //1E44 2889
			fm2018_write(0x1E, 0x4F, 0x00, 0x10); //1E4F 0010
			fm2018_write(0x1E, 0x51, 0xC0, 0x00); /* no need to reload parameter */
			fm2018_write(0x1E, 0x3A, 0x00, 0x00);
			return 0;

		case SET_FM_PD_ENV1:
			/*
			#ENVIRONMENT
			MicSet 18
			Mic0InLevel 0
			Mic1InLevel 0
			Mic0EchoLevel 0
			Mic1EchoLevel 0
			LineOutLevel 900
			LineInLevel 400
			VoltagDivider 1
			MicDiff 2.00
			MicNominalDiff 0.00
			TuningStep NULL
			*/

			fm2018_write(0x1E, 0x34, 0x00, 0xC9);//1E34 00C9
			fm2018_write(0x1E, 0x45, 0x00, 0x1E);//1E45 001E
			fm2018_write(0x1E, 0x47, 0x19, 0x00);//1E47 1900
			fm2018_write(0x1E, 0x52, 0x00, 0x13);//1E52 0013
			fm2018_write(0x1E, 0x58, 0x00, 0x13);//1E58 0013
			fm2018_write(0x1E, 0x60, 0x00, 0x00);//1E60 0000
			fm2018_write(0x1E, 0x70, 0x05, 0xC0);//1E70 05C0
			fm2018_write(0x1E, 0xA1, 0x21, 0x9A);//1EA1 219A
			fm2018_write(0x1F, 0x00, 0x2A, 0x62);//1F00 2A62
			fm2018_write(0x1F, 0x01, 0x28, 0x00);//1F01 2800
			fm2018_write(0x1F, 0x0D, 0x0F, 0x00);//1F0D 0F00
			fm2018_write(0x1F, 0x10, 0x7F, 0xFF);//1F10 7FFF
			fm2018_write(0x1F, 0x11, 0x60, 0x50);//1F11 6050
			fm2018_write(0x1F, 0x12, 0x4C, 0x9F);//1F12 4C9F
			fm2018_write(0x1F, 0x13, 0x3F, 0x82);//1F13 3F82
			fm2018_write(0x1F, 0x14, 0x36, 0x3E);//1F14 363E
			fm2018_write(0x1F, 0x15, 0x2E, 0xB2);//1F15 2EB2
			fm2018_write(0x1F, 0x16, 0x27, 0x11);//1F16 2711
			fm2018_write(0x1F, 0x17, 0x1E, 0xF8);//1F17 1EF8
			fm2018_write(0x1F, 0x18, 0x17, 0xCB);//1F18 17CB
			fm2018_write(0x1F, 0x19, 0x12, 0xF0);//1F19 12F0
			fm2018_write(0x1E, 0x51, 0xC0, 0x00); /* no need to reload parameter */
			fm2018_write(0x1E, 0x3A, 0x00, 0x00);
			return 0;

		default:
			pr_err("[FM2018]IOCTL: Command not found!\n");
			return -1;
	}
}

static int fm2018_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	struct i2c_client *client = fm2018_data.client;

	pr_debug("[FM2018] fm2018 ioctl \n");
	if(_IOC_TYPE(cmd) != FM2018_DEV_IOCTLID){
		pr_err("[FM2018] cmd magic type error\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > IOC_MAXNR){
		pr_err("cmd number error\n");
		return -ENOTTY;
	}
	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err){
		pr_err("[FM2018] cmd access_ok error\n");
		return -EFAULT;
	}
	if( client == NULL){
		pr_err("[FM2018] I2C driver not install (fm2018_ioctl)\n");
		return -EFAULT;
	}

	switch(cmd){
		case IOCTL_GET_FM_PD_STATUS:
			return fm2018_set_procedure(0);;

		case IOCTL_SET_FM_PD_DEFAULT:
			pr_debug("[FM2018] IOCTL_SET_FM_2018 procedure!!!!!\n");
			return fm2018_set_procedure(1);

		default:
			pr_err("[FM2018]IOCTL: Command not found!\n");
			return -1;
	}
}

static void __exit fm2018_exit(void)
{
	i2c_del_driver(&fm2018_driver);
}

static int __init fm2018_init(void)
{
	int res=0;

	if (hw_version >= 4) {
		pr_err("[FM2018] fm2018-380 device init failed, v1.0 without fm2018!\n");
		return -ENOTSUPP;
	}

	res = i2c_add_driver(&fm2018_driver);
	if (res){
		pr_err("[FM2018]i2c_add_driver failed! \n");
		return res;
	}

	pr_info("[FM2018] fm2018-380 device init ok!\n");
	return 0;
}

module_init(fm2018_init);
module_exit(fm2018_exit);

MODULE_AUTHOR("Andyl Liu <Andyl_Liu@acer.com.tw>");
MODULE_DESCRIPTION("FM2018-380 driver");
