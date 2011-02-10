/*
 * First release: 2009.04.15
 * - Enable keypad single touch mode
 * - IOCTL for backlight
 * - IOCTL for keypad toggle
 */
#if defined (CONFIG_ACER_DEBUG)
#define DEBUG
#endif

#include <linux/input.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <mach/msm_rpcrouter.h>

#define DEV_IOCTLID               0x11
#define IOC_MAXNR                 18
#define IOCTL_SET_BL_ON           _IOW(DEV_IOCTLID, 1, int)
#define IOCTL_SET_BL_OFF          _IOW(DEV_IOCTLID, 2, int)
#define IOCTL_SET_BL_LV           _IOW(DEV_IOCTLID, 3, int)
#define IOCTL_SET_LED1_ON         _IOW(DEV_IOCTLID, 4, int)
#define IOCTL_SET_LED1_OFF        _IOW(DEV_IOCTLID, 5, int)
#define IOCTL_SET_LED2_ON         _IOW(DEV_IOCTLID, 6, int)
#define IOCTL_SET_LED2_OFF        _IOW(DEV_IOCTLID, 7, int)
#define IOCTL_KEY_LOCK_TOGGLE     _IOW(DEV_IOCTLID, 8, int)
#define IOCTL_SET_LED_ON          _IOW(DEV_IOCTLID, 10, int)
#define IOCTL_SET_LED_OFF         _IOW(DEV_IOCTLID, 11, int)
#define IOCTL_SIMPLE_TEST_ON      _IOW(DEV_IOCTLID, 12, int)
#define IOCTL_SIMPLE_TEST_OFF     _IOW(DEV_IOCTLID, 13, int)
#define IOCTL_TEST_KEY            _IOW(DEV_IOCTLID, 15, int)
#define IOCTL_TEST_KEY_UP         _IOW(DEV_IOCTLID, 16, int)
#define IOCTL_TEST_KEY_DOWN       _IOW(DEV_IOCTLID, 17, int)

#define AVR_DRIVER_NAME           "avr"
#define KEY_STATUS                0xD4

#define MAX_BACKLIGHT_BRIGHTNESS  255
/* Modify AVR_BKL_LVL for backlight level 30 to 255 */
#define AVR_BKL_MAX_LVL           0x20
#define AVR_BKL_MIN_LVL           0x01
#define AVR_BKL_ON                AVR_BKL_MAX_LVL
#define AVR_BKL_OFF               0x00
#define I2C_REG_FW                0xD0
#define I2C_REG_LED_1             0xD1
#define I2C_REG_LED_2             0xD2
#define I2C_REG_BKL               0xD3
#define I2C_REG_KEY_STATUS        0xD4
#define I2C_REG_KEY_LOCK          0xD8
#define I2C_REG_LOW_POWER         0xD9
#define AVR_LED_MAX_LVL           0x20
#define AVR_LED_ON                AVR_LED_MAX_LVL
#define AVR_LED_OFF               0x00
#define AVR_POWER_NORMAL          0x00
#define AVR_POWER_LOW             0x01

/* AVR Keycode */
#define AVR_KEY_MENU              (1<<0)
#define AVR_KEY_LEFT              (1<<1)
#define AVR_KEY_DOWN              (1<<2)
#define AVR_KEY_RIGHT             (1<<3)
#define AVR_KEY_BACK              (1<<4)
#define AVR_KEY_UP                (1<<5)

#define AVR_KEYMASK_DIRECTION     (AVR_KEY_UP|AVR_KEY_DOWN|AVR_KEY_LEFT|AVR_KEY_RIGHT)

#define AVR_LED_DELAY_TIME        5000

#define BACKLIGHT_LEVEL_ON        0x8

/* AVR Sensitivity */
#define USE_FS                    1
#define SENSITIVITY_REG           0x60
#define SENSITIVITY               30

/* Vibrator */
#define VIB_DELAY_TIME        35
void pmic_vibrator_on(struct work_struct *work);
void pmic_vibrator_off(struct work_struct *work);

static int __init avr_init(void);
static int avr_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int avr_remove(struct i2c_client *client);
static int avr_suspend(struct i2c_client *client, pm_message_t mesg);
static int avr_resume(struct i2c_client *client);
static irqreturn_t avr_interrupt(int irq, void *dev_id);
static void avr_work_func(struct work_struct *work);
static int __init avr_register_input(struct input_dev *input);
static int avr_open(struct inode *inode, struct file *file);
static int avr_close(struct inode *inode, struct file *file);
static int avr_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int i2c_write(struct i2c_client *client, char *buf);
static int i2c_read(struct i2c_client *client, char *buf, int count);
static void led_on(struct i2c_client *client);
static void led_off(struct i2c_client *client);
static void low_power_mode(struct i2c_client *client, int mode);
static void key_clear(struct i2c_client *client);
static void avr_led_work_func(struct work_struct *work);
static void avr_vib_work_func(struct work_struct *work);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void avr_early_suspend(struct early_suspend *h);
static void avr_early_resume(struct early_suspend *h);
#endif
static int kpd_fw = 0xff;
static bool bsimple_test = false;
static bool kpd_fw_check = false;
static bool kpd_resume_check = true;
static bool kpd_pwr_key_check = false;
static struct mutex avr_mutex;
static struct delayed_work led_wq;
static struct delayed_work vib_wq;

static int vibr=1;
module_param(vibr, int, S_IRUGO | S_IWUSR | S_IWGRP);

static const struct i2c_device_id avr_id[] = {
	{ AVR_DRIVER_NAME, 0 },
	{ }
};

/* Data for I2C driver */
static struct avr_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	wait_queue_head_t wait;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	unsigned long last_jiffies;
	int prekey;
	int suspended;
} avr_data;

/*File operation of AVR device file */
static const struct file_operations avr_fops = {
	.owner     = THIS_MODULE,
	.open      = avr_open,
	.release   = avr_close,
	.ioctl     = avr_ioctl,
};

/* new style I2C driver struct */
static struct i2c_driver avr_driver = {
	.probe     = avr_probe,
	.remove    = avr_remove,
	.id_table  = avr_id,
	.suspend   = avr_suspend,
	.resume    = avr_resume,
	.driver    = {
		.name      = AVR_DRIVER_NAME,
	},
};

static struct miscdevice avr_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AVR_DRIVER_NAME,
	.fops = &avr_fops,
};

#if USE_FS

static uint8_t ts_atoi(const char *name)
{
    uint8_t val = 0;

    for (;; name++) {
	switch (*name) {
	    case '0' ... '9':
		val = 10*val+(*name-'0');
		break;
	    default:
		return val;
	}
    }
}

static ssize_t set_avr_sensitivity(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	uint8_t data_buf[2] = {0};

	if(!(kpd_fw>0x38))
		return count;

	data_buf[0] = SENSITIVITY_REG;
	data_buf[1] = ts_atoi(buf);

	pr_info("[AVR] Threshold value = %d\n",data_buf[1]);

	if ( 0 != i2c_write(avr_data.client, data_buf))
		pr_err("[AVR] Set AVR threshold value error\n");

	return count;
}

static struct device_attribute avr_attrs =
__ATTR(threshold, S_IRWXUGO,NULL, set_avr_sensitivity);

static ssize_t get_avr_firmware(struct device *dev, struct device_attribute *attr,
             char *buf)
{
	int fw_check = 0xff;
	fw_check = (kpd_fw>0x38) ? 1 : 0;

	pr_info("[AVR] Firmware ver=0x%02X, fw_check = %d\n", kpd_fw, fw_check);

	return sprintf(buf, "%d\n", fw_check);
}

static struct device_attribute avr_fw_attrs =
__ATTR(fw_check, S_IRUGO,get_avr_firmware, NULL);

#endif

static int avr_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	int err = 0;
	uint8_t data_buf[2] = {0};
	int count = 0;
	int rc;

	avr_data.client = client;
	avr_data.prekey = 1;
	avr_data.last_jiffies = 0;

	pr_debug("[AVR] %s ++ entering\n", __FUNCTION__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[AVR] i2c_check_functionality error!\n");
		return -ENOTSUPP;
	}
	strlcpy(client->name, AVR_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, &avr_data);

	mutex_init(&avr_mutex);

	INIT_WORK(&avr_data.work, avr_work_func);
	init_waitqueue_head(&avr_data.wait);

	INIT_DELAYED_WORK(&led_wq, avr_led_work_func);
	INIT_DELAYED_WORK(&vib_wq, avr_vib_work_func);

	/* input register */
	avr_data.input = input_allocate_device();
	if (avr_data.input == NULL) {
		pr_err("[AVR] input_allocate_device error!\n");
		return -ENOMEM;
	}

	err = avr_register_input(avr_data.input);
	if (err < 0) {
		pr_err("[AVR] AVR_register_input error\n");
		goto error;
	}

	/* Enable AVR chip by reset gpio 29, 91, It was moved to Touch driver. */
	if (client->irq) {
		err = request_irq(client->irq, avr_interrupt, IRQF_TRIGGER_FALLING,
				  AVR_DRIVER_NAME, &avr_data);
		if (err < 0) {
			pr_err("[AVR] request_irq error! %d\n", err);
			free_irq(client->irq, &avr_data);
		}
	}

	err = misc_register(&avr_dev);
	if (err) {
		pr_err("avr_probe: avr_dev register failed\n");
		goto error_avr_dev;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	/* To set BLANK_SCREEN level that prevent wrong-touch while talking */
	avr_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	avr_data.early_suspend.suspend = avr_early_suspend;
	avr_data.early_suspend.resume = avr_early_resume;
	register_early_suspend(&avr_data.early_suspend);
#endif

	data_buf[0] = I2C_REG_BKL;
	data_buf[1] = BACKLIGHT_LEVEL_ON;

	while( count < 5 ){
		rc = i2c_write(client, data_buf);

		if(rc){
			/* Retry if i2c_read error */
			msleep(200);
		}else{
			/* i2c_read success */
			break;
		}
		count++;
	}

	count = 0;

	/* Get keypad FW version */
	data_buf[0] = I2C_REG_FW;
	data_buf[1] = 0;

	while( count < 5 ){
		rc = i2c_read(client, data_buf, 1);
		if (rc) {
			/* Retry if i2c_read error */
			msleep(200);
		}else{
			/* i2c_read success */
			break;
		}
		count++;
	}

	if (rc) {
		pr_err("[AVR] i2c_read fail\n");
		goto error_avr_dev;
	}

	kpd_fw = data_buf[0];
	/* To check keypad for Type Proposal_A  / Type Proposal_C (0x30) */
	kpd_fw_check = (data_buf[0]>0x30) ? true : false;

	pr_info("[AVR] Firmware ver=0x%02X\n", data_buf[0]);

#if USE_FS
	if(device_create_file(&client->dev, &avr_attrs))
		pr_err("[AVR] device_create_file avr_attrs error \n");

	if(device_create_file(&client->dev, &avr_fw_attrs))
		pr_err("[AVR] device_create_file avr_fw_attrs error \n");
#endif

	pr_debug("[AVR] %s -- leaving\n", __FUNCTION__);

	return 0;

error_avr_dev:
	free_irq(client->irq, &avr_data);
error:
	input_free_device(avr_data.input);
	pr_err("[AVR] probe error\n");
	return err;
}

static int avr_remove(struct i2c_client *client)
{
	struct avr_data *tp = i2c_get_clientdata(client);
	input_unregister_device(tp->input);
	free_irq(client->irq, tp);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&tp->early_suspend);
#endif

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void avr_early_suspend(struct early_suspend *h)
{
	pr_debug("[AVR] %s ++ entering\n", __FUNCTION__);

	kpd_resume_check = false;
	avr_data.suspended=1;
	key_clear(avr_data.client);
	led_off(avr_data.client);
	disable_irq(avr_data.client->irq);
	low_power_mode(avr_data.client, 1);

	pr_debug("[AVR] %s -- leaving\n", __FUNCTION__);
}

static void avr_early_resume(struct early_suspend *h)
{
	pr_debug("[AVR] %s ++ entering\n", __FUNCTION__);

	low_power_mode(avr_data.client,0);
	enable_irq(avr_data.client->irq);

	avr_data.suspended=0;
	kpd_resume_check = true;

	if(kpd_pwr_key_check){
		kpd_pwr_key_check = false;
		led_on(avr_data.client);
	}

	pr_debug("[AVR] %s -- leaving\n", __FUNCTION__);
}
#endif

static int avr_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pr_debug("[AVR] low power suspend init done.\n");

	return 0;
}

static int avr_resume(struct i2c_client *client)
{
	pr_debug("[AVR] normal resume init done.\n");

	return 0;
}

static void avr_work_func(struct work_struct *work)
{
	struct i2c_client *client = avr_data.client;

	uint8_t data_buf[2] = {0};
	int key_st = 0;
	int key_code = 0;
	int count = 0;

	mutex_lock(&avr_mutex);

	/* Step 1. Scan Key */
	data_buf[0] = KEY_STATUS;
	data_buf[1] = 0;

	while( count < 5 ){
		if ( i2c_read(client, data_buf, 1) ) {
			if (count == 1){
				input_report_key(avr_data.input, avr_data.prekey, 0);
				avr_data.prekey = key_code;
			}
			/* Retry if i2c_read error */
			msleep(200);
		}else{
			/* i2c_read success */
			break;
		}
		count++;
	}
	/* Step 1. End Scan key */

	/* Step 2. Send Key event */
	key_st = data_buf[0];

	if(key_st == KEY_STATUS || key_st == kpd_fw || count > 1){
		key_clear(client);
		mutex_unlock(&avr_mutex);
		return;
	}
	
	cancel_delayed_work(&led_wq);

	/* TODO: Check KPD LED Function */
	if ( key_st != 0) 
		led_on(client);
	else
		schedule_delayed_work(&led_wq, msecs_to_jiffies(AVR_LED_DELAY_TIME));



	if(kpd_fw_check) {
		switch(key_st){
		case AVR_KEY_MENU:
			if(!bsimple_test)
				key_code = KEY_HOME;
			else
				key_code = KEY_SEND;
			break;
		case AVR_KEY_LEFT:
			key_code = KEY_SEARCH;
			break;
		case AVR_KEY_DOWN:
			key_code = KEY_BACK;
			break;
		case AVR_KEY_RIGHT:
			key_code = 0xE5; /* MENU */
			break;
		default:
			key_code = 0;
			break;
		}
	}
	else {
		switch(key_st){
		case AVR_KEY_MENU:
			if(!bsimple_test)
				key_code = KEY_HOME;
			else
				key_code = KEY_SEND;
			break;
		case AVR_KEY_LEFT:
			key_code = KEY_SEARCH;
			break;
		case AVR_KEY_RIGHT:
			key_code = KEY_BACK;
			break;
		case AVR_KEY_BACK:
			key_code = 0xE5; /* MENU */
			break;
		default:
			key_code = 0;
			break;
		}
	}

	pr_debug("%s: key_st=0x%x, key_code=0x%x, pre=0x%x\n",
             __func__, key_st, key_code, avr_data.prekey);

	/* Send key release if not equal to last key */
	if( key_code != avr_data.prekey ){
		input_report_key(avr_data.input, avr_data.prekey, 0);
		if(vibr == 1 && key_code != 0) {
			pmic_vibrator_on(NULL);
			schedule_delayed_work(&vib_wq, msecs_to_jiffies(VIB_DELAY_TIME));
		}
	}
	/* Send key press if key_code != 0 */
	if( key_code ) {
		input_report_key(avr_data.input, key_code, 1);
	}
	/* TODO: Add pressed check for gesture or miss touch. */
	avr_data.prekey = key_code;
	/* Step 2. End Send Key event */

	mutex_unlock(&avr_mutex);
}

static irqreturn_t avr_interrupt(int irq, void *dev_id)
{
	/* TODO: Remove mdelay() to prevent listening */
	/*       music delay on BT Headset via A2DP   */
	disable_irq(irq);
	schedule_work(&avr_data.work);
	enable_irq(irq);
	return IRQ_HANDLED;
}

static int __init avr_register_input(struct input_dev *input)
{
	input->name = AVR_DRIVER_NAME;
	input->id.bustype = BUS_I2C;
	input->evbit[0] = BIT_MASK(EV_SYN)|BIT_MASK(EV_KEY);
	input->keybit[BIT_WORD(KEY_HOME)] = BIT_MASK(KEY_HOME);
	input->keybit[BIT_WORD(KEY_BACK)] = BIT_MASK(KEY_BACK)|BIT_MASK(KEY_MENU);
	input->keybit[BIT_WORD(KEY_SEND)] |= BIT_MASK(KEY_SEND);
	input->keybit[BIT_WORD(0xE5)] |= BIT_MASK(0xE5);
	input->keybit[BIT_WORD(KEY_SEARCH)] |= BIT_MASK(KEY_SEARCH);
	return input_register_device(input);
}

/* open command for AVR device file	*/
static int avr_open(struct inode *inode, struct file *file)
{
	return 0;
}
/* close command for AVR device file */
static int avr_close(struct inode *inode, struct file *file)
{
	return 0;
}

static int avr_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	uint8_t data_buf[2] = {0};
	struct i2c_client *client = avr_data.client;
	uint32_t bl_lvl;

	/* check cmd */
	if(_IOC_TYPE(cmd) != DEV_IOCTLID){
		pr_err("cmd magic type error\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > IOC_MAXNR){
		pr_err("cmd number error\n");
		return -ENOTTY;
	}

	if( client == NULL){
		pr_err("I2C driver not install (AVR_ioctl)\n");
		return -EFAULT;
	}

	/* cmd mapping */
	switch(cmd){
	case IOCTL_SIMPLE_TEST_ON:
		bsimple_test = true;
		pr_debug("[AVR] IOCTL_bSIMPLE_TEST_ON. \n");
		break;
	case IOCTL_SIMPLE_TEST_OFF:
		bsimple_test = false;
		pr_debug("[AVR] IOCTL_bSIMPLE_TEST_ON. \n");
		break;
	case IOCTL_SET_LED_ON:
		if(kpd_resume_check){
			data_buf[0] = I2C_REG_LED_1;
			data_buf[1] = AVR_LED_ON;
			i2c_write(client, data_buf);

			data_buf[0] = I2C_REG_LED_2;
			data_buf[1] = AVR_LED_ON;
			i2c_write(client, data_buf);
		}else{
			kpd_pwr_key_check = true;
		}

		pr_debug("[AVR] IOCTL_SET_LED_ON.\n");
		break;
	case IOCTL_SET_LED_OFF:
		data_buf[0] = I2C_REG_LED_1;
		data_buf[1] = AVR_LED_OFF;
		i2c_write(client, data_buf);

		data_buf[0] = I2C_REG_LED_2;
		data_buf[1] = AVR_LED_OFF;
		i2c_write(client, data_buf);

		pr_debug("[AVR] IOCTL_SET_LED_OFF.\n");
		break;
	case IOCTL_SET_BL_ON:
		data_buf[0] = I2C_REG_BKL;
		data_buf[1] = AVR_BKL_ON;
		i2c_write(client, data_buf);

		pr_debug("[AVR] IOCTL_SET_BL_ON. \n");
		break;
	case IOCTL_SET_BL_OFF:
		data_buf[0] = I2C_REG_BKL;
		data_buf[1] = AVR_BKL_OFF;
		i2c_write(client, data_buf);

		pr_debug("[AVR] IOCTL_SET_BL_OFF. \n");
		break;
	case IOCTL_SET_BL_LV:
		/* This maps android backlight level 0 to 255 into
		   driver backlight level 0 to bl_max with rounding */
		/* Modify bl_lvl for backlight level 30 to 255 */
		bl_lvl = (2 * arg * (AVR_BKL_MAX_LVL+3) + MAX_BACKLIGHT_BRIGHTNESS)
			/(2 * MAX_BACKLIGHT_BRIGHTNESS) -3;

		/* Fixed for backlight level < 20 in timeout*/
		if(!bl_lvl) bl_lvl = 0x1;

		/* 2's Complement Check */
		if ((bl_lvl >>7) & 1){
			if (arg > 9)
				bl_lvl = 0x01; //Fixed for backlight level > 10 in timeout with L-sensor
			else
				bl_lvl = 0;
		}

		data_buf[0] = I2C_REG_BKL;
		data_buf[1] = bl_lvl;
		mutex_lock(&avr_mutex);
		i2c_write(client, data_buf);
		mutex_unlock(&avr_mutex);

		pr_debug("[AVR] IOCTL_SET_BL_LV, Set backlight 0x%02X (asked 0x%02X). \n", data_buf[1], arg);
		return err;
	case IOCTL_KEY_LOCK_TOGGLE:
		data_buf[0] = I2C_REG_KEY_LOCK;
		data_buf[1] = (unsigned int)arg;
		if(data_buf[1]!=0 || data_buf[1]!=1){
			data_buf[1]=0;
		}
		i2c_write(client, data_buf);
		break;
	case IOCTL_TEST_KEY:
		/* if touch locked, unlock it!. */
		data_buf[0] = I2C_REG_KEY_STATUS;
		data_buf[1] = (unsigned int)arg;
		if(data_buf[1]<=0 || data_buf[1]>=0x80){
			data_buf[1]=0;
		}
		i2c_write(client, data_buf);
		pr_debug("[AVR] IOCTL_TEST_KEY, Set key as 0x%02X. \n", data_buf[1]);
		return err;
	case IOCTL_TEST_KEY_UP:
		pr_debug("[AVR] IOCTL_TEST_KEY_UP, KEY %d UP! \n", (unsigned int)arg);
		input_report_key(avr_data.input, (unsigned int)arg, 0);
		break;
	case IOCTL_TEST_KEY_DOWN:
		pr_debug("[AVR] IOCTL_TEST_KEY_DOWN, KEY %d DOWN! \n", (unsigned int)arg);
		input_report_key(avr_data.input, (unsigned int)arg, 1);
		break;
	default:
		return -1;
	}

	return 0;
}

static void __exit avr_exit(void)
{
	i2c_del_driver(&avr_driver);
}

/*
 * client: target client
 * buf: target register
 * count: length of response
 */
static int i2c_read(struct i2c_client *client, char *buf, int count)
{
	/* Send target reg. info. */
	if(1 != i2c_master_send(client, buf, 1)){
		return -1;
	}

	/* Get response data and set to buf */
	if(count != i2c_master_recv(client, buf, count)){
		return -1;
	}
	return 0;
}

/*
 * client: target client
 * buf: target register with command
 */
static int i2c_write(struct i2c_client *client, char *buf)
{
	/* buf[0] -> target reg. info. */
	/* buf[1] -> cmd */
	if(2 != i2c_master_send(client, buf, 2)){
		return -1;
	}
	return 0;
}

static void led_on(struct i2c_client *client){
	uint8_t data_buf[2] = {0};

	if(!client)
		return;
	data_buf[0] = I2C_REG_LED_1;
	data_buf[1] = AVR_LED_ON;
	i2c_write(client, data_buf);

	mdelay(10);

	data_buf[0] = I2C_REG_LED_2;
	data_buf[1] = AVR_LED_ON;
	i2c_write(client, data_buf);
}

static void led_off(struct i2c_client *client){
	uint8_t data_buf[2] = {0};

	data_buf[0] = I2C_REG_LED_1;
	data_buf[1] = AVR_LED_OFF;
	i2c_write(client, data_buf);

	mdelay(10);

	data_buf[0] = I2C_REG_LED_2;
	data_buf[1] = AVR_LED_OFF;
	i2c_write(client, data_buf);
}

static void low_power_mode(struct i2c_client *client, int mode){
	uint8_t data_buf[2] = {0};

	if (mode) {
		data_buf[0] = I2C_REG_LOW_POWER;
		data_buf[1] = AVR_POWER_LOW;
		i2c_write(client, data_buf);
		pr_debug("[AVR] Enter Low Power\n");
	} else {
		data_buf[0] = I2C_REG_LOW_POWER;
		data_buf[1] = AVR_POWER_NORMAL;
		i2c_write(client, data_buf);
		pr_debug("[AVR] Enter Normal power\n");
	}
}

static void key_clear(struct i2c_client *client){
	/* To prevent uP holding KEY_INT pin to low without getting value */
	uint8_t data_buf[2] = {0};

	data_buf[0] = KEY_STATUS;
	data_buf[1] = 0;
	i2c_read(client, data_buf, 1);

	pr_debug("[AVR] Clear Key Value.\n");
}

static void avr_led_work_func(struct work_struct *work) {
	led_off(avr_data.client);
	pr_debug("[AVR] Enter LED delay 5 Sec\n");
}

static void avr_vib_work_func(struct work_struct *work) {
	pmic_vibrator_off(NULL);
}

//Blinking code

static struct delayed_work blink_wq;
static int status=0;

void avr_blink(int value) {
	status=0;
	if(value) {
		if(avr_data.suspended)
			low_power_mode(avr_data.client, 0);
		printk("Scheduling blinking\n");
		schedule_delayed_work(&blink_wq, msecs_to_jiffies(30));
	} else {
		cancel_delayed_work(&blink_wq);
		if(avr_data.suspended) {
			low_power_mode(avr_data.client, 1);
		} else {
			led_on(avr_data.client);
			printk("Scheduling 5s off\n");
			schedule_delayed_work(&led_wq, msecs_to_jiffies(AVR_LED_DELAY_TIME));
		}
	}
}

static void blink_work_func(struct work_struct *work) {
	status=!status;
	if(status) {
		printk("Blink turning on\n");
		led_on(avr_data.client);
		schedule_delayed_work(&blink_wq, msecs_to_jiffies(500));
	} else {
		printk("Blink turning off\n");
		led_off(avr_data.client);
		schedule_delayed_work(&blink_wq, msecs_to_jiffies(4500));
	}
}

static int __init avr_init(void)
{
	int res=0;

	INIT_DELAYED_WORK(&blink_wq, blink_work_func);
	cancel_delayed_work(&blink_wq);
	res = i2c_add_driver(&avr_driver);

	if (res){
		pr_err("[AVR]i2c_add_driver failed! \n");
		return res;
	}

	return 0;
}

module_init(avr_init);
module_exit(avr_exit);

MODULE_AUTHOR("Elay Hu <Elay_Hu@acer.com.tw>");
MODULE_DESCRIPTION("AVR micro-P driver");
