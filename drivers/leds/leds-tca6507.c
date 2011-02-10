#if defined (CONFIG_ACER_DEBUG)
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
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#define LED_DRIVER_NAME         "tca6507"
#define LED_DEV_IOCTLID         0x12
#define IOC_MAXNR               4
#define IOCTL_SET_MID_LED       _IOW(LED_DEV_IOCTLID, 1, int)
#define IOCTL_SET_RIGHT_LED     _IOW(LED_DEV_IOCTLID, 2, int)
#define IOCTL_SET_LEFT_LED      _IOW(LED_DEV_IOCTLID, 3, int)
#define IOCTL_SET_All_LED       _IOW(LED_DEV_IOCTLID, 4, int)

#define CTRL_REG_CMD_SELECT0    0x0
#define CTRL_REG_CMD_SELECT1    0x1
#define CTRL_REG_CMD_SELECT2    0x2

#define I2C_REG_LED_OFF         0x0
#define I2C_REG_MID_LED         0x1
#define I2C_REG_RIGHT_LED       0x2
#define I2C_REG_LEFT_LED        0x4
#define I2C_REG_ALL_LED         0x7

#define MID_LED_MASK            0xFE
#define RIGHT_LED_MASK          0xFD
#define LEFT_LED_MASK           0xFB
#define ALL_LED_MASK            0xF8

#define FADE_ON_TIME_ADDR           0x3
#define FULLY_ON_TIME_ADDR          0x4
#define FADE_OFF_TIME_ADDR          0x5
#define FIRST_FULLY_OFF_TIME_ADDR   0x6
#define SECOND_FULLY_OFF_TIME_ADDR  0x7
#define MAX_INTENSITY_ADDR          0x8

#define FADE_ON_TIME_DATA           0x31
#define FULLY_ON_TIME_DATA          0x40
#define FADE_OFF_TIME_DATA          0x31
#define FIRST_FULLY_OFF_TIME_DATA   0x62
#define SECOND_FULLY_OFF_TIME_DATA  0x62
#define MAX_INTENSITY_DATA          0xFF


#define BLINKING_TIMER_0        0x0	// TIMER(MS)    0
#define BLINKING_TIMER_1        0x1	//             64
#define BLINKING_TIMER_2        0x2	//            128
#define BLINKING_TIMER_3        0x3	//            192
#define BLINKING_TIMER_4        0x4	//            256  (DEFAULT TIMER)
#define BLINKING_TIMER_5        0x5	//            384
#define BLINKING_TIMER_6        0x6	//            512
#define BLINKING_TIMER_7        0x7	//            768
#define BLINKING_TIMER_8        0x8	//           1024
#define BLINKING_TIMER_9        0x9	//           1536
#define BLINKING_TIMER_10       0xA	//           2048
#define BLINKING_TIMER_11       0xB	//           3072
#define BLINKING_TIMER_12       0xC	//           4096
#define BLINKING_TIMER_13       0xD	//           5760
#define BLINKING_TIMER_14       0xE	//           8128
#define BLINKING_TIMER_15       0xF	//          16320

#define TCA6507_GPIO_EN_PIN     33

enum{
	LED_OFF,
	SLOW_BLINKING,
	FAST_BLINKING,
	ALWAYS_ON,
};

static int __init tca6507_init(void);
static void __exit tca6507_exit(void);
static int tca6507_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tca6507_remove(struct i2c_client *client);
static int tca6507_suspend(struct i2c_client *client, pm_message_t mesg);
static int tca6507_resume(struct i2c_client *client);
static int tca6507_close(struct inode *inode, struct file *file);
static int tca6507_open(struct inode *inode, struct file *file);
static int tca6507_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int i2c_read(struct i2c_client *client, char *buf, int count);
static int i2c_write(struct i2c_client *client, char *buf);
static struct i2c_driver tca6507_driver;
//
// Access through /sys for easier handling in homebrew.
static ssize_t mail_store(struct class *class, const char *buf,
		size_t count) {
	uint32_t value;
	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;
	if(value<0 || value > 3)
		return -EINVAL;
	tca6507_ioctl(NULL, NULL, IOCTL_SET_MID_LED, value);
	return count;
}

static ssize_t call_store(struct class *class, const char *buf,
		size_t count) {
	uint32_t value;
	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;
	if(value<0 || value > 3)
		return -EINVAL;
	tca6507_ioctl(NULL, NULL, IOCTL_SET_RIGHT_LED, value);
	return count;
}

static ssize_t power_store(struct class *class, const char *buf,
		size_t count) {
	uint32_t value;
	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;
	if(value<0 || value > 3)
		return -EINVAL;
	tca6507_ioctl(NULL, NULL, IOCTL_SET_LEFT_LED, value);
	return count;
}

//drivers/i2c/chips/avr.c
void avr_blink(int val);
static ssize_t bottom_store(struct class *class, const char *buf,
		size_t count) {
	uint32_t value;
	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;
	//0 leave it to default
	//1 fast blink (200ms)
	avr_blink(value);
	return count;
}

static struct class_attribute leds2_class_attrs[] = {
	__ATTR(mail, 0222, NULL, mail_store),
	__ATTR(call, 0222, NULL, call_store),
	__ATTR(power, 0222, NULL, power_store),
	__ATTR(bottom, 0222, NULL, bottom_store),
	__ATTR_NULL,
};

static struct class leds2_class = {
	.name = "leds2",
	.class_attrs = leds2_class_attrs,
};

static const struct i2c_device_id tca6507_id[] = {
	{ LED_DRIVER_NAME, 0 },
	{ }
};

static struct tca6507_data {
	struct i2c_client *client;
	wait_queue_head_t wait;
} tca6507_data;

static const struct file_operations tca6507_fops = {
	.owner    = THIS_MODULE,
	.open     = tca6507_open,
	.release  = tca6507_close,
	.ioctl    = tca6507_ioctl,
};

static struct i2c_driver tca6507_driver = {
	.probe    = tca6507_probe,
	.remove   = tca6507_remove,
	.suspend  = tca6507_suspend,
	.resume   = tca6507_resume,
	.id_table = tca6507_id,
	.driver   = {
	    .name = LED_DRIVER_NAME,
	},
};

static struct miscdevice tca6507_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = LED_DRIVER_NAME,
	.fops  = &tca6507_fops,
};

static int __init tca6507_init(void)
{
	int res = 0;
	pr_debug("led_tca6507_init +++\n");
	res = i2c_add_driver(&tca6507_driver);
	if (res){
		pr_err("[LED] i2c_add_driver failed! \n");
		return res;
	}
	pr_debug("led_tca6507_init ---\n");
	return 0;
}

static int tca6507_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	uint8_t i2c_buf[2] = {0};
	tca6507_data.client = client;

	err = gpio_request(TCA6507_GPIO_EN_PIN, "LED_DRIVER_EN");
	if(err){
		pr_err("[LED] gpio_request failed on pin %d (rc=%d)\n",TCA6507_GPIO_EN_PIN,err);
		goto error2;
	}
	/* Set tca6507_en_pin as output high*/
	gpio_direction_output(TCA6507_GPIO_EN_PIN,1);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[LED] i2c_check_functionality error!\n");
		return -ENOMEM;
	}
	strlcpy(client->name, LED_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, &tca6507_data);
	init_waitqueue_head(&tca6507_data.wait);

	i2c_buf[0] = CTRL_REG_CMD_SELECT0;
	i2c_buf[1] = I2C_REG_LED_OFF;
	err = i2c_write(client, i2c_buf);
	if(err)
		goto error;

	i2c_buf[0] = CTRL_REG_CMD_SELECT1;
	i2c_buf[1] = I2C_REG_LED_OFF;
	err = i2c_write(client, i2c_buf);
	if(err)
		goto error;

	i2c_buf[0] = CTRL_REG_CMD_SELECT2;
	i2c_buf[1] = I2C_REG_LED_OFF;
	err = i2c_write(client, i2c_buf);
	if(err)
		goto error;

	i2c_buf[0] = FADE_ON_TIME_ADDR;
	i2c_buf[1] = FADE_ON_TIME_DATA;
	err = i2c_write(client, i2c_buf);
	if(err)
		goto error;

	i2c_buf[0] = FULLY_ON_TIME_ADDR;
	i2c_buf[1] = FULLY_ON_TIME_DATA;
	err = i2c_write(client, i2c_buf);
	if(err)
		goto error;

	i2c_buf[0] = FADE_OFF_TIME_ADDR;
	i2c_buf[1] = FADE_OFF_TIME_DATA;
	err = i2c_write(client, i2c_buf);
	if(err)
		goto error;

	i2c_buf[0] = FIRST_FULLY_OFF_TIME_ADDR;
	i2c_buf[1] = FIRST_FULLY_OFF_TIME_DATA;
	err = i2c_write(client, i2c_buf);
	if(err)
		goto error;

	i2c_buf[0] = SECOND_FULLY_OFF_TIME_ADDR;
	i2c_buf[1] = SECOND_FULLY_OFF_TIME_DATA;
	err = i2c_write(client, i2c_buf);
	if(err)
		goto error;

	i2c_buf[0] = MAX_INTENSITY_ADDR;
	i2c_buf[1] = MAX_INTENSITY_DATA;
	err = i2c_write(client, i2c_buf);
	if(err)
		goto error;


	err = misc_register(&tca6507_dev);
	if (err)
		goto error;
	pr_debug("[LED]probe done\n");
	class_register(&leds2_class);
	return 0;
error:
	gpio_free(TCA6507_GPIO_EN_PIN);
error2:
	pr_err("[LED] probe error = 0x%x\n", err);
	return err;
}

static int tca6507_remove(struct i2c_client *client)
{
	gpio_free(TCA6507_GPIO_EN_PIN);
	misc_deregister(&tca6507_dev);
	return 0;
}

static int tca6507_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pr_debug("[led_tca6507_suspend +++\n");
	/*When suspend, led must be work.
	  So, keep GPIO PIN with high, don't do anything here*/
	pr_debug("led_tca6507_suspend ---\n");
	return 0;
}

static int tca6507_resume(struct i2c_client *client)
{
	pr_debug("[led_tca6507_resume +++\n");
	gpio_direction_output(TCA6507_GPIO_EN_PIN,1);
	pr_debug("led_tca6507_resume ---\n");
	return 0;
}

static int tca6507_open(struct inode *inode, struct file *file)
{
	struct i2c_client *client = tca6507_data.client;
	if( client == NULL ){
		return -1;
	}
	pr_debug("[LED] has been opened\n");
	return 0;
}

static int tca6507_close(struct inode *inode, struct file *file)
{
	pr_debug("[LED] has been closed\n");
	return 0;
}

static int tca6507_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	uint8_t select0_Buf[2],select1_Buf[2],select2_Buf[2];
	uint8_t select0[1] , select1[1] , select2[1];

	struct i2c_client *client = tca6507_data.client;

	if(_IOC_TYPE(cmd) != LED_DEV_IOCTLID){
		pr_err("[LED] cmd magic type error\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > IOC_MAXNR){
		pr_err("cmd number error, cmd number is %d\n",cmd);
		return -ENOTTY;
	}
	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err){
		pr_err("[LED] cmd access_ok error\n");
		return -EFAULT;
	}
	if( client == NULL){
		pr_err("[LED] I2C driver not install (tca6507_ioctl)\n");
		return -EFAULT;
	}

	select0[0] = CTRL_REG_CMD_SELECT0;
	i2c_read(client, select0, 1);
	select1[0] = CTRL_REG_CMD_SELECT1;
	i2c_read(client, select1, 1);
	select2[0] = CTRL_REG_CMD_SELECT2;
	i2c_read(client, select2, 1);

	select0_Buf[0] = CTRL_REG_CMD_SELECT0;
	select1_Buf[0] = CTRL_REG_CMD_SELECT1;
	select2_Buf[0] = CTRL_REG_CMD_SELECT2;

	switch(cmd){
		case IOCTL_SET_MID_LED:
			pr_debug("[LED] IOCTL_SET_MID_LED !!!!!\n");
			if(LED_OFF == arg){
				select2_Buf[1] = select2[0] & MID_LED_MASK;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] & MID_LED_MASK;
				i2c_write(client, select1_Buf);
			}else if(ALWAYS_ON == arg){
				select2_Buf[1] = select2[0] | I2C_REG_MID_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] & MID_LED_MASK;
				i2c_write(client, select1_Buf);
			}else if(FAST_BLINKING == arg){
				select2_Buf[1] = select2[0] | I2C_REG_MID_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] | I2C_REG_MID_LED;
				i2c_write(client, select1_Buf);
				select0_Buf[1] = select0[0] & MID_LED_MASK;
				i2c_write(client, select0_Buf);
			}else if(SLOW_BLINKING == arg){
				select2_Buf[1] = select2[0] | I2C_REG_MID_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] | I2C_REG_MID_LED;
				i2c_write(client, select1_Buf);
				select0_Buf[1] = select0[0] | I2C_REG_MID_LED;
				i2c_write(client, select0_Buf);
			}
			return err;
		case IOCTL_SET_RIGHT_LED:
			pr_debug("[LED] IOCTL_SET_RIGHT_LED\n");
			if(LED_OFF == arg){
				select2_Buf[1] = select2[0] & RIGHT_LED_MASK;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] & RIGHT_LED_MASK;
				i2c_write(client, select1_Buf);
			}else if(ALWAYS_ON == arg){
				select2_Buf[1] = select2[0] | I2C_REG_RIGHT_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] & RIGHT_LED_MASK;
				i2c_write(client, select1_Buf);
			}else if(FAST_BLINKING == arg){
				select2_Buf[1] = select2[0] | I2C_REG_RIGHT_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] | I2C_REG_RIGHT_LED;
				i2c_write(client, select1_Buf);
				select0_Buf[1] = select0[0] & RIGHT_LED_MASK;
				i2c_write(client, select0_Buf);
			}else if(SLOW_BLINKING == arg){
				select2_Buf[1] = select2[0] | I2C_REG_RIGHT_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] | I2C_REG_RIGHT_LED;
				i2c_write(client, select1_Buf);
				select0_Buf[1] = select0[0] | I2C_REG_RIGHT_LED;
				i2c_write(client, select0_Buf);
			}
			return err;
		case IOCTL_SET_LEFT_LED:
			pr_debug("[LED] IOCTL_SET_LEFT_LED\n");
			if(LED_OFF == arg){
				select2_Buf[1] = select2[0] & LEFT_LED_MASK;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] & LEFT_LED_MASK;
				i2c_write(client, select1_Buf);
			}else if(ALWAYS_ON == arg){
				select2_Buf[1] = select2[0] | I2C_REG_LEFT_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] & LEFT_LED_MASK;
				i2c_write(client, select1_Buf);
			}else if(FAST_BLINKING == arg){
				select2_Buf[1] = select2[0] | I2C_REG_LEFT_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] | I2C_REG_LEFT_LED;
				i2c_write(client, select1_Buf);
				select0_Buf[1] = select0[0] & LEFT_LED_MASK;
				i2c_write(client, select0_Buf);
			}else if(SLOW_BLINKING == arg){
				select2_Buf[1] = select2[0] | I2C_REG_LEFT_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] | I2C_REG_LEFT_LED;
				i2c_write(client, select1_Buf);
				select0_Buf[1] = select0[0] | I2C_REG_LEFT_LED;
				i2c_write(client, select0_Buf);
			}
			return err;
		case IOCTL_SET_All_LED:
			pr_debug("[LED] IOCTL_SET_ALL_LED\n");
			if(LED_OFF == arg){
				select2_Buf[1] = select2[0] & ALL_LED_MASK;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] & ALL_LED_MASK;
				i2c_write(client, select1_Buf);
			}else if(ALWAYS_ON == arg){
				select2_Buf[1] = select2[0] | I2C_REG_ALL_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] & ALL_LED_MASK;
				i2c_write(client, select1_Buf);
			}else if(FAST_BLINKING == arg){
				select2_Buf[1] = select2[0] | I2C_REG_ALL_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] | I2C_REG_ALL_LED;
				i2c_write(client, select1_Buf);
				select0_Buf[1] = select0[0] & ALL_LED_MASK;
				i2c_write(client, select0_Buf);
			}else if(SLOW_BLINKING == arg){
				select2_Buf[1] = select2[0] | I2C_REG_ALL_LED;
				i2c_write(client, select2_Buf);
				select1_Buf[1] = select1[0] | I2C_REG_ALL_LED;
				i2c_write(client, select1_Buf);
				select0_Buf[1] = select0[0] | I2C_REG_ALL_LED;
				i2c_write(client, select0_Buf);
			}
			return err;
		default:
			pr_err("[LED]IOCTL: Command not found!\n");
			return -1;
	}
}

static void __exit tca6507_exit(void)
{
	i2c_del_driver(&tca6507_driver);
}

static int i2c_read(struct i2c_client *client, char *buf, int count){
	if(1 != i2c_master_send(client, buf, 1)){
		pr_err("[LED] i2c_read --> Send reg. info error\n");
		return -1;
	}

	if(count != i2c_master_recv(client, buf, count)){
		pr_err("[LED] i2c_read --> get response error\n");
		return -1;
	}
	return 0;
}

static int i2c_write(struct i2c_client *client, char *buf){
	if(2 != i2c_master_send(client, buf, 2)){
		pr_err("[LED] i2c_write --> Send reg. info error\n");
		return -1;
	}
	return 0;
}


module_init(tca6507_init);
module_exit(tca6507_exit);

MODULE_AUTHOR("Clark  <Clark_Chen@acer.com.tw>");
MODULE_DESCRIPTION("i2c tca6507 driver");
