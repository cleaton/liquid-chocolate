#if defined (CONFIG_ACER_DEBUG)
#define DEBUG
#endif

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include "smd_private.h"
#include <mach/msm_rpcrouter.h>
#include <linux/delay.h>
#include <mach/board.h>
#include <linux/earlysuspend.h>

#define BATTERY_DRIVER_NAME             "acer-battery"
#define POLLING_TIME                    5000 /* milliseconds */
#define POLLING_TIME_NO_CHARGER         30000 /* milliseconds */
#define I2C_CMD_TEMP                    0x06
#define I2C_CMD_VOLTAGE                 0x08
#define I2C_CMD_NAC                     0x0c
#define I2C_CMD_LMD                     0x0e
#define UNKNOWN                         0
#define MAX_TEMPERATURE                 450 /* celsius 45.0 */
#define RE_CHARG_TEMP                   400 /* celsius 40.0 */
#define FAKE_BATT_VOLT                  3900 /* voltage 3.9 V */
#define CONVERT_TEMPERATURE(DATA,DATA1) (((DATA <<8)|DATA1)*10/4) -2730

#define PMAPP_GENPROG                   0x30000060
#define PMAPP_GENVERS                   0x00010002
#define CHARGER_NOTIFY_CB_PROC          100

struct bq27210_data {
	bool have_battery;
	uint16_t voltage;
	uint8_t capacity;
	int temperature;
	uint8_t health;
	int fs_data;
	bool bFirst;

	acer_smem_flag_t *charge_type;

	struct power_supply ac;
	struct power_supply usb;
	struct power_supply battery;

	struct i2c_client *client;
	struct timer_list polling_timer;
	struct work_struct work;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_resume;
#endif

};

static struct i2c_driver battery_driver;
static struct platform_driver bq27210_device;
static void polling_timer_func(unsigned long unused);

static struct bq27210_data *battery_data;

/* Send CallBack ID to Server*/
static int cb_register_arg(struct msm_rpc_client *client, void *buf, void *data)
{
	int size = 0;
	*((uint32_t *)buf) = cpu_to_be32(1);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	return size;
}

/* Server will Callback this function and reply procedure code for checking it*/
static int batt_cb_func(struct msm_rpc_client *client, void *buffer, int in_size)
{
	int ret = 0;
	struct rpc_request_hdr *req;
	req = (struct rpc_request_hdr *)buffer;

	switch (be32_to_cpu(req->procedure)) {
	case CHARGER_NOTIFY_CB_PROC:
		schedule_work(&battery_data->work);
		msm_rpc_start_accepted_reply(client, be32_to_cpu(req->xid),
						RPC_ACCEPTSTAT_SUCCESS);
		ret = msm_rpc_send_accepted_reply(client, 0);
		if (ret)
			pr_err("[BATT] msm_rpc_send_accepted_reply error \n");
		break;
	default:
		pr_err("[BATT] %s: procedure not supported %d\n", __func__,
						be32_to_cpu(req->procedure));
		break;
	}
	return ret;
}

static ssize_t set_charging(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	if (buf[0] == '0') {
		battery_data->charge_type->acer_batt_temp_info = ACER_BATT_TEMP_ERROR_LV0;
		battery_data->fs_data = 1;
	} else if(buf[0] == '2'){
		battery_data->charge_type->acer_batt_temp_info = ACER_BATT_TEMP_OK;
		battery_data->fs_data = 2;
	} else {
		battery_data->charge_type->acer_batt_temp_info = ACER_BATT_TEMP_OK;
		battery_data->fs_data = 0;
	}
	pr_info("[BATT] charger status : %s (%d) Charger type = %d\n",
		(battery_data->charge_type->acer_batt_temp_info)?"Disable":"Enable",
		battery_data->fs_data,
		battery_data->charge_type->acer_charger_type);
	return count;
}

static struct device_attribute battery_attrs =
__ATTR(charging, S_IRUGO| S_IWUSR | S_IWGRP,NULL, set_charging);

static int read_batt_status(uint8_t *data_addr , int count)
{
	if (1 != i2c_master_send(battery_data->client, data_addr, 1))
		goto i2c_err;
	if (count !=i2c_master_recv(battery_data->client, data_addr, count))
		goto i2c_err;
	return 0;
i2c_err:
	pr_debug("[BATT] get property i2c error\n");
	return -1;
}

static void battery_work(struct work_struct *work)
{
	uint8_t data[2] = { 0 };
	int32_t nac = 0;
	int32_t lmd = 0;
	int new_capacity = 0;
	int new_temp = 0;
	static uint8_t wCount[3] = {0};/* [0]: no data [1]: capacity [2]: temperature */

	/* Get voltage */
	data[0] = I2C_CMD_VOLTAGE;
	if(read_batt_status(data,2))
		goto no_data;
	battery_data->voltage = (((uint16_t)data[1])<<8)|data[0];

	/* Get temperature */
	data[0] = I2C_CMD_TEMP;
	if(read_batt_status(data,2))
		goto no_data;
	new_temp = CONVERT_TEMPERATURE((uint16_t)data[1], data[0]);
	if((abs(new_temp - battery_data->temperature) < 50)
				|| battery_data->bFirst == 1 || wCount[2] >= 5) {
		battery_data->temperature = new_temp;
		wCount[2] = 0;
	} else {
		pr_info("[BATT] new_temp = %d\n",new_temp);
		wCount[2]++;
	}

	/* Transform NAC and LMD into SOC */
	data[0] = I2C_CMD_NAC;
	if(read_batt_status(data,2))
		goto no_data;
	nac = (((uint16_t)data[1])<<8)|data[0];

	data[0] = I2C_CMD_LMD;
	if(read_batt_status(data,2))
		goto no_data;
	lmd = (((uint16_t)data[1])<<8)|data[0];
	if(!lmd){
		pr_err("[BATT] Wrong data: lmd = 0\n");
		wCount[0]++;
		if(wCount[0] >= 5)
			goto wrong_data;
		return;
	}

	new_capacity = ((nac*5000-lmd*600)/(lmd*43));

	if((abs(new_capacity - battery_data->capacity) < 5)
				|| battery_data->bFirst == 1 || wCount[1] >= 5) {

		if(new_capacity >= 100) {
			battery_data->capacity = 100;

		} else if(new_capacity <= 0) {
			pr_info("[BATT] capacity = 0\n");
			battery_data->capacity = 0;

		} else {
			battery_data->capacity = new_capacity;
		}
		if((battery_data->capacity != 0) || wCount[1] >= 5) {
			battery_data->bFirst = 0;
			wCount[1] = 0;
		}
	} else {
		pr_err("[BATT] nac =0x%x lmd = 0x%x new_capacity = %d\n",
							nac,lmd,new_capacity);
		wCount[1]++;
		return;
	}

	battery_data->have_battery = 1;
	goto change_status;

no_data:
	wCount[0]++;
	if( wCount[0] < 5 ){
		/* Waiting for i2c release */
		msleep(50);
		schedule_work(&battery_data->work);
		return;
	}
wrong_data:
	battery_data->have_battery = 0;
	pr_err("[BATT] No battery !!!!!!!\n");
	if(!test_mode)
		battery_data->charge_type->acer_batt_temp_info = ACER_BATT_TEMP_ERROR_LV0;
change_status:
	power_supply_changed(&battery_data->ac);
	power_supply_changed(&battery_data->usb);
	power_supply_changed(&battery_data->battery);
	wCount[0] = 0;
}

static void polling_timer_func(unsigned long unused)
{
	schedule_work(&battery_data->work);
	if(ACER_CHARGER_TYPE_NO_CHARGER == battery_data->charge_type->acer_charger_type) {
		mod_timer(&battery_data->polling_timer, jiffies +
			msecs_to_jiffies(POLLING_TIME_NO_CHARGER));
	}
	else {
		mod_timer(&battery_data->polling_timer, jiffies +
			msecs_to_jiffies(POLLING_TIME));
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void battery_early_resume(struct early_suspend *h)
{
	battery_data->bFirst =1;
	schedule_work(&battery_data->work);
}
#endif

static int ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (ACER_CHARGER_TYPE_IS_AC ==
				battery_data->charge_type->acer_charger_type)?1:0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (ACER_CHARGER_TYPE_IS_USB ==
				battery_data->charge_type->acer_charger_type)?1:0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	if(!battery_data->have_battery && !test_mode) {
		val->intval = UNKNOWN;
		return 0;
	}
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = ((ACER_CHARGER_TYPE_NO_CHARGER ==
				battery_data->charge_type->acer_charger_type) || (
				battery_data->charge_type->acer_batt_temp_info !=
				ACER_BATT_TEMP_OK) || (100 == battery_data->capacity))?
				POWER_SUPPLY_STATUS_NOT_CHARGING:
				POWER_SUPPLY_STATUS_CHARGING;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if(test_mode == 1) {
			battery_data->health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		} else if(battery_data->temperature > MAX_TEMPERATURE ||
				(battery_data->health == POWER_SUPPLY_HEALTH_OVERHEAT &&
				battery_data->temperature > RE_CHARG_TEMP)) {
			battery_data->health = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else if(battery_data->temperature < 0){
			battery_data->health = POWER_SUPPLY_HEALTH_COLD;
		} else {
			battery_data->health = POWER_SUPPLY_HEALTH_GOOD;
		}
		val->intval = battery_data->health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battery_data->voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = battery_data->capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = battery_data->temperature;
		if (!battery_data->fs_data) {
			if((ACER_CHARGER_TYPE_IS_AC ==
				battery_data->charge_type->acer_charger_type)
			||(ACER_CHARGER_TYPE_IS_USB ==
				battery_data->charge_type->acer_charger_type))
			{
				if(ACER_BATT_TEMP_ERROR_LV0 ==
					battery_data->charge_type->acer_batt_temp_info ||
				   ACER_BATT_TEMP_ERROR_LV1 ==
					battery_data->charge_type->acer_batt_temp_info)
				{
					if(battery_data->temperature <= RE_CHARG_TEMP &&
							battery_data->temperature >= 0 ) {
						battery_data->charge_type->acer_batt_temp_info =
						ACER_BATT_TEMP_OK;
					}
				} else {
					if(battery_data->temperature > MAX_TEMPERATURE ||
							battery_data->temperature < 0 ) {
						pr_info("[BATT] Temperature anomaly(%d)!\n",
							battery_data->temperature);
						battery_data->charge_type->acer_batt_temp_info =
						ACER_BATT_TEMP_ERROR_LV1;
					}
				}
			}
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int bq27210_probe(struct platform_device *pdev)
{
	int ret = 0;
	int retry = 0;
	static struct msm_rpc_client *rpc_clt = NULL;
	pr_debug("[BATT] Enter %s\n",__func__);
	battery_data = kzalloc(sizeof(*battery_data), GFP_KERNEL);
	if (battery_data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	memset(battery_data,0,sizeof(*battery_data));
	battery_data->bFirst =1;

	battery_data->ac.properties = ac_props;
	battery_data->ac.num_properties = ARRAY_SIZE(ac_props);
	battery_data->ac.get_property = ac_get_property;
	battery_data->ac.name = "ac";
	battery_data->ac.type = POWER_SUPPLY_TYPE_MAINS;

	battery_data->usb.properties = usb_props;
	battery_data->usb.num_properties = ARRAY_SIZE(usb_props);
	battery_data->usb.get_property = usb_get_property;
	battery_data->usb.name = "usb";
	battery_data->usb.type = POWER_SUPPLY_TYPE_USB;

	battery_data->battery.properties = battery_props;
	battery_data->battery.num_properties = ARRAY_SIZE(battery_props);
	battery_data->battery.get_property = battery_get_property;
	battery_data->battery.name = "battery";
	battery_data->battery.type = POWER_SUPPLY_TYPE_BATTERY;

	battery_data->charge_type = smem_alloc(SMEM_ID_VENDOR0, sizeof(acer_smem_flag_t));
	if(!battery_data->charge_type)
		goto err_data_alloc_failed;

	ret = i2c_add_driver(&battery_driver);
	if (ret)
		goto err_i2c_failed;

	ret = power_supply_register(&pdev->dev, &battery_data->ac);
	if (ret)
		goto err_ac_failed;

	ret = power_supply_register(&pdev->dev, &battery_data->usb);
	if (ret)
		goto err_usb_failed;

	ret = power_supply_register(&pdev->dev, &battery_data->battery);
	if (ret)
		goto err_battery_failed;

	ret = device_create_file(battery_data->battery.dev, &battery_attrs);
	if (ret)
		pr_err("[BATT] device_create_file error \n");

	platform_set_drvdata(pdev, battery_data);

	INIT_WORK(&battery_data->work, battery_work);

	/* Use RPC CallBack */
	do{
		rpc_clt = msm_rpc_register_client("batt_cb",PMAPP_GENPROG,
						PMAPP_GENVERS, 0,batt_cb_func);
		if (retry > 3) {
			pr_err("[BATT] msm_rpc_register_client error\n");
			goto err_data_alloc_failed;
		}
		if (!rpc_clt) {
			retry++;
			msleep(100);
		}
	}while(!rpc_clt);
	ret = msm_rpc_client_req(rpc_clt,CHARGER_NOTIFY_CB_PROC,
					cb_register_arg, NULL, NULL, NULL, -1);
	if (ret)
		pr_err("[BATT] msm_rpc_client_req error \n");

	setup_timer(&battery_data->polling_timer, polling_timer_func, 0);
	mod_timer(&battery_data->polling_timer, jiffies + msecs_to_jiffies(POLLING_TIME));

#ifdef CONFIG_HAS_EARLYSUSPEND
	battery_data->early_resume.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	battery_data->early_resume.resume = battery_early_resume;
	register_early_suspend(&battery_data->early_resume);
#endif
	pr_info("[BATT] probe done\n");
	return 0;

err_battery_failed:
	power_supply_unregister(&battery_data->battery);
err_usb_failed:
	power_supply_unregister(&battery_data->usb);
err_ac_failed:
	power_supply_unregister(&battery_data->ac);
err_i2c_failed:
	i2c_del_driver(&battery_driver);
err_data_alloc_failed:
	pr_err("[BATT] probe error\n");
	return ret;
}

static int bq27210_remove(struct platform_device *pdev)
{
	struct bq27210_data *battery_data = platform_get_drvdata(pdev);

	power_supply_unregister(&battery_data->ac);
	power_supply_unregister(&battery_data->usb);
	power_supply_unregister(&battery_data->battery);

	kfree(battery_data);
	battery_data = NULL;
	return 0;
}

static int battery_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	battery_data->client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		pr_err("[BATT] i2c_check_functionality error\n");

	strlcpy(client->name, BATTERY_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, &battery_data);

	return 0;
}

static int battery_remove(struct i2c_client *client)
{
	i2c_del_driver(&battery_driver);
	return 0;
}


static const struct i2c_device_id battery_id[] = {
	{ BATTERY_DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver battery_driver = {
	.probe		= battery_probe,
	.remove		= battery_remove,
	.id_table	= battery_id,
	.driver		= {
		.name = BATTERY_DRIVER_NAME,
	},
};

static struct platform_driver bq27210_device = {
	.probe		= bq27210_probe,
	.remove		= bq27210_remove,
	.driver = {
		.name = BATTERY_DRIVER_NAME
	}
};

static int __init bq27210_init(void)
{
	return platform_driver_register(&bq27210_device);
}

static void __exit bq27210_exit(void)
{
	platform_driver_unregister(&bq27210_device);
}

module_init(bq27210_init);
module_exit(bq27210_exit);

MODULE_AUTHOR("Allan Lin <Allan_Lin@acer.com.tw>");
MODULE_LICENSE("GPL v2");
