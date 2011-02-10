#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>

#include <mach/msm_rpcrouter.h>

#define PM_LIBPROG      0x30000061
#define PM_LIBVERS      0x10001

#define ONCRPC_PM_VIB_MOT_SET_VOLT_PROC 22
#define ONCRPC_PM_VIB_MOT_SET_MODE_PROC 23

static struct work_struct work_vibrator_on;
static struct work_struct work_vibrator_off;
static struct hrtimer vibe_timer;

static void set_pmic_vibrator(int on)
{
	static struct msm_rpc_endpoint *vib_endpoint;
	struct set_vib_on_off_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	if (!vib_endpoint) {
		vib_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
		if (IS_ERR(vib_endpoint)) {
			pr_err("[VIBRATOR] init vib rpc failed!\n");
			vib_endpoint = 0;
			return;
		}
	}
	req.data = 0;
	msm_rpc_call(vib_endpoint, ONCRPC_PM_VIB_MOT_SET_MODE_PROC, &req,
		sizeof(req), 5 * HZ);

	if (on)
		req.data = cpu_to_be32(3100);
	else
		req.data = cpu_to_be32(0);

	msm_rpc_call(vib_endpoint, ONCRPC_PM_VIB_MOT_SET_VOLT_PROC, &req,
		sizeof(req), 5 * HZ);

}

void pmic_vibrator_on(struct work_struct *work)
{
	set_pmic_vibrator(1);
}

void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(0);
}

static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_on);
}

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_off);
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	hrtimer_cancel(&vibe_timer);
	flush_work(&work_vibrator_on);
	flush_work(&work_vibrator_off);

	if (value == 0)
		timed_vibrator_off(dev);
	else {
		value = (value > 15000 ? 15000 : value);

		timed_vibrator_on(dev);

		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	timed_vibrator_off(NULL);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int __init init_pmic_vibrator(void)
{
	int res;
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	res = timed_output_dev_register(&pmic_vibrator);
	if(res){
		pr_err("[VIBRATOR]Init Error\n");
	}
	else{
		pr_info("[VIBRATOR]Init Done\n");
	}
	return res;
}

module_init(init_pmic_vibrator);

MODULE_AUTHOR("Allan Lin <Allan_Lin@acer.com.tw>");
MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

