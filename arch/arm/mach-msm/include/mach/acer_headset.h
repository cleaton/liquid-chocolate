/*
 * Acer Headset device detection driver
 *
 *
 * Copyright (C) 2009 acer Corporation.
 *
 * Authors:
 *    Lawrence Hou <Lawrence_Hou@acer.com.tw>
 */

#ifndef __ACER_HEADSET_H
#define __ACER_HEADSET_H

#include <linux/switch.h>
#include <asm/uaccess.h>
#define ACER_HS_IOCTL_MAGIC 'g'
#define ACER_HS_IOC_MAXNR	2

#define HPH_AMP_EN 39
#define EXPIRES 3

#define ACER_HS_CHANGE_CONTROL    _IO(ACER_HS_IOCTL_MAGIC, 1)
#define ACER_HS_ENABLE_AMP    _IO(ACER_HS_IOCTL_MAGIC, 2)

struct hs_res {
	struct switch_dev sdev;
	unsigned int det;
	unsigned int irq;
	unsigned int mic_bias_en;
	unsigned int hph_amp_en;
	bool headsetOn;
	struct hrtimer timer;
	ktime_t debounce_time;
};
extern void hs_amp(bool enable);
extern bool control;
#endif
