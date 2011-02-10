/* arch/arm/mach-msm/include/mach/qdsp6/tpa2018.h
 *
 * Copyright (C) 2008 acer Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 */

#ifndef __LINUX_TPA2018_H
#define __LINUX_TPA2018_H
#endif

#include <linux/ioctl.h>

#define TPA2018_IOCTL_MAGIC 'f'
#define IOC_MAXNR	4
#define SPK_AMP_EN 142
#define CAMERA_EN 94

#define TPA2018_SET_FIXED_GAIN	_IO(TPA2018_IOCTL_MAGIC, 1)
#define TPA2018_SET_STREAM_TYPE	_IO(TPA2018_IOCTL_MAGIC, 2)
#define TPA2018_OPEN	_IO(TPA2018_IOCTL_MAGIC, 3)
#define TPA2018_CLOSE	_IO(TPA2018_IOCTL_MAGIC, 4)

extern int tpa2018_set_control(int commad, int regiter, int value);
extern int tpa2018_software_shutdown(int command);
extern void set_adie_flag(int flag);
extern int get_adie_flag(void);
