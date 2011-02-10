/*
 * Copyright (c) 2009, Acer Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include "gpio_chip.h"
#include "devices.h"
#include <mach/board.h>

#include <asm/mach/flash.h>
#include <asm/setup.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/android_pmem.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_iomap.h>
#include <asm/mach/mmc.h>
#include <mach/board.h>

unsigned hw_version=0;
unsigned lcm_id=0;
unsigned test_mode=0;
unsigned boot_in_recovery=0;
char AMSS_version[MAX_AMSS_LEN];
/* For USB Serial No */
char usb_serialno[MAX_USBSN_LEN];

#define RECOVERY_STRING "recovery"
#define OS_STRING "OS000000"

static int __init hw_version_setup(char *version)
{
	hw_version=(version[0]-'0');
	pr_debug("hw_version=%d\n",hw_version);
	return 1;
}
__setup("hw_ver=", hw_version_setup);


static int __init amss_version_setup(char *version)
{
	strncpy(AMSS_version,version,MAX_AMSS_LEN);
	pr_debug("amss_version=%s\n",AMSS_version);
	return 1;
}
__setup("amss_ver=", amss_version_setup);


static int __init lcm_id_setup(char *id)
{
	lcm_id=(id[0]-'0');
	pr_debug("lcm_id=%d\n",lcm_id);
	return 1;
}
__setup("lcm_id=", lcm_id_setup);


static int __init test_mode_setup(char *id)
{
	test_mode=(id[0]-'0');
	pr_debug("test_mode=%d\n",test_mode);
	return 1;
}
__setup("test_mode=", test_mode_setup);

/* For USB SN Usage */
static int __init usb_serialno_setup(char *serialno)
{
	int i;
	int len = 0;

	if(strncmp(serialno, RECOVERY_STRING, sizeof(RECOVERY_STRING)) == 0){
		//RECOVERY MODE!
		for(i=0;i<32;i++)
			usb_serialno[i] = 0;
		boot_in_recovery = 1;
		pr_debug("RECOVERY_MODE!\n");
		return 1;
	}else if(strncmp(serialno, OS_STRING, sizeof(OS_STRING)) == 0){
		//OS MODE and no serial number
		for(i=0;i<32;i++)
			usb_serialno[i] = 0;
		pr_debug("OS_MODE!\n");
		return 1;
	}else
		strncpy(usb_serialno,serialno,MAX_USBSN_LEN);

	len = strlen(usb_serialno);
	if(len < 16){
		usb_serialno[16] = 0;
		for(i=15; i>=0; i--){
			if(--len>=0){
				usb_serialno[i] = usb_serialno[len];
			}else{
				usb_serialno[i] = '0';
			}
		}
	}

	pr_debug("usb_serialno=%s, len = %d\n",usb_serialno, strlen(usb_serialno));
	return 1;
}
__setup("androidboot.serialno=", usb_serialno_setup);
