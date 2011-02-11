EXTRA_CFLAGS = -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2 -DBCMWAPI_WPI -DUNRELEASEDCHIP -DCONFIG_WIRELESS_EXT -DBDC -DTOE -DDHD_BCMEVENTS -DSHOW_EVENTS -DBCMSDIO -DDHD_GPL -DBCMLXSDMMC -DBCMPLATFORM_BUS -DSDIO_ISR_THREAD -DCUSTOMER_HW -DIOCTL_RESP_TIMEOUT=1000 -Wall -Wstrict-prototypes -DDHD_DEBUG -DSDTEST -DSRCBASE="\"$(SRCROOT)\"" 

#ifeq ($(CONFIG_MMC_MSM),y)
#CONFIG_MMC_MSM7X00A = y
#EXTRA_CFLAGS += -DCONFIG_MMC_MSM7X00A=1
#endif

#ifeq ($(CONFIG_MMC_MSM7X00A),y)
#EXTRA_CFLAGS += -Dlinux
#EXTRA_CFLAGS += -DDHD_SDALIGN=64 -DMAX_HDR_READ=64 -DDHD_FIRSTREAD=64
#endif

#################################################################### OPTIONAL FEATURES ###################################################################################
#TO ENALBLE OPTIONAL FEATURES UNCOMMENT THE CORRESPONDING FLAGS

# SOFTAP
EXTRA_CFLAGS += -DSOFTAP
EXTRA_LDFLAGS += --strip-debug

###########################################################################################################################################################################

EXTRA_CFLAGS += -I$(src)/src/include/
EXTRA_CFLAGS += -I$(src)/src/dhd/sys/
EXTRA_CFLAGS += -I$(src)/src/dongle/
EXTRA_CFLAGS += -I$(src)/src/bcmsdio/sys/
EXTRA_CFLAGS += -I$(src)/src/wl/sys/
EXTRA_CFLAGS += -I$(src)/src/shared/

KBUILD_CFLAGS += -I$(LINUXDIR)/include -I$(shell pwd)


#obj-$(CONFIG_BROADCOM_WIFI)-m	+= bcm4325.o
obj-m	+= bcm4329.o

bcm4329-y := src/dhd/sys/dhd_linux.o \
		src/shared/bcmutils.o \
		src/dhd/sys/dhd_common.o \
		src/shared/siutils.o \
		src/shared/sbutils.o \
		src/shared/aiutils.o \
		src/shared/hndpmu.o \
		src/wl/sys/wl_iw.o \
		src/shared/bcmwifi.o \
		src/dhd/sys/dhd_cdc.o \
		src/dhd/sys/dhd_linux_sched.o\
		src/dhd/sys/dhd_sdio.o \
		src/dhd/sys/dhd_custom_gpio.o \
		src/bcmsdio/sys/bcmsdh_sdmmc.o \
		src/bcmsdio/sys/bcmsdh.o \
		src/bcmsdio/sys/bcmsdh_linux.o \
		src/bcmsdio/sys/bcmsdh_sdmmc_linux.o \
		src/shared/linux_osl.o
all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules

clean: 
	rm -rf *.o *.ko *.mod.c *~ .*.cmd Module.symvers modules.order .tmp_versions	\
	 	src/dhd/sys/dhd_linux.o \
		src/shared/bcmutils.o \
		src/dhd/sys/dhd_common.o \
		src/shared/siutils.o \
		src/shared/sbutils.o \
		src/shared/aiutils.o \
		src/shared/hndpmu.o \
		src/wl/sys/wl_iw.o \
		src/shared/bcmwifi.o \
		src/dhd/sys/dhd_cdc.o \
		src/dhd/sys/dhd_linux_sched.o\
		src/dhd/sys/dhd_sdio.o \
		src/dhd/sys/dhd_custom_gpio.o \
		src/bcmsdio/sys/bcmsdh_sdmmc.o \
		src/bcmsdio/sys/bcmsdh.o \
		src/bcmsdio/sys/bcmsdh_linux.o \
		src/bcmsdio/sys/bcmsdh_sdmmc_linux.o \
		src/bcmsdio/sys/wlgpio.o \
		src/shared/linux_osl.o

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install
