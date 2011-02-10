# bcm4325
DHDCFLAGS =  -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2 -DBCMWAPI_WPI    \
             -DUNRELEASEDCHIP -DCONFIG_WIRELESS_EXT -DDHD_GPL -DDHD_SCHED -DDHD_DEBUG   \
             -DSDTEST -DBDC -DTOE -DDHD_BCMEVENTS -DSHOW_EVENTS -DSDIO_ISR_THREAD       \
             -DBCMSDIO -DDHD_GPL -DBCMLXSDMMC -DBCMPLATFORM_BUS -DSOFTAP -DCUSTOMER_HW  \
             -Wall -Wstrict-prototypes -Werror                                          \
             -Idrivers/net/wireless/bcm4325 -Idrivers/net/wireless/bcm4325/include

DHDOFILES = dhd_linux.o linux_osl.o bcmutils.o dhd_common.o dhd_custom_gpio.o   \
            wl_iw.o siutils.o sbutils.o aiutils.o hndpmu.o bcmwifi.o dhd_sdio.o \
            dhd_linux_sched.o dhd_cdc.o bcmsdh_sdmmc.o bcmsdh.o bcmsdh_linux.o  \
            bcmsdh_sdmmc_linux.o

obj-$(CONFIG_BCM4325) += dhd.o
dhd-objs += $(DHDOFILES)
EXTRA_CFLAGS = $(DHDCFLAGS)
EXTRA_LDFLAGS += --strip-debug
