#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/board.h>

//#DVT-1
#define DM_CLK 29
#define DM_EN 116
#define DM_BYPASS 117
#define DM_RESET 125
#define DM_PWD 126

//#DVT-2
#define DM_CLK_V03 29
#define DM_EN_V03 116
#define DM_BYPASS_V03 150
#define DM_RESET_V03 148
#define DM_PWD_V03 101

#define GET_FM_PD_STATUS	0
#define SET_FM_PD_DEFAULT	1
#define SET_FM_PD_SW_BYPASS	2
#define SET_FM_PD_ENV1		3

struct ard_denoise_mic_gpios {
	s32 clk;
	s32 en;
	s32 bypass;
	s32 reset;
	s32 pwd;
};

extern int fm2018_set_procedure(int commad);
extern int fm2018_set_pwd(int com);
