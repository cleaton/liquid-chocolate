/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include "mt9p012.h"

/*=============================================================
    SENSOR REGISTER DEFINES
==============================================================*/
#define MT9P012_REG_MODEL_ID         0x0000
#define MT9P012_MODEL_ID             0x2801
#define REG_GROUPED_PARAMETER_HOLD   0x0104
#define GROUPED_PARAMETER_HOLD       0x0100
#define GROUPED_PARAMETER_UPDATE     0x0000
#define REG_COARSE_INT_TIME          0x3012
#define REG_VT_PIX_CLK_DIV           0x0300
#define REG_VT_SYS_CLK_DIV           0x0302
#define REG_PRE_PLL_CLK_DIV          0x0304
#define REG_PLL_MULTIPLIER           0x0306
#define REG_OP_PIX_CLK_DIV           0x0308
#define REG_OP_SYS_CLK_DIV           0x030A
#define REG_SCALE_M                  0x0404
#define REG_FRAME_LENGTH_LINES       0x300A
#define REG_LINE_LENGTH_PCK          0x300C
#define REG_X_ADDR_START             0x3004
#define REG_Y_ADDR_START             0x3002
#define REG_X_ADDR_END               0x3008
#define REG_Y_ADDR_END               0x3006
#define REG_X_OUTPUT_SIZE            0x034C
#define REG_Y_OUTPUT_SIZE            0x034E
#define REG_FINE_INTEGRATION_TIME    0x3014
#define REG_ROW_SPEED                0x3016
#define MT9P012_REG_RESET_REGISTER   0x301A
#define MT9P012_RESET_REGISTER_PWON  0x10CC
#define MT9P012_RESET_REGISTER_PWOFF 0x10C8
#define REG_READ_MODE                0x3040
#define REG_GLOBAL_GAIN              0x305E
#define REG_TEST_PATTERN_MODE        0x3070

#define MT9P012_REV_7

#if defined(CONFIG_MACH_ACER_A1)
#define USE_EEPROM
#endif

enum mt9p012_test_mode {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum mt9p012_resolution {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum mt9p012_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

enum mt9p012_setting {
	RES_PREVIEW,
	RES_CAPTURE
};

/* actuator's Slave Address */
#define MT9P012_AF_I2C_ADDR   0x18

#ifdef USE_EEPROM
/*EEPROM's Slave Address*/
#define MT9P012_EEPROM_ADDR   0xA0
#endif

/* AF Total steps parameters */
#define MT9P012_STEPS_NEAR_TO_CLOSEST_INF  40
#define MT9P012_TOTAL_STEPS_NEAR_TO_FAR    40

#if defined(CONFIG_MACH_ACER_A1)
#define MT9P012_PREVIEW_DUMMY_PIXELS 0
#define MT9P012_PREVIEW_DUMMY_LINES  0
#else
#define MT9P012_MU5M0_PREVIEW_DUMMY_PIXELS 0
#define MT9P012_MU5M0_PREVIEW_DUMMY_LINES  0
#endif

/* Time in milisecs for waiting for the sensor to reset.*/
#define MT9P012_RESET_DELAY_MSECS   66

/* for 20 fps preview */
#define MT9P012_DEFAULT_CLOCK_RATE  24000000
#define MT9P012_DEFAULT_MAX_FPS     26	/* ???? */

struct mt9p012_work {
	struct work_struct work;
};
static struct mt9p012_work *mt9p012_sensorw;
static struct i2c_client *mt9p012_client;

struct mt9p012_ctrl {
	const struct msm_camera_sensor_info *sensordata;

	int sensormode;
	uint32_t fps_divider;	/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;	/* init to 1 * 0x00000400 */

	uint16_t curr_lens_pos;
	uint16_t init_curr_lens_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;

	enum mt9p012_resolution prev_res;
	enum mt9p012_resolution pict_res;
	enum mt9p012_resolution curr_res;
	enum mt9p012_test_mode set_test;
};
static uint16_t update_type = UPDATE_PERIODIC;
static struct mt9p012_ctrl *mt9p012_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9p012_wait_queue);
DEFINE_MUTEX(mt9p012_mut);

#if defined(CONFIG_MACH_ACER_A1)
static uint8_t  mode_mask = 0x02;
static uint16_t step_position_table[MT9P012_TOTAL_STEPS_NEAR_TO_FAR+1];
static uint16_t non_linear_boundary1     = 2;
static uint16_t nl_region_code_per_step1 = 50; /* 10 bit */
static uint16_t non_linear_boundary2     = 5;
static uint16_t nl_region_code_per_step2 = 24; /* 10 bit */
static uint16_t l_region_code_per_step   = 16;
static uint8_t damping_us_per_step = 45; /* 111 DAC */
static uint8_t damping_threshold   = 5;
static uint8_t damping_mode        = 0x2;
#endif

#ifdef USE_EEPROM
static struct mt9p012_i2c_reg_conf eeprom_tbl[] = {
	{0x360A, 0},
	{0x360C, 0},
	{0x360E, 0},
	{0x3610, 0},
	{0x3612, 0},
	{0x364A, 0},
	{0x364C, 0},
	{0x364E, 0},
	{0x3650, 0},
	{0x3652, 0},
	{0x368A, 0},
	{0x368C, 0},
	{0x368E, 0},
	{0x3690, 0},
	{0x3692, 0},
	{0x36CA, 0},
	{0x36CC, 0},
	{0x36CE, 0},
	{0x36D0, 0},
	{0x36D2, 0},
	{0x370A, 0},
	{0x370C, 0},
	{0x370E, 0},
	{0x3710, 0},
	{0x3712, 0},
	{0x3600, 0},
	{0x3602, 0},
	{0x3604, 0},
	{0x3606, 0},
	{0x3608, 0},
	{0x3640, 0},
	{0x3642, 0},
	{0x3644, 0},
	{0x3646, 0},
	{0x3648, 0},
	{0x3680, 0},
	{0x3682, 0},
	{0x3684, 0},
	{0x3686, 0},
	{0x3688, 0},
	{0x36C0, 0},
	{0x36C2, 0},
	{0x36C4, 0},
	{0x36C6, 0},
	{0x36C8, 0},
	{0x3700, 0},
	{0x3702, 0},
	{0x3704, 0},
	{0x3706, 0},
	{0x3708, 0},
	{0x3614, 0},
	{0x3616, 0},
	{0x3618, 0},
	{0x361A, 0},
	{0x361C, 0},
	{0x3654, 0},
	{0x3656, 0},
	{0x3658, 0},
	{0x365A, 0},
	{0x365C, 0},
	{0x3694, 0},
	{0x3696, 0},
	{0x3698, 0},
	{0x369A, 0},
	{0x369C, 0},
	{0x36D4, 0},
	{0x36D6, 0},
	{0x36D8, 0},
	{0x36DA, 0},
	{0x36DC, 0},
	{0x3714, 0},
	{0x3716, 0},
	{0x3718, 0},
	{0x371A, 0},
	{0x371C, 0},
	{0x361E, 0},
	{0x3620, 0},
	{0x3622, 0},
	{0x3624, 0},
	{0x3626, 0},
	{0x365E, 0},
	{0x3660, 0},
	{0x3662, 0},
	{0x3664, 0},
	{0x3666, 0},
	{0x369E, 0},
	{0x36A0, 0},
	{0x36A2, 0},
	{0x36A4, 0},
	{0x36A6, 0},
	{0x36DE, 0},
	{0x36E0, 0},
	{0x36E2, 0},
	{0x36E4, 0},
	{0x36E6, 0},
	{0x371E, 0},
	{0x3720, 0},
	{0x3722, 0},
	{0x3724, 0},
	{0x3726, 0},
	{0x3782, 0},
	{0x3784, 0},
	{0x3780, 0},
};
#endif
/*=============================================================*/

static int mt9p012_i2c_rxdata(unsigned short saddr, unsigned char *rxdata,
			      int length)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = saddr,
		 .flags = 0,
		 .len = 2,
		 .buf = rxdata,
		 },
		{
		 .addr = saddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxdata,
		 },
	};

	if (i2c_transfer(mt9p012_client->adapter, msgs, 2) < 0) {
		pr_err("mt9p012_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t mt9p012_i2c_read_w(unsigned short saddr, unsigned short raddr,
				  unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

#if defined(CONFIG_MACH_ACER_A1)
	{
		int i;
		for (i=0; i<=3; i++) {
			memset(buf, 0, sizeof(buf));
			buf[0] = (raddr & 0xFF00) >> 8;
			buf[1] = (raddr & 0x00FF);

			rc = mt9p012_i2c_rxdata(saddr, buf, 2);
			if (rc >= 0)
				break;
			else {
				pr_err("mt9p012_i2c_read_w failed, retry %d time\n", i);
				pr_err("saddr = %04x, raddr = %04x\n", saddr, raddr);
			}
		}
	}

	*rdata = buf[0] << 8 | buf[1];

	return rc;
#else
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);

	rc = mt9p012_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		pr_err("mt9p012_i2c_read failed!\n");

	return rc;
#endif
}

#ifdef USE_EEPROM
static int eeprom_i2c_rxdata(unsigned short saddr, unsigned char *rxdata,
			      int length)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = saddr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxdata,
		 },
		{
		 .addr = saddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxdata,
		 },
	};

	if (i2c_transfer(mt9p012_client->adapter, msgs, 2) < 0) {
		pr_err("eeprom_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t eeprom_i2c_read_w(unsigned short saddr, unsigned short raddr,
				  unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	{
		int i;
		for (i=0; i<=3; i++) {
			memset(buf, 0, sizeof(buf));
			buf[0] = raddr;

			rc = eeprom_i2c_rxdata(saddr, buf, 2);
			if (rc >= 0)
				break;
			else {
				pr_err("eeprom_i2c_read_w failed, retry %d time\n", i);
				pr_err("saddr = %04x, raddr = %04x\n", saddr, raddr);
			}
		}
	}

	*rdata = buf[0] << 8 | buf[1];

	return rc;

	/*
	memset(buf, 0, sizeof(buf));

	buf[0] = raddr;

	rc = eeprom_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		pr_err("mt9p012_i2c_read failed!\n");

	return rc;
	*/
}
#endif

static int32_t mt9p012_i2c_txdata(unsigned short saddr, unsigned char *txdata,
				  int length)
{
	struct i2c_msg msg[] = {
		{
		 .addr = saddr,
		 .flags = 0,
		 .len = length,
		 .buf = txdata,
		 },
	};

	if (i2c_transfer(mt9p012_client->adapter, msg, 1) < 0) {
		pr_err("mt9p012_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int32_t mt9p012_i2c_write_b(unsigned short saddr, unsigned short baddr,
				   unsigned short bdata)
{
	int32_t rc = -EIO;
	unsigned char buf[2];

#if defined(CONFIG_MACH_ACER_A1)
	{
		int i;
		for (i=0; i<=3; i++) {
			memset(buf, 0, sizeof(buf));
			buf[0] = baddr;
			buf[1] = bdata;

			rc = mt9p012_i2c_txdata(saddr, buf, 2);
			if (rc >= 0)
				break;
			else {
				pr_err("mt9p012_i2c_write_b failed, retry %d time\n", i);
				pr_err("saddr = %04x, baddr = %04x, bdata = %04x\n", saddr, baddr, bdata);
			}
		}
	}

	return rc;
#else
	memset(buf, 0, sizeof(buf));
	buf[0] = baddr;
	buf[1] = bdata;
	rc = mt9p012_i2c_txdata(saddr, buf, 2);

	if (rc < 0)
		pr_err("i2c_write failed, saddr = 0x%x addr = 0x%x, val =0x%x!\n",
		     saddr, baddr, bdata);

	return rc;
#endif
}

static int32_t mt9p012_i2c_write_w(unsigned short saddr, unsigned short waddr,
				   unsigned short wdata)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

#if defined(CONFIG_MACH_ACER_A1)
	{
		int i;
		for (i=0; i<=3; i++) {
			memset(buf, 0, sizeof(buf));
			buf[0] = (waddr & 0xFF00) >> 8;
			buf[1] = (waddr & 0x00FF);
			buf[2] = (wdata & 0xFF00) >> 8;
			buf[3] = (wdata & 0x00FF);

                        rc = mt9p012_i2c_txdata(saddr, buf, 4);
                        if (rc >= 0)
                                break;
			else {
	                        pr_err("mt9p012_i2c_write_w failed, retry %d time\n", i);
				pr_err("saddr = %04x, waddr = %04x, wdata = %04x\n", saddr, waddr, wdata);
			}
                }
        }

        return rc;
#else
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);

	rc = mt9p012_i2c_txdata(saddr, buf, 4);

	if (rc < 0)
		pr_err("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
		     waddr, wdata);

	return rc;
#endif
}

static int32_t mt9p012_i2c_write_w_table(struct mt9p012_i2c_reg_conf const
					 *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num; i++) {
		rc = mt9p012_i2c_write_w(mt9p012_client->addr,
					 reg_conf_tbl->waddr,
					 reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}

	return rc;
}

static int32_t mt9p012_test(enum mt9p012_test_mode mo)
{
	int32_t rc = 0;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	if (mo == TEST_OFF)
		return 0;
	else {
		rc = mt9p012_i2c_write_w_table(mt9p012_regs.ttbl,
					       mt9p012_regs.ttbl_size);
		if (rc < 0)
			return rc;

		rc = mt9p012_i2c_write_w(mt9p012_client->addr,
					 REG_TEST_PATTERN_MODE, (uint16_t) mo);
		if (rc < 0)
			return rc;
	}

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9p012_lens_shading_enable(uint8_t is_enable)
{
	int32_t rc = 0;

	CDBG("%s: entered. enable = %d\n", __func__, is_enable);

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr, 0x3780,
				 ((uint16_t) is_enable) << 15);
	if (rc < 0)
		return rc;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_UPDATE);

	CDBG("%s: exiting. rc = %d\n", __func__, rc);
	return rc;
}

static int32_t mt9p012_set_lc(void)
{
	int32_t rc;

#ifdef USE_EEPROM
	rc = mt9p012_i2c_write_w_table(eeprom_tbl,
					ARRAY_SIZE(eeprom_tbl));
#else
	rc = mt9p012_i2c_write_w_table(mt9p012_regs.rftbl,
				       mt9p012_regs.rftbl_size);
#endif

	return rc;
}

static void mt9p012_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider;	/*Q10 */
	uint32_t pclk_mult;	/*Q10 */
	uint32_t d1;
	uint32_t d2;

	d1 =
		(uint32_t)(
		(mt9p012_regs.reg_pat[RES_PREVIEW].frame_length_lines *
		0x00000400) /
		mt9p012_regs.reg_pat[RES_CAPTURE].frame_length_lines);

	d2 =
		(uint32_t)(
		(mt9p012_regs.reg_pat[RES_PREVIEW].line_length_pck *
		0x00000400) /
		mt9p012_regs.reg_pat[RES_CAPTURE].line_length_pck);

	divider = (uint32_t) (d1 * d2) / 0x00000400;

	pclk_mult =
		(uint32_t) ((mt9p012_regs.reg_pat[RES_CAPTURE].pll_multiplier *
		0x00000400) /
		(mt9p012_regs.reg_pat[RES_PREVIEW].pll_multiplier));

	/* Verify PCLK settings and frame sizes. */
	*pfps = (uint16_t) (fps * divider * pclk_mult / 0x00000400 /
			    0x00000400);
}

static uint16_t mt9p012_get_prev_lines_pf(void)
{
	if (mt9p012_ctrl->prev_res == QTR_SIZE)
		return mt9p012_regs.reg_pat[RES_PREVIEW].frame_length_lines;
	else
		return mt9p012_regs.reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9p012_get_prev_pixels_pl(void)
{
	if (mt9p012_ctrl->prev_res == QTR_SIZE)
		return mt9p012_regs.reg_pat[RES_PREVIEW].line_length_pck;
	else
		return mt9p012_regs.reg_pat[RES_CAPTURE].line_length_pck;
}

static uint16_t mt9p012_get_pict_lines_pf(void)
{
	return mt9p012_regs.reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9p012_get_pict_pixels_pl(void)
{
	return mt9p012_regs.reg_pat[RES_CAPTURE].line_length_pck;
}

static uint32_t mt9p012_get_pict_max_exp_lc(void)
{
	uint16_t snapshot_lines_per_frame;

	if (mt9p012_ctrl->pict_res == QTR_SIZE)
		snapshot_lines_per_frame =
		    mt9p012_regs.reg_pat[RES_PREVIEW].frame_length_lines - 1;
	else
		snapshot_lines_per_frame =
		    mt9p012_regs.reg_pat[RES_CAPTURE].frame_length_lines - 1;

	return snapshot_lines_per_frame * 24;
}

static int32_t mt9p012_set_fps(struct fps_cfg *fps)
{
	/* input is new fps in Q10 format */
	int32_t rc = 0;
	enum mt9p012_setting setting;

	mt9p012_ctrl->fps_divider = fps->fps_div;
	mt9p012_ctrl->pict_fps_divider = fps->pict_fps_div;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return -EBUSY;

	if (mt9p012_ctrl->sensormode == SENSOR_PREVIEW_MODE)
		setting = RES_PREVIEW;
	else
		setting = RES_CAPTURE;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
		REG_FRAME_LENGTH_LINES,
		(mt9p012_regs.reg_pat[setting].frame_length_lines *
		fps->fps_div / 0x00000400));
	if (rc < 0)
		return rc;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_UPDATE);

	return rc;
}

static int32_t mt9p012_write_exp_gain(uint16_t gain, uint32_t line)
{
	uint16_t max_legal_gain = 0x01FF;
	uint32_t line_length_ratio = 0x00000400;
	enum mt9p012_setting setting;
	int32_t rc = 0;

	CDBG("Line:%d mt9p012_write_exp_gain \n", __LINE__);

	if (mt9p012_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		mt9p012_ctrl->my_reg_gain = gain;
		mt9p012_ctrl->my_reg_line_count = (uint16_t) line;
	}

	if (gain > max_legal_gain) {
		CDBG("Max legal gain Line:%d \n", __LINE__);
		gain = max_legal_gain;
	}

	/* Verify no overflow */
	if (mt9p012_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
		line = (uint32_t) (line * mt9p012_ctrl->fps_divider /
				   0x00000400);
		setting = RES_PREVIEW;
	} else {
		line = (uint32_t) (line * mt9p012_ctrl->pict_fps_divider /
				   0x00000400);
		setting = RES_CAPTURE;
	}

	/* Set digital gain to 1 */
#ifdef MT9P012_REV_7
	gain |= 0x1000;
#else
	gain |= 0x0200;
#endif

	if ((mt9p012_regs.reg_pat[setting].frame_length_lines - 1) < line) {
		line_length_ratio = (uint32_t) (line * 0x00000400) /
		    (mt9p012_regs.reg_pat[setting].frame_length_lines - 1);
	} else
		line_length_ratio = 0x00000400;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr, REG_GLOBAL_GAIN, gain);
	if (rc < 0) {
		pr_err("mt9p012_i2c_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_COARSE_INT_TIME, line);
	if (rc < 0) {
		pr_err("mt9p012_i2c_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}

	CDBG("mt9p012_write_exp_gain: gain = %d, line = %d\n", gain, line);

	return rc;
}

static int32_t mt9p012_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	CDBG("Line:%d mt9p012_set_pict_exp_gain \n", __LINE__);

	rc = mt9p012_write_exp_gain(gain, line);
	if (rc < 0) {
		pr_err("Line:%d mt9p012_set_pict_exp_gain failed... \n",
		     __LINE__);
		return rc;
	}

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 MT9P012_REG_RESET_REGISTER, 0x10CC | 0x0002);
	if (rc < 0) {
		pr_err("mt9p012_i2c_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}

	mdelay(5);

	/* camera_timed_wait(snapshot_wait*exposure_ratio); */
	return rc;
}

static int32_t mt9p012_setting(enum mt9p012_reg_update rupdate,
			       enum mt9p012_setting rt)
{
	int32_t rc = 0;
	switch (rupdate) {
	case UPDATE_PERIODIC:
		if (rt == RES_PREVIEW || rt == RES_CAPTURE) {
			struct mt9p012_i2c_reg_conf ppc_tbl[] = {
				{REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_HOLD},
				{REG_ROW_SPEED,
				 mt9p012_regs.reg_pat[rt].row_speed},
				{REG_X_ADDR_START,
				 mt9p012_regs.reg_pat[rt].x_addr_start},
				{REG_X_ADDR_END,
				 mt9p012_regs.reg_pat[rt].x_addr_end},
				{REG_Y_ADDR_START,
				 mt9p012_regs.reg_pat[rt].y_addr_start},
				{REG_Y_ADDR_END,
				 mt9p012_regs.reg_pat[rt].y_addr_end},
				{REG_READ_MODE,
				 mt9p012_regs.reg_pat[rt].read_mode},
				{REG_SCALE_M, mt9p012_regs.reg_pat[rt].scale_m},
				{REG_X_OUTPUT_SIZE,
				 mt9p012_regs.reg_pat[rt].x_output_size},
				{REG_Y_OUTPUT_SIZE,
				 mt9p012_regs.reg_pat[rt].y_output_size},

				{REG_LINE_LENGTH_PCK,
				 mt9p012_regs.reg_pat[rt].line_length_pck},
				{REG_FRAME_LENGTH_LINES,
				 (mt9p012_regs.reg_pat[rt].frame_length_lines *
				  mt9p012_ctrl->fps_divider / 0x00000400)},
				{REG_COARSE_INT_TIME,
				 mt9p012_regs.reg_pat[rt].coarse_int_time},
				{REG_FINE_INTEGRATION_TIME,
				 mt9p012_regs.reg_pat[rt].fine_int_time},
				{REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_UPDATE},
			};
			if (update_type == REG_INIT) {
				update_type = rupdate;
				return rc;
			}
			rc = mt9p012_i2c_write_w_table(&ppc_tbl[0],
						ARRAY_SIZE(ppc_tbl));
			if (rc < 0)
				return rc;

			rc = mt9p012_test(mt9p012_ctrl->set_test);
			if (rc < 0)
				return rc;

			rc = mt9p012_i2c_write_w(mt9p012_client->addr,
						 MT9P012_REG_RESET_REGISTER,
						 MT9P012_RESET_REGISTER_PWON |
						 0x0002);
			if (rc < 0)
				return rc;

			mdelay(5);	/* 15? wait for sensor to transition */

			return rc;
		}
		break;		/* UPDATE_PERIODIC */

	case REG_INIT:
		if (rt == RES_PREVIEW || rt == RES_CAPTURE) {
			struct mt9p012_i2c_reg_conf ipc_tbl1[] = {
				{MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWOFF},
				{REG_VT_PIX_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].vt_pix_clk_div},
				{REG_VT_SYS_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].vt_sys_clk_div},
				{REG_PRE_PLL_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].pre_pll_clk_div},
				{REG_PLL_MULTIPLIER,
				 mt9p012_regs.reg_pat[rt].pll_multiplier},
				{REG_OP_PIX_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].op_pix_clk_div},
				{REG_OP_SYS_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].op_sys_clk_div},
#ifdef MT9P012_REV_7
				{0x30B0, 0x0001},
				{0x308E, 0xE060},
				{0x3092, 0x0A52},
				{0x3094, 0x4656},
				{0x3096, 0x5652},
				{0x30CA, 0x8006},
				{0x312A, 0xDD02},
				{0x312C, 0x00E4},
				{0x3170, 0x299A},
				{0x309E, 0x5D00},
#endif
				/* optimized settings for noise */
				{0x3088, 0x6FF6},
				{0x3154, 0x0282},
				{0x3156, 0x0381},
				{0x3162, 0x04CE},
				{0x0204, 0x0010},
				{0x0206, 0x0010},
				{0x0208, 0x0010},
				{0x020A, 0x0010},
				{0x020C, 0x0010},
				{MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWON},
			};

			struct mt9p012_i2c_reg_conf ipc_tbl2[] = {
				{MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWOFF},
				{REG_VT_PIX_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].vt_pix_clk_div},
				{REG_VT_SYS_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].vt_sys_clk_div},
				{REG_PRE_PLL_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].pre_pll_clk_div},
				{REG_PLL_MULTIPLIER,
				 mt9p012_regs.reg_pat[rt].pll_multiplier},
				{REG_OP_PIX_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].op_pix_clk_div},
				{REG_OP_SYS_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].op_sys_clk_div},
#ifdef MT9P012_REV_7
				{0x30B0, 0x0001},
				{0x308E, 0xE060},
				{0x3092, 0x0A52},
				{0x3094, 0x4656},
				{0x3096, 0x5652},
				{0x30CA, 0x8006},
				{0x312A, 0xDD02},
				{0x312C, 0x00E4},
				{0x3170, 0x299A},
#endif
				/* optimized settings for noise */
				{0x3088, 0x6FF6},
				{0x3154, 0x0282},
				{0x3156, 0x0381},
				{0x3162, 0x04CE},
				{0x0204, 0x0010},
				{0x0206, 0x0010},
				{0x0208, 0x0010},
				{0x020A, 0x0010},
				{0x020C, 0x0010},
				{MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWON},
			};

			struct mt9p012_i2c_reg_conf ipc_tbl3[] = {
				{REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_HOLD},
				/* Set preview or snapshot mode */
				{REG_ROW_SPEED,
				 mt9p012_regs.reg_pat[rt].row_speed},
				{REG_X_ADDR_START,
				 mt9p012_regs.reg_pat[rt].x_addr_start},
				{REG_X_ADDR_END,
				 mt9p012_regs.reg_pat[rt].x_addr_end},
				{REG_Y_ADDR_START,
				 mt9p012_regs.reg_pat[rt].y_addr_start},
				{REG_Y_ADDR_END,
				 mt9p012_regs.reg_pat[rt].y_addr_end},
				{REG_READ_MODE,
				 mt9p012_regs.reg_pat[rt].read_mode},
				{REG_SCALE_M, mt9p012_regs.reg_pat[rt].scale_m},
				{REG_X_OUTPUT_SIZE,
				 mt9p012_regs.reg_pat[rt].x_output_size},
				{REG_Y_OUTPUT_SIZE,
				 mt9p012_regs.reg_pat[rt].y_output_size},
				{REG_LINE_LENGTH_PCK,
				 mt9p012_regs.reg_pat[rt].line_length_pck},
				{REG_FRAME_LENGTH_LINES,
				 mt9p012_regs.reg_pat[rt].frame_length_lines},
				{REG_COARSE_INT_TIME,
				 mt9p012_regs.reg_pat[rt].coarse_int_time},
				{REG_FINE_INTEGRATION_TIME,
				 mt9p012_regs.reg_pat[rt].fine_int_time},
				{REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_UPDATE},
			};

			/* reset fps_divider */
			mt9p012_ctrl->fps_divider = 1 * 0x0400;

			rc = mt9p012_i2c_write_w_table(&ipc_tbl1[0],
						       ARRAY_SIZE(ipc_tbl1));
			if (rc < 0)
				return rc;

			rc = mt9p012_i2c_write_w_table(&ipc_tbl2[0],
						       ARRAY_SIZE(ipc_tbl2));
			if (rc < 0)
				return rc;

			mdelay(5);

			rc = mt9p012_i2c_write_w_table(&ipc_tbl3[0],
						       ARRAY_SIZE(ipc_tbl3));
			if (rc < 0)
				return rc;

			/* load lens shading */
			rc = mt9p012_i2c_write_w(mt9p012_client->addr,
						 REG_GROUPED_PARAMETER_HOLD,
						 GROUPED_PARAMETER_HOLD);
			if (rc < 0)
				return rc;

			rc = mt9p012_set_lc();
			if (rc < 0)
				return rc;

			rc = mt9p012_i2c_write_w(mt9p012_client->addr,
						 REG_GROUPED_PARAMETER_HOLD,
						 GROUPED_PARAMETER_UPDATE);

			if (rc < 0)
				return rc;
		}
		update_type = rupdate;
		break;		/* case REG_INIT: */

	default:
		rc = -EINVAL;
		break;
	}			/* switch (rupdate) */

	return rc;
}

static int32_t mt9p012_video_config(int mode, int res)
{
	int32_t rc;

	switch (res) {
	case QTR_SIZE:
		rc = mt9p012_setting(UPDATE_PERIODIC, RES_PREVIEW);
		if (rc < 0)
			return rc;

		CDBG("mt9p012 sensor configuration done!\n");
		break;

	case FULL_SIZE:
		rc = mt9p012_setting(UPDATE_PERIODIC, RES_CAPTURE);
		if (rc < 0)
			return rc;

		break;

	default:
		return 0;
	}			/* switch */

	mt9p012_ctrl->prev_res = res;
	mt9p012_ctrl->curr_res = res;
	mt9p012_ctrl->sensormode = mode;

	rc = mt9p012_write_exp_gain(mt9p012_ctrl->my_reg_gain,
				    mt9p012_ctrl->my_reg_line_count);

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 MT9P012_REG_RESET_REGISTER, 0x10cc | 0x0002);

	return rc;
}

static int32_t mt9p012_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = mt9p012_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9p012_ctrl->curr_res = mt9p012_ctrl->pict_res;

	mt9p012_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9p012_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = mt9p012_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9p012_ctrl->curr_res = mt9p012_ctrl->pict_res;

	mt9p012_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9p012_power_down(void)
{
	int32_t rc = 0;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWOFF);

	mdelay(5);
	return rc;
}

#if defined(CONFIG_MACH_ACER_A1)
static void mt9p012_setup_step_position_table(void)
{
	int i;

	step_position_table[0] = 0;

	for(i=1; i <= MT9P012_TOTAL_STEPS_NEAR_TO_FAR; i++) {

		if ( i <= non_linear_boundary1) {
			step_position_table[i] = step_position_table[i-1] +
									nl_region_code_per_step1;
		} else if ( i <= non_linear_boundary2) {
			step_position_table[i] = step_position_table[i-1] +
									nl_region_code_per_step2;
		} else {
			step_position_table[i] = step_position_table[i-1] +
									l_region_code_per_step;
		}
	}
}


static int32_t mt9p012_move_focus(int direction, int32_t num_steps)
{
	int16_t step_direction;
	int16_t next_position;
	int16_t next_step_position;
	uint8_t code_val_msb, code_val_lsb;
	int16_t time_wait_per_step;

	if (num_steps > MT9P012_TOTAL_STEPS_NEAR_TO_FAR)
		num_steps = MT9P012_TOTAL_STEPS_NEAR_TO_FAR;
	else if (num_steps == 0) {
		return 0;
	}

	if (direction == MOVE_NEAR)
		step_direction = 1;
	else if (direction == MOVE_FAR)
		step_direction = -1;
	else {
		pr_err("mt9p012_move_focus failed at line %d ...\n", __LINE__);
		return -EINVAL;
	}

	next_step_position = mt9p012_ctrl->curr_lens_pos + (step_direction * num_steps);

	if (next_step_position < 0)
		next_step_position = 0;
	else if (next_step_position > MT9P012_TOTAL_STEPS_NEAR_TO_FAR)
		next_step_position = MT9P012_TOTAL_STEPS_NEAR_TO_FAR;

	next_position = step_position_table[next_step_position];
	CDBG("next_position = %d, next_step = %d", next_position, next_step_position);

	time_wait_per_step = 5000 / (step_direction * (next_position - step_position_table[mt9p012_ctrl->curr_lens_pos]));

	if (time_wait_per_step >= 400) {
		/* 400~ */
		mode_mask = 0x4;
	} else if (time_wait_per_step >= 200) {
		/* 200~400 */
		mode_mask = 0x3;
	} else if (time_wait_per_step >= 100) {
		/* 100~200 */
		mode_mask = 0x2;
	} else if (time_wait_per_step >= 50) {
		/* 50~100 */
		mode_mask = 0x1;
	} else if (step_direction == -1 &&
			next_step_position <= damping_threshold &&
			time_wait_per_step <= damping_us_per_step) {
		/* extra damping needed */
		mode_mask = damping_mode;
	} else {
		mode_mask = 0xB;
	}

	code_val_msb = next_position >> 4;
	code_val_lsb = (next_position & 0x000F) << 4;
	code_val_lsb |= mode_mask;

	/* Writing the digital code for current to the actuator */
	if (mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1,
				code_val_msb, code_val_lsb) < 0) {
		pr_err("mt9p012_move_focus failed at line %d ...\n", __LINE__);
		return -EBUSY;
	}

	/* Storing the current lens Position */
	mt9p012_ctrl->curr_lens_pos = next_step_position;

	return 0;
}
#else
static int32_t mt9p012_move_focus(int direction, int32_t num_steps)
{
	int16_t step_direction;
	int16_t actual_step;
	int16_t next_position;
	uint8_t code_val_msb, code_val_lsb;

	if (num_steps > MT9P012_TOTAL_STEPS_NEAR_TO_FAR)
		num_steps = MT9P012_TOTAL_STEPS_NEAR_TO_FAR;
	else if (num_steps == 0) {
		pr_err("mt9p012_move_focus failed at line %d ...\n", __LINE__);
		return -EINVAL;
	}

	if (direction == MOVE_NEAR)
		step_direction = 16;	/* 10bit */
	else if (direction == MOVE_FAR)
		step_direction = -16;	/* 10 bit */
	else {
		pr_err("mt9p012_move_focus failed at line %d ...\n", __LINE__);
		return -EINVAL;
	}

	if (mt9p012_ctrl->curr_lens_pos < mt9p012_ctrl->init_curr_lens_pos)
		mt9p012_ctrl->curr_lens_pos = mt9p012_ctrl->init_curr_lens_pos;

	actual_step = (int16_t) (step_direction * (int16_t) num_steps);
	next_position = (int16_t) (mt9p012_ctrl->curr_lens_pos + actual_step);

	if (next_position > 656)
		next_position = 656;
	else if (next_position < 0)
		next_position = 0;

	code_val_msb = next_position >> 4;
	code_val_lsb = (next_position & 0x000F) << 4;
	/* code_val_lsb |= mode_mask; */

	/* Writing the digital code for current to the actuator */
	if (mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1,
				code_val_msb, code_val_lsb) < 0) {
		pr_err("mt9p012_move_focus failed at line %d ...\n", __LINE__);
		return -EBUSY;
	}

	/* Storing the current lens Position */
	mt9p012_ctrl->curr_lens_pos = next_position;

	return 0;
}
#endif

static int32_t mt9p012_set_default_focus(void)
{
	int32_t rc = 0;
	uint8_t code_val_msb, code_val_lsb;

	code_val_msb = 0x00;
	code_val_lsb = 0x00;

	/* Write the digital code for current to the actuator */
	rc = mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1,
				 code_val_msb, code_val_lsb);

	mt9p012_ctrl->curr_lens_pos = 0;
	mt9p012_ctrl->init_curr_lens_pos = 0;

	return rc;
}

static int mt9p012_probe_init_done(const struct msm_camera_sensor_info *data)
{
	gpio_direction_output(data->sensor_reset, 0);
	gpio_free(data->sensor_reset);
	return 0;
}

static int mt9p012_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc;
	uint16_t chipid;

	rc = gpio_request(data->sensor_reset, "mt9p012");
	if (!rc)
		gpio_direction_output(data->sensor_reset, 1);
	else
		goto init_probe_done;

	msleep(20);

	/* RESET the sensor image part via I2C command */
	CDBG("mt9p012_sensor_init(): reseting sensor.\n");
	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 MT9P012_REG_RESET_REGISTER, 0x10CC | 0x0001);
	if (rc < 0) {
		pr_err("sensor reset failed. rc = %d\n", rc);
		goto init_probe_fail;
	}

	msleep(MT9P012_RESET_DELAY_MSECS);

	/* 3. Read sensor Model ID: */
	rc = mt9p012_i2c_read_w(mt9p012_client->addr,
				MT9P012_REG_MODEL_ID, &chipid);
	if (rc < 0)
		goto init_probe_fail;

	/* 4. Compare sensor ID to MT9T012VC ID: */
	if (chipid != MT9P012_MODEL_ID) {
		pr_err("mt9p012 wrong model_id = 0x%x\n", chipid);
		rc = -ENODEV;
		goto init_probe_fail;
	}
#if defined(CONFIG_MACH_ACER_A1)
	pr_info("[CAM]mt9p012 sensor detected,model_id = = 0x%x\n", chipid);
#endif

	rc = mt9p012_i2c_write_w(mt9p012_client->addr, 0x306E, 0x9000);
	if (rc < 0) {
		pr_err("REV_7 write failed. rc = %d\n", rc);
		goto init_probe_fail;
	}

	/* RESET_REGISTER, enable parallel interface and disable serialiser */
	CDBG("mt9p012_sensor_init(): enabling parallel interface.\n");
	rc = mt9p012_i2c_write_w(mt9p012_client->addr, 0x301A, 0x10CC);
	if (rc < 0) {
		pr_err("enable parallel interface failed. rc = %d\n", rc);
		goto init_probe_fail;
	}

	/* To disable the 2 extra lines */
	rc = mt9p012_i2c_write_w(mt9p012_client->addr, 0x3064, 0x0805);

	if (rc < 0) {
		pr_err("disable the 2 extra lines failed. rc = %d\n", rc);
		goto init_probe_fail;
	}
	goto init_probe_done;

init_probe_fail:
	mt9p012_probe_init_done(data);
init_probe_done:
	return rc;
}

#ifdef USE_EEPROM
static void mt9p012_read_eeprom(uint32_t *rg_ratio, uint32_t *bg_ratio)
{
	uint16_t awb_data;
	uint32_t gb_gr_ratio;
	int rc;

	rc = mt9p012_i2c_read_w(MT9P012_EEPROM_ADDR>>1,0XCE00,&awb_data);
	if (rc < 0) {
		*rg_ratio = 1;
	} else {
		CDBG("rg = %x\n", awb_data);
		*rg_ratio = awb_data & 0x00FF;
	}
	rc = mt9p012_i2c_read_w(MT9P012_EEPROM_ADDR>>1,0XD000,&awb_data);
	if (rc < 0) {
		*bg_ratio = 1;
		gb_gr_ratio = 1;
	} else {
		CDBG("gb and bg = %x\n", awb_data);
		gb_gr_ratio = ((awb_data & 0xFF00) >> 8);
		*bg_ratio = awb_data & 0x00FF;
	}
	pr_info("EEPROM Data r_gr %x, gb_gr %x, b_gr %x\n",
		*rg_ratio, gb_gr_ratio, *bg_ratio);
}
#endif
static int mt9p012_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	struct vreg *gp2, *gp3;
	int32_t rc;

	mt9p012_ctrl = kzalloc(sizeof(struct mt9p012_ctrl), GFP_KERNEL);
	if (!mt9p012_ctrl) {
		pr_err("mt9p012_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	mt9p012_ctrl->fps_divider = 1 * 0x00000400;
	mt9p012_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9p012_ctrl->set_test = TEST_OFF;
	mt9p012_ctrl->prev_res = QTR_SIZE;
	mt9p012_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9p012_ctrl->sensordata = data;

	/* power-up sequence */
	gp2 = vreg_get(0, "gp2");
	gp3 = vreg_get(0, "gp3");
	vreg_enable(gp2);
	vreg_enable(gp3);
	msleep(1);
	vreg_disable(gp2);
	msleep(1);
	vreg_enable(gp2);

	if (hw_version >= 3)
		gpio_direction_output(data->sensor_pwd, 1);

	/* enable mclk first */
	msm_camio_clk_rate_set(MT9P012_DEFAULT_CLOCK_RATE);
	mdelay(20);

	msm_camio_camif_pad_reg_reset();
	mdelay(20);

	rc = mt9p012_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail1;

	if (mt9p012_ctrl->prev_res == QTR_SIZE)
		rc = mt9p012_setting(REG_INIT, RES_PREVIEW);
	else
		rc = mt9p012_setting(REG_INIT, RES_CAPTURE);

	if (rc < 0) {
		pr_err("mt9p012_setting failed. rc = %d\n", rc);
		goto init_fail1;
	}

	/* sensor : output enable */
	CDBG("mt9p012_sensor_open_init(): enabling output.\n");
	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWON);
	if (rc < 0) {
		pr_err("sensor output enable failed. rc = %d\n", rc);
		goto init_fail1;
	}

	/* enable AF actuator */
	if (mt9p012_ctrl->sensordata->vcm_enable) {
		CDBG("enable AF actuator, gpio = %d\n",
			 mt9p012_ctrl->sensordata->vcm_pwd);
		rc = gpio_request(mt9p012_ctrl->sensordata->vcm_pwd,
						"mt9p012");
		if (!rc)
			gpio_direction_output(
				mt9p012_ctrl->sensordata->vcm_pwd,
				 1);
		else {
			CDBG("mt9p012_ctrl gpio request failed!\n");
			goto init_fail1;
		}
		msleep(20);

#if defined(CONFIG_MACH_ACER_A1)
        mt9p012_setup_step_position_table();
#endif

        rc = mt9p012_set_default_focus();
		if (rc < 0) {
			gpio_direction_output(mt9p012_ctrl->sensordata->vcm_pwd,
								0);
			gpio_free(mt9p012_ctrl->sensordata->vcm_pwd);
		}
	}
	if (rc >= 0)
		goto init_done;
init_fail1:
	mt9p012_probe_init_done(data);
	kfree(mt9p012_ctrl);
init_done:
	return rc;
}

static int mt9p012_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9p012_wait_queue);
	return 0;
}

static int32_t mt9p012_set_sensor_mode(int mode, int res)
{
	int32_t rc = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = mt9p012_video_config(mode, res);
		break;

	case SENSOR_SNAPSHOT_MODE:
		rc = mt9p012_snapshot_config(mode);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = mt9p012_raw_snapshot_config(mode);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

int mt9p012_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	int rc = 0;

	if (copy_from_user(&cdata,
			   (void *)argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	mutex_lock(&mt9p012_mut);

	CDBG("%s: cfgtype = %d\n", __func__, cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		mt9p012_get_pict_fps(cdata.cfg.gfps.prevfps,
				     &(cdata.cfg.gfps.pictfps));

		if (copy_to_user((void *)argp, &cdata,
				 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf = mt9p012_get_prev_lines_pf();

		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl = mt9p012_get_prev_pixels_pl();

		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf = mt9p012_get_pict_lines_pf();

		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl = mt9p012_get_pict_pixels_pl();

		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc = mt9p012_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = mt9p012_set_fps(&(cdata.cfg.fps));
		break;

	case CFG_SET_EXP_GAIN:
		rc = mt9p012_write_exp_gain(cdata.cfg.exp_gain.gain,
					    cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_PICT_EXP_GAIN:
		CDBG("Line:%d CFG_SET_PICT_EXP_GAIN \n", __LINE__);
		rc = mt9p012_set_pict_exp_gain(cdata.cfg.exp_gain.gain,
					       cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_MODE:
		rc = mt9p012_set_sensor_mode(cdata.mode, cdata.rs);
		break;

	case CFG_PWR_DOWN:
		rc = mt9p012_power_down();
		break;

	case CFG_MOVE_FOCUS:
		CDBG("mt9p012_ioctl: CFG_MOVE_FOCUS: cdata.cfg.focus.dir=%d \
				cdata.cfg.focus.steps=%d\n",
				cdata.cfg.focus.dir, cdata.cfg.focus.steps);
		rc = mt9p012_move_focus(cdata.cfg.focus.dir,
					cdata.cfg.focus.steps);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc = mt9p012_set_default_focus();
		break;

	case CFG_SET_LENS_SHADING:
		CDBG("%s: CFG_SET_LENS_SHADING\n", __func__);
		rc = mt9p012_lens_shading_enable(cdata.cfg.lens_shading);
		break;

	case CFG_GET_AF_MAX_STEPS:
		cdata.max_steps = MT9P012_STEPS_NEAR_TO_CLOSEST_INF;
		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_EFFECT:
		break;
#ifdef USE_EEPROM
	case CFG_GET_WB_GAINS:
		CDBG("%s: CFG_GET_WB_GAINS\n", __func__);
		cdata.cfg.wb_gains.g_gain = 0x1;
		cdata.cfg.wb_gains.r_gain = 0x1;
		cdata.cfg.wb_gains.b_gain = 0x1;
		mt9p012_read_eeprom(&cdata.cfg.wb_gains.r_gain,
				    &cdata.cfg.wb_gains.b_gain);
		if (copy_to_user((void *)argp, &cdata,
				 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
#endif
	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&mt9p012_mut);
	return rc;
}

int mt9p012_sensor_release(void)
{
	struct vreg *gp2, *gp3;
	int rc = -EBADF;

	mutex_lock(&mt9p012_mut);

	mt9p012_power_down();

	gpio_direction_output(mt9p012_ctrl->sensordata->sensor_reset, 0);
	gpio_free(mt9p012_ctrl->sensordata->sensor_reset);

	gpio_direction_output(mt9p012_ctrl->sensordata->vcm_pwd, 0);
	gpio_free(mt9p012_ctrl->sensordata->vcm_pwd);

	// turn off the power rails of the sensor module
	if (hw_version >= 3)
		gpio_direction_output(mt9p012_ctrl->sensordata->sensor_pwd, 0);

	gp2 = vreg_get(0, "gp2");
	gp3 = vreg_get(0, "gp3");
	vreg_disable(gp3);
	vreg_disable(gp2);

	kfree(mt9p012_ctrl);
	mt9p012_ctrl = NULL;

	CDBG("mt9p012_release completed\n");

	mutex_unlock(&mt9p012_mut);
	return rc;
}

static int mt9p012_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("mt9p012_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	mt9p012_sensorw = kzalloc(sizeof(struct mt9p012_work), GFP_KERNEL);
	if (!mt9p012_sensorw) {
		pr_err("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9p012_sensorw);
	mt9p012_init_client(client);
	mt9p012_client = client;

	mdelay(50);

	CDBG("mt9p012_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	pr_err("mt9p012_probe failed! rc = %d\n", rc);
	return rc;
}

static const struct i2c_device_id mt9p012_i2c_id[] = {
	{"mt9p012", 0},
	{}
};

static struct i2c_driver mt9p012_i2c_driver = {
	.id_table = mt9p012_i2c_id,
	.probe = mt9p012_i2c_probe,
	.remove = __exit_p(mt9p012_i2c_remove),
	.driver = {
		   .name = "mt9p012",
		   },
};

static int mt9p012_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	struct vreg *gp2, *gp3;
#ifdef USE_EEPROM
	uint16_t eeprom_data = 0;
	int i,j = 0;
	uint16_t read_addr;
#endif

	int rc = i2c_add_driver(&mt9p012_i2c_driver);
	if (rc < 0 || mt9p012_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

	/* power-up sequence */
	gp2 = vreg_get(0, "gp2");
	gp3 = vreg_get(0, "gp3");
	vreg_enable(gp2);
	vreg_enable(gp3);
	msleep(1);
	vreg_disable(gp2);
	msleep(1);
	vreg_enable(gp2);

	if (hw_version >= 3)
		gpio_direction_output(info->sensor_pwd, 1);

	msm_camio_clk_rate_set(MT9P012_DEFAULT_CLOCK_RATE);
	mdelay(20);

#ifdef USE_EEPROM
	for (i = 0,j = 0; i <= 204; i=i+2,j++) {
		read_addr = i;
		if (eeprom_i2c_read_w(MT9P012_EEPROM_ADDR >> 1,
			read_addr, &eeprom_data) < 0) {
		pr_err("EEPROM read failed at line %d ...\n", __LINE__);
		}
		CDBG("[CAM]eeprom_data  %d = 0x%x\n",j,eeprom_data);
		eeprom_tbl[j].wdata = eeprom_data;
	}
#endif
	rc = mt9p012_probe_init_sensor(info);
	if (rc < 0)
		goto probe_done;

	s->s_init = mt9p012_sensor_open_init;
	s->s_release = mt9p012_sensor_release;
	s->s_config = mt9p012_sensor_config;
	mt9p012_probe_init_done(info);

	// turn off the power rails of the sensor module
	if (hw_version >= 3)
		gpio_direction_output(info->sensor_pwd, 0);

	vreg_disable(gp3);
	vreg_disable(gp2);

probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __mt9p012_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, mt9p012_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9p012_probe,
	.driver = {
		   .name = "msm_camera_mt9p012",
		   .owner = THIS_MODULE,
		   },
};

static int __init mt9p012_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9p012_init);
