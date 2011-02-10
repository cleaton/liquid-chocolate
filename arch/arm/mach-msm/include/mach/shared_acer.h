#ifndef __SHARE_ACER_H
#define __SHARE_ACER_H

typedef enum
{
	ACER_SMEM_PROC_CMD_NORMAL_POWER_ON, // No parameter
	ACER_SMEM_PROC_CMD_I2C_TO_GPIO,     // No parameter
	ACER_SMEM_PROC_CMD_I2C_TO_HW_CTRL,  // No parameter
	ACER_SMEM_PROC_CMD_QPST_SWITCH,     // acer_qpst_switch_cmd_type
	ACER_SMEM_PROC_CMD_POWOFF,          // reset reason
	ACER_SMEM_PROC_CMD_OS_RAM_DUMP,     // No parameter
	ACER_SMEM_PROC_CMD_VIBRATION,       // acer_vib_cmd_type
	ACER_SMEM_PROC_CMD_LCDC_CLK_21487,  // No parameter
	ACER_SMSM_PROC_CMD_SD_DOWNLOAD,     // acer_sddl_cmd_type
	ACER_SMSM_PROC_CMD_READ_CLEAN_BOOT, // Return acer_master_cln_cmd_type
	ACER_SMSM_PROC_CMD_SET_CLEAN_BOOT,  // acer_master_cln_cmd_type
	ACER_SMEM_PROC_CMD_INVALID = 0xFFFFFFFF
} acer_smem_proc_cmd_type;

typedef enum
{
	ACER_QPST_SWITCH_ON,
	ACER_QPST_SWITCH_OFF,
	ACER_QPST_SWITCH_CMD_INVALID = 0xFFFFFFFF
} acer_qpst_switch_cmd_type;

typedef enum
{
	ACER_VIB_ON,
	ACER_VIB_OFF,
	ACER_VIB_INVALID = 0xFFFFFFFF
} acer_vib_cmd_type;


typedef enum
{
	ACER_SDDL_ALL,
	ACER_SDDL_AMSS_ONLY,
	ACER_SDDL_OS_ONLY,
	ACER_SDDL_INVALID = 0xFFFFFFFF
} acer_sddl_cmd_type;

typedef enum
{
	ACER_MASTER_CLN_SET,
	ACER_MASTER_CLN_CLEAN,
	ACER_MASTER_CLN_INVALID = 0xFFFFFFFF
} acer_master_cln_cmd_type;

#endif //__SHARE_ACER_H
