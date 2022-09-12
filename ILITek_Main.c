/*
 * Copyright (c) 2019 ILI Technology Corp.
 *
 * This file is part of ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 */
#ifndef _ILITEK_MAIN_C_
#define _ILITEK_MAIN_C_

/* Includes of headers ------------------------------------------------------*/
#include "ILITek_Protocol.h"
#include "ILITek_CMDDefine.h"
#include "ILITek_Device.h"
#include "ILITek_Main.h"
#include "API/ILITek_Upgrade.h"
#include "API/ILITek_MpResult.h"
#include <string.h>
#include <time.h>
#include <arpa/inet.h>

/* Private define ------------------------------------------------------------*/
#define FUNC_CMD_LEN 	20
#define FUNC_STR_LEN 	100
#define FUNC_NUM 	30 // size of Function
#define INTERFACE_NUM	2
/* Private macro ------------------------------------------------------------*/
#define LF	0x0A
#define	CR	0x0D
#define CHK_DELAY(x) ((*(x)=='D') && (*(x+1)=='e') && (*(x+2)=='l') && (*(x+3)=='a') && (*(x+4)=='y'))
#define CHK_I2C(x) ((*(x)=='I') && (*(x+1)=='2') && (*(x+2)=='C'))
#define CHK_USB(x) ((*(x)=='U') && (*(x+1)=='S') && (*(x+2)=='B'))
#define MAX_SCRIPT_CMD_SIZE	512


#define CHROME_DESC		"Show Info. for ChromeOS"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define CHROME_USB	{ "USB", "V3/V6", "null", "null", "", "", "" }
#define	CHROME_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "", "", "" }

#define FWID_DESC		"Get FWID from EDID table firstly and FW query secondly"
#define FWID_USB	{ "USB", "V3/V6", "null", "null", "", "", "" }
#define	FWID_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "", "", "" }

#define PANEL_INFOR_DESC	"Show the sensor panel information"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	PANEL_INFOR_USB	{ "USB", "V3/V6", "null", "null", "", "", "" }
#define	PANEL_INFOR_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "", "", "" }

#define FW_UPDATE_DESC		"Run FW update"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	FW_UPDATE_USB	{ "USB", "V3/V6", "null", "null", "Hex Path", "[Version]", "" }
#define	FW_UPDATE_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "Hex Path", "[Version]", "" }

/* Private function prototypes -----------------------------------------------*/
int Func_Chrome(int argc, char *argv[]);
int Func_FWID(int argc, char *argv[]);
int Func_PanelInfo(int argc, char *argv[]);
int Func_FWUpgrade(int argc, char *argv[]);

/* Private typedef -----------------------------------------------------------*/

typedef int Function_t(int argc, char *argv[]);

typedef struct
{
	char Interface[FUNC_CMD_LEN];
	char Protocol[FUNC_CMD_LEN];
	char Device[FUNC_CMD_LEN];
	char Addr[FUNC_CMD_LEN];
	char Ctrl_para1[FUNC_CMD_LEN];
	char Ctrl_para2[FUNC_CMD_LEN];
	char Ctrl_para3[FUNC_CMD_LEN];
} S_PARA;

typedef struct
{
	char FuncStrs[FUNC_CMD_LEN];
	Function_t *pFuncPoint;
	char FuncDesc[FUNC_STR_LEN];
	S_PARA Para[INTERFACE_NUM];
} S_FUNC_MAP;

S_FUNC_MAP au8FuncStrs[] =
{
	{"Chrome",		Func_Chrome,		CHROME_DESC,		{CHROME_USB, CHROME_I2C},},
	{"FWID",		Func_FWID,		FWID_DESC,		{FWID_USB, FWID_I2C},},
	{"PanelInfor",		Func_PanelInfo,		PANEL_INFOR_DESC,	{PANEL_INFOR_USB, PANEL_INFOR_I2C},},
	{"FWUpgrade",		Func_FWUpgrade,		FW_UPDATE_DESC,		{FW_UPDATE_USB, FW_UPDATE_I2C},},
};

struct cmd_option cmd_opt;

/* Private functions ---------------------------------------------------------*/
#include <stdarg.h>
static int scan_args(const char *src, const char *tag, const char *_fmt, ...)
{
	char fmt[2048];
	va_list args;
	int src_len = strlen(src);
	int tag_len = strlen(tag);

	memset(fmt, 0, sizeof(fmt));

	if (strncmp(src, tag, tag_len))
		return -EFAULT;

	if (!strlen(_fmt))
		return (src_len == tag_len) ? 0 : -EFAULT;

	sprintf(fmt, "%s%s", tag, _fmt);

	va_start(args, _fmt);
	vsscanf(src, fmt, args);
	va_end(args);

	return 0;
}

int Func_Chrome(int argc, char *argv[])
{
	int error;

	UNUSED(argc);
	UNUSED(argv);

	if ((error = viSetTestMode(true, 0)) < 0 ||
	    (error = GetFWVersion()) < 0 ||
	    (error = viSetTestMode(false, 0)) < 0)
		return error;

	/*
	 * Chromebook script used, please don't modify string format.
	 */
	LD_MSG("fw-version-tag: [%02X%02X.%02X%02X.%02X%02X.%02X%02X]\n",
		ptl.fw_ver[0], ptl.fw_ver[1], ptl.fw_ver[2], ptl.fw_ver[3],
		ptl.fw_ver[4], ptl.fw_ver[5], ptl.fw_ver[6], ptl.fw_ver[7]);

	return 0;
}

static void check_fwid_args(int argc, char *argv[])
{
	int i;

	memset(cmd_opt.fwid_map_file, 0, sizeof(cmd_opt.fwid_map_file));

	for (i = 0; i < argc; i++) {
		if (!scan_args(argv[i], "--fwid-map-file=", "%s",
			       cmd_opt.fwid_map_file))
			continue;

		if (strcmp(argv[i], "-h") && strcmp(argv[i], "--help"))
			continue;

		LD_MSG("FWID command options:\n\n"
		       "--fwid-map-file=<filepath>	FWID lookup table\n\n\n");
		exit(0);
	}

}

int get_fwid_by_edid(char *fwid_map_file, uint16_t *fwid)
{
	int error;
	FILE *fp;
	char line[1024], *line_ptr;
	struct edid_block edid;
	char device_edid[64];
	bool skip_first_line = false;

	char *file_edid = NULL;
	char *file_customer_id = NULL;
	char *file_fwid = NULL;
	char *file_sensor_id = NULL;
	char *file_hex_name = NULL;

	memset(&edid, 0, sizeof(struct edid_block));
	if ((error = get_edid(&edid, device_edid, sizeof(device_edid))) < 0)
		return error;

	LD_DBG("find %s in fwid map file: %s\n", device_edid, fwid_map_file);

	if (!(fp = fopen(fwid_map_file, "r"))) {
		LD_ERR("Invalid fwid map file: %s\n", fwid_map_file);
		return -EINVAL;
	}

	while (!feof(fp)) {
		if (!fgets(line, sizeof(line), fp))
			continue;
		line_ptr = line;

		/* skip the first line */
		if (!skip_first_line) {
			skip_first_line = true;
			continue;
		}

		/* fetch edid string */
		file_edid = strsep(&line_ptr, ",");
		if (!file_edid)
			continue;

		/* lookup table by edid */
		if (strcmp(file_edid, device_edid))
			continue;

		LD_DBG("device_edid: %s matched\n", device_edid);

		file_customer_id = strsep(&line_ptr, ",");
		if (!file_customer_id)
			break;

		file_fwid = strsep(&line_ptr, ",");
		if (!file_fwid)
			break;

		file_sensor_id = strsep(&line_ptr, ",");
		if (!file_sensor_id)
			break;

		file_hex_name = strsep(&line_ptr, ",");
		if (!file_hex_name)
			break;

		break;
	}
	fclose(fp);

	if (!file_fwid || !strlen(file_fwid) ||!strcmp(file_fwid, "\n")) {
		LD_ERR("No matched edid string: %s found\n", device_edid);
		return -EFAULT;
	}

	LD_DBG("fwid: %s was found\n", file_fwid);

	sscanf(file_fwid, "%hx", fwid);

	return 0;
}

int Func_FWID(int argc, char *argv[])
{
	int error;
	uint16_t fwid;

	check_fwid_args(argc, argv);

	/*
	 * Get FWID from EDID lookup table first.
	 * If no matched edid string in the table, get FWID from FW secondly.
	 * get FWID from FW should be accessible for both AP and BL mode.
	 */
	do {
		error = get_fwid_by_edid(cmd_opt.fwid_map_file, &fwid);
		if (!error)
			break;

		if ((error = GetKernelVer()) < 0)
			return error;

		/* Lego series except 29xx series */
		if (ptl.flash_data_start != 0x2C000) {
			fwid = ptl.ic;
			break;
		}

		/* 29XX series only */
		if ((error = get_fwid_by_mpresult(&fwid)) < 0)
			return error;
	} while (false);

	/*
	 * Chromebook script used, please don't modify string format.
	 */
	LD_MSG("fwid-tag: [%04x]\n", fwid);

	return 0;
}

int Func_PanelInfo(int argc, char *argv[])
{
	int error;
	int i;

	UNUSED(argv);

	if ((inConnectStyle == _ConnectStyle_I2C_ && argc >= 6) ||
	    (inConnectStyle == _ConnectStyle_USB_ && argc >= 4) ||
	    (inConnectStyle == _ConnectStyle_I2CHID_ && argc >= 4))
		error = viGetPanelInfor();
	else
		return -EINVAL;

	if (error < 0)
		return error;

	/*
	 * Chromebook script used, please don't modify string format.
	 */
	LD_MSG("fw-version-tag: [%02X%02X.%02X%02X.%02X%02X.%02X%02X]\n",
		ptl.fw_ver[0], ptl.fw_ver[1], ptl.fw_ver[2], ptl.fw_ver[3],
		ptl.fw_ver[4], ptl.fw_ver[5], ptl.fw_ver[6], ptl.fw_ver[7]);
	LD_MSG("protocol-version-tag: [%02X.%02X]\n", ptl.ptl_ver[0], ptl.ptl_ver[1]);
	LD_MSG("fw-mode-tag: [%s]\n", ptl.ic_mode_str[0]);
	LD_MSG("ic-type-tag: [%04X]\n", ptl.ic);
	LD_MSG("module-name-tag: [%s]\n", ptl.module_name);

	LD_MSG("[Protocol Version]: %x.%x.%x\n",
		ptl.ptl_ver[0], ptl.ptl_ver[1], ptl.ptl_ver[2]);
	LD_MSG("[Kernel Version] %.2X.%.2X.%.2X.%.2X.%.2X, IC: %#04X\n",
		ptl.ic_ver[0], ptl.ic_ver[1], ptl.ic_ver[2],
		ptl.ic_ver[3], ptl.ic_ver[4], ptl.ic);
	LD_MSG("[Firmware Version] 0x%X.0x%X.0x%X.0x%X.0x%X.0x%X.0x%X.0x%X\n",
		ptl.fw_ver[0], ptl.fw_ver[1], ptl.fw_ver[2], ptl.fw_ver[3],
		ptl.fw_ver[4], ptl.fw_ver[5], ptl.fw_ver[6], ptl.fw_ver[7]);

	if (ptl.ic_mode[0] == 0x55) {
		LD_MSG("[Current Mode] Master: %#X\n", ptl.ic_mode[0]);
		return error;
	}

	LD_MSG("[CoreVersion] %#02X.%#02X.%#02X.%#02X\n",
		ptl.core_ver[0], ptl.core_ver[1],
		ptl.core_ver[2], ptl.core_ver[3]);
	LD_MSG("[FW Mode] %#02X\n", ptl.fw_mode);


	LD_MSG("[Panel Information] X/Y resolution: %u/%u\n",
		ptl.x_max, ptl.y_max);
	LD_MSG("[Panel Information] X/Y channel: %u/%u\n",
		ptl.x_ch, ptl.y_ch);
	LD_MSG("[Panel Information] Support %d keys\n", ptl.key_num);
	if (ptl.key_num > 0)
		LD_MSG("[Panel Information] key mode: %u\n", ptl.key_mode);

	if (inProtocolStyle == _Protocol_V3_) {
		LD_MSG("[Current Mode] Master: %#X\n", ptl.ic_mode[0]);
		LD_MSG("[Check Code] AP: 0x%X, Data: 0x%X\n",
			ptl.ap_crc_v3, ptl.df_crc_v3);
	} else {
		if (ptl.ver > PROTOCOL_V6_0_6)
			LD_MSG("[Panel Information] Pen Mode: %s\n", ptl.pen_str);
		if (ptl.ver > PROTOCOL_V6_0_2)
			LD_MSG("[Panel Information] Block Number: %d\n", ptl.block_num);
		LD_MSG("[Panel Information] Chip count: %u\n", ptl.ic_num);
		LD_MSG("[Panel Information] Support %d modes\n", ptl.mode_num);

		LD_MSG("[Current Mode] Master: %#X, %s\n",
			ptl.ic_mode[0], ptl.ic_mode_str[0]);
		for (i = 1; i < ptl.ic_num; i++) {
			LD_MSG("[Current Mode] Slave[%d]: %#X, %s\n",
				i, ptl.ic_mode[i], ptl.ic_mode_str[i]);
		}
		LD_MSG("[FW CRC] Master: %#X\n", ptl.ap_crc_v6[0]);
		for (i = 1; i < ptl.ic_num; i++) {
			LD_MSG("[FW CRC] Slave[%d]: %#X\n",
				i, ptl.ap_crc_v6[i]);
		}

		LD_MSG("[FW CRC] Flash Start Addr: %#X\n", ptl.flash_data_start);
	}

	return 0;
}

static void check_upgrade_args(int argc, char *argv[])
{
	int i;

	upg.force_update = false;
	upg.fw_check_only = false;
	upg.args_fw_ver_check = false;

	for (i = 0; i < argc; i++) {
		if (!scan_args(argv[i], "--force-upgrade", "")) {
			upg.force_update = true;
		} else if (!scan_args(argv[i], "--check-only", "")) {
			upg.fw_check_only = true;
		} else if (!strncmp(argv[i], "--fw-ver=", 9)) {
			upg.args_fw_ver_check = true;
			upg.args_len = sscanf(argv[i], "--fw-ver=%hhx.%hhx.%hhx.%hhx.%hhx.%hhx.%hhx.%hhx",
				upg.args_fw_ver, upg.args_fw_ver + 1,
				upg.args_fw_ver + 2, upg.args_fw_ver + 3,
				upg.args_fw_ver + 4, upg.args_fw_ver + 5,
				upg.args_fw_ver + 6, upg.args_fw_ver + 7);
		}

		if (strcmp(argv[i], "-h") && strcmp(argv[i], "--help"))
			continue;

		LD_MSG("FWUpgrade command options:\n\n"
		       "--force-upgrade	Enable force upgrade option\n"
		       "--check-only	Enable FW check only option\n\n"
		       "To enable FW version check before upgrade\n"
		       "--fw-ver=%%hhx.%%hhx.%%hhx.%%hhx.%%hhx.%%hhx.%%hhx.%%hhx\n\n\n");
		exit(0);
	}

	LD_MSG("force upgrade: %d, check only: %d\n",
		upg.force_update, upg.fw_check_only);
	LD_MSG("fw check: %d, args len: %d, fw: %hhx.%hhx.%hhx.%hhx.%hhx.%hhx.%hhx.%hhx\n",
		upg.args_fw_ver_check, upg.args_len, upg.args_fw_ver[0],
		upg.args_fw_ver[1], upg.args_fw_ver[2], upg.args_fw_ver[3],
		upg.args_fw_ver[4], upg.args_fw_ver[5], upg.args_fw_ver[6],
		upg.args_fw_ver[7]);
}

int Func_FWUpgrade(int argc, char *argv[])
{
	char filename[UPGRAD_FILE_PATH_SIZE];

	if (argc < 7)
		return -EINVAL;

	memset(filename, 0, UPGRAD_FILE_PATH_SIZE);
	strcpy(filename, argv[6]);
	LD_MSG("Hex filename:%s\n", filename);

	return viRunFiremwareUpgrade(filename);
}

static void show_command_help(int argc, char *argv[])
{
	int i, j;

	if (argc > 2 && !strcmp(argv[2], "-help")) {
		for (i = 0; i < FUNC_NUM; i++) {
			if (strcmp(argv[1], au8FuncStrs[i].FuncStrs))
				continue;
			LD_MSG("%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|\n",
				"----------------", "----------------", "----------------", "----------------",
				"----------------", "----------------", "----------------", "----------------");
			LD_MSG("%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|\n",
				"Function", "Interface", "Protocol", "Device", "I2C address",
				"Ctrl param1", "Ctrl param2", "Ctrl param3");
			LD_MSG("%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|\n",
				"----------------", "----------------", "----------------", "----------------",
				"----------------", "----------------", "----------------", "----------------");
			for (j = 0; j < INTERFACE_NUM; j++) {
				LD_MSG("%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|\n",
					au8FuncStrs[i].FuncStrs,
					au8FuncStrs[i].Para[j].Interface,
					au8FuncStrs[i].Para[j].Protocol,
					au8FuncStrs[i].Para[j].Device,
					au8FuncStrs[i].Para[j].Addr,
					au8FuncStrs[i].Para[j].Ctrl_para1,
					au8FuncStrs[i].Para[j].Ctrl_para2,
					au8FuncStrs[i].Para[j].Ctrl_para3);
				LD_MSG("%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|%-16s|\n",
					"----------------", "----------------", "----------------", "----------------",
					"----------------", "----------------", "----------------", "----------------");
			}
			exit(0);
		}
	}
}

int viGetPanelInfor_V3()
{
	int error;

	if ((error = GetProtocol()) < 0 || (error = GetKernelVer()) < 0 ||
	    (error = GetFWVersion()) < 0 || (error = GetICMode(1)) < 0)
		return error;

	/* return success if it's BL mode */
	if (ptl.ic_mode[0] == 0x55) {
		LD_ERR("FW in BL mode, wait for FWUpgrade...\n");
		return 0;
	}

	if ((error = GetCoreVersion()) < 0 || (error = GetFWMode()) < 0 ||
	    (error = PanelInfor_V3()) < 0 || (error = GetCRC_V3()) < 0)
	    	return error;

	return 0;
}

int viGetPanelInfor_V6()
{
	int error;

	if ((error = GetProtocol()) < 0 || (error = GetKernelVer()) < 0 ||
	    (error = GetFWVersion()) < 0 || (error = GetICMode(1)) < 0)
		return error;

	/* return success if it's BL mode */
	if (ptl.ic_mode[0] == 0x55) {
		LD_ERR("FW in BL mode, wait for FWUpgrade...\n");
		return 0;
	}

	if ((error = GetCoreVersion()) < 0 ||
	    (error = GetFWMode()) < 0 ||
	    (error = PanelInfor_V6()) < 0 ||
	    (error = GetCRC_V6(ptl.ic_num)) < 0)
		return error;

	if (ptl.ic_num > 1 && (error = GetICMode(ptl.ic_num)) < 0)
		return error;

	return 0;
}

int viSetTestMode(bool setTest, int delay_ms)
{
	if (inProtocolStyle == _Protocol_V3_) {
		if (setTest)
			return EnterTestMode(delay_ms);
		else
			return ExitTestMode(delay_ms);
	} else if (inProtocolStyle == _Protocol_V6_) {
		if (setTest)
			return ModeCtrl_V6(ENTER_SUSPEND_MODE,
					  ENABLE_ENGINEER, delay_ms);
		else
			return ModeCtrl_V6(ENTER_NORMAL_MODE,
					  DISABLE_ENGINEER, delay_ms);
	}

	return -EINVAL;
}

int viGetPanelInfor()
{
	if (inProtocolStyle == _Protocol_V3_)
		return viGetPanelInfor_V3();

	return viGetPanelInfor_V6();
}

int ChangeToBootloader(unsigned int start, unsigned int end)
{
	int ret;
	int count;

	for (count = 0; count < 5; count++) {
		ret = GetICMode(1);
		if (ptl.ic_mode[0] != OP_MODE_BOOTLOADER) {
			ret = SetProgramKey(start, end);
			usleep(20000);

			ret = ChangeTOBL();

			/*
			 * Lego's old BL may trigger unexpected INT after 0xC2,
			 * so make I2C driver handle it ASAP
			 */
			if (cmd_opt.support_INT_ack &&
			    inProtocolStyle == _Protocol_V6_)
				switch_irq(1);

			if (inConnectStyle == _ConnectStyle_I2CHID_)
				usleep(1000000 + count * 100000);
			else
				usleep(210000 + count * 20000);

			if (inConnectStyle == _ConnectStyle_USB_)
				break;
		} else {
			LD_MSG("%s, current op mode is bootloader mode, can update firmware(%u/5)\n", __func__, count);
			break;
		}
	}

	if (inConnectStyle == _ConnectStyle_USB_) {
		CloseDevice();
		usleep(500000);
		if (inProtocolStyle == _Protocol_V6_)
			sleep(1);
		for (count = 1; count < 13; count++) {
			if (InitDevice() < 0) {
				LD_MSG("%s, currently in AP mode, wait 5 sec, after change to bootloader mode, %u\n",
					__func__, count);
				usleep(5000000);
				if (count == 12)
					return -EFAULT;
				continue;
			}

			if (is_usb_hid_old_bl == 1)
				break;

			ret = GetICMode(1);
			if (ptl.ic_mode[0] != OP_MODE_BOOTLOADER) {
				LD_MSG("%s, currently in AP mode, wait 5 sec, after change to bootloader mode, 0x%X, %u\n",
						__func__, ptl.ic_mode[0], count);
				usleep(5000000);
				if (count == 12)
					return -EFAULT;
				continue;
			}

			LD_MSG("%s, check again, currently bootloader mode, can update firmware, 0x%X, ret=%u\n",
					__func__, ptl.ic_mode[0], ret);
			break;
		}
	}

	GetProtocol_in_BL();

	if (inProtocolStyle == _Protocol_V6_)
		GetFWVersion_BL();

	return 0;
}

//M3 + M2V
int ChangeToBootloader_M3_M2V()
{
	int ret = 0;
	int count;

	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	//read current op mode
	for (count = 0; count < 5; count++) {
		Wbuff[0] = 0xcb;
		Wbuff[1] = 0x80;
		Wbuff[2] = (uint8_t)ILITEK_TP_CMD_READ_OP_MODE;
		ret = TransferData(Wbuff, 3, Rbuff, 1, 1000);
		LD_MSG("%s, M2V Mode:0x%X, %s mode\n", __func__, Rbuff[0], (Rbuff[0] == 0x55) ? "BL" : "AP");

		if (Rbuff[0] != OP_MODE_BOOTLOADER) {
			//set op mode as bootloader if current op mode is not boorloader
			Wbuff[0] = 0xcb;
			Wbuff[1] = 0x80;
			Wbuff[2] = 0xc2;
			ret = TransferData(Wbuff, 3, Rbuff, 0, 1000);
			usleep(210000 + count * 20000);
		} else {
			LD_MSG("%s, current op mode is bootloader mode, can update firmware(%u/5)\n", __func__, count);
			break;
		}
	}

	//check again
	if (inConnectStyle == _ConnectStyle_USB_) {
		CloseDevice();
		usleep(500000);
		if (inProtocolStyle == _Protocol_V6_)
			sleep(1);
		for (count = 1; count < 13; count++) {
			if (InitDevice() != -1) {
				if (is_usb_hid_old_bl == 1)
					break;

				//read current op mode
				//ret = GetICMode();
				Wbuff[0] = 0xcb;
				Wbuff[1] = 0x80;
				Wbuff[2] = (uint8_t)ILITEK_TP_CMD_READ_OP_MODE;
				ret = TransferData(Wbuff, 3, Rbuff, 1, 1000);
				LD_MSG("%s, M2V Mode:0x%X, %s mode\n", __func__, Rbuff[0], (Rbuff[0] == 0x55) ? "BL" : "AP");

				if (Rbuff[0] != OP_MODE_BOOTLOADER) {
					LD_MSG("%s, currently in AP mode, wait 5 sec, after change to bootloader mode, 0x%X, %u\n",
							__func__, Rbuff[0], count);
					usleep(5000000);
					if (count == 12)
						return -7;
				} else {
					LD_MSG("%s, check again, currently bootloader mode, can update firmware, 0x%X, ret=%u\n",
							__func__, Rbuff[0], ret);
					break;
				}
			} else {
				LD_MSG("%s, currently in AP mode, wait 5 sec, after change to bootloader mode, %u\n",
						__func__, count);
				usleep(5000000);
				if (count == 12)
					return -7;
			}
		}
	}
	return 0;
}

int ChangeToAPMode_M3_M2V()
{
	int ret = -1;
	int count;

	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	for (count = 0; count < 5; count++) {
		Wbuff[0] = 0xcb;
		Wbuff[1] = 0x80;
		Wbuff[2] = 0xc1;
		ret = TransferData(Wbuff, 3, Rbuff, 0, 1000);

		usleep(500000 + count * 100000);

		//read current op mode
		if (inConnectStyle != _ConnectStyle_I2C_)
			break;

		Wbuff[0] = 0xcb;
		Wbuff[1] = 0x80;
		Wbuff[2] = (uint8_t)ILITEK_TP_CMD_READ_OP_MODE;
		ret = TransferData(Wbuff, 3, Rbuff, 1, 1000);

		if (Rbuff[0] == OP_MODE_APPLICATION) {
			LD_MSG("%s, current op mode is AP mode(%u/5), 0x%X, ret=%u\n", __func__, count, Rbuff[0], ret);
			return 0;
		}
	}

	//check again
	if (inConnectStyle != _ConnectStyle_I2C_) {
		CloseDevice();
		usleep(500000);
		for (count = 1; count < 13; count++) {
			if (InitDevice() != -1) {
				//read current op mode
				Wbuff[0] = 0xcb;
				Wbuff[1] = 0x80;
				Wbuff[2] = (uint8_t)ILITEK_TP_CMD_READ_OP_MODE;
				ret = TransferData(Wbuff, 3, Rbuff, 1, 1000);

				if (Rbuff[0] == OP_MODE_APPLICATION) {
					LD_MSG("%s, upgrade firmware finish\n", __func__);
					return 0;
				} else {
					LD_MSG("%s, currently in BL mode, wait 5 sec, after change to AP mode, %u\n",
							__func__, count);
					usleep(5000000);

					if (count == 12) {
						LD_MSG("%s, upgrade firmware failed\n", __func__);
						return -8;
					}
				}
			} else {
				LD_MSG("%s, currently in BL mode, wait 5 sec, after change to AP mode, %u\n",
						__func__, count);
				usleep(5000000);

				if (count == 12) {
					LD_MSG("%s, upgrade firmware failed\n", __func__);
					return -8;
				}
			}
		}
	}
	return -1;
}
//

int ChangeToAPMode(unsigned int start, unsigned int end)
{
	int ret;
	int count;

	for (count = 0; count < 5; count++) {
		ret = SetProgramKey(start, end);
		usleep(20000);

		//send changing bootloader command
		ret = ChangeTOAP();
		usleep(10000);

		usleep(500000 + count * 100000);
		//read current op mode
		if (inConnectStyle != _ConnectStyle_I2C_)
			break;
		ret = GetICMode(1);
		if (ptl.ic_mode[0] == OP_MODE_APPLICATION) {
			LD_MSG("%s, current op mode is AP mode(%u/5), 0x%X, ret=%u\n",
				__func__, count, ptl.ic_mode[0], ret);
			return 0;
		}
	}

	//check again
	if (inConnectStyle != _ConnectStyle_I2C_) {
		CloseDevice();
		usleep(500000);
		for (count = 1; count < 13; count++) {
			if (InitDevice() < 0) {
				LD_MSG("%s, currently in BL mode, wait 5 sec, after change to AP mode, %u\n",
						__func__, count);
				usleep(5000000);
				continue;
			}

			ret = GetICMode(1);
			if (ptl.ic_mode[0] != OP_MODE_APPLICATION) {
				LD_MSG("%s, currently in BL mode, wait 5 sec, after change to AP mode, %u\n",
						__func__, count);
				usleep(5000000);
				continue;
			}

			LD_MSG("%s, upgrade firmware finish\n", __func__);
			return 0;
		}
	}

	LD_ERR("%s, upgrade firmware failed\n", __func__);

	return -EFAULT;
}

unsigned int get_file_size(char *filename)
{
	unsigned int size;
	FILE *file = fopen(filename, "r");

	fseek(file, 0, SEEK_END);
	size = ftell(file);
	fclose(file);
	return size;
}

int DealWithFunctions(int argc, char *argv[])
{
	unsigned char u8ID = 0;
	int error;

	LD_MSG("Para:%s\n", argv[2]);

	for (u8ID = 0; u8ID < FUNC_NUM; u8ID++) {
		if (!strcmp(argv[1], au8FuncStrs[u8ID].FuncStrs)) {
			error = au8FuncStrs[u8ID].pFuncPoint(argc, argv);
			if (error < 0) {
				LD_ERR("Error! %s Failed!!\n", au8FuncStrs[u8ID].FuncStrs);
			}
			else {
				LD_MSG("%s, Success!!\n", au8FuncStrs[u8ID].FuncStrs);
			}
			return (error < 0) ? error : 0;
		}
	}

	return -EINVAL;
}

void check_args(int argc, char *argv[])
{
	int i;
	struct edid_block edid;

	cmd_opt.use_log_file = false;
	cmd_opt.log_file = NULL;

	memset(cmd_opt.save_path, 0, sizeof(cmd_opt.save_path));
	memset(cmd_opt.log_path, 0, sizeof(cmd_opt.log_path));

	memset(&cmd_opt, 0, sizeof(cmd_opt));
	memset(cmd_opt.usb.id, 0, sizeof(cmd_opt.usb.id));
	cmd_opt.hidraw_select_dev = false;
	cmd_opt.usb_hidraw = false;
	cmd_opt.dbg_level = LOG_LEVEL_MSG;

	cmd_opt.check_vendor_define = false;

	cmd_opt.i2c_devnode = false;

	cmd_opt.support_INT_ack = true;
	cmd_opt.INT_ack_flag_assigned = false;

	OTHER_VID = 0;

	cmd_opt.INT_skip_time_ms = 0;

	if (argc > 1 && !strcmp(argv[1], "-help")) {
		LD_MSG("%-20s %s\n", "Test Function", "Function Descriotion");
		for (i = 0; i < FUNC_NUM; i++)
			LD_MSG("%-20s %s\n", au8FuncStrs[i].FuncStrs, au8FuncStrs[i].FuncDesc);
		exit(0);
	}

	if (argc > 1 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) {
		LD_MSG("Common command options:\n\n"
		       "-v      Show daemon tool version\n"
		       "--edid  Show EDID\n"
		       "-help   List command description\n"
		       "--log   Enable saving daemon running log\n"
		       "--log=<folder name %%s>  Saving daemon running log to specific folder\n"
		       "--err   Change log level to error message only\n"
		       "--msg   Change log level to common message (*Default)\n"
		       "--dbg   Change log level to all debug message\n"
		       "--none  Stop showing any daemon running log\n"
		       "--no-reset      Disable SW/HW reset\n"
		       "--reset         Enable SW/HW reset (*Default)\n"
		       "--usb=<bus-port string %%s>  Using specific USB bus/port number\n"
		       "--lsusb                 List all usb port that connect to ILITEK device\n"
		       "--lsusb=<vid %%x>        List all usb port that connect to ILITEK device with special VID\n"
		       "--hidraw        USB interface only, using hidraw not libusb API\n"
		       "--hidraw=<hidraw number %%d>    Using specific hidraw number\n"
		       "--i2c-node      I2C interface only, using /dev/i2c-X not ILITEK I2C driver\n"
		       "--INT-ack       I2C interface only, enable INT ack check (*Default)\n"
		       "--no-INT-ack    I2C interface only, disable INT ack check\n"
		       "--INT-skip-ms=<time_ms %%d>     I2C interface only, to prevent unexpected INT/ISR, skip time_ms after IRQ enable\n"
		       "--save-path=<folder name %%s>   Saving CDC/ SensorTest/ Frequency report to specific folder\n"
		       "\nFor using hidraw with Two interface FW\n"
		       "--check-vendor-define   Check specific hidraw with correct report descriptor\n\n\n");
		exit(0);
	}

	for (i = 0; i < argc; i++) {
		if (!scan_args(argv[i], "--log=", "%s", cmd_opt.log_path) ||
		    !scan_args(argv[i], "--log", "")) {
			cmd_opt.use_log_file = true;
			continue;
		}

		if (!strcmp(argv[i], "--err")) {
			cmd_opt.dbg_level = LOG_LEVEL_ERR;
		} else if (!strcmp(argv[i], "--msg")) {
			cmd_opt.dbg_level = LOG_LEVEL_MSG;
		} else if (!strcmp(argv[i], "--dbg")) {
			cmd_opt.dbg_level = LOG_LEVEL_DBG;
		} else if (!strcmp(argv[i], "--dbg-packet")) {
			cmd_opt.dbg_level = LOG_LEVEL_DBGP;
		} else if (!strcmp(argv[i], "--none")) {
			cmd_opt.dbg_level = LOG_LEVEL_NONE;
		} else if (!strcmp(argv[i], "--no-reset")) {
			cmd_opt.no_sw_reset = true;
		} else if (!strcmp(argv[i], "--reset")) {
			cmd_opt.no_sw_reset = false;
		} else if (!scan_args(argv[i], "--usb=", "%s",
				      cmd_opt.usb.id)) {
			continue;
		} else if (!scan_args(argv[i], "--hidraw=",
				      "%d", &cmd_opt.hidraw_id)) {
			cmd_opt.hidraw_select_dev = true;
		} else if (!strcmp(argv[i], "--hidraw")) {
			cmd_opt.usb_hidraw = true;
		} else if (!strcmp(argv[i], "--i2c-node")) {
			cmd_opt.i2c_devnode = true;
		} else if (!scan_args(argv[i], "--save-path=", "%s",
				      cmd_opt.save_path)) {
			continue;
		} else if (!strcmp(argv[i], "--check-vendor-define")) {
			cmd_opt.check_vendor_define = true;
		}

		if (!strcmp(argv[i], "--no-INT-ack")) {
			cmd_opt.support_INT_ack = false;
			cmd_opt.INT_ack_flag_assigned = true;
		} else if (!strcmp(argv[i], "--INT-ack")) {
			cmd_opt.support_INT_ack = true;
			cmd_opt.INT_ack_flag_assigned = true;
		}

		if (!strcmp(argv[i], "-v")) {
			LD_MSG("%s\n", TOOL_VERSION);
			exit(0);
		}

		if (!strcmp(argv[i], "--edid")) {
			get_edid(&edid, NULL, 0);
			exit(0);
		}

		if (!strcmp(argv[i], "--lsusb") ||
		    !scan_args(argv[i], "--lsusb=", "%x", &OTHER_VID)) {
			list_ilitek_usb_ports(OTHER_VID);
			exit(0);
		}

		if (!scan_args(argv[i], "--INT-skip-ms=", "%d",
			&cmd_opt.INT_skip_time_ms))
			continue;
	}
}

static void check_INT_ack()
{
	uint32_t driver_ver = 0;

	if (inConnectStyle != _ConnectStyle_I2C_)
		return;

	/* If /dev/i2c-X node is used, INT ack should be unavailable */
	if (cmd_opt.i2c_devnode) {
		cmd_opt.support_INT_ack = false;
		return;
	}

	if (!cmd_opt.INT_ack_flag_assigned) {
		cmd_opt.support_INT_ack = true;

		driver_ver = get_driver_ver();
		if (driver_ver < 0x05090004)
			cmd_opt.support_INT_ack = false;
	}

	LD_MSG("I2C check INT ack: %s\n", (cmd_opt.support_INT_ack) ? "ON" : "OFF");
}

void check_command_args(int argc, char *argv[])
{
	if (argc < 2)
		return;
	if (!strcmp(argv[1], "FWUpgrade")) {
		check_upgrade_args(argc, argv);
		return;
	}
}

int main(int argc, char *argv[])
{
	int error;
	int cnt = 0;
	LD_MSG("argc=%d\n", argc);
	for (cnt = 0; cnt < argc; cnt++)
		LD_MSG("%s", argv[cnt]);
	check_args(argc, argv);
	show_command_help(argc, argv);
	check_command_args(argc, argv);

	if (cmd_opt.use_log_file)
		cmd_opt.log_file = log_openfile(cmd_opt.log_path,
						(char *)"ld_log");

	LD_MSG("%s\n", TOOL_VERSION);

	if ((error = SetConnectStyle(argv)) < 0)
		goto err_return;
	if ((error = InitDevice()) < 0)
		goto err_return;

	check_INT_ack();

	switch_irq(0);

	if (strcmp(argv[1], "Chrome"))
		viSetTestMode(true, 100);

	error = DealWithFunctions(argc, argv);

	if (strcmp(argv[1], "Chrome")) {
		viSetTestMode(false, 100);
		software_reset();
	}

	switch_irq(1);

	CloseDevice();

err_return:
	LD_MSG("main ret = %d\n", error);
	if (cmd_opt.use_log_file) {
		log_closefile(cmd_opt.log_file);
		cmd_opt.log_file = NULL;
	}

	return (error < 0) ? error : 0;
}

#endif
