// SPDX-License-Identifier: GPL-2.0
/*
 * ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 *
 * The code could be used by anyone for any purpose,
 * and could perform firmware update for ILITEK's touch IC.
 */

#ifndef _ILITEK_MAIN_C_
#define _ILITEK_MAIN_C_

/* Includes of headers ------------------------------------------------------*/
#include "ILITek_Protocol.h"
#include "ILITek_CMDDefine.h"
#include "ILITek_Device.h"
#include "ILITek_Debug.h"
#include "ILITek_Wifi.h"
#include "ILITek_Main.h"
#include "API/ILITek_Frequency.h"
#include "API/ILITek_RawData.h"
#include "API/ILITek_SensorTest.h"
#include "API/ILITek_Upgrade.h"
#include "API/ILITek_MpResult.h"
#include <stdio.h>
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

#define DEBUG_DESC		"Show ILITEK debug use"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define DEBUG_USB	{ "USB", "V3/V6", "null", "null", "Debug Para", "", "" }
#define	DEBUG_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "Debug Para", "", "" }

#define PANEL_INFOR_DESC	"Show the sensor panel information"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	PANEL_INFOR_USB	{ "USB", "V3/V6", "null", "null", "", "", "" }
#define	PANEL_INFOR_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "", "", "" }

#define RAW_DATA_DESC		"Get the sensor raw data"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	RAW_DATA_USB	{ "USB", "V3/V6", "null", "null", "Frames", "", "" }
#define	RAW_DATA_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "Frames", "", "" }

#define BG_RAW_DATA_DESC	"Get the sensor delta C data (BG - Raw)"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	BG_RAW_DATA_USB	{ "USB", "V3/V6", "null", "null", "Frames", "", "" }
#define	BG_RAW_DATA_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "Frames", "", "" }

#define BG_DATA_DESC		"Get thhe sensor background data"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	BG_DATA_USB	{ "USB", "V3/V6", "null", "null", "Frames", "", "" }
#define	BG_DATA_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "Frames", "", "" }

#define SENSOR_TEST_DESC	"Run MP sensor test"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	SENSOR_TEST_USB	{ "USB", "V3/V6", "null", "null", "Functions", "[Profile path]", "" }
#define	SENSOR_TEST_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "Functions", "[Profile path]", "" }

#define FREQ_DESC		"Get different frequency data"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	FREQ_USB	{ "USB", "V3/V6", "null", "null", "Start Value", "End Value", "Step" }
#define	FREQ_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "Start Value", "End Value", "Step" }

#define FW_UPDATE_DESC		"Run FW update"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	FW_UPDATE_USB	{ "USB", "V3/V6", "null", "null", "Hex Path", "[Version]", "" }
#define	FW_UPDATE_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "Hex Path", "[Version]", "" }

#define CONSOLE_DESC		"Run ILITEK CMD"
//daemon function format	Interface, Write len, Read len, Write Data
#define	CONSOLE_USB	{ "USB", "Write len", "Read len", "Write Data", "", "", "" }
#define	CONSOLE_I2C	{ "I2C", "Write len", "Read len", "Write Data", "", "", "" }

#define SCRIPT_DESC		"Run script CMD"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	SCRIPT_USB	{ "USB", "null", "null", "null", "[Script path]", "", "" }
#define	SCRIPT_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "[Script path]", "", "" }

#define CTRL_MODE_DESC		"Control FW mode"
//daemon function forma		Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	CTRL_MODE_USB	{ "USB", "V3/V6", "null", "null", "set mode", "", "" }
#define	CTRL_MODE_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "set mode", "", "" }

#define CDC_DESC		"Run CDC"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	CTRL_CDC_USB	{ "USB", "V3/V6", "null", "null", "set CDC type", "Frames", "" }
#define	CTRL_CDC_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "set CDC type", "Frames", "" }

#define MP_RESULT_DESC	"Read MP Result"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	MP_RESULT_USB	{ "USB", "V3/V6", "null", "null", "", "", "" }
#define	MP_RESULT_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "", "", "" }

#define EXC_DESC		"Run monitor extension (USB only)"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	EXC_USB		{ "USB", "null","null", "null", "[Script path]","","" }
#define EXC_I2C		{ "I2C","Write len","Read len","Data","","","" }

#define COU_DESC		"Run monitor copy (USB only)"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	COU_USB		{ "USB", "V3", "null", "null", "", "", "" }
#define	COU_I2C		{ "I2C", "V3", "/dev/ilitek_ctrl", "41", "", "", "" }

#define FCU_DESC		"Show FW status (USB only)"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	FCU_USB		{ "USB", "V3", "null", "null", "", "", "" }
#define	FCU_I2C		{ "I2C", "V3", "/dev/ilitek_ctrl", "41", "", "", "" }

#define SRU_DESC		"Run FW soft reset (USB only)"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	SRU_USB		{ "USB", "V3", "null", "null", "", "", "" }
#define	SRU_I2C		{ "I2C", "V3", "/dev/ilitek_ctrl", "41", "", "", "" }

#define STU_DESC		"Run FW test mode (USB only)"
//daemon function format	Interface, Protocol, Device, Addr, Ctrl_para1, Ctrl_para2, Ctrl_para3
#define	STU_USB		{ "USB", "V3", "null", "null", "", "", "" }
#define	STU_I2C		{ "I2C", "V3", "/dev/ilitek_ctrl", "41", "", "", "" }

#define WIFI_DESC		"for Remonte/Wifi use"
#define	WIFI_USB	{ "USB", "V3/V6", "null", "null", "Server IP", "", "" }
#define	WIFI_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "Server IP", "", "" }

#define Test_ChangeMode_DESC	"for Linux Daemon Functional Test use"
#define	Test_ChangeMode_USB	{ "USB", "V3/V6", "null", "null", "", "", "" }
#define	Test_ChangeMode_I2C	{ "I2C", "V3/V6", "/dev/ilitek_ctrl", "41", "", "", "" }

#define READFLASH_DESC		"Show Flash data"
//daemon function format	connect IP

/* Private function prototypes -----------------------------------------------*/
int Func_Chrome(int argc, char *argv[]);
int Func_Debug(int argc, char *argv[]);
int Func_PanelInfo(int argc, char *argv[]);
int Func_RawData(int argc, char *argv[]);
int Func_BGRawData(int argc, char *argv[]);
int Func_BGData(int argc, char *argv[]);
int Func_SensorTest(int argc, char *argv[]);
int Func_Frequency(int argc, char *argv[]);
int Func_FWUpgrade(int argc, char *argv[]);
int Func_Console(int argc, char *argv[]);
int Func_Script(int argc, char *argv[]);
int Func_ReadFlash(int argc, char *argv[]);
int Func_CtrlMode(int argc, char *argv[]);
int Func_MpResult(int argc, char *argv[]);
int Func_CDC(int argc, char *argv[]);
int Func_Exu(int argc, char *argv[]);
int Func_Cou(int argc, char *argv[]);
int Func_Fcu(int argc, char *argv[]);
int Func_Sru(int argc, char *argv[]);
int Func_Stu(int argc, char *argv[]);
int Func_Wifi(int argc, char *argv[]);
int Func_Test_ChangeMode(int argc, char *argv[]);


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
	{"Debug",		Func_Debug,		DEBUG_DESC,		{DEBUG_USB, DEBUG_I2C},},
	{"PanelInfor",		Func_PanelInfo,		PANEL_INFOR_DESC,	{PANEL_INFOR_USB, PANEL_INFOR_I2C},},
	{"RawData",		Func_RawData,		RAW_DATA_DESC,		{RAW_DATA_USB, RAW_DATA_I2C},},
	{"BG-RawData",		Func_BGRawData,		BG_RAW_DATA_DESC,	{BG_RAW_DATA_USB, BG_RAW_DATA_I2C},},
	{"BGData",		Func_BGData,		BG_DATA_DESC,		{BG_DATA_USB, BG_DATA_I2C},},
	{"SensorTest",		Func_SensorTest,	SENSOR_TEST_DESC,	{SENSOR_TEST_USB, SENSOR_TEST_I2C},},
	{"Frequency",		Func_Frequency,		FREQ_DESC,		{FREQ_USB, FREQ_I2C},},
	{"FWUpgrade",		Func_FWUpgrade,		FW_UPDATE_DESC,		{FW_UPDATE_USB, FW_UPDATE_I2C},},
	{"Console",		Func_Console,		CONSOLE_DESC,		{CONSOLE_USB, CONSOLE_I2C},},
	{"Script",		Func_Script,		SCRIPT_DESC,		{SCRIPT_USB, SCRIPT_I2C},},
	{"ControlMode",		Func_CtrlMode,		CTRL_MODE_DESC,		{CTRL_MODE_USB, CTRL_MODE_I2C},},
	{"CDC",			Func_CDC,		CDC_DESC,		{CTRL_CDC_USB, CTRL_CDC_I2C},},
	{"MpResult",		Func_MpResult,		MP_RESULT_DESC,		{MP_RESULT_USB, MP_RESULT_I2C},},
	{"-exu",		Func_Exu,		EXC_DESC,		{EXC_USB, EXC_I2C},},
	{"-cou",		Func_Cou,		COU_DESC,		{COU_USB, COU_I2C},},
	{"-fcu",		Func_Fcu,		FCU_DESC,		{FCU_USB, FCU_I2C},},
	{"-sru",		Func_Sru,		SRU_DESC,		{SRU_USB, SRU_I2C},},
	{"-stu",		Func_Stu,		STU_DESC,		{STU_USB, STU_I2C},},
	{"ReadFlash",		Func_ReadFlash,		READFLASH_DESC,		{},},
	{"Wifi",		Func_Wifi,		WIFI_DESC,		{WIFI_USB, WIFI_I2C},},
	{"Test_ChangeMode",	Func_Test_ChangeMode,	Test_ChangeMode_DESC,	{Test_ChangeMode_USB, Test_ChangeMode_I2C},},
};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
int Func_Test_ChangeMode(int argc, char *argv[])
{
	int i;
	int error = 0;

	LD_MSG("[%s] start\n", __func__);

	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "--BL"))
			error = ChangeToBootloader();
		else if (!strcmp(argv[i], "--AP"))
			error = ChangeToAPMode();
	}

	return error;
}

int Func_Chrome(int argc, char *argv[])
{
	int ret = _FAIL;

	UNUSED(argc);
	UNUSED(argv);

	if (viSetTestMode(true, 0))
		return _FAIL;
	if (GetFWVersion() != _FAIL)
		ret = _SUCCESS;
	if (viSetTestMode(false, 0))
		return _FAIL;
	return ret;
}

int Func_Debug(int argc, char *argv[])
{
	int ret = _FAIL;

	UNUSED(argv);

	if (argc >= 6)
		ret = Debug_Main();

	return ret;
}

int Func_PanelInfo(int argc, char *argv[])
{
	int ret = _FAIL;

	UNUSED(argv);

	if ((inConnectStyle == _ConnectStyle_I2C_ && argc >= 6) ||
	    (inConnectStyle == _ConnectStyle_USB_ && argc >= 4) ||
	    (inConnectStyle == _ConnectStyle_I2CHID_ && argc >= 4))
		ret = viGetPanelInfor();
	return ret;
}

int Func_MpResult(int argc, char *argv[])
{
	int ret = _FAIL;

	UNUSED(argv);

	if ((inConnectStyle == _ConnectStyle_I2C_ && argc >= 6) ||
	    (inConnectStyle == _ConnectStyle_USB_ && argc >= 4) ||
	    (inConnectStyle == _ConnectStyle_I2CHID_&& argc >= 4))
		ret = viGetMpResult();
	return ret;
}

int Func_RawData(int argc, char *argv[])
{
	int ret = _FAIL;

	if (argc >= 7)
		ret = viRunCDCData(atoi(argv[6]));
	return ret;
}

int Func_BGRawData(int argc, char *argv[])
{
	int ret = _FAIL;

	if (argc >= 7)
		ret = viRunBGMinusCDCData(atoi(argv[6]));
	return ret;
}

int Func_BGData(int argc, char *argv[])
{
	int ret = _FAIL;

	if (argc >= 7)
		ret = viRunBGData(atoi(argv[6]));

	return ret;
}

int Func_CDC(int argc, char *argv[])
{
	int ret = _FAIL;

	if (argc >= 7)
		ret = viRunCDCType(argv);
	return ret;
}

int Func_SensorTest(int argc, char *argv[])
{
	int ret = _FAIL;
	int inFunctions = 0;

	if (argc >= 7) {
		inFunctions = atoi(argv[6]);
		if (inProtocolStyle == _Protocol_V6_)
			InitialSensorTestV6Parameter();
		else if (inProtocolStyle == _Protocol_V3_)
			InitialSensorTestV3Parameter();

		if (argc == 8 && strcmp(argv[7], "null") != 0) {
			memset(IniPath, 0, sizeof(IniPath));
			strcpy((char *)IniPath, argv[7]);
		}
		ret = viRunSensorTest(inFunctions);
	}

	return ret;
}

int Func_Frequency(int argc, char *argv[])
{
	int ret = _FAIL;

	if (argc >= 8) {
		LD_MSG("[%s] start:%d, end:%d, step:%d\n", __func__,
			atoi(argv[6]), atoi(argv[7]), atoi(argv[8]));
		ret = viRunFre(argv);
	}
	return ret;
}

void check_upgrade_args(int argc, char *argv[])
{
	int i;

	upg.force_update = false;
	upg.args_fw_ver_check = false;

	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "--force-upgrade")) {
			upg.force_update = true;
		} else if (!strncmp(argv[i], "--fw-ver=", 9)) {
			upg.args_fw_ver_check = true;
			upg.args_len = sscanf(argv[i], "--fw-ver=%hhu.%hhu.%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
				upg.args_fw_ver, upg.args_fw_ver + 1,
				upg.args_fw_ver + 2, upg.args_fw_ver + 3,
				upg.args_fw_ver + 4, upg.args_fw_ver + 5,
				upg.args_fw_ver + 6, upg.args_fw_ver + 7);
		}
	}

	LD_MSG("force upgrade: %d\n", upg.force_update);
	LD_MSG("fw check: %d, args len: %d, fw: %hhu.%hhu.%hhu.%hhu.%hhu.%hhu.%hhu.%hhu\n",
		upg.args_fw_ver_check, upg.args_len, upg.args_fw_ver[0],
		upg.args_fw_ver[1], upg.args_fw_ver[2], upg.args_fw_ver[3],
		upg.args_fw_ver[4], upg.args_fw_ver[5], upg.args_fw_ver[6],
		upg.args_fw_ver[7]);
}


int Func_FWUpgrade(int argc, char *argv[])
{
	char filename[UPGRAD_FILE_PATH_SIZE];

	if (argc < 7)
		return _FAIL;

	memset(filename, 0, UPGRAD_FILE_PATH_SIZE);

	strcpy(filename, argv[6]);
	LD_MSG("Hex filename:%s\n", filename);

	check_upgrade_args(argc, argv);

	return viRunFiremwareUpgrade(filename);
}


int Func_Console(int argc, char *argv[])
{
	int ret = _FAIL;

	if (argc >= 9)
		ret = viConsoleData(argv);
	return ret;
}

int Func_Script(int argc, char *argv[])
{
	int ret = _FAIL;

	if (argc >= 6)
		ret = viScript(argv);
	return ret;
}

int Func_Wifi(int argc, char *argv[])
{
	int ret = _FAIL;

	if (argc >= 7)
		ret = Wifi_Main(argv[6]);
	return ret;
}

int Func_ReadFlash(int argc, char *argv[])
{
	int ret = _FAIL;
	int start_addr = 0;
	int read_len = 0;

	UNUSED(argc);

	sscanf(argv[6], "%x", &start_addr);
	sscanf(argv[7], "%x", &read_len);
	LD_MSG("Read Flash, Start Address:0x%x, End Address:0x%x, Read Lenght:%d\n",
		start_addr, start_addr + read_len - 1, read_len);
	if (inProtocolStyle == _Protocol_V6_) {
		if (GetFlashData_V6(start_addr, read_len, argv[8]) == _SUCCESS) {
			ret = _SUCCESS;
			LD_MSG("Read Flash SUCCESS\n");
		} else {
			LD_ERR("Read Flash FAIL\n");
		}
	} else if (inProtocolStyle == _Protocol_V3_) {
		if (GetFlashData_V3(start_addr, read_len, argv[8]) == _SUCCESS) {
			ret = _SUCCESS;
			LD_MSG("Read Flash SUCCESS\n");
		} else {
			LD_ERR("Read Flash FAIL\n");
		}
	}

	return ret;
}

int Func_CtrlMode(int argc, char *argv[])
{
	int ret = _FAIL;
	int mode = atoi(argv[6]);

	if (argc >= 6 && mode < 10 && mode >= 0)
		ret = viSwitchMode(mode);
	return ret;
}

int Func_Exu(int argc, char *argv[])
{
	UNUSED(argc);
	UNUSED(argv);

	return monitor_extend();
}

int Func_Cou(int argc, char *argv[])
{
	UNUSED(argc);
	UNUSED(argv);

	return monitor_Copy();
}

int Func_Fcu(int argc, char *argv[])
{
	UNUSED(argc);
	UNUSED(argv);

	return check_status();
}

int Func_Sru(int argc, char *argv[])
{
	UNUSED(argc);
	UNUSED(argv);

	return software_reset();

}

int Func_Stu(int argc, char *argv[])
{
	UNUSED(argc);

	return switch_testmode((uint8_t *)argv[2], (uint8_t *)argv[3]);
}

int print_tool_info(char *argv[])
{
	int i, j, ret = _FAIL;

	if (!strcmp(argv[1], "-v")) {
		LD_MSG("%s\n", TOOL_VERSION);
		ret = _SUCCESS;
	} else if (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help") ||
		   !strcmp(argv[1], "-help")) {
		LD_MSG("%-20s %s\n", "Test Function", "Function Descriotion");
		for (i = 0; i < FUNC_NUM; i++ )
			LD_MSG("%-20s %s\n", au8FuncStrs[i].FuncStrs, au8FuncStrs[i].FuncDesc);
		ret = _SUCCESS;
	} else if (!strcmp(argv[2], "-h") || !strcmp(argv[2], "--help") ||
		   !strcmp(argv[2], "-help")) {
		for (i = 0; i < FUNC_NUM; i++ ) {
			if (!strcmp(argv[1], au8FuncStrs[i].FuncStrs)) {
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
				ret = _SUCCESS;
				break;
			}
		}
	}

	return ret;
}

int viGetPanelInfor_V3()
{
	if (GetProtocol() < 0 || GetKernelVer() < 0 || GetFWVersion() < 0 ||
	    GetICMode() < 0)
		return _FAIL;

	/* return success if it's BL mode */
	if (ICMode == 0x55) {
		LD_ERR("FW in BL mode, wait for FWUpgrade...\n");
		return _SUCCESS;
	}

	if (GetCoreVersion() < 0 || GetFWMode() < 0 || PanelInfor_V3() < 0 ||
	    GetCRC_V3() < 0)
	    	return _FAIL;

	return _SUCCESS;
}

int viGetPanelInfor_V6()
{
	if (GetProtocol() < 0 || GetKernelVer() < 0 || GetFWVersion() < 0 ||
	    GetICMode() < 0)
		return _FAIL;

	/* return success if it's BL mode */
	if (ICMode == 0x55) {
		LD_ERR("FW in BL mode, wait for FWUpgrade...\n");
		return _SUCCESS;
	}

	if (GetCoreVersion() < 0 || GetFWMode() < 0 || PanelInfor_V6() < 0 ||
	    GetCRC_V6() < 0)
		return _FAIL;

	return _SUCCESS;
}

int viSetTestMode(bool setTest, int delay_ms)
{
	int ret = _FAIL;

	if (inProtocolStyle == _Protocol_V3_) {
		if (setTest)
			ret = EnterTestMode(delay_ms);
		else
			ret = ExitTestMode(delay_ms);
	} else if (inProtocolStyle == _Protocol_V6_) {
		if (setTest)
			ret = ModeCtrl_V6(ENTER_SUSPEND_MODE,
					  ENABLE_ENGINEER, delay_ms);
		else
			ret = ModeCtrl_V6(ENTER_NORMAL_MODE,
					  DISABLE_ENGINEER, delay_ms);
	}
	return ret;
}

int viGetPanelInfor()
{
	int ret = _FAIL;

	LD_MSG("%s\n", TOOL_VERSION);
	if (inProtocolStyle == _Protocol_V3_)
		ret = viGetPanelInfor_V3();
	else if (inProtocolStyle == _Protocol_V6_)
		ret = viGetPanelInfor_V6();

	return ret;
}

int ChangeToBootloader()
{
	int ret = _SUCCESS;
	int count;

	for (count = 0; count < 5; count++) {
		ret = GetICMode();
		if (ICMode != OP_MODE_BOOTLOADER) {
			ret = SetProgramKey();
			usleep(20000);

			ret = ChangeTOBL();

			/*
			 * Lego's old BL may trigger unexpected INT after 0xC2,
			 * so make I2C driver handle it ASAP
			 */
			if (support_INT_ack && inProtocolStyle == _Protocol_V6_)
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
			if (InitDevice() != _FAIL) {
				if (is_usb_hid_old_bl == 1)
					break;

				ret = GetICMode();
				if (ICMode != OP_MODE_BOOTLOADER) {
					LD_MSG("%s, currently in AP mode, wait 5 sec, after change to bootloader mode, 0x%X, %u\n",
							__func__, ICMode, count);
					usleep(5000000);
					if (count == 12)
						return _FAIL;
				} else {
					LD_MSG("%s, check again, currently bootloader mode, can update firmware, 0x%X, ret=%u\n",
							__func__, ICMode, ret);
					break;
				}
			} else {
				LD_MSG("%s, currently in AP mode, wait 5 sec, after change to bootloader mode, %u\n",
						__func__, count);
				usleep(5000000);
				if (count == 12)
					return _FAIL;
			}
		}
	}

	GetProtocol_in_BL();

	if (inProtocolStyle == _Protocol_V6_)
		GetFWVersion_BL();

	return _SUCCESS;
}

int ChangeToAPMode()
{
	int ret = _FAIL;
	int count;

	for (count = 0; count < 5; count++) {
		ret = SetProgramKey();
		usleep(20000);

		//send changing bootloader command
		ret = ChangeTOAP();
		usleep(10000);

		usleep(500000 + count * 100000);
		//read current op mode
		if (inConnectStyle != _ConnectStyle_I2C_)
			break;
		ret = GetICMode();
		if (ICMode == OP_MODE_APPLICATION) {
			LD_MSG("%s, current op mode is AP mode(%u/5), 0x%X, ret=%u\n", __func__, count, ICMode, ret);
			return _SUCCESS;
		}
	}

	//check again
	if (inConnectStyle != _ConnectStyle_I2C_) {
		CloseDevice();
		usleep(500000);
		for (count = 1; count < 13; count++) {
			if (InitDevice() != _FAIL) {
				//read current op mode
				ret = GetICMode();
				if (ICMode == OP_MODE_APPLICATION) {
					LD_MSG("%s, upgrade firmware finish\n", __func__);
					return _SUCCESS;
				} else {
					LD_MSG("%s, currently in BL mode, wait 5 sec, after change to AP mode, %u\n",
							__func__, count);
					usleep(5000000);

					if (count == 12) {
						LD_ERR("%s, upgrade firmware failed\n", __func__);
						return _FAIL;
					}
				}
			} else {
				LD_MSG("%s, currently in BL mode, wait 5 sec, after change to AP mode, %u\n",
						__func__, count);
				usleep(5000000);

				if (count == 12) {
					LD_ERR("%s, upgrade firmware failed\n", __func__);
					return _FAIL;
				}
			}
		}
	}
	return _FAIL;
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
	int ret = _FAIL;

	LD_MSG("Para:%s\n", argv[2]);

	for (u8ID = 0; u8ID < FUNC_NUM; u8ID++) {
		if (!strcmp(argv[1], au8FuncStrs[u8ID].FuncStrs)) {
			ret = au8FuncStrs[u8ID].pFuncPoint(argc, argv);
			if (ret < 0)
				LD_ERR("Error! %s Failed!!\n", au8FuncStrs[u8ID].FuncStrs);
			else
				LD_MSG("%s, Success!!\n", au8FuncStrs[u8ID].FuncStrs);
		}
	}
	return ret;
}

unsigned char chartohex(char *str)
{
	unsigned int temp;

	sscanf(str, "%x", &temp);
	LD_MSG("the temp is %x\n", temp);
	return temp;
}

int viConsoleData(char *argv[])
{
	int count = 0;
	int inWlen = 0, inRlen = 0;
	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	inWlen = atoi(argv[6]);
	inRlen = atoi(argv[7]);


	for (count = 0; count < inWlen; count++)
		Wbuff[count] = (unsigned char)chartohex(argv[8 + count]);

	if (TransferData(Wbuff, inWlen, Rbuff, inRlen, 1000) < 0)
		return _FAIL;

	LD_MSG("%s, Return data: ", __func__);
	for (count = 0; count < inRlen; count++) {
		tempbuff[count] = Rbuff[count];
		LD_MSG("%.2X.", tempbuff[count]);
	}
	LD_MSG("\n");

	return _SUCCESS;
}

int viScript(char *argv[])
{
	FILE *fp;
	unsigned char i = 0;
	unsigned int u16_delay_time = 1;
	unsigned char u8_chk_buf[MAX_SCRIPT_CMD_SIZE];
	unsigned char u8_chk_cnt = 0;
	int inWlen = 0, inRlen = 0;

	LD_MSG("Script filename:%s\n", argv[6]);

	fp = fopen(argv[6], "r");
	if (fp == NULL) {
		LD_ERR("%s, cannot open %s file\n", __func__, argv[6]);
		return _FAIL;
	}

	while (!feof(fp)) {
		if (fgets((char *)u8_chk_buf, MAX_SCRIPT_CMD_SIZE, fp)!=NULL) {
			if (CHK_I2C(u8_chk_buf)) {
				sscanf((char *)u8_chk_buf, "I2C %d %d", &inWlen, &inRlen);
				/* Parser write data */
				for (i=0; i<strlen((char *)u8_chk_buf); i++) {
					if (*(u8_chk_buf+i)==' ') {
						if (u8_chk_cnt++ < 2)
							continue;
						else
							sscanf((char *)(u8_chk_buf+i), "%2X", (int *)&buff[u8_chk_cnt-3]);
					}
				}
				if (TransferData(buff, inWlen, buff, inRlen, 1000) < 0)
					return _FAIL;
				LD_MSG("%s, Return data: ", __func__);
				for (i=0; i<inRlen; i++) {
					tempbuff[i] = buff[i];
					LD_MSG("%.2X.", tempbuff[i]);
				}
			} else if (CHK_DELAY(u8_chk_buf)) {
				u16_delay_time = 1;
				sscanf((char *)u8_chk_buf, "Delay %d", &u16_delay_time);
				usleep(u16_delay_time*1000);
			}
		}
	}

	fclose(fp);

	return _SUCCESS;
}

int viSwitchMode(int mode)
{
	int ret = _FAIL;

	LD_MSG("%s\n", TOOL_VERSION);
	if (inProtocolStyle == _Protocol_V3_)
		ret = viSwitchMode_V3(mode);
	else if (inProtocolStyle == _Protocol_V6_)
		ret = viSwitchMode_V6(mode);
	return ret;
}

FILE *log_file = NULL;
int dbg_level = LOG_LEVEL_MSG;
bool use_log_file = false;
void log_openfile()
{
	if (!use_log_file)
		return;

	time_t rawtime;
	struct tm *timeinfo;
	char timebuf[60], log_filename[512];
	char log_dirname[] = "ilitek_ld_log";

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(timebuf, 60, "%Y%m%d_%I%M%S", timeinfo);

	if (access(log_dirname, 0) < 0) {
		if (mkdir(log_dirname, 0777)) {
			LD_ERR("create directory %s failed!\n", log_dirname);
			return;
		}
	}

	sprintf(log_filename,"%s/ld_log_%s.txt", log_dirname, timebuf);
	log_file = fopen(log_filename, "w");

	LD_MSG("*******************************************\n");
	LD_MSG("************** Start Logging **************\n");
	LD_MSG("*******************************************\n\n");
}

void log_closefile()
{
	if (!use_log_file || !log_file)
		return;

	LD_MSG("\n");
	LD_MSG("*******************************************\n");
	LD_MSG("************** End of Logging *************\n");
	LD_MSG("*******************************************\n");
	fclose(log_file);
}

bool no_sw_reset = false;
void check_args(int argc, char *argv[])
{
	int i;

	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "--log"))
			use_log_file = true;
		else if (!strcmp(argv[i], "--err"))
			dbg_level = LOG_LEVEL_ERR;
		else if (!strcmp(argv[i], "--msg"))
			dbg_level = LOG_LEVEL_MSG;
		else if (!strcmp(argv[i], "--dbg"))
			dbg_level = LOG_LEVEL_DBG;
		else if (!strcmp(argv[i], "--none"))
			dbg_level = LOG_LEVEL_NONE;
		else if (!strcmp(argv[i], "--no-reset"))
			no_sw_reset = true;
		else if (!strcmp(argv[i], "--reset"))
			no_sw_reset = false;
	}
}

bool support_INT_ack = true;
void check_INT_ack(int argc, char *argv[])
{
	int i;
	uint32_t driver_ver = 0;

	if (inConnectStyle != _ConnectStyle_I2C_)
		return;

	driver_ver = get_driver_ver();
	if (driver_ver < 0x05090004)
		support_INT_ack = false;

	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "--no-INT-ack"))
			support_INT_ack = false;
		else if (!strcmp(argv[i], "--INT-ack"))
			support_INT_ack = true;
	}

	LD_MSG("I2C check INT ack: %s\n", (support_INT_ack) ? "ON" : "OFF");
}

int main(int argc, char *argv[])
{
	int ret = _FAIL;

	if(argc < 7){
		LD_ERR("Incorrect number of arguments\n");
		LD_ERR("Arguments should be: FWUpgrade <Interface> <Protocol> <i2c driver file node> <i2c addr> <Hex path>\n");
		LD_ERR("Example arguments: FWUpgrade USB V3 /dev/ilitek_ctrl 41 firmware.hex\n");
		return ret;
	}

	check_args(argc, argv);

	log_openfile();

	if (print_tool_info(argv) == _SUCCESS) {
		ret = _SUCCESS;
	} else if (SetConnectStyle(argv) == _SUCCESS) {
		if (InitDevice() == _SUCCESS) {
			check_INT_ack(argc, argv);

			switch_irq(0);

			if (strcmp(argv[1], "Console") &&
			    strcmp(argv[1], "Chrome") &&
			    strcmp(argv[1], "Debug"))
				viSetTestMode(true, 100);

			if (DealWithFunctions(argc, argv) < 0)
				ret = _FAIL;
			else
				ret = _SUCCESS;

			if (strcmp(argv[1], "Console") &&
			    strcmp(argv[1], "Chrome") &&
			    strcmp(argv[1], "Debug")) {
				viSetTestMode(false, 100);
				software_reset();
			}

			switch_irq(1);
			CloseDevice();
		} else {
			LD_ERR("InitDevice Error\n");
		}
	} else {
		LD_ERR("argv error");
	}

	LD_MSG("main ret = %d\n", ret);

	log_closefile();

	return ret;
}

#endif
