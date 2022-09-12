/*
 * Copyright (c) 2019 ILI Technology Corp.
 *
 * This file is part of ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 */
#include "ILITek_Device.h"
#include "ILITek_CMDDefine.h"
#include "ILITek_Protocol.h"
#include "API/ILITek_Upgrade.h"
#include "ILITek_Main.h"

struct ptl_data ptl;

int software_reset()
{
	int error;
	uint8_t Wbuff[64], Rbuff[64];

	if (cmd_opt.no_sw_reset)
		return 0;

	/* Do not software reset for I2C-HID interface */
	if (inConnectStyle == _ConnectStyle_I2CHID_)
		return 0;

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_SOFTWARE_RESET;
	error = TransferData(Wbuff, 1, Rbuff, 0, 0);

	if (inProtocolStyle == _Protocol_V3_)
		usleep(300000);
	else
		usleep(1000000);

	if (error < 0)
		return error;

	LD_MSG("[%s] err: %d\n", __func__, error);

	return 0;
}

int GetCoreVersion()
{
	int ret;
	uint8_t Wbuff[64], Rbuff[64];

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_GET_INTERNAL_VERSION;
	ret = TransferData(Wbuff, 1, Rbuff, 7, 1000);
	memcpy(ptl.core_ver, Rbuff, 7);

	LD_DBG("[CoreVersion] %#02X.%#02X.%#02X.%#02X\n",
		ptl.core_ver[0], ptl.core_ver[1],
		ptl.core_ver[2], ptl.core_ver[3]);

	return ret;
}

int GetFWVersion()
{
	int ret;

	uint8_t Wbuff[64], Rbuff[64];

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_GET_FIRMWARE_VERSION;
	ret = TransferData(Wbuff, 1, Rbuff, 8, 1000);

	memcpy(ptl.fw_ver, Rbuff, 8);
	LD_DBG("[Firmware Version] %#02X.%#02X.%#02X.%#02X.%#02X.%#02X.%#02X.%#02X\n",
		ptl.fw_ver[0], ptl.fw_ver[1], ptl.fw_ver[2], ptl.fw_ver[3],
		ptl.fw_ver[4], ptl.fw_ver[5], ptl.fw_ver[6], ptl.fw_ver[7]);

	return ret;
}

//M3 + M2V
int GetFWVersion_M3_M2V()
{
	int ret = 0;

	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	int i = 0, retry = 5;

	for(i = 0; i < retry; i++) {
		Wbuff[0] = 0xcb;
		Wbuff[1] = 0x80;
		Wbuff[2] = (uint8_t)ILITEK_TP_CMD_GET_FIRMWARE_VERSION;
		ret = TransferData(Wbuff, 3, Rbuff, 8, 1000);

		if(Rbuff[0] != 0x5A) {
			break;
		}

		usleep(100000);
	}

	LD_MSG("%s, M2V firmware version: 0x%02X.0x%02X.0x%02X.0x%02X.0x%02X.0x%02X.0x%02X.0x%02X, ret = %u, i = %d\n", __func__,
	Rbuff[0], Rbuff[1], Rbuff[2], Rbuff[3], Rbuff[4], Rbuff[5], Rbuff[6], Rbuff[7], ret, i);

	return ret;
}

int GetProtocol()
{
	int ret;
	uint8_t Wbuff[64], Rbuff[64];

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_GET_PROTOCOL_VERSION;
	ret = TransferData(Wbuff, 1, Rbuff, 3, 1000);
	memcpy(ptl.ptl_ver, Rbuff, 3);
	ptl.ver = (Rbuff[0] << 16) + (Rbuff[1] << 8) + Rbuff[2];

	LD_DBG("[Protocol Version]: %x.%x.%x\n",
		ptl.ptl_ver[0], ptl.ptl_ver[1], ptl.ptl_ver[2]);

	return ret;
}

int GetProtocol_in_BL()
{
	int ret = 0;
	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_GET_PROTOCOL_VERSION;

	if (inProtocolStyle == _Protocol_V3_) {
    		ret = TransferData(Wbuff, 1, Rbuff, 2,1000);
		ptl.bl_ver = (Rbuff[0] << 8) + Rbuff[1];
	} else if (inProtocolStyle == _Protocol_V6_) {
		ret = TransferData(Wbuff, 1, Rbuff, 3,1000);
		ptl.bl_ver = (Rbuff[0] << 16) + (Rbuff[1] << 8) + Rbuff[2];
	}

	LD_MSG("[%s] BL vers: 0x%X\n", __func__, ptl.bl_ver);

	return ret;
}

int GetCRC_V3()
{
	int error;
	uint8_t Wbuff[64], Rbuff[64];

	Wbuff[0] = ILITEK_TP_GET_AP_CRC;
	if (ptl.ic == 0x2312 || ptl.ic == 0x2315) {
		if (inConnectStyle == _ConnectStyle_I2C_) {
			error = TransferData(Wbuff, 1, NULL, 0, 5000);
			usleep(100000);
			error = TransferData(NULL, 0, Rbuff, 4, 5000);
		} else {
			error = TransferData(Wbuff, 1, Rbuff, 4, 5000);
		}

		ptl.ap_crc_v3 = get_le16(Rbuff + 2) << 16 | get_le16(Rbuff);
	} else {
		if (inConnectStyle == _ConnectStyle_I2C_) {
			error = TransferData(Wbuff, 1, NULL, 0, 5000);
			usleep(100000);
			error = TransferData(NULL, 0, Rbuff, 2, 5000);
		} else {
			error = TransferData(Wbuff, 1, Rbuff, 2, 5000);
		}
		ptl.ap_crc_v3 = get_le16(Rbuff);
	}

	Wbuff[0] = ILITEK_TP_GET_DATA_CRC;
	error = TransferData(Wbuff, 1, Rbuff, 4, 1000);
	ptl.df_crc_v3 = get_le16(Rbuff + 2) << 16 | get_le16(Rbuff);

	LD_DBG("[Check Code] AP: 0x%X, Data: 0x%X\n",
		ptl.ap_crc_v3, ptl.df_crc_v3);

	return error;
}

int PanelInfor_V3()
{
	int ret;
	uint8_t Wbuff[64], Rbuff[64];

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_GET_RESOLUTION;
	ret = TransferData(Wbuff, 1, Rbuff, 15, 1000);
	ptl.x_max = ((unsigned int)Rbuff[1]) * 256 + Rbuff[0];
	ptl.y_max = ((unsigned int)Rbuff[3]) * 256 + Rbuff[2];
	ptl.x_ch = Rbuff[4];
	ptl.y_ch = Rbuff[5];

	ptl.point_max = Rbuff[6];
	ptl.key_num = Rbuff[8];

	LD_DBG("%s, max_x=%u, max_y=%u, xch=%u, ych=%u, Key Number:%u, ret=%u\n", __func__, ptl.x_max, ptl.y_max, ptl.x_ch, ptl.y_ch, ptl.key_num, ret);
	if (ptl.key_num > 0) {
		if (Rbuff[10] == 0xFF && Rbuff[11] == 0xFF && Rbuff[12] == 0xFF && Rbuff[13])
			ptl.key_mode = ILITEK_HW_KEY_MODE;
		LD_DBG("%s, key mode:%d\n", __func__, ptl.key_mode);
	}
	return ret;
}

int GetKeyInfor_V3(int key_num) {
	int ret = 0;
	uint8_t Wbuff[64] = {0}, *Rbuff;

	int r_len = key_num * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V3_HEADER;
	Rbuff = (uint8_t *)calloc(r_len, sizeof(uint8_t));
	Wbuff[0] = (unsigned char)ILITEK_TP_CMD_GET_KEY_INFORMATION;
	ret = TransferData(Wbuff, 1, Rbuff, r_len, 1000);
	free(Rbuff);
	return ret;
}

int GetFWMode()
{
	int ret;
	uint8_t Wbuff[64], Rbuff[64];

	Wbuff[0]=(uint8_t)ILITEK_TP_CMD_SWITCH_MODE;
	ret = TransferData(Wbuff, 1, Rbuff, 3, 1000);
	ptl.fw_mode = Rbuff[2];

	LD_DBG("[FW Mode] %#02X\n", ptl.fw_mode);

	if (ptl.fw_mode == 0xFF)
		LD_ERR("FW Mode: no support\n");

	return (ret < 0) ? ret : 0;
}

int GetICMode(int ic_num)
{
	int i, ret;
	uint8_t Wbuff[64], Rbuff[64];

	for (i = 0; i < 32; i++) {
		memset(ptl.ic_mode_str[i], 0, sizeof(ptl.ic_mode_str[i]));
		sprintf(ptl.ic_mode_str[i], "UNKNOWN");
	}

	if (ic_num > 32) {
		LD_ERR("[%s] unexpected IC number: %d\n", __func__, ic_num);
		return -EINVAL;
	}

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_READ_OP_MODE;

	ret = TransferData(Wbuff, 1, Rbuff, 2 * ic_num, 1000);

	ptl.ic_mode[0] = Rbuff[0];
	LD_DBG("[Current Mode] Master: %#X\n", ptl.ic_mode[0]);

	if (ptl.ic_mode[0] == OP_MODE_APPLICATION)
		sprintf(ptl.ic_mode_str[0], "AP");
	else if (ptl.ic_mode[0] == OP_MODE_BOOTLOADER)
		sprintf(ptl.ic_mode_str[0], "BL");

	for (i = 1; i < ic_num; i++) {
		ptl.ic_mode[i] = Rbuff[i * 2];
		LD_DBG("[Current Mode] Slave[%d]: %#hhX\n", i, ptl.ic_mode[i]);

		if (ptl.ic_mode[0] == OP_MODE_APPLICATION)
			sprintf(ptl.ic_mode_str[i], "AP");
		else if (ptl.ic_mode[0] == OP_MODE_BOOTLOADER)
			sprintf(ptl.ic_mode_str[i], "BL");
	}

	return (ret < 0) ? ret : 0;
}

int SetProgramKey(unsigned int start, unsigned int end)
{
	if (inProtocolStyle == _Protocol_V3_)
		return SetProgramKey_V3();

	return SetProgramKey_V6(start, end);
}

int SetProgramKey_V3()
{
	int ret;
	uint8_t Wbuff[64] = {0};

	Wbuff[0]=(uint8_t)ILITEK_TP_CMD_WRITE_ENABLE;
	Wbuff[1]=0x5A;
	Wbuff[2]=0xA5;
	ret = TransferData(Wbuff, 3, NULL, 0, 1000);
	LD_MSG("%s, ret=%u\n", __func__, ret);

	return (ret < 0) ? ret : 0;
}

int ChangeTOBL()
{
	int ret;

	uint8_t Wbuff[64] = {0};

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_SET_BL_MODE;

	ret = TransferData(Wbuff, 1, NULL, 0, 1000);

	LD_MSG("%s, ret=%d\n", __func__, ret);

	return (ret < 0) ? ret : 0;
}

int EnterTestMode(int delay_ms)
{
	int error;
	buff[0]=0xF2;
	buff[1]=0x01;

	if ((error = TransferData(buff, 2, NULL, 0, 1000)) < 0) {
		LD_ERR("%s, EnterTestMode Failed\n", __func__);
		return error;
	}

	if (delay_ms > 0)
		usleep(delay_ms * 1000);

	return 0;
}

int ExitTestMode(int delay_ms)
{
	int error;
	buff[0]=0xF2;
	buff[1]=0x00;

	if ((error = TransferData(buff, 2, NULL, 0, 1000)) < 0) {
		LD_ERR("%s, ExitTestMode Failed\n", __func__);
		return error;
	}

	if (delay_ms > 0)
		usleep(delay_ms * 1000);

	return 0;
}

int ChangeTOAP()
{
	int ret;
	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	Wbuff[0]=(uint8_t)ILITEK_TP_CMD_SET_AP_MODE;
	ret = TransferData(Wbuff, 1, Rbuff, 0, 1000);
	LD_MSG("%s, ret=%d\n", __func__, ret);

	return (ret < 0) ? ret : 0;
}

int GetKernelVer()
{
	int ret;
	uint8_t Wbuff[64], Rbuff[64];

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_GET_MCU_KERNEL_VER;
	ret = TransferData(Wbuff, 1, Rbuff, 32, 1000);
	memcpy(ptl.ic_ver, Rbuff, 5);
	ptl.ic = Rbuff[0] + (Rbuff[1] << 8);
	ptl.flash_data_start = Rbuff[2] + (Rbuff[3] << 8) + (Rbuff[4] << 16);

	LD_DBG("[Kernel Version] %.2X.%.2X.%.2X.%.2X.%.2X, IC: %#04X\n",
		ptl.ic_ver[0], ptl.ic_ver[1], ptl.ic_ver[2],
		ptl.ic_ver[3], ptl.ic_ver[4], ptl.ic);

	memset(ptl.module_name, 0, sizeof(ptl.module_name));
	memcpy(ptl.module_name, Rbuff + 6, 26);

	return ret;
}

int GetKernelVer_in_BL()
{
	int ret;
	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	Wbuff[0]=(uint8_t)ILITEK_TP_CMD_GET_MCU_KERNEL_VER;
    	ret = TransferData(Wbuff, 1, Rbuff, 6, 1000);

	ptl.ic = Rbuff[0] + (Rbuff[1] << 8);
	upg.df_start_addr = (Rbuff[2] << 16) + (Rbuff[3] << 8) + Rbuff[4];

	return ret;
}

int WriteDataFlashKey(unsigned int df_end_addr, unsigned int df_check)
{
	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_WRITE_ENABLE;//0xC4
	Wbuff[1] = 0x5A;
	Wbuff[2] = 0xA5;
	Wbuff[3] = 0x01;//data flash
	Wbuff[4] = df_end_addr >> 16;
	Wbuff[5] = (df_end_addr >> 8) & 0xFF;
	Wbuff[6] = (df_end_addr) & 0xFF;
	Wbuff[7] = df_check >> 16;
	Wbuff[8] = (df_check >> 8) & 0xFF;
	Wbuff[9] = df_check & 0xFF;

	return TransferData(Wbuff, 10, Rbuff, 0, 1000);
}

int WriteAPCodeKey(unsigned int ap_end_addr,unsigned int ap_check)
{
	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_WRITE_ENABLE;//0xC4
	Wbuff[1] = 0x5A;
	Wbuff[2] = 0xA5;
	Wbuff[3] = 0x00;//AP Code
	Wbuff[4] = ap_end_addr >> 16;
	Wbuff[5] = (ap_end_addr >> 8) & 0xFF;
	Wbuff[6] = (ap_end_addr) & 0xFF;
	Wbuff[7] = ap_check >> 16;
	Wbuff[8] = (ap_check >> 8) & 0xFF;
	Wbuff[9] = ap_check & 0xFF;

	return TransferData(Wbuff, 10, Rbuff, 0, 1000);
}

//M3 + M2V
int WriteAPCodeKey_M3_M2V(unsigned int ap_end_addr, unsigned int ap_check)
{
	int ret = -1;

	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	Wbuff[0] = 0xcb;
	Wbuff[1] = 0x80;
	Wbuff[2] = 0xc4;
	Wbuff[3] = ap_end_addr >> 16;
	Wbuff[4] = (ap_end_addr >> 8) & 0xFF;
	Wbuff[5] = (ap_end_addr) & 0xFF;
	Wbuff[6] = ap_check >> 16;
	Wbuff[7] = (ap_check >> 8) & 0xFF;
	Wbuff[8] = ap_check & 0xFF;
	ret = TransferData(Wbuff, 9, Rbuff, 0, 1000);

	LD_MSG("%s, ret = %u\n", __func__, ret);

	if(ret < 0) {
		return -1;
	}

	return 0;
}
//

int EraseDataFlash()
{
	int error;
	uint8_t Wbuff[64];

	memset(Wbuff, 0xFF, sizeof(Wbuff));

	if (ptl.ptl_ver[0] == 0x01 && ptl.ptl_ver[1] == 0x07) {
		memset(Wbuff, 0xFF, 33);
		error = WriteDataFlashKey(0x1F01F, 0xFF * 32);
		usleep(10000);
		Wbuff[0] = (unsigned char)ILITEK_TP_CMD_WRITE_DATA;
		error = TransferData(Wbuff, 33, NULL, 0, 1000);
	} else {
		error = SetProgramKey(0, 0);
		usleep(100000);
		Wbuff[0] = 0x63;
		Wbuff[1] = 0x02;
		error = TransferData(Wbuff, 2, NULL, 0, 1000);
	}

	LD_MSG("%s, ret=%u\n", __func__, error);

	return (error < 0) ? error : 0;
}

int CheckBusy(int count, int delay, int type)
{
	if (inProtocolStyle == _Protocol_V3_)
		return CheckBusy_3X(count, delay);

	return CheckBusy_6X(count, delay, type);
}

int CheckBusy_3X(int count, int delay)
{
	int busyState=0;
	uint8_t Wbuff[64], Rbuff[64];

	do {
		Wbuff[0] = 0x80;
		TransferData(Wbuff, 1, Rbuff, 1, 1000);
		busyState=Rbuff[0];
		if (busyState != 0x50)
			usleep(delay * 1000);
		count--;
	} while(count > 0 && busyState != 0x50);
	if (busyState == 0x50)
		return 0;

	LD_MSG("%s, FW is busy, ret=%u\n", __func__, busyState);
	return -EFAULT;
}

unsigned int GetCodeCheckSum(uint8_t ucType)
{
	unsigned int uiCheckSum=0;
	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	Wbuff[0]=ILITEK_TP_GET_AP_CRC;
	if(ucType!=0)
	{
		TransferData(Wbuff, 1, Rbuff, 1, 1000);
		uiCheckSum=Rbuff[0];
	}
	else
	{
		TransferData(Wbuff, 1, Rbuff, 4, 1000);
		uiCheckSum=Rbuff[0]+(Rbuff[1] * 256)+(Rbuff[2] * 256 * 256)+(Rbuff[3] * 256 *256 * 256);
	}
	//LD_MSG("%s, Check Sum=%u,ret=%u\n", __func__, uiCheckSum,ret);
	return uiCheckSum;
}


