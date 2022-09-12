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

int GetFWVersion_BL()
{
	int ret;

	uint8_t Wbuff[64], Rbuff[64];

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_GET_FIRMWARE_VERSION;
	ret = TransferData(Wbuff, 1, Rbuff, 8, 1000);

	memcpy(ptl.fw_ver, Rbuff, 8);
	LD_MSG("[%s] %#02X.%#02X.%#02X.%#02X.%#02X.%#02X.%#02X.%#02X\n",
		__func__, Rbuff[0], Rbuff[1], Rbuff[2], Rbuff[3],
		Rbuff[4], Rbuff[5], Rbuff[6], Rbuff[7]);

	return ret;
}

int GetCRC_V6(unsigned char ic_num)
{
	int i, error;
	uint8_t Wbuff[64], Rbuff[64];
	uint32_t CRC = 0;

	if (ic_num > 32) {
		LD_ERR("[%s] unexpected IC number: %d\n", __func__, ic_num);
		return -EINVAL;
	}

	Wbuff[0] = ILITEK_TP_GET_AP_CRC;
	error = TransferData(Wbuff, 1, Rbuff, 2 * ic_num, 1000);

	CRC = get_le16(Rbuff);
	LD_DBG("[FW CRC] Master: 0x%X\n", CRC);
	ptl.ap_crc_v6[0] = CRC;

	for (i = 1; i < ic_num; i++) {
		CRC = get_le16(Rbuff + 2 * i);
		LD_DBG("[FW CRC] Slave[%d]: 0x%X\n", i, CRC);
		ptl.ap_crc_v6[i] = CRC;
	}

	return error;
}

int PanelInfor_V6()
{
	int ret;
	uint8_t Wbuff[64], Rbuff[64];
	uint8_t i;

#define X(_enum, _id, _code, _name) {_code, _name},
	const struct {
		const int code;
		const char *str;
	} pen_modes[] = { PEN_MODES };
#undef X

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_GET_RESOLUTION;
	ret = TransferData(Wbuff, 1, Rbuff, 16, 1000);

	ptl.x_max = ((uint32_t)Rbuff[1]) * 256 + Rbuff[0];
	ptl.y_max = ((uint32_t)Rbuff[3]) * 256 + Rbuff[2];
	ptl.x_ch = ((uint32_t)Rbuff[5]) * 256 + Rbuff[4];
	ptl.y_ch = ((uint32_t)Rbuff[7]) * 256 + Rbuff[6];

	ptl.point_max = Rbuff[8];

	ptl.key_num = Rbuff[9];
	ptl.ic_num = Rbuff[10];
	ptl.mode_num = Rbuff[11];
	ptl.report_fmt = Rbuff[12];

	if (ptl.ver > PROTOCOL_V6_0_2)
		ptl.block_num = Rbuff[14];

	ptl.pen_modes = 0;
	if (ptl.ver > PROTOCOL_V6_0_6) {
		ptl.pen_modes = Rbuff[15];

		do {
			memset(ptl.pen_str, 0, sizeof(ptl.pen_str));

			if (!ptl.pen_modes) {
				strcpy(ptl.pen_str, "Disable");
				break;
			}

			for (i = 0; i < _ARRAY_SIZE(pen_modes); i++) {
				if (!(ptl.pen_modes & pen_modes[i].code))
					continue;

				sprintf(ptl.pen_str + strlen(ptl.pen_str),
					"%s,", pen_modes[i].str);
			}
		} while (false);
	}

	LD_DBG("%s, max_x=%u, max_y=%u, xch=%u, ych=%u, IC Number:%d, Key Number:%d, Block Number:%d, Support Mode:%d, ret=%d\n",
		__func__, ptl.x_max, ptl.y_max, ptl.x_ch, ptl.y_ch,
		ptl.ic_num, ptl.key_num, ptl.block_num, ptl.mode_num, ret);

	if (ptl.key_num > 0)
		ret = GetKeyInfor_V6(ptl.key_num);

	return ret;
}

int GetKeyInfor_V6(int key_num) {
	int ret = 0;
	uint8_t Wbuff[64], Rbuff[256];
	int r_len = key_num * ILITEK_KEYINFO_FORMAT_LENGTH + ILITEK_KEYINFO_V6_HEADER;

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_GET_KEY_INFORMATION;

	if (inConnectStyle == _ConnectStyle_I2C_) {
		ret = TransferData(Wbuff, 1, Rbuff, r_len, 1000);
		ptl.key_mode = Rbuff[0];
	} else {
		ret = TransferData(Wbuff, 1, Rbuff, sizeof(Rbuff), 1000);
		ptl.key_mode = Rbuff[6];
	}
	LD_DBG("%s, key mode:%d, ret=%d\n", __func__, ptl.key_mode, ret);

	return ret;
}

int SetDataLength_V6(uint32_t data_len)
{
	int ret=0;
	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_SET_DATA_LENGTH;
	Wbuff[1] = (uint8_t)(data_len & 0xFF);
	Wbuff[2] = (uint8_t)(data_len >> 8);
	ret = TransferData(Wbuff, 3, Rbuff, 0, 1000);

	return ret;
}

uint32_t GetICBlockCrcAddr(uint32_t start, uint32_t end, uint32_t type)
{
	int error;
	uint32_t crc = 0;
	uint8_t Wbuff[64], Rbuff[64];

	Wbuff[0] = ILITEK_TP_CMD_GET_BLOCK_CRC_FOR_ADDR;
	if (type) {
		Wbuff[1] = 0;
		Wbuff[2] = start;
		Wbuff[3] = (start >> 8) & 0xFF;
		Wbuff[4] = (start >> 16) & 0xFF;
		Wbuff[5] = end & 0xFF;
		Wbuff[6] = (end >> 8) & 0xFF;
		Wbuff[7] = (end >> 16) & 0xFF;

		if ((error = write_and_wait_ack(Wbuff, 8, 5000,
						50, 50, SYSTEM_BUSY)) < 0)
			return error;
	}

	Wbuff[1] = 1;
	if ((error = TransferData(Wbuff, 2, Rbuff, 2, 1000)) < 0)
		return error;

	crc = Rbuff[0]+(Rbuff[1] << 8);

	return crc;
}

int WriteFlashEnable_BL1_8(uint32_t start,uint32_t end)
{
	uint8_t Wbuff[64], Rbuff[64];

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_WRITE_FLASH_ENABLE;
	Wbuff[1] = 0x5A;
	Wbuff[2] = 0xA5;
	Wbuff[3] = start & 0xFF;
	Wbuff[4] = (start >> 8) & 0xFF;
	Wbuff[5] = start >> 16;
	Wbuff[6] = end & 0xFF;
	Wbuff[7] = (end >> 8) & 0xFF;
	Wbuff[8] = end >> 16;

	return TransferData(Wbuff, 9, Rbuff, 0, 1000);
}

int WriteSlaveFlashEnable_BL1_8(uint32_t start,uint32_t end)
{
	int error;
	uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};

	Wbuff[0] = (unsigned char)ILITEK_TP_CMD_WRITE_FLASH_ENABLE;
	Wbuff[1] = 0x5A;
	Wbuff[2] = 0xA5;
	Wbuff[3] = start & 0xFF;
	Wbuff[4] = (start >> 8) & 0xFF;
	Wbuff[5] = start >> 16;
	Wbuff[6] = end & 0xFF;
	Wbuff[7] = (end >> 8) & 0xFF;
	Wbuff[8] = end >> 16;

	LD_MSG("Please wait updating...\n");

	if (inConnectStyle == _ConnectStyle_I2C_) {
		error = TransferData(Wbuff, 9, Rbuff, 0, 1000);
		sleep(20);
	} else {
		error = write_and_wait_ack(Wbuff, 9, 20000, 10, 100,
					   SYSTEM_BUSY);
		CloseDevice();
		sleep(5);
		InitDevice();
	}

	return (error < 0) ? error : 0;
}

int SetProgramKey_V6(unsigned int start, unsigned int end)
{
	uint8_t Wbuff[64], Rbuff[64];

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_WRITE_FLASH_ENABLE;
	Wbuff[1] = 0x5A;
	Wbuff[2] = 0xA5;
	Wbuff[3] = start & 0xFF;
	Wbuff[4] = (start >> 8) & 0xFF;
	Wbuff[5] = start >> 16;
	Wbuff[6] = end & 0xFF;
	Wbuff[7] = (end >> 8) & 0xFF;
	Wbuff[8] = end >> 16;

	return TransferData(Wbuff, 9, Rbuff, 0, 1000);
}

int ModeCtrl_V6(uint8_t mode, uint8_t engineer, int delay_ms)
{
	int ret;
	uint8_t Wbuff[64], Rbuff[64];

	switch (mode) {
	case ENTER_NORMAL_MODE:
		LD_MSG("Change to Normal mode:");
		break;
	case ENTER_DEBUG_MODE:
		LD_MSG("Change to Debug mode:");
		break;
	case ENTER_SUSPEND_MODE:
		LD_MSG("Change to Suspend mode:");
		break;
	case ENTER_TEST_MODE:
		LD_MSG("Change to Test mode:");
		break;
	}

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_SET_MODE_CONTORL;
	Wbuff[1] = mode;
	Wbuff[2] = engineer;
	ret = TransferData(Wbuff, 3, Rbuff, 0, 1000);

	if (delay_ms > 0)
		usleep(delay_ms * 1000);

	if (ret < 0) {
		LD_ERR("Fail\n");
		return ret;
	}

	LD_MSG("Success\n");
	return 0;
}

int SetAccessSlave(uint8_t type)
{
	uint8_t Wbuff[64];

	Wbuff[0] = (uint8_t)ILITEK_TP_CMD_ACCESS_SLAVE;
	Wbuff[1] = 0x3;
	Wbuff[2] = type;

	return write_and_wait_ack(Wbuff, 3, 5000, 10, 100, SYSTEM_BUSY);
}

int CheckBusy_6X(int count, int delay, int type)
{
	int busyState;
	uint8_t Wbuff[64], Rbuff[64];

	do {
		Wbuff[0] = ILITEK_TP_CMD_GET_SYSTEM_BUSY;
		TransferData(Wbuff, 1, Rbuff, 1, 1000);
		busyState = Rbuff[0] & (SYSTEM_RETRY + type);
		if (busyState != SYSTEM_RETRY)
			usleep(delay * 1000);
		count--;
	} while(count > 0 && busyState != SYSTEM_RETRY);
	if (busyState == SYSTEM_RETRY)
		return 0;

	LD_ERR("%s, FW is busy, ret=%u\n", __func__, Rbuff[0]);
	return -EFAULT;
}

/* Need to get IC mode and protocol version first */
int GetFWID(uint16_t *customer_id, uint16_t *fwid)
{
	uint8_t Wbuff[64], Rbuff[64];
	int error;

	*customer_id = 0;
	*fwid = 0;

	if ((ptl.ic_mode[0] == OP_MODE_BOOTLOADER &&
		ptl.ver < 0x010802) ||
	    (ptl.ic_mode[0] == OP_MODE_APPLICATION &&
	    	ptl.ver < PROTOCOL_V6_0_7))
	    	return -EINVAL;

	Wbuff[0] = ILITEK_TP_CMD_GET_FWID;

	if ((error = TransferData(Wbuff, 1, Rbuff, 4, 1000)) < 0)
		return error;

	*customer_id = Rbuff[0] | (Rbuff[1] << 8);
	*fwid = Rbuff[2] | (Rbuff[3] << 8);

	return 0;
}

