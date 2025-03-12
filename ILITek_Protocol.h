/*
 * Copyright (c) 2019 ILI Technology Corp.
 *
 * This file is part of ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 */
#ifndef INC_ILITEK_PROTOCOL_H_
#define INC_ILITEK_PROTOCOL_H_

#define PROTOCOL_V3_4_0         0x30400
#define PROTOCOL_V6_0_0         0x60000
#define PROTOCOL_V6_0_2         0x60002
#define PROTOCOL_V6_0_4         0x60004
#define PROTOCOL_V6_0_5         0x60005
#define PROTOCOL_V6_0_6         0x60006
#define PROTOCOL_V6_0_7         0x60007


#define BL_V1_7			0x0107
#define BL_V1_6			0x0106
//------------------------------------------------
#include <stdint.h>
#include <stdbool.h>

struct ptl_data {
	unsigned int pid;

	uint8_t ic_num;
	unsigned int x_max;
	unsigned int y_max;
	uint8_t point_max;

	uint32_t ap_crc_v3, df_crc_v3;

	uint8_t fw_ver[8];
	uint8_t core_ver[7];
	uint8_t ic_mode[32];
	uint16_t ap_crc_v6[32];
	uint8_t ic_prod_info[8];
	char ic_mode_str[32][64];

	struct {
		char module_name[30];
		uint8_t ic_ver[6];
		unsigned int ic;
		unsigned int flash_data_start;
	};

	struct {
		uint8_t ptl_ver[3];
		uint32_t ver;
	};

	unsigned int x_ch;
	unsigned int y_ch;
	uint8_t mode_num;
	uint8_t report_fmt;
	uint8_t fw_mode;
	uint8_t block_num;
	unsigned int key_num;
	uint8_t key_mode;
	unsigned int bl_ver;

	uint8_t pen_modes;
	uint8_t curr_pen_mode;
	char pen_str[64];
	char dri_ver[8];
};

extern struct ptl_data ptl;


//-------------------------------------------------
extern int GetFWVersion();

//M3 + M2V
extern int GetFWVersion_M3_M2V();
//

extern int GetFWVersion_BL();
extern int GetCoreVersion();
extern int GetProtocol();
extern int GetICMode(int ic_num);
extern int ExitTestMode(int delay_ms);
extern int EnterTestMode(int delay_ms);
extern int SetProgramKey(unsigned int start, unsigned int end);
extern int ChangeTOBL();
extern int ChangeTOAP();
extern int GetKernelVer();
extern int GetKernelVer_in_BL();
extern int GetProtocol_in_BL();
extern int WriteDataFlashKey(unsigned int df_end_addr,unsigned int df_check);
extern int WriteAPCodeKey(unsigned int ap_end_addr,unsigned int ap_check);

//M3 + M2V
extern int WriteAPCodeKey_M3_M2V(unsigned int ap_end_addr,unsigned int ap_check);
//

extern int EraseDataFlash();
extern int CheckBusy(int count, int delay, int type);
extern unsigned int GetCodeCheckSum(unsigned char ucType);
extern int GetFWMode();
extern int software_reset();

//-------------------------------AP V3---------------------------------
extern int GetCRC_V3();
extern int PanelInfor_V3();
int SetProgramKey_V3();
extern int CheckBusy_3X(int count, int delay);
extern int GetKeyInfor_V3(int key_num);
//-------------------------------AP V6---------------------------------
extern int GetCRC_V6(unsigned char ic_num);
extern int PanelInfor_V6();
extern unsigned int GetICBlockCrcAddr(unsigned int start, unsigned int end, unsigned int type);
extern int SetProgramKey_V6(unsigned int start, unsigned int end);
extern int CheckBusy_6X(int count, int delay, int type);

extern int SetDataLength_V6(uint32_t data_len);
extern int GetKeyInfor_V6(int key_num);
extern int SetAccessSlave(uint8_t type);
extern int Program_Slave_Pro1_8(void);
extern int ModeCtrl_V6(uint8_t mode, uint8_t engineer, int delay_ms);
//-------------------------------BL V1.8----------------------------------
extern int WriteFlashEnable_BL1_8(unsigned int start,unsigned int end);
extern int WriteSlaveFlashEnable_BL1_8(uint32_t start,uint32_t end);

extern int GetFWID(uint16_t *fwid);
extern int GetSensorID(uint8_t *sensor_id);

#endif
