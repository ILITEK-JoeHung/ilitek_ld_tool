/*
 * Copyright (c) 2019 ILI Technology Corp.
 *
 * This file is part of ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 */
#ifndef _ILITEK_UPGRADE_H_
#define _ILITEK_UPGRADE_H_
#include <stdbool.h>
/* Includes of headers ------------------------------------------------------*/
/* Extern define ------------------------------------------------------------*/
#define UPGRAD_FILE_PATH_SIZE               256
#define HEX_FWVERSION_ADDRESS               0x0C
#define HEX_FWVERSION_SIZE                  8
#define HEX_DATA_FLASH_ADDRESS              0x22
#define HEX_DATA_FLASH_SIZE                 3
#define HEX_KERNEL_VERSION_ADDRESS          0x6
#define HEX_KERNEL_VERSION_SIZE             6
#define HEX_MEMONY_MAPPING_VERSION_SIZE     3
#define HEX_MEMONY_MAPPING_VERSION_ADDRESS  0x0
#define HEX_FLASH_BLOCK_NUMMER_ADDRESS      80
#define HEX_FLASH_BLOCK_NUMMER_SIZE         1
#define HEX_FLASH_BLOCK_INFO_ADDRESS        84
#define HEX_FLASH_BLOCK_INFO_SIZE           3
#define HEX_FLASH_BLOCK_END_ADDRESS         123
#define HEX_MEMONY_MAPPING_FLASH_SIZE       128
#define UPGRADE_LENGTH_BLV1_8               1024    //usb 4096 have transfer issue.
#define CRC_CALCULATION_FROM_IC             1
#define CRC_GET_FROM_FLASH                  0
#define NEED_UPGRADE_FW                     1
#define NO_NEED_UPGRADE_FW                  0
#define LEGO_AP_START_ADDRESS			0x3000

/* Extern typedef -----------------------------------------------------------*/

struct BLOCK_DATA {
	unsigned int start;
	unsigned int end;
	unsigned short ic_crc;
	unsigned short dae_crc;
	bool chk_crc;               //false: ic and daemon are different.

	/* Wifi used for calculating progress bar */
	unsigned int offset;
};

struct UPGRADE_DATA {
	unsigned char *filename;
	unsigned int ap_start_addr;
	unsigned int df_start_addr;
	unsigned int exaddr;
	unsigned int ap_end_addr;
	unsigned int df_end_addr;
	unsigned int ap_check;
	unsigned int df_check;
	unsigned int total_check;
	unsigned char hex_fw_ver[HEX_FWVERSION_SIZE];

	bool args_fw_ver_check;
	unsigned char args_fw_ver[HEX_FWVERSION_SIZE];
	int args_len;

	unsigned char hex_ic_type[HEX_KERNEL_VERSION_SIZE];
	bool hex_info_flag;
	bool df_tag_exist;
	unsigned int map_ver;
	unsigned int blk_num;
	struct BLOCK_DATA blk[10];

	/* Wifi used for calculating progress bar */
	unsigned int progress_curr;
	unsigned int progress_max;
	uint8_t progress;

	bool force_update;
	bool fw_check_only;
	
	//M3 + M2V
	bool hex_m2v;
	//
};
extern struct UPGRADE_DATA upg;

/* Extern macro -------------------------------------------------------------*/
/* Extern variables ---------------------------------------------------------*/
/* Extern function prototypes -----------------------------------------------*/
/* Extern functions ---------------------------------------------------------*/

int viCheckFWNeedUpgrade(unsigned char *cFWVersion);
int viRunFiremwareUpgrade(char *filename);

int FiremwareUpgrade(unsigned char *filename);
int UpgradeFirmware_Pro1_8(unsigned char *filename);
int UpgradeFirmware_Pro1_7(unsigned char *filename);
int UpgradeFirmware_Pro1_6(unsigned char *filename);

//M3 + M2V
int hex_file_convert(unsigned char *pbuf, unsigned char *buffer, unsigned char *m2v_buffer, unsigned int hexfilesize);
//

void hex_mapping_convert(unsigned int addr, unsigned char *buffer);

unsigned int get_dri_checksum(unsigned int startAddr, unsigned int endAddr, unsigned char *input);

#endif

