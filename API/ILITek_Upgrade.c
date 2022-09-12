/*
 * Copyright (c) 2019 ILI Technology Corp.
 *
 * This file is part of ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 */
#ifndef _ILITEK_UPGRADE_C_
#define _ILITEK_UPGRADE_C_

/* Includes of headers ------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "../ILITek_CMDDefine.h"
#include "../ILITek_Device.h"
#include "ILITek_Upgrade.h"
#include "../ILITek_Protocol.h"
#include "../ILITek_Main.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define HEX_TYPE_DATA			0x00
#define HEX_TYPE_EOF			0x01	// End Of File
#define HEX_TYPE_ESA			0x02	// Extended Segment Address
#define HEX_TYPE_SSA			0x03	// Start Segment Address
#define HEX_TYPE_ELA			0x04	// Extended Linear Address
#define HEX_TYPE_SLA			0x05	// Start Linear Address
#define HEX_TYPE_ILI_MEM_MAP		0xAC	// ILI memory mapping
#define HEX_TYPE_ILI_SDA		0xAD	// ILI separate data from AP
#define HEX_START_CODE_LEN		1		// Start code lengh
#define HEX_BYTE_CNT_LEN		2		// Byte count lengh
#define HEX_ADDR_LEN			4		// Address length
#define HEX_RECORD_TYPE_LEN		2		// Record type length
#define HEX_DATA_POS_HEAD		(HEX_START_CODE_LEN+HEX_BYTE_CNT_LEN+HEX_ADDR_LEN+HEX_RECORD_TYPE_LEN)		// Start code len (1) + Byte count len (2) + Address len (4) + Record type len (2), -1 is array head is 0
#define HEX_CHKSUM_LEN			2		// Check sum length

#include<sys/time.h>
#include<unistd.h>
struct timeval tv;
struct timezone tz;
__time_t basetime;

struct UPGRADE_DATA upg;

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int viCheckFWNeedUpgrade(unsigned char *fw_ver)
{
	int i;

	for (i = 0; i < 8; i++) {
		LD_MSG("Compare FW Version: IC[%d]:%hhu, external[%d]:%hhu\n",
			i, ptl.fw_ver[i], i, fw_ver[i]);
		if (fw_ver[i] > ptl.fw_ver[i])
			return NEED_UPGRADE_FW;
	}

	return NO_NEED_UPGRADE_FW;
}

int check_ictype(char *ucTempData)
{
	unsigned char ucCount;

	if ((ptl.ic_ver[0] == 0x0 && ptl.ic_ver[1] == 0x0) ||
	    (ptl.ic_ver[0] == 0xFF && ptl.ic_ver[1] == 0xFF))
		return -EFAULT;

	for (ucCount = 0; ucCount < 2; ucCount++) {
		if (ucTempData[ucCount] != ptl.ic_ver[ucCount])
			return -EFAULT;
	}

	return 0;
}

int FW_ForceUpdate_Chk(unsigned char *buffer)
{
	int error;
	unsigned int i = 0;
	bool not_matched;

	if ((error = GetICMode(1)) < 0) {
		LD_MSG("%s, Check IC mode fail, force update!!", __func__);
		return error;
	}

	if (inProtocolStyle == _Protocol_V3_) {
		if ((ptl.ic_ver[0] == 0x12 || ptl.ic_ver[0] == 0x15) &&
		    ptl.ic_ver[1] == 0x23) {
			not_matched = (ptl.ap_crc_v3 != upg.ap_check ||
				       (upg.df_tag_exist &&
				        ptl.df_crc_v3 != upg.df_check));
		} else {
			upg.ap_check = CheckFWCRC(upg.ap_start_addr,
						  upg.ap_end_addr - 2, buffer);
			not_matched = (ptl.ap_crc_v3 != upg.ap_check ||
		    		       (upg.df_tag_exist &&
		    		        ptl.df_crc_v3 != upg.df_check));
		}

		LD_MSG("AP (IC:%#X/FW:%#X), Data (IC:%#X/FW:%#X) %s\n",
			ptl.ap_crc_v3, upg.ap_check,
			ptl.df_crc_v3, upg.df_check,
			(!not_matched) ? "matched" : "not matched");

		if (not_matched)
			error = -EFAULT;
	}

	if (ptl.ic_mode[0] == OP_MODE_BOOTLOADER) {
		LD_MSG("%s, IC in BL mode, force update!!\n", __func__);
		return -EFAULT;
	}

	if (inProtocolStyle == _Protocol_V6_) {
		ModeCtrl_V6(ENTER_SUSPEND_MODE, DISABLE_ENGINEER, 100);

		for (i = 0; i < upg.blk_num; i++) {
			if (i == 0) {
				GetCRC_V6(1);
				upg.blk[0].ic_crc = ptl.ap_crc_v6[0];
			} else {
				upg.blk[i].ic_crc = GetICBlockCrcAddr(upg.blk[i].start, upg.blk[i].end, CRC_CALCULATION_FROM_IC);
			}
			upg.blk[i].dae_crc = CheckFWCRC(upg.blk[i].start, upg.blk[i].end - 1, buffer);
			if (upg.blk[i].ic_crc == upg.blk[i].dae_crc &&
			    !upg.force_update) {
				upg.blk[i].chk_crc = true;
			} else {
				upg.blk[i].chk_crc = false;
				error = -EFAULT;
			}
			LD_MSG("Block:%d, IC CRC:%#x, FW CRC:%#x %s\n",
				i, upg.blk[i].ic_crc, upg.blk[i].dae_crc,
				(upg.blk[i].chk_crc) ? "matched" : "not matched");
		}

		for (i = 1; i < ptl.ic_num; i++) {
			LD_MSG("ILI%x CRC hex:%#x/slave[%d]:%#x, %#x mode\n",
				ptl.ic, upg.blk[0].dae_crc, i, ptl.ap_crc_v6[i],
				ptl.ic_mode[i]);

			if (ptl.ap_crc_v6[i] == upg.blk[0].dae_crc &&
			    ptl.ic_mode[i] == OP_MODE_APPLICATION)
				continue;

			error = -EFAULT;
			LD_ERR("ILI%x CRC or IC mode not match\n", ptl.ic);
		}

		ModeCtrl_V6(ENTER_NORMAL_MODE, DISABLE_ENGINEER, 100);
	}

	if (upg.force_update)
		return -EFAULT;

	return error;
}

unsigned int get_dri_checksum(unsigned int startAddr, unsigned int endAddr, unsigned char *input)
{
    unsigned int sum = 0;
    unsigned int i = 0;

    for(i = startAddr; i < endAddr; i++)
        sum += input[i];

    return sum;
}

unsigned int findstr(unsigned char *data, unsigned int start, unsigned int end, const unsigned char *str, unsigned int strLen, unsigned int bufSize)
{
	unsigned int i;

	for (i = start; i <= (end - strLen) && i < bufSize; i++) {
		if (!strncmp((const char *)(data + i), (const char *)str, strLen))
			return i;
	}

	return 0;
}

unsigned int get_block_endaddr(unsigned int startAddr, unsigned int endAddr, unsigned char *buf, unsigned int bufSize) {
	unsigned int addr;
	const unsigned char TekTag[] ="ILITek END TAG  ";
	unsigned char tag[32];

	memset(tag, 0xff, 32);
	memcpy(tag + 16, TekTag, 16);

	addr = findstr(buf, startAddr, endAddr, tag, 32, bufSize) + 32 + 1;
	if (addr == 33)
		addr = endAddr;

	return addr;
}

unsigned int get_ap_endaddr(unsigned int startAddr, unsigned int endAddr, unsigned char *buf, unsigned int bufSize) {
	unsigned int addr;
	const unsigned char TekTag[] ="ILITek AP CRC   ";
	unsigned char tag[32];

	memset(tag, 0xff, 32);
	memcpy(tag + 16, TekTag, 16);

	addr = findstr(buf, startAddr, endAddr, tag, 32, bufSize) + 32 + 1;
	if (addr == 33)
		addr = endAddr;

	return addr;
}

void update_progressInfo()
{
	unsigned int i, last_end = 0, last_offset = 0;

	upg.progress = 0;
	upg.progress_max = 0;
	upg.progress_curr = 0;

	if (inProtocolStyle == _Protocol_V6_) {
		for (i = 0; i < upg.blk_num; i++) {
			if (upg.blk[i].chk_crc == true)
				continue;

			upg.progress_max += upg.blk[i].end - upg.blk[i].start;
			last_offset += upg.blk[i].start - last_end;
			upg.blk[i].offset = last_offset;

			last_end = upg.blk[i].end;
		}
	} else if (inProtocolStyle == _Protocol_V3_) {
		if (upg.df_end_addr > upg.df_start_addr)
			upg.progress_max += upg.df_end_addr - upg.df_start_addr;
		if (upg.ap_end_addr > upg.ap_start_addr)
			upg.progress_max += upg.ap_end_addr - upg.ap_start_addr;
	}
}

int decode_hex(unsigned char *buffer, unsigned char *m2v_buffer, char *filename)
{
	int fd, error;
	unsigned char *hexbuf;
	unsigned int hexfilesize;

	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		LD_ERR("[%s] cannot open %s file\n", __func__, filename);
		return -EFAULT;
	}

	hexfilesize = get_file_size((char *)filename);
	LD_MSG("[%s] hex file size: 0x%d\n", __func__, hexfilesize);

	hexbuf = (unsigned char *)calloc(1, hexfilesize);
	if (!hexbuf) {
		LD_ERR("[%s] allocate read buffer, error\n", __func__);
		error = -EFAULT;
		goto err_close_fd;
	}

	error = read(fd, hexbuf, hexfilesize);

	if ((unsigned int)error != hexfilesize)
		LD_ERR("[%s] read failed, ret != readsize\n", __func__);
	else
		LD_MSG("[%s] read hex file to memoery, completed\n", __func__);

	hex_file_convert(hexbuf, buffer, m2v_buffer, hexfilesize);
	if (!upg.hex_info_flag) {
		LD_MSG("No support check IC type\n");
		goto err_free_buf;
	}

	if (check_ictype((char *)upg.hex_ic_type) < 0) {
		LD_ERR("IC:%x%x,Hex:%x%x\n",
			ptl.ic_ver[1], ptl.ic_ver[0],
			upg.hex_ic_type[1], upg.hex_ic_type[0]);
		LD_ERR("Hex and IC MCU version is diff\n");
		error = -EFAULT;
		goto err_free_buf;
	}

err_free_buf:
	free(hexbuf);
err_close_fd:
	close(fd);

	return (error < 0) ? error : 0;
}

int decode_bin(unsigned char *buffer, char *filename)
{
	int fd, error;
	unsigned int i, binfileSize;

	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		LD_ERR("%s, cannot open %s file\n", __func__, filename);
		return -EFAULT;
	}

	binfileSize = get_file_size(filename);
	LD_MSG("%s, bin file size, %d bytes\n", __func__, binfileSize);

	error = read(fd, buffer, binfileSize);
	if (error < 0)
		goto err_close_fd;

	if ((unsigned int)error != binfileSize)
		LD_ERR("%s, read failed, ret != readsize\n", __func__);
	else
		LD_MSG("%s, read bin file to buffer, completed\n", __func__);

	if (inProtocolStyle == _Protocol_V6_) {
		/* Needs to check Kernel Version first */
		if (ptl.flash_data_start == 0x2C000)
			hex_mapping_convert(0x4020, buffer);
		else
			hex_mapping_convert(0x3020, buffer);

		LD_MSG("------------Daemon Block information------------\n");
		for (i = 0; i < upg.blk_num; i++) {
			if (i == 0)
				upg.blk[i].end = get_ap_endaddr(
					upg.blk[i].start, upg.blk[i].end,
					buffer, binfileSize);
            		else
                		upg.blk[i].end = get_block_endaddr(
                			upg.blk[i].start, upg.blk[i].end,
                			buffer, binfileSize);
            		LD_MSG("Block: %d, start=0x%x, end=0x%x\n", i,
				upg.blk[i].start, upg.blk[i].end);
        	}
	} else if (inProtocolStyle == _Protocol_V3_) {
		if ((error = ChangeToBootloader(0, 0)) < 0 ||
		    (error = GetProtocol_in_BL()) < 0 ||
		    (error = GetKernelVer_in_BL()) < 0) {
			LD_ERR("%s, ChangeToBootloader failed\n", __func__);
			goto err_close_fd;
		}

		if (ptl.bl_ver == 0x0107) {
			LD_MSG("BL version: 1.7\n");
			hex_mapping_convert(0x2020, buffer);
			upg.ap_start_addr = 0x2000;
			upg.ap_end_addr = get_ap_endaddr(upg.ap_start_addr,
							 upg.df_start_addr,
							 buffer,
							 binfileSize) + 1;
			upg.ap_check = get_dri_checksum(upg.ap_start_addr,
							upg.ap_end_addr,
							buffer);
		} else if (ptl.bl_ver == 0x0106) {
			LD_MSG("BL version: 1.6\n");
			hex_mapping_convert(0x0500, buffer);
			upg.ap_start_addr = 0x0;
			upg.ap_end_addr = get_ap_endaddr(upg.ap_start_addr,
							 upg.df_start_addr,
							 buffer,
							 binfileSize) + 3;
			upg.ap_check = get_dri_checksum(upg.ap_start_addr,
							upg.ap_end_addr,
							buffer);
		} else {
			LD_ERR("unexpected BL version: 0x%04X\n", ptl.bl_ver);
			error = -EINVAL;
			goto err_close_fd;
		}

		/* upg.df_start_addr should update after calling GetKernelVer_in_BL() */
		upg.df_end_addr = binfileSize;
		upg.df_check = get_dri_checksum(upg.df_start_addr,
						upg.df_end_addr, buffer);

		if (upg.df_start_addr < upg.df_end_addr)
			upg.df_tag_exist = true;
		else
			upg.df_tag_exist = false;
	} else {
		LD_ERR("%s, un-supported protocol:%d\n", __func__, inProtocolStyle);
	}

	if ((error = check_ictype((char *)upg.hex_ic_type)) < 0) {
		LD_MSG("IC:%x%x,Hex:%x%x\n", ptl.ic_ver[1], ptl.ic_ver[0],
			upg.hex_ic_type[1],upg.hex_ic_type[0]);
		LD_MSG("Hex and IC MCU version is different\n");
		goto err_close_fd;
	}

err_close_fd:
	close(fd);

	return (error < 0) ? error : 0;
}

int viRunFiremwareUpgrade(char *filename)
{
	int error;
	int iUpdate = NO_NEED_UPGRADE_FW;
	unsigned char *buffer;

	//M3 + M2V
	unsigned char *m2v_buffer = NULL;
	//

	upg.ap_start_addr = 0xFFFF;
	upg.ap_end_addr = 0x0;
	upg.ap_check = 0x0;
	upg.df_start_addr = 0xFFFF;
	upg.df_end_addr = 0x0;
	upg.df_check = 0x0;
	upg.hex_info_flag = false;
	upg.df_tag_exist = false;

	//M3 + M2V
	upg.hex_m2v = false;
	//

	gettimeofday(&tv , &tz);
	basetime = tv.tv_sec;

	if ((error = viGetPanelInfor()) < 0) {
		LD_ERR("Get Panel information Failed!!!");
		return error;
	}

	buffer = (unsigned char *)calloc(1, ILITEK_DEFAULT_I2C_MAX_FIRMWARE_SIZE);
	if (!buffer) {
		LD_ERR("%s, allocate firmware buffer error\n", __func__);
		return -ENOMEM;
	}

	memset(buffer, 0xFF, ILITEK_DEFAULT_I2C_MAX_FIRMWARE_SIZE);

	//M3 + M2V
	m2v_buffer = (unsigned char *)calloc(1, ILITEK_DEFAULT_I2C_MAX_FIRMWARE_SIZE);
	if (!m2v_buffer) {
		LD_ERR("%s, allocate firmware m2v_buffer error\n", __func__);
		error = -1;
		goto err_free_buf;
	}
	//initial jbuffer(firmware buffer) and set buffer(firmware buffer) size
	memset(m2v_buffer, 0xFF, ILITEK_DEFAULT_I2C_MAX_FIRMWARE_SIZE);
	//

	if (ptl.ic == 0x2312 || ptl.ic == 0x2315)
		memset(buffer + 0x1F000, 0, 0x1000);

	if (!strcasecmp(filename + strlen(filename) - 4, ".hex")) {
		error = decode_hex(buffer, m2v_buffer, filename);
	} else if (!strcasecmp(filename + strlen(filename) - 4, ".bin")) {
		error = decode_bin(buffer, filename);
	} else {
		LD_ERR("un-recognized file extension\n");
		error = -EINVAL;
	}
	if (error < 0)
		goto err_free_buf;

	if (FW_ForceUpdate_Chk(buffer) < 0)
		iUpdate = NEED_UPGRADE_FW;

	update_progressInfo();

	if (upg.args_fw_ver_check) {
		if (upg.args_len == 8) {
			iUpdate = viCheckFWNeedUpgrade(upg.args_fw_ver);
		} else if (upg.hex_info_flag) {
			LD_MSG("[%s] Support auto compare FW ver\n", __func__);
			iUpdate = viCheckFWNeedUpgrade(upg.hex_fw_ver);
		}
	}

	if (upg.fw_check_only) {
		/*
		 * decode_bin would switch V3 IC into BL mode first, switch
		 * back to AP mode is required for fw check only option
		 */
		if (inProtocolStyle == _Protocol_V3_ &&
		    ptl.ic_mode[0] != OP_MODE_APPLICATION)
			ChangeToAPMode(0, 0);
		goto err_free_buf;
	}

	if (iUpdate == NEED_UPGRADE_FW) {
		/* start general FW upgrade flow */
		error = FiremwareUpgrade(buffer);
	} else if (inProtocolStyle == _Protocol_V3_ && !upg.df_tag_exist) {
		error = EraseDataFlash();
		LD_MSG("Message: Don't Need to Update!, Erase data flash\n");
	}

	//M3 + M2V
#ifdef Read_DualUSB_Info
	if(upg.hex_m2v == true) {
		LD_MSG("\nerror = %d, Start Upgrade M2V Firmware\n", error);
		//1. f0 3
		ModeCtrl_V6(ENTER_SUSPEND_MODE, DISABLE_ENGINEER, 100);

		//read garbage
		GetProtocol();
		GetProtocol();

		//
		uint8_t Wbuff[64] = {0}, Rbuff[64] = {0};
		Wbuff[0] = 0xcb;
		Wbuff[1] = 0x80;
		Wbuff[2] = 0xc7;
		TransferData(Wbuff, 3, Rbuff, 3, 1000);
		LD_MSG("%s, ap_check = 0x%06X, M2V ap crc = 0x%06X\n", __func__, upg.ap_check, (Rbuff[0] + (Rbuff[1] * 256) + (Rbuff[2] * 256 * 256)));
		LD_MSG("%s, force_upgrade = %s\n", __func__, (upg.force_update == true) ? "true" : "false");

		if((upg.ap_check != (Rbuff[0] + (Rbuff[1] * 256U) + (Rbuff[2] * 256U * 256U))) || upg.force_update)
		{
			//2. bl
			ChangeToBootloader_M3_M2V();

			usleep(500000);

			//3. c9 set length
			SetDataLength_V6(UPGRADE_LENGTH_BLV1_8);
			//4. c4
			if((upg.ap_end_addr % UPGRADE_LENGTH_BLV1_8) != 0) {
				upg.ap_check = upg.ap_check + ((UPGRADE_LENGTH_BLV1_8 - (upg.ap_end_addr % UPGRADE_LENGTH_BLV1_8)) * 0xff);
			}
			LD_MSG("%s, M2V ap_start_address:0x%06X, ap_end_address:0x%06X, ap_check = 0x%06X\n", __func__, upg.ap_start_addr, upg.ap_end_addr, upg.ap_check);
			WriteAPCodeKey_M3_M2V(upg.ap_end_addr, upg.ap_check);
			//5. write
			unsigned int i, k;
			for (i = upg.ap_start_addr; i < upg.ap_end_addr; i += UPGRADE_LENGTH_BLV1_8) {
				buff[0] = 0xb0;
				for (k = 0; k < UPGRADE_LENGTH_BLV1_8; k++) {
					buff[1 + k] = m2v_buffer[i + k];
				}

				//gettimeofday(&tv , &tz);
				//printf("viRunFiremwareUpgrade start time:%lu.%d\n", tv.tv_sec - basetime, tv.tv_usec);
				int r = 100;
				r = TransferData(buff, UPGRADE_LENGTH_BLV1_8 + 1, Rbuff, 1, 1000000);
				//printf("Rbuff[0] = 0x%x, r = %d\n", Rbuff[0], r);
				if(r < 0) {
					//gettimeofday(&tv , &tz);
					//printf("viRunFiremwareUpgrade start time:%lu.%d\n", tv.tv_sec - basetime, tv.tv_usec);
					LD_MSG("%s, Write Datas: Wait ACK Failed\n", __func__);
					return -1;
				}

				printf("%c",0x0D);
				LD_MSG("ILITEK: Firmware Upgrade, %02d%c. ", ((i - upg.ap_start_addr + 1) * 100) / (upg.ap_end_addr - upg.ap_start_addr), '%');

				//
				//usleep(500000);
				usleep(800000);
				Rbuff[0] = 0x0;
			}
			//6. set back ap
			ChangeToAPMode_M3_M2V();
			//7. read fw version
			GetFWVersion_M3_M2V();
		}
		else
		{
			//ret = 2;
		}

		//8. f0 0
		ModeCtrl_V6(ENTER_NORMAL_MODE, DISABLE_ENGINEER, 100);
	}
#endif

	upg.progress = 100;

err_free_buf:
	free(buffer);

	//M3 + M2V
	free(m2v_buffer);
	//

	gettimeofday(&tv , &tz);
	LD_MSG("[%s] finished, err: %d, time span: %ld.%ld sec\n",
		__func__, error, tv.tv_sec - basetime, tv.tv_usec);

	return error;
}

int FiremwareUpgrade(unsigned char *buffer)
{
	int error;

	if ((error = ChangeToBootloader(upg.blk[0].start, upg.blk[0].end)) < 0)
		return error;

	GetProtocol();

	if (ptl.ptl_ver[0] == 0x01 && ptl.ptl_ver[1] >= 0x08)
		return UpgradeFirmware_Pro1_8(buffer);
	else if (ptl.ptl_ver[0] == 0x01 && ptl.ptl_ver[1] >= 0x07)
		return UpgradeFirmware_Pro1_7(buffer);
	else if (ptl.ptl_ver[0] == 0x01 && ptl.ptl_ver[1] >= 0x04)
		return UpgradeFirmware_Pro1_6(buffer);

	LD_ERR("Not support BL protocol: %#x\n", ptl.ver);
	return -EINVAL;
}

void hex_mapping_convert(unsigned int addr,unsigned char *buffer)
{
	unsigned int hex_dfaddr_start = 0;
	unsigned int hex_icaddr_start = 0;
	unsigned int start = 0, count = 0, index = 0;

	//get fw version
	start = addr + HEX_FWVERSION_ADDRESS;
	for (count = start, index = HEX_FWVERSION_SIZE - 1; count < start + HEX_FWVERSION_SIZE; count++, --index)
		upg.hex_fw_ver[index] = buffer[count];
	LD_MSG("Hex FW version:");
	for (count = 0; count < HEX_FWVERSION_SIZE; count++)
		LD_MSG("0x%x,", upg.hex_fw_ver[count]);
	//get df start address
	LD_MSG("\nhex df start address:");
	hex_dfaddr_start = addr + HEX_DATA_FLASH_ADDRESS;
	for (count = hex_dfaddr_start, index = HEX_DATA_FLASH_SIZE - 1; count < hex_dfaddr_start + HEX_DATA_FLASH_SIZE; count++, --index)
		LD_MSG("0x%x,", buffer[count]);

	LD_MSG("\nhex ic type:");
	hex_icaddr_start = addr + HEX_KERNEL_VERSION_ADDRESS;
	for (count = hex_icaddr_start, index = 0; count < hex_icaddr_start + HEX_KERNEL_VERSION_SIZE; count++,index++) {
		LD_MSG("0x%x,", buffer[count]);
		upg.hex_ic_type[index] = buffer[count];
	}
	//get memony mapping version
	start = addr + HEX_MEMONY_MAPPING_VERSION_ADDRESS;
	for (count = start, index = 0; count < start + HEX_MEMONY_MAPPING_VERSION_SIZE; count++, index++)
		upg.map_ver += buffer[count] << (index*8);

	LD_MSG("Hex Mapping Version: 0x%x\n", upg.map_ver);
	if (upg.map_ver >= 0x10000) {
		memset(upg.blk, 0, sizeof(upg.blk));

		upg.blk_num = buffer[addr + HEX_FLASH_BLOCK_NUMMER_ADDRESS];
		LD_MSG("------------Hex Block information------------\n");
		LD_MSG("Hex flash block number: %d\n", upg.blk_num);
		for (count = 0; count < upg.blk_num; count++) {
			start = addr + HEX_FLASH_BLOCK_INFO_ADDRESS + HEX_FLASH_BLOCK_INFO_SIZE * count;
			upg.blk[count].start = buffer[start] + (buffer[start+1] << 8) + (buffer[start+2] << 16);
			if (count == upg.blk_num - 1) {
				addr = addr + HEX_FLASH_BLOCK_END_ADDRESS;
				upg.blk[count].end = buffer[addr] + (buffer[addr+1] << 8) + (buffer[addr+2] << 16);
			} else {
				upg.blk[count].end = buffer[start+3] + (buffer[start+4] << 8) + (buffer[start+5] << 16) - 1;
			}
			LD_MSG("Hex Block:%d, start:0x%x end:0x%x\n", count, upg.blk[count].start,  upg.blk[count].end);
		}
	}
}

int hex_file_convert(unsigned char *pbuf, unsigned char *buffer, unsigned char *m2v_buffer, unsigned int hexfilesize)
{
	unsigned int exaddr = 0;
	unsigned int i = 0, j = 0, k = 0;
	unsigned int start_addr = 0xFFFF, hex_info_addr = 0;
	unsigned int count = 0;
	bool read_mapping = false;
	unsigned int len = 0, addr = 0, type = 0;

	for (i = 0; i < hexfilesize;) {
		int offset;

		len = hex_2_dec((char *)&pbuf[i + 1], HEX_BYTE_CNT_LEN);
		addr = hex_2_dec((char *)&pbuf[i + 3], HEX_ADDR_LEN);
		type = hex_2_dec((char *)&pbuf[i + 7], HEX_RECORD_TYPE_LEN);

		if (type == HEX_TYPE_ELA)
			exaddr = hex_2_dec((char *)&pbuf[i + HEX_DATA_POS_HEAD], (len * 2)) << 16;
		if (type == HEX_TYPE_ESA)
			exaddr = hex_2_dec((char *)&pbuf[i + HEX_DATA_POS_HEAD], (len * 2)) << 4;
		addr = addr + exaddr;
		if (type == HEX_TYPE_ILI_MEM_MAP) {
			hex_info_addr = hex_2_dec((char *)&pbuf[i + HEX_DATA_POS_HEAD], (len * 2));
			LD_MSG("%s, hex_info_addr = 0x%x\n", __func__, hex_info_addr);
			upg.hex_info_flag = true;
		}

		//M3 + M2V
		if (type == 0xBA) {
			LD_MSG("Hex have m2v\n");
			if(hex_2_dec((char *)&pbuf[i + HEX_DATA_POS_HEAD], (len * 2)) == 2) {
				upg.hex_m2v = true;
				upg.ap_start_addr = 0xFFFF;
				upg.ap_end_addr = 0x0;
				upg.ap_check = 0x0;
				upg.df_start_addr = 0xFFFF;
				upg.df_end_addr = 0x0;
				upg.df_check = 0x0;
			}
		}
		//

		if (addr >= hex_info_addr + HEX_MEMONY_MAPPING_FLASH_SIZE && upg.hex_info_flag && read_mapping == false && upg.hex_m2v == false) {
			read_mapping = true;
			hex_mapping_convert(hex_info_addr, buffer);
		}

		/* calculate checksum */
		for (j = HEX_DATA_POS_HEAD; j < (HEX_DATA_POS_HEAD + (len * 2)); j += 2) {
			if (type == HEX_TYPE_DATA) {
				if (addr + (j - HEX_DATA_POS_HEAD) / 2 < upg.df_start_addr) {
					upg.ap_check += hex_2_dec((char *)&pbuf[i + j], 2);
 				} else {
					upg.df_check += hex_2_dec((char *)&pbuf[i + j], 2);
				}
			}
		}

		if (pbuf[i + j + 2 + 1] == 0x0A) // CR+LF (0x0D 0x0A)
			offset = 2;
		else	// CR  (0x0D)
			offset = 1;

		if (type == HEX_TYPE_DATA) {
			//for bl protocol 1.4+, ap_start_addr and ap_end_addr
			if (addr < start_addr)
				start_addr = addr;

			if (addr < upg.ap_start_addr)
				upg.ap_start_addr = addr;

			//for BL protocol 1.8
			if(upg.hex_m2v == false) {
				for (count = 0; count < upg.blk_num; count++) {
					if (addr + len - 1 > upg.blk[count].start &&
						count == upg.blk_num - 1)
						upg.blk[count].end = addr + len - 1;
					else if (addr + len - 1 > upg.blk[count].start &&
						count + 1 < _ARRAY_SIZE(upg.blk) &&
						addr + len - 1 < upg.blk[count + 1].start)
						upg.blk[count].end = addr + len - 1;

				}
			}
			//

			if ((addr + len) > upg.ap_end_addr && (addr < upg.df_start_addr))
				upg.ap_end_addr = addr + len;

			if ((addr < upg.ap_start_addr) && (addr >= upg.df_start_addr))
				upg.df_start_addr = addr;

			//for bl protocol 1.4+, bl_end_addr
			if ((addr + len) > upg.df_end_addr && (addr >= upg.df_start_addr))
				upg.df_end_addr = addr + len;

			//fill data
			for (j = 0, k = 0; j < (len * 2); j += 2, k++) {
				if(upg.hex_m2v == false) {
					buffer[addr + k] = hex_2_dec((char *)&pbuf[i + HEX_DATA_POS_HEAD + j], 2);
				}
				else {
					m2v_buffer[addr + k] = hex_2_dec((char *)&pbuf[i + HEX_DATA_POS_HEAD + j], 2);
				}
			}
		}
		if (type == HEX_TYPE_ILI_SDA) {
			upg.df_tag_exist = true;
			upg.df_start_addr = hex_2_dec((char *)&pbuf[i + HEX_DATA_POS_HEAD], (len * 2));
			LD_MSG("-----------Data Flash Start address:0x%x\n", upg.df_start_addr);
		}
		i += HEX_DATA_POS_HEAD + (len * 2) + HEX_CHKSUM_LEN + offset;
	}

	if (inProtocolStyle == _Protocol_V6_) {
#ifdef USE_ANDROID
		LD_MSG("------------APK Block information------------\n");
#else
		LD_MSG("------------Daemon Block information------------\n");
#endif
		for (count = 0; count < upg.blk_num; count++)
			LD_MSG("Block %d, start=0x%x end=0x%x\n", count, upg.blk[count].start, upg.blk[count].end);

		//M3 + M2V
		if(upg.hex_m2v == true) {
			LD_MSG("%s, M2V ap_start_address:0x%06X, ap_end_address:0x%06X, ap_check = 0x%06X\n",
				   __func__, upg.ap_start_addr, upg.ap_end_addr, upg.ap_check);
			LD_MSG("%s, M2V df_start_address:0x%06X, df_end_address:0x%06X, df_check = 0x%06X\n",
				   __func__, upg.df_start_addr, upg.df_end_addr, upg.df_check);
			LD_MSG("%s, M2V parsing hex file completed\n", __func__);
		}
		//
	} else if (inProtocolStyle == _Protocol_V3_) {
		LD_MSG("%s, ap_start_address:0x%06X, ap_end_address:0x%06X, ap_check = 0x%06X\n",
		       __func__, upg.ap_start_addr, upg.ap_end_addr, upg.ap_check);
		LD_MSG("%s, df_start_address:0x%06X, df_end_address:0x%06X, df_check = 0x%06X\n",
		       __func__, upg.df_start_addr, upg.df_end_addr, upg.df_check);
		LD_MSG("%s, parsing hex file completed\n", __func__);
	}

	return 0;
}

int UpgradeFirmware_Pro1_6(unsigned char *buffer)
{
	int error;
	unsigned int i = 0, j = 0, k = 0, DFdataChSum = 0, APdataChSum = 0;
	unsigned char iFileCheckSum = 0;
	unsigned char iRetCheckSum = 0;

	LD_MSG("ap start addr = 0x%x, ap end addr = 0x%x, ap checksum=0x%x\n",
		upg.ap_start_addr, upg.ap_end_addr, upg.ap_check);
	LD_MSG("df start addr = 0x%x, df end addr = 0x%x, df checksum=0x%x\n",
		upg.df_start_addr, upg.df_end_addr, upg.df_check);

	error = WriteDataFlashKey(upg.df_end_addr, upg.df_check);
	usleep(10000);

	for (i = upg.df_start_addr; i < upg.df_end_addr; i += 32) {
		buff[0] = (unsigned char)ILITEK_TP_CMD_WRITE_DATA;
		for (k = 0; k < 32; k++) {
			buff[1 + k] = buffer[i + k];
			iFileCheckSum += buff[1 + k];
			DFdataChSum += buff[1 + k];
		}
		error = TransferData(buff, 33, NULL, 0, 1000);

		if ((j % 16) == 0)
			usleep(50000);
		else
			usleep(3000);

		++j;
		LD_MSG("%c", 0x0D);
		LD_MSG("ILITEK: Firmware Upgrade(data code), %02d%c. ", ((i - upg.df_start_addr) * 100) / (upg.df_end_addr - upg.df_start_addr), '%');

		/* Wifi used for calculating progress bar */
		upg.progress_curr += 32;
		upg.progress_curr = get_min(upg.progress_curr, upg.progress_max);
		upg.progress = (100 * upg.progress_curr) / upg.progress_max;
	}
	LD_MSG("%s, \nupgrade firmware(data code), 100%c\n", __func__, '%');

	//write ap code
	if (((upg.ap_end_addr + 1) % 32) != 0) {
		upg.ap_check = upg.ap_check + (32 + 32 - ((upg.ap_end_addr + 1) % 32)) * 0xff;
		upg.ap_end_addr = upg.ap_end_addr + 32 + 32 - ((upg.ap_end_addr + 1) % 32);
	} else {
		upg.ap_check = (upg.ap_check + 32 * 0xff) & 0xFF;
		upg.ap_end_addr = ((upg.ap_end_addr + 32)) & 0xFF;
	}
	error = WriteAPCodeKey(upg.ap_end_addr - 1, upg.ap_check);
	LD_MSG("AP start addr = 0x%x, AP end addr = 0x%x, AP checksum=0x%x\n", upg.ap_start_addr, upg.ap_end_addr, upg.ap_check);
	usleep(30000);
	j = 0;
	for (i = upg.ap_start_addr; i < upg.ap_end_addr; i += 32) {
		buff[0] = (unsigned char)ILITEK_TP_CMD_WRITE_DATA;
		for (k = 0; k < 32; k++) {
			buff[1 + k] = buffer[i + k];
			if (i + 32 > upg.ap_end_addr && k == 31)
				buff[1 + k] = 0;

			iFileCheckSum += buff[1 + k];
			APdataChSum += buff[1 + k];
		}
		error = TransferData(buff, 33, NULL, 0, 1000);
		if ((j % (16)) == 0)
			usleep(50000);
		else
			usleep(3000);

		++j;
		LD_MSG("%c", 0x0D);
		LD_MSG("ILITEK: Firmware Upgrade(ap code), %02d%c. ", (i * 100) / upg.ap_end_addr, '%');

		/* Wifi used for calculating progress bar */
		upg.progress_curr += 32;
		upg.progress_curr = get_min(upg.progress_curr, upg.progress_max);
		upg.progress = (100 * upg.progress_curr) / upg.progress_max;
	}
	LD_MSG("%s, upgrade firmware(ap code), 100%c\n", __func__, '%');
	usleep(100000);
	iRetCheckSum =(unsigned char)(GetCodeCheckSum(1));
	LD_MSG("\n");
	LD_MSG("Read AP Write checksum =0x%x, DF Write checksum=0x%x\n", APdataChSum, DFdataChSum);
	LD_MSG("Read CheckSum Real=0x%x,Get=0x%x\n", iFileCheckSum, iRetCheckSum);
	usleep(20000);
	if ((error = ChangeToAPMode(0, 0)) < 0) {
		LD_ERR("%s, Change to ap mode failed\n", __func__);
		return error;
	}

	if (!upg.df_tag_exist)
		error = EraseDataFlash();

	LD_MSG("%s,Firmware Upgrade Success\n", __func__);
	return (error < 0) ? error : 0;
}

int UpgradeFirmware_Pro1_7(unsigned char *buffer)
{
	unsigned int get_Check;
	unsigned int i = 0, k = 0;
	int error;

	EraseDataFlash();

	upg.df_check = CheckFWCRC((upg.df_start_addr + 2), upg.df_end_addr, buffer);

	LD_MSG("ap start addr = 0x%x, ap end addr = 0x%x, ap checksum=0x%x\n",
		upg.ap_start_addr, upg.ap_end_addr, upg.ap_check);
    	LD_MSG("df start addr = 0x%x, df end addr = 0x%x, df crc=0x%x\n",
		upg.df_start_addr, upg.df_end_addr, upg.df_check);

	WriteDataFlashKey(upg.df_end_addr, upg.df_check);
	usleep(200000);
	if ((error = CheckBusy(100, 10, NO_NEED)) < 0) {
		LD_ERR("%s,WriteDataFlashKey: CheckBusy Failed\n", __func__);
		return error;
	}

	for (i = upg.df_start_addr; i < upg.df_end_addr; i += 32) {
		buff[0] = (unsigned char)ILITEK_TP_CMD_WRITE_DATA;
		for (k = 0; k < 32; k++) {

			if ((i + k) >= upg.df_end_addr)
				buff[1 + k] = 0x00;
			else
				buff[1 + k] = buffer[i + k];
		}
		TransferData(buff, 33, NULL, 0, 1000);
		usleep(5000);
		if ((error = CheckBusy(10, 10, NO_NEED)) < 0) {
			LD_ERR("%s, WriteDataFlash: CheckBusy Failed\n", __func__);
			return error;
		}

		/* Wifi used for calculating progress bar */
		upg.progress_curr += 32;
		upg.progress_curr = get_min(upg.progress_curr, upg.progress_max);
		upg.progress = (100 * upg.progress_curr) / upg.progress_max;

		//	LD_MSG("%s, upgrade firmware(data flash), %02d%c\n", __func__, (i * 100) / df_end_addr, '%');
		//usleep(10000);
	}

	LD_MSG("%s, upgrade firmware(data flash), 100%c\n", __func__, '%');
	usleep(50000);

	if (upg.df_end_addr > upg.df_start_addr) {
		buff[0] = ILITEK_TP_GET_AP_CRC;
		TransferData(buff, 1, NULL, 0, 1000);

		if ((error = CheckBusy(10, 10, NO_NEED)) < 0) {
			LD_ERR("%s, WriteDataFlas Last: CheckBusy Failed\n", __func__);
			return error;
		}
		get_Check = GetCodeCheckSum(0);
		if (upg.df_check != get_Check) {
			LD_ERR("%s, WriteDataFlas: CheckSum Failed! Real=%u,Get=%u\n", __func__, upg.df_check, get_Check);
			return -EFAULT;
		}
	}

	//write ap code
	//ILI2312/ILI2315 No support CRC check.
	if (ptl.ic_ver[1] == 0x23 && (ptl.ic_ver[0] == 0x12 || ptl.ic_ver[0] == 0x15))
		LD_ERR("not support crc\n");
	else
		upg.ap_check = CheckFWCRC(upg.ap_start_addr, upg.ap_end_addr - 2, buffer);
	WriteAPCodeKey(upg.ap_end_addr, upg.ap_check);

	LD_MSG("%s, ap_start_addr=0x%x,ap_end_addr=0x%06X,ap_check=0x%06X\n", __func__, upg.ap_start_addr, upg.ap_end_addr, upg.ap_check);
	usleep(20000);
	if ((error = CheckBusy(10, 10, NO_NEED)) < 0) {
		LD_ERR("%s, WriteAPCodeKey: CheckBusy Failed\n", __func__);
		return error;
	}

	for (i = upg.ap_start_addr; i < upg.ap_end_addr; i += 32) {
		buff[0] = (unsigned char)ILITEK_TP_CMD_WRITE_DATA;
		memcpy(buff + 1, buffer + i, 32);

		TransferData(buff, 33, NULL, 0, 1000);
		usleep(5000);
		if ((error = CheckBusy(10, 10, NO_NEED)) < 0) {
			LD_ERR("%s, WriteAPDatas: CheckBusy Failed\n", __func__);
			return error;
		}

		LD_MSG("%c", 0x0D);
		LD_MSG("ILITEK: Firmware Upgrade, %02d%c. ", (i * 100) / upg.ap_end_addr, '%');

		/* Wifi used for calculating progress bar */
		upg.progress_curr += 32;
		upg.progress_curr = get_min(upg.progress_curr, upg.progress_max);
		upg.progress = (100 * upg.progress_curr) / upg.progress_max;
	}
	LD_MSG("\n");
	LD_MSG("%s, upgrade firmware(ap code), 100%c\n", __func__, '%');

	usleep(20000);

	if (upg.ap_end_addr > upg.ap_start_addr) {
		buff[0] = 0xC7;
		TransferData(buff, 1, NULL, 0, 1000);
		if ((error = CheckBusy(10, 10,NO_NEED)) < 0) {
			LD_ERR("%s, WriteAPDatas Last: CheckBusy Failed\n", __func__);
			return error;
		}
		get_Check = GetCodeCheckSum(0);
		if (upg.ap_check != get_Check) {
			LD_ERR("%s, WriteAPData: CheckSum Failed! Real=0x%x,Get=0x%x\n", __func__, upg.ap_check, get_Check);
			return -EFAULT;
		}
	}

	if ((error = ChangeToAPMode(0, 0)) < 0) {
		LD_ERR("%s, Change to ap mode failed\n", __func__);
		return error;
	}

	usleep(100000);

	if (!upg.df_tag_exist) {
		EraseDataFlash();
	} else {
		software_reset();
		if (inConnectStyle == _ConnectStyle_USB_) {
			CloseDevice();
			usleep(5000000);
			InitDevice();
		}
	}

	/* Notify FW switch to HID-I2C report format */
	if (inConnectStyle == _ConnectStyle_I2CHID_)
		set_engineer(false);

	LD_MSG("%s,Firmware Upgrade Success\n", __func__);

	return 0;
}

int Program_Block_Pro1_8(uint8_t *buffer, int block, uint32_t len)
{
	uint16_t dae_crc = 0, ic_crc = 0;
	unsigned int i;
	int retry = 3, error;

err_retry:
	WriteFlashEnable_BL1_8(upg.blk[block].start, upg.blk[block].end);

	for (i = upg.blk[block].start; i < upg.blk[block].end; i += len) {
		buff[0] = (unsigned char)ILITEK_TP_CMD_WRITE_DATA;
		memcpy(buff + 1, buffer + i, len);

		error = write_and_wait_ack(buff, len + 1, 5000, 40, 50,
				       SYSTEM_BUSY);

		if (error < 0) {
			if (retry-- > 0)
				goto err_retry;
			return error;
		}

		/* Wifi used for calculating progress bar */
		upg.progress_curr = get_min(i + len - upg.blk[block].offset, upg.progress_max);
		upg.progress = (100 * upg.progress_curr) / upg.progress_max;

		LD_MSG("%c ILITEK: Firmware Upgrade, %02d%%.", 0x0D,
			((i - upg.blk[block].start + 1) * 100) / (upg.blk[block].end - upg.blk[block].start));
		fflush(stdout);
	}

	LD_MSG("\n[%s] upgrade firmware, 100%%\n", __func__);

	dae_crc = CheckFWCRC(upg.blk[block].start, upg.blk[block].end - 1, buffer);
	ic_crc = GetICBlockCrcAddr(upg.blk[block].start, upg.blk[block].end, CRC_GET_FROM_FLASH);
  	if (dae_crc != ic_crc) {
		LD_ERR("CRC not matched, Hex: 0x%x, IC: 0x%x\n", dae_crc, ic_crc);
		return -EFAULT;
	}

	LD_MSG("[%s] Block:%d start:0x%x end:0x%x Hex: 0x%x, IC: 0x%x\n\n",
		__func__, block, upg.blk[block].start, upg.blk[block].end,
		dae_crc, ic_crc);

	return 0;
}

int Program_Slave_Pro1_8(void)
{
	int i, retry;
	int error;

	if (ptl.ver < PROTOCOL_V6_0_0) {
		LD_ERR("It is protocol not support\n");
		return -EINVAL;
	}

	ModeCtrl_V6(ENTER_SUSPEND_MODE, ENABLE_ENGINEER, 100);

	retry = 1;
	do {
		GetCRC_V6(ptl.ic_num);

		for (i = 1; i < ptl.ic_num; i++) {
			if (ptl.ap_crc_v6[0] == ptl.ap_crc_v6[i] &&
			    !upg.force_update)
				continue;

			LD_ERR("Master CRC:%#x, IC[%d] CRC:%#x not match or force upgrade, retry: %d\n",
				ptl.ap_crc_v6[0], i, ptl.ap_crc_v6[i], retry);

			if (retry <= 0)
				return -EFAULT;

			SetAccessSlave(PROTOCOL_V6_ACCESS_SLAVE_PROGRAM);
			WriteSlaveFlashEnable_BL1_8(upg.blk[0].start,
						    upg.blk[0].end);
			break;
		}
	} while (--retry > 0);

	retry = 1;
	do {
		GetICMode(ptl.ic_num);

		for (i = 1; i < ptl.ic_num; i++) {
			if (ptl.ic_mode[i] == OP_MODE_APPLICATION)
				continue;

			LD_ERR("IC[%d] Mode:%#x not in AP, retry: %d\n",
				i, ptl.ic_mode[i], retry);

			if (retry <= 0)
				return -EFAULT;

			SetAccessSlave(PROTOCOL_V6_ACCESS_SLAVE_SET_APMODE);
			sleep(2);
			break;

		}
	} while (--retry > 0);

	if ((error = viGetPanelInfor()) < 0) {
		LD_ERR("Get Panel information Failed!!!");
		return error;
	}

	ModeCtrl_V6(ENTER_NORMAL_MODE, DISABLE_ENGINEER, 100);

	return 0;
}

int UpgradeFirmware_Pro1_8(unsigned char *buffer)
{
	int error;
	unsigned int count = 0, update_len = UPGRADE_LENGTH_BLV1_8;

	error = SetDataLength_V6(update_len);
	for (count = 0; count < upg.blk_num; count++) {
		if (upg.blk[count].chk_crc == false) {
			error = Program_Block_Pro1_8(buffer, count, update_len);
			if (error < 0) {
				LD_ERR("Upgrade Block:%d Fail\n", count);
				return error;
			}
		}
	}

	if ((error = ChangeToAPMode(upg.blk[0].start, upg.blk[0].end)) < 0) {
		LD_ERR("Change to ap mode failed\n");
		return error;
	}

	usleep(100000);

	ModeCtrl_V6(ENTER_SUSPEND_MODE, ENABLE_ENGINEER, 100);
	if ((error = viGetPanelInfor()) < 0) {
		LD_ERR("Get Panel information Failed!!!\n");
		return error;
	}
	ModeCtrl_V6(ENTER_NORMAL_MODE, DISABLE_ENGINEER, 100);

	if (ptl.ic_num > 1) {
		LD_MSG("Firmware Upgrade on Slave\n");
		if ((error = Program_Slave_Pro1_8()) < 0)
			return error;
	}
	LD_MSG("Firmware Upgrade Success\n");

	/* i2c only */
	viDriverCtrlReset();

	return error;
}
#endif

