/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 *
 * The code could be used by anyone for any purpose,
 * and could perform firmware update for ILITEK's touch IC.
 */

#ifndef INC_ILITEK_CMDDEFINE_H_
#define INC_ILITEK_CMDDEFINE_H_

/* Includes of headers ------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <linux/ioctl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <math.h>
#include "ILITek_Device.h"

#ifdef CONFIG_ILITEK_USE_LIBUSB
#include <usb.h>
#endif

#include <linux/hid.h>
#include <linux/hiddev.h>
#include <linux/hidraw.h>

#include <stdint.h>
#include <sys/socket.h>
#include <linux/netlink.h>

//#include <jni.h>
#include <dirent.h>
#include <malloc.h>
#include <sys/socket.h>
#include <linux/netlink.h>

#include <math.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <linux/hidraw.h>

/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 */
#ifndef HIDIOCSFEATURE
#warning Please have your distro update the userspace kernel headers
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

/* Extern define ------------------------------------------------------------*/
#define LD_ERR(f_, ...)							 \
		do {							 \
			if (dbg_level >= LOG_LEVEL_ERR)			 \
				printf("[LD][ERR][%s][%d] "f_, __func__, __LINE__, ##__VA_ARGS__);	 \
			PRINTF_LOG("[LD][ERR][%s][%d] "f_, __func__, __LINE__, ##__VA_ARGS__);	 \
		} while (false)

#define LD_MSG(f_, ...)							\
		do {							\
			if (dbg_level >= LOG_LEVEL_MSG)			\
				printf(f_, ##__VA_ARGS__);	 	\
			PRINTF_LOG(f_, ##__VA_ARGS__);	 		\
		} while (false)

#define LD_DBG(f_, ...)							\
		do {							\
			if (dbg_level >= LOG_LEVEL_DBG)			\
				printf(f_, ##__VA_ARGS__);		\
			if (dbg_level >= LOG_LEVEL_DBG)			\
				PRINTF_LOG(f_, ##__VA_ARGS__);		\
		} while (false)


#define PRINTF_LOG(f_, ...)						\
		do {							\
			if (use_log_file && log_file)			\
				fprintf(log_file, (f_), ##__VA_ARGS__); \
		} while (false)

enum LOG_LEVEL {
	LOG_LEVEL_NONE = -1,
	LOG_LEVEL_ERR = 0,
	LOG_LEVEL_MSG,
	LOG_LEVEL_DBG,
};

#define _FAIL 		-1
#define _UNKNOWN 	-2
#define _SUCCESS 	0


//connect style
#define _ConnectStyle_I2C_      1
#define _ConnectStyle_USB_      2
#define _ConnectStyle_USBPID_   3
#define _ConnectStyle_I2CHID_	4


#define _Protocol_V3_           5
#define _Protocol_V6_           6

#define _DataFormat_8_Bit_      1
#define _DataFormat_16_Bit_     2
#define _FastMode_              3
#define _SlowMode_              4

//sensor Test default value
#define _SensorTest_Open_Threshold_V6_                  400
#define _SensorTest_Open_FrameCount_V6_                 1
#define _SensorTest_Short_Thresshold_V6_                1000
#define _SensorTestOpenTxAverDiff_V6_                   330
#define _SensorTest_Short_FrameCount_V6_                1
#define _SensorTest_Short_Dump1_V6_                     0x75
#define _SensorTest_Short_Dump2_V6_                     0x15
#define _SensorTest_Short_Vref_V6_                      0.3
#define _SensorTest_Short_posidleL_V6_                  4
#define _SensorTest_Short_posidleH_V6_                  0

#define _SensorTest_Uniformaity_Max_Threshold_V6_       10000
#define _SensorTest_Uniformaity_Min_Threshold_V6_       4000
#define _SensorTest_Uniformity_FrameCount_V6_           1
#define _SensorTest_Uniformity_Win1_Threshold_V6_       120
#define _SensorTest_Uniformity_Win1_FailCount_V6_       20
#define _SensorTest_Uniformity_Win2_Threshold_V6_       120
#define _SensorTest_Uniformity_Win2_FailCount_V6_       20
#define _SensorTest_Uniformity_RawData_Tolerance        1

//SensorTest
#define _SensorTestShortThreshold_  4
#define _SensorTestShortThreshold_V6  1000
#define _SensorTestOpenThreshold_   220
#define _SensorTestOpenDeltaRX_     120
#define _SensorTestOpenTxAverDiff_  120
#define _SensorTestOpenRXTolerance_ 100
#define _SensorTestOpenDeltaTX_     120
#define _SensorTestOpenTXTolerance_ 3

#define _2315SensorTestSelfMaximum_     4596
#define _2315SensorTestSelfMinimum_     3596
#define _2315SensorTestSelfP2P_         15
#define _2315SensorTestSelfP2PEdge_     25
#define _2315SensorTestSelfFramCnt_     100

#define _2510SensorTestSelfMaximum_     2198
#define _2510SensorTestSelfMinimum_     1898
#define _2510SensorTestSelfP2P_         15
#define _2510SensorTestSelfP2PEdge_     25
#define _2510SensorTestSelfFramCnt_     100

#define _SensorTestDACSPMax_ 80
#define _SensorTestDACSPMin_ -80
#define _SensorTestDACSNMax_ 80
#define _SensorTestDACSNMin_ -80
#define _SensorTestDACMPMax_ 80
#define _SensorTestDACMPMin_ -80
#define _SensorTestDACMNMax_ 80
#define _SensorTestDACMNMin_ -80

#define _2315SensorTestAllNodeDeltaThreshold_   30
#define _2315SensorTestAllNodePanelTolerance_   20
#define _2315SensorTestAllNodeTXTolerance_      3
#define _2315SensorTestAllNodeMaximum_          8372
#define _2315SensorTestAllNodeMinimum_          8072

#define _2510SensorTestAllNodeDeltaThreshold_   450
#define _2510SensorTestAllNodePanelTolerance_   20
#define _2510SensorTestAllNodeTXTolerance_      3
#define _2510SensorTestAllNodeMaximum_          4200
#define _2510SensorTestAllNodeMinimum_          4000

//SensorItem
#define MICROSHORT_TEST    		(0x1 << 0)
#define OPEN_TEST               (0x1 << 1)
#define SELF_TEST               (0x1 << 2)
#define DAC_TEST                (0x1 << 3)
#define ALL_NODE_TEST         	(0x1 << 4)
#define UNIFORMITY_TEST       	(0x1 << 5)
#define FWVERTION_TEST       	(0x1 << 6)
#define MIRCO_OPEN_TEST       	(0x1 << 7)

//define i2c ioctl
#define I2C_RETRIES				0x0701
#define I2C_TIMEOUT				0x0702
#define I2C_SLAVE				0x0703
#define I2C_TENBIT				0x0704
#define I2C_FUNCS				0x0705
#define I2C_SLAVE_FORCE			0x0706

//define the ioctl for communicating with driver
#define ILITEK_IOCTL_BASE                       100
#define ILITEK_IOCTL_I2C_WRITE_DATA             _IOWR(ILITEK_IOCTL_BASE, 0, unsigned char*)
#define ILITEK_IOCTL_I2C_WRITE_LENGTH           _IOWR(ILITEK_IOCTL_BASE, 1, int)
#define ILITEK_IOCTL_I2C_READ_DATA              _IOWR(ILITEK_IOCTL_BASE, 2, unsigned char*)
#define ILITEK_IOCTL_I2C_READ_LENGTH            _IOWR(ILITEK_IOCTL_BASE, 3, int)
#define ILITEK_IOCTL_DRIVER_INFORMATION		_IOWR(ILITEK_IOCTL_BASE, 8, int)
#define ILITEK_IOCTL_I2C_INT_FLAG		_IOWR(ILITEK_IOCTL_BASE, 10, int)
#define ILITEK_IOCTL_I2C_UPDATE                 _IOWR(ILITEK_IOCTL_BASE, 11, int)
#define ILITEK_IOCTL_STOP_READ_DATA             _IOWR(ILITEK_IOCTL_BASE, 12, int)
#define ILITEK_IOCTL_START_READ_DATA            _IOWR(ILITEK_IOCTL_BASE, 13, int)
#define ILITEK_IOCTL_GET_INTERFANCE		_IOWR(ILITEK_IOCTL_BASE, 14, int)
#define ILITEK_IOCTL_I2C_SWITCH_IRQ		_IOWR(ILITEK_IOCTL_BASE, 15, int)
#define ILITEK_IOCTL_UPDATE_FLAG		_IOWR(ILITEK_IOCTL_BASE, 16, int)
#define ILITEK_IOCTL_SET_RESET			_IOWR(ILITEK_IOCTL_BASE, 19, int)
#define ILITEK_IOCTL_I2C_UPDATE_FW		_IOWR(ILITEK_IOCTL_BASE, 18, int)
#define ILITEK_IOCTL_DEBUG_SWITCH		_IOWR(ILITEK_IOCTL_BASE, 21, int)
#define ILITEK_IOCTL_I2C_INT_CLR		_IOWR(ILITEK_IOCTL_BASE, 22, int)
#define ILITEK_IOCTL_I2C_INT_POLL		_IOWR(ILITEK_IOCTL_BASE, 23, unsigned char*)

//define i2c controller
#define ILITEK_DEFAULT_I2C_TIMEOUT				100
#define ILITEK_DEFAULT_I2C_RETRY				10
#define ILITEK_DEFAULT_I2C_MAX_FIRMWARE_SIZE	(256 * 1024)

//define interface
#define ACTIVE_INTERFACE_ILITEK_CTRL_I2C		0
#define ACTIVE_INTERFACE_ILITEK_CTRL_USB		1
#define ACTIVE_INTERFACE_ILITEK_CTRL_ALL		2
#define ACTIVE_INTERFACE_I2C_ADAPTER			3

//i2c command for ilitek touch screen
#define ILITEK_TP_CMD_GET_RESOLUTION			0x20
#define ILITEK_TP_CMD_GET_KEY_INFORMATION		0x22
#define ILITEK_TP_CMD_GET_RAW_DATA_INFOR		0x25
#define ILITEK_TP_CMD_GET_FIRMWARE_VERSION		0x40
#define ILITEK_TP_CMD_GET_VENDOR_VERSION		0x41
#define ILITEK_TP_CMD_GET_PROTOCOL_VERSION		0x42
#define ILITEK_TP_CMD_GET_INTERNAL_VERSION		0x43
#define ILITEK_TP_CMD_SOFTWARE_RESET			0x60
#define ILITEK_TP_CMD_GET_MCU_KERNEL_VER		0X61
#define ILITEK_TP_CMD_SWITCH_MODE			0X68
#define ILITEK_TP_CMD_SET_FS_INFO			0X69
#define ILITEK_TP_CMD_SET_SHORT_INFO			0X6A
#define ILITEK_TP_CMD_SET_OPEN_INFO			0X6D
#define ILITEK_TP_CMD_GET_SYSTEM_BUSY			0X80
#define ILITEK_TP_CMD_READ_OP_MODE			0xC0
#define ILITEK_TP_CMD_SET_AP_MODE			0xC1
#define ILITEK_TP_CMD_SET_BL_MODE			0xC2
#define ILITEK_TP_CMD_WRITE_DATA			0xC3
#define ILITEK_TP_CMD_WRITE_ENABLE			0xC4
#define ILITEK_TP_CMD_GET_FLASH				0xC5
#define ILITEK_TP_GET_AP_CRC				0xC7
#define ILITEK_TP_CMD_SET_FLASH_ADDRESS			0xC8
#define ILITEK_TP_GET_DATA_CRC				0xCA
#define ILITEK_TP_CMD_TEST_MODE_CONTROL			0xF2
#define ILITEK_TP_CMD_SWITCH_TOUCH			0xF5
#define ILITEK_TP_CMD_SET_DATA_LENGTH			0xC9
//protocol v6 command
#define ILITEK_TP_CMD_PARAMETER_V6			0X65
#define ILITEK_TP_CMD_ACCESS_SLAVE			0xCB
#define ILITEK_TP_CMD_WRITE_FLASH_ENABLE		0xCC
#define ILITEK_TP_CMD_GET_BLOCK_CRC_FOR_ADDR		0xCD
#define ILITEK_TP_CMD_GET_BLOCK_CRC_FOR_NUM		0xCF
#define ILITEK_TP_CMD_SET_MODE_CONTORL			0xF0
#define ILITEK_TP_CMD_SET_CDC_INITOAL_V6		0xF1
#define ILITEK_TP_CMD_GET_CDC_DATA_V6			0xF2

//define op mode value
#define OP_MODE_APPLICATION				0x5A
#define OP_MODE_BOOTLOADER				0x55

//define key information
#define ILITEK_KEYINFO_V3_HEADER			4
#define ILITEK_KEYINFO_V6_HEADER			5
#define ILITEK_KEYINFO_FORMAT_LENGTH			5
#define ILITEK_HW_KEY_MODE				2

//Access Slave
#define PROTOCOL_V6_ACCESS_SLAVE_SET_APMODE		0xC1
#define PROTOCOL_V6_ACCESS_SLAVE_SET_BLMODE		0xC2
#define PROTOCOL_V6_ACCESS_SLAVE_PROGRAM		0xC3
//define ILITEK USB_HID VID
#define ILITEK_VENDOR_ID				0x222A
#define OTHER_VENDOR_ID					0x04E7
#define ILITEK_BL_PRODUCT_ID				0x0010

#define RDValue_MCUKernel_CDCVersion_10bit2301_		0x06
#define RDValue_MCUKernel_CDCVersion_08bit2301_		0x07
#define RDValue_MCUKernel_CDCVersion_10bit2115_		0x08
#define RDValue_MCUKernel_CDCVersion_08bit2115_		0x09
#define RDValue_MCUKernel_CDCVersion_10bit2117_		0x0A
#define RDValue_MCUKernel_CDCVersion_8bit2315_		0x0B
#define RDValue_MCUKernel_CDCVersion_16bit2510_		0x0D

//define Mode Control value
#define ENTER_NORMAL_MODE                       0
#define ENTER_TEST_MODE                         1
#define ENTER_DEBUG_MODE                        2
#define ENTER_SUSPEND_MODE                      3  //for 2326 Suspend Scan
#define ENABLE_ENGINEER                         1
#define DISABLE_ENGINEER                        0

#define IN_ENGINEER_MODE                        1
#define OUT_ENGINEER_MODE                       0

#define SYSTEM_RETRY                            0x50
#define NO_NEED                                 0
#define SYSTEM_BUSY                             0x1
#define INITIAL_BUSY                            0x1 << 1

#define FLASH_READY                             0
#define FLASH_PREPARE                           1

#define BYTE_64                                 64
#define BYTE_256                                256
#define BYTE_1K                                 1024
#define BYTE_2K                                 2048
#define BYTE_4K                                 4096
//define tool version

/* Extern typedef -----------------------------------------------------------*/
/* Extern macro -------------------------------------------------------------*/
#define PRINTFTEST(f_, ...)					\
	do {							\
		LD_MSG("[%s][%d][Test] "f_,			\
			__func__, __LINE__, ##__VA_ARGS__);	\
	} while (false)

/* Extern variables ---------------------------------------------------------*/
extern FILE *log_file;
extern int dbg_level;
extern bool use_log_file;

/* Extern function prototypes -----------------------------------------------*/
/* Extern functions ---------------------------------------------------------*/

#endif
