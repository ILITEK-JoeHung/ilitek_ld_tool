/*
 * Copyright (c) 2019 ILI Technology Corp.
 *
 * This file is part of ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 */
#ifndef INC_ILITEK_DEVICE_H_
#define INC_ILITEK_DEVICE_H_
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <arpa/inet.h>
#include <linux/netlink.h>
#include <libgen.h>

#define _ARRAY_SIZE(arr)		(sizeof(arr) / sizeof(arr[0]))

#ifdef CONFIG_ILITEK_USE_LIBUSB
#ifdef USE_ANDROID
#include "libusb-0.1.12/usb.h"
#else
#include <usb.h>
#endif
#endif

#ifdef USE_ANDROID
#define TOOL_VERSION "ILITEK ANDROID STUDIO V2.0.2.0"
#else
#define TOOL_VERSION "ILITEK LINUX DAEMON V3.0.9.2"
#endif

#define _APMode_  0x01
#define _BLMode_  0x02
#define _MaxChanelNum_          300
#define _UpdatePageDelayTime_   10000
#define _UpdateDelayTime_       5000

#define TRANSFER_MAX_BUFFER     5000

#define REPORT_ID_64_BYTE        0x0203
#define REPORT_ID_256_BYTE       0x0307
#define REPORT_ID_1024_BYTE      0x0308
#define REPORT_ID_2048_BYTE      0x0309
#define REPORT_ID_4096_BYTE      0x030A
#define TRANS_USB_256_BYTE       256+1

#define EDID_SYS_PATH		"/sys/class/drm"
#define EDID_LENGTH		0x80

#define PEN_MODES				\
	X(PEN_WGP, 	1,	0x1, "WGP")	\
	X(PEN_USI, 	2,	0x2, "USI")	\
	X(PEN_MPP, 	3,	0x4, "MPP")

#define X(_enum, _id, _code, _name) _enum = _id,
enum PEN_MODE_ITEM {
	PEN_MODES
	PEN_MODE_ITEM_SIZE,
};
#undef X

#define UNUSED(x) (void)(x)

extern int inConnectStyle;
extern int inProtocolStyle;
extern int is_usb_hid_old_bl;
extern int ILITEK_PID, ILITEK_VID, OTHER_VID;

struct cmd_option {
	int dbg_level;

	bool check_vendor_define;

	bool use_log_file;
	FILE *log_file;

	bool no_sw_reset;

	bool INT_ack_flag_assigned;
	bool support_INT_ack;

	struct {
		char id[64];
		int busnum;
		int devnum;
	} usb;

	bool hidraw_select_dev;
	int hidraw_id;

	bool usb_hidraw;

	char i2c_dev_path[64];
	unsigned int i2c_addr;
	bool i2c_devnode;

	char save_path[256];
	char log_path[256];

	struct {
		bool to_BL;
		bool to_AP;
		unsigned int start_addr;
	} test;

	char fwid_map_file[512];

	int INT_skip_time_ms;
};

extern struct cmd_option cmd_opt;

struct edid_block {
   uint8_t  header[8];               // EDID header "00 FF FF FF FF FF FF 00"
   uint16_t manufacturerCode;        // EISA 3-character ID
   uint16_t productCode;             // Vendor assigned code
   uint32_t serialNumber;            // Serial number
   uint8_t  manufacturedWeek;        // Week number
   uint8_t  manufacturedYear;        // Year number + 1990
   uint8_t  version;                 // EDID version
   uint8_t  revision;                // EDID revision
   uint8_t  videoInputDefinition;
   uint8_t  maxHorizontalImageSize;  // in cm
   uint8_t  maxVerticalImageSize;    // in cm
   uint8_t  displayGamma;            // gamma
   uint8_t  dpmSupport;              // DPMS
   uint8_t  redGreenLowBits;         // Rx1 Rx0 Ry1 Ry0 Gx1 Gx0 Gy1Gy0
   uint8_t  blueWhiteLowBits;        // Bx1 Bx0 By1 By0 Wx1 Wx0 Wy1 Wy0
   uint8_t  redX;                    // Red-x Bits 9 - 2
   uint8_t  redY;                    // Red-y Bits 9 - 2
   uint8_t  greenX;                  // Green-x Bits 9 - 2
   uint8_t  greenY;                  // Green-y Bits 9 - 2
   uint8_t  blueX;                   // Blue-x Bits 9 - 2
   uint8_t  blueY;                   // Blue-y Bits 9 - 2
   uint8_t  whiteX;                  // White-x Bits 9 - 2
   uint8_t  whiteY;                  // White-x Bits 9 - 2
   uint8_t  establishedTimings[3];
   uint8_t  standardTimings[16];
   uint8_t  descriptionBlock1[18];
   uint8_t  descriptionBlock2[18];
   uint8_t  descriptionBlock3[18];
   uint8_t  descriptionBlock4[18];
   uint8_t  extensions;              // Number of (optional) 128-byte EDID extension blocks
   uint8_t  checksum;
} __attribute__((packed));


#define get_min(a, b)	(((a) > (b)) ? (b) : (a))
#define get_max(a, b)	(((a) < (b)) ? (b) : (a))

struct Netlink_Handle {
	int fd;
	int data_size;
	struct sockaddr_nl src_addr;
	struct sockaddr_nl dest_addr;
	struct nlmsghdr *nlh;
	struct msghdr msg;
	struct iovec iov;
}; 

extern uint16_t get_le16(const uint8_t *p);
extern int netlink_connect(struct Netlink_Handle *nl, const char *str,
			   uint32_t size, uint32_t timeout_ms);
extern void netlink_disconnect(struct Netlink_Handle *nl, const char *str);
extern int netlink_recv(struct Netlink_Handle *nl, char *buf);

extern unsigned char buff[TRANSFER_MAX_BUFFER];
extern unsigned char tempbuff[256];

extern short int uiTestDatas[_MaxChanelNum_][_MaxChanelNum_];
extern short int uiTestDatas_1[_MaxChanelNum_][_MaxChanelNum_];

extern unsigned char ucSignedDatas;
//----------------------------------------------------------------
extern unsigned int hex_2_dec(char *hex, int len);
unsigned int UpdateCRC(unsigned int crc,unsigned char newbyte);
extern unsigned int CheckFWCRC(unsigned int startAddr,unsigned int endAddr,unsigned char input[]);
extern int TransferData_HID(uint8_t *OutBuff, int writelen, uint8_t *InBuff, int readlen, int timeout_ms);
extern int TransferData(uint8_t *OutBuff, int writelen, uint8_t *InBuff, int readlen, int timeout_ms);
//----------------------------------------------------------------
extern int SetConnectStyle(char *argv[]);
extern int InitDevice();
extern void CloseDevice();
extern int write_data(int fd, unsigned char *buf, int len);
extern int read_data(int fd, unsigned char *buf, int len);
extern int switch_irq(int flag);
extern void viDriverCtrlReset();
extern int viWaitAck(uint8_t cmd, uint8_t *buf, int timeout_ms,
		     bool check_validity);
extern void i2c_read_data_enable(bool enable);
extern int hidraw_read(int fd, uint8_t *buf, int len, int timeout_ms,
		       uint8_t cmd, bool check_validity, bool check_ack);
extern void debug_print_buf(const char *str, uint8_t *buf, int len, bool force);
extern int read_report(char *buf, int len, int t_ms, struct Netlink_Handle *nl);

extern void init_INT();
extern bool wait_INT(int timeout_ms);
extern uint32_t get_driver_ver();

extern int write_and_wait_ack(uint8_t *Wbuff, int wlen, int timeout_ms, int cnt, int delay_ms, int type);

extern int set_engineer(bool enable);

extern FILE *log_openfile(char *log_dirname, char *prefix);
extern void log_closefile(FILE *file);

extern unsigned long getTimeMs(void);

extern int get_edid(struct edid_block *edid, char *str, int str_size);
extern void list_ilitek_usb_ports(uint16_t special_vid);

#endif
