/*
 * Copyright (c) 2019 ILI Technology Corp.
 *
 * This file is part of ILITEK Linux Daemon Tool
 *
 * Copyright (c) 2021 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (c) 2021 Joe Hung <joe_hung@ilitek.com>
 */
#include "ILITek_Protocol.h"
#include "ILITek_CMDDefine.h"
#include "ILITek_Device.h"
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>


#define get_sysfs_val(node, fmt, ...)						\
	do {									\
		FILE *fp;							\
		if (!(fp = fopen(node, "r"))) {					\
			LD_DBG("fopen: %s failed, err: %d\n", node, -errno);	\
			break;							\
		}								\
										\
		if (fscanf(fp, fmt, ##__VA_ARGS__) < 0) {			\
			LD_DBG("fscanf fmt: %s failed, err: %d\n", fmt, -errno);\
			break;							\
		}								\
		fclose(fp);							\
	} while (false)



#define CRC_POLY 0x8408      // CRC16-CCITT FCS (X^16+X^12+X^5+1)

int is_usb_hid_old_bl = 0;

//------------------------------
int inConnectStyle;
int inProtocolStyle;
//USB_HID detail
int ENDPOINT_IN = 0x81;
int ENDPOINT_OUT = 0x02;
int HID_INTERFACE_COUNT = 1;
int ILITEK_PID = 0x0,ILITEK_VID = 0x0, OTHER_VID =0x0;
unsigned char *temp_ILITEK_PID;

struct usb_dev_handle *hdev;

//I2C
int fd;

//Remote
int sockfd;
struct sockaddr_in server_addr;
int inRemote_Flag = 0;
//-----------------------------------
unsigned char buff[TRANSFER_MAX_BUFFER];
unsigned char tempbuff[256];

short int uiTestDatas[_MaxChanelNum_][_MaxChanelNum_];
short int uiTestDatas_1[_MaxChanelNum_][_MaxChanelNum_];

unsigned char ucSignedDatas=0;
//-----------------------------------

#include <stdarg.h>

unsigned int hex_2_dec(char *hex, int len)
{
	unsigned int ret = 0, temp = 0;
	int i, shift = (len - 1) * 4;

	for (i = 0; i < len; shift -= 4, i++) {
		if ((hex[i] >= '0') && (hex[i] <= '9'))
			temp = hex[i] - '0';
		else if((hex[i] >= 'a') && (hex[i] <= 'f'))
			temp = (hex[i] - 'a') + 10;
		else if((hex[i] >= 'A') && (hex[i] <= 'F'))
			temp = (hex[i] - 'A') + 10;
		else
			return -1;//return -EINVAL;
		ret |= (temp << shift);
	}
	return ret;
}


unsigned int UpdateCRC(unsigned int crc,unsigned char newbyte)
{
	unsigned char i;                                  // loop counter

	crc = crc ^ newbyte;

	for (i = 0; i < 8; i++)
	{
		if (crc & 0x01)
		{
			crc = crc >> 1;
			crc ^= CRC_POLY;
		}
		else
		{
			crc = crc >> 1;
		}
	}
	return crc;
}


unsigned int CheckFWCRC(unsigned int startAddr,unsigned int endAddr,unsigned char input[])
{
	unsigned int CRC = 0;
	unsigned int i = 0;

	for (i = startAddr; i < endAddr; i++)
		CRC = UpdateCRC(CRC, input[i]);

	return CRC;
}

int SetConnectStyle(char *argv[])
{
	if (!strcmp(argv[2], "I2C")) {
		inConnectStyle = _ConnectStyle_I2C_;
		strcpy(cmd_opt.i2c_dev_path, argv[4]);
		cmd_opt.i2c_addr = hex_2_dec(argv[5], 2);
	} else if (!strcmp(argv[2], "USB")) {
		inConnectStyle = _ConnectStyle_USB_;
		OTHER_VID = hex_2_dec(argv[5], 4);
	} else if (!strcmp(argv[2], "I2C-HID")) {
		inConnectStyle = _ConnectStyle_I2CHID_;
		OTHER_VID = hex_2_dec(argv[5], 4);
	} else {
		return -EINVAL;
	}

	if (!strcmp(argv[3], "V3"))
		inProtocolStyle = _Protocol_V3_;
	else if (!strcmp(argv[3], "V6"))
		inProtocolStyle = _Protocol_V6_;
	else
		return -EINVAL;

	return 0;
}

bool has_vendor_define(uint8_t *src, unsigned int src_size,
			 uint8_t *tag, unsigned int tag_size)
{
	unsigned int i = 0, j = 0;

	for (i = 0; i < src_size - tag_size; i++) {
		if (src[i] == tag[j]) {
			if (++j == tag_size) {
				LD_MSG("vendor define is found...\n");
				return true;
			}
			continue;
		}

		j = 0;
	}

	return false;
}

int open_hidraw_device(uint32_t bus_type, unsigned long timeout_ms)
{
	struct hidraw_devinfo dev_info;
	char dev_name[256];
	DIR *dir = NULL;
	struct dirent *ptr;
	char hidraw_path[512];
	int hidraw_id;
	int desc_size;
	struct hidraw_report_descriptor report_desc;

	uint8_t vendor_define[] = {0x9, 0x1, 0x85, 0x3};

	unsigned long t_init, t_diff;

	t_init = getTimeMs();

	do {

		dir = opendir("/dev");
		if (!dir) {
			LD_ERR("can't open \"/dev\" directory\n");
			return -ENOENT;
		}

		while ((ptr = readdir(dir))) {
			/* filter out non-character device */
			if (ptr->d_type != DT_CHR ||
			    strncmp(ptr->d_name, "hidraw", 6))
				continue;

			sscanf(ptr->d_name, "hidraw%d", &hidraw_id);
			if (cmd_opt.hidraw_select_dev &&
			    hidraw_id != cmd_opt.hidraw_id)
				continue;

			memset(hidraw_path, 0, sizeof(hidraw_path));
			snprintf(hidraw_path, sizeof(hidraw_path),
				 "/dev/%s", ptr->d_name);

			fd = open(hidraw_path, O_RDWR | O_NONBLOCK);
			if (fd < 0) {
				LD_ERR("can't open %s, fd: %d, err: %d\n",
					hidraw_path, fd, errno);
				continue;
			}

			if (cmd_opt.check_vendor_define) {
				ioctl(fd, HIDIOCGRDESCSIZE, &desc_size);
				LD_DBG("[%s] fd: %d, desc size: %d\n",
					ptr->d_name, fd, desc_size);

				memset(&report_desc, 0, sizeof(report_desc));
				report_desc.size = desc_size;

				ioctl(fd, HIDIOCGRDESC, &report_desc);
				debug_print_buf("rpt_desc: ", report_desc.value,
						report_desc.size, true);

				if (!has_vendor_define(report_desc.value, report_desc.size,
						       vendor_define, sizeof(vendor_define)))
					goto err_continue;
			}

			ioctl(fd, HIDIOCGRAWINFO, &dev_info);
			if ((dev_info.vendor != ILITEK_VENDOR_ID &&
			     dev_info.vendor != OTHER_VID)) {
				LD_DBG("Invalid vendor id: %x, should be %x or %x\n",
					dev_info.vendor, ILITEK_VENDOR_ID, OTHER_VID);
				goto err_continue;
			}

			if (dev_info.bustype != bus_type) {
				LD_DBG("invalid bus type: %u, should be %u\n",
					dev_info.bustype, bus_type);
				goto err_continue;
			}

			ioctl(fd, HIDIOCGRAWNAME(256), dev_name);

			LD_MSG("name:%s, bustype: %u, path: %s, vid: %#x, pid: %#x\n",
				dev_name, dev_info.bustype, hidraw_path,
				dev_info.vendor, dev_info.product);

			/* hidraw may be changed after FW reset or re-enumerate */
			cmd_opt.hidraw_select_dev = false;

			ptl.pid = dev_info.product;

			closedir(dir);
			return 0;

err_continue:
			close(fd);
		}

		closedir(dir);

		t_diff = getTimeMs() - t_init;

	} while ((!timeout_ms || t_diff < timeout_ms));


	LD_ERR("No ilitek hidraw file node found!\n");

	return -ENODEV;
}

void list_ilitek_usb_ports(uint16_t special_vid)
{
	DIR *dir;
	struct dirent *ptr;
	const char usb_devs[64] = "/sys/bus/usb/devices";
	char node[512];
	uint16_t vid, pid;

	if (!(dir = opendir(usb_devs))) {
		LD_ERR("can't open %s directory\n", usb_devs);
		return;
	}

	while ((ptr = readdir(dir))) {
		memset(node, 0, sizeof(node));
		sprintf(node, "%s/%s/idVendor", usb_devs, ptr->d_name);

		/* check file exists */
		if (access(node, F_OK))
			continue;

		vid = 0;
		get_sysfs_val(node, "%hx", &vid);
		if (vid != ILITEK_VENDOR_ID &&
		    vid != OTHER_VENDOR_ID &&
		    vid != special_vid)
			continue;

		memset(node, 0, sizeof(node));
		sprintf(node, "%s/%s/idProduct", usb_devs, ptr->d_name);
		get_sysfs_val(node, "%hx", &pid);

		LD_MSG("VID: 0x%04hx, PID: 0x%04hx, Port: %s\n",
			vid, pid, ptr->d_name);
	}

	closedir(dir);
}

void get_usb_bus_dev_num(char *usb_id, int *busnum, int *devnum)
{
	char cmd_str[256];

	*busnum = 0;
	*devnum = 0;

	memset(cmd_str, 0, sizeof(cmd_str));
	sprintf(cmd_str, "/sys/bus/usb/devices/%s/busnum", usb_id);
	get_sysfs_val(cmd_str, "%d", busnum);

	memset(cmd_str, 0, sizeof(cmd_str));
	sprintf(cmd_str, "/sys/bus/usb/devices/%s/devnum", usb_id);
	get_sysfs_val(cmd_str, "%d", devnum);

	LD_MSG("get busnum: %d, devnum: %d\n", *busnum, *devnum);
}

#ifdef CONFIG_ILITEK_USE_LIBUSB
#ifdef USE_ANDROID
static	struct usb_device dev;
int UsbFd;
int open_usb_hid_device()
{
	LD_MSG("[usb_dev_handle] fd: %d,inConnectStyle=%d, inProtocolStyle=%d", UsbFd, inConnectStyle, inProtocolStyle);
	struct usb_dev_handle *_hdev;
	_hdev = usb_open_fd(&dev, UsbFd);
	if (!_hdev) {
		LD_ERR("open device error\n");
		return -ENODEV;
	}


	ENDPOINT_IN = 0x81;
	ENDPOINT_OUT = 0x02;
	hdev  = _hdev;
	return 0;
}
#else
int open_usb_hid_device()
{
	struct usb_bus *busses, *bus;
	struct usb_device *dev;
	struct usb_dev_handle *_hdev;
	int busnum;

	is_usb_hid_old_bl = 0;
	usb_init();
	usb_find_busses();
	usb_find_devices();

	hdev = NULL;

	/* usb id should be a fix id string and not changed during FW Upgrade */
	if (strlen(cmd_opt.usb.id)) {
		get_usb_bus_dev_num(cmd_opt.usb.id, &cmd_opt.usb.busnum,
				    &cmd_opt.usb.devnum);
	}

	busses = usb_get_busses();
	for (bus = busses; bus; bus = bus->next) {
		for (dev = bus->devices; dev; dev = dev->next) {
			if ((dev->descriptor.idVendor != ILITEK_VENDOR_ID) &&
			    (dev->descriptor.idVendor != OTHER_VENDOR_ID) &&
			    (dev->descriptor.idVendor != OTHER_VID))
				continue;

			LD_MSG("devnum: %d, filename: %s, busnum: %s, location: %d, VID: %#x, PID: %#x\n",
				dev->devnum, dev->filename,
				bus->dirname, bus->location,
				dev->descriptor.idVendor,
				dev->descriptor.idProduct);

			sscanf(bus->dirname, "%d", &busnum);
			if (strlen(cmd_opt.usb.id) &&
			    (cmd_opt.usb.busnum != busnum ||
			     cmd_opt.usb.devnum != dev->devnum))
				continue;

			ILITEK_PID = dev->descriptor.idProduct;
			ILITEK_VID = dev->descriptor.idVendor;

			_hdev = usb_open(dev);
			if (!_hdev) {
				LD_ERR("open device error\n");
				return -ENODEV;
			}

			HID_INTERFACE_COUNT = dev->config->bNumInterfaces;
			usb_detach_kernel_driver_np(_hdev, dev->config->interface->altsetting->bInterfaceNumber);
			if (HID_INTERFACE_COUNT == 2)
				usb_detach_kernel_driver_np(_hdev, 1);

			if (usb_claim_interface(_hdev, dev->config->interface->altsetting->bInterfaceNumber) != 0) {
				LD_ERR("claim interface 0 error\n");
				usb_close(_hdev);
				return -ENODEV;
			}

			if (HID_INTERFACE_COUNT == 2) {
				if (usb_claim_interface(_hdev, 1) != 0) {
					LD_ERR("claim interface 1 error\n");
					usb_close(_hdev);
					return -ENODEV;
				}

				ENDPOINT_IN = 0x81;
				ENDPOINT_OUT = 0x82;
			} else {
				ENDPOINT_IN = 0x81;
				ENDPOINT_OUT = 0x02;
			}

			if (dev->descriptor.idProduct == ILITEK_BL_PRODUCT_ID)
				is_usb_hid_old_bl = 1;

			LD_MSG("[%s] ILITEK usb_hid[0x%04X:0x%04X] found, devnum: %u, cnt: %d\n",
				__func__, dev->descriptor.idVendor,
				dev->descriptor.idProduct, dev->devnum,
				HID_INTERFACE_COUNT);

			ptl.pid = dev->descriptor.idProduct;

			hdev = _hdev;

			return 0;
		}
	}

	LD_ERR("[%s] ILITEK usb_hid device not found\n", __func__);

	return -ENODEV;
}
#endif
#endif

void i2c_read_data_enable(bool enable)
{
	if (enable)
		ioctl(fd, ILITEK_IOCTL_START_READ_DATA, 1);
	else
		ioctl(fd, ILITEK_IOCTL_STOP_READ_DATA, 0);
}

int open_i2c_dev_node(char *dev_node, unsigned int addr)
{
	if ((fd = open(dev_node, O_RDWR)) < 0) {
		LD_ERR("open file: %s failed\n", dev_node);
		return -ENOENT;
	}

	/*
	 * Use this slave address, even if it
	 * is already in use by a driver!
	 */
	if (ioctl(fd, I2C_SLAVE_FORCE, addr) < 0) {
		LD_ERR("set address to 0x41 failed, %s\n", strerror(errno));
		return (errno) ? -errno : -EFAULT;
	}

	/* set timeout in units of 10 ms */
	if (ioctl(fd, I2C_TIMEOUT, 100) < 0) {
		LD_ERR("set i2c timeout failed, %s\n", strerror(errno));
		return (errno) ? -errno : -EFAULT;
	}

	/*
	 * number of times a device address should
	 * be polled when not acknowledging
	 */
	if (ioctl(fd, I2C_RETRIES, 100) < 0) {
		LD_ERR("set i2c retry failed, %s\n", strerror(errno));
		return (errno) ? -errno : -EFAULT;
	}

	return 0;
}

int OpenI2CDevice(char *dev_node)
{
	LD_MSG("device node is %s\n", dev_node);
#ifdef USE_ANDROID
	fd = open(dev_node, O_RDONLY);
	jni.ctrl.fd = fd;
#else
	fd = open(dev_node, O_RDWR);
#endif
	if (fd < 0) {
		LD_ERR("%s, ilitek controller doesn't exist\n", __func__);
		return -ENOENT;
	}

	i2c_read_data_enable(false);

	LD_MSG("[%s] driver_ver: %x\n", __func__, get_driver_ver());

	return 0;
}

int InitDevice()
{
	switch (inConnectStyle) {
	case _ConnectStyle_I2C_:
		if (cmd_opt.i2c_devnode)
			return open_i2c_dev_node(cmd_opt.i2c_dev_path,
						 cmd_opt.i2c_addr);
		return OpenI2CDevice(cmd_opt.i2c_dev_path);

	case _ConnectStyle_I2CHID_:
		return open_hidraw_device(BUS_I2C, 5000);

	case _ConnectStyle_USB_:
		if (cmd_opt.usb_hidraw)
			return open_hidraw_device(BUS_USB, 5000);

#ifdef CONFIG_ILITEK_USE_LIBUSB
		return open_usb_hid_device();
#endif

	default:
		break;
	}

	return -EINVAL;
}

int write_data(int fd, unsigned char *buf, int len)
{
	int ret;

	if (cmd_opt.i2c_devnode)
		return write(fd, buf, len);

	ret = ioctl(fd, ILITEK_IOCTL_I2C_WRITE_LENGTH, len);
	ret = ioctl(fd, ILITEK_IOCTL_I2C_WRITE_DATA, buf);

	return ret;
}

int read_data(int fd, unsigned char *buf, int len)
{
	int ret;

	if (cmd_opt.i2c_devnode)
		return read(fd, buf, len);

	ret = ioctl(fd, ILITEK_IOCTL_I2C_READ_LENGTH, len);
	ret = ioctl(fd, ILITEK_IOCTL_I2C_READ_DATA, buf);

	return ret;
}

int hidraw_read(int fd, uint8_t *buf, int len, int timeout_ms,
		uint8_t cmd, bool check_validity, bool check_ack)
{
	int ret = 0, t_ms = 0;

	if (!buf)
		return -EINVAL;

	do {
		ret = read(fd, buf, len);

		if ((!check_validity && ret > 0) || (ret == len &&
		     ((buf[0] == 0x03 && buf[1] == 0xA3 && buf[2] == cmd) ||
		      buf[0] == 0xAA))) {
		      if ((check_ack && buf[4] == 0xAC) || !check_ack)
				return ret;
		}

		usleep(1000);
		t_ms += 1;
	} while (t_ms < timeout_ms);

	return -ETIME;
}

int usb_read(uint8_t *buf, int len, int timeout_ms,
	     uint8_t cmd, bool check_validity, bool check_ack)
{
	int error = -EINVAL;
	unsigned long t_init, t_diff, tout_ms = timeout_ms;

	if (!buf)
		return -EINVAL;

	t_init = getTimeMs();

	UNUSED(cmd);
	UNUSED(len);

	do {
#ifdef CONFIG_ILITEK_USE_LIBUSB
		error = usb_interrupt_read(hdev, ENDPOINT_IN, (char *)buf,
					   len, timeout_ms);
#endif

		if (!check_validity || (buf[0] == 0x03 && buf[1] == 0xA3)) {
			if (!check_ack || (check_ack && buf[4] == 0xAC))
				return error;
		}

		t_diff = getTimeMs() - t_init;
	} while ((!tout_ms || t_diff < tout_ms));

	return -ETIME;
}

unsigned int getLength(unsigned int len, unsigned int *reportID)
{
	if (len <= BYTE_64) {
		*reportID = REPORT_ID_64_BYTE;
		return BYTE_64;
	} else if (len <= BYTE_256) {
		*reportID = REPORT_ID_256_BYTE;
		return BYTE_256 + 1 + 6;
	} else if (len <= BYTE_1K + 1) {
		*reportID = REPORT_ID_1024_BYTE;
		return BYTE_1K + 1 + 6;
	} else if (len <= BYTE_2K + 1) {
		*reportID = REPORT_ID_2048_BYTE;
		return BYTE_2K + 1 + 6;
	}

	*reportID = REPORT_ID_4096_BYTE;

	return BYTE_4K + 1 + 6;
}

void debug_print_buf(const char *str, uint8_t *buf, int len, bool force)
{
	int i = 0;

	if (!buf || !len)
		return;

	if (force) {
		LD_MSG("%s", str);
		for (i = 0; i < len; i++)
			LD_MSG("%02X,", buf[i]);
		LD_MSG(", len: %d\n", len);

		return;
	}

	LD_DBGP("%s", str);
	for (i = 0; i < len; i++)
		LD_DBGP("%02X,", buf[i]);
	LD_DBGP(", len: %d\n", len);
}

uint16_t get_le16(const uint8_t *p)
{
	return p[0] | p[1] << 8;
}

/*
 * TransferData_HID will return HID format packet,
 * no matter which interface is selected.
 *
 * Write/Read length will be modified to aligned power of 2.
 * will return write/read length on success.
 */
int TransferData_HID(uint8_t *OutBuff, int writelen,
		     uint8_t *InBuff, int readlen, int timeout_ms)
{
	int ret = 0;
	unsigned int wlen, rlen;
	unsigned int w_report, r_report;
	uint8_t cmd;
	int __attribute__((unused)) retry = 50;

	wlen = getLength(writelen, &w_report);
	rlen = getLength(readlen, &r_report);

	if (!OutBuff)
		cmd = 0;
	else if (OutBuff[0] == 0x03)
		cmd = OutBuff[4];
	else
		cmd = OutBuff[6];

	if (writelen > 0)
		debug_print_buf("[OutBuff]:", OutBuff, wlen, false);

	if (inConnectStyle == _ConnectStyle_I2C_) {
		if (writelen > 0) {
			if (w_report == REPORT_ID_64_BYTE && OutBuff[1] == 0xA3)
				ret = write_data(fd, OutBuff + 4, writelen);
			else
				ret = write_data(fd, OutBuff + 6, writelen);
			if (ret < 0) {
				LD_ERR("[%s] I2C write fail, cmd: 0x%x, wlen: %d, ret: %d\n",
					__func__, cmd, writelen, ret);
				return ret;
			}
			usleep(1000);
		}
		if (readlen > 0) {
			if (r_report == REPORT_ID_64_BYTE &&
			    OutBuff[1] == 0xA3) {
				ret = read_data(fd, InBuff + 4, readlen);
				InBuff[0] = 0x03;
				InBuff[1] = 0xA3;
				InBuff[2] = cmd;
				InBuff[3] = readlen;
			} else {
				ret = read_data(fd, InBuff, readlen);
			}

			if (ret < 0)
				return ret;
		}
	} else if (inConnectStyle == _ConnectStyle_USB_ &&
		   !cmd_opt.usb_hidraw) {
#ifdef CONFIG_ILITEK_USE_LIBUSB
WRITE_AGAIN:
		if (writelen > 0) {
			ret = usb_control_msg(hdev, 0x21, 0x09, w_report, 0,
					      (char *)OutBuff, wlen, 10000);
			if (ret < 0) {
				if (cmd != ILITEK_TP_CMD_SOFTWARE_RESET) {
					LD_ERR("[%s] USB write fail, cmd: 0x%x, ret:%d, wlen:%d\n",
						__func__, cmd, ret, wlen);
				}
				return ret;
			}
		}

		if (readlen > 0) {
			if (r_report == REPORT_ID_64_BYTE) {
				ret = usb_read(InBuff, rlen, timeout_ms,
					       cmd, true, false);
				if (ret < 0) {
					LD_ERR("USB read failed write again, cmd: %x\n",
						cmd);
					if (--retry > 0)
						goto WRITE_AGAIN;
				}
			} else {
				ret = usb_control_msg(hdev, 0xA1, 0x01,
						      r_report, 0,
						      (char *)InBuff, rlen,
						      10000);
			}

			if (ret < 0)
				return ret;
		}
#endif
	} else if (inConnectStyle == _ConnectStyle_I2CHID_ ||
		   cmd_opt.usb_hidraw) {
		if (writelen > 0) {
			/* only V3 251x HID-I2C need write syscall */
			if (inProtocolStyle == _Protocol_V3_ &&
			    w_report == REPORT_ID_64_BYTE)
				ret = write(fd, OutBuff, wlen);
			else
				ret = ioctl(fd, HIDIOCSFEATURE(wlen), OutBuff);

			if (ret < 0) {
				if (cmd != ILITEK_TP_CMD_SOFTWARE_RESET) {
					LD_ERR("[%s] hidraw write fail, cmd: %#x, ret:%d, wlen:%d\n",
						__func__, cmd, ret, wlen);
				}
				return ret;
			}
		}

		if (readlen > 0) {
			if (r_report == REPORT_ID_64_BYTE) {
				ret = hidraw_read(fd, InBuff, rlen,
						  timeout_ms + 100,
						  cmd, true, false);
			} else {
				/* Must set report id before IOCTL */
				InBuff[0] = r_report & 0xFF;
				ret = ioctl(fd, HIDIOCGFEATURE(rlen), InBuff);
			}

			if (ret < 0) {
				LD_ERR("[%s] hidraw Read fail, cmd: %#x, ret:%d\n",
					__func__, cmd, ret);
				return ret;
			}
		}
	} else {
		LD_ERR("unexpected inConnectStyle: %d\n", inConnectStyle);
		return -EINVAL;
	}

	if (readlen > 0)
		debug_print_buf("[InBuff]:", InBuff, rlen, false);

	return 0;
}

/*
 * TransferData will make sure write buffer is HID format
 * before enter _TransferData.
 */
int TransferData(uint8_t *OutBuff, int writelen, uint8_t *InBuff,
		 int readlen, int timeout_ms)
{
	int error;
	uint8_t WriteBuff[8192], ReadBuff[8192];
	uint32_t w_report = 0, wlen = 0;
	uint32_t r_report = 0, rlen = 0;

	wlen = getLength(writelen, &w_report);
	rlen = getLength(readlen, &r_report);

	if (writelen > 0 && w_report == REPORT_ID_64_BYTE &&
	    readlen > 0 && r_report != REPORT_ID_64_BYTE) {
		error = TransferData(OutBuff, writelen, NULL, 0, timeout_ms);
		if (error < 0)
			return error;
		return TransferData(NULL, 0, InBuff, readlen, timeout_ms);
	}

	if (writelen > 0)
		debug_print_buf("[Write]:", OutBuff, writelen, false);

	memset(WriteBuff, 0, wlen);
	memset(ReadBuff, 0, rlen);

	if (w_report == REPORT_ID_64_BYTE) {
		WriteBuff[0] = w_report & 0xFF;
		WriteBuff[1] = 0xA3;
		WriteBuff[2] = writelen;
		WriteBuff[3] = readlen;
		memcpy(WriteBuff + 4, OutBuff, writelen);
	} else {
		WriteBuff[0] = w_report & 0xFF;
		WriteBuff[1] = 0xA3;
		WriteBuff[2] = writelen & 0xFF;
		WriteBuff[3] = (writelen >> 8) & 0xFF;
		WriteBuff[4] = readlen & 0xFF;
		WriteBuff[5] = (readlen >> 8) & 0xFF;
		memcpy(WriteBuff + 6, OutBuff, writelen);
	}

	error = TransferData_HID(WriteBuff, writelen,
				 ReadBuff, readlen, timeout_ms);

	switch (inConnectStyle) {
	case _ConnectStyle_I2C_:
		if (r_report == REPORT_ID_64_BYTE)
			memcpy(InBuff, ReadBuff + 4, readlen);
		else
			memcpy(InBuff, ReadBuff, readlen);
		break;

	case _ConnectStyle_USB_:
	case _ConnectStyle_I2CHID_:
		if (r_report == REPORT_ID_64_BYTE)
			memcpy(InBuff, ReadBuff + 4, readlen);
		else
			memcpy(InBuff, ReadBuff, rlen);
		break;
	default:
		LD_ERR("unexpected inConnectStyle: %d\n", inConnectStyle);
	};

	if (error < 0) {
		if ((OutBuff && OutBuff[0] != ILITEK_TP_CMD_SOFTWARE_RESET) ||
		    !OutBuff)
			return error;
	}

	if (readlen > 0)
		debug_print_buf("[Read]:", InBuff, readlen, false);

	return 0;
}

void CloseDevice()
{
	switch (inConnectStyle) {
	case _ConnectStyle_I2C_:
		i2c_read_data_enable(true);
		close(fd);
		break;
	case _ConnectStyle_I2CHID_:
		close(fd);
		break;
	case _ConnectStyle_USB_:
		if (cmd_opt.usb_hidraw) {
			close(fd);
			break;
		}

#ifdef CONFIG_ILITEK_USE_LIBUSB
		usb_reset(hdev);

		usb_release_interface(hdev, 0);
		if (HID_INTERFACE_COUNT == 2)
			usb_release_interface(hdev, 1);

		usb_close(hdev);
#endif
		break;
	}
}

int switch_irq(int flag)
{
	int buf[16];

	if (inConnectStyle != _ConnectStyle_I2C_)
		return 0;

	buf[0] = flag;

	return ioctl(fd, ILITEK_IOCTL_I2C_SWITCH_IRQ, buf);
}

void viDriverCtrlReset()
{
	if (cmd_opt.no_sw_reset)
		return;
	if (inConnectStyle == _ConnectStyle_I2C_) {
		LD_MSG("Set Driver ioctl reset TP\n");
		ioctl(fd, ILITEK_IOCTL_SET_RESET, 0);
	}
}

/* Default wait ack timeout should be 1500000 us */
int viWaitAck(uint8_t cmd, uint8_t *buf, int timeout_ms, bool check_validity)
{
	int error;

	switch (inConnectStyle) {
	case _ConnectStyle_USB_:
		if (cmd_opt.usb_hidraw)
			error = hidraw_read(fd, buf, 64, timeout_ms, cmd,
					    check_validity, true);
		else
			error = usb_read(buf, 64, timeout_ms, cmd,
					 check_validity, true);
		break;
	case _ConnectStyle_I2CHID_:
		error = hidraw_read(fd, buf, 64, timeout_ms, cmd,
				    check_validity, true);
		break;
	default:
		error = -EINVAL;
		break;
	}

	if (error < 0) {
		LD_ERR("timeout_ms: %d, cmd: %x, err: %d\n",
			timeout_ms, cmd, error);
		return error;
	}

	return 0;
}

int read_report(char *buf, int len, int t_ms, struct Netlink_Handle *nl)
{
	switch (inConnectStyle) {
	case _ConnectStyle_USB_:
		if (cmd_opt.usb_hidraw)
			return hidraw_read(fd, (uint8_t *)buf, len, t_ms, 0, false, false);
		else
			return usb_read((uint8_t *)buf, len, t_ms, 0, false, false);
	case _ConnectStyle_I2CHID_:
		return hidraw_read(fd, (uint8_t *)buf, len, t_ms, 0, false, false);
	case _ConnectStyle_I2C_:
		return netlink_recv(nl, buf);
	}

	return -EINVAL;
}

/*
 * Create netlink socket and Initialization msg buffer
 * Notify user thread's pid to kernel driver
 * Please make sure kernel driver is ready and supports netlink flow
 */
int netlink_connect(struct Netlink_Handle *nl, const char *str,
		    uint32_t size, uint32_t tout_ms)
{
	int error;
	struct timeval tv;

	nl->data_size = size;

	/* Create netlink socket and bind with thread's pid */
	nl->fd = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_USERSOCK);
	if (nl->fd < 0) {
		LD_ERR("netlink socket create failed, err: %d\n", nl->fd);
		return -EFAULT;
	}

	tv.tv_sec = tout_ms / 1000;
	tv.tv_usec = 1000 * (tout_ms % 1000);
	setsockopt(nl->fd, SOL_SOCKET, SO_RCVTIMEO,
		   (const char*)&tv, sizeof(tv));

	memset(&nl->src_addr, 0, sizeof(nl->src_addr));
	nl->src_addr.nl_family = AF_NETLINK;
	nl->src_addr.nl_pid = getpid();	/* bind sockt with specific pid */
	nl->src_addr.nl_groups = 0;

	if (bind(nl->fd, (struct sockaddr *)&nl->src_addr,
	    sizeof(nl->src_addr)) < 0)
		goto err_close_socket;

	/* msg struct initialization */
	nl->nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(size));
	if (!nl->nlh)
		goto err_close_socket;
	memset(nl->nlh, 0, NLMSG_SPACE(size));
	memset(&nl->dest_addr, 0, sizeof(nl->dest_addr));
	memset(&nl->msg, 0, sizeof(nl->msg));
	nl->dest_addr.nl_family = AF_NETLINK;
	nl->iov.iov_base = (void *)nl->nlh;
	nl->iov.iov_len = NLMSG_SPACE(size);
	nl->msg.msg_name = (void *)&nl->dest_addr;
	nl->msg.msg_namelen = sizeof(nl->dest_addr);
	nl->msg.msg_iov = &nl->iov;
	nl->msg.msg_iovlen = 1;

	/* Notify user thread pid to kernel */
	nl->nlh->nlmsg_pid = getpid();	/* kernel unicast to specific pid */
	nl->nlh->nlmsg_len = NLMSG_SPACE(size);
	strcpy((char *)NLMSG_DATA(nl->nlh), str);

	error = sendmsg(nl->fd, &nl->msg, 0);
	if (error < 0) {
		LD_ERR("netlink sendmsg failed, err: %d\n", error);
		goto err_free_nlh;
	}

	LD_MSG("netlink socket create success, nl_socket: %d\n", nl->fd);

	return 0;

err_free_nlh:
	free(nl->nlh);
err_close_socket:
	close(nl->fd);

	/* notify following flow netlink is not connected */
	nl->fd = -1;
	nl->nlh = NULL;

	LD_ERR("%s failed\n", __func__);

	return -EFAULT;
}

void netlink_disconnect(struct Netlink_Handle *nl, const char *str)
{
	strcpy((char *)NLMSG_DATA(nl->nlh), str);
	sendmsg(nl->fd, &nl->msg, 0);

	if (nl->nlh)
		free(nl->nlh);
	if (nl->fd >= 0)
		close(nl->fd);
}

int netlink_recv(struct Netlink_Handle *nl, char *buf)
{
	int error;

	if (!nl || !nl->nlh || nl->fd < 0)
		return -EINVAL;

	error = recvmsg(nl->fd, &nl->msg, 0);
	if (error < 0)
		return error;

	memcpy(buf, NLMSG_DATA(nl->nlh), nl->data_size);

	return 0;
}

void init_INT()
{
	if (!cmd_opt.support_INT_ack || inProtocolStyle != _Protocol_V6_ ||
	    inConnectStyle != _ConnectStyle_I2C_)
		return;

	ioctl(fd, ILITEK_IOCTL_I2C_INT_CLR, 0);
	switch_irq(1);

	if (cmd_opt.INT_skip_time_ms) {
		/*
		 * delay msec to allow unexpected ISR handling
		 * then clear flag again
		 */
		usleep(cmd_opt.INT_skip_time_ms * 1000);
		ioctl(fd, ILITEK_IOCTL_I2C_INT_CLR, 0);
	}
}

bool wait_INT(int timeout_ms)
{
	uint8_t get_INT[64];
	bool ret = false;
	int t_ms = timeout_ms;

	if (!cmd_opt.support_INT_ack || inProtocolStyle != _Protocol_V6_ ||
	    inConnectStyle != _ConnectStyle_I2C_) {
		LD_DBG("[%s] not support or disabled\n", __func__);
		return false;
	}

	while (t_ms > 0) {
		ioctl(fd, ILITEK_IOCTL_I2C_INT_POLL, &get_INT);

		if (get_INT[0]) {
			ret = true;
			break;
		}
		t_ms--;
		usleep(1000);
	}

	switch_irq(0);
	if (!ret)
		LD_ERR("%d ms timeout failed\n", timeout_ms);

	return ret;
}

uint32_t get_driver_ver()
{
	int error;
	uint32_t driver_ver;
	uint8_t buf[8];

	if (inConnectStyle != _ConnectStyle_I2C_)
		return 0;

	error = ioctl(fd, ILITEK_IOCTL_DRIVER_INFORMATION, &buf);
	if (error < 0)
		return 0;
	memcpy(ptl.dri_ver, buf, sizeof(buf));
	driver_ver = (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];

	return driver_ver;
}

int write_and_wait_ack(uint8_t *Wbuff, int wlen, int timeout_ms,
	int cnt, int delay_ms, int type)
{
	int error;
	uint8_t Rbuff[64];
	uint8_t cmd;

	switch (inConnectStyle) {
	case _ConnectStyle_USB_:
	case _ConnectStyle_I2CHID_:
		if (Wbuff[0] == ILITEK_TP_CMD_ACCESS_SLAVE)
			cmd = Wbuff[2];
		else
			cmd = Wbuff[0];

		error = TransferData(Wbuff, wlen, NULL, 0, 0);
		if (viWaitAck(cmd, Rbuff, timeout_ms, true) < 0) {
			if (CheckBusy(cnt, delay_ms, type) < 0)
				return -ETIME;
		}

		break;
	case _ConnectStyle_I2C_:
		init_INT();

		error = TransferData(Wbuff, wlen, NULL, 0, 0);

		if (!wait_INT(timeout_ms)) {
			if (CheckBusy(cnt, delay_ms, type) < 0)
				return -ETIME;
		}

		break;
	default:
		LD_ERR("unexpected interface: %d\n", inConnectStyle);
		return -EINVAL;
	}

	return error;
}

int set_engineer(bool enable)
{
	uint8_t buf[64];

	buf[0] = 0x03;
	buf[1] = 0xF1;

	if (enable)
		buf[2] = 0x01;
	else
		buf[2] = 0x00;

	if (inConnectStyle == _ConnectStyle_I2CHID_)
		return write(fd, buf, 64);

	return 0;
}

FILE *log_openfile(char *log_dir, char *prefix)
{
	FILE *file;

	time_t rawtime;
	struct tm *timeinfo;
	char timebuf[60], filename[512];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(timebuf, 60, "%Y%m%d_%I%M%S", timeinfo);

	if (strlen(log_dir) && access(log_dir, 0) < 0) {
		if (mkdir(log_dir, 0777)) {
			LD_ERR("create directory %s failed!\n", log_dir);
			return NULL;
		}
	}

	if (strlen(log_dir))
		sprintf(filename, "%s/%s_%s.txt", log_dir, prefix, timebuf);
	else
		sprintf(filename, "./%s_%s.txt", prefix, timebuf);

	file = fopen(filename, "w");

	LD_MSG("*******************************************\n");
	LD_MSG("************** Start Logging **************\n");
	LD_MSG("*******************************************\n\n");

	return file;
}

void log_closefile(FILE *file)
{
	if (!file)
		return;

	LD_MSG("\n");
	LD_MSG("*******************************************\n");
	LD_MSG("************** End of Logging *************\n");
	LD_MSG("*******************************************\n");
	fclose(file);
}

unsigned long getTimeMs(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

bool check_edid(const char *dirpath)
{
	int fd;
	char filename[512];
	char buf[128];

	memset(filename, 0, sizeof(filename));
	sprintf(filename, "%s/%s", dirpath, "status");

	if (access(filename, F_OK))
		return false;

	fd = open(filename, O_RDONLY);
	if (fd < 0)
		return false;

	if (read(fd, buf, sizeof(buf) - 1) < 0) {
		close(fd);
		return false;
	}

	close(fd);

	return !strncmp("connected", buf, 9) ? true : false;
}

int read_edid(const char *dirpath, struct edid_block *edid)
{
	FILE *file;
	char filename[512];
	int error;

	memset(filename, 0, sizeof(filename));
	sprintf(filename, "%s/%s", dirpath, "edid");

	file = fopen(filename, "rb");
	if (!file)
		return -ENOENT;

	error = fread(edid, sizeof(uint8_t), EDID_LENGTH, file);
	if (error != EDID_LENGTH) {
		LD_ERR("read edid failed, get: %d, expected: %d\n",
			error, EDID_LENGTH);
		fclose(file);
		return -EINVAL;
	}

	fclose(file);

	debug_print_buf("EDID raw:", (uint8_t *)edid, EDID_LENGTH, true);
	LD_MSG("Manufacturer: %#x\n", edid->manufacturerCode);
	LD_MSG("Product Code: %#x\n", edid->productCode);
	LD_MSG("Serial Number: %#x\n", edid->serialNumber);
	LD_MSG("Week Number: %d\n", edid->manufacturedWeek);
	LD_MSG("Year Number: %d\n", edid->manufacturedYear + 1990);
	LD_MSG("EDID Ver.: %#x\n", edid->version);
	LD_MSG("EDID Rev.: %#x\n", edid->revision);
	LD_MSG("Horizon Screen Size: %d cm\n", edid->maxHorizontalImageSize);
	LD_MSG("Vertical Screen Size: %d cm\n", edid->maxVerticalImageSize);

	return 0;
}

int get_edid(struct edid_block *edid, char *str, int str_size)
{
	char fpath[512];
	DIR *dir;
	struct dirent *entry;

	dir = opendir(EDID_SYS_PATH);

	if (str) {
		memset(str, 0, sizeof(str_size));
		sprintf(str, "not available");
	}

	if (!dir) {
		LD_ERR("opendir failed\n");
		return -ENOENT;
	}

	while ((entry = readdir(dir))) {
		memset(fpath, 0, sizeof(fpath));
		sprintf(fpath, "%s/%s", EDID_SYS_PATH, entry->d_name);

		if (!check_edid(fpath))
			continue;

		LD_MSG("path: %s edid connected\n", fpath);

		if (read_edid(fpath, edid) < 0)
			continue;

		if (str) {
			sprintf(str, "%04x-%04x",
				edid->manufacturerCode, edid->productCode);
		}

		return 0;
	}

	return -ENOENT;
}
