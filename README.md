# ILITEK Touchscreen Firmware Update Tool

## How To Build
1. Install dependencies by using 
      ```sudo apt-get install libusb-dev```
2. Run ```make```

## How To Run
1. Put the ILITEK firmware update tool (called "ilitek_ld") into the target system under an executable path
2. Verify the *.hex firmware file exists on the target system and is accessible
3. Run the following command with correct arguments:

``` sudo ilitek_ld FWUpgrade <Interface> <Protocol> <i2c driver file node> <i2c addr> <Hex path> ```

### Command Line Arguments
- ```<Interface>``` Interface between ILTIEK Touch IC and system, should be ```I2C```, ```USB```, or ```I2C-HID```
- ```<Protocol>```  ILITEK Touch IC command protocol, should be V3 or V6,
  - **V3**: ILI2312/2315/2510/2511
  - **V6**: ILI2316/2130/2131/2132/2322/2323/2326/2520/2521
- ```<i2c driver file node>``` For I2C interface only, should be /dev/ilitek_ctrl
- ```<i2c addr>```  For I2C interface only, should be 41
- ```<Hex path>```  Hex path to upgrade

### Example
``` sudo ./ilitek_ld FWUpgrade USB V3 /dev/ilitek_ctrl 41 firmware.hex ```