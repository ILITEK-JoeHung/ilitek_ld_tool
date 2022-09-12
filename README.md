# ilitek_ld_tool
**Linux_Daemon_ITS tool for ChromeOS touch firmware update**

**This is a firmware update tool for ILITEK touch controller**

**How to build (TBU):**

**How to run (TBU):**

**How to perform a Firmware Update:**
  1. Put ILITEK firmware update tool "ilitek_ld" into target system (executable path ex. /usr/local/.)
  2. Check *.hex firmware is ready and accessible in the target system.
  3. key-in command as below, please check and replace each command argument with the following description.
  ```
  sudo ilitek_ld FWUpgrade <Interface> <Protocol> <I2C driver file node> <I2C addr.> <.hex file path>
        <Interface> support three touch controller interface as below.
                    I2C       (for I2C interface)
                    USB       (for USB interface)
                    I2C-HID   (for HID-over-I2C interface)
        <Protocol>  support two controller firmware protocol as below.
                    V3  (for ILI2312/2315/2510/2511)
                    V6  (for ILI2316/2130/2131/2132/2322/2323/2326/2520/2521/2901)
        <I2C driver file node>  replace with below setting.
                    /dev/ilitek_ctrl  (for I2C interface only)
                    null              (for others interface)
        <I2C addr.>  replace with below setting.
                    41                (for I2C interface only)
                    null              (for others interface)
        <.hex file path>  replace with your .hex file path
        
  ```
