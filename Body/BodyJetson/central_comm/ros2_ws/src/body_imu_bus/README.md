### IMU Bus
# i²c clockspeed:
Clockspeed is set to 100kHz:
```
# Make a copy of your existing Device Tree
sudo cp \
  '/boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb' \
  /boot/dtb/i2c7-100k.dtb
```
```
# Change ONLY the I2C7 frequency to 100 kHz
sudo fdtput -t i /boot/dtb/i2c7-100k.dtb \
  /bus@0/i2c@c250000 clock-frequency 100000
```
```
# Verify
fdtget -t i /boot/dtb/i2c7-100k.dtb \
  /bus@0/i2c@c250000 clock-frequency
```
```
# Tell Jetson to use it
sudo cp /boot/extlinux/extlinux.conf \
  /boot/extlinux/extlinux.conf.backup
```
open file:
```
sudo nano /boot/extlinux/extlinux.conf
```
Add this one line:
```
 LABEL primary
       MENU LABEL primary kernel
       LINUX /boot/Image
+      FDT /boot/dtb/i2c7-100k.dtb
       INITRD /boot/initrd
       APPEND ${cbootargs} root=/dev/nvme0n1p1 rw ...
```

Reboot after that. 
Check for i²c bus speed:
```
od -An -tu4 --endian=big \
  /sys/class/i2c-dev/i2c-7/device/of_node/clock-frequency
```