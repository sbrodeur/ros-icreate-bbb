Linux kernel driver for bmp085/bmp180 sensor
--------------------------------------------

This is kernel driver from Linux/drivers/misc/ 
with aditional module for i2c sensor detection 

*Build
------
$ make 
$ sudo make install 
$ sudo depmod -a

*Config
-------
$ sudo nano /etc/modules 
Add lines as follow
bmp085
bmp085-i2c
bmp085-probe

*Test:
-------
$ sudo modprobe bmp085 bmp085-i2c bmp085-probe 

$ cat /sys/bus/i2c/drivers/bmp085/2-0077/temp0_input
$ cat /sys/bus/i2c/drivers/bmp085/2-0077/pressure0_input
