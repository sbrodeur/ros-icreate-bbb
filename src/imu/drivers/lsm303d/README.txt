Linux kernel driver for lsm303d sensor
--------------------------------------------

This is kernel driver from ST

*Build
------
$ make 
$ sudo make install 
$ sudo depmod -a


*Config
-------
$ sudo nano /etc/modules 
Add lines as follow
lsm303d

*Test:
-------
$ sudo modprobe lsm303d

$ cat /sys/bus/i2c/drivers/bmp085/1-0077/temp0_input
$ cat /sys/bus/i2c/drivers/bmp085/1-0077/pressure0_input
