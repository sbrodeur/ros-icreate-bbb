Linux kernel driver for l3gd20 sensor
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
l3gd20

*Test:
-------
$ sudo modprobe l3gd20

$ cat /sys/bus/i2c/drivers/bmp085/1-0077/temp0_input
$ cat /sys/bus/i2c/drivers/bmp085/1-0077/pressure0_input
