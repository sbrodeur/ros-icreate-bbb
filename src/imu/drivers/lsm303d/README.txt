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
lsm303d-probe

*Test:
-------
$ sudo modprobe lsm303d lsm303d-probe

$ cat /sys/bus/i2c/drivers/lsm303d/2-001d/enable_temperature
$ cat /sys/bus/i2c/drivers/lsm303d/2-001d/read_temperature

