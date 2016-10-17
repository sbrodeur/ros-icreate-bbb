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
l3gd20-probe

*Test:
-------
$ sudo modprobe l3gd20 l3gd20-probe

$ cat /sys/bus/i2c/drivers/l3gd20_gyr/2-006b/name
$ cat /sys/bus/i2c/drivers/l3gd20_gyr/2-006b/range

Put the following content in file /etc/udev/rules.d/20-imu.rules:
KERNEL=="event*", SUBSYSTEM=="input", SUBSYSTEMS=="input", ATTRS{name}=="l3gd20_gyr", SYMLINK+="l3gd20_gyr"

This will map the gyroscope input to /dev/l3gd20_gyr instead of /dev/input/event*

