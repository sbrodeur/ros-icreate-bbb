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

Put the following content in file /etc/udev/rules.d/20-imu.rules:
KERNEL=="event*", SUBSYSTEM=="input", SUBSYSTEMS=="input", ATTRS{name}=="lsm303d_acc", SYMLINK+="lsm303d_acc"
KERNEL=="event*", SUBSYSTEM=="input", SUBSYSTEMS=="input", ATTRS{name}=="lsm303d_mag", SYMLINK+="lsm303d_mag"

This will map the accelerometer input to /dev/lsm303d_acc and magnetometer to /dev/lsm303d_mag instead of /dev/input/event*
