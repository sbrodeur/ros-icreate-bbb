#!/bin/bash

# Project directory on local computer
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

rmmod bmp085
rmmod bmp085-probe
rmmod l3gd20
rmmod l3gd20-probe
rmmod lsm303d
rmmod lsm303d-probe

cd $DIR/bmp180
make clean
make 
make install 
depmod -a

cd $DIR/l3gd20
make clean
make 
make install 
depmod -a

cd $DIR/lsm303d
make clean
make 
make install 
depmod -a




modprobe bmp085
modprobe bmp085-probe
modprobe l3gd20
modprobe l3gd20-probe
modprobe lsm303d
modprobe lsm303d-probe


echo 'All done.'
