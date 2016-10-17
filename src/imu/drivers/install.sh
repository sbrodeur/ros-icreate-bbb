#!/bin/bash

# Project directory on local computer
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

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

echo 'All done.'
