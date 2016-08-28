#!/bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

sudo cp $DIR/ros.service /etc/systemd/system/
systemctl enable ros.service
