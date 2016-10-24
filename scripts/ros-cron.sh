#!/bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

now=$(date +"%T")

echo "[$now] Configure Quickcam Pro 9000 cameras disabled"
exit 0


if [ -e "/dev/video6" ]
then
	# Configure Quickcam Pro 9000 camera (left)
	echo "[$now] Configuring Quickcam Pro 9000 camera (left)"
	v4l2-ctl --device=/dev/video6 --set-ctrl exposure_auto=1 # Manual Mode
	v4l2-ctl --device=/dev/video6 --set-ctrl backlight_compensation=0
	v4l2-ctl --device=/dev/video6 --set-ctrl exposure_absolute=800
	v4l2-ctl --device=/dev/video6 --set-ctrl gain=200
	v4l2-ctl --device=/dev/video6 --set-ctrl focus=0
	v4l2-ctl --device=/dev/video6 --set-ctrl white_balance_temperature_auto=0
	v4l2-ctl --device=/dev/video6 --set-ctrl white_balance_temperature=1000
	v4l2-ctl --device=/dev/video6 --set-ctrl led1_mode=0 # Off
else
	echo "[$now] Could not configure Quickcam Pro 9000 camera (left): device not found on /dev/video6"
fi

if [ -e "/dev/video7" ]
then
	# Configure Quickcam Pro 9000 camera (right)
	echo "[$now] Configuring Quickcam Pro 9000 camera (right)"
	v4l2-ctl --device=/dev/video7 --set-ctrl exposure_auto=1 # Manual Mode
	v4l2-ctl --device=/dev/video7 --set-ctrl backlight_compensation=0
	v4l2-ctl --device=/dev/video7 --set-ctrl exposure_absolute=800
	v4l2-ctl --device=/dev/video7 --set-ctrl gain=200
	v4l2-ctl --device=/dev/video7 --set-ctrl focus=0
	v4l2-ctl --device=/dev/video7 --set-ctrl white_balance_temperature_auto=0
	v4l2-ctl --device=/dev/video7 --set-ctrl white_balance_temperature=1000
	v4l2-ctl --device=/dev/video7 --set-ctrl led1_mode=0 # Off
else
	echo "[$now] Could not configure Quickcam Pro 9000 camera (right): device not found on /dev/video7"
fi
