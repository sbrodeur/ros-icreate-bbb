#!/bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
CATKIN_WORKSPACE_DIR=$( cd "$( dirname "${DIR}/../.." )" && pwd )

# Start joystick driver in userspace
# NOTE: we can set priority to realtime to avoid failing under heavy load
xboxdrv --mimic-xpad-wireless --priority=normal --silent &
sleep 4

# Start roscore instance
export ROS_MASTER_URI=http://192.168.7.2:11311
export ROS_IP=192.168.7.2
export ROS_PORT=11311

source /opt/ros/kinetic/setup.bash
source $CATKIN_WORKSPACE_DIR/devel/setup.bash

/opt/ros/kinetic/bin/roscore -p $ROS_PORT &
sleep 4

# Configure barometer and temperature sensor (bmp180) for standard accuracy (faster sampling)
echo 1 > /sys/bus/i2c/drivers/bmp085/2-0077/oversampling

# Configure Quickcam Pro 9000 camera (left)
v4l2-ctl --device=/dev/video6 --set-ctrl exposure_auto=1 # Manual Mode
v4l2-ctl --device=/dev/video6 --set-ctrl backlight_compensation=0
v4l2-ctl --device=/dev/video6 --set-ctrl exposure_absolute=800
v4l2-ctl --device=/dev/video6 --set-ctrl gain=200
v4l2-ctl --device=/dev/video6 --set-ctrl power_line_frequency=2 # 60 Hz
v4l2-ctl --device=/dev/video6 --set-ctrl focus=0
v4l2-ctl --device=/dev/video6 --set-ctrl white_balance_temperature_auto=0
v4l2-ctl --device=/dev/video6 --set-ctrl white_balance_temperature=1000
v4l2-ctl --device=/dev/video6 --set-ctrl led1_mode=0 # Off

# Configure Quickcam Pro 9000 camera (right)
v4l2-ctl --device=/dev/video7 --set-ctrl exposure_auto=1 # Manual Mode
v4l2-ctl --device=/dev/video7 --set-ctrl backlight_compensation=0
v4l2-ctl --device=/dev/video7 --set-ctrl exposure_absolute=800
v4l2-ctl --device=/dev/video7 --set-ctrl gain=200
v4l2-ctl --device=/dev/video7 --set-ctrl power_line_frequency=2 # 60 Hz
v4l2-ctl --device=/dev/video7 --set-ctrl focus=0
v4l2-ctl --device=/dev/video7 --set-ctrl white_balance_temperature_auto=0
v4l2-ctl --device=/dev/video7 --set-ctrl white_balance_temperature=1000
v4l2-ctl --device=/dev/video7 --set-ctrl led1_mode=0 # Off

# Start capture nodes
/opt/ros/kinetic/bin/roslaunch $CATKIN_WORKSPACE_DIR/scripts/ros-nodes-record.launch
