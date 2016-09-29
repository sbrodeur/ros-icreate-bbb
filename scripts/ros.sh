#!/bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
CATKIN_WORKSPACE_DIR=$( cd "$( dirname "${DIR}/../.." )" && pwd )

# Start roscore instance
export ROS_MASTER_URI=http://192.168.7.2:11311
export ROS_IP=192.168.7.2
export ROS_PORT=11311

source /opt/ros/kinetic/setup.bash
source $CATKIN_WORKSPACE_DIR/devel/setup.bash
export PYTHONPATH=$CATKIN_WORKSPACE_DIR/src/irobot_create/src:$PYTHONPATH

/opt/ros/kinetic/bin/roscore -p $ROS_PORT &
sleep 4

# Configure Quickcam Pro 9000 camera
v4l2-ctl --device=/dev/video6 --set-ctrl exposure_auto=1
v4l2-ctl --device=/dev/video6 --set-ctrl exposure_absolute=255
v4l2-ctl --device=/dev/video6 --set-ctrl gain=255
v4l2-ctl --device=/dev/video6 --set-ctrl led1_mode=0

# Configure Quickcam Pro 9000 camera
v4l2-ctl --device=/dev/video7 --set-ctrl exposure_auto=1
v4l2-ctl --device=/dev/video7 --set-ctrl exposure_absolute=255
v4l2-ctl --device=/dev/video7 --set-ctrl gain=255
v4l2-ctl --device=/dev/video7 --set-ctrl led1_mode=0

# Start capture nodes
/opt/ros/kinetic/bin/roslaunch $CATKIN_WORKSPACE_DIR/scripts/roomba_robot.launch
