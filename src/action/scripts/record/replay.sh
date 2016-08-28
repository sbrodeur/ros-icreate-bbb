#!/usr/bin/env bash

export IP=`sudo ifconfig eth0 | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p'`
export ROS_IP=$IP
export ROS_PORT=13111
export ROS_MASTER_URI="http://${ROS_IP}:${ROS_PORT}/"
roscore -p $ROS_PORT

echo 'Launching replay...'
rosbag play --quiet $WORK/catkin_ws/src/action/scripts/record/data/session_2016.01.25-10.41.44.bag

echo 'End of replay.'
echo 'All done.'
