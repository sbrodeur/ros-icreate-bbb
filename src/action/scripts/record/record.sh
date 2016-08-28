#!/usr/bin/env bash

export ROS_IP=192.168.7.2
export ROS_MASTER_URI=http://192.168.7.2:11311

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
current_time=$(date "+%Y.%m.%d-%H.%M.%S")

echo 'Launching roslaunch...'
#roslaunch record_autonomous.launch path:=$WORK/catkin_ws/src/action/scripts/record/data/ current_time:=${current_time}
roslaunch record_control.launch path:=$WORK/catkin_ws/src/action/scripts/record/data/ current_time:=${current_time}

echo 'Stopping robot...'
rosservice call /irobot_create/brake 1

echo 'End of recording.'

echo 'All done.'
