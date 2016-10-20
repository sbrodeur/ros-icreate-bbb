#!/bin/bash

#On Debian 8, armf architecture
# see: http://wiki.ros.org/kinetic/Installation/Source

# Installing bootstrap dependencies
sudo pip install -U rosdep rosinstall_generator wstool rosinstall
sudo pip install --upgrade setuptools

# Initializing rosdep
sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update

# Building the catkin Packages
mkdir -p $HOME/build
cd $HOME/build
mkdir ros_catkin_ws_kinetic
cd ros_catkin_ws_kinetic
rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init src kinetic-ros_comm-wet.rosinstall
rosinstall_generator orocos_kdl python_orocos_kdl tf2_geometry_msgs diagnostics image_common image_transport_plugins common_msgs rosconsole_bridge --rosdistro kinetic --deps --wet-only --tar > add-pkgs.rosinstall
wstool merge -t src add-pkgs.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
# NOTE: since we compile on a system with only 512MB of RAM, configure gcc gargabe collector to be aggresive and save temporary files to disk.
#       see: http://jkroon.blogs.uls.co.za/it/scriptingprogramming/preventing-gcc-from-trashing-the-system 
#            https://gcc.gnu.org/onlinedocs/gcc/Option-Summary.html
CATKIN_COMPILE_FLAGS="\"-O3 -save-temps --param ggc-min-expand=10 --param ggc-min-heapsize=4096\""
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS=$CATKIN_COMPILE_FLAGS -DCMAKE_CXX_FLAGS=$CATKIN_COMPILE_FLAGS --install-space /opt/ros/kinetic

echo "source /opt/ros/kinetic/setup.bash" >>  $HOME/.profile

