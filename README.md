# A mobile robotic platform based on the iRobot Create to record multimodal datasets
ROS (Robotic Operating System) packages for the iRobot Create with onboard BeagleBone Black

![alt tag](https://raw.githubusercontent.com/sbrodeur/ros-icreate-bbb/master/doc/images/robot.png)

## Hardware

* iRobot Roomba iCreate 1, with Roomba Charging Dock and Roomba Virtual Wall Scheduler Units
* CNC-cut clear acrylic mounts, mounting spacers and screws, see CAD files (libreCAD)
* Beaglebone black with acrylic case
* Plugable USB 2.0 4-Port Hub with 12.5W Power Adapter with BC 1.2 Charging
* Dual Logitech Quickcam Pro 9000 camera
* 5A DC-DC Adjustable Buck Step Down Module 24V/12V/5V Voltage Regulator Converter (unregulated battery to 5V)
* Real Time Clock Memory Module (DS3231) 
* 10DOF IMU: accelerometer and magnetometer (LSM303D), gyroscope (L3GD20), pressure sensor (BMP180, not used)
* 802.11n/g/b 150Mbps Mini USB WiFi Wireless Adapter Network LAN Card w/Antenna (Ralink RT5370)

For remote control:
* Wired USB Vibration Shock Gamepad Game Controller Joystick Joypad for PC Laptop
* Xbox 360 Wireless Controller, with Xbox 360 Wireless Gaming Receiver

Note that the right camera is physically inversed.

## Compiling ROS on the Beaglebone Black

Install bootstrap dependencies:
```
sudo pip install -U rosdep rosinstall_generator wstool rosinstall
sudo pip install --upgrade setuptools
```

Initialize rosdep:
```
sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
```

Build the catkin packages and install at default location (/opt/ros/kinetic):
```
mkdir -p $HOME/build
cd $HOME/build
mkdir ros_catkin_ws_kinetic
cd ros_catkin_ws_kinetic
rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init src kinetic-ros_comm-wet.rosinstall
rosinstall_generator diagnostics image_common image_transport_plugins common_msgs rosconsole_bridge --rosdistro kinetic --deps --wet-only --tar > add-pkgs.rosinstall
wstool merge -t src add-pkgs.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
CATKIN_COMPILE_FLAGS="\"-O3 -save-temps --param ggc-min-expand=10 --param ggc-min-heapsize=4096\""
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS=$CATKIN_COMPILE_FLAGS -DCMAKE_CXX_FLAGS=$CATKIN_COMPILE_FLAGS --install-space /opt/ros/kinetic
```

Note that since we compile on a system with only 512MB of RAM, we need to configure gcc gargabe collector to be aggresive and save temporary files to disk.


Set environment variables:
```
echo "source /opt/ros/kinetic/setup.bash" >>  $HOME/.profile
```

For more information on compiling ROS from source, see <http://wiki.ros.org/kinetic/Installation/Source>

## Compiling the code

Download the source code from the git repository:
```
git clone https://github.com/sbrodeur/ros-icreate-bbb.git
```

Create a symlink to the ROS distribution for the catkin workspace (here for ROS kinetic):
```
cd ros-icreate-bbb
ln -s /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt
```

Compile workspace with catkin:
```
CATKIN_COMPILE_FLAGS="\"-O3 -save-temps --param ggc-min-expand=10 --param ggc-min-heapsize=4096\""
alias catkin_make="catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS=$CATKIN_COMPILE_FLAGS -DCMAKE_CXX_FLAGS=$CATKIN_COMPILE_FLAGS"
catkin_make
```

## Dataset specifications

Sensors recorded:
* Left and right RGB cameras (320x240, JPEG, 30 Hz sampling rate)
* Left and right microphones (16000 Hz sampling rate, 64 ms frame length)
* Inertial measurement unit: accelerometer, gyroscope, magnetometer (90 Hz sampling rate)
* Battery and charging state (50 Hz sampling rate)
* Left and right motor velocities (50 Hz sampling rate)
* Infrared sensors (50 Hz sampling rate)
* Contact sensors (50 Hz sampling rate)
* Odometry (50 Hz sampling rate)
* Atmospheric pressure (50 Hz sampling rate)
* Air temperature (1 Hz sampling rate)

Note that the odometry distance is based on the internal wheel sensors (velocities) may not be accurate.

Other information included:
* Room location, date and time of the session.
* Stereo calibration parameters for the RGB cameras.

## Dataset usage

The provided multimonal dataset can have multiple usages:
* Multimodal unsupervised object learning: learn the statistical regularities (structures) of the sensor inputs.
* Multimodal prediction: learn to predict future states of the sensory inputs. 
* Egomotion detection: learn to predict motor states from the other sensory inputs (e.g. visual optical flow)
* Causality detection: learn to predict when the robot affects its own sensory inputs (i.e. due to motors), and when the environment is perturbating the sensory inputs (e.g. the user moves the robot around, the robot sees a human moving).
* Object avoidance and collision handling: learn to correct avoid obstacles based on the sensory inputs.

NOTE: audio localization based on interaural time difference (ITD) may not be possible because the audio channels are not synchronized perfectly due to variable USB latencies. Interaural level difference (ILD) can however be used.
