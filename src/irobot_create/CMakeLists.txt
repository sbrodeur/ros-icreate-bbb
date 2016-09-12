cmake_minimum_required(VERSION 2.8.3)
project(irobot_create)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs)

add_message_files(DIRECTORY msg FILES MotorControl.msg Contact.msg MotorRequest.msg IrRange.msg) 
add_service_files(DIRECTORY srv FILES Brake.srv Circle.srv Demo.srv Dock.srv Leds.srv Reset.srv Start.srv Stop.srv Tank.srv Turn.srv)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs )

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

catkin_install_python(PROGRAMS
  bin/driver.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/irobot_create
)

