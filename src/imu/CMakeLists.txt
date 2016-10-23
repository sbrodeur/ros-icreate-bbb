cmake_minimum_required(VERSION 2.8.3)
project(imu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs message_generation)

## pkg-config libraries
find_package(PkgConfig REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES ImuBatch.msg MagneticFieldBatch.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable

add_executable(${PROJECT_NAME}_capture_acc_gyro nodes/capture_acc_gyro.cpp )
add_dependencies(${PROJECT_NAME}_capture_acc_gyro ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_capture_acc_gyro
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_capture_mag nodes/capture_mag.cpp )
add_dependencies(${PROJECT_NAME}_capture_mag ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_capture_mag
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_capture_baro nodes/capture_baro.cpp)
add_dependencies(${PROJECT_NAME}_capture_baro ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_capture_baro
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_capture_temp nodes/capture_temp.cpp)
add_dependencies(${PROJECT_NAME}_capture_temp ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_capture_temp
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_capture_acc_gyro
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_capture_mag
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_capture_baro
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_capture_temp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
