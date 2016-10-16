cmake_minimum_required(VERSION 2.8.3)
project(camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS image_transport roscpp std_msgs sensor_msgs camera_info_manager cv_bridge)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)
pkg_check_modules(avformat libavformat REQUIRED)
pkg_check_modules(avdevice libavdevice REQUIRED)
pkg_check_modules(avutil libavutil REQUIRED)

find_package(OpenCV REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(include
  include/segment
  include/meanshift
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
  ${avformat_INCLUDE_DIRS}
  ${avdevice_INCLUDE_DIRS}
  ${avutil_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Build the camera library
add_library(${PROJECT_NAME} src/capturev4l2.cpp)
target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_capture nodes/capture.cpp)
target_link_libraries(${PROJECT_NAME}_capture
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${avformat_LIBRARIES}
  ${avdevice_LIBRARIES}
  ${avutil_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_viewer nodes/viewer.cpp)
target_link_libraries(${PROJECT_NAME}_viewer
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_capture ${PROJECT_NAME}_viewer ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
