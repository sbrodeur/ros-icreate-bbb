cmake_minimum_required(VERSION 2.8.3)
project(create)

find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  rosbag
  message_generation
  sensor_msgs
  tf
  diagnostic_msgs
  diagnostic_updater
)

add_message_files(DIRECTORY msg FILES MotorSpeed.msg Contact.msg IrRange.msg ) 
add_service_files(DIRECTORY srv FILES Brake.srv Circle.srv Demo.srv Dock.srv Leds.srv Reset.srv Start.srv Stop.srv Tank.srv Turn.srv Beep.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

## Declare cpp library
add_library(${PROJECT_NAME}
  src/create.cpp
  src/serial.cpp
  src/serial_stream.cpp
  src/serial_query.cpp
  src/data.cpp
  src/packet.cpp
  src/types.cpp
)

if(THREADS_HAVE_PTHREAD_ARG)
  set_property(TARGET ${PROJECT_NAME} PROPERTY COMPILE_OPTIONS "-pthread")
  set_property(TARGET ${PROJECT_NAME} PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
endif()

if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(${PROJECT_NAME} "${CMAKE_THREAD_LIBS_INIT}")
endif()

target_link_libraries(create
  ${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_driver nodes/driver.cpp)
add_dependencies(${PROJECT_NAME}_driver ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_driver
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_odometry nodes/odometry.cpp)
add_dependencies(${PROJECT_NAME}_odometry ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_odometry
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_odometry_rosbag nodes/odometry_rosbag.cpp)
add_dependencies(${PROJECT_NAME}_odometry_rosbag ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_odometry_rosbag
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES
        include/create/create.h
        include/create/serial.h
        include/create/types.h
        include/create/data.h
        include/create/packet.h
        include/create/util.h
        DESTINATION include/create)

install(TARGETS ${PROJECT_NAME}_driver ${PROJECT_NAME}_odometry ${PROJECT_NAME}_odometry_rosbag
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
        