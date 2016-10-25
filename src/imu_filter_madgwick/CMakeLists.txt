cmake_minimum_required(VERSION 2.8.3)
project(imu_filter_madgwick)

find_package(catkin REQUIRED COMPONENTS roscpp rosbag message_filters sensor_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros pluginlib message_filters dynamic_reconfigure)

find_package(Boost REQUIRED COMPONENTS system thread signals program_options)

# Generate dynamic parameters
generate_dynamic_reconfigure_options(cfg/ImuFilterMadgwick.cfg)


catkin_package(
  DEPENDS Boost
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs pluginlib message_filters dynamic_reconfigure
  INCLUDE_DIRS
  LIBRARIES imu_filter
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


# create imu_filter library
add_library (imu_filter src/imu_filter.cpp  src/imu_filter_ros.cpp src/stateless_orientation.cpp)
add_dependencies(imu_filter ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# create imu_filter_node executable
add_executable(imu_filter_node src/imu_filter_node.cpp)
add_dependencies(imu_filter_node ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_filter_node imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# create imu_filter_rosbag executable
add_executable(imu_filter_rosbag src/imu_filter_rosbag.cpp)
add_dependencies(imu_filter_rosbag ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_filter_rosbag imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS imu_filter imu_filter_node imu_filter_rosbag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
