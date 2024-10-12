cmake_minimum_required(VERSION 2.8.3)
project(hexapod_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable( odom_frame_publisher src/odom_frame_publisher.cpp )
target_link_libraries( odom_frame_publisher ${catkin_LIBRARIES})
add_executable( imu_preprocess src/imu_preprocess.cpp )
target_link_libraries( imu_preprocess ${catkin_LIBRARIES})

roslaunch_add_file_check(launch)

install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

