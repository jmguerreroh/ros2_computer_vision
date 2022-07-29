# Modified by José Miguel Guerrero Hernández
cmake_minimum_required(VERSION 3.4.0)
project(pal_gazebo_worlds)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_auto REQUIRED)

# PAL Robotics stricter build flags, other developers should not worry about this
find_package(ament_cmake_pal QUIET)

#include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

# Will migrate when https://github.com/ros-simulation/gazebo_ros_pkgs/pull/973 is merged
# Or when get/set physics_properties services are added to gazebo
#add_executable(increase_real_time_factor src/increase_real_time_factor.cpp)
#target_link_libraries(increase_real_time_factor ${catkin_LIBRARIES})

#install(TARGETS increase_real_time_factor
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  find_package(roscpp REQUIRED)
#  find_package(gazebo_msgs REQUIRED)
#  include_directories(SYSTEM ${roscpp_INCLUDE_DIRS} ${gazebo_msgs_INCLUDE_DIRS})

#  add_rostest_gtest(run_gazebo_server-test test/run_gazebo_server.test test/test_run_gazebo_server.cpp)
#  target_link_libraries(run_gazebo_server-test ${roscpp_LIBRARIES})
#endif(CATKIN_ENABLE_TESTING)


ament_auto_package(INSTALL_TO_SHARE launch models worlds urdf meshes photos)

