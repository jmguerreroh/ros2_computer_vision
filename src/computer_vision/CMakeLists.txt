cmake_minimum_required(VERSION 3.8)
project(computer_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#######

# OpenCV node
add_executable(cv_node src/cv_node.cpp)
target_include_directories(cv_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cv_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  cv_node
  "rclcpp"
  "image_transport"
  "cv_bridge"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
)

target_link_libraries(cv_node ${PCL_LIBRARIES})

install(TARGETS cv_node
  DESTINATION lib/${PROJECT_NAME})


# PCL node
add_executable(pcl_node src/pcl_node.cpp)
target_include_directories(pcl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pcl_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pcl_node
  "rclcpp"
  "image_transport"
  "cv_bridge"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
)

target_link_libraries(pcl_node ${PCL_LIBRARIES})

install(TARGETS pcl_node
  DESTINATION lib/${PROJECT_NAME})


# OpenCV and PCL node
add_executable(cv_pcl_node src/cv_pcl_node.cpp)
 target_include_directories(cv_pcl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cv_pcl_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  cv_pcl_node
  "rclcpp"
  "image_transport"
  "cv_bridge"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
)
  
target_link_libraries(cv_pcl_node ${PCL_LIBRARIES})
  
install(TARGETS cv_pcl_node
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(cv_pcl_node ${PCL_LIBRARIES})

install(TARGETS cv_pcl_node
  DESTINATION lib/${PROJECT_NAME})

  
# Launcher
install(DIRECTORY  launch
        DESTINATION share/${PROJECT_NAME})

# Configuration
install(DIRECTORY  config
        DESTINATION share/${PROJECT_NAME})

#######

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
