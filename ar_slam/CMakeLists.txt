cmake_minimum_required(VERSION 3.8)
project(ar_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)

# ROS2
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Messages
find_package(ar_slam_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# LIBRARIES
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${YAML_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++17 -ggdb")

# ar_slam_util
add_library(ar_slam_util SHARED
  src/ar_slam_util.cpp
)
ament_target_dependencies(ar_slam_util
  ar_slam_interfaces
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(ar_slam_util ceres Eigen3::Eigen ${OpenCV_LIBS})

add_library(ar_slam_components SHARED
  src/ar_slam.cpp
  src/aruco_detector.cpp
  src/bag_recorder.cpp
  src/image_loader.cpp
  src/merge_detections.cpp
)
ament_target_dependencies(ar_slam_components
  ar_slam_interfaces
  cv_bridge
  geometry_msgs
  rclcpp
  rosbag2_cpp
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(ar_slam_components
  ar_slam_util
  rclcpp_components::component
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${YAML_CPP_LIBRARIES}
)
rclcpp_components_register_nodes(ar_slam_components "ar_slam::ArSlam")
rclcpp_components_register_nodes(ar_slam_components "ar_slam::ArucoDetector")
rclcpp_components_register_nodes(ar_slam_components "ar_slam::BagRecorder")
rclcpp_components_register_nodes(ar_slam_components "ar_slam::ImageLoader")
rclcpp_components_register_nodes(ar_slam_components "ar_slam::MergeDetections")

set(AR_SLAM_EXTERNAL_LINK_LIBS ceres Eigen3::Eigen ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

# ar_slam_cli
add_executable(ar_slam_cli src/ar_slam_cli.cpp)
target_link_libraries(ar_slam_cli ar_slam_util ceres Eigen3::Eigen ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

# ar_loc
add_executable(ar_loc src/ar_loc.cpp)
target_link_libraries(ar_loc ar_slam_util ceres Eigen3::Eigen ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

# ar_gen
add_executable(ar_gen src/ar_gen.cpp)
target_link_libraries(ar_gen ${OpenCV_LIBS})

install(
  TARGETS ar_slam_components ar_slam_util
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ar_gen ar_loc ar_slam_cli
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY resources/images
  DESTINATION share/${PROJECT_NAME}/resources
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(util_test test/util_test.cpp)
  target_include_directories(util_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(util_test
    std_msgs
  )
  target_link_libraries(util_test ar_slam_util ${AR_SLAM_EXTERNAL_LINK_LIBS})
endif()

ament_python_install_package( ${PROJECT_NAME} )
install(
  PROGRAMS scripts/load_images_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME load_images_client
)

install(
  PROGRAMS launch/ar_slam_launch.py launch/demo_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
