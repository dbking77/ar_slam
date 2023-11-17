cmake_minimum_required(VERSION 3.5)

project(ar_slam)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${YAML_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++17 -ggdb")

# ar_slam_util
add_library(ar_slam_util ar_slam_util.cpp)
target_link_libraries(ar_slam_util ceres Eigen3::Eigen ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

# ar_slam
add_executable(ar_slam ar_slam.cpp)
target_link_libraries(ar_slam ar_slam_util ceres Eigen3::Eigen ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

# ar_loc
add_executable(ar_loc ar_loc.cpp)
target_link_libraries(ar_loc ar_slam_util ceres Eigen3::Eigen ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

# ar_detect
add_executable(ar_detect ar_detect.cpp)
target_link_libraries(ar_detect ar_slam_util ceres Eigen3::Eigen ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

# ar_gen
add_executable(ar_gen ar_gen.cpp)
target_link_libraries(ar_gen ${OpenCV_LIBS})
