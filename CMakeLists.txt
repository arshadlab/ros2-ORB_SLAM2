cmake_minimum_required(VERSION 3.5)
project(ros2_orbslam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")
LIST(APPEND CMAKE_MODULE_PATH /home/arshad/mapped/projects/orbslam_ws/ORB_SLAM3/Thirdparty/g2o/cmake_modules)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED )
find_package(Eigen  REQUIRED /home/arshad/mapped/projects/orbslam_ws/eigen/cmake)
find_package(G2O REQUIRED /home/arshad/mapped/projects/orbslam_ws/ORB_SLAM3/Thirdparty/g2o/cmake_modules)
find_package(OpenCV 4.4)
find_package(realsense2)
find_package(fmt CONFIG REQUIRED)

message("**** The value is:  ${EIGEN3_INCLUDE_DIRS}")

add_executable(slam 
  src/slam.cpp
  #src/monocular-slam-node.cpp
  src/rgbd-slam-node.cpp
  #  src/stereo/stereo-slam-node.cpp
)

target_link_libraries(slam
-lfmt
)

ament_target_dependencies(slam rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)


install(TARGETS slam 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
