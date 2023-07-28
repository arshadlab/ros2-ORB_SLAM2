cmake_minimum_required(VERSION 3.5)
project(ros2_orbslam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

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
find_package(Pangolin REQUIRED )
find_package(ORB_SLAM3 REQUIRED )
find_package(Eigen3 3.1.0 REQUIRED)
find_package(OpenCV 4.4)


add_executable(orbslam
  src/orbslam.cpp
)

target_link_libraries(orbslam
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
$ENV{ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
$ENV{ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib/libg2o.so
-lboost_serialization
-lcrypto
)

ament_target_dependencies(orbslam rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)


install(TARGETS orbslam
  DESTINATION lib/${PROJECT_NAME})

ament_package()
