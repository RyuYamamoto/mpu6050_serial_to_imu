cmake_minimum_required(VERSION 3.5)
project(mpu6050_serial_to_imu)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost COMPONENTS system thread filesystem)

add_executable(mpu6050_imu_node src/mpu6050_serial_to_imu_node.cpp)
ament_target_dependencies(mpu6050_imu_node serial rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS
  mpu6050_imu_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
