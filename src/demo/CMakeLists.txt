cmake_minimum_required(VERSION 3.5)
project(demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED) # 小车底盘控制需要

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

add_executable(lidar_client_demo src/lidar_client_demo.cpp)
ament_target_dependencies(lidar_client_demo "rclcpp" "sensor_msgs")

add_executable(imu_client_demo src/imu_client_demo.cpp)
ament_target_dependencies(imu_client_demo rclcpp std_msgs sensor_msgs)

# 小车底盘控制
include_directories(/usr/local/include/)
link_directories(/usr/local/lib)
add_executable(robot_demo src/robot_demo.cpp)
target_link_libraries(robot_demo ugv_sdk Threads::Threads)
target_compile_definitions(robot_demo PUBLIC ASIO_ENABLE_OLD_SERVICES)

install(TARGETS
        lidar_client_demo
        imu_client_demo
        robot_demo
        DESTINATION lib/${PROJECT_NAME})