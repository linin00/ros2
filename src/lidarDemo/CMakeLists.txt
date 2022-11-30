cmake_minimum_required(VERSION 2.8)

project(lidarDemo)

add_definitions(-std=c++11) # Use C++11
find_package(ydlidar_sdk REQUIRED)
message("\nydldar_sdk_include_dirs:\n${YDLIDAR_SDK_INCLUDE_DIRS}\n")
message("\nydlidar_sdk_libraries:\n${YDLIDAR_SDK_LIBRARIES}\n")
include_directories(
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)

set(lidar_demo_dir ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB APP_LIST "${lidar_demo_dir}/*.cpp")
foreach(child ${APP_LIST})
  string(REPLACE "${lidar_demo_dir}/" "" app_main ${child})
  string(REPLACE ".cpp" "" APP_NAME ${app_main})
  add_executable(${APP_NAME} ${app_main})
  link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})
  target_link_libraries(${APP_NAME} ${YDLIDAR_SDK_LIBRARIES})
  install(TARGETS ${APP_NAME} DESTINATION lib/${PROJECT_NAME})
endforeach()