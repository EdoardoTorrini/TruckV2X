cmake_minimum_required(VERSION 3.8)
project(tv2x_base)
set (CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tv2x_common REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)
include_directories(${CMAKE_CURRENT_LIST_DIR})

add_library(${PROJECT_NAME}_lib SHARED
  src/tv2x_base.cpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavSatHeading.msg"
  "msg/GpsSpeed.msg"
  # Add custom msg dependecies to standard ROS2 msgs
  DEPENDENCIES std_msgs sensor_msgs
 )

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
)

if (USE_EDF)
  message("EDF scheduling ENABLED")
  target_compile_definitions(${PROJECT_NAME}_lib PUBLIC USE_EDF)
else ()
  message("EDF scheduling DISABLE")
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
