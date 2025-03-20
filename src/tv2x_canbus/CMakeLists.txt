cmake_minimum_required(VERSION 3.8)
project(tv2x_canbus)
set (CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(USE_EDF OFF)
set(DEBUG_CAN ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tv2x_base REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  ackermann_msgs
  tv2x_base
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

if (USE_EDF)
  message("EDF scheduling ENABLED")
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_EDF)
else ()
  message("EDF scheduling DISABLE")
endif()

if (DEBUG_CAN)
  message("DEBUG CAN scheduling ENABLED")
  target_compile_definitions(${PROJECT_NAME} PUBLIC DEBUG_CAN)
else()
  message("DEBUG CAN scheduling DISABLE")
endif()

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS tv2x_canbus
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
