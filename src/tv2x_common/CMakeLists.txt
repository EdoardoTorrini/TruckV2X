cmake_minimum_required(VERSION 3.8)
project(tv2x_common)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS cmake/tv2x_common-extras.cmake)