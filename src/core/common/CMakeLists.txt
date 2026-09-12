cmake_minimum_required(VERSION 3.0.2)
project(common)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED)

# Header-only: no library is built, only the include directory is exported.
catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
