cmake_minimum_required(VERSION 3.0.2)
project(path_evaluator)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  common
)

catkin_package(
  INCLUDE_DIRS  include
  LIBRARIES     ${PROJECT_NAME}
  CATKIN_DEPENDS common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB metrics_src_files src/metrics/*.cpp)

add_library(${PROJECT_NAME}
  ${metrics_src_files}
  src/evaluator.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_smoothness_metric test/test_smoothness_metric.cpp)
  target_link_libraries(test_smoothness_metric ${PROJECT_NAME} ${catkin_LIBRARIES})

  catkin_add_gtest(test_tracking_error_metric test/test_tracking_error_metric.cpp)
  target_link_libraries(test_tracking_error_metric ${PROJECT_NAME} ${catkin_LIBRARIES})

  catkin_add_gtest(test_evaluator test/test_evaluator.cpp)
  target_link_libraries(test_evaluator ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
