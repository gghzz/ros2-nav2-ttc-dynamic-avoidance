cmake_minimum_required(VERSION 3.8)
project(moving_obstacle_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_definitions(
  ${GAZEBO_CXX_FLAGS}
)

add_library(${PROJECT_NAME} SHARED
  src/moving_obstacle_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

ament_package()