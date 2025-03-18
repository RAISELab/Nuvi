cmake_minimum_required(VERSION 3.8)
project(test_tetra)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(test_tetra src/test_tetra.cpp)

ament_target_dependencies(test_tetra 
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
)

install(TARGETS
  test_tetra
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
