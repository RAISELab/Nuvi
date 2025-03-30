cmake_minimum_required(VERSION 3.5)
project(tracking_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


add_executable(goal_navigator src/goal_navigator.cpp)
ament_target_dependencies(goal_navigator 
  rclcpp
  geometry_msgs
  nav2_msgs
  rclcpp_action
)

add_executable(tracking_sim src/tracking_sim.cpp)
ament_target_dependencies(tracking_sim rclcpp nav_msgs ament_index_cpp)

install(TARGETS
  tracking_sim
  goal_navigator
  DESTINATION lib/${PROJECT_NAME}/
  )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()