cmake_minimum_required(VERSION 3.8)
project(eta_estimator)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 종속성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(ETA_human_calculator src/ETA_human_calculator.cpp)

ament_target_dependencies(ETA_human_calculator
  rclcpp
  nav2_msgs
  nav_msgs
  geometry_msgs
  rclcpp_action
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  ETA_human_calculator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
