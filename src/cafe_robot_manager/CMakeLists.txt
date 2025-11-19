cmake_minimum_required(VERSION 3.8)
project(cafe_robot_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(manager_node src/manager_node.cpp)
ament_target_dependencies(manager_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

install(TARGETS
  manager_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  config
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
