cmake_minimum_required(VERSION 3.8)
project(ros2_mecanum_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mecanum_navigation_node src/mecanum_navigation_node.cpp)
ament_target_dependencies(mecanum_navigation_node rclcpp geometry_msgs std_msgs)

install(TARGETS mecanum_navigation_node
  DESTINATION lib/${PROJECT_NAME})

# Python simulator node (PyGame). Installed as executable "mecanum_sim".
install(PROGRAMS scripts/mecanum_sim.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mecanum_sim)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
