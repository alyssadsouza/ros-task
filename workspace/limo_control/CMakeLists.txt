cmake_minimum_required(VERSION 3.5)
project(limo_control)

# C++ settings:
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(pd_controller src/pd_controller.cpp)

ament_target_dependencies(pd_controller
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  pd_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()