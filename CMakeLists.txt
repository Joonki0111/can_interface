cmake_minimum_required(VERSION 3.8)
project(autoware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)

add_executable(autoware_interface src/autoware_interface.cpp)

ament_target_dependencies(
  autoware_interface 
  rclcpp 
  std_msgs
  can_msgs
  autoware_auto_vehicle_msgs
  autoware_auto_control_msgs
)

install(TARGETS
  autoware_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
