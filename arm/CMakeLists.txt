cmake_minimum_required(VERSION 3.5)
project(arm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Build
add_executable(arm_control_node src/arm_control_node.cpp)
ament_target_dependencies(arm_control_node
  dynamixel_sdk
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs  
)

add_executable(arm_send_cmd_node src/arm_send_cmd_node.cpp)
ament_target_dependencies(arm_send_cmd_node
  rclcpp
  sensor_msgs  
)

# Install
install(TARGETS
  arm_control_node
  arm_send_cmd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
