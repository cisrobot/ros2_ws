cmake_minimum_required(VERSION 3.5)
project(nav2_custom)

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_bringup REQUIRED)

# Launch 및 Params 디렉토리 설치
install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# ament 패키지 등록
ament_package()
