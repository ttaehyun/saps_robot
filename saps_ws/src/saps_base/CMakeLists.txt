cmake_minimum_required(VERSION 3.8)
project(saps_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(saps_interfaces REQUIRED)

# --- VL53L8CX C/C++ Node Build Instructions ---
# 1. include 폴더(드라이버 헤더파일 위치)를 포함 디렉토리로 설정
include_directories(include)

# 1. 실행 파일 생성 및 컴파일 대상 파일 전부 묶어주기 (C파일들과 C++노드 통합)
add_executable(vl53l8cx_spi_node
  src/vl53l8cx_spi_node.cpp
  src/platform.c
  src/vl53l8cx_api.c
)

# 2. 인클루드 경로 바인딩
target_include_directories(vl53l8cx_spi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. ROS 2 링크 라이브러리 추가
ament_target_dependencies(vl53l8cx_spi_node
  rclcpp
  std_msgs
  sensor_msgs
)
# 4. install 타겟 지정 (ros2 run 명령어 인식용)
install(TARGETS
  vl53l8cx_spi_node
  DESTINATION lib/${PROJECT_NAME}
)
# install(DIRECTORY
#   model
#   DESTINATION share/${PROJECT_NAME}
# )

install(PROGRAMS
  scripts/icm20948_node.py
  scripts/mecanumBaseNode.py
  scripts/mpu6050_node.py
  scripts/uwb_receiver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
