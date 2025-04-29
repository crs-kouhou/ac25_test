cmake_minimum_required(VERSION 3.8)
project(ac25_test)

set(CMAKE_C_COMPILER "/usr/bin/gcc-14")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-14")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # C++23を使う
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++23 -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#include pathを通す
include_directories(
  include
  ~/ac25_ws/ac_semi_2025/include  # 場合によって変えてね
)

# メッセージ型の生成
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Pose2d.msg
)
ament_export_dependencies(rosidl_default_runtime)

# 同一パッケージ内のメッセージ型を使いたいときのお約束
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)
# これも必要
target_link_libraries(test_node "${cpp_typesupport_target}")

install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME})

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
