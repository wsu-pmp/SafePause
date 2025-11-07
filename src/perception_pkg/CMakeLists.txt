cmake_minimum_required(VERSION 3.8)
project(perception_pkg)

# explicit cpp version for clangd
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=gnu++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(safepause_msgs REQUIRED)

# # topic dependent
# find_package(sensor_msgs REQUIRED)


include_directories(
  include
)


add_executable(perception_node 
  src/perception_node.cpp
)

ament_target_dependencies(perception_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  safepause_msgs
  # # topic dependent
  # sensor_msgs
)

target_link_libraries(perception_node
  yaml-cpp
)

install(TARGETS
  perception_node
  DESTINATION lib/${PROJECT_NAME}
)


# install config/
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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
