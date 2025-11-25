cmake_minimum_required(VERSION 3.8)
project(training_recorder)

# explicit cpp version for clangd
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=gnu++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(safepause_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(
  include
)


# topic_handler
add_library(topic_handler_lib
    src/topic_handler.cpp
)
ament_target_dependencies(topic_handler_lib
  rclcpp
  rosbag2_cpp
)


# recorder_node
add_library(recorder_node_lib
    src/recorder_node.cpp
)
ament_target_dependencies(recorder_node_lib
  rclcpp
  rosbag2_cpp
  rosidl_runtime_cpp
)


# recording_controller.cpp
add_executable(recording_controller 
  src/recording_controller.cpp
)
ament_target_dependencies(recording_controller
  rclcpp
  std_srvs
  safepause_msgs
  sensor_msgs
)
target_link_libraries(recording_controller
  topic_handler_lib
  recorder_node_lib
)



# install headers
install(DIRECTORY include/
  DESTINATION include/
)


# install libs
install(TARGETS
  topic_handler_lib
  recorder_node_lib
  EXPORT training_recorderTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# install executables
install(TARGETS
  recording_controller
  DESTINATION lib/${PROJECT_NAME}
)


# planning_scene_collision_adder
ament_python_install_package(${PROJECT_NAME})

# Install Python node as executable
install(
  PROGRAMS
    training_recorder/planning_scene_collision_adder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME planning_scene_collision_adder
)


# install launch/
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
