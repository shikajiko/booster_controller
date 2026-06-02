cmake_minimum_required(VERSION 3.8)
project(booster_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(booster_interface REQUIRED)
find_package(booster_joint_interface REQUIRED)
find_package(action_interface REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/controller_manager/joint_manager.cpp
  src/node/controller_manager_node.cpp
  src/mode_transition_controller.cpp
  src/set_joint_controller.cpp
  src/torque_controller.cpp
  src/utils/command_constructor.cpp
  src/interpolator.cpp
  src/trajectory_controller.cpp
)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../booster_robotics_sdk/include
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  booster_interface
  booster_joint_interface
  action_interface
)

add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME}_lib)
ament_target_dependencies(main rclcpp rclcpp_action booster_interface booster_joint_interface action_interface)

add_executable(read_joints src/read_joints.cpp)
target_link_libraries(read_joints ${PROJECT_NAME}_lib)
ament_target_dependencies(read_joints rclcpp booster_interface)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib main read_joints
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_action booster_interface booster_joint_interface action_interface)

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
