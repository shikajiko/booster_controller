cmake_minimum_required(VERSION 3.8)
project(booster_joint_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(booster_interface REQUIRED)
find_package(keisan REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/joint_manager/joint_manager/joint_manager.cpp
  src/node/joint_manager_node.cpp
  src/robot_mode_validator/robot_mode_validator.cpp
  src/utils/command_constructor.cpp
  src/utils/joint_mapping.cpp
)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../booster_robotics_sdk/include
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  booster_interface
  keisan
)

add_executable(joint_state_manager_node src/main.cpp)
target_link_libraries(joint_state_manager_node ${PROJECT_NAME}_lib)
ament_target_dependencies(joint_state_manager_node rclcpp booster_interface keisan)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib joint_state_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp booster_interface keisan)

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
