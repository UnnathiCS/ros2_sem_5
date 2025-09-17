cmake_minimum_required(VERSION 3.8)
project(myseepackage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Find only the CMake‐exported packages you really link against
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# ------------------------------------------------------------------------------
# Add your own node (if you have one)
# ------------------------------------------------------------------------------
add_executable(my_node src/my_node.cpp)
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(my_node PUBLIC cxx_std_17)

ament_target_dependencies(my_node
  rclcpp
)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Install launch, model, config, worlds
# ------------------------------------------------------------------------------
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  model/
  DESTINATION share/${PROJECT_NAME}/model
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(FILES
  config/snake_bot_controllers.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# ------------------------------------------------------------------------------
# Install Python scripts
# ------------------------------------------------------------------------------
install(PROGRAMS
  src/snake_wiggler.py
  src/snake_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Testing (linting only)
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
