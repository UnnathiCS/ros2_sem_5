cmake_minimum_required(VERSION 3.8)
project(myseepackage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install resource folders (if they exist)
install(PROGRAMS
  src/snake_slither_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

