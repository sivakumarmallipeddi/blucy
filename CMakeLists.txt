cmake_minimum_required(VERSION 3.8)
project(blucy)

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 / Ignition dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)

# Plugin build
add_library(ApplyBuoyancySystem SHARED
  plugins/ApplyBuoyancySystem.cc
)

target_include_directories(ApplyBuoyancySystem PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins
  ${IGNITION-GAZEBO6_INCLUDE_DIRS}
  ${IGNITION-MATH6_INCLUDE_DIRS}
)

target_link_libraries(ApplyBuoyancySystem
  ignition-gazebo6::ignition-gazebo6
  ignition-math6::ignition-math6
)

# Install plugin library
install(TARGETS ApplyBuoyancySystem
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Install resources (models, worlds, etc.)
install(DIRECTORY meshes urdf launch config world
  DESTINATION share/${PROJECT_NAME}
)
install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()