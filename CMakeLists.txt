cmake_minimum_required(VERSION 3.8)
project(blucy)

# ============================
# Compiler warnings
# ============================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================
# Dependencies
# ============================
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(Protobuf REQUIRED)

# ============================
# Include directories
# ============================
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins
)

# ============================
# Plugin 1: ApplyBuoyancySystem
# ============================
add_library(ApplyBuoyancySystem SHARED
  plugins/ApplyBuoyancySystem.cc
)

target_link_libraries(ApplyBuoyancySystem
  ignition-gazebo6::ignition-gazebo6
  ignition-math6::ignition-math6
)

target_include_directories(ApplyBuoyancySystem PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins
)

set_target_properties(ApplyBuoyancySystem PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# ============================
# Plugin 2: ApplyHydrodynamicSystem
# ============================
add_library(ApplyHydrodynamicSystem SHARED
  plugins/ApplyHydrodynamicSystem.cc
)

target_link_libraries(ApplyHydrodynamicSystem
  ignition-gazebo6::ignition-gazebo6
  ignition-math6::ignition-math6
  ${Protobuf_LIBRARIES}   # Only needed if you really use protobuf
)

target_include_directories(ApplyHydrodynamicSystem PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins
)

set_target_properties(ApplyHydrodynamicSystem PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# ============================
# Plugin 3: ApplyThrusterSystem
# ============================
add_library(ApplyThrusterSystem SHARED
  plugins/ApplyThrusterSystem.cc
)

target_link_libraries(ApplyThrusterSystem
  ignition-gazebo6::ignition-gazebo6
  ignition-math6::ignition-math6
  ${Protobuf_LIBRARIES}   # Only if the thruster system uses protobuf
)

target_include_directories(ApplyThrusterSystem PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins
)

set_target_properties(ApplyThrusterSystem PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# ============================
# Installation
# ============================
install(TARGETS
  ApplyBuoyancySystem
  ApplyHydrodynamicSystem
  ApplyThrusterSystem
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes urdf launch config world
  DESTINATION share/${PROJECT_NAME}
)

install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
