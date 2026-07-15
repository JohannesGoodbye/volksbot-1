cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies finden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include-Verzeichnis für den Header bekannt machen
include_directories(include)

# Executable erstellen (C++ Node und C-Datei zusammenfügen!)
add_executable(obstacle_avoidance_node
  src/obstacle_avoidance_node.cc
  src/k-freespace.c
)

# Abhängigkeiten mit dem Executable verknüpfen
ament_target_dependencies(obstacle_avoidance_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Installation im Workspace-Target-Verzeichnis festlegen
install(TARGETS
  obstacle_avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()