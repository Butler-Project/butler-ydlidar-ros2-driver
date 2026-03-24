cmake_minimum_required(VERSION 3.8)
project(tmini_driver_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Protocol library (no ROS deps)
add_library(tmini_lidar src/tmini_lidar.cpp)
target_include_directories(tmini_lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(NODE_DEPS rclcpp rclcpp_components sensor_msgs)

# Component library
add_library(tmini_node_component SHARED src/tmini_node.cpp)
target_compile_definitions(tmini_node_component PRIVATE COMPONENT_ONLY)
ament_target_dependencies(tmini_node_component ${NODE_DEPS})
target_link_libraries(tmini_node_component tmini_lidar)
rclcpp_components_register_nodes(tmini_node_component "TminiDriverNode")

# Standalone executable
add_executable(tmini_driver_node src/tmini_node.cpp)
ament_target_dependencies(tmini_driver_node ${NODE_DEPS})
target_link_libraries(tmini_driver_node tmini_lidar)

install(TARGETS
  tmini_lidar
  tmini_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS tmini_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY params launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
