cmake_minimum_required(VERSION 3.5)
project(create3_project)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  control_msgs
  geometry_msgs
  sensor_msgs
  irobot_create_msgs
)

# IR intensity vector publisher
add_library(ir_publisher_lib SHARED)
target_sources(
  ir_publisher_lib
  PRIVATE
    src/ir_intensity_publisher.cpp
)
target_include_directories(ir_publisher_lib PUBLIC include)
ament_target_dependencies(ir_publisher_lib
  ${dependencies}
)

rclcpp_components_register_node(ir_publisher_lib
  PLUGIN "create3_project::IrIntensityPublisher"
  EXECUTABLE ir_intensity_publisher)

set(libraries_names
  ir_publisher_lib
)

install(TARGETS ${libraries_names}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${libraries_names})
ament_export_dependencies(${dependencies})

ament_package()
