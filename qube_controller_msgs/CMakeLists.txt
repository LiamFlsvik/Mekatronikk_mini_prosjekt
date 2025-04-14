cmake_minimum_required(VERSION 3.8)
project(qube_controller_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetReference.srv"
)
include_directories(include)

add_executable(qube_controller_client_node src/main.cpp)
ament_target_dependencies(qube_controller_client_node rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(qube_controller_client_node "${cpp_typesupport_target}")

install(TARGETS 
qube_controller_client_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
