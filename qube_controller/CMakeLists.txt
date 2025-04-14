cmake_minimum_required(VERSION 3.8)
project(qube_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(qube_controller_msgs)

include_directories(include)
include_directories(launch)

add_executable(qube_controller_node src/main.cpp include/pid_controller.hpp include/qube_controller_node.hpp)
ament_target_dependencies(qube_controller_node rclcpp std_msgs sensor_msgs qube_controller_msgs)




install(TARGETS
qube_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
launch
  DESTINATION share/${PROJECT_NAME}
)

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
