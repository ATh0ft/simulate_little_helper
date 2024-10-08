cmake_minimum_required(VERSION 3.8)
project(simulate_little_helper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(rviz2 REQUIRED)
find_package(ros_gz_bridge REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(fortress_trajectory_controller src/joint_trj_controller.cpp)
ament_target_dependencies(fortress_trajectory_controller rclcpp std_msgs rosgraph_msgs control_msgs rclcpp_action)



ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/ign_hooks.dsv.in")

install(DIRECTORY launch 
                  worlds
        DESTINATION share/${PROJECT_NAME})
install(TARGETS fortress_trajectory_controller DESTINATION lib/${PROJECT_NAME})

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
