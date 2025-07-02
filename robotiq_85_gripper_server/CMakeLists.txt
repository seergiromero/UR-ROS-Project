cmake_minimum_required(VERSION 3.8)
project(robotiq_85_gripper_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robotiq_85_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_export_dependencies(rclcpp)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch  
    srv
  DESTINATION share/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GripperOrder.srv"
  "srv/GripperOpen.srv"
  "srv/GripperClose.srv"
  DEPENDENCIES rclcpp
 )

add_executable(gripper_server src/gripper_server.cpp)
add_executable(gripper_server_sim src/gripper_server_sim.cpp)
ament_target_dependencies(gripper_server rclcpp robotiq_85_msgs)
ament_target_dependencies(gripper_server_sim rclcpp rclcpp_action control_msgs)

install(TARGETS
  gripper_server
  gripper_server_sim
  DESTINATION lib/${PROJECT_NAME})
  
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gripper_server "${cpp_typesupport_target}")
target_link_libraries(gripper_server_sim "${cpp_typesupport_target}")

ament_package()
