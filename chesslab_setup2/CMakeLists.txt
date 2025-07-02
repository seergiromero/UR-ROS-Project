cmake_minimum_required(VERSION 3.8)
project(chesslab_setup2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur_description REQUIRED)
find_package(robotiq_85_description REQUIRED)
find_package(robotiq_85_gz REQUIRED)
find_package(chesslab_setup2_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(robotiq_85_gripper_server REQUIRED)

set(dependencies
    rclcpp
    visualization_msgs
    sensor_msgs
    geometry_msgs 
    std_srvs
    ur_description
    tf2_ros
    tf2
    robotiq_85_description
    robotiq_85_gz
    chesslab_setup2_interfaces
    robotiq_85_gripper_server
)

# include directories
include_directories(include)

add_executable(chesslab_setup2_demo src/chesslab_setup2_demo.cpp src/chesslab.cpp)
target_include_directories(chesslab_setup2_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(chesslab_setup2_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(chesslab_setup2_demo  ${dependencies})

# Install node and files
install(TARGETS chesslab_setup2_demo  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

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

ament_export_dependencies(${dependencies})
ament_package()
