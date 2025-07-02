cmake_minimum_required(VERSION 3.8)
project(tablesens)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

set(msg_files
   "msg/MarkerFix.msg"
   "msg/MarkerFixList.msg"
)

set(srv_files
   "srv/ConfigureFromService.srv"
)

set(tablesens_interfaces_dependencies # Add packages that above message depend on
  "tf2"
  "tf2_ros"
  "std_msgs"
  "aruco_msgs"
  "geometry_msgs"
)

rosidl_generate_interfaces(${PROJECT_NAME}
   ${msg_files}
   ${srv_files}
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include ${rclcpp_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${rclcpp_LIBRARY_DIRS})

add_executable(tablecalibrator ./src/cameraposfinder.cpp ./src/tablecalibrator.cpp)
ament_target_dependencies(tablecalibrator rclcpp tf2_eigen ${tablesens_interfaces_dependencies})

install(
  TARGETS tablecalibrator
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# # Per poder utilitzar missatges i serveis creats en el mateix pkg:
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(tablecalibrator "${cpp_typesupport_target}")

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
