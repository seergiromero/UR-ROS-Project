cmake_minimum_required(VERSION 3.8)
project(final_work)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_image REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(kinenikros2 REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(robotiq_85_gripper_server REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    ros_gz_bridge
    ros_gz_image
    ros_gz_sim
    sensor_msgs
    aruco_msgs
    tf2
    tf2_geometry_msgs
    ros_gz_interfaces
    yaml-cpp
    kinenikros2
    control_msgs
    trajectory_msgs
    robotiq_85_gripper_server
    ament_index_cpp
)



set(action_files
   "action/MovePiece.action"
   "action/KillPiece.action"
   "action/Castling.action"
  )

set(srv_files
  "srv/SetArucoId.srv"
)

set(msg_files
  "msg/PiecePose.msg"
  "msg/PiecePoseArray.msg"
  "msg/PieceUbicationArray.msg"
)



rosidl_generate_interfaces(${PROJECT_NAME}
   ${action_files}
   ${srv_files}
   ${msg_files}
   DEPENDENCIES
     geometry_msgs
     std_msgs

)

ament_export_dependencies(rosidl_default_runtime)

add_executable(listener_node src/listener_node.cpp)
ament_target_dependencies(listener_node ${dependencies})

add_executable(custom_grasps_tf2_broadcaster src/custom_grasps_tf2_broadcaster.cpp)
ament_target_dependencies(custom_grasps_tf2_broadcaster ${dependencies})

add_library(move_piece_controller
  src/move_piece_controller.cpp
  src/action_client_node.cpp
)
ament_target_dependencies(move_piece_controller ${dependencies})

add_executable(move_piece_action
  src/move_piece_action.cpp
  src/move_piece_controller.cpp
  src/action_client_node.cpp
)
ament_target_dependencies(move_piece_action ${dependencies})


add_executable(kill_piece_action
  src/kill_piece_action.cpp
  src/kill_piece_node.cpp
  src/move_piece_controller.cpp
  src/action_client_node.cpp
)
ament_target_dependencies(kill_piece_action ${dependencies})

add_executable(castling_action
  src/castling_action.cpp
  src/move_piece_controller.cpp
  src/action_client_node.cpp
)

ament_target_dependencies(castling_action ${dependencies})

add_executable(piece_ubication_node
  src/piece_ubication_node.cpp)
ament_target_dependencies(piece_ubication_node ${dependencies})

add_executable(chess_interface
  src/chess_interface.cpp)
ament_target_dependencies(chess_interface ${dependencies})


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link interfaces to targets
target_link_libraries(listener_node ${cpp_typesupport_target})
target_link_libraries(custom_grasps_tf2_broadcaster ${cpp_typesupport_target})
target_link_libraries(move_piece_controller ${cpp_typesupport_target})
target_link_libraries(move_piece_action ${cpp_typesupport_target})
target_link_libraries(kill_piece_action ${cpp_typesupport_target})
target_link_libraries(castling_action ${cpp_typesupport_target})
target_link_libraries(piece_ubication_node ${cpp_typesupport_target})
target_link_libraries(chess_interface ${cpp_typesupport_target})



install(TARGETS
  listener_node
  move_piece_controller
  custom_grasps_tf2_broadcaster
  move_piece_action
  kill_piece_action
  castling_action
  piece_ubication_node
  chess_interface
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(${dependencies})
ament_package()
