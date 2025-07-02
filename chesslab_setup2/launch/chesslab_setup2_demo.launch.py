from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    GroupAction,
    ExecuteProcess,
    SetEnvironmentVariable,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, 
    FindExecutable, 
    LaunchConfiguration, 
    PathJoinSubstitution, 
    EnvironmentVariable,
    PythonExpression
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import (OnExecutionComplete)

import os

from pathlib import Path, PurePath

from ament_index_python import get_package_share_directory

def generate_launch_description():

    # General arguments
    prefix = LaunchConfiguration("prefix", default='')
    gui = LaunchConfiguration("gui", default='False')
    launch_visualization = LaunchConfiguration('launch_gazebo', default=True)
    publish_markers_only = LaunchConfiguration('publish_markers_only', default=False)
    publish_camera_markers = LaunchConfiguration('publish_camera_markers', default=True)

    # UR Arguments
    ur_type = LaunchConfiguration("ur_type", default='ur3')

    tf_chess_frame = Node(
        package = "tf2_ros", 
        name="tf_chessboard_in_world",
        executable = "static_transform_publisher",
        output="screen",
        arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "flat_chessboard"]
        #arguments = ["0.0", "-0.37", "-0.0385", "1.5708", "0.0", "0.0", "base", "chess_frame"]
    )

    tf_camera = Node(
        package = "tf2_ros", 
        name="tf_camera_in_baselink",
        executable = "static_transform_publisher",
        output="screen",
        arguments = ["0.0", "-0.25", "0.34", "0.0", "0.0", "-2.5708", "flat_chessboard", "camera_color_optical_frame"]
    )

    chesslab_setup2_demo = Node(
        package="chesslab_setup2",
        executable="chesslab_setup2_demo",
        name="chesslab_setup2_demo",
        parameters=[
            {"publish_markers_only": publish_markers_only},
            {"publish_camera_markers": publish_camera_markers},
            {"use_sim_time": True}  # Añadir esto
        ],
        output="screen",
    )

    kinenikros2 = Node(
        package="kinenikros2",
        executable="kinenik_srv_server",
        name="kinenik",
        output="screen",
    )
 
    # Launch Gazebo
    ur_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('chesslab_setup2'), 'launch', 'chesslab_gz.launch.py']
        )),
        launch_arguments=[('ur_type', ur_type),
                          ('prefix', prefix),
                          ('gui', gui)],
        condition=IfCondition(launch_visualization)
    )
   
    # Arguments
    ld =  LaunchDescription()

    ld.add_action(tf_chess_frame)
    ld.add_action(tf_camera)
    ld.add_action(chesslab_setup2_demo)
    ld.add_action(ur_visualization)
    ld.add_action(kinenikros2)

    return ld


