from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

  realsense2_camera_include_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_launch.py'
      ])
    ]),
    launch_arguments={
      "camera_namespace": '/',
      "rgb_camera.profile": '1920x1080x30',
    }.items(),
  )

  image_proc_node_launch = Node(
    package='image_proc',
    executable='image_proc',
    # name='image_proc',
    # namespace='/rs_camera',
    remappings=[
      ('image','/camera/color/image_raw'),
      ('camera_info', '/camera/color/camera_info'),
      ('image_rect', '/camera/color/image_rect'),
    ],
  )

  return [
    realsense2_camera_include_launch,
    image_proc_node_launch,
  ]

def generate_launch_description():
  declared_arguments = []
  
  return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])