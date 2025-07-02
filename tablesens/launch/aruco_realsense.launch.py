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

  marker_size_arg = LaunchConfiguration('marker_size')
  reference_frame_arg = LaunchConfiguration('reference_frame')
  camera_frame_arg = LaunchConfiguration('camera_frame')
  camera_info_newtopic_arg = LaunchConfiguration('camera_info')
  image_newtopic_arg = LaunchConfiguration('image')

  aruco_marker_publisher_params = {
    'marker_size': marker_size_arg,
    'reference_frame': reference_frame_arg,
    'camera_frame': camera_frame_arg,
    'image_is_rectified': True,
    'use_camera_info': True,
  }

  realsense_camera_include_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      FindPackageShare('tablesens'),'/launch','/realsense_camera.launch.py'
    ])
  )

  marker_pub_node_launch = Node(
    package='aruco_ros',
    executable='marker_publisher',
    name='aruco_marker_publisher',
    parameters=[aruco_marker_publisher_params],
    remappings=[
      ('/camera_info',camera_info_newtopic_arg),
      ('/image', image_newtopic_arg)
    ],
    output={'both': 'screen'},
  )

  rqt_image_view_node_launch = Node(
    package='rqt_image_view',
    executable='rqt_image_view',
    name='rqt_image_view',
  )

  return [
    realsense_camera_include_launch,
    marker_pub_node_launch,
    rqt_image_view_node_launch,
  ]


def generate_launch_description():
  declared_arguments = []

  declared_arguments.append(
    DeclareLaunchArgument(
      'marker_size',
      default_value= '0.026',
      description=('Marker size in m.')
    )
  )

  declared_arguments.append(
    DeclareLaunchArgument(
      'reference_frame',
      default_value= "camera_link",
      description=('Leave it empty and the pose will be published wrt param parent_name.')
    )
  )

  declared_arguments.append(
    DeclareLaunchArgument(
      'camera_frame',
      default_value= "camera_color_optical_frame",
      description=('Complete name of the camera frame.')
    )
  )

  declared_arguments.append(
    DeclareLaunchArgument(
      'camera_info',
      default_value= '/camera/color/camera_info',
      description=('Remap of /camera_info.')
    )
  )

  declared_arguments.append(
    DeclareLaunchArgument(
      'image',
      default_value= '/camera/color/image_rect',
      description=('Remap of /image.')
    )
  )

  return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])