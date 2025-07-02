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

  yaml_config = LaunchConfiguration('yaml_config')

  tablecalibrator_node_launch = Node(
      package='tablesens',
      executable='tablecalibrator',
      name='tablecalibrator',
      arguments=[],
      parameters=[yaml_config],
      output={'both': 'screen'},
  )

  return [
      tablecalibrator_node_launch,
  ]


def generate_launch_description():
  declared_arguments = []
  
  declared_arguments.append(
    DeclareLaunchArgument(
      'yaml_config',
      default_value= os.path.join(
        get_package_share_directory('tablesens'),
        'config',
        # 'tablesetup.yaml'
        # 'tablesetup-ioc-aruco.yaml'
        # 'tablesetup-ioc.yaml'
        'tablesetup-esaii-pl2.yaml'
      ),
      description=('Absolute path to yaml config file.')
    )
  )

  return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])