from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  gripper_server_node = Node(
          package='robotiq_85_gripper_server',
          executable='gripper_server',
          output='screen')

  ld = LaunchDescription()
  ld.add_action(gripper_server_node)

  return ld