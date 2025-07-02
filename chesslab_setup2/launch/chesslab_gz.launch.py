# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable
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
    TextSubstitution
)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    gui_config_file = LaunchConfiguration("gui_config_file")

    marker_size = LaunchConfiguration("marker_size")
    reference_frame = LaunchConfiguration("reference_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    camera_info = LaunchConfiguration("camera_info")
    image = LaunchConfiguration("image")
    image_is_rectified = False
    use_camera_info = True

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "chesslab_setup2_gz.rviz"]
    )

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
   
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            prefix,
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )


    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    gripper_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_85_gripper_controller", "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", "chesslab_robot",
            "-allow_renaming", "true",
        ],
    )

    set_ign_gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([FindPackageShare(runtime_config_package), 'models']),
            TextSubstitution(text=':'),
            EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value='/opt/ros/humble/share'),
        ]
    )

    world_gz_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "worlds", world_file]
    )
    world_gz_gui_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", gui_config_file]
    )

    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [
                " -r -v 4 ",
                world_gz_file,
                " --gui-config ",
                world_gz_gui_config_file
            ]
        }.items(),
        condition=IfCondition(gazebo_gui),
    )

    gz_launch_description_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -s -r -v 4 ", world_gz_file]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    # Make the /clock topic available in ROS
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/flat_chesslab/model/camera_with_support/model/camera/link/realsense_link/sensor/cameracolor/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/world/flat_chesslab/model/camera_with_support/model/camera/link/realsense_link/sensor/cameracolor/image@sensor_msgs/msg/Image[ignition.msgs.Image'
            ],
        remappings=[
            ('/world/flat_chesslab/model/camera_with_support/model/camera/link/realsense_link/sensor/cameracolor/camera_info', '/realsense_camera/camera_info'),
            ('/world/flat_chesslab/model/camera_with_support/model/camera/link/realsense_link/sensor/cameracolor/image', '/realsense_camera/image')
        ]
    )

    gripper_server_sim_node = Node(
          package='robotiq_85_gripper_server',
          executable='gripper_server_sim',
          output='screen')
    


    aruco_marker_publisher_params = {
        'marker_size': marker_size,
        'reference_frame': reference_frame,
        'camera_frame': camera_frame,
        'image_is_rectified': True,
        'use_camera_info': True,
    }

    marker_pub_node_launch = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_marker_publisher',
        parameters=[aruco_marker_publisher_params],
        remappings=[
        ('/camera_info',camera_info),
        ('/image', image)
        ],
        output={'both': 'screen'},
    )
    
    inverse_kinematics_service = Node(
    	package = "kinenikros2",
    	executable = "kinenik_srv_server",
    	output = "screen"
    )    
    
    aruco_pl2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("aruco_broadcaster"),  # Nombre del paquete
                "launch",
                "aruco_pl2.launch.py"  # Ruta al archivo de lanzamiento
            ])
        ]),
        # Pasar parámetros
        launch_arguments={
            "yaml_config": PathJoinSubstitution([
                FindPackageShare("aruco_broadcaster"),
                "config",
                "pl2esaii.yaml" 
            ]),
            "use_rviz": "false"  # Desactivar RViz si ya se usa en chesslab_gz
        }.items()
    )
        
    grasp_tf_node = Node(
    	package = "final_work",
    	executable = "custom_grasps_tf2_broadcaster",
    	output = "screen"
    )

    action_service = Node(
    	package = "final_work",
    	executable = "move_piece_action",
    	output = "screen"
    )

    kill_piece_action_node = Node(
    	package = "final_work",
    	executable = "kill_piece_action",
    	output = "screen"
    )

    castling_action_node = Node(
    	package = "final_work",
    	executable = "castling_action",
    	output = "screen"
    )   

    listener_node = Node(
    	package = "final_work",
    	executable = "listener_node",
    	output = "screen"
    )   

    piece_ubication_node = Node(
    	package = "final_work",
    	executable = "piece_ubication_node",
    	output = "screen"
    )   
    

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        gripper_controller_spawner_started,
        set_ign_gazebo_resource_path,
        gz_spawn_entity,
        gz_launch_description_with_gui,
        gz_launch_description_without_gui,
        gz_sim_bridge,
        gripper_server_sim_node,
        marker_pub_node_launch,
        inverse_kinematics_service,
        aruco_pl2_launch,
        grasp_tf_node,
        action_service,
    	# move_piece_action_node,
    	kill_piece_action_node, 
    	castling_action_node,   
        listener_node,
        piece_ubication_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur3e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="chesslab_setup2",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="chesslab_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="chesslab_setup2",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="chesslab_gz.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="flat_chesslab.sdf",
            description="Gazebo world file (absolute path or filename from the gazebosim worlds collection) containing a custom world.",
        )
    )
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
            default_value= "camera_link",
            description=('Complete name of the camera frame.')
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_info',
            default_value= '/realsense_camera/camera_info',
            description=('Remap of /camera_info.')
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'image',
            default_value= '/realsense_camera/image',
            description=('Remap of /image.')
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui_config_file",
            default_value="gui.config",
            description="Gazebo gui config file with e.g. the camera view",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
