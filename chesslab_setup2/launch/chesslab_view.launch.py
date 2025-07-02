from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    # Uncomment if only rviz will be used
    gui_arg = DeclareLaunchArgument(name='gui', default_value='True',description='Flag to enable joint_state_publisher_gui')

    tf_prefix = LaunchConfiguration('tf_prefix', default='')

    # UR description
    # Type/series of used UR robot.
    # choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"]
    ur_type = LaunchConfiguration('ur_type', default='ur3')
    description_package = LaunchConfiguration('description_package', default='chesslab_setup2')
    description_file = LaunchConfiguration('description_file', default='chesslab.urdf.xacro')
    
    marker_size = LaunchConfiguration("marker_size", default='0.026')
    reference_frame = LaunchConfiguration("reference_frame", default="camera_link")
    camera_frame = LaunchConfiguration("camera_frame", default="camera_link")
    camera_info = LaunchConfiguration("camera_info", default='/realsense_camera/camera_info')
    image = LaunchConfiguration("image", default='/realsense_camera/image')

    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("chesslab_setup2"), "rviz", "chesslab_setup2_demo.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )
    #robot_description = {"robot_description": robot_description_content}
    #https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": False}, robot_description],      
    )

    joint_state_publisher = Node(
        condition = UnlessCondition(LaunchConfiguration('gui')),
            name='joint_state_publisher',
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[robot_description, {'publish_default_positions': False}, {'source_list': ['ur_joint_states']}],
    )

    # joint_state_publisher_gui = Node(
    #     condition = IfCondition(LaunchConfiguration('gui')),
    #         package='joint_state_publisher_gui',
    #         executable='joint_state_publisher_gui',
    #         name='joint_state_publisher_gui'
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        on_exit=Shutdown(),
    )
    
    inverse_kinematics_service = Node(
        package="kinenikros2",
        executable="kinenik_srv_server",
        output="screen"
    )

    aruco_pl2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("aruco_broadcaster"),  
                "launch",
                "aruco_pl2.launch.py"  
            ])
        ]),
        launch_arguments={
            "yaml_config": PathJoinSubstitution([
                FindPackageShare("aruco_broadcaster"),
                "config",
                "pl2esaii.yaml"  
            ]),
            "use_rviz": "false"  
        }.items()
    )

    grasp_tf_node = Node(
        package="final_work",
        executable="custom_grasps_tf2_broadcaster",
        output="screen"
    )
    
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

    # Gripper launch files
    robotiq_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("robotiq_85_driver"),
                "launch",
                "gripper_driver.launch.py"
            ])
        ]),
    )

    robotiq_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("robotiq_85_gripper_server"),
                "launch",
                "robotiq_85_gripper_server.launch.py"
            ])
        ]),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("tablesens"),
                "launch",
                "aruco_realsense.launch.py"
            ])
        ]),
    )

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ]),
        launch_arguments={
            "initial_joint_controller": "joint_trajectory_controller",
            "ur_type": ur_type,      
            'launch_rviz': "false",           
            "robot_ip": "10.5.20.79" # USE THE CORRECT IP
        }.items()
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

    # Description
    ld =  LaunchDescription()

    ld.add_action(gui_arg)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(joint_state_publisher_gui)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)

    ld.add_action(marker_pub_node_launch)
    ld.add_action(robotiq_driver_launch)
    ld.add_action(robotiq_server_launch)
    ld.add_action(camera_launch)
    ld.add_action(inverse_kinematics_service)
    ld.add_action(aruco_pl2_launch)
    ld.add_action(grasp_tf_node)
    ld.add_action(action_service)
    ld.add_action(kill_piece_action_node)
    ld.add_action(castling_action_node)   
    ld.add_action(listener_node)
    ld.add_action(ur_control_launch)

    ld.add_action(piece_ubication_node)

    return ld

