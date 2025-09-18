import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    package_name = 'nav2_odin'
    package_dir = get_package_share_directory(package_name) 
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='If true, use simulated clock'
    )
    declare_lidar_serial_port = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/rplidar',
        description='Specifying usb port to connected lidar'
    )

    # Paths to files
    robot_description_xacro_file = os.path.join(
        package_dir,
        'description',
        'robot.urdf.xacro'
    )
    twist_mux_params_file = os.path.join(
        package_dir, 
        'config', 
        'twist_mux.yaml'
    )
    slam_params_file = os.path.join(
        package_dir,
        'config',
        'mapper_params_online_async.yaml'
    )

    # robot_state_publisher setup
    robot_description_config = Command([
        'xacro ', 
        robot_description_xacro_file, 
        ' use_ros2_control:=false'
    ])

    params = {
        'robot_description': ParameterValue(robot_description_config, value_type=str), 
        'use_sim_time': use_sim_time
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Static transform for lidar (if not already in URDF)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.23', '0', '0.098', '0', '0', '0', 'base_link', 'lidar_frame']
    )

    # Ackermann steering node
    steering_node = Node(
        package='nav2_odin',
        executable='steering_node',
        name='steering_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/esp32',
            'baud_rate': 115200,
            'timeout': 0.5,
            'command_timeout': 0.5,
        }]
    )

    # Twist mux node
    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_params_file,
            {'use_stamped': True},
        ],
        remappings=[
            ('cmd_vel_out', 'cmd_vel'),
        ],
    )

    # RPLIDAR launch include
    node_rplidar_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py'
            )
        ]), 
        launch_arguments={
            'serial_port': lidar_serial_port, 
            'frame_id': 'lidar_frame'
        }.items()
    )

    # SLAM Toolbox with LiDAR odometry configuration
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {
            'use_sim_time': use_sim_time,
            'publish_odom': True,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan'
        }],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Nav2 launch with autostart disabled
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'params_file': os.path.join(package_dir, 'config', 'nav2_params.yaml'),
            'use_sim_time': use_sim_time,
            'autostart': 'false'
        }.items()
    )

    # Custom lifecycle manager with proper ordering
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'slam_toolbox',       
                'controller_server',   
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor'
            ],
            'bond_timeout': 20.0,
            'configure_timeout': 120.0,
            'activate_timeout': 120.0,
            'cleanup_timeout': 5.0,
            'deactivate_timeout': 5.0,
            'shutdown_timeout': 5.0
        }]
    )

    # TF debug nodes
    tf_debug1 = Node(
        package='tf2_ros',
        executable='tf2_echo',
        arguments=['map', 'base_footprint'],
        name='tf_debug_map_base'
    )

    tf_debug2 = Node(
        package='tf2_ros',
        executable='tf2_echo',
        arguments=['odom', 'base_footprint'],
        name='tf_debug_odom_base'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lidar_serial_port)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(static_tf_node)  # Added static transform for lidar
    ld.add_action(steering_node)
    ld.add_action(node_twist_mux)
    ld.add_action(node_rplidar_drive)
    ld.add_action(slam_toolbox)
    ld.add_action(TimerAction(
        period=10.0,  
        actions=[nav2_launch, lifecycle_manager]
    ))
    ld.add_action(tf_debug1)
    ld.add_action(tf_debug2)

    return ld