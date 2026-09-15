import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

def generate_launch_description():

    pkg_path = get_package_share_directory('nav_robot')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.expanduser('~/robot_worlds/worlds/small_house/small_house.world')
    map_yaml_file = os.path.expanduser('~/robot_worlds/worlds/small_house/maps/turtlebot3_waffle_pi/map.yaml')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time':True
        }]
    )

    # Launch Gazebo with world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'nav_robot',
        '-topic', 'robot_description',
        '-x', '-2.0',
        '-y', '0.0',
        '-z', '0.2'
    ],
    parameters=[
                {'use_sim_time': True}
            ],
    output='screen',
    )


#     slam = Node(
#         package='slam_toolbox',
#         executable='sync_slam_toolbox_node',
#         name='slam_toolbox',
#         output='screen',
#         parameters=[
#             os.path.join(pkg_path, 'config', 'slam_toolbox.yaml'),
#             {'use_sim_time': True}
#         ]
#     )
#     configure_slam = TimerAction(
#         period=10.0,
#         actions=[
#             ExecuteProcess(
#                 cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
#                 output='screen'
#             )
#     ]
# )

#     activate_slam = TimerAction(
#         period=13.0,  # 3 seconds after configure
#         actions=[
#             ExecuteProcess(
#                 cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
#                 output='screen'
#             )
#         ]
#     )
    # Bridge ROS2 <-> Gazebo topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '--ros-args',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
#     nav2_bringup = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource([
#         os.path.join(
#             get_package_share_directory('nav2_bringup'),
#             'launch',
#             'bringup_launch.py'
#         )
#     ]),
#     launch_arguments={
#         'map': '/home/pranav/robot_worlds/worlds/small_house/maps/turtlebot3_waffle_pi/map.yaml',
#         'use_sim_time': 'True',
#         'params_file': os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
#         'autostart': 'true'
#     }.items()
# )

    lifecycle_nodes = ['map_server', 'amcl', 'controller_server', 'planner_server',
                    'behavior_server', 'bt_navigator', 'velocity_smoother']

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
            {'yaml_filename': map_yaml_file, 'use_sim_time': True}
        ]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'nav2_params.yaml'), {'use_sim_time': True}]
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'nav2_params.yaml'), {'use_sim_time': True}]
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'nav2_params.yaml'), {'use_sim_time': True}]
    )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'nav2_params.yaml'), {'use_sim_time': True}]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'nav2_params.yaml'), {'use_sim_time': True}]
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'nav2_params.yaml'), {'use_sim_time': True}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': lifecycle_nodes
        }]
    )

    static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    ,parameters=[
                    {'use_sim_time': True}
                ],
    )

    static_tf_scan = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'base_scan'],
    parameters=[{'use_sim_time': True}]
    )

    initial_pose_pub = TimerAction(
    period=15.0,  # wait for AMCL to be ready
    actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once',
                '/initialpose',
                'geometry_msgs/msg/PoseWithCovarianceStamped',
                '{header: {frame_id: "map"}, '
                'pose: {pose: {position: {x: -2.0, y: 0.0, z: 0.0}, '
                'orientation: {w: 1.0}}, '
                'covariance: [0.25, 0, 0, 0, 0, 0, '
                '0, 0.25, 0, 0, 0, 0, '
                '0, 0, 0, 0, 0, 0, '
                '0, 0, 0, 0, 0, 0, '
                '0, 0, 0, 0, 0, 0, '
                '0, 0, 0, 0, 0, 0.06853891945200942]}}'
            ],
            output='screen'
        )
    ]
)
    

    return LaunchDescription([
    gazebo,
    robot_state_publisher,
    spawn_robot,
    bridge,
    static_tf,
    static_tf_scan,
    map_server_node,
    amcl_node,
    controller_node,
    planner_node,
    behavior_node,
    bt_navigator_node,
    velocity_smoother_node,
    lifecycle_manager,
    initial_pose_pub,
])