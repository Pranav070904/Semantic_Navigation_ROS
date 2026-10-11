from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='gemini-3.6-flash',
        description='Gemini model for reasoning_node. Can also be switched at runtime '
                    'with: ros2 param set /reasoning_node model <name>',
    )

    orchestrator = Node(
        package = 'nav_robot',
        executable='orch_node',
        name = 'orch_node',
        output='screen',
    )

    reasoning = Node(
        package='nav_robot',
        executable='reasoning_node',
        name='reasoning_node',
        output = 'screen',
        parameters=[{'model': LaunchConfiguration('model')}],
    )

    return LaunchDescription([
        model_arg, orchestrator, reasoning,
    ])
