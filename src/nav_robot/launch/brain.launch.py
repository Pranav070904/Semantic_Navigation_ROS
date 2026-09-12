from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
    )

    return LaunchDescription([
        orchestrator,reasoning,
    ])