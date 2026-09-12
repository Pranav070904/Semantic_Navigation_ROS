from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    grounding_dino = Node(
        package='nav_robot',
        executable='grounding_dino_node',
        name='grounding_dino_node',
        output='screen',
    )

    local_search = Node(
        package='nav_robot',
        executable='local_search_node',
        name='local_search_node',
        output='screen',
    )


    return LaunchDescription([
        grounding_dino,local_search,
    ])