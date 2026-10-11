from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'dino_model',
        default_value='IDEA-Research/grounding-dino-tiny',
        description='Grounding DINO checkpoint. Try IDEA-Research/grounding-dino-base '
                    'for better detection now that headless Gazebo freed up VRAM.',
    )

    grounding_dino = Node(
        package='nav_robot',
        executable='grounding_dino_node',
        name='grounding_dino_node',
        output='screen',
        parameters=[{'model': LaunchConfiguration('dino_model')}],
    )

    local_search = Node(
        package='nav_robot',
        executable='frontier_search_node',
        name='frontier_search_node',
        output='screen',
    )


    return LaunchDescription([
        model_arg, grounding_dino, local_search,
    ])
