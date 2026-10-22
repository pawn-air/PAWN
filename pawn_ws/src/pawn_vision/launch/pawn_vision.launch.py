import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1',
        description='ID of the drone (e.g. 1, 2) for multi-drone namespacing'
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='multiplexed',
        description='Processing mode: multiplexed (30-40 FPS) or synchronous'
    )

    drone_id = LaunchConfiguration('drone_id')
    mode = LaunchConfiguration('mode')

    vision_node = Node(
        package='pawn_vision',
        executable='vision_node',
        name='pawn_vision_node',
        namespace=['drone_', drone_id],
        parameters=[{
            'drone_id': drone_id,
            'mode': mode,
        }],
        output='screen'
    )

    return LaunchDescription([
        drone_id_arg,
        mode_arg,
        vision_node
    ])
