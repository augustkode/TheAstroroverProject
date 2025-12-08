from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    config_dir = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws/src/carto_localization/config'
    )

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_localization',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_localization.lua'
            ]
        ),

        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='carto_grid',
            output='screen',
            parameters=[{'resolution': 0.05}]
        )
    ])
