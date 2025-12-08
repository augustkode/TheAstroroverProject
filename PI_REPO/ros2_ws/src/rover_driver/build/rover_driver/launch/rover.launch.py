from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket'
        ),
        Node(
            package='rover_driver',
            executable='motor_driver',
            name='motor_driver'
        )
    ])

