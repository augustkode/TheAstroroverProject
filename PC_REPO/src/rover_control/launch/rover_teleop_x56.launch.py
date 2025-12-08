from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node'),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=['config/x56_teleop.yaml']
        ),
        Node(package='rover_control', executable='motor_driver', name='motor_driver'),
    ])

config = os.path.join(get_package_share_directory('rover_control'), 'config', 'x56_teleop.yaml')
Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    parameters=[config]
)