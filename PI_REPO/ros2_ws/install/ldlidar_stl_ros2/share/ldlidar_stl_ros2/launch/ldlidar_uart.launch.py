from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ldlidar_node',
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD06',
                'port_name': '/dev/ttyS0',
                'port_baudrate': 230400,
                'frame_id': 'laser_frame',
                'topic_name': 'scan',
                'publish_freq': 10.0,
                'point_num': 1440,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 0.0,
                'angle_crop_max': 0.0,
            }]
        )
    ])
