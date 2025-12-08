from setuptools import setup

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rover_teleop_x56.launch.py']),
        ('share/' + package_name + '/config', ['config/x56_teleop.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sp',
    maintainer_email='sp@example.com',
    description='',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_driver = rover_control.motor_driver:main',
        ],
    },
)
