import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # legg til launch-filer
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sp',
    maintainer_email='sp@todo.todo',
    description='Rover motor driver',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = rover_driver.motor_driver:main',
            'test_publisher = rover_driver.test_publisher:main',
            'imu_publisher = sensehat_imu.imu_publisher:main',
        ],
    },
)

