from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2_system_monitor',
    version='0.2.2',
    packages=find_packages(
        include=('ros2_system_monitor', 'ros2_system_monitor.*')),
)
