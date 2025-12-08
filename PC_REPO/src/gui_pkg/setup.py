from setuptools import setup

package_name = 'gui_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='augustkode',
    maintainer_email='augustkode@example.com',
    description='GUI node for Smarte Systemer rover',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gui_node = gui_pkg.ros2_gui:main',
        ],
    },
)
