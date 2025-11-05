from setuptools import setup

package_name = 'darp_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # Just the main package, DARP is external
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 wrapper for DARP multi-robot coverage path planning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_planner = darp_ros2.simple_planner_node:main',
        ],
    },
)
