from setuptools import setup

package_name = 'robot_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=['robot_driver'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/robot_driver_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='ROS2 Humble driver for履带式小车底盘',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver_node = robot_driver.robot_driver_node:main',
        ],
    },
)
