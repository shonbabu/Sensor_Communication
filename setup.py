from setuptools import setup
import os
from glob import glob

package_name = 'sensor_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package for TCP sensor communication with GUI control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = sensor_communication.sensor_node:main',
            'mock_sensor_server = sensor_communication.mock_sensor_server:main',
            'sensor_gui = sensor_communication.sensor_gui:main',
        ],
    },
)
