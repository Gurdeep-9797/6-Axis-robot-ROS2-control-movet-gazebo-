from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_hardware_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Team',
    maintainer_email='robot@example.com',
    description='Hardware Bridge - RELAY ONLY, NO AUTHORITY',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = robot_hardware_bridge.bridge_node:main',
            'moveit_adapter = robot_hardware_bridge.moveit_adapter_node:main',
        ],
    },
)
