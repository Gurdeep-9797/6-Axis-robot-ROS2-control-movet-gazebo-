from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_analysis'

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
    description='Analysis and logging tools',
    license='MIT',
    entry_points={
        'console_scripts': [
            'accuracy_logger = robot_analysis.accuracy_logger:main',
        ],
    },
)
