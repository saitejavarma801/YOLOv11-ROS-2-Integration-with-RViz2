from setuptools import setup
import os
from glob import glob

package_name = 'yolo_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    # Install non-Python files
    data_files=[
        # Required for ament_python
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name,
         ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        # RViz config files
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='stv',
    maintainer_email='stv@todo.todo',

    description='YOLOv11 ROS 2 integration with RViz2 visualization',
    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'yolo_node = yolo_ros.yolo_node:main',
        ],
    },
)

