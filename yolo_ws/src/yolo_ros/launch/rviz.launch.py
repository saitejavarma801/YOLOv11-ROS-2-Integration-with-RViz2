from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('yolo_ros')
    rviz_config = os.path.join(pkg_path, 'rviz', 'yolo.rviz')

    return LaunchDescription([

        # YOLO node
        Node(
            package='yolo_ros',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),

        # RViz2 with saved config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
