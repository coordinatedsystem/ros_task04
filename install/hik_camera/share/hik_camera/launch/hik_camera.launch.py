# launch/hik_camera.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file_path = os.path.join(
        get_package_share_directory('hik_camera'),
        'config',
        'camera_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_camera',
            output='screen',
            parameters=[params_file_path]
        )
    ])