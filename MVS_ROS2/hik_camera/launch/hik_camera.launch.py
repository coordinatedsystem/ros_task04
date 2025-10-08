# launch/hik_camera.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取参数文件路径
    params_file_path = os.path.join(
        get_package_share_directory('hik_camera'),
        'config',
        'camera_params.yaml'
    )

    return LaunchDescription([
        # 启动海康相机节点
        Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_camera',
            output='screen',
            parameters=[params_file_path]
        ),

        # 启动 RViz2 并加载配置文件
        ExecuteProcess(
            cmd=[
                FindExecutable(name='rviz2'),
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('hik_camera'),
                    'config',
                    'rviz_config.rviz'
                ])
            ],
            name='rviz2',
            output='screen'
        )
    ])
