import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('dsr_robotiq2f'), 'config', 'gripper.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('auto_activate', default_value='false'),
        Node(package='dsr_robotiq2f', executable='gripper_node', name='gripper',
             namespace='', output='screen',
             parameters=[cfg, {'auto_activate': LaunchConfiguration('auto_activate')}]),
    ])
