import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    rviz_config_dir = os.path.join(get_package_share_directory('fastbot_slam'), 'rviz', 'fastbot_map.rviz')
   
    return LaunchDescription([
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'),
    ])
