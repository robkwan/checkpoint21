import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    cartographer_config_dir = os.path.join(
        get_package_share_directory('fastbot_slam'), 'config')
    configuration_basename = 'cartographer.lua'

    return LaunchDescription([

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            remappings=[('/scan', '/fastbot_1/scan'),
                        ('/odom', '/fastbot_1/odom')]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.025', '-publish_period_sec', '1.0']
        ),
    ])
