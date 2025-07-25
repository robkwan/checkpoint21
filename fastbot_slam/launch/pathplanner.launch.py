import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory(
        'fastbot_slam'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory(
        'fastbot_slam'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory(
        'fastbot_slam'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory(
        'fastbot_slam'), 'config', 'recovery.yaml')

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=[('/cmd_vel', '/fastbot/cmd_vel'),
                        ('/odom', '/fastbot/odom')]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen',
            remappings=[('/cmd_vel', '/fastbot/cmd_vel'),
                        ('/odom', '/fastbot/odom')]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}])
    ])
