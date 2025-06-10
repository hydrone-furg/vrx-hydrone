# https://drive.google.com/file/d/1jGBganF9v9IsWr_qm_3b6WALzRW_C3Mz/view

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_vrx_navigation = get_package_share_directory('vrx_navigation')
    ekf_config_path = os.path.join(pkg_vrx_navigation, 'config', 'ekf.yaml')
    navsat_config_path = os.path.join(pkg_vrx_navigation, 'config', 'navsat.yaml')

    return LaunchDescription([
        # EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[
                ('/odometry/filtered', '/localization')
            ]
        ),
        
        # GPS
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_config_path],
            remappings=[
                ('/imu/data', '/imu'),
                ('/gps/fix', '/wamv/sensors/gps/gps/fix'),
                ('/odometry/filtered', '/localization')
            ]
        ),
        
        # Waypoints
        Node(
            package='vrx_navigation',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            output='screen'
        )
    ])
