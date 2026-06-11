import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true'  , choices=['true', 'false'], description='Use sim time'),
    DeclareLaunchArgument('use_real', default_value='true', choices=['true', 'false'], description='Use real robot'),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_real = LaunchConfiguration('use_real')
    rviz_path = get_package_share_directory('husky_localisation') + '/rviz/map.rviz'
    
    #Dead reckoning
    odom_node = Node(
        package='husky_localisation',
        executable='odom_node',
        name='odom_node',
        output='screen'
    )

    slam_node = Node(
        package='husky_localisation',
        executable='slam_node',
        name='slam_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
        condition=IfCondition(use_real)
    )

    #include lidar tf if necessary only
    lidar_tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        *ARGUMENTS,
        #odom_node,
        slam_node,
        rviz_node
    ])
