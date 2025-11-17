import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('cafe_robot_manager'),
        'config',
        'slam_params.yaml'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Simulasyon zmaanı kullan'
    )

    start_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name = 'slam_toolbox',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time' : LaunchConfiguration('use_sim_time')}
        ]
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('cafe_robot_manager'),
        'rviz',
        'slam.rviz'
    )

    start_rviz_node = Node(
        package = 'rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}],
        output='screen'
    )


    return LaunchDescription([
        declare_use_sim_time_arg,
        start_slam_toolbox_node,
        start_rviz_node
    ])