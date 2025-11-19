import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_cafe_robot_manager = get_package_share_directory('cafe_robot_manager')
    # pkg_cafe_simulation = get_package_share_directory('cafe_simulation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    map_file_path = os.path.join(
        pkg_cafe_robot_manager,
        'maps',
        'cafe.yaml'
    )

    # map_file_path = "/home/muammer/baristar_ws/src/cafe_robot_manager/maps/cafe.yaml"

    # RViz henüz slam rvizini kullandım
    rviz_config_file = os.path.join(
        pkg_cafe_robot_manager,
        'rviz',
        'tb3_navigation2.rviz' 
    )

    params_file_path = os.path.join(
        pkg_cafe_robot_manager, 
        'config',               
        'nav2_params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=map_file_path)
    params_file = LaunchConfiguration('params_file', default=params_file_path)


    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'True'
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        nav2_bringup_launch,
        rviz_node
    ])