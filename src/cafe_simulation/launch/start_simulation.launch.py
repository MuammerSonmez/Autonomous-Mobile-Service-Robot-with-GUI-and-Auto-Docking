import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                           SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 

def generate_launch_description():
    pkg_cafe_simulation = get_package_share_directory('cafe_simulation')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Model path ayarları
    cafe_models_path = os.path.join(pkg_cafe_simulation, 'models')
    gazebo_models_path = os.path.join(pkg_turtlebot3_gazebo, 'models')
    
    os.environ['GAZEBO_MODEL_PATH'] = cafe_models_path + ':' + gazebo_models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    
    gazebo_model_path_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.environ['GAZEBO_MODEL_PATH']
    )
    
    # World dosyası
    world_file_path = os.path.join(pkg_cafe_simulation, 'world', 'cafe.world')
    
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Gazebo world dosyasının tam yolu'
    )
    
    # use_sim_time
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Gazebo
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )
    
    # TEMİZ URDF dosyası
    urdf_file = os.path.join(pkg_cafe_simulation, 'urdf', 'turtlebot3_burger.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc_content = infp.read()
    
    # Robot State Publisher
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc_content
        }]
    )
    
    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_burger',
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_world_arg,
        declare_use_sim_time,
        gazebo_model_path_env,
        start_gazebo,
        start_robot_state_publisher,
        spawn_entity,
    ])