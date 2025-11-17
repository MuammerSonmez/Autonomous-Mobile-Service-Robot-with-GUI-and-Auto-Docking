import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,RegisterEventHandler,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node 
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_cafe_simulation = get_package_share_directory('cafe_simulation')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_desc = get_package_share_directory('turtlebot3_description')

    cafe_models_path = os.path.join(pkg_cafe_simulation, 'models') # Cafe model klasörün (varsa)
    gazebo_models_path = os.path.join(pkg_turtlebot3_gazebo, 'models')
    
    # Mevcut path'leri koruyarak üzerine ekle
    os.environ['GAZEBO_MODEL_PATH'] = cafe_models_path + ':' + gazebo_models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')

    gazebo_model_path_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.environ['GAZEBO_MODEL_PATH']
    )

    world_file_path = os.path.join(pkg_cafe_simulation, 'world' , 'cafe.world')
    
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Gazebo dosyasının tam yolu'
    )

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros , 'launch' , 'gazebo.launch.py')
        ),

        launch_arguments={'world' : LaunchConfiguration('world')}.items(),
    )

    urdf_file = os.path.join(pkg_cafe_simulation, 'urdf', 'turtlebot3_burger.urdf')
    robot_description_content = Command(['cat ',urdf_file ])

    sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger' , 'model.sdf')


    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[{
            'use_sim_time':True,
            'robot_description':robot_description_content
        }]
    )

    start_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', sdf_file,
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_world_arg,
        gazebo_model_path_env,
        start_gazebo,
        start_robot_state_publisher,
        start_spawn_entity,
    ])