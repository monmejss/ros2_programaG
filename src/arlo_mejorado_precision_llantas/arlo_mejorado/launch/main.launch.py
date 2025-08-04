from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_arlo_path = get_package_share_directory('arlo_mejorado')
    world_path = os.path.join(package_arlo_path, "worlds", "laberinto_01.world")
    sdf_path = os.path.join(package_arlo_path, "models", "arlodrive", "model.sdf")

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, 
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so', 
                '-s', 'libgazebo_ros_force_system.so'],
            output='screen'
        )

    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', sdf_path,'-entity', 'arlo_mejorado_precision_llantas'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_model
    ])
