import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')

    # File directories
    default_world_path = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')

    # Launch configuration variables 
    use_sim_time = LaunchConfiguration('use_sim_time')
    verbose = LaunchConfiguration('verbose')
    world = LaunchConfiguration('world')

    declare_arguments = [
        
        DeclareLaunchArgument(  
            name='verbose',
            default_value='true',
            description='Set "true" to increase messages written to terminal.'
        ),

        DeclareLaunchArgument(
            name='world',
            default_value=default_world_path,
            description='Gazebo world file of simulation environment'
        )
    ]


    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')), 
    )



    return LaunchDescription(
        declare_arguments + [
            start_gazebo
        ]
    )