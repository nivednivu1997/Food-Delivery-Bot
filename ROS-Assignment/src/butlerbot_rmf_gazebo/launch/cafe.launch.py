import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare('butlerbot_rmf_gazebo').find('butlerbot_rmf_gazebo')
    pkg_rmf = FindPackageShare('butlerbto_rmf').find('butlerbot_rmf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    failover_mode = LaunchConfiguration('failover_mode')

    declare_arguments = [

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use gazebo clock'
        ),
        
        DeclareLaunchArgument(
        name= 'failover_mode',
        default_value='false',
        description='Enable or disable failover mode'
        ),
    ]       

    # Launch RMF
    launch_rmf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rmf, 'launch', 'cafe.launch.py')),
        launch_arguments={
            'use_sim_time'  : use_sim_time,
            'failover_mode' : failover_mode
        }.items()
    )

    # Launch simulation
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'map_name'  : 'cafe',
        }.items()
    )

    
    return LaunchDescription(
        declare_arguments + [
            launch_rmf,
            launch_simulation
        ]
    )