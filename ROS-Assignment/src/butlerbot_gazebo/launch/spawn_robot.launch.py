import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_butlerbot_description = FindPackageShare('butlerbot_description').find('butlerbot_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            choices=['true','false'],
            description="Use Gazebo Clock"
        ),

        DeclareLaunchArgument(
            name='robot_name', 
            description='Name of the robot'
        ),

        DeclareLaunchArgument(
            name='spawn_x', 
            default_value='0.0',
            description='Robot spawn position in X axis'
        ),

        DeclareLaunchArgument(
            name='spawn_y', 
            default_value='0.0',
            description='Robot spawn position in Y axis'
        ),

        DeclareLaunchArgument(
            name='spawn_z', 
            default_value='0.0',
            description='Robot spawn position in Z axis'
        ),
            
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='0.0',
            description='Robot spawn orientation'
        ),
    ]

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_butlerbot_description, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time'    : use_sim_time,
            'robot_name'      : robot_name,
            }.items()
    )

    spawn_entity_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-timeout', '120.0',
            '-topic', 'robot_description', 
            '-entity', robot_name,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
            ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription(
        declare_arguments + [
            robot_state_publisher,
            spawn_entity_node
        ]
    )
