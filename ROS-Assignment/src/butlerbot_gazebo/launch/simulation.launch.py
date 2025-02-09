import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Packages share directory
    pkg_butlerbot_gazebo = FindPackageShare('butlerbot_gazebo').find('butlerbot_gazebo')

    # Files paths 
    default_world_path = os.path.join(pkg_butlerbot_gazebo, 'worlds/french_door_cafe.world')
    models_path = os.path.join(pkg_butlerbot_gazebo, 'models')
    rviz_config = os.path.join(pkg_butlerbot_gazebo, 'rviz/display.rviz')

    world = LaunchConfiguration('world')
    verbose = LaunchConfiguration('verbose')
    use_rviz = LaunchConfiguration('use_rviz') 


    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = models_path

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))

    
    declare_arguments = [
        DeclareLaunchArgument(
            name='world',
            default_value=default_world_path, 
            description='Absolute path of gazebo WORLD file'
        ),

        DeclareLaunchArgument(  
            name='verbose',
            default_value='true',
            description='Set "true" to increase messages written to terminal.'
        ),

        DeclareLaunchArgument(  
            name='use_rviz',
            default_value='false',
            description='To open rviz tool'
        ),
    ]


    # Open simulation environment
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_butlerbot_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world'     : world,
            'verbose'   : verbose
        }.items()
    )

    # Spawn robot in simulation environment
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_butlerbot_gazebo, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={
            'robot_name': 'butlerbot',
            'spawn_x'   : '4.0',
            'spawn_y'   : '0.4',
            'spawn_z'   : '0.5',
            'spawn_yaw' : '1.5'
        }.items()
    )

    # Start RViz
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True
        }],
        condition=IfCondition(use_rviz)
    )


    return LaunchDescription(
        declare_arguments + [
            start_gazebo,
            spawn_robot,
            start_rviz
        ] 
    )