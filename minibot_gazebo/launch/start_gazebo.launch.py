import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    world_name = DeclareLaunchArgument(
        "world_name", default_value="empty.world")
    
    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
        EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        '/usr/share/gazebo-11/models/:',
        str(Path(get_package_share_directory(
            'minibot_description')).parent.resolve()),
        ':',
        str(Path(get_package_share_directory(
            'minibot_gazebo')).parent.resolve()) + "/minibot_gazebo/models",
    ])

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'),
            '/launch/gzserver.launch.py'
        ]),
        launch_arguments={
            "verbose": "true",
            "physics": "ode",
            "lockstep": "true",
            "world": PathJoinSubstitution([
                        FindPackageShare('minibot_gazebo'),
                        'worlds',
                        LaunchConfiguration('world_name'),
            ])
        }.items(),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'),
            '/launch/gzclient.launch.py'
        ]),
    )

    return LaunchDescription([
        gz_resource_path,
        world_name,
        gz_server,
        gz_client,
    ])
