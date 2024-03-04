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
    robot_name = DeclareLaunchArgument("robot_name", default_value="minibot_1")
    robot_prefix = DeclareLaunchArgument("robot_prefix", default_value="")
    
    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('minibot_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments={
            'is_sim': 'true',
            'prefix': LaunchConfiguration('robot_prefix')
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        namespace=LaunchConfiguration('robot_prefix'),
        output='screen',
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-timeout', '20.0',
            '-x', LaunchConfiguration('x_pose', default='0.0'),
            '-y', LaunchConfiguration('y_pose', default='0.0'),
            '-z', '0.0',
            '-Y', LaunchConfiguration('yaw', default='0.0'),
            '-package_to_model'
        ],
        prefix="bash -c 'sleep 0.0; $0 $@' ",
        parameters=[{
            "use_sim_time": True,
            "namespace": LaunchConfiguration('robot_prefix')
        }],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('robot_prefix'),
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "controller_manager"]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('robot_prefix'),
        arguments=["base_controller",
                   "--controller-manager", "controller_manager"]
        
    )
    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        ),
        robot_name,
        robot_prefix,
        upload_robot,
        spawn_robot,
    
    ])
