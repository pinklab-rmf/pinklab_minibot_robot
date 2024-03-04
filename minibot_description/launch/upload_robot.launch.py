import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    ld = LaunchDescription()

    is_sim = DeclareLaunchArgument("is_sim", default_value="false")
    prefix = DeclareLaunchArgument("prefix", default_value="")
    
    frame_prefix = PythonExpression([
        '"', LaunchConfiguration('prefix'), '"'
    ])
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('prefix'),
        output='screen',
        parameters=[{
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('minibot_description'),
                        'urdf',
                        'robot.urdf.xacro',
                    ]),
                    ' is_sim:=', LaunchConfiguration('is_sim'),
                    ' prefix:=', LaunchConfiguration('prefix'),
                ]),
        }],
        remappings=remappings
    )
    
    ld.add_action(is_sim)
    ld.add_action(prefix)
    ld.add_action(rsp_node)

    return ld
