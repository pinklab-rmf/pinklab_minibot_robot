import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("minibot_navigation2"),
                                    'config', 'mapper_params.yaml'),
    )

    slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )
    
        # RViz2 설정 파일 경로 설정
    rviz_config_file = os.path.join(
        get_package_share_directory('minibot_navigation2'),
        'rviz',
        'map_building.rviz'
    )

    start_rviz_cmd = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )
    
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(slam_toolbox_node)
    ld.add_action(start_rviz_cmd)  # RViz 실행 액션 추가


    return ld
