#!/usr/bin/env python3
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from pathlib import Path

def generate_launch_description():
    ld = LaunchDescription()
    result_path = LaunchConfiguration('result_path')
    sampling_rate = LaunchConfiguration('sampling_rate')
    logger = LaunchConfiguration("log_level")

    declare_result_path = DeclareLaunchArgument('result_path', default_value=str(Path.home()/ 'tmp' / 'node_parser'))
    declare_sampling_rate = DeclareLaunchArgument('sampling_rate', default_value="10")
    declare_log_level = DeclareLaunchArgument("log_level", default_value=["WARN"], description="Logging level")

    graph_node = launch_ros.actions.Node(
            package='ros2_graph_quest',
            name='node_parser',
            executable='node_parser',
            parameters=[
                {'result_path': result_path},
                {'sampling_rate': sampling_rate}],
            arguments=['--ros-args', '--log-level', logger],
            output='screen')

    ld.add_action(declare_result_path)
    ld.add_action(declare_sampling_rate)
    ld.add_action(declare_log_level)

    ld.add_action(graph_node)
    return ld
