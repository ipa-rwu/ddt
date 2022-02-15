from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from pathlib import Path

def generate_launch_description():
    ld = LaunchDescription()
    result_path = LaunchConfiguration('result_path')
    application_name = LaunchConfiguration('application_name')
    sampling_rate = LaunchConfiguration('sampling_rate')
    mode = LaunchConfiguration('mode')
    logger = LaunchConfiguration("log_level")

    declare_result_path = DeclareLaunchArgument('result_path', default_value=str(Path.home()))
    declare_application_name = DeclareLaunchArgument('application_name',  default_value="test")
    declare_sampling_rate = DeclareLaunchArgument('sampling_rate', default_value="10")
    declare_mode = DeclareLaunchArgument('mode', default_value="NODE_NODE_GRAPH") # NODE_NODE_GRAPH, NODE_TOPIC_GRAPH, NODE_TOPIC_ALL_GRAPH
    declare_log_level = DeclareLaunchArgument("log_level", default_value=["WARN"], description="Logging level")

    graph_node = launch_ros.actions.Node(
            package='ros2_graph_quest',
            name='rosgraph_creator',
            executable='graph_parser',
            parameters=[
                {'result_path': result_path},
                {'application_name': application_name},
                {'sampling_rate': sampling_rate},
                {'mode': mode}],
            arguments=['--ros-args', '--log-level', logger],
            output='screen')

    ld.add_action(declare_result_path)
    ld.add_action(declare_application_name)
    ld.add_action(declare_sampling_rate)
    ld.add_action(declare_mode)
    ld.add_action(declare_log_level)

    ld.add_action(graph_node)
    return ld
