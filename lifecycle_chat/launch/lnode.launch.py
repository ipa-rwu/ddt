from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

import sys

def generate_launch_description():

    ld = LaunchDescription()
    logger = LaunchConfiguration("log_level")

    all_my_node_cmd = []

    declare_log_level = DeclareLaunchArgument("log_level", default_value=["WARN"], description="Logging level")

    i = 0
    fnode_name=f"fnode_{i}"
    node_name=f"debug_lnode_{i}"
    start_lnode3_cmd = launch_ros.actions.Node(
            package='lifecycle_chat',
            executable='lifecycle_listener',
            name=node_name,
            parameters=[{'sub_to': f'/{fnode_name}/lifecycle_input'},
                        {'sub_transition': f'/{fnode_name}/lifecycle_transition'}],
            output='log',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        #     arguments=['--ros-args', '--log-level', logger]
            )
    all_my_node_cmd.append(start_lnode3_cmd)

    for cmd in all_my_node_cmd:
        ld.add_action(cmd)
    # ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(declare_log_level)

    return ld
