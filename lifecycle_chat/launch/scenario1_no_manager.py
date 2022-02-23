from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

import sys

def generate_launch_description():

    ld = LaunchDescription()

    autostart = True
    number_of_talkers = 1
    use_sim_time = True
    logger = LaunchConfiguration("log_level")

    for arg in sys.argv:
        if arg.startswith("number_of_talkers:="):
            number_of_talkers = int(arg.split(":=")[1])
    print(number_of_talkers)

    lifecycle_nodes = []
    all_my_node_cmd = []

    declare_log_level = DeclareLaunchArgument("log_level", default_value=["WARN"], description="Logging level")

    node_name="tnode1"
    start_tnode1_cmd = launch_ros.actions.Node(
        package='lifecycle_chat',
        executable='lifecycle_talker',
        name=node_name,
        # arguments=['--ros-args', '--log-level', logger],
        output='log',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        )
    all_my_node_cmd.append(start_tnode1_cmd)
    lifecycle_nodes.append(node_name)

    node_name="fnode2"
    start_fnode2_cmd = launch_ros.actions.Node(
            package='lifecycle_chat',
            executable='lifecycle_forwarder',
            name=node_name,
            remappings=[('/lifecycle_forward', '/lifecycle_input')],
            output='log',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        #     arguments=['--ros-args', '--log-level', logger]
            )
    all_my_node_cmd.append(start_fnode2_cmd)
    lifecycle_nodes.append(node_name)


    node_name="lnode3"
    start_lnode3_cmd = launch_ros.actions.Node(
            package='lifecycle_chat',
            executable='lifecycle_listener',
            name=node_name,
            output='log',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        #     arguments=['--ros-args', '--log-level', logger]
            )
    all_my_node_cmd.append(start_lnode3_cmd)


    life_m =  f'lifecycle_manager'
    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name=life_m,
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                    {'bond_timeout': 4.0}],
        # arguments=['--ros-args', '--log-level', logger]
        )
    # all_my_node_cmd.append(start_lifecycle_manager_cmd)


    for cmd in all_my_node_cmd:
        ld.add_action(cmd)
    # ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(declare_log_level)

    return ld
