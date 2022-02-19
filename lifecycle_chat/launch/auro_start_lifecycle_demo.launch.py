from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import GroupAction
import launch_ros.actions

import sys
def generate_launch_description():
    autostart = True
    number_of_talkers = 1
    use_sim_time = True
    for arg in sys.argv:
        if arg.startswith("number_of_talkers:="):
            number_of_talkers = int(arg.split(":=")[1])
    print(number_of_talkers)

    lifecycle_nodes = list()
    start_talkers = list()
    start_lifes = list()
    for i in range(number_of_talkers):
        node_name = f'lc_talker_{i+1}'
        start_talker_cmd = launch_ros.actions.Node(
                package='lifecycle_chat',
                executable='lifecycle_talker',
                name=node_name,
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                )
        start_talkers.append(start_talker_cmd)

        lifecycle_nodes = [node_name]

        life_m =  f'lifecycle_manager_{i+1}'
        start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=life_m,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
        start_lifes.append(start_lifecycle_manager_cmd)

    ld = LaunchDescription()
    for cmd in start_talkers + start_lifes:
        ld.add_action(cmd)
    # ld.add_action(start_lifecycle_manager_cmd)


    return ld
