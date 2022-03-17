from launch import LaunchDescription
import launch_ros.actions

import sys
def generate_launch_description():
    autostart = True
    number_of_node_pair = 1
    use_sim_time = True
    for arg in sys.argv:
        if arg.startswith("number_of_node_pair:="):
            number_of_node_pair = int(arg.split(":=")[1])
    print(number_of_node_pair)

    lifecycle_nodes = []
    all_my_node_cmd = []

    for i in range(number_of_node_pair):
        tnode_name=f"tnode_{i}"
        start_tnode1_cmd = launch_ros.actions.Node(
            package='lifecycle_chat',
            executable='lifecycle_talker',
            name=tnode_name,
            # arguments=['--ros-args', '--log-level', logger],
        #     remappings=[('/lifecycle_chatter', f'/{tnode_name}/lifecycle_chatter')],
            output='log',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'no_bond': False},
                        {'talk_to': f'/{tnode_name}/lifecycle_chatter'}],
            )
        all_my_node_cmd.append(start_tnode1_cmd)
        lifecycle_nodes.append(tnode_name)

        fnode_name=f"fnode_{i}"
        start_fnode2_cmd = launch_ros.actions.Node(
                package='lifecycle_chat',
                executable='lifecycle_forwarder',
                name=fnode_name,
                remappings=[('/lifecycle_forward', f'/{fnode_name}/lifecycle_input')],
                output='log',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[{'no_bond': False},
                            {'sub_to': f'/{tnode_name}/lifecycle_chatter'}],
            #     arguments=['--ros-args', '--log-level', logger]
                )
        all_my_node_cmd.append(start_fnode2_cmd)
        lifecycle_nodes.append(fnode_name)


        lnode_name=f"lnode_{i}"
        start_lnode3_cmd = launch_ros.actions.Node(
                package='lifecycle_chat',
                executable='lifecycle_listener',
                name=lnode_name,
                parameters=[{'sub_to': f'/{fnode_name}/lifecycle_input'},
                            {'sub_transition': f'/{fnode_name}/lifecycle_transition'}],
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
                    {'bond_timeout': 4.0}])
    all_my_node_cmd.append(start_lifecycle_manager_cmd)


    ld = LaunchDescription()
    for cmd in all_my_node_cmd:
        ld.add_action(cmd)


    return ld
