from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import GroupAction

import sys
def generate_launch_description():
    number_of_talkers = 1
    for arg in sys.argv:
        if arg.startswith("number_of_talkers:="):
            number_of_talkers = int(arg.split(":=")[1])
    print(number_of_talkers)
    list_of_talkers = []
    for i in range(number_of_talkers):
        list_of_talkers.append(LifecycleNode(package='lifecycle_chat', executable='lifecycle_talker',
                                name=f'lc_talker_{i+1}', namespace='', output='screen'))
    load_nodes = GroupAction(
        actions= list_of_talkers + [
                                    Node(package='lifecycle_chat', executable='lifecycle_listener', output='screen'),
                                    Node(package='lifecycle_chat', executable='lifecycle_service_client', output='screen')
                                    ]
    )

    ld = LaunchDescription()
    ld.add_action(load_nodes)


    return ld
