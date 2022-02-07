"""Launch talkers"""

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
        list_of_talkers.append(LifecycleNode(package='chatter_py', executable='talker',
                                name=f'talker_{i+1}', namespace='', output='screen'))
    load_nodes = GroupAction(
        actions= list_of_talkers
    )

    ld = LaunchDescription()
    ld.add_action(load_nodes)


    return ld
