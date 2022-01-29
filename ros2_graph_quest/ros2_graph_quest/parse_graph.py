#! /usr/bin/env python3

import subprocess
import threading
import traceback
from pathlib import Path

import rclpy
import rclpy.action
from rclpy.qos import qos_profile_sensor_data
from ros2topic.api import get_msg_class
from ros2topic.api import get_topic_names_and_types

from rqt_graph.dotcode import RosGraphDotcodeGenerator
from rqt_graph.rosgraph2_impl import Graph
from qt_dotgraph.pydotfactory import PydotFactory

import rclpy.node

class ParserNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('graph_parser')
        timer_period = 2  # seconds

        self.declare_parameters(
            namespace='',
            parameters=[
                ('result_path', None),
                ('sampling_rate', 1)
            ]
        )

        result_path = self.get_parameter('result_path').get_parameter_value().string_value
        if result_path is None:
            result_path = pathlib.Path.home()

        self.timer = self.create_timer(self.get_parameter('sampling_rate').value, self.timer_callback)

        self.graph = None
        self.get_graph()
    # Get topology
    def get_graph(self):
        self.graph = Graph(self)
        self.graph.set_node_stale(5.0)
        self.graph.update()
        print(self.graph.nn_nodes)
        print(self.graph.nt_nodes)
        print(list(self.graph.nn_edges))

    def timer_callback(self):
        self.graph.update()
        print(self.graph.nn_nodes)
        # self.get_logger().info('Hello !')


def main():
    rclpy.init()
    node = ParserNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
