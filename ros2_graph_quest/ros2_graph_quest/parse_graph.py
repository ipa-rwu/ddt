#! /usr/bin/env python3
import subprocess
import threading
import traceback
from pathlib import Path

import rclpy
import rclpy.action

from rqt_graph.dotcode import RosGraphDotcodeGenerator
from rqt_graph.rosgraph2_impl import Graph
from qt_dotgraph.pydotfactory import PydotFactory

import rclpy.node
class ParserNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('graph_parser')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('result_path', ''),
                ('application_name', "test"),
                ('sampling_rate', 10),
                ('mode', "NODE_TOPIC_ALL_GRAPH")
            ]
        )

        self.result_path = Path(self.get_parameter('result_path').get_parameter_value().string_value).resolve()
        if self.result_path is None or self.result_path =="":
            self.result_path = Path.home()
        self.timer = self.create_timer(self.get_parameter('sampling_rate').value, self.timer_callback)
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.app_name = self.get_parameter('application_name').get_parameter_value().string_value

        self.graph = Graph(self)
        self.graph.set_node_stale(5.0)

        self.get_logger().info(f"Path: {self.result_path}, name: {self.app_name}, Mode: {self.mode}")

    def gen_dotcode(self, graph):
        dotcode_factory = PydotFactory()
        dotcode_generator = RosGraphDotcodeGenerator(self)
        dotcode = dotcode_generator.generate_dotcode(
            rosgraphinst=graph,
            ns_filter="",
            topic_filter="",
            graph_mode=self.mode,
            cluster_namespaces_level=5,
            dotcode_factory=dotcode_factory,
            orientation="LR",
            quiet=True,
        )
        return dotcode

    def write_into_dot(self, graph):
        # Write topology
        with open(self.result_path / f'{self.app_name}.dot', "w") as f:
            f.write(self.gen_dotcode(graph))
        subprocess.run(["dot", "-Tpng", "-O", self.result_path / f'{self.app_name}.dot'])

    def timer_callback(self):
        self.graph.update()
        self.get_logger().info(str(self.graph.nn_nodes))
        self.write_into_dot(self.graph)

def main():
    rclpy.init()
    node = ParserNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
