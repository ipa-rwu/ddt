#! /usr/bin/env python3
import rclpy.node
from ros2node.api import get_action_client_info
from ros2node.api import get_action_server_info
from ros2node.api import get_node_names
from ros2node.api import get_publisher_info
from ros2node.api import get_service_client_info
from ros2node.api import get_service_server_info
from ros2node.api import get_subscriber_info
from ros2node.api import parse_node_name
from ros2_graph_quest.rosmodel import Node, Interface
from pathlib import Path
from shutil import rmtree

def parse_interface(Node, typee, value = list()):
    for v in value:
        interface = Interface(name=v.name, types=v.types)
        getattr(Node, typee).append(interface)

class RunTimeNode(Node):
    pass

    def __init__(self, *, node: Node, **data):
        super().__init__(name=node.name, full_name = node.full_name, namespace = node.namespace, **data)

    class Config:
        arbitrary_types_allowed = True

    def get_publishers(self, node):
        info = get_publisher_info(
                    node=node, remote_node_name=self.full_name)
        parse_interface(self, "publishers", info)
    def get_subscribers(self, node):
        info = get_subscriber_info(
                    node=node, remote_node_name=self.full_name)
        parse_interface(self, "subscribers", info)
    def get_service_servers(self, node):
        info = get_service_server_info(
                    node=node, remote_node_name=self.full_name)
        parse_interface(self, "service_servers", info)
    def get_service_clients(self, node):
        info = get_service_client_info(
                    node=node, remote_node_name=self.full_name)
        parse_interface(self, "service_clients", info)
    def get_action_servers(self, node):
        info = get_action_server_info(
                    node=node, remote_node_name=self.full_name)
        parse_interface(self, "action_servers", info)
    def get_action_clients(self, node):
        info = get_action_client_info(
                    node=node, remote_node_name=self.full_name)
        parse_interface(self, "action_clients", info)

class ParserNodes(rclpy.node.Node):
    def __init__(self):
        super().__init__('node_parser')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('result_path', ''),
                ('application_name', "test"),
                ('sampling_rate', 10),
            ]
        )

        self.result_path = Path(self.get_parameter('result_path').get_parameter_value().string_value).resolve()
        if self.result_path is None or self.result_path =="":
            self.result_path = Path.home() / 'tmp' / 'node_parser'
        self.result_path.mkdir(parents=True, exist_ok=True)
        self.timer = self.create_timer(self.get_parameter('sampling_rate').value, self.timer_callback)

        self.get_logger().info(f"Path: {self.result_path}")

    def timer_callback(self):
        node_names = get_node_names(node=self)
        folder = Path(self.result_path.resolve())
        rmtree(folder, ignore_errors=True)
        for n in node_names:
            # new_n = parse_node_name(n.name)
            # self.get_logger().info(f"raw name: {n.name}, {n.namespace}, {n.full_name}")
            # self.get_logger().info(f"parse_node_name: {new_n.name}, {new_n.namespace}, {new_n.full_name}")
            parsed_node = RunTimeNode(node=Node(name=n.name,
                                                namespace=n.namespace,
                                                full_name=n.full_name))
            parsed_node.get_publishers(self)
            parsed_node.get_subscribers(self)
            parsed_node.get_service_servers(self)
            parsed_node.get_service_clients(self)
            # parsed_node.get_action_clients(self)
            # parsed_node.get_action_servers(self)
            f_path = Path(str(folder) + f'{n.full_name}.json')
            f_path.parent.mkdir(parents=True, exist_ok=True)
            with open(f_path, "w+") as f:
                f.write(parsed_node.json(indent=4, sort_keys=True))

def main():
    rclpy.init()
    node = ParserNodes()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
