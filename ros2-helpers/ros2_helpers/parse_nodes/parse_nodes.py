#! /usr/bin/env python3
from ros2node.api import get_action_client_info
from ros2node.api import get_action_server_info
from ros2node.api import get_node_names
from ros2node.api import get_publisher_info
from ros2node.api import get_service_client_info
from ros2node.api import get_service_server_info
from ros2node.api import get_subscriber_info
from ros2cli.node.strategy import NodeStrategy

from ros2_model import Node, Interface, NodeName, NodeArgs
from ros2_helpers.utils import save_to_file

def parse_interface(Node, typee, value = list()):
    for v in value:
        interface = Interface(name=v.name, types=v.types)
        getattr(Node, typee).append(interface)

def get_names():
    args = NodeArgs(node_name="get_all_nodes")
    with NodeStrategy(args) as node:
        node_names = get_node_names(node=node, include_hidden_nodes=args.include_hidden_nodes)
        return node_names

class RunTimeNode(Node):
    pass

    def __init__(self, *, node: Node, **data):
        super().__init__(nodename=node.nodename, **data)

    class Config:
        arbitrary_types_allowed = True

    def get_publishers(self, node):
        info = get_publisher_info(
                    node=node, remote_node_name=self.nodename.full_name)
        parse_interface(self, "publishers", info)
    def get_subscribers(self, node):
        info = get_subscriber_info(
                    node=node, remote_node_name=self.nodename.full_name)
        parse_interface(self, "subscribers", info)
    def get_service_servers(self, node):
        info = get_service_server_info(
                    node=node, remote_node_name=self.nodename.full_name)
        parse_interface(self, "service_servers", info)
    def get_service_clients(self, node):
        info = get_service_client_info(
                    node=node, remote_node_name=self.nodename.full_name)
        parse_interface(self, "service_clients", info)
    def get_action_servers(self, node):
        info = get_action_server_info(
                    node=node, remote_node_name=self.nodename.full_name)
        parse_interface(self, "action_servers", info)
    def get_action_clients(self, node):
        info = get_action_client_info(
                    node=node, remote_node_name=self.nodename.full_name)
        parse_interface(self, "action_clients", info)

def parse(nodename: NodeName):
    args = NodeArgs(node_name=f'get_{nodename.full_name}_info')
    with NodeStrategy(args) as node:
        parsed_node = RunTimeNode(node=Node(nodename=nodename))
        parsed_node.get_publishers(node)
        parsed_node.get_subscribers(node)
        parsed_node.get_service_servers(node)
        parsed_node.get_service_clients(node)
        parsed_node.get_action_clients(node)
        parsed_node.get_action_servers(node)
        return parsed_node

def main(result_path):
    nodes = get_names()
    for n in nodes:
        node = NodeName(name=n.name, full_name = n.full_name, namespace=n.namespace)
        parsed_node = parse(node)
        save_to_file(result_path, n.full_name, parsed_node)

if __name__ == '__main__':
    main()