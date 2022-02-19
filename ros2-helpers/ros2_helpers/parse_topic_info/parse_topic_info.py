#! /usr/bin/env python3
from ros2cli.node.strategy import NodeStrategy

from ros2_model import Connection, NodeArgs, Interface, Node, NodeName

def get_connection_info(connection: Interface):
    args = NodeArgs(node_name="get_connection_info", verbose=True)
    con = Connection(entity=connection)
    with NodeStrategy(args) as node:
        topic_types = connection.types
        topic_name = connection.name
        try:
            for info in node.get_publishers_info_by_topic(topic_name):
                node = Node(nodename=NodeName(name=info.name,
                                                namespace = info.node_namespace,
                                                full_name=('/').join(info.node_namespace, info.name))
                            )
                con.add_emission(node)
        except NotImplementedError as e:
            return str(e)
        try:
            for info in node.get_subscriptions_info_by_topic(topic_name):
                node = Node(nodename=NodeName(name=info.name,
                                namespace = info.node_namespace,
                                full_name=('/').join(info.node_namespace, info.name))
                            )
                con.add_reception(node)
        except NotImplementedError as e:
            return str(e)
    return con
