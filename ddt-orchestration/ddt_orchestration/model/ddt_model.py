from .rosmodel import Node

class Pod(object):
    def __init__(self, name, ip, socket_id):
        self.name = name
        self.ip = ip
        self.socket_id = socket_id
        self.nodes = list()
    def add_node(self, node):
        if isinstance(node, Node):
            self.nodes.append(node)

class Application(object):
    def __init__(self, name):
        self.name = name
        self.pods = list()
    def add_pod(self, pod):
        if isinstance(pod, Pod):
            self.pods.append(pod)
    def find_node(self, node_name):
        for pod in self.pods:
            for node in pod.nodes:
                if node.name == node_name:
                    return pod, node


