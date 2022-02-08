from fcntl import F_GETLEASE
from .rosmodel import Node
import json as Json

class Message:
    def __init__(self, **my_dict):
        for key in my_dict:
            setattr(self, key, my_dict[key])
    def toJSON(self):
        dump =  Json.dumps(self, default=lambda o: o.__dict__,
            sort_keys=True, indent=4)
        return dump
    def dict(self):
        return Json.loads(self.toJSON())

class Pod(object):
    def __init__(self, name, ip, socket_id):
        self.name = name
        self.ip = ip
        self.socket_id = socket_id
        self.nodes = list()
    def add_node(self, node):
        if isinstance(node, Node):
            self.nodes.append(node)

class Process(object):
    def __init__(self, name, pid=None, state=False):
        self.name = name
        self.state = state
        self.pid = pid
    def set(self, **kwargs):
        for k,v in kwargs.items():
            if k in self.__dict__.keys():
                setattr(self, k, v)
    def start(self, pid):
        self.state = True
        self.pid = pid
    def stop(self):
        self.state = False
        self.pid = None
    def dict(self):
        return dict((key, value) for key, value in self.__dict__.items() if not callable(value) and not key.startswith('__'))
class Application(object):
    def __init__(self, name):
        self.name = name
        self.pods = list()
        self.processes = list()
    def add_pod(self, pod):
        if isinstance(pod, Pod):
            self.pods.append(pod)
    def find_node(self, node_name):
        for pod in self.pods:
            for node in pod.nodes:
                if node.name == node_name:
                    return pod, node
    def add_process(self, name):
        p = Process(name)
        self.processes.append(p)
    def find_process(self, name):
        for p in self.processes:
            if p.name == name:
                return p
    def update_process(self, name, **kwargs):
        p = self.find_process(name)
        p.set(**kwargs)
        print(p.dict())
