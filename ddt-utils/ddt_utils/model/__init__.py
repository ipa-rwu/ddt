from pydantic import BaseModel
import json as Json
from typing import List, Optional
from ros2_model import Node, LifeCycleNode
import logging

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

class Process(BaseModel):
    name: str
    state = False
    pid: Optional[int]

    class Config:
        arbitrary_types_allowed = True

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
    # def dict(self):
    #     return dict((key, value) for key, value in self.__dict__.items() if not callable(value) and not key.startswith('__'))

class Pod(BaseModel):
    name: str
    ip: str
    socket_id: str
    nodes: List[Node] = list()
    lifecycle_nodes: List[LifeCycleNode] = list()
    processes: List[Process] = list()
    domain_id: int
    node_list: Optional[List[str]]
    lifecycle_node_list: Optional[List[str]]
    debug: List[Node] = list()
    class Config:
        arbitrary_types_allowed = True

    def get_node_names(self):
        for node in self.nodes:
            yield node.nodename.full_name
    def get_lifecycle_node_names(self):
        for node in self.nodes:
            yield node.nodename.full_name
    def add_node(self, node: Node):
        self.update()
        if node.nodename.full_name not in self.node_list:
            self.nodes.append(node)
    def add_lifecycle_node(self, node: LifeCycleNode):
        self.update()
        if node.nodename.full_name not in self.lifecycle_node_list:
            self.lifecycle_nodes.append(node)
    def add_process(self, name):
        p = Process(name)
        self.processes.append(p)
    def find_process(self, name):
        for p in self.processes:
            if p.name == name:
                return p
    def update(self):
        self.lifecycle_node_list = list(self.get_lifecycle_node_names())
        self.node_list = list(self.get_node_names())
    def find_node(self, name):
        self.update()
        if name in self.node_list:
            for node in self.nodes:
                if node.nodename.full_name == name:
                    return node
        else:
            logging.error(f"Couldn't find Node[{name}] in Pod[{self.name}]")
            raise KeyError
    def find_lifecycle_node(self, name):
        self.update()
        if name in self.lifecycle_node_list:
            print(self.lifecycle_nodes)
            for node in self.lifecycle_nodes:
                if node.nodename.full_name == name:
                    return node
        else:
            logging.error(f"Couldn't find Lifecycle Node[{name}] in Pod[{self.name}]")