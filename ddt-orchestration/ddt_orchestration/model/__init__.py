from pydantic import BaseModel, ValidationError
import json as Json
from typing import List, Optional
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

class Interface(BaseModel):
    name: str
    types: List[str]

class Node(BaseModel):
    name: str
    namespace: str
    full_name : str
    publishers:      List[Interface]=list()
    subscribers:     List[Interface]=list()
    action_servers:  List[Interface]=list()
    action_clients:  List[Interface]=list()
    service_servers: List[Interface]=list()
    service_clients: List[Interface]=list()

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
    processes: List[Process]
    domain_id: int

    class Config:
        arbitrary_types_allowed = True

    def get_node_names(self):
        for node in self.nodes:
            yield node.full_name
    def add_node(self, node: Node):
        if node.full_name not in self.get_node_names():
            self.nodes.append(node)
    def add_process(self, name):
        p = Process(name)
        self.processes.append(p)
    def find_process(self, name):
        for p in self.processes:
            if p.name == name:
                return p


class Application(BaseModel):
    name: str
    pods : List[Pod] = list()
    class Config:
        arbitrary_types_allowed = True
    def add_pod(self, pod):
        if isinstance(pod, Pod):
            self.pods.append(pod)
        else:
            logging.error(f'Could add pod to {self.name}')
    def all_pods(self):
        all = []
        try:
            for pod in self.pods:
                all.append(pod.name)
        except TypeError:
            return []
        return all
    def add_nodes(self, pod_id, nodes):
        pod = self.find_pod(pod_id)
        for node in nodes:
            pod.add_node(node)
    def find_node(self, node_name):
        for pod in self.pods:
            for node in pod.nodes:
                if node.name == node_name:
                    return pod, node
    def find_processes(self, name):
        for pod in self.pods:
            for process in pod.processes:
                if process.name == name:
                    yield pod, process
    def find_pod(self, pod_id):
        for p in self.pods:
            if p.name == pod_id:
                return p
    def update_process(self, name, **kwargs):
        p = self.find_process(name)
        p.set(**kwargs)
        print(p.dict())
    def remove_pod(self, pod):
        _pod = pod
        if type(pod) is str:
            _pod = self.find_pod(pod)
        if _pod:
            self.pods.remove(pod)
        else:
            logging.ERROR(f'Could not find {pod}, so cannot delete')
