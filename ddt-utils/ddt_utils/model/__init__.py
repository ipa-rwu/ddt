from pydantic import BaseModel
import json as Json
from typing import List, Optional
from ros2_model import Interface, InterfaceType, Node, LifeCycleNode
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

class DebugElement(BaseModel):
    node: str
    pod: str

class DebugInterfaceInfo(BaseModel):
    interface: Interface
    destinations: List[DebugElement] = list()
    sources: List[DebugElement] = list()

class DebugNodeInfo(BaseModel):
    name: str
    interface_info : List[DebugInterfaceInfo] = list()

class DebugPodInfo(BaseModel):
    name: str
    nodes: List[DebugNodeInfo] = list()

    # return DebugNodeInfo
    def get_debug_node_info(self, node_id, **kwargs):
        logger = kwargs.get('logger')
        for node_modle in self.nodes:
            if node_id == node_modle.name:
                msg = f'Find Debug Node [{node_id}] in debug Pod [{self.name}]'
                logger.info(msg) if logger else print(msg)
                return node_modle
        else:
            msg = f"Can't find Debug Node [{node_id}] in debug Pod [{self.name}]"
            logger.info(msg) if logger else print(msg)
            return None

    def get_peers(self, app_model, topic, typee, **kwargs):
        logger = kwargs.get('logger')
        ignore_interfaces = kwargs.get("ignore_interfaces")
        if ignore_interfaces and topic in ignore_interfaces:
            return
        for pod in app_model.pods:
            for node in pod.nodes:
                if typee == InterfaceType.Publisher:
                    for interface in node.subscribers:
                        msg = f'[get_peers] Try to find Node [{node.nodename.full_name}] from Pod [{pod.name}] subscriber Topic [{interface.name}]'
                        logger.info(msg) if logger else print(msg)
                        if not(interface.name in ignore_interfaces) and interface.name == topic:
                            msg = f'[get_peers] Find Node [{node.nodename.full_name}] from Pod [{pod.name}] subscriber Topic [{interface.name}]!'
                            logger.info(msg) if logger else print(msg)
                            yield node, pod
                if typee == InterfaceType.Subscriber:
                    for interface in node.publishers:
                        msg = f'[get_peers] Try to find Node [{node.nodename.full_name}] from Pod [{pod.name}] publish Topic [{interface.name}]'
                        logger.info(msg) if logger else print(msg)
                        if not(interface.name in ignore_interfaces) and interface.name == topic:
                            msg = f'[get_peers] Find Node [{node.nodename.full_name}] from Pod [{pod.name}] publish Topic [{interface.name}]!'
                            logger.info(msg) if logger else print(msg)
                            yield node, pod
    def _update_dest_source(self,app_model,  node_model, target_node_id, **kwargs):
        # return DebugInterfaceInfos
        logger = kwargs.get('logger')
        if node_model.nodename.full_name == target_node_id:
            if len(node_model.publishers):
                for interface in node_model.publishers:
                    try:
                        nodes, pods = zip(*self.get_peers(app_model=app_model, topic=interface.name, typee=InterfaceType.Publisher, **kwargs))
                        nodes = list(nodes)
                        pods = list(pods)
                        msg = f'Find nodes {[node.nodename.full_name for node in nodes]} from {[pod.name for pod in pods]} subscriber [{interface.name}]'
                        logger.info(msg) if logger else print(msg)
                    except ValueError:
                        nodes = list()
                        pods  = list()
                    if len(nodes):
                        for i, node in enumerate(nodes):
                            if node.nodename.full_name == target_node_id:
                                nodes.pop(i)
                                pods.pop(i)
                        eles = [DebugElement(node=node.nodename.full_name, pod=pod.name) for node, pod in zip(nodes, pods)]
                        yield DebugInterfaceInfo(interface=interface, destinations = eles)

            if len(node_model.subscribers):
                for interface in node_model.subscribers:
                    try:
                        nodes, pods = zip(*self.get_peers(app_model=app_model, topic=interface.name, typee=InterfaceType.Subscriber, **kwargs))
                        nodes = list(nodes)
                        pods = list(pods)
                        msg = f'Find nodes {[node.nodename.full_name for node in nodes]} from {[pod.name for pod in pods]} publish to [{interface.name}]'
                        logger.info(msg) if logger else print(msg)
                    except ValueError:
                        nodes = list()
                        pods  = list()
                    if len(nodes):
                        for i, node in enumerate(nodes):
                            if node.nodename.full_name == target_node_id:
                                nodes.pop(i)
                                pods.pop(i)
                        eles = [DebugElement(node=node.nodename.full_name, pod=pod.name) for node, pod in zip(nodes, pods)]
                        yield DebugInterfaceInfo(interface=interface, sources = eles)

    def update_node_info(self, app_model, node_id, **kwargs):
        # return DebugNodeInfo
        logger = kwargs.get('logger')
        if not node_id in [node.name for node in self.nodes]:
            for pod in app_model.pods:
                for node in pod.nodes:
                    if node.nodename.full_name == node_id:
                        node_model =DebugNodeInfo(name=node_id,
                                                interface_info=list(self._update_dest_source(app_model=app_model,
                                                                                            node_model=node,
                                                                                            target_node_id=node_id,
                                                                                            **kwargs)))
                        msg = f'Add Debug Node [{node_id}] in Debug Pod [{self.name}]: {node_model.json()}'
                        logger.info(msg) if logger else print(msg)
                        msg = f'Before append: Debug Pod [{self.name}] has Debug Nodes: {[node.name for node in self.nodes]}'
                        logger.info(msg) if logger else print(msg)
                        self.nodes.append(node_model)
                        msg = f'Debug Pod [{self.name}] has Debug Nodes: {[node.name for node in self.nodes]}'
                        logger.info(msg) if logger else print(msg)

                        return node_model
            else:
                msg = f"Can't requested debug node [{node_id}] in any pods"
                logger.error(msg) if logger else print(msg)
                return None

    def update(self, app_model,  node_id, **kwargs):
        logger = kwargs.get('logger')
        def _update(name):
            if self.get_debug_node_info(name) is None:
                msg = f'Update Debug Pod [{self.name}], add debug node [{node_id}]'
                logger.info(msg) if logger else print(msg)
                self.update_node_info(app_model=app_model, node_id=name, **kwargs)
                msg = f'Finish Update! Debug Pod [{self.name}] has Debug Nodes: {[node.name for node in self.nodes]}'
                logger.info(msg) if logger else print(msg)
        if isinstance(node_id, str):
            _update(node_id)
        if isinstance(node_id, list):
            for id in node_id:
                _update(id)
