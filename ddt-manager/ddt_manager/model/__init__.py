from pydantic import BaseModel
from typing import List
import logging
from ros2_model import LifeCycleAction, LifeCycleNode
from ddt_utils.model import Pod

class DebugElement(BaseModel):
    node: str
    pod: str

class Application(BaseModel):
    name: str
    pods : List[Pod] = list()
    debug: List[DebugElement] = list()
    class Config:
        arbitrary_types_allowed = True
    def add_pod(self, pod):
        if isinstance(pod, Pod):
            self.pods.append(pod)
        else:
            logging.error(f"Application Model: Couldn't add pod to {self.name}")
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
    def add_lifecycle_nodes(self, pod_id, nodes):
        pod = self.find_pod(pod_id)
        for node in nodes:
            pod.add_lifecycle_node(node)
    def add_debug_ele(self, elemenent):
        self.debug.append(elemenent)
    def find_node(self, node_name, pod_name=None):
        def _find_node(pod):
            for node in pod.nodes:
                if node.name == node_name:
                    return pod, node
        if pod_name is None:
            for pod in self.pods:
                _find_node(pod)
        elif isinstance(pod_name, Pod):
            _find_node(pod_name)
        else:
            _find_node(self.find_pod(pod_name))
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
