from pydantic import BaseModel
from typing import List
import logging
from ddt_utils.model import DebugPodInfo, Pod

class Application(BaseModel):
    name: str
    pods : List[Pod] = list()
    debug_pods : List[Pod] = list()
    debug_infocenter: List[DebugPodInfo] = list()
    class Config:
        arbitrary_types_allowed = True
    def add_pod(self, pod):
        if isinstance(pod, Pod):
            self.pods.append(pod)
        else:
            logging.error(f"Application Model: Couldn't add pod to {self.name}")
    def add_debug_pod(self, pod):
        if isinstance(pod, Pod):
            self.debug_pods.append(pod)
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
    def find_node(self, node_name, pod_name=None):
        def _find_node(pod):
            for node in pod.nodes:
                print("find_node_name: ", node.nodename.full_name, node_name)
                if node.nodename.full_name == node_name:
                    return node
        if pod_name is None:
            for pod in self.pods:
                return _find_node(pod)
        elif isinstance(pod_name, Pod):
            return _find_node(pod_name)
        elif isinstance(pod_name, str):
            return _find_node(self.find_pod(pod_name))
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

    def update_debug_model(self, pod_id, node_id, **kwargs):
        logger = kwargs.get('logger')
        pod_model = self.get_debug_pod_info(pod_id)
        if pod_model is None:
            pod_model = DebugPodInfo(name=pod_id, nodes=list())
            msg = f'Add new Debug Pod [{pod_id}]'
            logger.info(msg) if logger else print(msg)
        msg = f'Debug Pod [{pod_id}] exist, will add Debug Node [{node_id}]'
        logger.info(msg) if logger else print(msg)
        pod_model.update(app_model=self, node_id = node_id, **kwargs)
        self.debug_infocenter.append(pod_model)

    def get_debug_pod_info(self, pod_id, **kwargs):
        logger = kwargs.get('logger')
        for pod_info in self.debug_infocenter:
            if pod_info.name == pod_id:
                msg = f'Find debug pod [{pod_id}]'
                logger.warn(msg) if logger else print(msg)
                return pod_info
            else:
                msg = f'Debug pod [{pod_id}] is not register yet'
                logger.warn(msg) if logger else print(msg)
                return None
    def get_debug_node_info(self, pod_id, node_id):
        pod = self.get_debug_pod_info(pod_id=pod_id)
        if pod:
            return pod.get_debug_node_info(node_id=node_id)