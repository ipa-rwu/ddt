from pathlib import Path
from enum import Enum
import os

TmpFolder= Path(os.getenv("DDT_FOLDER", Path.home() / 'tmp'))

class _ExtendedEnum(Enum):
    @classmethod
    def list(cls):
        return list(map(lambda c: c.name, cls))

class ProcessList(_ExtendedEnum):
    RosGraphProcess =  'show_graph'
    DebugBridgeProcess = "set_debug_bridge"
    GraphBridgeProcess = 'set_graph_bridge'
    NodeParserProcess = 'get_node_models'
    LifeCycleParserProcess = 'get_lifecycle_node_models'

def get_app_folder(app_id):
    f = Path(TmpFolder / app_id)
    f.mkdir(parents=True, exist_ok=True)
    return  f

def get_pod_folder(app_id, pod_id):
    f = Path(get_app_folder(app_id) / pod_id)
    f.mkdir(parents=True, exist_ok=True)
    return f

def get_pod_node_folder(app_id, pod_id):
    f = Path(get_pod_folder(app_id, pod_id) / 'nodes')
    f.mkdir(parents=True, exist_ok=True)
    return  f

def get_pod_lifecycle_folder(app_id, pod_id):
    f = Path(get_pod_folder(app_id, pod_id) / 'lifecycle_nodes')
    f.mkdir(parents=True, exist_ok=True)
    return  f