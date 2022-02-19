from pathlib import Path
from enum import Enum, IntEnum, auto
import os
import re

from ddt_utils.model import Node, LifeCycleNode

TmpFolder= Path(os.getenv("DDT_FOLDER", Path.home() / 'tmp'))

DebugPodPrefix = f'DDT-debug-'
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

class BridgeMode(IntEnum):
    NoNeed   = 0
    Listener = auto()
    Peer     = auto()
    ListenerPeer = auto()

ShowRosGraphProcesses = [ProcessList.RosGraphProcess.name, ProcessList.LifeCycleParserProcess.name, ProcessList.NodeParserProcess.name]
class SocketActionList(_ExtendedEnum):
    Debug = "start_debug"
    PauseGraph = 'pause_graph'

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

def get_debug_pod_name(node_name):
    return f"{DebugPodPrefix}{re.sub('[^a-zA-Z-]+', '-', node_name)}"

def update_rosmodels(app_id, pod_id, logger = None):
    path = get_pod_node_folder(app_id, pod_id)
    p = Path(path).glob('**/*.json')
    files = [x for x in p if x.is_file()]
    try:
        for f in files:
            node = Node.parse_file(str(Path(f).resolve()))
            yield node
    except FileNotFoundError as e:
        if logger is not None:
            logger.error(e)
        else:
            return False

def update_lifecycle_models(app_id, pod_id, logger = None):
    path = get_pod_lifecycle_folder(app_id, pod_id)
    p = Path(path).glob('**/*.json')
    files = [x for x in p if x.is_file()]
    try:
        for f in files:
            node = LifeCycleNode.parse_file(str(Path(f).resolve()))
            yield node
    except FileNotFoundError as e:
        if logger is not None:
            logger.error(e)
        else:
            return False
