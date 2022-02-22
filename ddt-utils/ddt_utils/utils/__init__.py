from pathlib import Path
from enum import Enum, IntEnum, auto
import os
import re

from ddt_utils.model import Node, LifeCycleNode

TmpFolder= Path(os.getenv("DDT_FOLDER", Path.home() / 'tmp'))
PodServerPort = os.getenv("DDT_PROBE_SERVER_PORT", 4000)

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

def pod_service_name(pod_id):
    # return f'{pod_id}_service'
    return pod_id

def remote_probe_server(pod_id):
    return f'{pod_service_name(pod_id)}:{PodServerPort}'

def remote_app_folder(app_id, pod_id):
    return f'{remote_probe_server(pod_id)}/{app_id}'

def app_folder(app_id, *, remote, **kwargs):
    logger = kwargs.get('logger')
    if remote:
        pod_id = kwargs.get('pod_id')
        if pod_id:
            f = remote_app_folder(app_id=app_id, pod_id = pod_id)
        else:
            msg = f'Please provide Pod name to get remote Application[{app_id}] folder'
            if logger:
                logger.error(msg)
            else:
                print(msg)
    else:
        f = Path(TmpFolder / app_id).resolve()
        f.mkdir(parents=True, exist_ok=True)
    return  f

def remote_pod_folder(app_id, pod_id):
    return f'{remote_probe_server(pod_id)}/{app_id}/{pod_id}'

def pod_folder(app_id, pod_id,*, remote):
    if remote:
        f = remote_pod_folder(app_id=app_id, pod_id = pod_id)
    else:
        f = Path(app_folder(app_id, remote=False) / pod_id).resolve()
        f.mkdir(parents=True, exist_ok=True)
    return f

def remote_pod_node_folder(app_id, pod_id):
    return f'{remote_probe_server(pod_id)}/{app_id}/{pod_id}/nodes'

def pod_node_folder(app_id, pod_id, remote):
    if remote:
        f = remote_pod_node_folder(app_id=app_id, pod_id = pod_id)
    else:
        f = Path(pod_folder(app_id, pod_id, remote=False) / 'nodes')
        f.mkdir(parents=True, exist_ok=True)
    return  f

def remote_pod_lifecycle_folder(app_id, pod_id):
    return f'{remote_probe_server(pod_id)}/{app_id}/{pod_id}/lifecycle_nodes'

def pod_lifecycle_folder(app_id, pod_id, *, remote):
    if remote:
        f = remote_pod_lifecycle_folder(app_id=app_id, pod_id = pod_id)
    else:
        f = Path(pod_folder(app_id, pod_id, remote=False) / 'lifecycle_nodes')
        f.mkdir(parents=True, exist_ok=True)
    return  f

def remote_dot_file(pod_id, app_id):
    return f'{remote_probe_server(pod_id)}/{app_id}/{pod_id}/{pod_id}.dot'

def dot_file_path(app_id, pod_id, *, remote):
    if remote:
        f = remote_dot_file(app_id=app_id, pod_id = pod_id)
    else:
        f = Path(pod_folder(app_id, pod_id, remote=False) /f'{pod_id}.dot')
    return f

def get_debug_pod_name(node_name):
    return f"{DebugPodPrefix}{re.sub('[^a-zA-Z-]+', '-', node_name)}"

def update_rosmodels(app_id, pod_id, logger = None):
    path = pod_node_folder(app_id, pod_id, remote=False)
    p = Path(path).glob('**/*.json')
    files = [x for x in p if x.is_file()]
    print(files)
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
    path = pod_lifecycle_folder(app_id, pod_id, remote=False)
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
