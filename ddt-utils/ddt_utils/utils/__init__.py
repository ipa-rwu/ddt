from pathlib import Path
from enum import Enum, IntEnum, auto
import os
import re
import socket

from ddt_utils.model import Node, LifeCycleNode

TmpFolder= Path(os.getenv("DDT_FOLDER", Path.home() / 'tmp'))
PodServerPort = os.getenv("DDT_PROBE_SERVER_PORT", 4000)

DebugPodPrefix = f'ddt-debug'

class _ExtendedEnum(Enum):
    @classmethod
    def list(cls):
        return list(map(lambda c: c.name, cls))

class ProcessList(_ExtendedEnum):
    RosGraphProcess =  'show_graph'
    DebugBridgeProcess = "set_debug_bridge"
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
    ConfirmDebug = "confirm_debug"

def getIP(d):
    """
    This method returns the first IP address string
    that responds as the given domain name
    """
    try:
        data = socket.gethostbyname(d)
        ip = repr(data)
        return ip
    except Exception:
        # fail gracefully!
        return False

"""
pod_ip
logger
"""
def pod_service(pod_id, **kwargs):
    logger = kwargs.get('logger')
    ip = getIP(pod_id)
    pod_ip = kwargs.get('pod_ip')
    if ip:
        msg = f"Find IP of pod [{pod_id}], will use pod ip [{ip}]"
        return ip
    elif pod_ip:
        msg = f"Couldn't find IP of pod [{pod_id}], will use pod ip [{pod_ip}]"
        logger.info(msg) if logger else print(msg)
        return pod_ip
    else:
        msg = f"Couldn't find IP of pod[{pod_id}], please provide IP direcly"
        logger.info(msg) if logger else print(msg)

def remote_probe_server(pod_id, **kwargs):
    return f'{pod_service(pod_id, **kwargs)}:{PodServerPort}'

def remote_app_folder(app_id, pod_id, **kwargs):
    return f'{remote_probe_server(pod_id, **kwargs)}/{app_id}'

def app_folder(app_id, *, remote, **kwargs):
    logger = kwargs.get('logger')
    if remote:
        pod_id = kwargs.get('pod_id')
        if pod_id:
            f = remote_app_folder(app_id=app_id, **kwargs)
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

def remote_pod_folder(app_id, pod_id, **kwargs):
    return f'{remote_probe_server(pod_id, **kwargs)}/{app_id}/{pod_id}'

def pod_folder(app_id, pod_id,*, remote, **kwargs):
    if remote:
        f = remote_pod_folder(app_id=app_id, pod_id = pod_id, **kwargs)
    else:
        f = Path(app_folder(app_id, remote=False, **kwargs) / pod_id).resolve()
        f.mkdir(parents=True, exist_ok=True)
    return f

def remote_pod_node_folder(app_id, pod_id, **kwargs):
    return f'{remote_probe_server(pod_id, **kwargs)}/{app_id}/{pod_id}/nodes'

def pod_node_folder(app_id, pod_id, remote, **kwargs):
    if remote:
        f = remote_pod_node_folder(app_id=app_id, pod_id = pod_id, **kwargs)
    else:
        f = Path(pod_folder(app_id, pod_id, remote=False, **kwargs) / 'nodes')
        f.mkdir(parents=True, exist_ok=True)
    return  f

def remote_pod_lifecycle_folder(app_id, pod_id,  **kwargs):
    return f'{remote_probe_server(pod_id, **kwargs)}/{app_id}/{pod_id}/lifecycle_nodes'

def pod_lifecycle_folder(app_id, pod_id, *, remote, **kwargs):
    if remote:
        f = remote_pod_lifecycle_folder(app_id=app_id, pod_id = pod_id, **kwargs)
    else:
        f = Path(pod_folder(app_id, pod_id, remote=False, **kwargs) / 'lifecycle_nodes')
        f.mkdir(parents=True, exist_ok=True)
    return  f

def remote_dot_file(app_id, pod_id, **kwargs):
    return f'{remote_probe_server(pod_id, **kwargs)}/{app_id}/{pod_id}/{pod_id}.dot'

def dot_file_path(app_id, pod_id, *, remote, **kwargs):
    if remote:
        f = remote_dot_file(app_id=app_id, pod_id = pod_id, **kwargs)
    else:
        f = Path(pod_folder(app_id, pod_id, remote=False, **kwargs) /f'{pod_id}.dot')
    return f

def debug_pod_name(app_name, node_name):
    return f"{DebugPodPrefix}-{app_name}{re.sub('[^0-9a-zA-Z]', '-', node_name)}"

def update_rosmodels(app_id, pod_id, **kwargs):
    path = pod_node_folder(app_id, pod_id, remote=False)
    p = Path(path).glob('**/*.json')
    logger = kwargs.get('logger')
    files = [x for x in p if x.is_file()]
    try:
        for f in files:
            node = Node.parse_file(str(Path(f).resolve()))
            yield node
    except FileNotFoundError as e:
        logger.error(e) if logger else print(e)

def update_lifecycle_models(app_id, pod_id, **kwargs):
    path = pod_lifecycle_folder(app_id, pod_id, remote=False)
    p = Path(path).glob('**/*.json')
    logger = kwargs.get('logger')
    files = [x for x in p if x.is_file()]
    try:
        for f in files:
            node = LifeCycleNode.parse_file(str(Path(f).resolve()))
            yield node
    except FileNotFoundError as e:
        logger.error(e) if logger else print(e)
