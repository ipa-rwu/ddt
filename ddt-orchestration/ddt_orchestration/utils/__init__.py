from pathlib import Path
from enum import Enum
from flask_socketio import rooms
from shutil import rmtree

AppNames = list()
Apps = list()
# public room
RoomWeb = list()
RoomWebName = 'Room-Web'
WebID = None
ManagerPwd = Path(__file__).parent.parent
TmpFolder = ManagerPwd / 'tmp'

class ExtendedEnum(Enum):
    @classmethod
    def list(cls):
        return list(map(lambda c: c.name, cls))

class ProcessList(ExtendedEnum):
    RosGraphProcess = 0
    DebugBridgeProcess = 1
    GraphBridgeProcess = 2
    NodeParserProcess = 3

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

def get_pod_rosgraph_path(app_id, pod_id):
    return  Path(get_pod_folder(app_id, pod_id) / f'{pod_id}.svg')

def get_pod_rosgraph_dot_path(app_id, pod_id):
    return  Path(get_pod_folder(app_id, pod_id) / f'{pod_id}.dot')

def get_app_rosgraph_path(app_id):
    return  Path(get_app_folder(app_id) / f'{app_id}.svg')

def get_pod_domain_svg(app_id, pod_id):
    return Path(get_pod_folder(app_id, pod_id) / f'domain_id.svg')

def get_rosmodle_path(app_id, pod_id, node_name):
    return  Path(str(get_pod_folder(app_id, pod_id)) + f'{node_name}.json')

def read_file(path):
    with open(path,"r") as file:
        content = file.readlines()
    return content

def get_rooms(sid):
    l = rooms(sid)
    l.remove(sid)
    return l

def name_app_room(app):
    return f'Room-{app}'

def name_app_obj(app):
    return f'app_{app}'

def name_pod_obj(pod):
    return f'pod_{pod}'

def cleanup_folder(app_id, pod_id=None):
    if pod_id:
        rmtree(str(get_pod_folder(app_id,pod_id)), ignore_errors=True)
    else:
        rmtree(str(get_app_folder(app_id)), ignore_errors=True)
