from enum import IntEnum, auto
from pathlib import Path
from flask_socketio import rooms
from shutil import rmtree
from flask import request
import datetime

import re
from ddt_utils.utils import app_folder
from ddt_utils.utils import debug_pod_name
from ddt_utils.utils import pod_folder
from ddt_utils.utils import TmpFolder
from ddt_utils.utils import DebugPodPrefix
from ddt_utils.model import DebugElement


AppNames = list()
Apps = list()
# public room
RoomWeb = set()
RoomWebName = 'Room-Web'
WebID = None
ALLOWED_EXTENSIONS = {'yaml', 'yml'}

class DDTManagerProcess(IntEnum):
    K8sDeployPod = auto()

DefaultInterface = ["/rosout", "/parameter_events"]

def get_log_path():
    timestampStr = datetime.datetime.now().strftime("%d-%b-%Y-%H-%M")
    Path(TmpFolder).mkdir(exist_ok=True, parents=True)
    return  Path(TmpFolder / f'{timestampStr}_log.txt')

def get_pod_rosgraph_path(app_id, pod_id):
    return  Path(pod_folder(app_id, pod_id, remote=False) / f'{pod_id}.svg')

def get_app_rosgraph_path(app_id):
    return  Path(app_folder(app_id, remote=False) / f'{app_id}.svg')

def get_pod_domain_svg(app_id, pod_id):
    return Path(pod_folder(app_id, pod_id, remote=False) / f'domain_id.svg')

app_deployment_folder = Path(TmpFolder / "AppDeployments")

def debug_deployment_folder(app_id):
    folder = Path(app_folder(app_id=app_id, remote=False) / 'debug_deployment')
    folder.mkdir(exist_ok=True, parents=True)
    return folder

def debug_deployment_file(app_id, node):
    return debug_deployment_folder(app_id=app_id, remote=False) / f'{DebugPodPrefix}{node}.yaml'

def get_rooms(sid):
    l = rooms(sid)
    l.remove(sid)
    return l

def name_app_room(app):
    return f'Room-{app}'

def name_app_obj(app):
    return f'app_{app}'

def cleanup_folder(app_id, pod_id=None):
    if pod_id:
        rmtree(str(pod_folder(app_id,pod_id, remote=False)), ignore_errors=True)
    else:
        rmtree(str(app_folder(app_id, remote=False)), ignore_errors=True)

def name_debug_nodes_list(app):
    return f'debug_nodenames_{app}'

def get_list(headers):
    values = [request.form.getlist(h) for h in headers]
    items = [{} for i in range(len(values[0]))]
    for x,i in enumerate(values):
        for _x,_i in enumerate(i):
            items[_x][headers[x]] = _i
    return items

def rename_to_new_debug_pod(app_name, debug_node_model, debug_list):
    for info in debug_node_model.interface_info:
        for ele in info.destinations + info.sources:
            if DebugElement.parse_obj(ele) in debug_list:
                ele.pod = debug_pod_name(app_name=app_name, node_name=ele.node)

def remove_prefix(text, prefix):
    return re.sub(r'^{0}'.format(re.escape(prefix)), '', text)