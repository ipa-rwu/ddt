from pathlib import Path
from flask_socketio import rooms
from shutil import rmtree
from flask import request
from ddt_utils.utils import get_app_folder
from ddt_utils.utils import get_pod_folder
from ddt_utils.utils import TmpFolder
import datetime

AppNames = list()
Apps = list()
# public room
RoomWeb = list()
RoomWebName = 'Room-Web'
WebID = None

def get_log_path():
    timestampStr = datetime.datetime.now().strftime("%d-%b-%Y-%H-%M")
    Path(TmpFolder).mkdir(exist_ok=True, parents=True)
    return  Path(TmpFolder / f'{timestampStr}_log.txt')

def get_pod_rosgraph_path(app_id, pod_id):
    return  Path(get_pod_folder(app_id, pod_id) / f'{pod_id}.svg')

def get_app_rosgraph_path(app_id):
    return  Path(get_app_folder(app_id) / f'{app_id}.svg')

def get_pod_domain_svg(app_id, pod_id):
    return Path(get_pod_folder(app_id, pod_id) / f'domain_id.svg')

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
        rmtree(str(get_pod_folder(app_id,pod_id)), ignore_errors=True)
    else:
        rmtree(str(get_app_folder(app_id)), ignore_errors=True)

def name_select_nodes(app):
    return f'debug_nodenames_{app}'

def get_list(headers):
    values = [request.form.getlist(h) for h in headers]
    items = [{} for i in range(len(values[0]))]
    for x,i in enumerate(values):
        for _x,_i in enumerate(i):
            items[_x][headers[x]] = _i
    return items
