from shutil import rmtree
import pygraphviz as pgv
from pathlib import Path
import svg_stack as ss
import svgwrite
from flask import Markup
import time

from ddt_utils.utils import pod_node_folder
from ddt_utils.utils import pod_folder
from ddt_utils.utils import pod_lifecycle_folder
from ddt_utils.utils import dot_file_path

from ddt_utils.actions import set_process_state
from ddt_utils.actions import start_command
from ddt_utils.actions import stop_command
from ddt_utils.actions import call_command

from ddt_utils.utils import update_lifecycle_models
from ddt_utils.utils import update_rosmodels

from ros2_model import Node, LifeCycleNode

from ddt_manager.utils import get_app_rosgraph_path
from ddt_manager.utils import get_pod_domain_svg
from ddt_manager.utils import get_pod_rosgraph_path
from ddt_manager.utils import name_app_obj
from ddt_manager.utils import DDTManagerProcess
from ddt_manager.utils import debug_deployment_file

def create_url(prefix, name):
    url = f'/{prefix}{name}/add'
    return url

def command_get_remote_file(link, file_path):
    return f'wget -O {file_path} {link} '

def get_remote_dot_file(app_id, pod_id, **kwargs):
    return start_command(command_get_remote_file(dot_file_path(app_id=app_id, pod_id=pod_id, remote=True, **kwargs),
                                            dot_file_path(pod_id=pod_id, app_id=app_id, remote=False, **kwargs)))

def _rewrite_dot(app_pid, pod_id, prefix, **kwargs):
    get_remote_dot_file(app_pid, pod_id, **kwargs)
    org_pt = Path(dot_file_path(pod_id=pod_id, app_id=app_pid, remote=False, **kwargs))
    new_pt = org_pt.parent / f'{pod_id}.svg'
    logger = kwargs.get('logger')
    if org_pt.is_file:
        try:
            ros_gh = pgv.AGraph(org_pt)
            for node in ros_gh.nodes():
                node.attr["URL"] = create_url(prefix, node.attr["label"].replace('/', '$'))
            ros_gh.draw(path=new_pt, prog='dot', format='svg' )
        except pgv.agraph.DotError:
            msg = f'{org_pt.name} of {org_pt.parent.name} from {org_pt.parent.parent.name} is not ready!'
            logger.error(msg) if logger else print(msg)

def command_remote_folder(link, folder):
    return f'wget -nd -np -P {folder} -r {link}'

def get_remote_node_folder(app_id, pod_id, **kwargs):
    call_command(command_remote_folder(pod_node_folder(app_id=app_id, pod_id=pod_id, remote=True, **kwargs),
                                            pod_node_folder(pod_id=pod_id, app_id=app_id, remote=False, **kwargs)))

def get_remote_lifecycle_node_folder(app_id, pod_id, **kwargs):
    call_command(command_remote_folder(pod_lifecycle_folder(app_id=app_id, pod_id=pod_id, remote=True, **kwargs),
                                            pod_lifecycle_folder(pod_id=pod_id, app_id=app_id, remote=False, **kwargs)))

def update_remote_ros_models(app_id, pod_id, **kwargs):
    rmtree(pod_node_folder(pod_id=pod_id, app_id=app_id, remote=False), ignore_errors=True)
    get_remote_node_folder(app_id, pod_id, **kwargs)
    return update_rosmodels(app_id=app_id, pod_id=pod_id, **kwargs)

def update_remote_lifecycle_models(app_id, pod_id, **kwargs):
    rmtree(pod_lifecycle_folder(pod_id=pod_id, app_id=app_id, remote=False, **kwargs), ignore_errors=True)
    get_remote_lifecycle_node_folder(app_id=app_id, pod_id=pod_id, **kwargs)
    return update_lifecycle_models(app_id=app_id, pod_id=pod_id, **kwargs)

def update_app_model(app_model, **kwargs):
    app_id = app_model.name
    logger = kwargs.get('logger')
    for pod in app_model.pods:
        pod_id = pod.name
        node_models = update_remote_ros_models(app_id, pod_id, pod_ip = pod.ip, **kwargs)
        if node_models:
            app_model.add_nodes(pod_id, node_models)
        life_models = update_remote_lifecycle_models(app_id, pod_id, pod_ip = pod.ip, **kwargs)
        if life_models:
            app_model.add_lifecycle_nodes(pod_id, life_models)
    if logger:
        logger.info(f'get ros graph of Application [{app_id}]')

def show_svg(path):
    svg = None
    if path.is_file():
        svg = open(str(path)).read()
    return Markup(svg)

def combine_rosgraphs(app_obj):
    merged_svgs = ss.Document()
    v_layout = ss.VBoxLayout()
    domain_list = list()
    for pod in app_obj.pods:
        if pod.domain_id not in domain_list:
            domain_list.append(pod.domain_id)
            pod_domain = _create_domain_svg(app_obj.name, pod.name, pod.domain_id)
            pod_rosgraph = get_pod_rosgraph_path(app_obj.name, pod.name)
            if pod_rosgraph.is_file() and pod_domain.is_file():
                v_layout.addSVG(str(pod_domain.resolve()), alignment=ss.AlignCenter)
                v_layout.addSVG(str(pod_rosgraph.resolve()), alignment=ss.AlignCenter)
    merged_svgs.setLayout(v_layout)
    merged_svgs.save(get_app_rosgraph_path(app_obj.name))
    return show_svg(get_app_rosgraph_path(app_obj.name))

def _create_domain_svg(app_id, pod_id, domain_id):
    svg_size_width = 200
    svg_size_height = 80
    file = get_pod_domain_svg(app_id, pod_id).resolve()
    dwg = svgwrite.Drawing(str(file) , (svg_size_width, svg_size_height))
    g = dwg.g(style="font-size:20;\
                    font-family:Comic Sans MS, Arial; \
                    font-weight:bold; \
                    font-style:oblique; \
                    stroke:black; \
                    stroke-width:1; \
                    fill:none")
    dwg.add(dwg.rect(insert=(0, 0),
            size=('100%', '100%'),
            rx=None,
            ry=None,
            fill='rgb(255,255,255)'))
    g.add(dwg.text(f"Domain_id: {domain_id}",
            insert = (svg_size_width/2, svg_size_height/2),
            fill = "rgb(0,0,0)"))
    dwg.add(g)
    dwg.save()
    return file

def update_ros_graph(app, pod_id, pod_ip, **kwargs):
    print(f'ip: {kwargs.get("pod_ip")}')
    return _rewrite_dot(app_pid=app, pod_id=pod_id, prefix = f'app_{app}/{pod_id}/__', pod_ip= pod_ip, **kwargs)

def check_state_emit(pod, process, msg, socketio, socket_id):
    process_in_pod = pod.find_process(process.name)
    if not process_in_pod.state:
        socketio.emit(process.value, msg, to = socket_id)

def set_processes_group(pod, processes, logger, **kwargs):
    for process in processes:
        set_process_state(pod, name = process['name'], pid=process['pid'], logger = logger, **kwargs)

def deploy_pod(app_id, node_id, **kwargs):
    logger = kwargs['logger']
    deployment_file = debug_deployment_file(app_id=app_id, node=node_id)
    if deployment_file.is_file():
        pid = start_command(f"kubectl apply -f {deployment_file}")
        return True
    elif logger:
        msg = f'Deployment file [{deployment_file}] is not found. Please upload it'
        logger.error(msg)
    else:
        print(msg)
    return False