import logging
import pygraphviz as pgv
from pathlib import Path
import svg_stack as ss
import svgwrite
from flask import Markup

from ddt_utils.utils import get_pod_node_folder
from ddt_utils.utils import get_pod_folder
from ddt_utils.utils import get_pod_lifecycle_folder

from ros2_model import Node, LifeCycleNode

from ddt_manager.utils import get_app_rosgraph_path
from ddt_manager.utils import get_pod_domain_svg
from ddt_manager.utils import get_pod_rosgraph_path
from ddt_manager.utils import name_app_obj

def create_url(prefix, name):
    url = f'/{prefix}/{name}/add'
    return url

def rewrite_dot(dot_path, name, prefix, logger):
    org_pt = Path(dot_path).resolve()
    new_pt = org_pt.parent / f'{name}.svg'
    if org_pt.is_file:
        try:
            ros_gh = pgv.AGraph(org_pt)
            for node in ros_gh.nodes():
                node.attr["URL"] = create_url(prefix, node.attr["URL"])
            ros_gh.draw(path=new_pt, prog='dot', format='svg' )
        except pgv.agraph.DotError:
            logger.error(f'{org_pt.name} of {org_pt.parent.name} from {org_pt.parent.parent.name} is not ready!')

def _make_ros_graph(pod, prefix, folder_path, logger):
    if folder_path.is_dir():
        dot_path = folder_path / f'{pod}.dot'
        if dot_path.is_file():
            rewrite_dot(dot_path, pod, prefix, logger)

def update_rosmodels(app_id, pod_id, logger):
    path = get_pod_node_folder(app_id, pod_id)
    p = Path(path).glob('**/*.json')
    files = [x for x in p if x.is_file()]
    try:
        for f in files:
            node = Node.parse_file(str(Path(f).resolve()))
            yield node
    except FileNotFoundError as e:
            logger.error(e)

def update_lifecycle_models(app_id, pod_id, logger):
    path = get_pod_lifecycle_folder(app_id, pod_id)
    p = Path(path).glob('**/*.json')
    files = [x for x in p if x.is_file()]
    try:
        for f in files:
            node = LifeCycleNode.parse_file(str(Path(f).resolve()))
            yield node
    except FileNotFoundError as e:
        logger.error(e)

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
    g = dwg.g(style="font-size:30;\
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

def update_ros_graph(app, pod, logger):
    return _make_ros_graph(pod, f'app_{app}/{pod}', get_pod_folder(app, pod), logger)

def check_state_emit(pod, process, msg, socketio, socket_id):
    process_in_pod = pod.find_process(process.name)
    if not process_in_pod.state:
        socketio.emit(process.value, msg, to = socket_id)

def set_process_state(app_id, pod_id, processes):
    for process in processes:
        # print(process)
        p = globals()[name_app_obj(app_id)].find_pod(pod_id).find_process(process['name'])
        if p:
            p.start(pid=process['pid'])
        else:
            p.stop(pid=process['pid'])
