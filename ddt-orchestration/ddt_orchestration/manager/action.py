
"""
click -> notice probe to start zenoh bridge `./root/zenoh-bridge-dds -d $ROS_DOMAIN_ID -e tcp/zenoh-router-service:7447`
->

"""
import logging
import subprocess
from shlex import split
from turtle import dot
import pygraphviz as pgv
from pathlib import Path
import os
from ddt_orchestration.utils import *
from ddt_orchestration.model import Node
import svg_stack as ss
import svgwrite
from flask import Markup

def start_command(command):
    print("Start command: ", command)
    c = split(command)
    pid = subprocess.Popen(c)
    return pid

def stop_command(pid):
    if isinstance(pid, int):
        os.kill(pid, subprocess.signal.SIGINT)
    else:
        pid.send_signal(subprocess.signal.SIGINT)

def start_zenoh(host):
    pid_zenoh = start_command(f"./root/zenoh-bridge-dds -d $ROS_DOMAIN_ID -e tcp/{host}:7447 -f")
    return pid_zenoh

def get_ros_graph(pod, folder_path):
    pid_graph_creator = start_command(f"ros2 launch ros2_graph_quest gen_dot.launch.py application_name:=\"{pod}\" result_path:=\"{folder_path}\" sampling_rate:=\"1\"")
    return pid_graph_creator

def get_ros_model(app_folder_path):
    pid_ros_model = start_command(f"ros2 launch ros2_graph_quest parse_nodes.launch.py result_path:=\"{app_folder_path}\" sampling_rate:=\"1\"")
    return pid_ros_model

def pause_ros_graph(pids):
    print("pause_ros_graph")
    for pid in pids:
        stop_command(pid)

def create_url(prefix, name):
    url = f'/{prefix}/{name}'
    return url

def rewrite_dot(dot_path, name, prefix):
    org_pt = Path(dot_path).resolve()
    new_pt = org_pt.parent / f'{name}.svg'
    if org_pt.is_file:
        try:
            ros_gh = pgv.AGraph(org_pt)
        except pgv.agraph.DotError:
            logging.ERROR(f'{org_pt.name} of {org_pt.parent.name} from {org_pt.parent.parent.name} is not ready!')
        for node in ros_gh.nodes():
            node.attr["URL"] = create_url(prefix, node.attr["URL"])
        ros_gh.draw(path=new_pt, prog='dot', format='svg' )

def pre_make_ros_graph(pod, folder_path, host = None, debug=True):
    if not debug:
        pid = start_zenoh(host)
        yield ProcessList.GraphBridgeProcess.name, pid
    pid = get_ros_graph(pod, folder_path)
    yield ProcessList.RosGraphProcess.name, pid

def make_ros_graph(pod, prefix, folder_path):
    if folder_path.is_dir():
        dot_path = folder_path / f'{pod}.dot'
        if dot_path.is_file():
            rewrite_dot(dot_path, pod, prefix)

def update_rosmodels(app_id, pod_id):
    path = get_pod_node_folder(app_id, pod_id)
    p = Path(path).glob('**/*.json')
    files = [x for x in p if x.is_file()]
    for f in files:
        logging.info(f'Reading file {f}')
        node = Node.parse_file(str(Path(f).resolve()))
        yield node

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
            pod_domain = create_domain_svg(app_obj.name, pod.name, pod.domain_id)
            pod_rosgraph = get_pod_rosgraph_path(app_obj.name, pod.name)
            if pod_rosgraph.is_file() and pod_domain.is_file():
                v_layout.addSVG(str(pod_domain.resolve()), alignment=ss.AlignCenter)
                v_layout.addSVG(str(pod_rosgraph.resolve()), alignment=ss.AlignCenter)
    merged_svgs.setLayout(v_layout)
    merged_svgs.save(get_app_rosgraph_path(app_obj.name))
    return show_svg(get_app_rosgraph_path(app_obj.name))

def create_domain_svg(app_id, pod_id, domain_id):
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
