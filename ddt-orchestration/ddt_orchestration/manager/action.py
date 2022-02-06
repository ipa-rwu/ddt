
"""
click -> notice probe to start zenoh bridge `./root/zenoh-bridge-dds -d $ROS_DOMAIN_ID -e tcp/zenoh-router-service:7447`
->

"""
import subprocess
from shlex import split
from turtle import dot
import pygraphviz as pgv
from pathlib import Path

def start_command(command):
    print("Start command: ", command)
    c = split(command)
    pid = subprocess.Popen(c)
    return pid

def stop_command(pid):
    pid.send_signal(subprocess.signal.SIGINT)

def start_zenoh(host):
    pid_zenoh = start_command(f"./root/zenoh-bridge-dds -d $ROS_DOMAIN_ID -e tcp/{host}:7447 -f")
    return pid_zenoh

def get_ros_graph(app, folder_path):
    print("Start ros Graph Creator")
    pid_graph_creator = start_command(f"ros2 launch ros2_graph_quest gen_dot.launch.py application_name:=\"{app}\" result_path:=\"{folder_path}\"")
    print(pid_graph_creator)
    return pid_graph_creator

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
    ros_gh = pgv.AGraph(org_pt)
    for node in ros_gh.nodes():
        node.attr["URL"] = create_url(prefix, node.attr["URL"])
    print("rewrite_dot")
    ros_gh.draw(path=new_pt, prog='dot', format='svg' )

def pre_make_ros_graph(app, folder_path, host = None, debug=True):
    if not debug:
        pid = start_zenoh(host)
        yield 'rosgraph_bridge', pid
    pid = get_ros_graph(app, folder_path)
    yield 'rosgraph', pid

def make_ros_graph(app, prefix, folder_path):
    if folder_path.is_dir():
        dot_path = folder_path / f'{app}.dot'
        print("folder", dot_path)
        if dot_path.is_file():
            rewrite_dot(dot_path, app, prefix)

def main():
    pass

if __name__ == '__main__':
    main()
