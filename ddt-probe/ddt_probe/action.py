import subprocess
from shlex import split
import os
from ddt_utils.utils import ProcessList
from ddt_utils.utils import get_pod_node_folder
from ddt_utils.utils import get_pod_lifecycle_folder

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
    pid = start_command(f"ros2 launch ros2_graph_quest gen_dot.launch.py application_name:=\"{pod}\" result_path:=\"{folder_path}\" sampling_rate:=\"1\"")
    yield ProcessList.RosGraphProcess.name, pid

def get_ros_model(app_id, pod_id):
    # pid_ros_model = start_command(f"ros2 launch ros2_graph_quest parse_nodes.launch.py result_path:=\"{app_folder_path}\" sampling_rate:=\"1\"")
    pid = start_command(f'ros2helper nodes -f {get_pod_node_folder(app_id, pod_id)} -r 1')
    yield ProcessList.NodeParserProcess.name, pid


def get_lifecycle_nodes(app_id, pod_id):
    # pid_ros_model = start_command(f"ros2 launch ros2_graph_quest parse_nodes.launch.py result_path:=\"{app_folder_path}\" sampling_rate:=\"1\"")
    pid = start_command(f'ros2helper lifecycle -f {get_pod_lifecycle_folder(app_id, pod_id)} -r 1')
    yield ProcessList.LifeCycleParserProcess.name, pid


def pause_ros_graph(pids):
    print("pause_ros_graph")
    for pid in pids:
        stop_command(pid)

# start rosgraph process
# save in folder_path/"pod".dot
def pre_make_ros_graph(pod, folder_path, host = None, debug=True):
    if not debug:
        pid = start_zenoh(host)
        yield ProcessList.GraphBridgeProcess.name, pid
    pid = get_ros_graph(pod, folder_path)
    yield ProcessList.RosGraphProcess.name, pid
