from ddt_utils.utils import ProcessList
from ddt_utils.utils import pod_node_folder
from ddt_utils.utils import pod_lifecycle_folder
from ddt_utils.utils import update_lifecycle_models
from ddt_utils.utils import update_rosmodels
from ddt_utils.utils import BridgeMode
from ddt_utils.utils import get_debug_pod_name
from ddt_utils.utils import dot_file_path

from ddt_utils.actions import get_bridge_mode
from ddt_utils.actions import start_command
from ddt_utils.actions import stop_command

from shutil import rmtree, copy
from pathlib import Path

from ros2_helpers.parse_topic_info.parse_topic_info import get_connection_info


'''
@paraeters:
* domain_id
* debug: info, debug
* dsts: [dst_service:dst_port]
* listen_server
* listen_port
* allow_topics
'''
def start_zenoh(**kwargs):
    if 'domain_id' in kwargs and kwargs["domain_id"] is not None:
        common = f'/root/zenoh-bridge-dds -d {kwargs["domain_id"]} '
    else:
        common = f'/root/zenoh-bridge-dds -d $ROS_DOMAIN_ID '
    debug=''
    if 'debug' in kwargs and kwargs["debug"] is not None:
        debug = f'RUST_LOG={kwargs["debug"].upper() }'

    peer = ''
    if 'dsts' in kwargs:
        dsts = kwargs['dsts']
        for dst in dsts:
            peer = f'-e tcp/{dst}'

    ls=''
    ls_service = '0.0.0.0'
    if 'listen_server' in kwargs:
        ls_service = kwargs['listen_server']
        ls_port = 8000
        ls = f'-l tcp/{ls_service}:{ls_port} '
    if 'listen_port' in kwargs:
        ls_port = kwargs['listen_port']
        ls = f'-l tcp/{ls_service}:{ls_port} '

    allow_topics=''
    if 'allow_topics' in kwargs:
        topics = kwargs['allow_topics']
        if isinstance(topics, list):
            print('|'.join(topics))
            all = '|'.join(topics)
            allow_topics = f'-a {all}'
        else:
            allow_topics = f'-a {topics}'

    pid_zenoh = start_command(f'{debug}{common}{peer}{ls}{allow_topics}')
    return pid_zenoh

def get_ros_graph(app_id, pod_id):
    dot_file = dot_file_path(app_id=app_id, pod_id=pod_id, remote=False)
    pid = start_command(f"ros2 launch ros2_graph_quest gen_dot.launch.py result_path:=\"{dot_file}\" sampling_rate:=\"1\"")
    yield ProcessList.RosGraphProcess.name, pid

def get_ros_model(app_id, pod_id):
    # pid_ros_model = start_command(f"ros2 launch ros2_graph_quest parse_nodes.launch.py result_path:=\"{app_folder_path}\" sampling_rate:=\"1\"")
    pid = start_command(f'ros2helper nodes -f {pod_node_folder(app_id, pod_id, remote=False)} -r 1')
    yield ProcessList.NodeParserProcess.name, pid


def get_lifecycle_nodes(app_id, pod_id):
    # pid_ros_model = start_command(f"ros2 launch ros2_graph_quest parse_nodes.launch.py result_path:=\"{app_folder_path}\" sampling_rate:=\"1\"")
    pid = start_command(f'ros2helper lifecycle -f {pod_lifecycle_folder(app_id, pod_id, remote=False)} -r 1')
    yield ProcessList.LifeCycleParserProcess.name, pid


def pause_ros_graph(pids, **kwargs):
    print("pause_ros_graph")
    for pid in pids:
        stop_command(pid, kwargs)

def update_nodes(app_id, pod_id, pod):
    for node in update_rosmodels(app_id, pod_id):
        pod.add_node(node)

def update_lifecycle_nodes(app_id, pod_id, pod):
    for node in update_lifecycle_models(app_id, pod_id):
        pod.add_lifecycle_node(node)

def do(node_model):
    for topic in node_model.publishers:
        # get subscribers of topic
        target_node_list = get_connection_info(topic).reception
        yield 'sub',  topic, target_node_list
    for topic in node_model.subscribers:
        # get oublishers of topic
        target_node_list = get_connection_info(topic).emission
        yield 'pub',  topic, target_node_list

def get_debugged_pod_name(node_name):
    return f'debug_{node_name}'

'''
@paraeters:
* domain_id
* debug: info, debug
* dsts =[]
* listen_server
* listen_port
* allow_topics
'''
def start_zenoh_process(allow_topics, **kwargs):
    debug = kwargs.get('debug', None)
    ls = kwargs.get('listen_server', '0.0.0.0')
    lp = kwargs.get('listen_port', 8000)
    domain_id = kwargs.get('domain_id', None)
    # if bridge_mode == BridgeMode.Listener:
    #     pid = start_zenoh(debug=debug,
    #                         domain_id=domain_id,
    #                         allow_topics=allow_topics,
    #                         listen_server=ls,
    #                         listen_port=lp)
    # if bridge_mode == BridgeMode.Peer:
    #     pid = start_zenoh(debug=debug,
    #                         domain_id=domain_id,
    #                         allow_topics=allow_topics,
    #                         dsts = kwargs['dsts'])
    # if bridge_mode == BridgeMode.ListenerPeer:
    pid = start_zenoh(debug=debug,
                        domain_id=domain_id,
                        allow_topics=allow_topics,
                        dsts = kwargs['dsts'],
                        listen_server=ls,
                        listen_port=lp)
    yield ProcessList.DebugBridgeProcess.name, pid

def decide_bridge_collect_debug_topics(debug_list, PodModel):
    flag = False
    bridge = BridgeMode.NoNeed
    def _combine_lists(a, b):
        return (a + list(set(b) - set(a)))
    def _topics(node_model):
        pub_topics = [interface.name for interface in node_model.publishers]
        sub_topics = [interface.name for interface in node_model.subscribers]
        return _combine_lists(pub_topics, sub_topics)

    topics = list()
    dsts = list()
    for name in debug_list:
        print(f'{name} from {debug_list}')
        try:
            node_model = PodModel.find_node(name)
        except KeyError:
            raise
        # if not flag:
        #     if bridge == BridgeMode.NoNeed:
        #         bridge = get_bridge_mode(node_model)
        #     if bridge == BridgeMode.Peer or bridge == BridgeMode.Listener:
        #         tmp = get_bridge_mode(node_model)
        #         if tmp is not BridgeMode.NoNeed and tmp != bridge:
        #             bridge = BridgeMode.ListenerPeer
        #             flag = True
        #         else:
        #             bridge = tmp
        # if get_bridge_mode(node_model) == BridgeMode.Peer:
        dsts.append(get_debug_pod_name(name))
        if len(topics) == 0:
            topics = _topics(node_model)
        else:
            tmp_list = topics
            topics = _combine_lists(tmp_list, _topics(node_model))

    return topics, dsts

def decide_bridge_mode(debug_list, PodModel):
    bridge = BridgeMode.NoNeed
    for name in debug_list:
        node_model = PodModel.find_node(name)
        if bridge == BridgeMode.NoNeed:
            bridge = get_bridge_mode(node_model)
        if bridge == BridgeMode.Peer or bridge == BridgeMode.Listener:
            tmp = get_bridge_mode(node_model)
            if tmp is not BridgeMode.NoNeed and tmp != bridge:
                return BridgeMode.ListenerPeer
            else:
                bridge = tmp
    return bridge

def backup_to_finial(result_path, bk_path):
    if bk_path.is_dir():
        if result_path.is_dir():
            rmtree(result_path, ignore_errors=True)
        result_path.mkdir(exist_ok=True, parents=True)
        for i in Path(bk_path).glob('*.json'):
            print(i)
            copy(i, result_path / i.name)

def kill_node():
    NotImplemented
