#! /usr/bin/env python3
import asyncio
from importlib.resources import path
import socketio
import logging
from pathlib import Path
import time

from ddt_utils.model import DebugPodInfo, Message
from ddt_utils.model import Pod
from ddt_utils.model import Process

from ddt_utils.utils import ProcessList, pod_lifecycle_folder, pod_node_folder
from ddt_utils.utils import SocketActionList
from ddt_utils.utils import update_lifecycle_models
from ddt_utils.utils import update_rosmodels
from ddt_utils.utils import ShowRosGraphProcesses

from ddt_utils.actions import set_process_state

from ros2_model import LifeCycleActionsEnum

from ddt_probe.action import backup_to_finial, collect_info_for_bridge, collect_info_for_debug_pod_bridge, get_related_nodes, get_ros_model
from ddt_probe.action import stop_command
from ddt_probe.action import get_ros_graph
from ddt_probe.action import get_lifecycle_nodes
from ddt_probe.action import update_nodes
from ddt_probe.action import update_lifecycle_nodes
from ddt_probe.action import start_zenoh_process

from ddt_probe.action import kill_node

from ros2_helpers.set_lifecycle_state import set_lifecycle_state

sio = socketio.AsyncClient(handle_sigint=False)

@sio.event
async def connect():
    logging.info(f'connected to server')
    await register()

def msg_start_process(app_id, pod_id, ps):
    new_msg = Message()
    logging.info(f'{pod_id} from {app_id} start processes: {ps}')
    new_msg.processes = ps
    new_msg.app_id = app_id
    new_msg.pod_id = pod_id
    return new_msg

def update_process_in_model(pod: Pod, ps):
    if ps['name'] not in [ process.name for process in pod.processes]:
        pod.processes.append(Process(name = ps['name'], pid = ps['pid']))
        logging.info(f'Add {pod.processes} to Pod [{pod.name}]')

@sio.on(ProcessList.RosGraphProcess.value)
async def show_graph(msg):
    m = Message(**msg)
    app_id = PodInfo.app_id
    pod_id = PodInfo.pod_id
    logging.info(f'{ProcessList.RosGraphProcess.value}: get message ({m})')
    if m.app_id == app_id and m.pod_id == pod_id:
        processes = get_ros_graph(app_id=app_id, pod_id=pod_id)
        pss = [dict(zip(("name", "pid"), (name, proc.pid))) for name, proc in processes]
        for ps in pss:
            update_process_in_model(PodModel, ps)
            set_process_state(PodModel, name=ps['name'], pid=ps['pid'], logger=logging, start=True)
        new_msg = msg_start_process(app_id, pod_id, pss)
        await sio.emit(f'started_{ProcessList.RosGraphProcess.value}', new_msg.dict())

@sio.on(ProcessList.NodeParserProcess.value)
async def get_node_model(msg):
    m = Message(**msg)
    app_id = PodInfo.app_id
    pod_id = PodInfo.pod_id
    if m.app_id == app_id and m.pod_id == pod_id:
        # create node model into files
        processes = get_ros_model(app_id, pod_id)
        pss = [dict(zip(("name", "pid"), (name, proc.pid))) for name, proc in processes]
        for ps in pss:
            update_process_in_model(PodModel, ps)
            set_process_state(PodModel, name=ps['name'], pid=ps['pid'], logger=logging, start=True)
        new_msg = msg_start_process(app_id, pod_id, pss)
        await sio.emit(f'started_{ProcessList.NodeParserProcess.value}', new_msg.dict())

@sio.on(ProcessList.LifeCycleParserProcess.value)
async def get_lifecycle_model(msg):
    m = Message(**msg)
    app_id = PodInfo.app_id
    pod_id = PodInfo.pod_id
    if m.app_id == app_id and m.pod_id == pod_id:
        # create node model into files
        processes = get_lifecycle_nodes(app_id, pod_id)
        pss = [dict(zip(("name", "pid"), (name, proc.pid))) for name, proc in processes]
        for ps in pss:
            update_process_in_model(PodModel, ps)
            set_process_state(PodModel, name=ps['name'], pid=ps['pid'], logger=logging, start=True)
        new_msg = msg_start_process(app_id, pod_id, pss)
        await sio.emit(f'started_{ProcessList.LifeCycleParserProcess.value}', new_msg.dict())

@sio.on(SocketActionList.Debug.value)
async def start_debug(msg):
    m = Message(**msg)
    app_id = PodInfo.app_id
    pod_id = PodInfo.pod_id
    debug_list = m.nodes
    debug_pod_info = DebugPodInfo.parse_obj(debug_list)
    for node_model in update_rosmodels(app_id, pod_id, logger=logging):
        PodModel.add_node(node_model)
    for node_model in update_lifecycle_models(app_id, pod_id, logger=logging):
        PodModel.add_lifecycle_node(node_model)

    if m.app_id == app_id and m.pod_id == pod_id:
        allow_topics, dst_names= collect_info_for_bridge(app_id=app_id , debug_list=debug_list, PodModel=PodModel)
        print(allow_topics, dst_names)
        logging.info(f'Start debug procedure! Lisen on: [{PodInfo.listen_server}:{PodInfo.listen_port}], \
                        Allow topics: {allow_topics}, \
                        Destinations: {dst_names}]')
        dsts = [ f'{name}:{PodInfo.dst_port}' for name in dst_names]
        # start bridge
        processes_pid = start_zenoh_process(debug=PodInfo.debug,
                                        domain_id=PodInfo.domain_id,
                                        allow_topics=list(allow_topics),
                                        dsts = dsts,
                                        listen_server=PodInfo.listen_server,
                                        listen_port=PodInfo.listen_port)
        global BridgeCounter
        pss = [dict(zip(("name", "pid"), (f'{ProcessList.DebugBridgeProcess.name}_{BridgeCounter}', processes_pid.pid)))]
        pss = [dict( name = f'{ProcessList.DebugBridgeProcess.name}_{BridgeCounter}',
                    pid =  processes_pid.pid)]
        BridgeCounter +=1
        logging.info(f'In {SocketActionList.Debug.value} strat: {pss}')

        for ps in pss:
            update_process_in_model(PodModel, ps)
            set_process_state(PodModel, name=ps['name'], pid=ps['pid'], logger=logging, start=True)
        new_msg = msg_start_process(app_id, pod_id, pss)
        await sio.emit(f'started_{SocketActionList.Debug.value}', new_msg.dict())
        # get all topics of this node -- add to allowded topic

        # find realted nodes per topic in same pod
            # if publisher, need to start a bridge with dst (pod service)
            # if subscriber, need to lisen on
        # debug nodw samw
    for name in [node.name for node in debug_pod_info.nodes]:
        if name in PodModel.node_list:
            if name in PodModel.lifecycle_node_list:
                life_model = PodModel.find_lifecycle_node(name)
                # Deactivate node
                res = set_lifecycle_state(name, LifeCycleActionsEnum.Shutdown.value)
                if res:
                    logging.info(f'Deactivate Lifecycle Node[{name}] successfully')
                else:
                    logging.error(f'Could Not deactivate Lifecycle Node[{name}]')
            else:
                try:
                    # kill node
                    kill_node(name)
                except:
                    NotImplemented

def event_minute_later(event):
    print(time.time()) # use for testing, comment out or delete for production
    return event + 60 < time.time()

# when debug pod up
@sio.on(ProcessList.DebugBridgeProcess.value)
async def start_debug_pod_bridge(msg):
    m = Message(**msg)
    app_id = PodInfo.app_id
    pod_id = PodInfo.pod_id
    debug_node_info = m.info
    logging.info(f'Zenoh bridge will start in Debug Pod [{pod_id}] for Application [{app_id}]')
    logging.info(f'Get debug infomation: {debug_node_info}')

    # create node model into file
    processes = get_lifecycle_nodes(app_id, pod_id)
    pss = [dict(zip(("name", "pid"), (name, proc.pid))) for name, proc in processes]
    for ps in pss:
        update_process_in_model(PodModel, ps)
        set_process_state(PodModel, name=ps['name'], pid=ps['pid'], logger=logging, start=True)

    processes = get_ros_model(app_id, pod_id)
    pss = [dict(zip(("name", "pid"), (name, proc.pid))) for name, proc in processes]
    for ps in pss:
        update_process_in_model(PodModel, ps)
        set_process_state(PodModel, name=ps['name'], pid=ps['pid'], logger=logging, start=True)

    await asyncio.sleep(10)

    for ps in PodModel.processes:
        stop_command(ps.pid, logger=logging)
        set_process_state(PodModel, name=ps.name, logger=logging, stop=True)

    allow_topics, dst_names= collect_info_for_debug_pod_bridge(app_id= app_id, debug_node=debug_node_info)

    logging.info(f'Start debug procedure from Debug Node! Lisen on: \
            [{PodInfo.listen_server}:{PodInfo.listen_port}], \
            Allow topics: {allow_topics}, Peers: {dst_names}]')
    dsts = [ f'{name}:{PodInfo.dst_port}' for name in dst_names]
    # start bridge
    processes_pid = start_zenoh_process(debug=PodInfo.debug,
                                    domain_id=PodInfo.domain_id,
                                    allow_topics=list(allow_topics),
                                    dsts = dsts,
                                    listen_server=PodInfo.listen_server,
                                    listen_port=PodInfo.listen_port)
    global BridgeCounter
    pss = [dict(zip(("name", "pid"), (f'{ProcessList.DebugBridgeProcess.name}_{BridgeCounter}', processes_pid.pid)))]
    BridgeCounter +=1

    for ps in pss:
        update_process_in_model(PodModel, ps)
        set_process_state(PodModel, name=ps['name'], pid=ps['pid'], logger=logging, start=True)
    new_msg = msg_start_process(app_id, pod_id, pss)
    await sio.emit(f'started_{ProcessList.DebugBridgeProcess.value}', new_msg.dict())

@sio.on(SocketActionList.PauseGraph.value)
async def pause_graph(msg):
    m = Message(**msg)
    pod_name = m.pod
    for ps in PodModel.processes:
        if ps.name in ShowRosGraphProcesses:
            stop_command(ps.pid, logger=logging)
            set_process_state(PodModel, name=ps.name, logger=logging, stop=True)
    node_folder =pod_node_folder(app_id=PodInfo.app_id, pod_id=PodInfo.pod_id, remote=False)
    life_node_folder=pod_lifecycle_folder(app_id=PodInfo.app_id, pod_id=PodInfo.pod_id, remote=False)
    backup_to_finial(result_path = node_folder,
                        bk_path=Path('_'.join([Path(node_folder).name, 'bk'])))
    backup_to_finial(result_path=life_node_folder,
                    bk_path=Path('_'.join([Path(life_node_folder).name, 'bk'])))
    update_nodes(PodInfo.app_id, PodInfo.pod_id, PodModel)
    update_lifecycle_nodes(PodInfo.app_id, PodInfo.pod_id, PodModel)
    PodModel.update()

@sio.event
async def disconnect():
    await sio.wait()
    logging.info('Disconnected from server')

@sio.event
async def register():
    msg = Message()
    msg.app_id = PodInfo.app_id
    msg.pod_id = PodInfo.pod_id
    msg.pod_ip = PodInfo.pod_ip
    msg.domain_id = PodInfo.domain_id
    global PodModel
    PodModel = Pod(name = PodInfo.pod_id,
                    ip = PodInfo.pod_ip,
                    domain_id = PodInfo.domain_id,
                    socket_id=sio.sid,
                    process=list())
    global BridgeCounter
    BridgeCounter = 0
    await sio.emit('register', msg.dict())

@sio.event
async def cleanup():
    msg = Message()
    msg.app_id = PodInfo.app_id
    msg.pod_id = PodInfo.pod_id
    msg.pod_ip = PodInfo.pod_ip
    msg.domain_id = PodInfo.domain_id
    for ps in ShowRosGraphProcesses:
        set_process_state(PodModel, name=ps, logger=logging, stop=True)
        pid = PodModel.find_process(ps).pid
        stop_command(pid, logger=logging)
    await sio.emit('leave', msg.dict())
    await sio.disconnect()

async def connect(server_ip, server_port):
    await sio.connect(f'http://{server_ip}:{server_port}')
    await sio.wait()

def init():
    logging.basicConfig(
            format='%(asctime)s %(levelname)-8s %(message)s',
            level=logging.INFO,
            datefmt='%Y-%m-%d %H:%M:%S')

def main(server_ip, server_port, podinfo):
    init()
    global PodInfo
    PodInfo = podinfo
    loop = asyncio.get_event_loop()
    asyncio.ensure_future(connect(server_ip, server_port))

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        loop.run_until_complete(cleanup())
    finally:
        loop.set_debug(True)
        logging.info('Closed all connections, shutting down')
