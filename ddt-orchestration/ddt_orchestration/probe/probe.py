#! /usr/bin/env python3
import asyncio
import socketio
import argparse
import logging
from pathlib import Path

from ddt_orchestration.model import Message
from ddt_orchestration.manager.action import make_ros_graph, get_ros_model, stop_command, pre_make_ros_graph #, quit_handler
from ddt_orchestration.utils import *

ManagerPwd = Path(__file__).parent.parent
TmpFolder = ManagerPwd / 'tmp'

sio = socketio.AsyncClient(handle_sigint=False)

def show_ros_graph(app):
    app_path = Path(TmpFolder / app)
    pids = make_ros_graph(app, f'app_{app}', app_path)
    return pids

@sio.event
async def connect():
    logging.info(f'connected to server')
    await register()

@sio.on('show_graph')
async def show_graph(msg):
    m = Message(**msg)
    app_id = PodInfo.app_id
    pod_id = PodInfo.pod_id
    if m.app_id == app_id and m.pod_id == pod_id:
        org_graph = get_pod_rosgraph_dot_path(app_id, pod_id)
        processes = pre_make_ros_graph(pod_id, Path(get_pod_folder(app_id, pod_id)))
        while org_graph.is_file():
            show_ros_graph(app_id)
            break
        new_msg = Message()
        ps = [dict(zip(("name", "pid"), (name, proc.pid))) for name, proc in processes]
        logging.info(f'{pod_id} from {app_id} start showing graph, processes: {ps}')
        new_msg.processes = ps
        new_msg.app_id = app_id
        new_msg.pod_id = pod_id
        await sio.emit('started_rosgrah', new_msg.dict())

@sio.on('get_node_model')
async def get_node_model(msg):
    m = Message(**msg)
    app_id = PodInfo.app_id
    pod_id = PodInfo.pod_id
    if m.app_id == app_id and m.pod_id == pod_id:
        # create node model into files
        process = get_ros_model(Path(get_pod_node_folder(app_id, pod_id)))
        new_msg = Message()
        new_msg.pid = process.pid
        new_msg.app_id = app_id
        new_msg.pod_id = pod_id
        await sio.emit('started_rosmodel_parser', new_msg.dict())

@sio.on('pause_graph')
async def pause_graph(msg):
    for pid in msg.values():
        logging.info(f'Stopping pid: {pid}')
        stop_command(pid)

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
    await sio.emit('register', msg.dict())

@sio.event
async def cleanup():
    msg = Message()
    msg.app_id = PodInfo.app_id
    msg.pod_id = PodInfo.pod_id
    msg.pod_ip = PodInfo.pod_ip
    msg.domain_id = PodInfo.domain_id
    await sio.emit('leave', msg.dict())
    await sio.disconnect()

async def main():
    await sio.connect('http://localhost:1234')
    await sio.wait()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", dest="app_id", help= "application name", default=None)
    parser.add_argument("-p", dest="pod_id", help= "pod name", default=None)
    parser.add_argument("-ip", dest="pod_ip", help= "pod ip", default=None)
    parser.add_argument("-d", dest="domain_id", help= "domain id", default=0)
    args = parser.parse_args()
    global PodInfo
    PodInfo = args

    logging.basicConfig(
            format='%(asctime)s %(levelname)-8s %(message)s',
            level=logging.INFO,
            datefmt='%Y-%m-%d %H:%M:%S')
    loop = asyncio.get_event_loop()
    asyncio.ensure_future(main())

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        loop.run_until_complete(cleanup())
    finally:
        loop.set_debug(True)
        logging.info('Closed all connections, shutting down')
