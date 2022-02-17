#! /usr/bin/env python3
import asyncio
import socketio
import logging
from pathlib import Path

from ddt_utils.model import Message
from ddt_utils.utils import get_pod_folder

from ddt_probe.action import get_ros_model
from ddt_probe.action import stop_command
from ddt_probe.action import pre_make_ros_graph

sio = socketio.AsyncClient(handle_sigint=False)

@sio.event
async def connect():
    logging.info(f'connected to server')
    await register()

@sio.on('show_graph')
async def show_graph(msg):
    m = Message(**msg)
    logging.info(f'show_graph: get message: {m.dict()}')

    app_id = PodInfo.app_id
    pod_id = PodInfo.pod_id
    if m.app_id == app_id and m.pod_id == pod_id:
        processes = pre_make_ros_graph(pod_id, Path(get_pod_folder(app_id, pod_id)))
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
        process = get_ros_model(app_id, pod_id)
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
