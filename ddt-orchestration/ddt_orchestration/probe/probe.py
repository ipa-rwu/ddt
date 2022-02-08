import asyncio
import socketio
import argparse
import json
import logging
from pathlib import Path
import os

from ddt_orchestration.model.ddt_model import Application, Pod, Message, Process
from ddt_orchestration.manager.action import make_ros_graph, pause_ros_graph, stop_command, pre_make_ros_graph

ManagerPwd = Path(__file__).parent.parent
TmpFolder = ManagerPwd / 'tmp'

sio = socketio.AsyncClient()

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
    app_id = m.app_id
    app_folder = Path(TmpFolder, app_id)
    org_graph = app_folder / f'{app_id}.dot'

    processes = pre_make_ros_graph(app_id, Path(TmpFolder / app_id))
    while org_graph.is_file():
        show_ros_graph(app_id)
        break
    new_msg = Message()
    ps = [dict(zip(("name", "pid"), (name, proc.pid))) for name, proc in processes]
    new_msg.processes = ps
    new_msg.app_id = app_id
    await sio.emit('start_rosgrah', new_msg.dict())

@sio.on('pause_graph')
async def pause_graph(msg):
    m = Message(**msg)
    for pid in msg.values():
        print("stop: ", pid)
        stop_command(pid)

@sio.event
async def disconnect():
    print('disconnected from server')

async def main():
    await sio.connect('http://localhost:1234')
    await sio.wait()

@sio.event
async def register():
    msg = {"App": ID.app_id,
    "Pod": ID.pod_id,
    "PodIP": ID.pod_ip,
    "Sio": sio.sid}
    data = json.dumps(msg)
    await sio.emit('register',{'data': data})


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", dest="app_id", help= "application name", default=None)
    parser.add_argument("-p", dest="pod_id", help= "pod name", default=None)
    parser.add_argument("-ip", dest="pod_ip", help= "pod ip", default=None)
    args = parser.parse_args()
    global ID
    ID = args

    logging.basicConfig(
            format='%(asctime)s %(levelname)-8s %(message)s',
            level=logging.INFO,
            datefmt='%Y-%m-%d %H:%M:%S')
    asyncio.run(main())
