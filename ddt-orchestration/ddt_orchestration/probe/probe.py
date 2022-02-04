import asyncio
import socketio
import argparse
import json
import logging
import time

sio = socketio.AsyncClient()

@sio.event
async def connect():
    logging.info(f'connected to server')
    # await sio.emit('connect', {'data': 'I\'m connected! as {sio.sid}'})
    await register()

@sio.event
async def join():
    msg = {"App": ID.app_id,
        "Pod": ID.pod_id,
        "PodIP": ID.pod_ip,
        "Sio": sio.sid}
    data = json.dumps(msg)
    logging.info(f'[{msg["Pod"]}] from [{msg["App"]}] registered to server as {sio.sid}')
    await sio.emit('my_join', data)

@sio.event
async def my_message(data):
    print('message received with ', data)
    await sio.emit('my response', {'response': 'my response'})

@sio.event
async def disconnect():
    print('disconnected from server')

async def main():
    await sio.connect('http://localhost:5000')
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

import socketio
