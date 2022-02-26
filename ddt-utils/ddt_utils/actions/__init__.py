from ddt_utils.utils import BridgeMode
from ros2_model import Node
from ddt_utils.model import Pod
import subprocess
from shlex import split
import os
import psutil
import signal
import time

def get_bridge_mode(node: Node, if_probe=True):
    bridge_mode = BridgeMode.NoNeed
    if len(node.publishers):
        bridge_mode = BridgeMode.Listener if if_probe else BridgeMode.Peer
        return BridgeMode.ListenerPeer if len(node.subscribers) else bridge_mode
    elif len(node.subscribers):
        return BridgeMode.Peer if len(node.subscribers) else BridgeMode.Listener
    else:
        return bridge_mode

def set_process_state(pod: Pod, *, name, **kwargs):
    p = pod.find_process(name)
    flag_start = kwargs.get('start', False)
    flag_stop = kwargs.get('stop', False)
    logger = kwargs.get('logger')
    msg = f'Set Process [{name}] state from [{"start" if p.state else "stop"}] to [{"start" if flag_start else "stop"}]'
    logger.info(msg) if logger else print(msg)
    if p.state is False and flag_start:
        try:
            pid = kwargs['pid']
            p.start(pid=pid)
            msg = f'Start Proceess[name: {name}, pid: {pid}]'
            logger.info(msg) if logger else print(msg)
        except KeyError:
            msg = f"Didn't find pid of Proceess[name: {name}], can't set state to start"
            logger.info(msg) if logger else print(msg)
    if p.state and flag_stop:
        msg = f'Stop Proceess[name: {name}, pid: {[p.pid]}]'
        p.stop()
        logger.info(msg) if logger else print(msg)

def start_command(command, **kwargs):
    logger = kwargs.get('logger')
    msg = f'Start command: {command}'
    c = split(command)
    pid = subprocess.Popen(c)
    return pid

def stop_command(pid, **kwargs):
    logger = kwargs.get('logger')
    result = False
    if not result:
        check_time = time.time()
    if pid:
        if isinstance(pid, int):
            p = psutil.Process(pid)
            # os.kill(pid, signal.SIGINT)
            p.send_signal(signal.SIGTERM)
            if logger:
                msg = f'kill process pid[{pid}]'
                logger.info(msg)
            else:
                print(msg)
            try:
                for child in p.children():
                    child.send_signal(signal.SIGTERM)
                    # os.kill(child.pid, signal.SIGINT)
                    if logger:
                        msg = f'Find child process pid[{pid}]'
                        logger.info(msg)
                    else:
                        print(msg)
            except psutil.NoSuchProcess:
                pass

        else:
            pid.send_signal(subprocess.signal.SIGTERM)
            if logger:
                msg = f'send signal to kill process pid[{pid}]'
                logger.info(msg)
            else:
                print(msg)

def call_command(command):
    print("Start command: ", command)
    c = split(command)
    subprocess.call(c)

def check_pid_running(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        msg = f'pid[{pid}] is unassigned"'
        return msg, time.time(), False
    else:
        msg = f'pid[{pid}] is in use"'
        return msg, time.time(), True
