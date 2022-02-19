from ddt_utils.utils import BridgeMode
from ros2_model import Node
from ddt_utils.model import Pod

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
    if not p.state and flag_start:
        try:
            pid = kwargs['pid']
            p.start(pid=pid)
            try:
                msg = f'Start Proceess[name: {name}, pid: {pid}]'
                kwargs['logger'].info(msg)
            except KeyError:
                print(msg)
        except KeyError:
            msg = f"Didn't find pid of Proceess[name: {name}], can't set state to start"
            try:
                kwargs['logger'].error(msg)
            except KeyError:
                print(msg)
    elif p.state and flag_stop:
        p.stop()
        try:
            msg = f'Stop Proceess[name: {name}]'
            kwargs['logger'].info(msg)
        except KeyError:
            print(msg)
