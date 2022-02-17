from lifecycle_msgs.srv import GetState
from lifecycle_msgs.srv import GetAvailableTransitions
import rclpy

def call_get_states(*, node, node_names):
    clients = {}
    futures = {}
    # create clients
    for node_name in node_names:
        clients[node_name] = \
            node.create_client(GetState, f'{node_name}/get_state')

    # wait until all clients have been called
    while True:
        for node_name in [n for n in node_names if n not in futures]:
            # call as soon as ready
            client = clients[node_name]
            if client.service_is_ready():
                request = GetState.Request()
                future = client.call_async(request)
                futures[node_name] = future

        if len(futures) == len(clients):
            break
        rclpy.spin_once(node, timeout_sec=1.0)

    # wait for all responses
    for future in futures.values():
        rclpy.spin_once(node, timeout_sec=1.0)

    # return current state or exception for each node
    states = {}
    for node_name, future in futures.items():
        if future.result() is not None:
            response = future.result()
            states[node_name] = response.current_state
        else:
            states[node_name] = future.exception()
    return states

def call_get_available_transitions(*, node, states):
    return _call_get_transitions(node, states, 'get_available_transitions')


def call_get_transition_graph(*, node, states):
    return _call_get_transitions(node, states, 'get_transition_graph')


def _call_get_transitions(node, states, service_name):
    clients = {}
    futures = {}
    # create clients
    for node_name in states.keys():
        clients[node_name] = node.create_client(
            GetAvailableTransitions, f'{node_name}/{service_name}')

    # wait until all clients have been called
    while True:
        for node_name in [n for n in states.keys() if n not in futures]:
            # call as soon as ready
            client = clients[node_name]
            if client.service_is_ready():
                request = GetAvailableTransitions.Request()
                future = client.call_async(request)
                futures[node_name] = future

        if len(futures) == len(clients):
            break
        rclpy.spin_once(node, timeout_sec=1.0)

    # wait for all responses
    for future in futures.values():
        rclpy.spin_once(node, timeout_sec=1.0)

    # return transitions from current state or exception for each node
    transitions = {}
    for node_name, future in futures.items():
        if future.result() is not None:
            response = future.result()
            transitions[node_name] = []
            for transition_description in response.available_transitions:
                if (
                    states[node_name] is None or
                    transition_description.start_state == states[node_name]
                ):
                    transitions[node_name].append(
                        transition_description)
        else:
            transitions[node_name] = future.exception()
    return transitions