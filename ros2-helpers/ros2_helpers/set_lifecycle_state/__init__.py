from ros2cli.node.direct import DirectNode

from ros2lifecycle.api import call_change_states
from ros2lifecycle.api import call_get_available_transitions

from ros2_model import NodeArgs
import sys

def set_lifecycle_state(node_name, action):  # noqa: D102
    args = NodeArgs(node_name=f'set_{node_name}_state')
    with DirectNode(args) as node:
        transitions = call_get_available_transitions(
            node=node, states={node_name: None})
        transitions = transitions[node_name]
        if isinstance(transitions, Exception):
            return 'Exception while calling service of node ' \
                f"'{args.node_name}': {transitions}"

        # identify requested transition
        for transition in [t.transition for t in transitions]:
            if transition.label == action:
                break
        else:
            for transition in [t.transition for t in transitions]:
                if str(transition.id) == action:
                    break
            else:
                return \
                    'Unknown transition requested, available ones are:' + \
                    ''.join(
                        f'\n- {t.transition.label} [{t.transition.id}]'
                        for t in transitions)

        results = call_change_states(
            node=node, transitions={node_name: transition})
        result = results[node_name]

        # output response
        if isinstance(result, Exception):
            print(
                'Exception while calling service of node '
                f"'{args.node_name}': {result}", file=sys.stderr)
        elif result:
            print('Transitioning successful')
            return True
        else:
            print('Transitioning failed', file=sys.stderr)
            return False