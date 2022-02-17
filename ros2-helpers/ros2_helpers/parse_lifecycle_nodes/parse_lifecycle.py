#! /usr/bin/env python3
from pathlib import Path

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import NodeStrategy
from ros2lifecycle.api import get_node_names
from ros2lifecycle.api import get_node_names

from ros2_helpers.utils import save_to_file
from ros2_helpers.parse_lifecycle_nodes.lifecycle import call_get_states, call_get_available_transitions
from ros2_model import LifeCycleNode,LifeCyclePrimaryState, NodeArgs, LifeCycleAction, LifeCycleActionsEnum, NodeName

def get_life_node_names():
    args = NodeArgs(node_name="get_all_lifecycle_nodes")
    with NodeStrategy(args) as node:
        node_names = get_node_names(
            node=node, include_hidden_nodes=args.include_hidden_nodes)
        return node_names

def get_state(node_name):  # noqa: D102
    args = NodeArgs(node_name=f'check_{node_name}_state')
    with DirectNode(args) as node:
        states = call_get_states(node=node, node_names=[node_name])
        # output exceptions
        for node_name in sorted(states.keys()):
            state = states[node_name]
            if isinstance(state, Exception):
                # print(
                #     'Exception while calling service of node '
                #     f"'{node_name}': {state}",
                #     file=sys.stderr)
                del states[node_name]
                if args.node_name:
                    return 1

        # output current states
        for node_name in sorted(states.keys()):
            state = states[node_name]
            if state is not None:
                return node_name, state.label, state.id
            else:
                return node_name, None, LifeCyclePrimaryState.Unknown

def get_potential_action(node_name):
    args = NodeArgs(node_name=f'check_{node_name}_state')
    with DirectNode(args) as node:
        transitions = call_get_available_transitions(
            node=node, states={node_name: None})
        transitions = transitions[node_name]
        if isinstance(transitions, Exception):
            return 'Exception while calling service of node ' \
                f"'{args.node_name}': {transitions}"
        if transitions is not None:
            for t in transitions:
                action = LifeCycleAction(name=LifeCycleActionsEnum.Wait, id=0)
                # print(
                #     f'- {t.transition.label} [{t.transition.id}]\n'
                #     f'\tStart: {t.start_state.label}\n'
                #     f'\tGoal: {t.goal_state.label}')
                action.name = t.transition.label
                action.id = t.transition.id
                yield action


def parse(nodename: NodeName):
    *_, state_id = get_state(nodename.full_name)
    actions = get_potential_action(nodename.full_name)
    parsed_node = LifeCycleNode(nodename=nodename,
                                current_state = LifeCyclePrimaryState(state_id),
                                potential_actions=list(actions))
    return parsed_node

def main(result_path):
    life_nodes = get_life_node_names()
    for l in life_nodes:
        life_node = NodeName(name=l.name, full_name = l.full_name, namespace=l.namespace)
        parsed_node = parse(life_node)
        save_to_file(result_path, l.full_name, parsed_node)

if __name__ == '__main__':
    main()
