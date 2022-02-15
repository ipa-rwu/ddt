#! /usr/bin/env python3
from ros2lifecycle.api import get_node_names
from ros2_graph_quest.lifecycle import call_get_states, call_get_available_transitions
from ros_model import LifeCycleNode,LifeCyclePrimaryState, NodeArgs, LifeCycleAction, LifeCycleActionsEnum
from pathlib import Path
from shutil import rmtree
import argparse

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import NodeStrategy
from ros2lifecycle.api import get_node_names


class RunTimeLifeCycleNode(LifeCycleNode):
    pass

    def __init__(self, *, node: LifeCycleNode, **data):
        super().__init__(
                        name=node.name,
                        full_name = node.full_name,
                        namespace = node.namespace,
                        current_state=node.current_state,
                        potential_actions = node.potential_actions,
                        **data)

    class Config:
        arbitrary_types_allowed = True

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", dest="folder", help= "folder path", default=None)
    args = parser.parse_args()
    result_path = args.folder
    if result_path is None:
        result_path = Path.home() / 'tmp' / 'lifecycle_parser'
    rmtree(result_path, ignore_errors=True)
    try:
        while True:
            life_node = get_life_node_names()
            for l in life_node:
                *_, state_id = get_state(l.full_name)
                actions = get_potential_action(l.full_name)
                parsed_node = RunTimeLifeCycleNode(node = LifeCycleNode(
                                                    name=l.name,
                                                    namespace=l.namespace,
                                                    full_name=l.full_name,
                                                    current_state = LifeCyclePrimaryState(state_id),
                                                    potential_actions=list(actions)))
                f_path = Path(str(result_path) + f'{l.full_name}.json')
                f_path.parent.mkdir(parents=True, exist_ok=True)
                with open(f_path, "w+") as f:
                    f.write(parsed_node.json(indent=4, sort_keys=True))
    except KeyboardInterrupt:
        print("quit")
if __name__ == '__main__':
    main()
