from pydantic import BaseModel
from typing import List
from typing import Optional
from enum import Enum, IntEnum, auto
import logging

class NodeName(BaseModel):
    name: str
    namespace: str
    full_name : str

class Interface(BaseModel):
    name: str
    types: List[str]

class InterfaceType(IntEnum):
    Publisher       = auto()
    Subscriber      = auto()
    ActionServer    = auto()
    ActionClient    = auto()
    ServiceServer   = auto()
    ServiceClient   = auto()

class Node(BaseModel):
    nodename:        NodeName
    publishers:      List[Interface]=list()
    subscribers:     List[Interface]=list()
    action_servers:  List[Interface]=list()
    action_clients:  List[Interface]=list()
    service_servers: List[Interface]=list()
    service_clients: List[Interface]=list()

    class Config:
        arbitrary_types_allowed = True

class Connection(BaseModel):
    entity: Interface
    emission: Optional[List[Node]]
    reception: Optional[List[Node]]

    def add_emission(self, node:Node):
        self.emission.append(node)
    def add_reception(self, node:Node):
        self.reception.append(node)

class LifeCyclePrimaryState(IntEnum):
    Unknown        = 0
    Unconfigured   = 1
    Inactive       = 2
    Active         = 3
    Finalized      = 4

class LifeCycleActionsEnum(str, Enum):
    Create         = "create"
    Configure      = "configure"
    Cleanup        = "cleanup"
    Activate       = "activate"
    Deactivate     = "deactivate"
    Shutdown       = "shutdown"
    Destroy        = "destroy"
    Wait           = "wait"

class LifeCycleAction(BaseModel):
    name: str
    id: int

class LifeCycleNode(BaseModel):
    current_state: LifeCyclePrimaryState=LifeCyclePrimaryState.Unknown
    potential_actions: List[LifeCycleAction]=list()
    nodename: NodeName

    class Config:
        arbitrary_types_allowed = True

    def get_transition_number(self, action_name):
        for action in self.potential_actions:
            if action_name == action.name:
                return action.id
        else:
            logging.error(f'Transition [{action_name}] is not available for Lifecycle Node [{self.nodename.full_name}]')

class NodeArgs(BaseModel):
    node_name: str
    include_hidden_nodes: bool=False
    verbose: bool=False
