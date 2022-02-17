from pydantic import BaseModel
from typing import List
from enum import Enum, IntEnum

class NodeName(BaseModel):
    name: str
    namespace: str
    full_name : str

class Interface(BaseModel):
    name: str
    types: List[str]

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

class NodeArgs(BaseModel):
    node_name: str
    include_hidden_nodes: bool=False