from pydantic import BaseModel
from typing import List

class Interface(BaseModel):
    name: str
    types: List[str]

class Node(BaseModel):
    name: str
    namespace: str
    full_name : str
    publishers:      List[Interface]=list()
    subscribers:     List[Interface]=list()
    action_servers:  List[Interface]=list()
    action_clients:  List[Interface]=list()
    service_servers: List[Interface]=list()
    service_clients: List[Interface]=list()

    class Config:
        arbitrary_types_allowed = True
