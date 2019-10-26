from services.MeshNode import MeshNode
from apis.Messages import *


class NonAgent(MeshNode):
    def __init__(self):
        super().__init__()
        # self.dispatchTable[]

    def isAgentNode(self) -> bool:
        return False
