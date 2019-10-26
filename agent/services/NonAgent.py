from typing import Dict, Callable

from MeshNode import MeshNode
from apis.Messages import *


class NonAgent(MeshNode):
    def __init__(self):
        super().__init__()
        #self.dispatchTable[]

    def dispatch(self, req: Request) -> Response:
        fcnName = req.target
        fcnArgs = req.args
        self.dispatchTable[fcnName](self, *fcnArgs)