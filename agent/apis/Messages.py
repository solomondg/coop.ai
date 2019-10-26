from abc import ABC
from typing import List
from enum import Enum
import pickle as pkl


class RequestStatus(Enum):
    SUCCESS = 0
    FAILURE = 1


class MessageType(Enum):
    REQUEST = 0
    RESPONSE = 1


class Message(ABC):
    type: MessageType

    def serialize(self) -> str:
        return pkl.dumps(self)

    @staticmethod
    def load(message: str):
        obj = pkl.loads(message)
        return obj


class Request(Message):
    function: str
    args: List

    def __init__(self, function: str, args=None):
        self.type = MessageType.REQUEST
        self.function = function
        self.args = args if args is not None else []


class Response(Message):
    status: RequestStatus
    response: object

    def __init__(self, status: RequestStatus, response=None):
        self.type = MessageType.RESPONSE
        self.status = status
        self.response = response


if __name__ == "__main__":
    req = Request(function="getSSID", args=["hello", "world", 3])
    resp = Response(RequestStatus.SUCCESS, response="933.x821FREEEEEEE")

    reqSerialized = req.serialize()
    respSerialized = resp.serialize()

    print(pkl.loads(reqSerialized).type)
    print(pkl.loads(respSerialized).type)
