from services.MeshNode import MeshNode
from services.NonAgent import NonAgent, Request



def test_NonAgent():
    na = NonAgent()
    na.bind()
    na.start()
    nb = NonAgent()
    nb.bind()
    nb.start()
    nc = NonAgent()
    nc.bind()
    nc.start()
    print("pn: ", nc.portNumber)
    p = nc.portNumber

    ssidReq = Request(function='ssid', args=[])
    fncsReq = Request(function='list_functions', args=[])

    ssidResponse = MeshNode.call(p, ssidReq)
    fncsResponse = MeshNode.call(p, fncsReq)
    dumbResponse = MeshNode.call(p, Request(function='is_coopai', args=[]))

    print("SSID query response: ", ssidResponse.response)
    print("fucntions query response: ", fncsResponse.response)
    print("dumbass query response: ", dumbResponse.response)

    del na
    del nb
    del nc


test_NonAgent()

