import subprocess

class WiFiHotSpot:
    def __init__(self, hotSpotName="DronesNetworks"):
        subprocess.call(["OBC/network/HotSpotMaker.bash", hotSpotName])

class WiFiClient:
    def __init__(self, hotSpotName="PupaLupa"):
        subprocess.call(["OBC/network/HotSpotConnector.bash", hotSpotName])
