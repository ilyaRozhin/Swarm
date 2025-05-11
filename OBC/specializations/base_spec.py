from FCU.control.commands import MAVLinkClient
from FCU.telemetry.data import Telemetry
from Devices.Camera.polling import PolledCameras
from Devices.HC_SR04.handler import Device
from network.WiFi import WiFiClient
from socket import socket

class BaseSpecClass:
    def __init__(self, id):
        self.id = id
        se
