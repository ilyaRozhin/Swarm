import cv2
from FCU.control.commands import MAVLinkClient
from FCU.utils import ConnectionInfo
from FCU.connection import launch_mavproxy
from FCU.telemetry.data import Telemetry
from Devices.Camera.polling import PolledCameras
from Devices.HC_SR04.handler import Device
from network.commutators import NetworkNode
import pandas as pd

BASE_PROTO = "udp"

class BaseSpecClass:
    def __init__(self, id, host, port, specialization, general_table):
        launch_mavproxy(id)
        self.id = id
        self.specialization = specialization
        self.host = host
        self.port = port
        self.general_table = general_table
        self.net = NetworkNode(
            id, host, port
        )
        self.mav = MAVLinkClient(
            ConnectionInfo(BASE_PROTO, "127.0.0.1", port=14550),
            sourceSystem=id
        )
        self.telem = Telemetry(
            self.mav
        )
        self.camera_device = cv2.VideoCapture(
            PolledCameras().activate()[-1]
        )
        self.ultrasonic_device = Device()
        self.threads = []

    def base_activate(self):
        self.mav.start_connection()
        self.threads.extend(self.net.activate)
        self.threads.append(self.telem.activate)
    
    def recv_telemetry(self):
        self.general_table


    
    
        
