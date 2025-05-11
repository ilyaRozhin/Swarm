from socket import *
import pickle
import threading
from network.WiFi import WiFiClient, WiFiHotSpot

BUFF_SIZE = 1024

class Client:
    def __init__(self, id, blocker):
        self.id = id
        self.udp_sock = socket(AF_INET, SOCK_DGRAM)
        self.blocker = blocker
        self.package = None
        self.retryies = 3
    
    def loop(self):
        while True:
            if not self.package is None:
                self.blocker.clear()
                for i in range(0, self.retryies):
                    self.udp_sock.sendto(pickle.dumps((self.id, self.package['data'])),
                                        (self.package['host'], self.package['port']))
                self.package = None
                self.blocker.set()

    def send_data(self, host, port, data):
        self.package = {'data': data, 'host': host, 'port': port}

class Server: 
    def __init__(self, buf_size, blocker):
        self.udp_sock = socket(AF_INET, SOCK_DGRAM)
        self.buf_size = buf_size
        self.data = []
        self.blocker = blocker

    def loop(self, host, port):
        self.udp_sock.bind((host, port))
        self.blocker.set()
        while True:
            if self.blocker.wait():
                data, _ = self.udp_sock.recvfrom(self.buf_size)
                package = pickle.loads(data)
                self.data.append(package)

class NetworkNode:
    def __init__(self, id, host, port, hotspot=False):
        self.id = id
        self.own_host = host
        self.own_port = port
        self.blocker = threading.Event()

        if hotspot:
            WiFiHotSpot()
        else:
            WiFiClient()
            
        self.client = Client(id, self.blocker)
        self.server = Server(BUFF_SIZE, self.blocker)
    
    def activate(self):
        return [threading.Thread(target=self.client.loop, daemon=True), 
                threading.Thread(target=self.server.loop, args=[self.own_host, self.own_port], daemon=True)]