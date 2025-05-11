import json
import subprocess
import time
import serial
from Devices.HC_SR04.HC_SR04_RasPi import DistanceReader

class Device:
    def __init__(self, settings="Devices/HC_SR04/using_variant.json"):
        self.config = None
        self.settings = settings
        with open(settings) as file:
            self.config = json.load(file)
        self.serPort = None
        self.baudRate = 9600
        self.ser = None
        self.result = None
        self.getter = None

    def polled(self):
        if self.config["port_name"] == "":
            available_devices = subprocess.getstatusoutput(f'ls /dev/tty*')[-1].split("\n")
            print("Push Arduino Device to USB ...")
            differ = []
            while differ == []:
                new_available_devices = subprocess.getstatusoutput(f'ls /dev/tty*')[-1].split("\n")
                differ = list(set(new_available_devices) - set(self.available_devices))
                time.sleep(1)
            self.config["port_name"] = differ[-1]
            with open(self.settings,'w') as fi:
                json.dump(self.config,fi)
            return differ[-1]

    def activate(self, request_type): 
        if self.config["link_type"] == "Arduino":
            self.serPort = self.polled()
            self.ser = serial.Serial(self.serPort, self.baudRate)
            self.getter = self.ser
            if request_type == "Generate":
                self.ser.write(1)
                time.sleep(1)
                self.ser.write(0)
            else:
                self.ser.write(2)
                time.sleep(0.7)
                self.result = int(self.getter.read(9))
                self.ser.write(0)

        elif self.config["link_type"] == "RasPi":
            self.getter = DistanceReader(self.config["trig_pin"], self.config["echo_pin"])
            if request_type == "Generate":
                self.getter.generate_signal(10, 1)
            else:
                self.result = self.getter.accept_signal()

if __name__ == "__main__":
    Device()