import subprocess
import time

class PolledCameras:
    def __init__(self):
        self.available_devices = []
    
    def activate(self):
        self.available_devices = subprocess.getstatusoutput(f'ls /dev/video*')[-1].split("\n")
        print("Push video device to USB ...")
        differ = []
        while differ == []:
            new_available_devices = subprocess.getstatusoutput(f'ls /dev/video*')[-1].split("\n")
            differ = list(set(new_available_devices) - set(self.available_devices))
            time.sleep(1)
            #print(differ)
        return differ[-1]

if __name__ == "__main__":
    PolledCameras().activate()