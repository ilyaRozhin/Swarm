import subprocess

BASE_DEVS = [
    "/dev/ttyACM0", 
    "/dev/ttyTCM0",
    "/dev/ttyS30"
]

def polled():
    available_devices = set(subprocess.getstatusoutput(f'ls /dev/tty*')[-1].split("\n"))
    #print(available_devices)
    return list(set(BASE_DEVS).intersection(available_devices))[0]

def launch_mavproxy(source_system):
    phys_device = polled()
    command = "mavproxy.py --master=$1 --out=127.0.0.1:14550 --baudrate=115200 --source_system=$2"
    try:
        subprocess.call(["mavproxy.py", "--master", f"{phys_device}", "--out", 
                     "127.0.0.1:14550", "--baudrate", "115200", "--source-system", f"{source_system}"])
    except:
        print("Device didn't exist ...")
