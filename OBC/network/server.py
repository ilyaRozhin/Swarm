from socket import create_server

class DroneSever:
    def __init__(self, address=("127.0.0.1", 8000)):
        self.input_gateway = create_server(address=address)
    def __enter__(self):
        return self.input_gateway
    def __exit__(self, *args):
        self.input_gateway.close()

if __name__ == "__main__":
    with DroneSever() as drone:
        while True:
            conn, addr = drone.accept()
            drone.
            

