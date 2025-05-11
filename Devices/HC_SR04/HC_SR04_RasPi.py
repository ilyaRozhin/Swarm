import threading
import pigpio

class DistanceReader:
    def __init__(self, TrigPin, EchoPin):
        self.Trig = TrigPin
        self.Echo = EchoPin
        self.done = threading.Event()
        self.pi = pigpio.pi()
        self.high = None
        self.low = None
        self.pi.set_mode(self.Trig, pigpio.OUTPUT)
        self.pi.set_mode(self.Echo, pigpio.INPUT)
        self.pi.callback(self.Echo, pigpio.RISING_EDGE, self.rise)
        self.pi.callback(self.Echo, pigpio.FALLING_EDGE, self.fall)

    def rise(self, gpio, level, tick):
        self.high = tick
    
    def fall(self, gpio, level, tick):
        self.low = tick - self.high
        self.done.set()

    def generate_signal(self, time, level):
        self.pi.gpio_trigger(self.Trig, time, level)
        return time

    def accept_signal(self):
        self.done.clear()
        if self.done.wait(timeout=5):
            return self.low / 29.0 / 100.0
        