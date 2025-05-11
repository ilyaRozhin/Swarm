#!/usr/bin/env python3

import json
from FCU.utils import calculate_flight_direction, VideoRecord, Coordinates
from enum import Enum
import time
import threading


class DroneStatus(Enum):
    OFFLINE = 0  #'offline'
    WAITING = 1  #'Waiting'
    ACCOMPLISHMENT = 2  #'mission accomplishment'
    TAKEOFF = 3  #'takeoff'
    RETURN = 4  #'return home'


class DroneData:
    def __init__(self):
        self.altitude: float
        self.f_coordinates: Coordinates
        self.battery_level: float
        self.heading: float
        self.speed: float
        self.status: str

    def update(
        self,
        altitude,
        f_coordinates,
        battery_level,
        heading,
        speed,
        status,
    ):
        self.altitude: float = altitude
        self.f_coordinates: Coordinates = f_coordinates
        self.battery_level: float = battery_level
        self.heading: float = heading
        self.speed: float = speed
        self.status: str = status

    def json(self):
        return json.dumps(
            {
                "position": {
                    "point": {
                        "lat": self.f_coordinates.latitude,
                        "long": self.f_coordinates.longitude,
                        "height": self.altitude,
                    },
                    "speed": self.speed,
                },
                "battery_level": self.battery_level,
                "heading": self.heading,
                "status": self.status,
            }
        )


class Telemetry:
    def __init__(self, mavlink):
        self.mavlink = mavlink
        self.data = DroneData()
        self.buffer: dict = {
            "mission_received": 0,
            "change_status": 0,
            "mission_completed": 0,
            "mission_cancelled": 0,
        }

    def activate(self):
        return threading.Thread(self.update)

    def update(self):
        while True:
            battery = self.get_bat()
            altitude, latitude, longitude = self.get_coor()
            direction = self.get_direction()
            heading, speed = self.get_sp_head()
            status = self.mavlink.tata
            self.data.update(
                altitude=altitude,
                f_coordinates=Coordinates(latitude, longitude),
                battery_level=battery,
                heading=heading,
                speed=speed,
                flight_direction=direction,
                status=status,
            )
            time.sleep(0.1)

    def get_bat(self):
        msg = self.mavlink.mav.recv_match(type="SYS_STATUS", blocking=True, timeout=2)
        if msg is not None:
            return msg.battery_remaining
        else:
            return None

    def get_coor(self):
        msg = self.mavlink.mav.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=2
        )
        if msg is not None:
            altitude = msg.relative_alt / 1000.0
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            return [altitude, lat, lon]
        else:
            return [None, None, None]

    def get_direction(self):
        msg = self.mavlink.mav.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=2
        )
        if msg is not None:
            vx = msg.vx / 100.0
            vy = msg.vy / 100.0
            flight_direction = calculate_flight_direction(vx, vy)
            return flight_direction
        else:
            return None

    def get_sp_head(self):
        msg = self.mavlink.mav.recv_match(type="VFR_HUD", blocking=True, timeout=2)
        if msg is not None:
            heading = msg.heading
            speed = msg.groundspeed
            return [heading, speed]
        else:
            return [None, None]
