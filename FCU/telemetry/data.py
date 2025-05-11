#!/usr/bin/env python3

import json
from FCU.utils import calculate_flight_direction, VideoRecord, Coordinates
from enum import Enum
import time

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
        self.mission_id: str
        self.command_id: str
        self.flight_direction: float
        self.video: int
        self.modecontrol: bool

    def update(
        self,
        altitude,
        f_coordinates,
        battery_level,
        heading,
        speed,
        status,
        mission_id,
        command_id,
        flight_direction,
        video,
        modecontrol,
        pause
    ):
        self.altitude: float = altitude
        self.f_coordinates: Coordinates = f_coordinates
        self.battery_level: float = battery_level
        self.heading: float = heading
        self.speed: float = speed
        self.status: str = status
        self.mission_id: str = mission_id
        self.command_id: str = command_id
        self.flight_direction: float = flight_direction
        self.video: bool = video
        self.modecontrol: bool = modecontrol
        self.pause: bool = pause

    def json(self):
        return json.dumps(
            {
                # "height": self.altitude,
                "position": {
                    "point": {
                        "lat": self.f_coordinates.latitude,
                        "long": self.f_coordinates.longitude,
                        "height": self.altitude,
                    },
                    "direction": self.flight_direction,
                    "speed": self.speed,
                },
                "battery_level": self.battery_level,
                "heading": self.heading,
                "status": self.status,
                "mission_id": self.mission_id,
                "command_id": self.command_id,
                "use_ai_video": self.video,
                "aim": None,
                "use_hand_mode": self.modecontrol,
                'pause' : self.pause,
                # "ram": None,
                # "f_coordinates": {'lat': None, 'lon': None},
            }
        )


class Telemetry:
    def __init__(self, mavlink_receiver, mavlink_sendler, mqtt):
        self.mavlink_receiver = mavlink_receiver
        self.mavlink_sendler = mavlink_sendler
        self.mqtt = mqtt
        self.data = DroneData()
        self.buffer: dict = {
            "mission_received": 0,
            "change_status": 0,
            "mission_completed": 0,
            "mission_cancelled": 0,
        }

    def update(self):
        while True:
            battery = self.get_bat()
            altitude, latitude, longitude = self.get_coor()
            direction = self.get_direction()
            heading, speed = self.get_sp_head()
            status = self.mavlink_sendler.tata
            mission_id = self.mqtt.states.mission_id
            command_id = self.mqtt.states.command_id
            video = True if self.mqtt.states.video == True else False
            modecontrol = True if self.mqtt.states.modecontrol else False
            pause = True if self.mqtt.states.pause else False
            self.data.update(
                altitude=altitude,
                f_coordinates=Coordinates(latitude, longitude),
                battery_level=battery,
                heading=heading,
                speed=speed,
                mission_id=mission_id,
                command_id=command_id,
                flight_direction=direction,
                video=video,
                modecontrol=modecontrol,
                status=status,
                pause=pause
            )
            self.mqtt.publish(f"{self.mqtt.drone_id}_data", self.data.json())
           
            self.mission_status()
            time.sleep(0.7)

    def get_bat(self):
        msg = self.mavlink_receiver.mav.recv_match(type="SYS_STATUS", blocking=True, timeout=2)
        if msg is not None:
            return msg.battery_remaining
        else:
            return None

    def get_coor(self):
        msg = self.mavlink_receiver.mav.recv_match(
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
        msg = self.mavlink_receiver.mav.recv_match(
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
        msg = self.mavlink_receiver.mav.recv_match(type="VFR_HUD", blocking=True, timeout=2)
        if msg is not None:
            heading = msg.heading
            speed = msg.groundspeed
            return [heading, speed]
        else:
            return [None, None]

    def mission_status(self):
        for key, value in self.mqtt.states.mission_status.items():
            if self.buffer.get(key) != value:
                topic = f"{self.mqtt.drone_id}_" + key
                try:
                    self.mqtt.publish(topic, str(value))
                    self.buffer[key] = value
                except Exception as e:
                    print(f"Error sending data via MQTT: {e}")
