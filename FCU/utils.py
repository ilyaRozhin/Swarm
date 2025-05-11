#!/usr/bin/env python3

import json
import math
from enum import Enum

# from Tools.missionExecutor import DroneCommandDto
from typing import List

from MAVProxy.modules.lib import mp_util
from pymavlink.mavutil import location

class ConnectionInfo:
    def __init__(self, protocol: str, host: str, port: int) -> None:
        self.protocol: str = protocol
        self.host: str = host
        self.port: int = port

    def get_str(self) -> str:
        return f"{self.protocol}:{self.host}:{self.port}"
    
    def get(self):
        return self.protocol, self.host, self.port


class Coordinates:
    def __init__(self, latitude=None, longitude=None):
        self.latitude: float = latitude
        self.longitude: float = longitude


class ActiveModes:
    def __init__(self):
        self.get_mission = False
        self.video_record = False


class ModeControl(Enum):
    HAND = 1
    AUTO = 0


class MissionCancel(Enum):
    NOT = 0
    LAND = 1
    RTL = 2


class TopicAppeal(Enum):
    CANCEL_MISSION = 0
    USE_AI_VIDEO = 1
    USE_HAND_MODE = 2
    RAM = 3
    GIMBAL_CONTROL = 4
    JOY_CONTROL = 5
    MISSION = 6
    CHANGE_REALTIME_STREAM = 7
    PAUSE_MISSION = 8
    CRITICAL_DISARM = 9
    GET_CURRENT_DATA = 10


class VideoRecord(Enum):
    OFF = False
    ON = True


class Direction(Enum):
    FORWARD = 0  # 'forward'
    BACK = 1  # 'back'
    LEFT = 2  # 'left'
    RIGHT = 3  # 'right'
    FORWARDLEFT = 4  # 'forward_left'
    FORWARDRIGHT = 5  # 'forward_right'
    BACKLEFT = 6  # 'back_left'
    BACKRIGHT = 7  # 'back_right';


class ObjectData:
    def __init__(self):
        self.id: int
        self.name: str
        self.time: str
        self.lat: float
        self.long: float
        self.speed: float
        self.lat_drone: float
        self.long_drone: float
        self.mission_id: str

    def update(
        self, id, name, time, lat, long, speed, lat_drone, long_drone, mission_id
    ):
        changed = False
        if id != self.id:
            self.id = id
            changed = True
        elif name != self.name:
            self.name = name
            changed = True
        elif time != self.time:
            self.time = time
            changed = True
        elif lat != self.lat:
            self.lat = lat
            changed = True
        elif long != self.long:
            self.long = long
            changed = True
        elif speed != self.speed:
            self.speed = speed
            changed = True
        elif lat_drone != self.lat_drone:
            self.lat_drone = lat_drone
            changed = True
        elif long_drone != self.long_drone:
            self.long_drone = long_drone
            changed = True
        elif mission_id != self.mission_id:
            self.mission_id = mission_id
            changed = True
        return changed

    def json(self):
        return json.dumps(
            {
                "id": self.id,
                "name": self.name,
                "time": self.time,
                "lat": self.lat,
                "long": self.long,
                "speed": self.speed,
                "lat_drone": self.lat_drone,
                "long_drone": self.long_drone,
                "mission_id": self.mission_id,
            }
        )


class GimbalData:
    def __init__(self):
        self.roll: float
        self.yaw: float
        self.pitch: float

    def update(self, roll, yaw, pitch):
        changed = False
        if roll != self.roll:
            self.roll = roll
            changed = True
        elif yaw != self.yaw:
            self.yaw = yaw
            changed = True
        elif pitch != self.pitch:
            self.pitch = pitch
            changed = True
        return changed

    def json(self):
        return json.dumps(
            {
                "roll": self.roll,
                "yaw": self.yaw,
                "pitch": self.pitch,
            }
        )


class ExtendedDroneCommandDto:
    def __init__(self):
        self.mission_id: str
        self.params: List
        self.subMissions = None
        self.startMissionId: str

    def update(self, mission_id, params, subMissions, startMissionId):
        self.mission_id = mission_id
        self.params = params
        self.subMissions = subMissions
        self.startMissionId = startMissionId

    def upload(self, data):
        json_data = json.loads(data)
        self.mission_id = json_data["mission_id"]
        self.params = json_data["params"]
        self.subMissions = json_data["subMissions"]
        self.startMissionId = json_data["startMissionId"]


class RamData:
    def __init__(self):
        self.name: str
        self.id: int
        self.lat: float
        self.long: float
        self.direction: str

    def update(self, name, id, lat, long, direction):
        self.name = name
        self.id = id
        self.lat = lat
        self.long = long
        self.direction = direction

    def upload(self, data):
        json_data = json.loads(data)
        self.name = json_data["name"]
        self.id = json_data["id"]
        self.lat = json_data["lat"]
        self.long = json_data["long"]
        self.direction = json_data["direction"]


class TelemData:
    def __init__(
        self, status=None, mission_id=None, video=None, control_mode=None
    ) -> None:
        self.status = status
        self.mission_id = mission_id
        self.video = video
        self.control_mode = control_mode


class ControlState:
    def __init__(self):
        self.cancel_mission_requested = False
        self.cor_video = 0
        self.mission_id = 0
        self.mode_control = 0
        self.change_status = "waiting"


def calc_global_vel(heading, vx, vy):
    yaw = math.radians(heading)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vx_global = vx * cos_yaw - vy * sin_yaw
    vy_global = vx * sin_yaw + vy * cos_yaw
    return vx_global, vy_global


def progress(text):
    """Utility to print message with current time."""
    # now = datetime.datetime.now()
    # formatted_text = "AT %s: %s" % (now.strftime('%H:%M:%S'), text)
    # print(formatted_text)


def longitude_scale(lat):
    ret = math.cos(lat * (math.radians(1)))
    # print("scale=%f" % ret)
    return ret


def get_distance(loc1, loc2):
    """Get ground distance between two locations."""
    return get_distance_accurate(loc1, loc2)


def get_distance_accurate(loc1, loc2):
    """Get ground distance between two locations."""
    try:
        lon1 = loc1.lng
        lon2 = loc2.lng
    except AttributeError:
        lon1 = loc1.lon
        lon2 = loc2.lon

    return mp_util.gps_distance(loc1.lat, lon1, loc2.lat, lon2)


def get_latlon_attr(loc, attrs):
    """return any found latitude attribute from loc"""
    ret = None
    for attr in attrs:
        if hasattr(loc, attr):
            ret = getattr(loc, attr)
            break
    if ret is None:
        raise ValueError("None of %s in loc(%s)" % (str(attrs), str(loc)))
    return ret


def get_lat_attr(loc):
    """return any found latitude attribute from loc"""
    return get_latlon_attr(loc, ["lat", "latitude"])


def get_lon_attr(loc):
    """return any found latitude attribute from loc"""
    return get_latlon_attr(loc, ["lng", "lon", "longitude"])


def get_distance_int(loc1, loc2):
    """Get ground distance between two locations in the normal "int" form
    - lat/lon multiplied by 1e7"""
    loc1_lat = get_lat_attr(loc1)
    loc2_lat = get_lat_attr(loc2)
    loc1_lon = get_lon_attr(loc1)
    loc2_lon = get_lon_attr(loc2)

    return get_distance_accurate(
        location(loc1_lat * 1e-7, loc1_lon * 1e-7),
        location(loc2_lat * 1e-7, loc2_lon * 1e-7),
    )


def calculate_flight_direction(vx, vy):
    angle = math.degrees(math.atan2(vy, vx))
    if angle < 0:
        angle += 360
    return angle


def direction(speed):
    full_speed = speed
    half_speed = 0.5 * speed
    fourth_speed = 0.25 * speed
    directions = {
        "forward_1": (fourth_speed, 0, 0),
        "back_1": (-fourth_speed, 0, 0),
        "left_1": (0, -fourth_speed, 0),
        "right_1": (0, fourth_speed, 0),
        "forward_left_1": (fourth_speed, -fourth_speed, 0),
        "forward_right_1": (fourth_speed, fourth_speed, 0),
        "back_left_1": (-fourth_speed, -fourth_speed, 0),
        "back_right_1": (-fourth_speed, fourth_speed, 0),
        "up_1": (0, 0, -fourth_speed),
        "down_1": (0, 0, fourth_speed),
        "stop_1": (0, 0, 0),
        "forward_2": (half_speed, 0, 0),
        "back_2": (-half_speed, 0, 0),
        "left_2": (0, -half_speed, 0),
        "right_2": (0, half_speed, 0),
        "forward_left_2": (half_speed, -half_speed, 0),
        "forward_right_2": (half_speed, half_speed, 0),
        "back_left_2": (-half_speed, -half_speed, 0),
        "back_right_2": (-half_speed, half_speed, 0),
        "up_2": (0, 0, -half_speed),
        "down_2": (0, 0, half_speed),
        "stop_2": (0, 0, 0),
        "forward_3": (full_speed, 0, 0),
        "back_3": (-full_speed, 0, 0),
        "left_3": (0, -full_speed, 0),
        "right_3": (0, full_speed, 0),
        "forward_left_3": (full_speed, -full_speed, 0),
        "forward_right_3": (full_speed, full_speed, 0),
        "back_left_3": (-full_speed, -full_speed, 0),
        "back_right_3": (-full_speed, full_speed, 0),
        "up_3": (0, 0, -full_speed),
        "down_3": (0, 0, full_speed),
        "stop_3": (0, 0, 0),
    }
    return directions


def direction_RC(command):
    directions = {
        "forward_1": (1500, 1350, 1500),
        "back_1": (1500, 1650, 1500),
        "left_1": (1350, 1500, 1500),
        "right_1": (1650, 1500, 1500),
        "forward_left_1": (1350, 1350, 1500),
        "forward_right_1": (1650, 1350, 1500),
        "back_left_1": (1350, 1650, 1500),
        "back_right_1": (1650, 1650, 1500),
        "up_1": (1500, 1500, 1650),
        "down_1": (1500, 1500, 1350),
        "stop_1": (1500, 1500, 1500),
        "forward_2": (1500, 1200, 1500),
        "back_2": (1500, 1800, 1500),
        "left_2": (1200, 1500, 1500),
        "right_2": (1500, 1800, 1500),
        "forward_left_2": (1200, 1200, 1500),
        "forward_right_2": (1800, 1200, 1500),
        "back_left_2": (1200, 1800, 1500),
        "back_right_2": (1800, 1800, 1500),
        "up_2": (1500, 1500, 1800),
        "down_2": (1500, 1500, 1200),
        "stop_2": (1500, 1500, 1500),
        "forward_3": (1500, 1000, 1500),
        "back_3": (1500, 2000, 1500),
        "left_3": (1000, 1500, 1500),
        "right_3": (2000, 1500, 1500),
        "forward_left_3": (1000, 1000, 1500),
        "forward_right_3": (2000, 1000, 1500),
        "back_left_3": (1000, 2000, 1500),
        "back_right_3": (2000, 2000, 1500),
        "up_3": (1500, 1500, 2000),
        "down_3": (1500, 1500, 1000),
        "stop_3": (1500, 1500, 1500),
        "center_0": (1500, 1500, 1500),#тут 
        "center": (1500, 1500, 1500), # тут 
    }
    return directions[command]
