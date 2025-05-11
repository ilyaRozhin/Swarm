#!/usr/bin/env python3

import os
import time
import math
import sys
import threading
from time import sleep
from exceptions import *
from FCU.utils import progress, get_distance_int, get_distance, MissionCancel
from pymavlink import mavutil, mavwp
from pymavlink.dialects.v20 import ardupilotmega
from pymavlink.rotmat import Vector3
import logging

# Ensure the logs directory exists
logs_dir = 'logs'
if not os.path.exists(logs_dir):
    os.makedirs(logs_dir)

logging.basicConfig(
    filename='logs/vehicleConnection.log',
    filemode='w',
    format='%(asctime)s - %(threadName)s - %(levelname)s - %(message)s',
    level=logging.DEBUG
)

class MAVLinkClient(object):
    def __init__(self, connectionInfo, streamrate = 5, system_id = 1, sourceSystem = 255) -> None:
        super(MAVLinkClient, self).__init__()
        os.environ['MAVLINK20'] = '1'
        self.connectionInfo = connectionInfo
        self.mav = None
        self.streamrate: int = streamrate
        self.target_system: int = system_id
        self.in_drain_mav = False
        self.tata = "waiting"
        self.heartbeat_interval_ms = 1000
        self.target_component = 1
        self.wploader: mavwp.MAVWPLoader = mavwp.MAVWPLoader()
        self.total_waiting_to_arm_time = 0
        self.waiting_to_arm_count = 0
        self.count_pov = 0 
        self.direction = None
        self.wp_received = {}
        self.sourcesystem = sourceSystem
        self.last_heartbeat_time_ms = None
        self.last_heartbeat_time_wc_s = 0
        self.wp_requested = {}
        self.wp_expected_count = 0
        self.mis_can = False
        self.hand_control = False
        self.pause = False
        self.critical = False
        self.cancel_type = MissionCancel.NOT
        self.lock = threading.Lock()  # Add a lock for thread-safe operations

    def start_connection(self):
        self.mav = mavutil.mavlink_connection(
            self.connectionInfo.get_str(),
            retries=1000,
            robust_parsing=True,
            source_system=self.sourcesystem,
            source_component=self.sourcesystem,
            autoreconnect=True,
            dialect="ardupilotmega",
        )
        self.set_streamrate(self.streamrate)
        self.mav.message_hooks.append(self.message_hook)
        self.mav.idle_hooks.append(self.idle_hook)
        connection_status = "Success" if self.mav is not None else "Fail"
        self.set_message_rate_hz(ardupilotmega.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT, 10)
        logging.debug(f"MAVLink Client Connected (Connection Status: {connection_status})")

    def idle_hook(self, mav):
        """Called when waiting for a mavlink message."""
        if self.in_drain_mav:
            return

    def message_hook(self, mav, msg):
        """Called as each mavlink msg is received.
        Display STATUSTEXT messages.
        Send the heartbeats if needed."""
        if msg.get_type() == 'STATUSTEXT':
            logging.debug("AP: %s" % msg.text)
        self.idle_hook(mav)
        self.do_heartbeats()

    def do_heartbeats(self, force=False):
        if self.heartbeat_interval_ms is None and not force:
            return
        x = self.mav.messages.get("SYSTEM_TIME", None)
        now_wc = time.time()
        if (force or
                x is None or
                self.last_heartbeat_time_ms is None or
                self.last_heartbeat_time_ms < x.time_boot_ms or
                x.time_boot_ms - self.last_heartbeat_time_ms > self.heartbeat_interval_ms or
                now_wc - self.last_heartbeat_time_wc_s > 1):
            # self.logging.debug("Sending heartbeat")
            if x is not None:
                self.last_heartbeat_time_ms = x.time_boot_ms
            self.last_heartbeat_time_wc_s = now_wc
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def set_streamrate(self, streamrate, timeout=20):
        """set MAV_DATA_STREAM_ALL; timeout is wallclock time"""
        tstart = time.time()
        while True:
            if time.time() - tstart > timeout:
                raise TimeoutException("Failed to set streamrate")
            self.mav.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                streamrate,
                1)
            m = self.mav.recv_match(type='SYSTEM_TIME',
                                    blocking=True,
                                    timeout=1)
            if m is not None:
                break

    def drain_mav_unparsed(self, mav=None, quiet=True, freshen_sim_time=False):
        if mav is None:
            mav = self.mav
        self.in_drain_mav = True
        count = 0
        tstart = time.time()
        while True:
            this = self.mav.recv(1000000)
            if len(this) == 0:
                break
            count += len(this)
        if quiet:
            return
        tdelta = time.time() - tstart
        if tdelta == 0:
            rate = "instantly"
        else:
            rate = "%f/s" % (count / float(tdelta),)
        logging.debug("Drained %u bytes from mav (%s).  These were unparsed." % (count, rate))
        self.in_drain_mav = False
        if freshen_sim_time:
            time.time()

    def drain_mav(self, mav=None, unparsed=False, quiet=True):
        if unparsed:
            return self.drain_mav_unparsed(quiet=quiet, mav=mav)
        if mav is None:
            mav = self.mav
        count = 0
        tstart = time.time()
        while mav.recv_match(blocking=False) is not None:
            count += 1
        if quiet:
            return
        tdelta = time.time() - tstart
        if tdelta == 0:
            rate = "instantly"
        else:
            rate = "%f/s" % (count / float(tdelta),)
        logging.debug("Drained %u messages from mav (%s)" % (count, rate))

    def wait_heartbeat(self, drain_mav=True, quiet=False, *args, **x):
        """as opposed to mav.wait_heartbeat, raises an exception on timeout.
        Also, ignores heartbeats not from our target system"""
        if drain_mav:
            self.drain_mav(quiet=quiet)
        orig_timeout = x.get("timeout", 10)
        x["timeout"] = 1
        tstart = time.time()
        while True:
            if time.time() - tstart > orig_timeout:
                # if not self.sitl_is_running():
                #     self.logging.debug("SITL is not running")
                raise TimeoutException("Did not receive heartbeat")
            m = self.mav.wait_heartbeat(*args, **x)
            if m is None:
                continue
            if m.get_srcSystem() == self.target_system:
                break

    def cancel_handler(self):
        if self.cancel_type == MissionCancel.RTL:
            self.do_RTL(distance_max=1000)
            self.change_mode("LAND")
            self.wait_landed_and_disarmed()
        if self.cancel_type == MissionCancel.LAND:
            self.change_mode("LAND")
            self.wait_landed_and_disarmed()
        self.cancel_type = MissionCancel.NOT
        self.tata = 'return home'
        self.mis_can = False

    def send_velocity_command(self, vx, vy, vz):
        self.mav.mav.set_position_target_local_ned_send(
            0,
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,  
            0b0000111111000111,  
            0, 0, 0, 
            vx, vy, vz,  
            0, 0, 0, 
            0, 0)
        
    def send_set_parameter_direct(self, name, value):
        self.mav.mav.param_set_send(self.target_system,
                                    1,
                                    name.encode('ascii'),
                                    value,
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def install_message_hook(self, hook):
        self.mav.message_hooks.append(hook)

    def remove_message_hook(self, hook):
        if self.mav is None:
            return
        oldlen = len(self.mav.message_hooks)
        self.mav.message_hooks = list(filter(lambda x: x != hook,
                                             self.mav.message_hooks))
        if len(self.mav.message_hooks) == oldlen:
            raise NotAchievedException("Failed to remove hook")
        
    def send_cmd(self, command, p1, p2, p3, p4, p5, p6, p7, 
                 target_sysid=None, target_compid=None):
        """Send a MAVLink command long."""
        if target_sysid is None:
            target_sysid = self.target_system
        if target_compid is None:
            target_compid = 1
        try:
            command_name = mavutil.mavlink.enums["MAV_CMD"][command].name
        except KeyError as e:
            command_name = "UNKNOWN=%u" % command
        logging.debug("Sending COMMAND_LONG to (%u,%u) (%s) (p1=%f p2=%f p3=%f p4=%f p5=%f p6=%f  p7=%f)" %
                 (target_sysid, target_compid, command_name, p1, p2, p3, p4, p5, p6, p7))
        self.mav.mav.command_long_send(target_sysid, target_compid, command, 1,
                                       p1, p2, p3, p4, p5, p6, p7)
        
    def wait_ekf_flags(self, required_value, error_bits, timeout=30):
        logging.debug("Waiting for EKF value %u" % required_value)
        self.drain_mav_unparsed()
        last_print_time = 0
        tstart = time.time()
        while timeout is None or time.time() < tstart + timeout:
            m = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=timeout)
            if m is None:
                continue
            current = m.flags
            errors = current & error_bits
            everything_ok = (errors == 0 and
                             current & required_value == required_value)
            if everything_ok or time.time() - last_print_time > 1:
                logging.debug("Wait EKF.flags: required:%u current:%u errors=%u" %
                              (required_value, current, errors))
                last_print_time = time.time()
            if everything_ok:
                logging.debug("EKF Flags OK")
                return True
        raise TimeoutException("Failed to get EKF.flags=%u" %
                               required_value)
    
    def missing_wps_to_request(self):
        ret = []
        tnow = time.time()
        next_seq = self.wploader.count()
        for i in range(2 * self.wp_expected_count):
            seq = next_seq + i
            if seq + 1 > self.wp_expected_count:
                continue
            if seq in self.wp_requested and tnow - self.wp_requested[seq] < 2:
                continue
            ret.append(seq)
        return ret

    def wp_from_mission_item_int(self, wp):
        '''convert a MISSION_ITEM_INT to a MISSION_ITEM'''
        wp2 = mavutil.mavlink.MAVLink_mission_item_message(wp.target_system, wp.target_component, wp.seq, 
                                                           wp.frame,wp.command,wp.current, wp.autocontinue, wp.param1, wp.param2, wp.param3,
                                                           wp.param4,wp.x * 1.0e-7,wp.y * 1.0e-7, wp.z)
        # preserve srcSystem as that is used for naming waypoint file
        wp2._header.srcSystem = wp.get_srcSystem()
        wp2._header.srcComponent = wp.get_srcComponent()
        return wp2

    def get_all_waypoints(self, timeout=30):
        logging.debug("Requesting Mission item count")
        self.mav.waypoint_request_list_send()
        tstart = time.time()
        while True:
            now = time.time()
            if now - tstart > timeout:
                logging.debug("Failed to get Mission total item")
                return
            msg = self.mav.recv_match(type=['WAYPOINT_COUNT', 'MISSION_COUNT'], blocking=True, timeout=3)
            if msg is None:
                continue
            self.wp_expected_count = msg.count
            logging.debug("Got %s waypoints to get" % msg.count)
            self.wploader.clear()
            break
        for seq in self.missing_wps_to_request():
            self.wp_requested[seq] = time.time()
            logging.debug("Requesting waypoint %d" % seq)
            self.mav.mav.mission_request_int_send(self.target_system, self.target_component, seq)
            tstart = time.time()
            while True:
                now = time.time()
                if now - tstart > timeout:
                    logging.debug("Failed to get Waypoint %d" % seq)
                    return
                msg = self.mav.recv_match(type=['WAYPOINT', 'MISSION_ITEM', 'MISSION_ITEM_INT'], blocking=True,
                                          timeout=3)
                if msg is None:
                    continue
                if msg.get_type() == 'MISSION_ITEM_INT':
                    if getattr(msg, 'mission_type', 0) != 0:
                        # this is not a mission item, likely fence
                        return
                    # our internal structure assumes MISSION_ITEM'''
                    msg = self.wp_from_mission_item_int(msg)
                if msg.seq < self.wploader.count():
                    #print("DUPLICATE %u" % m.seq)
                    return
                if msg.seq + 1 > self.wp_expected_count:
                    logging.debug("Unexpected waypoint number %u - expected %u" % (msg.seq, self.wploader.count()))
                self.wp_received[msg.seq] = msg

                next_seq = self.wploader.count()
                while next_seq in self.wp_received:
                    m = self.wp_received.pop(next_seq)
                    self.wploader.add(m)
                    next_seq += 1
                if self.wploader.count() != self.wp_expected_count:
                    logging.debug("m.seq=%u expected_count=%u" % (msg.seq, self.wp_expected_count))
                    break
                if self.wploader.count() == self.wp_expected_count:
                    logging.debug("Got all Waypoints")
                    break

        for i in range(self.wploader.count()):
            w = self.wploader.wp(i)
            #print("%u %u %.10f %.10f %f p1=%.1f p2=%.1f p3=%.1f p4=%.1f cur=%u auto=%u" % (w.command, w.frame, w.x, w.y, w.z,
            #                                                                               w.param1, w.param2, w.param3, w.param4,
            #                                                                               w.current, w.autocontinue))
        self.wp_requested = {}
        self.wp_received = {}
        return self.wploader.count()
    
    def send_all_waypoints(self, timeout=60):
        """send all waypoints to vehicle"""
        self.mav.waypoint_clear_all_send()
        logging.debug("Sending %d waypoints" % self.wploader.count())
        if self.wploader.count() == 0:
            return
        self.mav.waypoint_count_send(self.wploader.count())
        tstart = time.time()
        while True:
            now = time.time()
            if now - tstart > timeout:
                logging.debug("Failed to send Mission")
                return
            msg = self.mav.recv_match(type=["MISSION_REQUEST", "WAYPOINT_REQUEST"], blocking=True, timeout=3)
            if msg is None:
                continue
            if msg.seq >= self.wploader.count():
                logging.debug("Request for bad waypoint %u (max %u)" % (msg.seq, self.wploader.count()))
                return
            wp = self.wploader.wp(msg.seq)
            wp_send = self.wp_to_mission_item_int(wp)

            self.mav.mav.send(wp_send)
            logging.debug("Sent waypoint %u : %s" % (msg.seq, self.wploader.wp(msg.seq)))
            if msg.seq == self.wploader.count() - 1:
                logging.debug("Sent all %u waypoints" % self.wploader.count())
                return

    def wp_to_mission_item_int(self, wp):
        """convert a MISSION_ITEM to a MISSION_ITEM_INT. We always send as MISSION_ITEM_INT
           to give cm level accuracy"""
        if wp.get_type() == 'MISSION_ITEM_INT':
            return wp
        wp_int = mavutil.mavlink.MAVLink_mission_item_int_message(wp.target_system, wp.target_component, wp.seq, wp.frame, wp.command,
                                                                  wp.current, wp.autocontinue, wp.param1, wp.param2, wp.param3, wp.param4,
                                                                  int(wp.x * 1.0e7), int(wp.y * 1.0e7), wp.z)
        return wp_int
    
    def init_wp(self):
        last_home = self.home_position_as_mav_location()
        self.wploader.clear()
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_system
        self.add_waypoint(last_home.lat, last_home.lng, last_home.alt)
        

    def add_waypoint(self, lat, lon, alt):
        self.wploader.add_latlonalt(lat, lon, alt, terrain_alt=False)

    def add_wp_takeoff(self, lat, lon, alt):
        p = mavutil.mavlink.MAVLink_mission_item_message(self.target_system,
                                                         self.target_component,
                                                         0,
                                                         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                                         0, 0, 0, 0, 0, 0,
                                                         lat, lon, alt)
        self.wploader.insert(1, p)

    def add_wp_rtl(self):
        p = mavutil.mavlink.MAVLink_mission_item_message(self.target_system,
                                                         self.target_component,
                                                         0,
                                                         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                         mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                                         0, 0, 0, 0, 0, 0,
                                                         0, 0, 0)
        self.wploader.add(p)

    def home_position_as_mav_location(self):
        m = self.poll_home_position()
        return mavutil.location(m.latitude * 1.0e-7, m.longitude * 1.0e-7, m.altitude * 1.0e-3, 0)
    
    def poll_home_position(self, quiet=True, timeout=30):
        old = self.mav.messages.get("HOME_POSITION", None)
        tstart = time.time()
        while True:
            if time.time() - tstart > timeout:
                raise NotAchievedException("Failed to poll home position")
            if not quiet:
                logging.debug("Sending MAV_CMD_GET_HOME_POSITION")
            try:
                self.run_cmd(mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,0, 0, 0, 0, 0, 0, 0, quiet=quiet)
            except ValueError as e:
                continue
            m = self.mav.messages.get("HOME_POSITION", None)
            if m is None:
                continue
            if old is None:
                break
            if m._timestamp != old._timestamp:
                break
        logging.debug("Polled home position (%s)" % str(m))
        return m

    def run_cmd(self, command, p1, p2, p3, p4, p5, p6, p7,
                want_result = mavutil.mavlink.MAV_RESULT_ACCEPTED,
                target_sysid: int = None,
                target_compid: int = None,
                timeout: int = 10,
                quiet: bool = False):
        self.drain_mav_unparsed()
        self.send_cmd(command,p1, p2, p3, p4, p5, p6, p7,
                              target_sysid=target_sysid,
                              target_compid=target_compid)
        self.run_cmd_get_ack(command, want_result, timeout, quiet=quiet)

    def run_cmd_get_ack(self, command, want_result, timeout, quiet=False):
        # note that the caller should ensure that this cached
        # timestamp is reasonably up-to-date!
        tstart = time.time()
        while True:
            delta_time = time.time() - tstart
            if delta_time > timeout:
                raise TimeoutException("Did not get good COMMAND_ACK within %fs" % timeout)
            m = self.mav.recv_match(type='COMMAND_ACK', blocking=False, timeout=0.1)
            if m is None:
                continue
            if not quiet:
                logging.debug("ACK received: %s (%fs)" % (str(m), delta_time))
            if m.command == command:
                if m.result != want_result:
                    raise ValueError("Expected %s got %s" % (
                        mavutil.mavlink.enums["MAV_RESULT"][want_result].name,
                        mavutil.mavlink.enums["MAV_RESULT"][m.result].name))
                break

    def set_message_rate_hz(self, id, rate_hz):
        """set a message rate in Hz; 0 for original, -1 to disable"""
        if type(id) == str:
            id = eval("mavutil.mavlink.MAVLINK_MSG_ID_%s" % id)
        if rate_hz == 0 or rate_hz == -1:
            set_interval = rate_hz
        else:
            set_interval = 1 / float(rate_hz) * 1000000.0
        self.run_cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, id, set_interval, 0, 0, 0, 0, 0)

    def send_set_parameter(self, name, value, verbose=False):
        if verbose:
            logging.debug("Send set param for (%s) (%f)" % (name, value))
        return self.send_set_parameter_direct(name, value)

    def run_cmd_do_set_mode(self, mode, timeout = 30, want_result = mavutil.mavlink.MAV_RESULT_ACCEPTED):
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = self.get_mode_from_mode_mapping(mode)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, base_mode, custom_mode,
                     0, 0, 0, 0, 0, want_result = want_result, timeout = timeout)

    def do_set_mode_via_command_long(self, mode, timeout=30):
        """Set mode with a command long message."""
        tstart = time.time()
        want_custom_mode = self.get_mode_from_mode_mapping(mode)
        while True:
            remaining = timeout - (time.time() - tstart)
            if remaining <= 0:
                raise TimeoutException("Failed to change mode")
            self.run_cmd_do_set_mode(mode, timeout=10)
            m = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=15)
            if m is None:
                raise ErrorException("Heartbeat not received")
            logging.debug("Got mode=%u want=%u" % (m.custom_mode, want_custom_mode))
            if m.custom_mode == want_custom_mode:
                return

    def change_mode(self, mode, timeout=60):
        """change vehicle flightmode"""
        #self.wait_heartbeat()
        #logging.debug("Changing mode to %s" % mode)
        self.do_set_mode_via_command_long(mode)

    def mode_is(self, mode, cached=False, drain_mav=True):
        if not cached:
            self.wait_heartbeat(drain_mav = drain_mav)
        try:
            return self.get_mode_from_mode_mapping(self.mav.flightmode) == self.get_mode_from_mode_mapping(mode)
        except Exception as e:
            pass
        # assume this is a number....
        return self.mav.messages['HEARTBEAT'].custom_mode == mode

    def wait_mode(self, mode, timeout=60):
        """Wait for mode to change."""
        logging.debug("Waiting for mode %s" % mode)
        tstart = time.time()
        while not self.mode_is(mode, drain_mav=False):
            custom_num = self.mav.messages['HEARTBEAT'].custom_mode
            logging.debug("mav.flightmode=%s Want=%s custom=%u" % (
                self.mav.flightmode, mode, custom_num))
            if (timeout is not None and
                    time.time() > tstart + timeout):
                raise WaitModeTimeout("Did not change mode")
        logging.debug("Got mode %s" % mode)

    def get_mode_from_mode_mapping(self, mode):
        """Validate and return the mode number from a string or int."""
        mode_map = self.mav.mode_mapping()
        if mode_map is None:
            mav_type = self.mav.messages['HEARTBEAT'].type
            mav_autopilot = self.mav.messages['HEARTBEAT'].autopilot
            raise ErrorException("No mode map for (mav_type=%s mav_autopilot=%s)" % (mav_type, mav_autopilot))
        if isinstance(mode, str):
            if mode in mode_map:
                return mode_map.get(mode)
        if mode in mode_map.values():
            return mode
        logging.debug("Available modes '%s'" % mode_map)
        raise ErrorException("Unknown mode '%s'" % mode)

    def distance_to_home(self, use_cached_home=False):
        m = self.mav.messages.get("HOME_POSITION", None)
        if use_cached_home is False or m is None:
            m = self.poll_home_position(quiet=True)
        here = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                   blocking=True)
        return get_distance_int(m, here)


    def get_lat_lon(self):
        point = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not point is None:
            return (point.lat / 1e7, point.lon / 1e7)
        else:
            return (None, None)

    def wait_altitude(self, altitude_min, altitude_max, relative=False, timeout=30, **kwargs):
        """Wait for a given altitude range."""
        assert altitude_min <= altitude_max, "Minimum altitude should be less than maximum altitude."

        def get_altitude(alt_relative=False, timeout2=30):
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout2)
            if msg:
                if alt_relative:
                    return msg.relative_alt / 1000.0  # mm -> m
                else:
                    return msg.alt / 1000.0  # mm -> m
            raise MsgRcvTimeoutException("Failed to get Global Position")

        def validator(value2, target2=None):
            if altitude_min <= value2 <= altitude_max:
                return True
            else:
                return False

        self.wait_and_maintain(value_name="Altitude", target=altitude_min, current_value_getter=lambda: get_altitude(relative, timeout),
                               accuracy=(altitude_max - altitude_min), validator=lambda value2, target2: validator(value2, target2), timeout=timeout, **kwargs)

    def wait_location(self, loc, accuracy=5.0, timeout=30, target_altitude=None, height_accuracy=-1, **kwargs):
        """Wait for arrival at a location."""
        def get_distance_to_loc():
            return get_distance(self.mav.location(), loc)

        def validator(value2, empty=None):
            if value2 <= accuracy:
                if target_altitude is not None:
                    height_delta = math.fabs(self.mav.location().alt - target_altitude)
                    if height_accuracy != -1 and height_delta > height_accuracy:
                        return False
                return True
            else:
                return False
        debug_text = "Distance to Location (%.4f, %.4f) " % (loc.lat, loc.lng)
        if target_altitude is not None:
            debug_text += ",at altitude %.1f height_accuracy=%.1f, d" % (target_altitude, height_accuracy)
        self.wait_and_maintain(value_name=debug_text, target=0, current_value_getter=lambda: get_distance_to_loc(),
                               accuracy=accuracy, validator=lambda value2, target2: validator(value2, None), timeout=timeout, **kwargs)

    def wait_distance_to_home(self, distance_min, distance_max, timeout=10, use_cached_home=True, **kwargs):
        """Wait for distance to home to be within specified bounds."""
        assert distance_min <= distance_max, "Distance min should be less than distance max."

        def get_distance():
            return self.distance_to_home(use_cached_home)

        def validator(value2, target2=None):
            return distance_min <= value2 <= distance_max

        self.wait_and_maintain(value_name="Distance to home", target=distance_min,
                               current_value_getter=lambda: get_distance(),
                               validator=lambda value2, target2: validator(value2, target2),
                               accuracy=(distance_max - distance_min), timeout=timeout, **kwargs)

    def wait_and_maintain(self, value_name, target, current_value_getter, validator=None, accuracy=2.0, timeout=30, **kwargs):
        tstart = time.time()
        achieving_duration_start = None
        if type(target) is Vector3:
            sum_of_achieved_values = Vector3()
            last_value = Vector3()
        else:
            sum_of_achieved_values = 0.0
            last_value = 0.0
        count_of_achieved_values = 0
        called_function = kwargs.get("called_function", None)
        minimum_duration = kwargs.get("minimum_duration", 0)
        if type(target) is Vector3:
            logging.debug("Waiting for %s=(%s) with accuracy %.02f" % (value_name, str(target), accuracy))
        else:
            logging.debug("Waiting for %s=%.02f with accuracy %.02f" % (value_name, target, accuracy))
        last_print_time = 0
        while time.time() < tstart + timeout: 
            if self.mis_can:
                return True
            if self.hand_control:
                # logging.info("Skip wait_and_maintain. Using hand control")
                continue
            if self.pause:
                continue
            last_value = current_value_getter()
            if called_function is not None:
                called_function(last_value, target)
            if time.time() - last_print_time > 1:
                if type(target) is Vector3:
                    logging.debug("%s=(%s) (want (%s) +- %f)" %
                                  (value_name, str(last_value), str(target), accuracy))
                else:
                    logging.debug("%s=%0.2f (want %f +- %f)" %
                                  (value_name, last_value, target, accuracy))
                last_print_time = time.time()
            if validator is not None:
                is_value_valid = validator(last_value, target)
            else:
                is_value_valid = math.fabs(last_value - target) <= accuracy
            if is_value_valid:
                sum_of_achieved_values += last_value
                count_of_achieved_values += 1.0
                if achieving_duration_start is None:
                    achieving_duration_start = time.time()
                if time.time() - achieving_duration_start >= minimum_duration:
                    if type(target) is Vector3:
                        logging.debug("Attained %s=%s" % (
                            value_name, str(sum_of_achieved_values * (1.0 / count_of_achieved_values))))
                    else:
                        logging.debug(
                            "Attained %s=%f" % (value_name, sum_of_achieved_values / count_of_achieved_values))
                    return True
            else:
                achieving_duration_start = None
                if type(target) is Vector3:
                    sum_of_achieved_values.zero()
                else:
                    sum_of_achieved_values = 0.0
                count_of_achieved_values = 0
        raise TimeoutException("Failed to attain %s want %s, reached %s" % (value_name, str(target), str(
            sum_of_achieved_values * (1.0 / count_of_achieved_values)) if count_of_achieved_values != 0 else str(
            last_value)))

    def wait_for_alt(self, alt_min=30, timeout=30, max_err=0.1):
        """Wait for minimum altitude to be reached."""
        self.wait_altitude(alt_min - 1, (alt_min + max_err), relative=True, timeout=timeout)

    def sensor_has_state(self, sensor, present=True, enabled=True, healthy=True, do_assert=False, verbose=False):
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if m is None:
            raise TimeoutException("Did not receive SYS_STATUS")
        if verbose:
            logging.debug("Status: %s" % str(mavutil.dump_message_verbose(sys.stdout, m)))
        reported_present = m.onboard_control_sensors_present & sensor
        reported_enabled = m.onboard_control_sensors_enabled & sensor
        reported_healthy = m.onboard_control_sensors_health & sensor
        if present:
            if not reported_present:
                if do_assert:
                    raise NotAchievedException("Sensor not present")
                return False
        else:
            if reported_present:
                if do_assert:
                    raise NotAchievedException("Sensor present when it shouldn't be")
                return False

        if enabled:
            if not reported_enabled:
                if do_assert:
                    raise NotAchievedException("Sensor not enabled")
                return False
        else:
            if reported_enabled:
                if do_assert:
                    raise NotAchievedException("Sensor enabled when it shouldn't be")
                return False

        if healthy:
            if not reported_healthy:
                if do_assert:
                    raise NotAchievedException("Sensor not healthy")
                return False
        else:
            if reported_healthy:
                if do_assert:
                    raise NotAchievedException("Sensor healthy when it shouldn't be")
                return False
        return True
    
    def user_takeoff(self, alt_min=5):
        """takeoff using mavlink takeoff command"""
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0, 0, 0, 0, 0, 0, alt_min)
        logging.debug("Ran command")
        self.wait_for_alt(alt_min)

    def land_and_disarm(self, timeout=60):
        """Land the quad."""
        logging.debug("STARTING LANDING")
        self.change_mode("LAND")
        self.wait_landed_and_disarmed(timeout=timeout)

    def wait_landed_and_disarmed(self, min_alt=1, timeout=250):
        """Wait to be landed and disarmed"""
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        alt = m.relative_alt / 1000.0
        if alt > min_alt:
            self.wait_for_alt(min_alt, timeout=timeout)
        self.wait_disarmed()
        self.tata = "waiting"
        self.mis_can = False

    def do_RTL(self, distance_min=None, check_alt=True, distance_max=10, timeout=250):
        """Enter RTL mode and wait for the vehicle to disarm at Home."""
        self.change_mode("RTL")
        self.tata = "return home"
        self.wait_rtl_complete(check_alt=check_alt, distance_max=distance_max, timeout=timeout)
        
    def wait_rtl_complete(self, check_alt=True, distance_max=10, timeout=250):
        """Wait for RTL to reach home and disarm"""
        logging.debug("Waiting RTL to reach Home and disarm")
        tstart = time.time()
        while time.time() < tstart + timeout:
            if self.mis_can:
                if self.cancel_type == MissionCancel.LAND:
                    self.change_mode("LAND")
                    self.wait_landed_and_disarmed(min_alt=10)
                    return
            if self.pause:
                continue
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt = m.relative_alt / 1000.0
            home_distance = self.distance_to_home(use_cached_home=True)
            home = ""
            alt_valid = alt <= 1
            distance_valid = home_distance < distance_max
            if check_alt:
                if alt_valid and distance_valid:
                    home = "HOME"
            else:
                if distance_valid:
                    home = "HOME"
            logging.debug("Alt: %.02f  HomeDist: %.02f %s" % (alt, home_distance, home))
            if not self.armed():
                if home == "":
                    raise Exception("Did not get home")
                return
        raise Exception("Did not get home and disarm")

    def armed(self):
        """Return true if vehicle is armed and safetyoff"""
        return self.mav.motors_armed()
    
    def get_tata(self):
        return self.tata

    def arm_vehicle(self, timeout=20):
        self.tata = 'takeoff'
        """Arm vehicle with mavlink arm message."""
        #logging.debug("Arm motors with MAVLink cmd")
        #self.drain_mav()
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0, timeout=timeout)
        #try:
        #    self.wait_armed()
        #except TimeoutException:
        #    raise TimeoutException("Failed to ARM with mavlink")
        return True
    
    def wait_ekf_happy(self, timeout=30, require_absolute=True):
        """Wait for EKF to be happy"""

        # all of these must be set for arming to happen:
        required_value = (mavutil.mavlink.EKF_ATTITUDE |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_HORIZ |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_VERT |
                          mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL |
                          mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_REL)
        # none of these bits must be set for arming to happen:
        error_bits = (mavutil.mavlink.ESTIMATOR_CONST_POS_MODE |
                      mavutil.mavlink.ESTIMATOR_ACCEL_ERROR)
        if require_absolute:
            required_value |= (mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS |
                               mavutil.mavlink.ESTIMATOR_POS_VERT_ABS |
                               mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_ABS)
            error_bits |= mavutil.mavlink.ESTIMATOR_GPS_GLITCH
        self.wait_ekf_flags(required_value, error_bits, timeout=timeout)

    def wait_gps_sys_status_not_present_or_enabled_and_healthy(self, timeout=30):
        logging.debug("Waiting for GPS health")
        tstart = time.time()
        while True:
            now = time.time()
            if now - tstart > timeout:
                raise TimeoutException("GPS status bits did not become good")
            m = self.mav.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
            if m is None:
                continue
            if (not (m.onboard_control_sensors_present & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                logging.debug("GPS not present")
                if now > 20:
                    return
                continue
            if (not (m.onboard_control_sensors_enabled & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                logging.debug("GPS not enabled")
                continue
            if (not (m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                logging.debug("GPS not healthy")
                continue
            logging.debug("GPS healthy")
            return

    def wait_ready_to_arm(self, timeout=120, require_absolute=True, check_prearm_bit=True):
        # wait for EKF checks to pass
        logging.debug("Waiting for ready to arm")
        start = time.time()
        self.wait_ekf_happy(timeout=timeout, require_absolute=require_absolute)
        if require_absolute:
            self.wait_gps_sys_status_not_present_or_enabled_and_healthy()
        armable_time = time.time() - start
        if require_absolute:
            m = self.poll_home_position()
            if m is None:
                raise NotAchievedException("Did not receive a home position")
        if check_prearm_bit:
            self.wait_prearm_sys_status_healthy(timeout=timeout)
        logging.debug("Took %u seconds to become armable" % armable_time)
        self.total_waiting_to_arm_time += armable_time
        self.waiting_to_arm_count += 1

    def wait_prearm_sys_status_healthy(self, timeout=60):
        tstart = time.time()
        while True:
            t2 = time.time()
            if t2 - tstart > timeout:
                logging.debug("Prearm bit never went true.  Attempting arm to elicit reason from autopilot")
                self.arm_vehicle()
                raise TimeoutException("Prearm bit never went true")
            if self.sensor_has_state(mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK, True, True, True):
                break

    def wait_armed(self, timeout=20):
        tstart = time.time()
        while time.time() - tstart < timeout:
            self.wait_heartbeat()
            if self.mav.motors_armed():
                logging.debug("Motors ARMED")
                return
        raise TimeoutException("Did not become armed")

    def disarm_vehicle(self, timeout=60, force=False):
        """Disarm vehicle with mavlink disarm message."""
        logging.debug("Disarm motors with MAVLink cmd")
        self.drain_mav_unparsed()
        p2 = 0
        if force:
            p2 = 21196  # magic force disarm value
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0, p2, 0, 0, 0, 0, 0, timeout)
        return self.wait_disarmed()

    def wait_disarmed_default_wait_time(self):
        return 100


    def send_start_RC(self):
        self.default_rc = [1500, 1500, 1000, 1500, 1800, 1000, 1000, 1800, 0]
        rc_val = self.default_rc
        
        self.mav.mav.rc_channels_override_send(
            self.target_system, self.target_component, *rc_val
        )
        logging.debug("Start RC command has been sent")


    def wait_disarmed(self, timeout=None, tstart=None):
        if timeout is None:
            timeout = self.wait_disarmed_default_wait_time()
        logging.debug("Waiting for DISARM")
        if tstart is None:
            tstart = time.time()
        last_print_time = 0
        while True:
            now = time.time()
            delta = now - tstart
            if delta > timeout:
                raise TimeoutException("Failed to DISARM within %fs" %
                                       (timeout,))
            if now - last_print_time > 1:
                logging.debug("Waiting for disarm (%.2fs so far of allowed %.2f)" % (delta, timeout))
                last_print_time = now
            self.wait_heartbeat(quiet=True)
            #            self.logging.debug("Got heartbeat")
            if not self.mav.motors_armed():
                logging.debug("DISARMED after %.2f seconds (allowed=%.2f)" %
                              (delta, timeout))
                return True

    def get_current_target(self, timeout=10):
        """Get and print POSITION_TARGET_GLOBAL_INT msg send by the drone.
           those message are always in MAV_FRAME_GLOBAL_INT frame."""
        tstart = time.time()
        while True:
            if time.time() - tstart > timeout:
                raise TimeoutException("Failed to set streamrate")
            msg = self.mav.recv_match(type='POSITION_TARGET_GLOBAL_INT', blocking=True, timeout=2)
            if msg is not None:
                logging.debug("Received local target: %s" % str(msg))
                return mavutil.location(msg.lat_int * 1.0e-7, msg.lon_int * 1.0e-7, msg.alt, msg.yaw)
    
    def wait_waypoint(self, wpnum_start, wpnum_end, allow_skip=True, max_dist=2, timeout=400):
        """Wait for waypoint ranges."""
        tstart = time.time()
        start_wp = self.mav.waypoint_current()
        current_wp = start_wp
        mode = self.mav.flightmode        
        self.tata = 'takeoff'
        last_wp_msg = 0
        
        with self.lock:  # Acquire the lock at the beginning of the function
            hand_control = self.hand_control
            current_mode = self.mav.flightmode
        
        while time.time() < tstart + timeout:
            with self.lock:  # Acquire the lock for each iteration
                hand_control = self.hand_control
                current_mode = self.mav.flightmode
            
            if self.mis_can:
                return True
            
            if hand_control:
                # logging.info("Skip wait_waypoint. Using hand control")
                continue
            
            if self.pause:
                if not self.mode_is("BRAKE"):
                    self.change_mode("BRAKE")
                continue
            
            elif not self.mode_is("AUTO") and not self.pause:
                if self.mode_is("BRAKE"):
                    self.change_mode("AUTO")
                continue
            
            seq = self.mav.waypoint_current()
            m = self.mav.recv_match(type='NAV_CONTROLLER_OUTPUT',
                                    blocking=True)
            wp_dist = m.wp_dist
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            
            # if current_mode != mode:
            #     raise WaitWaypointTimeout('Exited %s mode' % mode)
            
            if time.time() - last_wp_msg > 1:
                logging.debug("WP %u (wp_dist=%u Alt=%.02f current_wp: %u, wpnum_end: %u" % (seq, wp_dist, m.alt, current_wp, wpnum_end))
                last_wp_msg = time.time()
            
            if seq == current_wp + 1 or (seq > current_wp + 1 and allow_skip):
                self.tata = 'mission accomplishment'
                logging.debug("test: Starting new waypoint %u" % seq)
                tstart = time.time()
                current_wp = seq
            
            if current_wp == wpnum_end and wp_dist < max_dist:
                logging.debug("Reached final waypoint %u" % seq)
                self.tata = 'return home'
                return True
            
            if seq >= 255:
                logging.debug("Reached final waypoint %u" % seq)
                self.tata = 'return home'
                return True
            
            if seq > current_wp + 1:
                raise WaitWaypointTimeout(("Skipped waypoint! Got wp %u expected %u" % (seq, current_wp + 1)))
        
        raise WaitWaypointTimeout("Timed out waiting for waypoint %u of %u" % (wpnum_end, wpnum_end))
