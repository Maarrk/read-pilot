"""
Thread responsible for connection to Pixhawk
"""

import time
import logging
from threading import Thread
import numpy as np

from dronekit import LocationGlobalRelative, VehicleMode
from pymavlink import mavutil  # Needed for command message definitions

import drone_globals as gl
from pixhawk import Pixhawk
from nav_utils import *

log = logging.getLogger()


class PilotThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.mainFlag = True
        self.home_location = None
        self.current_position = None
        self.loop_sleep_time = 0.5
        self.pix = None

        self.target_speed = gl.speed
        self.target_altitude = gl.alt
        self.connection_string = gl.pix_connection_string
        self.connection_baud = gl.pix_connection_baud
        self.waypoints = []
        self.last_sent = [0, 0]

        # Testing with sitl:
        if gl.args.sitl_on:
            self.connection_string = gl.sitl_connection_string
            self.connection_baud = gl.sitl_connection_baud

    def run(self):
        self.pix = Pixhawk(self.connection_string, self.connection_baud)
        success = self.pix.initialize()
        if not success:
            self.mainFlag = False

        if self.mainFlag:
            self.pix.vehicle.groundspeed = self.target_speed
            self.arm_and_takeoff()

        while self.mainFlag:
            # get location
            if len(self.waypoints) > 0:
                wp = LocationGlobalRelative(self.waypoints[0][0], self.waypoints[0][1], self.target_altitude)
                dist = get_distance_metres(self.pix.vehicle.location.global_relative_frame, wp)
                print('Distance to target: {}'.format(dist))
                if dist <= gl.wpt_radius:
                    self.waypoints.pop(0)
                    print("reached waypoint, {} more to go".format(len(self.waypoints)))
                else:
                    self.fly_waypoint(wp.lat, wp.lon)

            time.sleep(self.loop_sleep_time)

    def land(self):
        while not self.pix.vehicle.mode == "RTL":
            self.pix.vehicle.mode = VehicleMode("RTL")
            time.sleep(0.5)

    def arm_and_takeoff(self):
        if self.home_location is None:
            self.home_location = self.pix.vehicle.location.global_relative_frame.lat, \
                                 self.pix.vehicle.location.global_relative_frame.lon

        # Don't try to arm until autopilot is ready
        while not self.pix.vehicle.is_armable:
            time.sleep(0.5)

        while not self.pix.vehicle.mode == "GUIDED":
            if gl.args.sitl_on:
                self.pix.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(0.5)

        while not self.pix.vehicle.armed:
            self.pix.vehicle.armed = True
            time.sleep(0.5)

        self.pix.vehicle.simple_takeoff(self.target_altitude)  # Take off to target altitude

        while True:
            print ("Altitude: ", self.pix.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.pix.vehicle.location.global_relative_frame.alt >= self.target_altitude * 0.95:
                print('achieved target altitude')
                if not gl.takeoff_event.is_set():
                    gl.takeoff_event.set()
                break
            time.sleep(1)

    def fly_waypoint(self, lat, lon):
        waypoint = LocationGlobalRelative(lat, lon, self.target_altitude)

        if self.last_sent[0] == waypoint.lat and self.last_sent[1] == waypoint.lon:
            pass
        else:
            gotoFunction = self.pix.vehicle.simple_goto
            gotoFunction(waypoint)
            self.last_sent[0] = waypoint.lat
            self.last_sent[1] = waypoint.lon
            print("command to fly to waypoint sent")

        # while self.pix.vehicle.mode.name == "GUIDED":
        #     currentLocation = self.pix.vehicle.location.global_relative_frame
        #     remainingDistance = get_distance_metres(currentLocation, waypoint)
        #     if remainingDistance <= 3:
        #         print("Reached target")
        #         break
        #     if not blocking:
        #         break
        #     time.sleep(self.loop_sleep_time)
        # else:
        #     print("uav not in GUIDED mode!!!")

    def position(self):
        return self.pix.vehicle.location.global_relative_frame.lat, \
               self.pix.vehicle.location.global_relative_frame.lon

    def position_ned(self):
        return np.array([self.pix.vehicle.location.local_frame.north,
                         self.pix.vehicle.location.local_frame.east,
                         self.pix.vehicle.location.local_frame.down])

    def attitude(self):
        return np.array([self.pix.vehicle.attitude.roll,
                         self.pix.vehicle.attitude.pitch,
                         self.pix.vehicle.attitude.yaw])

    def goto_position_target_local_ned(self, north, east, down):
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.
        """
        msg = self.pix.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            north, east, down,
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.pix.vehicle.send_mavlink(msg)


    def set_roi(self, lat, lon):
        # create the MAV_CMD_DO_SET_ROI command
        msg = self.pix.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,  # command
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4
            lat,
            lon,
            self.target_altitude
        )
        # send command to vehicle
        self.pix.vehicle.send_mavlink(msg)


    def stop(self):
        self.mainFlag = False


def main():
    pilot = PilotThread()
    pilot.waypoints = [[-35.365261, 149.165430], [-35.363061, 149.165430], [-35.363461, 149.165030]]
    pilot.run()


if __name__ == '__main__':
    main()
