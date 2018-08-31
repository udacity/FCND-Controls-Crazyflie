"""class and script for an implementation of the udacidrone Drone to control a crazyflie using
attitude / thrust commands.

A full implementation of the drone class to work with the `OuterLoopController` and `InnerLoopController to remotely
control the position of the crazyflie using the attitude command API.

Make sure to read through the examples below to help you out getting started with running this flyer.
Note that the initial configuration of the file is to have the crazyflie takeoff and try and hold (0,0,-0.5).

To run the script:

python attitude_flyer.py --uri radio://0/80/2M

IMPORTANT NOTE: if at any time you need to stop the script and control of the crazyflie, if ctrl+c does not work,
simply unplug the crazyradio dongle from your computer and the crazyflie will go into "stop" mode after a couple
seconds (this will make it fall out of the sky).

@author Adrien Perkins
"""

import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, CrazyflieConnection  # noqa: F401
from udacidrone.messaging import MsgID

# import the controllers we want to use
from inner_controller import InnerLoopController
from outer_controller import OuterLoopController

# NOTE: a waypoint here is defined as [North, East, Down]

###### EXAMPLES ######
#
# here are a set of example waypoint sets that you might find useful for testing.
# each has a bit of a description to help with the potential use case for the waypoint set.
#
# NOTE: the waypoint lists are defined as a list of lists, which each entry in a list of the
# [North, East, Down] to command.  Also recall for the crazyflie, North and East are defined
# by the starting position of the drone (straight and left, respectively), not world frame North
# and East.
#
###### ######## ######

######
# 1. have the crazyflie hover in a single place.
#
# For this to work best, make sure to comment out the waypoint transition code (see block comment
# in `local_position_callback`) to ensure that the crazyflie attempts to hold this position.
######

WAYPOINT_LIST = [[0.0, 0.0, -0.5]]


######
# 2. there and back.
#
# Simple 2 point waypoint path to go away and come back.
######

# WAYPOINT_LIST = [
#     [1.5, 0.0, -0.5],
#     [0.0, 0.0, -0.5]
#     ]


######
# 3. simple box.
#
# A simple box, much like what you flew for the backyard flyer examples.
######

# WAYPOINT_LIST = [
#     [1.0, 0.0, -0.5],
#     [1.0, 1.0, -0.5],
#     [0.0, 1.0, -0.5],
#     [0.0, 0.0, -0.5]
#     ]


# that height to which the drone should take off
TAKEOFF_ALTITUDE = 0.5

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class AttitudeFlyer(Drone):
    """Implementation of the udacidrone Drone class to control a crazyflie using attitude commands.

    uses both the inner and outer controller to compute the desired commands to send to the
    crazyflie when each respective callback is triggered.
    """

    def __init__(self, connection):
        super().__init__(connection)

        # some local variables
        self._target_position = np.array([0.0, 0.0, 0.0])  # [North, East, Down]
        self._target_velocity = np.array([0.0, 0.0, 0.0])  # [Vn, Ve, Vd]
        self._all_waypoints = []
        self._in_mission = True

        # initial state
        self._flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        # get the controller that will be used
        self._outer_controller = OuterLoopController()
        self._inner_controller = InnerLoopController()

        # velocity commands
        # recall that the first level of controller computes velocity commands, and then
        # those commands are converted to attitude commands
        #
        # this is done in 2 separate callbacks as the computation of the velocity command does not
        # need to happen at the same time as the attitude command computation
        self._velocity_cmd = np.array([0.0, 0.0, 0.0])  # this is [Vn, Ve, Vd]

    def local_position_callback(self):
        if self._flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > -0.95 * self._target_position[2]:
                self._all_waypoints = self.calculate_box()
                self.waypoint_transition()

        # NOTE: as configured this controller handles the takeoff and landing conditions as well as waypoint flight.
        # however, you can configure it to let the crazyflie control it by uncommenting the
        # `self.land()` and `self.takeoff()` functions in their respective transition functions
        # and by removing those states from this elif line.
        elif self._flight_state == States.WAYPOINT or self._flight_state == States.TAKEOFF or self._flight_state == States.LANDING:

            # DEBUG
            # print("curr pos: ({:.2f}, {:.2f}, {:.2f}), desired pos: ({:.2f}, {:.2f}, {:.2f})".format(
            #     self.local_position[0], self.local_position[1], self.local_position[2],
            #     self._target_position[0], self._target_position[1], self._target_position[2]))

            ########################### Waypoint Incrementing Block ################################
            #
            # NOTE: comment out this block of code if you wish to have the crazyflie simply hold
            # its first waypoint position.
            # This is a good way to be able to test your initial set of gains without having to
            # worry about your crazyflie flying away too quickly.
            #
            # self.check_and_increment_waypoint()
            ########################################################################################

            # run the outer loop controller (position controller -> to velocity command)
            self._velocity_cmd = self.run_outer_controller()

            # NOTE: not sending the velocity command here!
            # this just sets the velocity command, which is used in the velocity callback, which
            # sends the attitude commands to the crazyflie.

    def velocity_callback(self):

        if self._flight_state == States.LANDING:
            if -1.0*self.local_position[2] < 0.1 and abs(self.local_velocity[2]) < 0.05:
                self.disarming_transition()

        # NOTE: this will run your controller during takeoff and landing.
        # to disable that functionality, you will need to remove those conditions from this if statement
        if self._flight_state == States.WAYPOINT or self._flight_state == States.TAKEOFF or self._flight_state == States.LANDING:
            # run the inner loop controller
            roll_cmd, pitch_cmd, thrust_cmd = self.run_inner_controller()

            # NOTE: yaw control is not implemented, just commanding 0 yaw
            self.cmd_attitude(roll_cmd, pitch_cmd, 0.0, thrust_cmd)

    def state_callback(self):
        if self._in_mission:
            if self._flight_state == States.MANUAL:
                self.arming_transition()
            elif self._flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
            elif self._flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def check_and_increment_waypoint(self):
        """helper function to handle waypoint checks and transitions

        check if the proximity condition has been met for a waypoint and
        transition the waypoint as needed.
        if there are no more waypoints, trigger the landing transition.
        """

        # NOTE: depending on how aggressive of paths you are flying, and how reliably you want
        # them to be flown, you may want to add the vertical axis to the distance check.
        if np.linalg.norm(self._target_position[0:2] - self.local_position[0:2]) < 0.2:
            if len(self._all_waypoints) > 0:
                self.waypoint_transition()
            else:
                if np.linalg.norm(self.local_velocity[0:2]) < 0.2:
                    self.landing_transition()

    def run_outer_controller(self):
        """helper function to run the outer loop controller.

        calls the outer loop controller to run the lateral position and altitude controllers to
        get the velocity vector to command.

        Returns:
            the velocity vector to command as [vn, ve, vd] in [m/s]
            numpy array of floats
        """

        lateral_vel_cmd = self._outer_controller.lateral_position_control(self._target_position,
                                                                          self.local_position,
                                                                          self._target_velocity)

        hdot_cmd = self._outer_controller.altitude_control(-self._target_position[2], -self.local_position[2])

        return np.array([lateral_vel_cmd[0], lateral_vel_cmd[1], -hdot_cmd])

    def run_inner_controller(self):
        """helper function to run the inner loop controller.

        calls the inner loop controller to run the velocity controller to get the attitude and
        thrust commands.
        note that thrust in this case is a normalized value (between 0 and 1)

        Returns:
            a tuple of roll, pitch, and thrust commands
            a tuple (roll, pitch, thrust)
        """
        return self._inner_controller.velocity_control(self._velocity_cmd, self.local_velocity)

    def calculate_box(self):
        print("Setting Home")
        local_waypoints = WAYPOINT_LIST
        return local_waypoints

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_as_current_position()
        self._flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = TAKEOFF_ALTITUDE
        self._target_position[2] = target_altitude

        # NOTE: the current configuration has the controller command everything from takeoff to landing
        # to let the crazyflie handle takeoff, uncomment the follow and change the conditions accordingly
        # in the velocity callback
        # self.takeoff(target_altitude)
        self._flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self._target_position = self._all_waypoints.pop(0)
        print('target position', self._target_position)
        self._flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        # NOTE: the current configuration has the controller command everything from takeoff to landing
        # to let the crazyflie handle landing, uncomment the follow and change the conditions accordingly
        # in the velocity callback
        # self.land()
        self._flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.release_control()
        self._flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.stop()
        self._in_mission = False
        self._flight_state = States.MANUAL

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        # self.connect()

        print("starting connection")
        # self.connection.start()

        super().start()

        # Only required if they do threaded
        # while self._in_mission:
        #    pass

        self.stop_log()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--uri', type=str, default='radio://0/80/2M', help="uri of crazyflie, i.e. 'radio://0/80/2M'")
    args = parser.parse_args()

    # create the connection
    conn = CrazyflieConnection('{}'.format(args.uri))

    # create the drone
    drone = AttitudeFlyer(conn)

    # a short delay and the start the script
    time.sleep(2)
    drone.start()