"""class and script for a trajectory flying crazyflie.

contains an implementation of the udacidrone Drone class to handle the control of a crazyflie through velocity
commands to be able to fly a given trajectory.
also contains a helper class to be able to parse a trajectory file and handle calculating the next point in the
trajectory throughout the flight.

To run the script:

python trajectory_flyer.py --uri radio://0/80/2M

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

# import the controller we want to use
from outer_controller import OuterLoopController


# that height to which the drone should take off
TAKEOFF_ALTITUDE = 0.5


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class TrajectoryHandler(object):

    def __init__(self, filename):

        # set up the variables of interest
        self._positions = []  # list of the [N, E, D] positions for the trajectory in
        self._rel_times = []  # list of the relative times for the trajectory in decimal seconds

        # load the trajectory in
        self._load_trajectory(filename)

        # TODO: determine if need a velocity trajectory array...

    def _load_trajectory(self, filename):
        """helper function to load in a trajectory file.

        given the filename (or path + name), load in the trajectory file defined as follows:
         - each row contains a trajectory point
         - each row consists of the following 4 comma separated values:
            - relative time (seconds into the flight)
            - north position (in [m])
            - east posiiton (in [m])
            - down position (in [m])

        Args:
            filename: the filename containing the trajectory information
        """

        # load the data in
        data  = np.loadtxt(filename, delimiter=',', dtype='Float64')

        for i in range(len(data[:, 0])):

            # first element in each row is the relative time for the trajectroy part
            self._rel_times.append(data[i, 0])

            # elements 2, 3, and 4 are the [N, E, D] position for the timestamp
            self._positions.append(data[i, 1:4])

        # convert the positions and relative times to numpy arrays -> for easier handling later
        self._positions = np.array(self._positions)
        self._rel_times = np.array(self._rel_times)

    def get_next_point(self, inflight_time):
        """get the position and velocity command for the given in flight time.

        given a in flight time (in seconds) determine what the position and velocity
        commands should be.

        Args:
            inflight_time: the time in flight (in seconds), NOT ABSOLUTE TIME
        """

        # get the index of the trajectory with the closest time
        ind_min = np.argmin(np.abs(self._rel_times - inflight_time))

        # get the time of this point
        time_ref = self._rel_times[ind_min]

        # create the position and velocity commands,
        # which depends on if current time is before or after the trajectory point
        if inflight_time < time_ref:  # before the current point

            p0 = self._positions[ind_min - 1]
            p1 = self._positions[ind_min]

            t0 = self._rel_times[ind_min - 1]
            t1 = self._rel_times[ind_min]

        else:  # after the current point

            # now need to check if we are at the end of the file
            if ind_min >= len(self._positions) - 1:

                p0 = self._positions[ind_min]
                p1 = self._positions[ind_min]

                t0 = 0.0
                t1 = 1.0

            else:

                p0 = self._positions[ind_min]
                p1 = self._positions[ind_min + 1]

                t0 = self._rel_times[ind_min]
                t1 = self._rel_times[ind_min + 1]

        # compute the position command (interpolating between points)
        position_cmd = (p1 - p0) * (inflight_time - t0) / (t1 - t0) + p0

        # compute the velocity command based on the desired points and time
        velocity_cmd = (p1 - p0) / (t1 - t0)

        return (position_cmd, velocity_cmd)

    def is_trajectory_completed(self, inflight_time):
        """check if the trajectory has been completed"""
        return (inflight_time > self._rel_times[-1])


class TrajectoryVelocityFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self._target_position = np.array([0.0, 0.0, 0.0])  # the target pos in [N, E, D]
        self._target_velocity = np.array([0.0, 0.0, 0.0])  # the target vel in [Vn, Ve, Vd]
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

        # get the trajectory handler
        self._traj_handler = TrajectoryHandler("line_traj.txt")
        self._start_time = 0.0  # this is the time that the flight started -> will be set on takeoff

    def local_position_callback(self):
        if self._flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self._target_position[2]:
                self._start_time = time.time()
                self.waypoint_transition()

            # TODO: decide if want to also run the position controller from here
            # TODO: if desired, could simply call the takeoff method, but would be more complete to also write it here

            # NOTE: one option for testing out the controller that people could do is use the takeoff method
            # and then simply send the thrust command for hover, this help tune the mass of the drone - if a scale isn't accessible
            # NOTE: this would be an effort to just try and tie in the content from the C++ project

            # TODO: the same dilema is of note for landing
            # could just open it up as a "hey if you want to see what your controller does for your entire flight"

        elif self._flight_state == States.WAYPOINT:

            # get the current in flight time
            rel_time = time.time() - self._start_time

            # check if trajectory is completed
            if self._traj_handler.is_trajectory_completed(rel_time):
                self.landing_transition()
                return

            # set the target position and velocity from the trajectory
            self._target_position, self._target_velocity = self._traj_handler.get_next_point(rel_time)

            # run the outer loop controller (position controller -> to velocity command)
            vel_cmd = self.run_outer_controller()

            # send the velocity command to the drone
            self.cmd_velocity(vel_cmd[0], vel_cmd[1], vel_cmd[2], 0.0)

    def velocity_callback(self):
        if self._flight_state == States.LANDING:
            if abs(self.local_velocity[2]) < 0.05:
                self.disarming_transition()

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
        hdot_cmd = self._outer_controller.altitude_control(-self._target_position[2],
                                                           -self.local_position[2],
                                                           -self._target_velocity[2])

        return np.array([lateral_vel_cmd[0], lateral_vel_cmd[1], -hdot_cmd])

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

        # NOTE: the configuration let's the crazyflie handle the takeoff
        self.takeoff(target_altitude)
        self._flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self._flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        # NOTE: the configuration let's the crazyflie handle the takeoff
        self.land()
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
    drone = TrajectoryVelocityFlyer(conn)

    # a short delay and the start the script
    time.sleep(2)
    drone.start()
