"""Controller class for computing attitude and thrust commands.

implementation of a lateral position, altitude, yaw, and velocity controllers for controlling a
crazyflie via the (Attitude, Thrust) API.

TODO: add copywright and license header

@author Adrien Perkins <adrien.perkins@udacity.com>
"""

import numpy as np

# some necessary constants
DRONE_M = 0.031         # [kg]
GRAVITY_MAG = 9.81      # [m/s^2] -> magnitude only
MAX_THRUST_N = 0.63 #0.625    # the maximum amount of thrust the crazyflie can generate in [N] - DO NOT EDIT


class InnerLoopController(object):

    def __init__(self):

        # the gains that are needed
        self._kp_vel = 0.14  # the gain on the velocity to get attitude (solution: XXX)
        self._kp_hdot = 1.0  # the gain on the vertical velocity to get accel (solution: XXX)

        # some limits to use
        self._bank_max = np.radians(20)     # max bank (roll and pitch) angle - in radians
        self._haccel_max = 1.2              # the maximum vertical acceleration in [m/s^2]

    def velocity_control(self, vel_cmd, vel):
        """compute attitude and thrust commands to achieve a commanded velocity vector.

        Use a PID controller (or your controller of choice) to compute the attitude (roll/pitch) to
        achieve the commanded (North, East) velocity and compute a normalized thrust to achieve the
        commanded down velocity.

        Args:
            vel_cmd: the commanded velocity vector as a numpy array [Vnorth, Veast, Vdown] in [m/s]
            vel: the current velocity vector as a numpy array [Vnorth, Veast, Vdown] in [m/s]
        """

        # change down velocities to up hdots
        hdot_cmd = -vel_cmd[2]
        hdot = -vel[2]

        # TODO: compute an attitude command from the given velocity command
        # TODO: compute a normalized thrust from the given hdot command
        # NOTE: the velocity command will be set by the lateral_position_control function above!

        # solution #
        pitch = -self._kp_vel * (vel_cmd[0] - vel[0])  # note the sign change!  Remember + pitch is up, meaning it will send out drone backwards!
        roll = self._kp_vel * (vel_cmd[1] - vel[1])

        # add some limits
        pitch_cmd = np.clip(pitch, -self._bank_max, self._bank_max)
        roll_cmd = np.clip(roll, -self._bank_max, self._bank_max)

        # compute acceleration and then thrust for vertical
        accel_cmd = self._kp_hdot * (hdot_cmd - hdot)
        accel_cmd = np.clip(accel_cmd, -self._haccel_max, self._haccel_max)
        thrust_cmd_N = DRONE_M * (accel_cmd + GRAVITY_MAG) / (np.cos(pitch_cmd) * np.cos(roll_cmd)) # positive up

        # need to normalize the thrust
        thrust_cmd = thrust_cmd_N / MAX_THRUST_N
        # solution #

        # return a tuple with the roll, pitch, and thrust commands
        return (roll_cmd, pitch_cmd, thrust_cmd)
