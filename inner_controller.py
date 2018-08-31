"""Controller class for computing attitude and thrust commands.

implementation of the inner loop controller which controls velocity through attitude and thrust commands.

@author Adrien Perkins
"""

import numpy as np

# some necessary constants
DRONE_M = 0.031         # [kg]
GRAVITY_MAG = 9.81      # [m/s^2] -> magnitude only
MAX_THRUST_N = 0.63     # the maximum amount of thrust the crazyflie can generate in [N] - DO NOT EDIT


class InnerLoopController(object):

    def __init__(self):

        # the gains that are needed
        self._kp_vel = 0.0   # the gain on the velocity to get attitude
        self._kp_hdot = 0.0  # the gain on the vertical velocity to get accel

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

        # Student TODO: compute an attitude command from the given velocity command
        # Student TODO: compute a normalized thrust from the given hdot command


        # return a tuple with the roll, pitch, and thrust commands
        return (roll_cmd, pitch_cmd, thrust_cmd)
