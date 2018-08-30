"""Controller that generates velocity commands

Class for the specific impelementation of a controller that will be comanding velocity commands.
Contains the solution for P controllers on altitude and lateral position to commands velocities
to a crazyflie over the wireless link.


TODO: add some description / comments on how this file can be edited???
Not sure where the best place to do that is....


@author: Adrien Perkins <adrien.perkins@udacity.com>
"""

import numpy as np


class OuterLoopController(object):
    """controller class for computing velocity commands to control lateral position and altitude.

    solution implementation to a controller that computes velocity commands from position and
    altitude commands.
    """

    def __init__(self):

        # define all the gains that will be needed
        self._kp_pos = 0.2  # gain for lateral position error (solution: XXX)
        self._kp_alt = 0.5  # gain for altitude error  (solution: XXX)
        self._kp_yaw = 0  # gain for yaw error (solution: XXX)

        # some limits to use
        self._v_max = 0.3       # the maximum horizontal velocity in [m/s]
        self._hdot_max = 0.5    # the maximum vertical velocity in [m/s]

    def lateral_position_control(self, pos_cmd, pos, vel_cmd=np.array([0.0, 0.0, 0.0])):
        """compute the North and East velocity command to control the lateral position.

        Use a PID controller (or your controller of choice) to compute the North and East velocity
        commands to be send to the crazyflie given the commanded position and current position.

        Args:
            pos_cmd: the commanded position [north, east, down] in [m]
            pos: the current position [north, east, down] in [m]
            vel_cmd: the commanded velocity [vn, ve, vd] in [m/s]

        Returns:
            the velocity command as a 2 element numpy array [Vn, Ve]
            numpy array
        """

        # TODO: compute a [Vn, Ve] command

        # solution #
        vel_cmd[0:2] += self._kp_pos * (pos_cmd[0:2] - pos[0:2])
        vel_cmd[0:2] = np.clip(vel_cmd[0:2], -self._v_max, self._v_max)
        # solution #

        return vel_cmd[0:2]

    def altitude_control(self, alt_cmd, alt, hdot_cmd=0.0):
        """compute the vertical velocity command to control the altitude.

        Use a PID controller (or your controller of choice) to compute the vertical velocity
        command to be send to the crazyflie given the commanded altitude and current altitude.

        Args:
            alt_cmd: the commanded altitude in [m] (positive up)
            alt: the current altitude in [m] (positive up)
            hdot_cmd: the commanded vertical velocity in [m/s] (positive up)

        Returns:
            the vertical velocity to command
            float
        """

        # TODO: compute a [Vup] command

        # solution #
        hdot_cmd += self._kp_alt * (alt_cmd - alt)
        hdot_cmd = np.clip(hdot_cmd, -self._hdot_max, self._hdot_max)
        # solution #

        return hdot_cmd
