from typing import Tuple

import numpy as np
import sys
sys.path.append('../')


def delta_phi(ticks: int, prev_ticks: int,resolution: int) -> Tuple[float, float]:
    """
    Args:
        ticks: Current tick count from the encoders.
        prev_ticks: Previous tick count from the encoders.
        resolution: Number of ticks per full wheel rotation returned by the encoder.
    Return:
        dphi: Rotation of the wheel in radians.
        ticks: current number of ticks.
    """


    alpha= 2*np.pi/resolution # wheel rotation per tick in radians for the right wheel
    dphi = alpha*(ticks-prev_ticks)
    
    return dphi, ticks


def pose_estimation(
    R: float,
    baseline: float,
    x_prev: float,
    y_prev: float,
    theta_prev: float,
    delta_phi_left: float,
    delta_phi_right: float,
) -> Tuple[float, float, float]:

    """
    Calculate the current Duckiebot pose using the dead-reckoning model.

    Args:
        R:                  radius of wheel (both wheels are assumed to have the same size) - this is fixed in simulation,
                            and will be imported from your saved calibration for the real robot
        baseline:           distance from wheel to wheel; 2L of the theory
        x_prev:             previous x estimate - assume given
        y_prev:             previous y estimate - assume given
        theta_prev:         previous orientation estimate - assume given
        delta_phi_left:     left wheel rotation (rad)
        delta_phi_right:    right wheel rotation (rad)

    Return:
        x_curr:                  estimated x coordinate
        y_curr:                  estimated y coordinate
        theta_curr:              estimated heading
    """

    d_left = R*delta_phi_left       # distance traveled by left wheel since last time step
    d_right = R*delta_phi_right     # distance traveled by right wheel since last time step

    d_A = (d_left + d_right)/2                  # distance traveled by the middle of the duckiebot
    delta_theta = (d_right-d_left)/baseline     # angle traveled since last time step

    theta_curr = theta_prev + delta_theta       # total angle traveled
    x_curr = x_prev + d_A*np.cos(theta_curr)    # current x-position
    y_curr = y_prev + d_A*np.sin(theta_curr)    # current y-position
    

    return x_curr, y_curr, theta_curr
