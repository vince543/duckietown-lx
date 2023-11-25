from typing import Tuple

import numpy as np
import sys
sys.path.append('../')

def LeftEncoderCallback(left_encoder_msg):
    
    N_tot_left = left_encoder_msg.resolution # number of ticks per wheel revolution on the left wheel
    ticks_left = left_encoder_msg.data # incremental count of ticks from the left encoder
    
    return N_tot_left, ticks_left

def RightEncoderCallback(right_encoder_msg):
    
    N_tot_right = right_encoder_msg.resolution # number of ticks per wheel revolution on the left wheel
    ticks_right = left_encoder_msg.data # incremental count of ticks from the left encoder
    
    return N_tot_right, ticks_right

def delta_phi(ticks_left, N_tot_left, prev_ticks_left: int, ticks_right, N_tot_right, prev_ticks_right: int) -> Tuple[float, float, float, float]:
    """
    Args:
        ticks: Current tick count from the encoders.
        prev_ticks: Previous tick count from the encoders.
        resolution: Number of ticks per full wheel rotation returned by the encoder.
    Return:
        dphi: Rotation of the wheel in radians.
        ticks: current number of ticks.
    """
    alpha_left = 2 * np.pi / N_tot_left # wheel rotation per tick in radians for the left wheel
    dphi_left = alpha_left*(ticks_left-prev_ticks_left)

    alpha_right = 2 * np.pi / N_tot_right # wheel rotation per tick in radians for the right wheel
    dphi_right = alpha_right*(ticks_right-prev_ticks_right)
    
    return dphi_left, ticks_left, dphi_right, ticks_right


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
