import numpy as np
from math import sqrt, atan2, pi, tan
from graph import *
from utils import *

def simulate_odo(q1, q2, sigma_d, sigma_phi):
    """Simulate odometry measurements from two poses

    Args:
        q1, q2 (np.ndarray): perfect poses

    Returns:
        list: list of odometry measurements
    """
    # Compute perfect measure
    dd = np.linalg.norm(q2[:2] - q1[:2])
    dphi = angle_mod_pi(q2[2] - q1[2])

    # Compute noisy measure
    d = np.random.normal(dd, sigma_d)
    phi = np.random.normal(dphi, sigma_phi)


    # Return measure
    return [d, phi]
    

def dead_recon(q_start,q_end, real_traj, P0, sigma_d, sigma_phi):
    """Dead reckoning algorithm

    Args:
        q_start (np.ndarray): starting pose
        q_end (np.ndarray): ending pose
        real_q (np.ndarray): real pose
        P0 (np.ndarray): initial covariance
        sigma_d (float): linear noise
        sigma_phi (float): angular noise

    Returns:
        np.ndarray: estimated pose
        np.ndarray: estimated covariance
    """
    # Initialize
    q_hat = [q_start]
    P = P0

    # Iterate
    for i in range(len(real_traj) - 1):
        # Simulate odometry
        [d_d, d_phi] = simulate_odo(real_traj[i], real_traj[i + 1], sigma_d, sigma_phi)
    
        q_hat[i+1] = q_hat[i] + np.array([d_d * np.cos(q_hat[i, 2]), d_d * np.sin(q_hat[i, 2]), d_phi]) 
       

    # Return
    return q_hat, P