import numpy as np
from math import sqrt, atan2, pi, tan
from graph import *
from utils import *

def simulate_odo(q1, q2):
    """Simulate odometry measurements from two poses

    Args:
        q1, q2 (np.ndarray): perfect poses

    Returns:
        list: list of odometry measurements
    """
    # Compute perfect measure
    dd = np.linalg.norm(q2[:2] - q1[:2])
    dphi = angle_mod_pi(q2[2] - q1[2])
    
    # Return measure
    return [dd, dphi]

def add_noise(value_arr, sigma_arr):
    if isinstance(value_arr, list) or isinstance(value_arr, tuple) or isinstance(value_arr, np.ndarray):
        # list of values given in input
        if (len(value_arr) != len(sigma_arr)):
            raise ValueError("Value and sigma array must have same length")
        value_arr = np.array(value_arr)
        sigma_arr = np.array(sigma_arr)

        return value_arr + np.random.normal(0, scale=sigma_arr)
    elif (isinstance(value_arr, float) or isinstance(value_arr, int)) and (isinstance(sigma_arr, float) or isinstance(sigma_arr, int)):
        # single value given in input
        value_arr = np.array([value_arr])
        sigma_arr = np.array([sigma_arr])
        return np.random.normal( value_arr , scale = sigma_arr)

def F_q(d_d, theta):
    # Compute F_q == dq_k+1/dq -- d_d should be without odometry noise	
    return np.array([[1, 0, -d_d * np.sin(theta)], [0, 1, d_d * np.cos(theta)], [0, 0, 1]])
def F_v(theta):
    # Compute F_v == dq_k+1/dv
    return np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])

def Hq(q, t_lm: tuple):
    # Compute H_q == dq/dq_k+1
    xlm, ylm = t_lm
    x, y, theta = q

    rho = sqrt((xlm - x)**2 + (ylm - y)**2)
    return np.array([[-(xlm - x)/rho, -(ylm - y)/rho, 0], [(ylm - y)/rho**2, -(xlm - x)/rho**2, -1]])

def Hv(q, t_lm: tuple):
    return np.eye(2)


def dead_recon(real_traj, P0, sigma_d, sigma_phi):
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
    q_hat = np.zeros(real_traj.shape)
    q_hat[0] = real_traj[0]
    # Initialize covariance array as M n*n matrix
    n = P0.shape[0]
    M = real_traj.shape[0]
    P_arr = np.zeros((M, n, n))
    P_arr[0] = P0

    # Iterate
    for i in range(len(real_traj) - 1):
        # Simulate perfect odometry with real trajectory
        [d_d, d_phi] = simulate_odo(real_traj[i], real_traj[i +1])
        # Compute F_q and F_v
        F_q_ = F_q(d_d, q_hat[i][2])
        F_v_ = F_v(q_hat[i][2])

        # Add noise to odometry
        [d_d, d_phi] = add_noise([d_d, d_phi], [sigma_d, sigma_phi])
        
        # Compute q_hat and P
        q_hat[i+1] = q_hat[i] + np.array(
            [d_d * np.cos(q_hat[i][2]), d_d * np.sin(q_hat[i][2]), d_phi]) 
        P_arr[i+1] = F_q_ @ P_arr[i] @ F_q_.T + F_v_ @ np.diag([sigma_d**2, sigma_phi**2]) @ F_v_.T

    # Return
    return q_hat, P_arr
    

def EKF(grid: Grid, real_traj, P0, sigma_d, sigma_phi, sigma_r, sigma_theta, lidar_range = 100):
    # Initialize
    q_hat = np.zeros(real_traj.shape)
    q_hat[0] = real_traj[0]

    # Initialize covariance array as M n*n matrix
    n = P0.shape[0]
    M = real_traj.shape[0]
    P_arr = np.zeros((M, n, n))
    P_arr[0] = P0
    lm_list = []

    # Iterate
    for i in range(len(real_traj) - 1): # just because we have a trajectory integrated with a the same frequency of the measurements
        
        # Simulate perfect odometry with real trajectory
        [d_d, d_phi] = simulate_odo(real_traj[i], real_traj[i +1])
        # Compute F_q and F_v
        F_q_ = F_q(d_d, q_hat[i][2])
        F_v_ = F_v(q_hat[i][2])

        # Add noise to odometry
        [d_d, d_phi] = add_noise([d_d, d_phi], [sigma_d, sigma_phi])
        
        # Compute q_hat and P
        q_hat[i+1] = q_hat[i] + np.array(
            [d_d * np.cos(q_hat[i][2]), d_d * np.sin(q_hat[i][2]), d_phi]) 
        P_arr[i+1] = F_q_ @ P_arr[i] @ F_q_.T + F_v_ @ np.diag([sigma_d**2, sigma_phi**2]) @ F_v_.T
        
        

        # Check if there is any landmark in the range of the sensor
        r_lm  = grid.find_landmark(real_traj[i+1], lidar_range) # 500 m range
        # r_rho contains (range, beta)
        if r_lm != [] and np.linalg.det(P_arr[i+1]) > 10**(-8):
            # landmarks detected
            # for each landmark, compute H_q and H_v
            if not np.isfinite(q_hat[i+1][2]):
                print("q_hat[2] is not finite")
                print(q_hat[i+1])
                raise ValueError
            for i_lm in range(len(r_lm)):
               
                # get the index of the landmark
                H_q_ = Hq(q_hat[i+1], r_lm[i_lm])
                H_v_ = Hv(q_hat[i+1], r_lm[i_lm])
                z_nl = obsv_landmark(real_traj[i+1], r_lm[i_lm]) # z_noiseless -- observed from real state
                z = add_noise(z_nl, [sigma_r, sigma_theta])
                # Compute Kalman gain                
                K = P_arr[i+1] @ H_q_.T @ np.linalg.inv(H_q_ @ P_arr[i+1] @ H_q_.T + H_v_ @ np.diag([sigma_r**2, sigma_theta**2]) @ H_v_.T)

                h_q = obsv_landmark(q_hat[i+1], r_lm[i_lm]) # h_q -- observed from estimated state
                # Update q_hat and P
                q_hat[i+1] = q_hat[i+1] + K @ (z - h_q)
                P_arr[i+1] = (np.eye(n) - K @ H_q_) @ P_arr[i+1]
    return q_hat, P_arr, 

   


