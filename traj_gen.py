import numpy as np
from math import sqrt, atan2, pi, tan
from graph import *
from rover import *
from utils import angle_mod_pi



class Trajectory:
    # list of links with informations about straight line or not
    def __init__(self, links = [], straight_arr = [], start_pose = 0, end_pose = 0):
        self.links = links
        self.straight_arr = straight_arr
        self.n_links = len(links)
        self.start_pose = start_pose
        self.end_pose = end_pose

    def add_links(self, links, straight = 0):
        # Adds a list of links. If straight is 1, all the link in the list have
        # the same direction
        self.links.append(links)
        self.straight_arr.append(straight)

        



def control_point(rover, q0, q0_dot, qf, dt, k_v = None, k_h = None):
    # rover: the rover object
    # q_start: the starting point of the link (x,y, theta)
    # q_end: the final point of the link (x, y, 0)
    # q_start_dot: the starting velocity of the link (x_dot, y_dot)
    # q_end_dot: the final velocity of the link (x_dot, y_dot)

    ''' With a timestep dt we can achieve a spatial accuracy of at worse
        v_max * dt, where v_max is the maximum velocity of the rover.
        So we need to make sure that the distance accuracy is within 
        the desired threshold.
    '''
    if k_v is None:
        k_v = 0.08
    if k_h is None:
        k_h = 0.4
    
    q = []
    q_dot = []
    d = 1
    q.append(q0)
    q_dot.append(q0_dot)


    

    # Calculate the velocity
    v_max = rover.vmax
    L = rover.Laxis
  

    while d > 0.5:
        # Calculate the distance between the two points
        d = sqrt((qf[0] - q[-1][0])**2 + (qf[1] - q[-1][1])**2)
        

        v = k_v * d
        h = atan2(qf[1] - q[-1][1], qf[0] - q[-1][0])
        gamma = angle_mod_pi(k_h * (h - q[-1][2]))

        if v > v_max:
            v = v_max
        x_dot = v * np.cos(q[-1][2])
        y_dot = v * np.sin(q[-1][2])
        theta_dot = (v/L)*tan(gamma)
        
        q_dot.append(np.array([x_dot, y_dot, theta_dot]))
        # Integrate the velocity to get the position
        q.append(integrate_step(q[-1], q_dot[-1], dt))
    return np.stack(q), np.stack(q_dot)
    







def control_straight(rover, q0, q0_dot, qf, dt, k_h = None, k_d = None, maxiter = 1000):
    # graph: the graph of the path
    # rover: the rover object
    #
    # a, b, c: rectilinear path parameters  
    # start_pose: the starting pose of the rover
    # end_pose: the final pose of the rover (if None, the rover will stop at the end of the path)
    

    
    if k_h is None:
        k_h = 0.4
    if k_d is None:
        k_d = 0.1
    min_dist = 99
    
   
    
    v_max = rover.vmax
    L = rover.Laxis


    q = []
    q_dot = []
    d = 1
    q.append(q0)
    q_dot.append(q0_dot)

    iterator = 0
    if (qf[0] - q0[0]) >= 0:
        v = v_max
    else:
        v = -v_max
    

     # Compute a, b, c from q0 and qf
    a = qf[1] - q0[1]
    b = q0[0] - qf[0]
    c = q0[1]*qf[0] - q0[0]*qf[1]

    # orthogonal distance from the line 
    while d > 0.2 and iterator < maxiter:
        d_ort = abs(a*q[-1][0] + b*q[-1][1] + c)/sqrt(a**2 + b**2)
        d = sqrt((qf[0] - q[-1][0])**2 + (qf[1] - q[-1][1])**2)
        if d < min_dist:
            min_dist = d

        #v = k_v * d
        h = atan2(-a, b)
        gamma = angle_mod_pi(k_h * (h - q[-1][2])  - k_d * d_ort) 


        
        x_dot = v * np.cos(q[-1][2])
        y_dot = v * np.sin(q[-1][2])
        theta_dot = (v/L)*tan(gamma)

        q_dot.append(np.array([x_dot, y_dot, theta_dot]))
        # Integrate the velocity to get the position
        q.append(integrate_step(q[-1], q_dot[-1], dt))
        iterator += 1
    print("min dist: ", min_dist)
    return np.stack(q), np.stack(q_dot)





def control_pose(rover,q_start, q_end, dt, k_rho = None, k_alpha = None, k_beta = None):
    # rover: the rover object
    # q_start: the starting point of the link (x,y, theta)
    # q_end: the final point of the link (x, y, 0)
    # q_start_dot: the starting velocity of the link (x_dot, y_dot)
    # q_end_dot: the final velocity of the link (x_dot, y_dot)

    ''' With a timestep dt we can achieve a spatial accuracy of at worse
        v_max * dt, where v_max is the maximum velocity of the rover.
        So we need to make sure that the distance accuracy is within 
        the desired threshold.
    '''
    if k_alpha is None:
        k_alpha = 8
    if k_beta is None:
        k_beta = -3
    if k_rho is None:
        k_rho = 3
    
    if k_rho > 0 and k_beta < 0 and k_alpha-k_rho > 0:
        print("Stable: k_rho, k_alpha, k_beta = (" + 
              str(k_rho)+ ", " +  str(k_alpha) + ", " + str(k_beta) + ")")
    else:
        raise ValueError("Not stable")
    q = []
    q_dot = []

    t = 0
    theta_star = q_end[2] # Assumed radians

    q.append(q_start)
    q_dot.append(np.array([0, 0, 0]))

    x_diff = q_end[0] - q_start[0]
    y_diff = q_end[1] - q_start[1]
    
    rho = sqrt(x_diff**2 + y_diff**2)
    while rho > 0.1:
        x_diff = q_end[0] - q[-1][0]
        y_diff = q_end[1] - q[-1][1]
        
        rho = sqrt(x_diff**2 + y_diff**2)
        alpha = angle_mod_pi(np.arctan2(y_diff, x_diff) - q[-1][2])    
        beta = angle_mod_pi(-q[-1][2] - alpha + theta_star)

        v = k_rho * rho
        w = k_alpha * alpha + k_beta * beta

        if alpha > np.pi/2 or alpha < -np.pi/2:
            v = -v
        if abs(v) > rover.vmax:
            v = np.sign(v)*rover.vmax
        if abs(w) > rover.wmax:
            w = np.sign(w)*rover.wmax
        
        
        x_dot = v * np.cos(q[-1][2])
        y_dot = v * np.sin(q[-1][2])
        theta_dot = w
        q_dot.append(np.array([x_dot, y_dot, theta_dot]))
        # Integrate the velocity to get the position
        q.append(integrate_step(q[-1], q_dot[-1], dt))
        t += dt
    return np.stack(q), np.stack(q_dot), t
    

        

        

            

def integrate_step(value0:np.ndarray, value0_dot:np.ndarray, dt):
    # Integrate a value with respect to time
    return value0 + value0_dot * dt