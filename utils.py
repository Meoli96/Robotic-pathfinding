# file for utility functions used in the project
import numpy as np
from grid import *
from graph import *


day_in_secs = 24 * 60 * 60 # day in seconds
sol_in_secs = day_in_secs * 1.0274912517 # sol in seconds

def angle_mod_pi(angle):
    # Normalize angle in [-pi, pi]
    return np.mod(angle + np.pi, 2 * np.pi) - np.pi
def angle_mod_2pi(angle):
    # Normalize angle in [0, 2pi]
    return np.mod(angle, 2 * np.pi)


def shorten_path(graph: Graph, theta0, theta_target):
    # path: a graph of nodes, solution of the A* algorithm
    # pose: the current pose of the robot

    # The idea of this function is to generate a list of straight lines and curves, to be
    # used by the controller. The controller will follow the lines and curves, and will
    # stop when it reaches the end of the path.
    
    changes = np.diff(graph.directions)
    
    # Create a new graph with less links, removing consecutive links with the same direction
    new_path = []
    straight_arr = []
    start_idx = 0
    counter = 4 # > 3 to have a straight line at the beginning
    curve_list = []
    was_straight = 1 

    for data_idx in range(len(changes)):
        if changes[data_idx] != 0:
            # Change in direction detected, reset counter of multiple links with the same direction
            
            if was_straight: # we are exiting from a straight line
                # save last straight line to new_path
                # we can create one link with link_start.tail, link_end-1.head == link_end.tail
                link_straight = Link(graph.links[start_idx].tail, graph.links[data_idx-1].head) # == graph.links[data_idx].head 
                new_path.append([link_straight])
                straight_arr.append(was_straight)
                
                was_straight = 0
                start_idx = data_idx
                curve_list = []

            
            counter = 0
            curve_list.append(graph.links[data_idx])
        else:
            # Same direction, increment counter 
            counter += 1
            if not was_straight and (counter < 3 or changes[data_idx + 1] != 0): # still in a curved line
                curve_list.append(graph.links[data_idx])

            elif not was_straight and counter == 3:
                # We are entering into a straight line, we are ok even if the next link is a curve
                curve_list.append(graph.links[data_idx])
                new_path.append(curve_list)
                straight_arr.append(was_straight)

                start_idx = data_idx+1
                was_straight = 1
                curve_list = []
            elif was_straight:
                # We are still in a straight line
                pass
    
    # save last straight line to new_path
    link = Link(graph.links[start_idx].tail, graph.links[-1].head)
    new_path.append([link])
                
    return new_path

def list_np_angle(array: np.ndarray):
    # return the list in degrees between 0 and 360 of the angles in array
    return [angle_mod_2pi(angle) * 180 / np.pi for angle in array]

def cost_path(array: np.ndarray):
    # return the cost of the array
    cost = 0
    for i in range(len(array) - 1):
        cost += np.linalg.norm(array[i+1] - array[i])
    return cost

def map_obstacle(map, obstacle):
    # Return a map with the obstacle superimposed
    map = map.copy()
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            if obstacle[i][j] == 255:
                map[i][j] = 255
    return map

def obsv_landmark(pos, landmark):
    # Return obserbables computed from pos and landmark
    # pos: robot pose
    # landmark: landmark position

    # Compute the distance between the robot and the landmark
    dist = np.linalg.norm(pos[:2] - landmark)
    # compute the angle between the robot and the landmark
    angle = np.arctan2(landmark[1] - pos[1], landmark[0] - pos[0]) - pos[2]

    return np.array([dist, angle])  