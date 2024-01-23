import numpy as np
from math import sqrt, atan2, pi
from graph import *
from rover import *

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

                start_idx = data_idx
                was_straight = 1
                curve_list = []
            elif was_straight:
                # We are still in a straight line
                pass
    
    # save last straight line to new_path
    link = Link(graph.links[start_idx].tail, graph.links[-1].head)
    new_path.append([link])
                
    return new_path

def control_point(rover, start_point, end_point):
    # graph: the graph of the path
    # rover: the rover object
    #  
    a =1 
def control_rect(rover, start_point, end_point):
    # graph: the graph of the path
    # rover: the rover object
    #
    pos = start_point
    v = rover.vmax
    Kv, Kh = 0.10, 0.10 # Velocity and heading gains
    while not np.allclose(pos, end_point):
        d = sqrt((pos[0] - end_point[0])**2 + (pos[1] - end_point[1])**2) # Distance to the end point
        v = Kv * d
        # Calculate the heading
        theta = atan2(end_point[1] - pos[1], end_point[0] - pos[0])      # should be in the range [-pi, pi]


def control_pose(rover,start_point, start_theta, end_point, end_theta):
    a =1
          
        

        

            

        


