''' This file contains the implementation of the Astar algorithm, and all
    auxiliary functions.
'''
from math import sqrt
import numpy as np
from utils import cost_path



def cost(coord1, coord2):
    return sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def h_oct(x_cur, x_target):
    # x_cur: current node
    # x_target: target node

    x1, y1 = x_cur
    x2, y2 = x_target
    

    dx = abs(x1 - x2)
    dy = abs(y1 - y2)

    return max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)

def reconstruct_path(cameFrom, current):
    # cameFrom: a list of nodes
    # current: the current node
    # return: path, a list of nodes

    total_path = [current]
    # traverse cameFrom from the end
    while current in cameFrom.keys():
        current = cameFrom[current]
        total_path.append(current)
    total_path.reverse()
    return total_path
    





from queue import PriorityQueue
from collections import defaultdict
def AStar(grid, x0, x_target):
    # x0:
    # x_target:
    # return: path, cost
    # path: a list of nodes
    # cost: the cost of the path

    # Initialization

    open_set = PriorityQueue()
    cameFrom = {}


    gScore = defaultdict(lambda: float('inf')) # If value not in dict, return inf
    gScore[x0] = 0
    fScore = defaultdict(lambda: float('inf'))
    fScore[x0] = h_oct(x0, x_target)
    open_set.put((fScore[x0], h_oct(x0, x_target), x0))

    while not open_set.empty(): 
        
        current = open_set.get()[2] # get only the position tuple
        if current == x_target:
            # return the path and the cost
            return reconstruct_path(cameFrom, current), gScore[current]
       
        # compute neighbors of current -- deletes points outside of boundary or inside obstacle
        cur_neighbors = grid.neighbors(current[0], current[1], obstacle_check=True)
        for neighbor in cur_neighbors:
           h_temp = h_oct(neighbor, x_target)
           tentative_gScore = gScore[current] + cost(current, neighbor)
           tentative_fScore = tentative_gScore + h_temp
           if tentative_fScore < fScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = tentative_fScore
                open_set.put((fScore[neighbor], h_temp, neighbor))


