''' This file contains the implementation of the Astar algorithm, and all
    auxiliary functions.
'''
from math import sqrt
from utils import coord2ij, cost
import numpy as np

def boundary_check(node, boundary_map):
    # node: a node
    # boundary_map: a 2D array
    # return: True if the node is in the boundary, False otherwise

    # Get the coordinates of the current node
    x1 = node.pos[0]
    y1 = node.pos[1]
    [i, j] = coord2ij(x1, y1, 0, 29990)
    if boundary_map[i][j] == 255 or x1 < 0 or x1 > 49990 or y1 < 0 or y1 > 29990:
        return False
    else:
        return True


def h_oct(x_cur, x_target):
    # x_cur: current node
    # x_target: target node

    x1 = x_cur.pos[0]
    y1 = x_cur.pos[1]
    x2 = x_target.pos[0]
    y2 = x_target.pos[1]

    dx = abs(x1 - x2)
    dy = abs(y1 - y2)

    return max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)

def reconstruct_path(cameFrom, current):
    # cameFrom: a list of nodes
    # current: the current node
    # return: path, a list of nodes

    total_path = [current]
    # traverse cameFrom from the end
    for prev_node in cameFrom.reverse():
        current = prev_node
        total_path.append(current)
    return total_path
 
def run(x_pos, x_target, boundary_map):
    # x_pos:
    # x_target:
    # return: path, cost
    # path: a list of nodes
    # cost: the cost of the path

    # Initialization
    open_set = [x_pos]
    
    cameFrom = []


    gScore = {}
    gScore[x_pos] = 0
    fScore = {}
    fScore[x_pos] = h_oct(x_pos, x_target)

    while open_set: # while open_set is not empty
        current = open_set.pop()
        if current == x_target:
            return reconstruct_path(cameFrom, current), gScore[current]
        # compute neighbors of current_node
        cur_neighbors = current.neighbors(10)
        for neighbor in cur_neighbors:
            ## Boundary check
            # Get the coordinates of the current node
            x1 = neighbor.pos[0]
            y1 = neighbor.pos[1]
            [i, j] = coord2ij(x1, y1, 0, 29990)
            if boundary_map[i][j] == 255:
                tentative_gScore = float('inf')
            else:
                tentative_gScore = gScore[current] + cost(current, neighbor)
            if neighbor not in gScore or tentative_gScore < gScore[neighbor]:
                cameFrom.append(current)
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + h_oct(neighbor,x_target)
                if neighbor not in open_set:
                    open_set.append(neighbor) 





class AStar:
    def __init__(self, x_pos, x_target, boundary_map):
        self.x_pos = x_pos
        self.x_target = x_target
        self.boundary_map = boundary_map
