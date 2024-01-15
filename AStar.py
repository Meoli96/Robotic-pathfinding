''' This file contains the implementation of the Astar algorithm, and all
    auxiliary functions.
'''
from math import sqrt

def boundary_check(node1, node2, obstacle_map):
    # node1: the first node
    # node2: the second node
    # obstacle_map: the obstacle map
    # return: True if the link is valid, False otherwise

    # Check value of the obstacle map at i, j computed with coord2ij
    a = 1


def h_oct(x_cur, x_target):
    # x_cur: current position
    # x_target: target position

    x1 = x_cur[0]
    y1 = x_cur[1]
    x2 = x_target[0]
    y2 = x_target[1]

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
 
def Astar(x_pos, x_target):
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
    fScore[x_pos] = h_oct(x_pos)

    while not open_set.empty():
        current = open_set.pop()
        if current == x_target:
            return reconstruct_path(cameFrom, current), gScore[current]
        for neighbor in current.neighbors:
            tentative_gScore = gScore[current] + neighbor.G()
            if neighbor not in gScore or tentative_gScore < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + h_oct(neighbor)
                if neighbor not in open_set:
                    open_set.append(neighbor) 








