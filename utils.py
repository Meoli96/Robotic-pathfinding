import numpy as np

def coord2ij(x, y, x0, y0):
    # x, y: coordinates of a point we are interested in
    # x0, y0: coordinates of the lower left corner of the grid X(0,0), Y(0,0)
    # i, j: indices of the grid cell that contains the point (x,y)

    res = 10 # 10m resolution
    j =  (x - x0)/res
    i = (y0 - y)/res
    return [int(i), int(j)]
import csv
def read_csv(file):
    matrix = []
    
    hFile = open(file)
    hCSVreader = csv.reader(hFile)
    for row in hCSVreader:
        matrix.append(row)
    return matrix

def cost(node1, node2):
    return np.sqrt((node1.pos[0] - node2.pos[0])**2 + (node1.pos[1] - node2.pos[1])**2)