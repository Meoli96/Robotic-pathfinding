def coord2ij(x, y, x0, y0):
    # x, y: coordinates of a point we are interested in
    # x0, y0: coordinates of the lower left corner of the grid X(0,0), Y(0,0)
    # i, j: indices of the grid cell that contains the point (x,y)

    res = 10 # 10m resolution
    j =  (x - x0 + res)/res
    i = (y0 - y + res)/res
    return int(i), int(j)

import csv
def read_csv(file):
    matrix = []
    
    hFile = open(file)
    hCSVreader = csv.reader(hFile)
    for row in hCSVreader:
        matrix.append(row)
    return matrix



def grid_generator(Xlim, Ylim, res):
    # Xlim, Ylim: the limits of the grid
    # res: the resolution of the grid
    # return: X, Y, the grid

    # Initialization
    X = []
    Y = []
    x0 = Xlim[0]
    x1 = Xlim[1]
    y0 = Ylim[0]
    y1 = Ylim[1]

    # Generate the grid
    for i in range(x0, x1, res):
        for j in range(y0, y1, res):
            X.append(i)
            Y.append(j)
    return X, Y