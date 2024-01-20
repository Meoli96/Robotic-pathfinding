import numpy as np
class Grid:
    def __init__(self, *, xlim=10000, ylim=10000, corner = 'ul', res = 10, 
                 image = None, obstacle = None):
        # Initialize the attributes of the Grid class
        # xlim: x limit of the grid
        # ylim: y limit of the grid
        # corner: the corner of the grid, 'ul' for upper left, 'll' for lower left
        # res: resolution of the grid
        # image: the image of the grid
        # obstacle: the obstacle of the grid
        self.xlim = xlim
        self.ylim = ylim

        self.ilim = int(xlim/res)
        self.jlim = int(ylim/res)


        self.corner = corner
        self.res = res
        self.image = image
        self.obstacle = obstacle
        
        # Initialize the grid
        self.X = np.arange(0, xlim, res)
        self.Y = np.arange(0, ylim, res)

        

        if corner == 'ul':
            # Do nothing, we're good to go
            pass
        elif corner == 'll':
            # Reverse the Y axis
            self.Y = self.Y[::-1]
        elif corner == 'ur':
            # Reverse the X axis
            self.X = self.X[::-1]
        elif corner =='lr':
            # Reverse both X and Y axes
            self.X = self.X[::-1]
            self.Y = self.Y[::-1]
        else:
            raise ValueError('Invalid corner value')
        
    def boundary_check(self, x, y, obstacle_check = True):
        # x: x coordinate
        # y: y coordinate
        # return: True if the point is in the boundary, False otherwise

        # check grid boundary
       b_outside = (x < 0 or x > self.xlim or y < 0 or y > self.ylim) # True if the point is outside the grid
       if self.obstacle is not None and not b_outside and obstacle_check:
           b_obstacle = self.obstacle[y, x] == 255
       else: 
            b_obstacle = False
       return not b_outside and not b_obstacle # True if the point is inside the grid and not an obstacle



    def xy2ij(self, x, y):
        # x: x coordinate
        # y: y coordinate
        # return: i, j, the indices of the grid
        # This function is going to retrieve negative indices if the point is 
        if not self.boundary_check(x, y):
            raise ValueError('Point out of boundary')
        i = int((self.Y[0] - y)/self.res)
        j = int((x - self.X[0])/self.res)
        return i, j
    
    def __getitem__(self, index): # i,j to x,y
        # index is a tuple (i, j)
        i, j = index
        if i < 0 or i >= self.ilim or j < 0 or j >= self.jlim:
            raise ValueError('Index out of range')
        return self.X[i], self.Y[j]
        

