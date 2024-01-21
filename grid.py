import numpy as np
class Grid:
    def __init__(self, *, xlim=10000, ylim=10000, corner = 'ul', res = 10, 
                 image: np.ndarray = None, obstacle: np.ndarray = None, landmarks:tuple = None):
        # Initialize the attributes of the Grid class
        # xlim: x limit of the grid
        # ylim: y limit of the grid
        # corner: the corner of the grid, 'ul' for upper left, 'll' for lower left
        # res: resolution of the grid
        # image: the image of the grid
        # obstacle: the obstacle of the grid

        # This grid doesnt store the actual grid, but the vectors X and Y 
        # used to retrieve the grid coordinates

        self.xlim = xlim
        self.ylim = ylim

        self.ilim = int(ylim/res)
        self.jlim = int(xlim/res)
        self.landmarks = landmarks


        self.corner = corner
        self.res = res
        self.image = image
        self.obstacle = obstacle
        
        # Initialize the grid
        self.X = tuple(np.arange(0, xlim, res))
        self.Y = tuple(np.arange(0, ylim, res))\
        
        if image is not None and obstacle is not None:
            # Build map from image superimposing obstacles for display purposes
            self.map = self.image.copy()
            for i in range(self.ilim):
                for j in range(self.jlim):
                    if self.obstacle[i][j] == 255:
                        self.map[i][j] = 255
        elif image is not None:
            self.map = self.image

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
        
    def boundary_check(self, x, y):
        return x >= 0 and x <= self.xlim and y >= 0 and y <= self.ylim # True if the point is in the boundary, False otherwise
    
    def obstacle_check(self, x, y):
        # x: x coordinate
        # y: y coordinate
        # return: True if the point is in the boundary, False otherwise

        # Get the i,j indices of the current point 
        try:
            i, j = self.xy2ij(x, y)
        except ValueError:
            return False
        # Check if the point is in the obstacle
        if self.obstacle is not None:
            return not self.obstacle[i][j] == 255	
        return True

    def neighbors(self, x, y, obstacle_check=True):
        # Generate a list of nodes neighbors of the x,y node, hoctal version

        # Generate inline nodes as tuples (x, y)
        n_up = (x, y + self.res)
        n_down = (x, y - self.res)
        n_left = (x - self.res, y)
        n_right = (x + self.res, y)

        # Generate diagonal nodes as tuples (x, y)
        n_up_left = (x - self.res, y + self.res)
        n_up_right = (x + self.res, y + self.res)
        n_down_left = (x - self.res, y - self.res)
        n_down_right = (x + self.res, y - self.res)

        # Add nodes to the list -- Anti-clockwise starting from up
        neighbors = [n_up, n_up_left, n_left, n_down_left, n_down, n_down_right, n_right, n_up_right]

        # Delete points outside of boundary and return
        if obstacle_check:
           return [n for n in neighbors if self.obstacle_check(n[0], n[1])]
        else:
            return [n for n in neighbors if self.boundary_check(n[0], n[1])]


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
        return (self.X[j], self.Y[i])
        

