import numpy as np
import matplotlib.pyplot as plt
class Grid:
    def __init__(self, *, xlim=0, ylim=0, corner = 'ul', res = 1, 
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
        self.Y = tuple(np.arange(0, ylim, res))

        self.x_off = 0
        self.y_off = 0
        
        self.map = image
        self.obstacle = obstacle

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
    
    def boundary_check_path(self, path):
        # Return True if all the points in the path are in the boundary, False otherwise
        return all([self.boundary_check(x, y) for x, y in path])
    
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

    
    
    def get_submap(self, x_start, x_end, y_start, y_end):
        # Check if the points are inside the boundary
        if not self.boundary_check(x_start, y_start) or not self.boundary_check(x_end, y_end):
            # Who is the offender?
            if not self.boundary_check(x_start, y_start):
                # Again, who is the offender?
                if x_start < 0:
                    x_start = 0
                if x_start > self.xlim:
                    x_start = self.xlim
                if y_start < 0:
                    y_start = 0
                if y_start > self.ylim:
                    y_start = self.ylim
            
            if not self.boundary_check(x_end, y_end):
                # Again, who is the offender?
                if x_end < 0:
                    x_end = 0
                if x_end > self.xlim:
                    x_end = self.xlim
                if y_end < 0:
                    y_end = 0
                if y_end > self.ylim:
                    y_end = self.ylim
        # Now we are sure that the points are inside the boundary
        # Get the indices of the points
        i_start, j_start = self.xy2ij(x_start, y_start)
        i_end, j_end = self.xy2ij(x_end, y_end)
        # Check if the points are in the right order\
        if i_start > i_end:
            i_start, i_end = i_end, i_start
        if j_start > j_end:
            j_start, j_end = j_end, j_start
        sub_image = self.image[ i_start:i_end, j_start:j_end]
        sub_obst = self.obstacle[ i_start:i_end, j_start:j_end]
        # get grid landmarks within the submap
        sub_landmarks = []
        for landmark in self.landmarks:
            if landmark[0] >= x_start and landmark[0] <= x_end and landmark[1] >= y_start and landmark[1] <= y_end:
                sub_landmarks.append(landmark)
        
        subgrid = Grid(xlim = x_end - x_start, ylim = y_end - y_start, corner = self.corner,landmarks=sub_landmarks, res = self.res, image = sub_image, obstacle = sub_obst)
        subgrid.x_off = x_start
        subgrid.y_off = y_start
        return subgrid
    
    def submap_path(self, path_list):
        # path_list: a list of paths (array of points)
        # return: a submap containing all the paths

        # Be sure that all paths are within the boundary of the main map
        for path in path_list:
            if not self.boundary_check_path(path):
                raise ValueError('Path out of boundary')
        
        # Now compute x_start, x_end, y_start, y_end from every path
            x_start = 0 
            x_end = self.x_end
            y_start = 0
            y_end = self.y_end
        # I need:
            # Minimum x coordinate from all paths
            # Maximum x coordinate from all paths
            # Minimum y coordinate from all paths
            # Maximum y coordinate from all paths
        x_start = min([min(path[:,0]) for path in path_list])
        x_end = max([max(path[:,0]) for path in path_list])
        y_start = min([min(path[:,1]) for path in path_list])
        y_end = max([max(path[:,1]) for path in path_list])
        
       
        return self.get_submap(x_start, x_end, y_start, y_end)

          
    def plot(self):
        # Plot the grid
        plt.imshow(np.flipud(self.map), cmap='gray')
        plt.gca().invert_yaxis()
        plt.gca().set_aspect('equal')
        # We are displaying an image, remove the axis ticks
        # Set the x and y ticks
        plt.xticks([])
        plt.yticks([])

    def plot_on(self, q, *args, submap = False, landmarks = False):

        # Plot the array on the grid
        # If submap is True, plot the array on a submap containing q
        # If landmarks is True, plot the landmarks
        
        # q can be:
        # - a point (x, y) -- list of points
        # - a path (array of points) -- list of paths
       
        


        # Plot the array
        plt.plot((q[0,:]-self.x_off)/self.res, (q[1,:]-self.y_off)/self.res, *args)

      




    def xy2ij(self, x, y):
        # x: x coordinate
        # y: y coordinate
        # return: i, j, the indices of the grid
        # 
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
    
    # This method should be part of a Trajectory class or Rover class,
    # but it's here for now
    def find_landmark(self, q, range_max):
        # q: robot pose
        # range_max: maximum range of the lidar
        # return: a list of the coordinates of the landmarks in the map
        r_landmarks = []
        
        r_dt = [] 

        for landmark in self.landmarks:
            # Check if the landmark is in the range of the sensor
            dist = np.linalg.norm(q[:2] - landmark)
            # compute the angle between the robot and the landmark
           
            if dist < range_max:
                r_landmarks.append(landmark)
        return r_landmarks



    



