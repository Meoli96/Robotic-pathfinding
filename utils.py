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

    angle = angle_mod_2pi(np.arctan2(landmark[1] - pos[1], landmark[0] - pos[0]) - pos[2])

    return np.array([dist, angle])  

from matplotlib.patches import Ellipse

def error_ellipse(xc, yc, cov, sigma=1, **kwargs):
    '''
    Plot an error ellipse contour over your data.
    Inputs:
    ax : matplotlib Axes() object
    xc : x-coordinate of ellipse center
    yc : x-coordinate of ellipse center
    cov : covariance matrix
    sigma : # sigma to plot (default 1)
    additional kwargs passed to matplotlib.patches.Ellipse()
    '''
    w, v = np.linalg.eigh(cov) # assumes symmetric matrix
    order = w.argsort()[::-1]
    w, v = w[order], v[:,order]
    theta = np.degrees(np.arctan2(*v[:,0][::-1]))
    ellipse = Ellipse(xy=(xc,yc),
                    width=2.*sigma*np.sqrt(w[0]),
                    height=2.*sigma*np.sqrt(w[1]),
                    angle=theta, **kwargs)
    ellipse.set_facecolor('none')
    return ellipse

    




### Python notebook utils functions
    
def plot_angle_and_rate(q, qd):
    theta_plot =  list_np_angle(q[:,2])
    theta_dot_plot = np.unwrap(qd[:,2])
    
    plt.figure()
    plt.subplot(2,1,1)
    plt.ylabel(r'$\theta$' + "°")
    plt.plot(theta_plot, 'b-')
    plt.subplot(2,1,2)
    plt.ylabel(r'$\dot{\theta}$' + " rad/s")
    plt.xlabel("t (s)")
    plt.plot(theta_dot_plot, 'r-')
    plt.show()

    
def plot_velocity(qd):
    # Plot the velocity of the robot
    # plot velocity module
    plt.figure()
    plt.plot(np.sqrt(qd[:,0]**2 + qd[:,1]**2))
    plt.ylabel(r'$v$' + " m/s")
    plt.xlabel("t (s)")  
    # plot velocity components
    plt.figure()
    plt.subplot(2,1,1)
    plt.ylabel(r'$v_x$' + " m/s")
    plt.plot(qd[:,0], 'b-')
    plt.subplot(2,1,2)
    plt.ylabel(r'$v_y$' + " m/s")
    plt.xlabel("t (s)")
    plt.plot(qd[:,1], 'r-')


def astar_md():
    # Print A* algorithm definition in markdown
    mdstr = """
```
def AStar(grid, x0, x_target):
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
```
"""
    return mdstr

def test_md():
    mdstr="""
Hello, this should just be a paragraph, and there should be a test of Markdown fenced code block with backticks:

```

Hello there
This should be in
a code block
```
"""
    return mdstr