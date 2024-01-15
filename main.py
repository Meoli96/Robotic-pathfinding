import scipy.io as sio
import numpy as np
import csv
import matplotlib.pyplot as plt
from graph import *
from utils import *

dataFolder = "data/"

### load data from csv file
# X, Y


[X, Y] = grid_generator([0, 4500], [0, 3000], 10) # 45km x 30km grid, 10m resolution [0,0] top left

# landmarks coordinates
Xlm = read_csv(dataFolder +"Xlm.csv")
Ylm = read_csv(dataFolder +"Ylm.csv")
assert len(Xlm) == len(Ylm)

# create list of landmark nodes
landmark_nodes = []
for i in range(len(Xlm)):
    landmark_nodes.append(Node(Xlm[i], Ylm[i]))


# Xvec, Yvec
Xvec = read_csv(dataFolder + "Xvec.csv")
Yvec = read_csv(dataFolder +"Yvec.csv")

# map and obstacle map
mars_map = read_csv(dataFolder +"map.csv")
obstacle_map = read_csv(dataFolder +"obstacle.csv")

## Constants of the problem

maxVel = 0.5; # [m/s] maximum velocity of the rover 
LAxis = 1; # [m] length of the rover wheel axis
###

### TASK 1 ###
''' TODO: 
        Capire come generare neighbors
        Generare path
        Passare dal path al controllo
'''
P0 = np.array([42.38, 11.59, 90])  # initial position, [km, km, deg]
P_target = np.array([33.07, 19.01, 180])  # target position, [km, km, deg]

plt.figure(1)
plt.imshow(mars_map, cmap='gray', origin='lower')



# 1.1) Plot the trajectory of the rover in the map
# 1.2) Plot the velocity profile of the rover
# 1.3) Plot the steering angle profile of the rover
# 1.4) Plot the rate of change of the steering angle profile of the rover

### TASK 2 ###