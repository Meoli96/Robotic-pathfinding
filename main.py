import scipy.io as sio
import numpy as np
import csv
from utils import *


### load data from csv file
# X, Y
X = read_csv("X.csv")
Y = read_csv("Y.csv")

# landmarks nodes
Xlm = read_csv("Xlm.csv")
Ylm = read_csv("Ylm.csv")

# Xvec, Yvec
Xvec = read_csv("Xvec.csv")
Yvec = read_csv("Yvec.csv")

# map and obstacle map
mars_map = read_csv("map.csv")
obstacle_map = read_csv("obstacle.csv")



