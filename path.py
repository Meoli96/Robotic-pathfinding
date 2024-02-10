import numpy as np


class Path:
    def __init__(self, np_array: np.ndarray = None, plotArgs = None):
        self.pos = np_array
        self.n = len(np_array.shape)
  
        self.plotArgs = plotArgs


    def set_path(self, np_array: np.ndarray):
        self.path = np_array
        self.n = np_array.shape[0]

    def set_plotArgs(self, plotArgs):
        self.plotArgs = plotArgs