import numpy as np


class Path:
    def __init__(self, np_state: np.ndarray = None, np_state_un: np.ndarray = None,  plotArgs = None):
        self.pos = np_state
        self.pos_un = np_state_un
        self.n = len(np_state.shape)
        self.len = len(np_state)
  
        self.plotArgs = plotArgs


    def set_path(self, np_state: np.ndarray, np_state_un: np.ndarray):
        self.path = np_state
        self.n = len(np_state.shape)
        self.len = len(np_state)
        self.path_un = np_state_un


    def set_plotArgs(self, plotArgs):
        self.plotArgs = plotArgs