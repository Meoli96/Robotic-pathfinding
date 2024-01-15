from math import sqrt
class Node:
    def __init__(self, pos):
        self.pos = pos
        self.visited = False

    def __getitem__(self, idx):
        return self.pos[idx]
    def __setitem__(self, idx, val):
        self.pos[idx] = val

    def __eq__(self, other):
        # Devo aggiungere il fatto che ho una risoluzione?
        # Quindi è uguale a se stesso piu o meno la risoluzione 
        return self.pos == other.pos
class Link:
    def __init__(self, node1, node2):
        self.node1 = node1
        self.node2 = node2
    def G(self):
        # Cost function of the link -- distance
        return sqrt((self.node1.pos[0] - self.node2.pos[0])**2 + (self.node1.pos[1] - self.node2.pos[1])**2)    
    