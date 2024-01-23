from math import sqrt
import numpy as np
class Link:
    def __init__(self, coord1: tuple, coord2: tuple):
        assert coord1 != coord2, "Error: coord1 and coord2 should not be the same"
        self.tail = coord1
        self.head = coord2
        self.cost = sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

        d_node = tuple(np.array(coord2) - np.array(coord1))
        if d_node[0] == 0:
            # Selects the up and down directions
            if d_node[1] > 0:
                # Up
                self.direction = 0
            else:
                # Down
                self.direction = 4
        elif d_node[0] > 0:
            # Selects the right half of the circle
            if d_node[1] > 0:
                # Up right
                self.direction = 1
            elif d_node[1] == 0:
                # Right
                self.direction = 2
            else:
                # Down right
                self.direction = 3
        else:
            # Selects the left half of the circle
            if d_node[1] > 0:
                # Up left
                self.direction = 7
            elif d_node[1] == 0:
                # Left
                self.direction = 6
            else:
                # Down left
                self.direction = 5


class Graph:
    # This class generates a graph from a list of nodes (tuples)
    def __init__(self, nodes = None, grid = None):
        self.links = []
        self.directions = []
        self.n_links = 0
        self.cost = 0
        if nodes is not None:
            for i in range(len(nodes)-1):
                link = Link(nodes[i], nodes[i+1])
                self.add_link(link)
        self.grid = grid
        
        
        
    def set_links(self, links):
        self.links = links
        self.n_links = len(links)
        self.cost = sum([link.cost for link in self.links])
        self.diff_arr = [self.links[i].direction - self.links[i+1].direction 
                             for i in range(self.n_links - 1)]
    def add_link(self, link):
        self.links.append(link)
        self.n_links += 1
        self.cost += link.cost
        self.directions.append(link.direction)
    
    def get_cost(self, coord1 = None, coord2 = None):
        # Return the cost of the link between the given nodes
        # If no nodes are given, return the cost of the whole graph
        if coord1 == None and coord2 == None:
            return self.cost
        # else return the cost of the link between the two nodes
        for link in self.links:
            if (link.coord1 == coord1 and link.coord2 == coord2) or (link.coord1 == coord2 and link.coord2 == coord1):
                return link.cost
        return None
    def get_dir_diff(self):
        # Return the direction difference between the links
        return [abs(self.links[i].direction - self.links[i+1].direction) for i in range(self.n_links - 1)]
    

