import numpy as np
from graph import *

def shorten_path(graph: Graph, theta0, theta_target):
    # path: a graph of nodes, solution of the A* algorithm
    # pose: the current pose of the robot
    changes = np.diff(graph.directions)
    
    # Create a new graph with less links, removing consecutive links with the same direction
    new_graph = Graph(None, graph.grid)

    start_idx = 0
    end_idx = 0

    for data in changes:
        if data != 0:
            # Add the link to the new graph
            node_start = graph.links[start_idx].head
            node_end = graph.links[end_idx].tail
            new_graph.add_link(Link(node_start, node_end))
            start_idx = end_idx
        end_idx += 1
    return new_graph
        

            

        


