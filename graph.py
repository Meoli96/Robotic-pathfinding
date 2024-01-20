from math import sqrt
class Node:
    def __init__(self, pos, orientation = 0):
        self.pos = pos

    def __eq__(self, other):
        # Devo aggiungere il fatto che ho una risoluzione?
        # Quindi è uguale a se stesso piu o meno la risoluzione 
        resolution = 10  # Define your resolution
        return abs(self.pos[0] - other.pos[0]) <= resolution and abs(self.pos[1] - other.pos[1]) <= resolution
    
    def __hash__(self):
        return hash((self.pos[0], self.pos[1]))
    
    def neighbors(self, res):
        # Generate a list of nodes neighbors of the current node, hoctal version
        # res: resolution of the map

        cur_pos = self.pos

        # Generate inline nodes
        n_up = Node([cur_pos[0], cur_pos[1] + res])
        n_down = Node([cur_pos[0], cur_pos[1] - res])
        n_left = Node([cur_pos[0] - res, cur_pos[1]])
        n_right = Node([cur_pos[0] + res, cur_pos[1]])

        # Generate diagonal nodes
        n_up_left = Node([cur_pos[0] - res, cur_pos[1] + res])
        n_up_right = Node([cur_pos[0] + res, cur_pos[1] + res])
        n_down_left = Node([cur_pos[0] - res, cur_pos[1] - res])
        n_down_right = Node([cur_pos[0] + res, cur_pos[1] - res])

        # Add nodes to the list
        # Remember the order you're returning: clockwise starting from up
        return [n_up, n_up_right, n_right, n_down_right, n_down, n_down_left, n_left, n_up_left]

class Link:
    def __init__(self, node1, node2):
        self.node1 = node1
        self.node2 = node2
        self.cost = sqrt((node1.pos[0] - node2.pos[0])**2 + (node1.pos[1] - node2.pos[1])**2)


class Graph:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links

    def add_node(self, node):
        self.nodes.append(node)

    def add_link(self, link):
        self.links.append(link)

    def get_node(self, pos):
        # Return the node with the given position
        for node in self.nodes:
            if node.pos == pos:
                return node
        return None

    def get_link(self, node1, node2):
        # Return the link between the given nodes
        for link in self.links:
            if (link.node1 == node1 and link.node2 == node2) or (link.node1 == node2 and link.node2 == node1):
                return link
        return None

    def get_neighbors(self, node):
        # Return the neighbors of the given node
        neighbors = []
        for link in self.links:
            if link.node1 == node:
                neighbors.append(link.node2)
            elif link.node2 == node:
                neighbors.append(link.node1)
        return neighbors

    def get_cost(self, node1, node2):
        # Return the cost of the link between the given nodes
        for link in self.links:
            if (link.node1 == node1 and link.node2 == node2) or (link.node1 == node2 and link.node2 == node1):
                return link.cost
        return None

 