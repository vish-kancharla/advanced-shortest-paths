from utils import load_file_into_dict
import constants


class Node:

    def __init__(self, value, parent=None, path_cost=0, num_nodes_processed=0):
        """
        Initializes a Node object with a given value, parent node, and path cost.

        Parameters:
        - value: Value associated with the node.
        - parent: The parent node of the current node.
        - path_cost: Cost of the path from the source node to this nodes.
        - num_nodes_processed: How many nodes were processed to reach this node?
        """
        self.value = value
        self.parent = parent
        self.path_cost = path_cost
        self.num_nodes_processed = num_nodes_processed

    def expand(self, graph):
  
        return (self.child_node(graph, neighbor)
                for neighbor in graph.actions(self.value))

    def child_node(self, graph, neighbor_value):
        
        next_node = Node(neighbor_value, self, self.path_cost + graph.path_cost(self.value, neighbor_value))
        return next_node

    def solution(self):
        """
        Retrieves the solution path from the current node to the source.
        """
        curr_node = self
        path = []
        while curr_node:
            path.append(curr_node.value)
            curr_node = curr_node.parent
        path = reversed(path)
        return tuple(path)

    def __lt__(self, other):
        """
        Defines the less-than comparison between two nodes based on their values.
        Useful when same values are present in the priority queue, we choose the node with 
        smallest value in those cases. 
        """
        return self.value < other.value




class Graph:
    
    def __init__(self, source, destination, limit=None):
        """
        Intialize a Graph object with source, destination, and limit.
        Load coordinates and edges from data files into dictionaries.

        Parameters:
        - source: Starting node of the graph.
        - destination: Goal node of the graph.
        - limit: Optional parameter to limit the number of numbers loaded from files.
        """
        self.source = source
        self.destination = destination
        self.coordinates = load_file_into_dict(constants.COORDINATE_MAP, "v", limit)
        self.distances = load_file_into_dict(constants.DISTANCE_MAP, "a", limit)
        self.limit = limit
        
    def actions(self, node_value):
    
        return list(self.distances[node_value].keys())

    
    def is_goal(self, node_value):
        
        return node_value == self.destination
    
    def path_cost(self, node_value, neighbor_value):
        """
        Returns the edge cost from a node to its neighbor.
        """
        return self.distances[node_value][neighbor_value]
    
    def reverse(self):
        """
        Creates a reversed graph. Useful for bidirectional algorithms

        Returns:
        - Reversed graph object with source, destination and edges swapped.
        """
        r_graph = Graph(self.destination, self.source, self.limit)
        reversed_distances = {}
        for key1, inner_dict in self.distances.items():
            for key2, value in inner_dict.items():
                if key2 not in reversed_distances:
                    reversed_distances[key2] = {}
                reversed_distances[key2][key1] = value
                    
        r_graph.distances = reversed_distances
        return r_graph
