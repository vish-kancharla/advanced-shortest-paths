from graph import Graph, Node
from queue import PriorityQueue
from utils import f

class AStar():
    
    def __init__(self, h, limit=None):
        """
        Parameters:
        - h: Heuristic function for AStar.
        - limit: Optional limit for the graph size.
        """
        self.h = h
        self.limit = limit 
        
    def query(self, source, destination):
        """
        Find the optimal path between source and destination nodes using AStar.

        Returns:
        - Node: The destination node, using which we can retrace the path.
        """
        graph = Graph(source, destination, self.limit)
        start_node = Node(value=source)
        
        frontier = PriorityQueue()
        frontier.put((f(self.h, graph, start_node), start_node))

        reached = {source: start_node}
        while not frontier.empty():
            current_node = frontier.get()[1]
            if graph.is_goal(current_node.value):
                current_node.num_nodes_processed = len(reached)
                return current_node

            for child in current_node.expand(graph):
                child_value = child.value
                child_path_cost = child.path_cost

                if child_value not in reached or child_path_cost < reached[child_value].path_cost:
                    reached[child_value] = child
                    frontier.put((f(self.h, graph, child), child))

        return None
