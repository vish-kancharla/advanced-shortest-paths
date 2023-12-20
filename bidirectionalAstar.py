from graph import Graph, Node
from queue import PriorityQueue
from utils import f


class BidirectionalAstar():

    def __init__(self, h, limit=None):
        """
        Parameters:
        - h: Heuristic function for Bidirectional AStar.
        - limit: Optional limit for the graph size.
        """
        self.h = h
        self.limit = limit

        
    def query(self, source, destination):
        """
        Find the optimal path between source and destination nodes using Bidirectional AStar.

        Returns:
        - List: The optimal path from source to destination, or None if no path is found.
        """
        forward_graph = Graph(source, destination)
        backward_graph = Graph(source, destination).reverse()
        
        forward_frontier = PriorityQueue()
        backward_frontier = PriorityQueue()

        forward_curr_node = Node(value=source)
        backward_curr_node = Node(value=destination)

        forward_frontier.put((f(self.h, forward_graph, forward_curr_node), forward_curr_node))
        backward_frontier.put((f(self.h, backward_graph, backward_curr_node), backward_curr_node))

        forward_reached = {source: forward_curr_node}
        backward_reached = {destination: backward_curr_node}

        while not forward_frontier.empty() and not backward_frontier.empty():
            
            forward_curr_node_cost, forward_curr_node = forward_frontier.get()
            backward_curr_node_cost, backward_curr_node = backward_frontier.get()
            sol = None
            
            #Either process forward or backward node whichever is less far from their desitnations
            if forward_curr_node_cost <= backward_curr_node_cost:
                backward_frontier.put((f(self.h, backward_graph, backward_curr_node), backward_curr_node))
                sol = self.proceed(forward_curr_node, forward_graph, forward_frontier, forward_reached, backward_reached, self.h)
                
            else:
                forward_frontier.put((f(self.h, forward_graph, forward_curr_node), forward_curr_node))
                sol = self.proceed( backward_curr_node, backward_graph, backward_frontier, backward_reached, forward_reached, self.h)

            if sol:
                #Return number of edges processed too along with solution
                return sol, len(forward_reached) + len(backward_reached)

        return None

    def proceed(self, current_node, graph, frontier, reached, reached2, h):
        """
        Expand the current node and update the frontier and reached nodes.

        Parameters:
        - current_node: Current node being expanded.
        - graph: Graph object representing the search space.
        - frontier: PriorityQueue for the current direction.
        - reached: Dictionary of reached nodes for the current direction.
        - reached2: Dictionary of reached nodes for the opposite direction.
        - h: Heuristic function.

        Returns:
        - List: The optimal path from source to destination, or None if no path is found.
        """
        for child in current_node.expand(graph):
            child_value = child.value
            child_path_cost = child.path_cost

            if child_value not in reached or child_path_cost < reached[child_value].path_cost:
                reached[child_value] = child
                frontier.put((f(h, graph, child), child))

            if child_value in reached2:
                #path found, reverse the backward solution and combine with the forward solution
                backward_path = list(reached2[child_value].solution())[:-1][::-1]
                forward_path = list(child.solution())
                return forward_path + backward_path

        return None
