import random
import pickle 
from astar import AStar
from graph import Node, Graph
from queue import PriorityQueue
from utils import f2, haversine


class LandmarkAstar():

    def __init__(self, h1, h2, num_landmarks=10, limit=None, cache=False):
        """
        Initialize the LandmarkAstar object with  heuristic functions, number of landmarks, and a limit.

        Parameters:
        - h1: AStar heuristic function.
        - h2: Landmark AStar heuristic function.
        - num_landmarks: Number of landmarks to be generated.
        - limit: Optional limit for the graph size.
        - cache: Boolean indicating whether to do the preprocessing or use old data.
        """
        self.h1 = h1
        self.h2 = h2
        self.limit = limit
        self.num_landmarks = num_landmarks
        
        if cache:
            # Load distances from landmarks to all vertices dict and
            # distances from all vertices to landmarks dict from a cached file
            with open('landmarks.pkl', 'rb') as pickle_file:
                dict_list = pickle.load(pickle_file)
            
            self.landmarks = dict_list[0]
            self.landmark_dict = dict_list[1]
            self.vertex_dict = dict_list[2]
        else:
            # If caching is not enabled, generate the data
            self.preprocess(self.num_landmarks)
    
        
    def preprocess(self, num_landmarks):
        """
        Generate landmarks and calculate distances between landmarks and vertices.

        """
        # Generate random landmarks
        landmarks = []
        for _ in range(num_landmarks):
            landmarks.append(random.randrange(1, self.limit + 1))
        
        
        astar = AStar(haversine, self.limit)
        
        landmark_dict = {}
        vertex_dict = {}
        
        for landmark in landmarks:
            landmark_dict[landmark] = {}
            for vertex in range(1, self.limit + 1):
                # Query AStar for distance from landmark to vertex
                d = astar.query(landmark, vertex)
                if d:
                    landmark_dict[landmark][vertex] = d.path_cost 
        
        for vertex in range(1, self.limit + 1):
            vertex_dict[vertex] = {}
            for landmark in landmarks:
                # Query AStar for distance from vertex to landmark
                d = astar.query(vertex, landmark)
                if d:
                    vertex_dict[vertex][landmark] = d.path_cost
        
        self.landmark_dict = landmark_dict
        self.vertex_dict = vertex_dict
        self.landmarks = landmarks
        
        dict_list = [landmarks, landmark_dict, vertex_dict]
        
        # Dump the data into a file for future use
        with open('landmarks.pkl', 'wb') as pickle_file:
            pickle.dump(dict_list, pickle_file)

    def query(self, source, destination):
        """
        Find the optimal path between source and destination nodes using landmark heuristics.

        Returns:
        - Node: The final node in the optimal path.
        """
        partial_graph = Graph(source, destination, self.limit)
        start_node = Node(value=source)
        frontier = PriorityQueue()
        # Use landmark heuristics f2 and h2
        frontier.put((f2(self.h2, self.landmarks, self.landmark_dict, self.vertex_dict, start_node, destination), start_node))

        reached = {source: start_node}
        while not frontier.empty():
            current_node = frontier.get()[1]
            if partial_graph.is_goal(current_node.value):
                return current_node

            for child in current_node.expand(partial_graph):
                child_value = child.value
                child_path_cost = child.path_cost

                if child_value not in reached or child_path_cost < reached[child_value].path_cost:
                    reached[child_value] = child
                    frontier.put((f2(self.h2, self.landmarks, self.landmark_dict, self.vertex_dict, child, destination), child))
        return None


        


