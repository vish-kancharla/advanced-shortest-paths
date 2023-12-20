import constants
from math import inf, radians, cos, sin, asin, sqrt, dist


def load_file_into_dict(file_path, delimiter, limit=None):
    """ 
    Loads dimacs data from a file into a dictionary.

    Parameters:
        - file_path: Path to the file.
        - delimiter: Character that separates actual data from comments and metadata in the file.
        - limit: Optional parameter to limit the number of numbers loaded from files.
    Returns:
        - A dictionary containing the data from the file.
    """

    result = {}
    with open(file_path) as file:
        for line in file:
            if line.startswith(delimiter):
                
                # Handle coordinates data
                if delimiter == "v": 
                    key, lat, long = map(int, line.split()[1:])
                    if not limit or key <= limit:
                        result[key] = {"lat": lat, "long": long}
                
                # Handle distance data
                elif delimiter == "a":
                    source, destination, cost = map(int, line.split()[1:])
                    if not limit or (source  <= limit and destination <= limit) :
                        result.setdefault(source, {})[destination] = cost
    
    # Ensure all nodes up to the limit are present in the result dictionary
    #To be useful in preprocessing
    if limit:
        for i in range(1,limit+1):
            if i not in result:
                result.setdefault(i, {})
    return result




def haversine(source, destination):
    """
    Calculate the great circle distance between two points on the earth
    To be used as a heuristic for Astar algorithms.
    """    
    
    # Extracting latitude and longitude values from the dictionaries
    # Divide by constants.M to convert lats and longs into proper convention
    lat1, lon1 = source['lat'] / constants.M, source['long'] / constants.M
    lat2, lon2 = destination['lat'] / constants.M, destination['long'] / constants.M

    # Convert decimals to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    lon_diff = lon2 - lon1
    lat_diff = lat2 - lat1 
    
    
    a = sin(lat_diff / 2) ** 2 + cos(lat1) * cos(lat2) * sin(lon_diff / 2) ** 2
   
    c = 2 * asin(sqrt(a)) 
    
    return c * constants.RADIUS



def landmark_heuristic(landmarks, landmarks_dict, vertex_dict, node, destination):
    """
    Heuristic value for landmarks  A* search algorithm.

    Parameters:
        - landmarks: List of landmarks.
        - landmarks_dict: Dictionary containing distances from all landmark to every vertex  distances.
        - vertex_dict: Dictionary containing distances from all vertex to every landmark.
        - node: The current node in the search.
        - destination: The destination node.
    """
    min_distance = inf / constants.M

    for landmark in landmarks:
        if landmark in vertex_dict[node.value] and destination in landmarks_dict[landmark]:
            distance = abs(vertex_dict[node.value][landmark] - landmarks_dict[landmark][destination])
            min_distance = min(min_distance, distance)

    return min_distance

def euclidean(source, destination):
    return dist([source['lat'], source['long']], [destination['lat'], destination['long']])


def f(h, graph, node):
    """
    Total cost function for A* search algorithm.
    """
    return h(graph.coordinates[node.value], graph.coordinates[graph.destination]) + node.path_cost


def f2(h, landmarks, landmarks_dict, vertex_dict, node, destination):
    """
    Total cost function for landmarks A* search algorithm.
    """
    return h(landmarks, landmarks_dict, vertex_dict, node, destination) + node.path_cost
