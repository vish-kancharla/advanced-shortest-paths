
import matplotlib.pyplot as plt
import time
import random
from astar import AStar
from bidirectionalAstar import BidirectionalAstar
from landmarkAstar import LandmarkAstar
from utils import haversine, euclidean, landmark_heuristic

def run_astar_comparison(number_of_vertices, num_runs=5):
    astar_euclidean = AStar(euclidean)
    astar_haversine = AStar(haversine)

    astar_euclidean_times = []
    astar_haversine_times = []

    for _ in range(num_runs):
        source = random.randrange(1, number_of_vertices)
        destination = random.randrange(1, number_of_vertices)

        euclidean_time = benchmark_astar_time(astar_euclidean, source, destination)
        haversine_time = benchmark_astar_time(astar_haversine, source, destination)

        astar_euclidean_times.append(euclidean_time)
        astar_haversine_times.append(haversine_time)

    plot_comparison(astar_euclidean_times, astar_haversine_times, 'Comparison of heuristics', 'Euclidean', 'Haversine', 'Time taken (seconds)')

def benchmark_astar_time(astar_instance, source, destination):
    start_time = time.time()
    astar_instance.query(source, destination)
    end_time = time.time()
    return (end_time - start_time)

def plot_comparison(data1, data2, title, data1_label, data2_label, ylabel):
    indices = range(1, len(data1) + 1)
    bar_width = 0.35

    plt.bar(indices, data1, width=bar_width, label=data1_label)
    plt.bar([i + bar_width for i in indices], data2, width=bar_width, label=data2_label)

    plt.xlabel('Run #')
    plt.ylabel(ylabel)
    plt.title(title)
    plt.xticks([i + bar_width / 2 for i in indices], indices)
    plt.legend()
    plt.tight_layout()

    plt.savefig(f'{title.replace(" ", "_").lower()}_plot.png')

    plt.show()

def run_bidirectional_astar_comparison(number_of_vertices, num_runs=5):
    bstar = BidirectionalAstar(haversine)
    astar_haversine = AStar(haversine)
    astar_nodes = []
    bidirectional_astar_nodes = []

    for _ in range(num_runs):
        source = random.randrange(1, number_of_vertices)
        destination = random.randrange(1, number_of_vertices)

        astar_node_count =  astar_haversine.query(source, destination).num_nodes_processed
        bidirectional_astar_node_count = bstar.query(source, destination)[1]

        astar_nodes.append(astar_node_count)
        bidirectional_astar_nodes.append(bidirectional_astar_node_count)

    plot_comparison(astar_nodes, bidirectional_astar_nodes, 'Comparison of Astar and BidirectionalAstar', 'AStar', 'BidirectionalAstar', 'Nodes processed')


def run_landmark_astar_comparison():
    landmark_astar_times = []
    astar_times = []
    landmark_astar = LandmarkAstar(haversine, landmark_heuristic, 10, 300, True)
    landmark_astar_times.append(benchmark_astar_time(landmark_astar, 1, 13))
    astar = AStar(haversine, 300)
    astar_times.append(benchmark_astar_time(astar, 1, 13))        
    landmark_astar_times.append(benchmark_astar_time(landmark_astar, 220,221))
    astar_times.append(benchmark_astar_time(astar, 220, 221))        
    print("In two runs astar took : " + str(astar_times) + " seconds")
    print("In two runs landmark astar took : " + str(landmark_astar_times) + " seconds")
    
def main():
    number_of_vertices = 264346
    run_astar_comparison(number_of_vertices)
    run_bidirectional_astar_comparison(number_of_vertices)
    run_landmark_astar_comparison()
    
    #  please uncomment below lines to do see output of all algorithms 
    print("Sample test run for all algorithms: ")
    source =  58003
    destination =  23996
    print("Running algorithms with source: {}, and destination: {}".format(source, destination))
    print("Path found by Astar:")
    astar_haversine = AStar(haversine)
    print(astar_haversine.query(source, destination).solution())
    print("Path found by Bidirectional Astar:")
    bstar = BidirectionalAstar(haversine)
    print(bstar.query(source,destination)[0])
    
    print("Running Landmark Astar between 1 and 13: ")
    landmark_astar = LandmarkAstar(haversine, landmark_heuristic, 10, 300, True)
    print(landmark_astar.query(1,13).solution())

if __name__ == "__main__":
    main()



