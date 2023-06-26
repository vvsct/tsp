from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from itertools import permutations, combinations
from math import inf
import time


# calculate the total distance of a given permutation of cities
def total_distance(matrix, perm):
    return sum(matrix[perm[i - 1]][perm[i]] for i in range(len(perm)))


# brute force solution to the TSP
def travelling_salesman_brute_force(matrix):
    num_nodes = len(matrix)
    # generate all possible permutations of cities
    perms = permutations(range(num_nodes))
    # select the permutation with the minimum total distance
    perm = min(perms, key=lambda perm: total_distance(matrix, perm))
    # convert permutation to path and add starting city at the end
    path = [i + 1 for i in perm] + [1]
    # calculate total distance of path
    tot_dis = total_distance(matrix, [i - 1 for i in path[:-1]])
    return tot_dis, path


# dynamic programming solution to the TSP
def travelling_salesman_dynamic_programming(matrix):
    num_nodes = len(matrix)
    dp_table = {}
    for k in range(1, num_nodes):
        # initialise table with solutions to sub-problems
        dp_table[(1 << k, k)] = (matrix[0][k], [0, k])
    # iterate over all subset sizes
    for subset_size in range(2, num_nodes):
        # iterate over all subsets of given size
        for subset in combinations(range(1, num_nodes), subset_size):
            bits = 0
            for bit in subset:
                # convert subset to bitmask representation
                bits |= 1 << bit
            for k in subset:
                mask = ~(1 << k)
                # remove city k from subset
                prev = bits & mask
                path = []
                for m in subset:
                    if m == 0 or m == k:
                        continue
                    # calculate cost of going from city m to city k
                    # add it to the cost of visiting all cities in prev except m and k
                    path.append((dp_table[(prev, m)][0] + matrix[m][k], dp_table[(prev, m)][1] + [k]))
                # store minimum cost path in table
                dp_table[(bits, k)] = min(path)
    # bitmask representation of all cities except the starting city
    bits = (2 ** num_nodes - 1) - 1
    path = []
    for k in range(1, n):
        # calculate cost of going from city k to starting city
        # add it to the cost of visiting all cities except starting city and k
        path.append((dp_table[(bits, k)][0] + matrix[k][0], dp_table[(bits, k)][1] + [0]))
    # return minimum cost path
    return min(path)


# OR-Tools solution to the TSP
def travelling_salesman_ortools(matrix):
    n = len(matrix)
    # create routing index manager
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    # create routing model
    routing = pywrapcp.RoutingModel(manager)

    # distance callback function that returns the distance between two cities
    def distance_callback(i, j):
        node_i = manager.IndexToNode(i)
        node_j = manager.IndexToNode(j)
        return matrix[node_i][node_j]

    # register distance callback function
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    # set arc cost evaluator for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # create search parameters object
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # set the first solution strategy as PATH_CHEAPEST_ARC
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # solve problem using search parameters
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        # get start index of the first vehicle's route
        index = routing.Start(0)
        path = []
        # iterate over route until end is reached
        while not routing.IsEnd(index):
            # append current city to path
            path.append(manager.IndexToNode(index))
            # get next city in route
            index = solution.Value(routing.NextVar(index))
        # append end city to path
        path.append(manager.IndexToNode(index))
        # return total cost and path of solution
        return solution.ObjectiveValue(), path
    else:
        # return infinite cost and empty path if no solution is found
        return inf, []


if __name__ == "__main__":
    filenames = ['g4.txt', 'g13.txt', 'g13.txt', 'g13.txt', 'g13.txt']
    no = [4, 4, 9, 10, 11]

    print('Input\tPoints\tPath BF\tPath DP\tLength BF\tLength DP\tTimes BF\tTimes DP')
    for filename, num_points in zip(filenames, no):
        with open(filename, 'r') as file:
            n = int(file.readline())
            matrix = [list(map(int, line.split()))[:num_points] for line in file][:num_points]

        start_time = time.time()
        bf_length, bf_path = travelling_salesman_brute_force(matrix)
        bf_time = time.time() - start_time

        start_time = time.time()
        dp_length, dp_path = travelling_salesman_dynamic_programming(matrix)
        dp_time = time.time() - start_time

        dp_path = [i + 1 for i in dp_path]

        print(f'{filename}\t{num_points}\t{bf_path}\t{dp_path}\t{bf_length}\t{dp_length}\t{bf_time:.6f}\t{dp_time:.6f}')

    print('Input\tPoints\tPath ORT\tLength ORT\tTimes ORT')
    for filename, num_points in zip(filenames, no):
        with open(filename, 'r') as file:
            n = int(file.readline())
            matrix = [list(map(int, line.split()))[:num_points] for line in file][:num_points]
        start_time = time.time()
        ortools_length, ortools_path = travelling_salesman_ortools(matrix)
        ortools_time = time.time() - start_time
        ortools_path = [i + 1 for i in ortools_path]
        print(f'{filename}\t{num_points}\t{ortools_path}\t{ortools_length}\t{ortools_time}')
