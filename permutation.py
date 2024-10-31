#finds all permutations (steps) of the graph given using SJT
import math


def find_permutations(graph):
    #basically performing SJT and appending to an array
    n = len(graph) #number of nodes

    permutation = list(range(n))  #first permutation
    permutations = []

    #check if the element can move in the direction it points at
    def can_move(index):
        if permutation[index] > 0:  #moving right if its positive
            return index + 1 < n and permutation[index + 1] > permutation[index]
        else:  #moving left if its negative
            return index - 1 >= 0 and permutation[index - 1] > permutation[index]

    #move the element in the direction it points at
    def move(index):
        if permutation[index] > 0:  #moving right if its positive
            permutation[index], permutation[index + 1] = permutation[index + 1], permutation[index]
        else:  #moving left if its negative
            permutation[index], permutation[index - 1] = permutation[index - 1], permutation[index]

    permutations.append(permutation[:])

    while True:
        #find the largest mobile integer
        largest_mobile_index = -1
        largest_mobile_value = -1

        for i in range(n):
            if can_move(i) and permutation[i] > largest_mobile_value:
                largest_mobile_value = permutation[i]
                largest_mobile_index = i

        if largest_mobile_index == -1:
            break  #no mobile element

        #move the largest mobile element
        move(largest_mobile_index)

        #change direction of elements larger than the largest mobile element
        for i in range(n):
            if permutation[i] > largest_mobile_value:
                permutation[i] = -permutation[i]

        #add permutation to the list
        permutations.append(permutation[:])

    return permutations

#is there a hamiltonian graph in the permutations param?
def validate_hamiltonian_cycle(graph, permutations):
    isFound = False
    hamiltonian_cycles = []
    for perm in permutations:
        is_cycle = True

        #loops through each number in the permutation
        for i in range(len(perm) - 1):
            current_node = perm[i]
            next_node = perm[i + 1]

            #is the next node in the adjacency list of the current node?
            if next_node not in graph[current_node][1]:
                is_cycle = False
                break

        #returns true if a hamiltonian cycle is found
        if is_cycle: #this loop is not being entered!!!
            isFound = True
            hamiltonian_cycles.append(perm)

        #returns false if no hamiltonian cycle is found
    return isFound, hamiltonian_cycles

#bonus: indicate which hamiltonian cycles are optimal in terms of overall distance
def optimal_cycle(permutations, graph):
    cycle_weights = []
    optimal_distance = float('inf')  #starting with infinity as we want to minimize
    optimal_permutation = None

    #iterate through each permutation
    for perm in permutations:
        total_distance = 0

        #find total distance
        for i in range(len(perm)):
            current_node_index = perm[i]
            next_node_index = perm[(i + 1) % len(perm)]  #complete the graph

            #getting coordinates
            current_coords = graph[current_node_index][0]
            next_coords = graph[next_node_index][0]

            #get euclidean distance
            distance = math.sqrt((next_coords[0] - current_coords[0]) ** 2 +
                                 (next_coords[1] - current_coords[1]) ** 2)
            total_distance += distance

        #store it if its less than what we already have
        cycle_weights.append(total_distance)

    if cycle_weights:
        min_index = cycle_weights.index(min(cycle_weights))
        optimal_distance = cycle_weights[min_index]
        optimal_permutation = permutations[min_index]
    else:
        optimal_distance = float('inf')
        optimal_permutation = None

    return optimal_distance, optimal_permutation

#bonus 2: indicate the largest "clique" aka a subset of nodes representing a complete graph (all nodes are connected to each other)
def largest_clique(graph):
    #generates subsets instead of permutations
    #return largest set of nodes that is a complete graph
    n = len(graph)  #number of nodes
    largest_clique_set = []

    #is subset complete graph?
    def is_complete_graph(nodes):
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                if nodes[j] not in graph[nodes[i]][1]:
                    return False
        return True

    def generate_subsets(current_set, index):
        if index == n:
            #check for complete graph
            if is_complete_graph(current_set) and len(current_set) > len(largest_clique_set):
                largest_clique_set[:] = current_set
            return

        generate_subsets(current_set + [index], index + 1)
        generate_subsets(current_set, index + 1)

    generate_subsets([], 0)

    return largest_clique_set
