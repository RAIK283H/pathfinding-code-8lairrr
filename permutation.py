#finds all permutations of natural numbers

def generate_permutations(arr):
    if len(arr) == 0:
        return [[]]
    result = []
    for i in range(len(arr)):
        current = arr[i]
        remaining = arr[:i] + arr[i + 1:]
        for perm in generate_permutations(remaining):
            result.append([current] + perm)
    return result

def find_permutations(graph):
    internal_nodes = range(1, len(graph) - 1)
    all_cycles = []

    internal_nodes_list = list(internal_nodes)
    for perm in generate_permutations(internal_nodes_list):
        cycle = [0] + perm + [len(graph) - 1]
        all_cycles.append(cycle)

    return all_cycles


def hamiltonian_cycle(graph, cycles):
    hamiltonian_cycles = []

    for path in cycles:
        internal_nodes = range(1, len(graph) - 1)
        visited_internal_nodes = [node for node in path[1:-1] if node in internal_nodes]

        if len(visited_internal_nodes) == len(internal_nodes) and path[0] in graph[path[-1]]:
            hamiltonian_cycles.append(path)

    return hamiltonian_cycles if hamiltonian_cycles else -1

#bonus: indicate which hamiltonian cycles are optimal in terms of overall distance
#def optimal_cycle(): #what is the param??? do I need to loop each graph or is it finding the shortest path?
    #return -1

#bonus 2: indicate the largest "clique" aka a subset of nodes representing a complete graph
#def largest_clique():
    #return -1
