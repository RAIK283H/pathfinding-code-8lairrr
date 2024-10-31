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


def is_valid_move(node, path, graph):
    if node not in graph[path[-1]]:
        return False  #not connected to the previous node in path
    if node in path:
        return False  #node is already visited
    return True

def hamiltonian_cycle_backtracking(graph, path):
    if len(path) == len(graph) - 1:
        if path[0] in graph[path[-1]]:
            return [path + [path[0]]]
        return []

    cycles = []
    for node in range(1, len(graph) - 1):
        if is_valid_move(node, path, graph):
            path.append(node)
            cycles.extend(hamiltonian_cycle_backtracking(graph, path))
            path.pop()

    return cycles

def hamiltonian_cycle(graph):
    all_hamiltonian_cycles = []
    internal_nodes = range(1, len(graph) - 1)

    for start_node in internal_nodes:
        path = [start_node]
        all_hamiltonian_cycles.extend(hamiltonian_cycle_backtracking(graph, path))

    return all_hamiltonian_cycles if all_hamiltonian_cycles else -1

#bonus: indicate which hamiltonian cycles are optimal in terms of overall distance
#def optimal_cycle(): #what is the param??? do I need to loop each graph or is it finding the shortest path?
    #return -1

#bonus 2: indicate the largest "clique" aka a subset of nodes representing a complete graph
#def largest_clique():
    #return -1
