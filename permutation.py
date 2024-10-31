#finds all permutations of natural numbers

def find_permutations(graph):
    #basically performing SJT and appending to an array
    permutations = []

    return permutations

def validate_hamiltonian_cycle(permutations): #validates that each index in the array of permutations is hamiltonian
    #check that each sequential number is in the number before adjacency list
    # (including that the start node is in the end node's)
    return -1


#bonus: indicate which hamiltonian cycles are optimal in terms of overall distance
#def optimal_cycle(permutations):
    #use hamiltonian_cycle on permutations
    #cycle_weights = []

    #use xy coords of each thing -> graph_data[a][b][0] = x-y coordinates as tuple of point b in graph a

    #return the lowest num and its corresponding permutation (same array in both arrays)

#bonus 2: indicate the largest "clique" aka a subset of nodes representing a complete graph (all nodes are connected to each other)
#use career fair lab !
#def largest_clique():
    #generates subsets instead of permutations
    #return largest set of nodes that is a complete graph
