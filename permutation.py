#finds all permutations (steps) of the graph given using SJT
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

        #does the last node connect back to the start node?
        if is_cycle and perm[-1] not in graph[perm[0]][1]:
            is_cycle = False

        #returns true if a hamiltonian cycle is found
        if is_cycle:
            return True

        #returns false if no hamiltonian cycle is found
    return False


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
