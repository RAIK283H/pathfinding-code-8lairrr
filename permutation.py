#finds all permutations of natural numbers

def find_permutations(arr):
    #Initialize the first permutation with <1 <2 ... <n
    firstPermutation = arr
    mobileInteger = firstPermutation[len(firstPermutation)-1]
    #while there exists a mobile integer
    while mobileInteger:
        for i in range (len(firstPermutation) - 1): #  find the largest mobile integer k
            if (firstPermutation[i] < firstPermutation[i + 1]) and (mobileInteger < firstPermutation[i + 1]):
                mobileInteger = firstPermutation[i + 1]

        #  swap k and the adjacent integer it is looking at
        #  reverse the direction of all integers larger than k

    #check for hamiltonian from nodes 1 until n-1 (not start or end node)

    # write unit tests here

    #returns sequence of nodes if hamiltonian path
    #returns None if no paths

#bonus: indicate which hamiltonian cycles are optimal in terms of overall distance
def optimal_cycle(): #what is the param??? do I need to loop each graph or is it finding the shortest path?
    return -1

#bonus 2: indicate the largest "clique" aka a subset of nodes representing a complete graph
def largest_clique():
    return -1
