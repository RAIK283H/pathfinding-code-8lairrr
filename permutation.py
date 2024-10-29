#finds all permutations of natural numbers

#Initialize the first permutation with <1 <2 ... <n
firstPermutation = []
mobileInteger = firstPermutation[len(firstPermutation)-1]
#while there exists a mobile integer
while (mobileInteger):
    for i in range (len(firstPermutation) - 1): #  find the largest mobile integer k
        if (firstPermutation[i] < firstPermutation[i + 1]) and (mobileInteger < firstPermutation[i + 1]):
            mobileInteger = firstPermutation[i + 1]

    #  swap k and the adjacent integer it is looking at
    #  reverse the direction of all integers larger than k

#check for hamiltonian from nodes 1 until n-1 (not start or end node)

#returns sequence of nodes if hamiltonian path
#returns None if no paths

#write unit tests

#bonus: indicate which hamiltonian cycles are optimal in terms of overall distance
#bonus 2: indicate the largest "clique" aka a subset of nodes representing a complete graph
