import permutation
from graph_data import hamiltonian_graph


def main():
    #SJT permutations
    permutations = permutation.find_permutations(hamiltonian_graph)

    #find hamiltonian cycle
    has_hamiltonian_cycle = permutation.validate_hamiltonian_cycle(hamiltonian_graph, permutations)

    if has_hamiltonian_cycle:
        print("A Hamiltonian cycle was found in the graph.")
    else:
        print("No Hamiltonian cycle exists in the graph.")

if __name__ == "__main__":
    main()

#bonus: print optimal graph
#bonus 2: print largest set of nodes that is a complete graph
