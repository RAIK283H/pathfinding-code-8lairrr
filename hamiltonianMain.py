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

    optimal_distance, optimal_permutation = permutation.optimal_cycle(permutations, hamiltonian_graph)
    print(f"The optimal cycle distance is: {optimal_distance}")
    print(f"The optimal cycle is: {optimal_permutation}")

    #finding largest clique
    clique = permutation.largest_clique(hamiltonian_graph)

    if clique:
        print("The largest clique found in the graph is:", clique)
    else:
        print("No clique found in the graph.")

if __name__ == "__main__":
    main()
