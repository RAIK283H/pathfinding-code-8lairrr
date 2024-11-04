import permutation
import graph_data


def main():
    current_graph = graph_data.hamiltonian_graph

    #SJT permutations
    permutations = permutation.find_permutations(current_graph)

    #find hamiltonian cycle
    has_hamiltonian_cycle, hamiltonian_cycles = permutation.validate_hamiltonian_cycle(current_graph, permutations)

    if has_hamiltonian_cycle:
        print("Hamiltonian cycles found in the graph:")
        for cycle in hamiltonian_cycles:
            print(cycle)
        optimal_distance, optimal_permutation = permutation.optimal_cycle(permutations, current_graph) #extra credit 1
        print(f"The optimal cycle distance is: {optimal_distance}")
        print(f"The optimal cycle is: {optimal_permutation}")
    else:
        print("No Hamiltonian cycle exists in the graph.")

    #finding largest clique
    clique = permutation.largest_clique(current_graph)

    if clique:
        print("The largest clique found in the graph is:", clique)
    else:
        print("No clique found in the graph.")

if __name__ == "__main__":
    main()
