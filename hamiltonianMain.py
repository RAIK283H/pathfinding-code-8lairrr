import global_game_data
import graph_data
from permutation import find_permutations, hamiltonian_cycle

def main():
    graphIndex = global_game_data.current_graph_index
    graphStuff = graph_data.graph_data
    currentGraph = graphStuff[graphIndex]

    all_cycles = find_permutations(currentGraph)  # finding all permutations

    hamiltonian_graph = graph_data.hamiltonian_graph
    valid_hamiltonian_cycles = hamiltonian_cycle(hamiltonian_graph)

    if valid_hamiltonian_cycles == -1 or not valid_hamiltonian_cycles:
        print(f"Hamiltonian graph: no valid Hamiltonian cycles found!")
    else:
        print(f"Hamiltonian graph: valid Hamiltonian cycles:")
        for cycle in valid_hamiltonian_cycles:
            print(cycle)

if __name__ == "__main__":
    main()
