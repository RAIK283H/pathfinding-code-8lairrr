import global_game_data
import graph_data
from permutation import find_permutations, hamiltonian_cycle

def main():
    graphIndex = global_game_data.current_graph_index
    graphStuff = graph_data.graph_data
    currentGraph = graphStuff[graphIndex]

    all_cycles = find_permutations(currentGraph) #finding all permutations

    valid_hamiltonian_cycles = hamiltonian_cycle(currentGraph, all_cycles) #finding hamiltonian cycles

    if valid_hamiltonian_cycles == -1 or not valid_hamiltonian_cycles: #if there are no hamiltonian cycles
        print("no valid hamiltonian cycles found!")
    else:
        print("valid hamiltonian cycles:")
        for cycle in valid_hamiltonian_cycles:
            print(cycle)

if __name__ == "__main__":
    main()
