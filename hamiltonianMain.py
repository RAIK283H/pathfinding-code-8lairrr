import global_game_data
import graph_data
import permutation

def main():
    graphIndex = global_game_data.current_graph_index
    graphStuff = graph_data.graph_data
    currentGraph = graphStuff[graphIndex]

    #prints if there is a hamiltonian graph in the permutations
    #bonus: print optimal graph
    #bonus 2: print largest set of nodes that is a complete graph

if __name__ == "__main__":
    main()
