import graph_data
import global_game_data

#transforms graph data to n * n matrix for input
def graph_data_to_graph_matrix_matrix(graph):
    n = len(graph) #number of nodes
    graph_matrix = [[float('inf')] * n for _ in range(n)] #initialize n x n matrix with infinity as the values

    for i, (_, adjacency_list) in enumerate(graph):
        graph_matrix[i][i] = 0 #distance from a node to itself is 0
        for neighbor in adjacency_list:
            if isinstance(neighbor, tuple): #if it has a weight
                graph_matrix[i][neighbor[0]] = neighbor[1]
            else: #there is no weight
                graph_matrix[i][neighbor] = 1

    return graph_matrix

#generates an n * n matrix of the parents of the graph_matrix matrix
def generate_parent_matrix(graph_matrix):
    n = len(graph_matrix) #number of nodes
    parent_matrix = [[None] * n for _ in range(n)] #initialize n x n matrix with None as the values

    for i in range(n):
        for j in range(n):
            if i != j and graph_matrix[i][j] != float('inf'):
                parent_matrix[i][j] = i #the parent of j in the shortest path is i
            else:
                parent_matrix[i][j] = None  #no connection = no parent

    return parent_matrix

#performs floyd warshall algorithm and returns all-pairs shortest path
def floyd_warshall(graph):
    graph_matrix = graph_data_to_graph_matrix_matrix(graph) #graph_matrix is a weighted graph matrix
    V = len(graph_matrix) #V is the number of nodes aka the height/weight of the graph_matrix matrix
    parent_matrix = generate_parent_matrix(graph_matrix) #parent_matrix is the matrix that holds parents of the graph_matrix matrix

    for k in range(V - 1):
        for i in range(V - 1):
            for j in range(V - 1):
                if graph_matrix[i][k] + graph_matrix[k][j] < graph_matrix[i][j]:
                    graph_matrix[i][j] = graph_matrix[i][k] + graph_matrix[k][j] #children matrix
                    parent_matrix[i][j] = parent_matrix[k][j] #parent matrix is updated for new path

    return graph_matrix, parent_matrix #return updated matrices

#finds paths from floyd warshall matrices
def floyd_warshall_paths(graph_matrix, parent_matrix):
    V = len(graph_matrix) #number of vertices
    all_paths = []

    for i in range(V):
        for j in range(V):
            if graph_matrix[i][j] == float('inf'): #no path exists
                all_paths.append((i, j, [])) #empty path for (i, j)
            else: #reconstructing the path
                path = []
                current = i
                visited = set()
                while current is not None and current not in visited:
                    visited.add(current)
                    path.insert(0, current) #prepend node
                    current = parent_matrix[i][current]
                    if current is not None:  #if a cycle is detected return an empty path
                        path = []
                    all_paths.append((i, j, path)) #add the path for (i, j)

    return all_paths

def main():
    current_graph = graph_data.graph_data[global_game_data.current_graph_index]

    #makes weight matrix
    graph_matrix = graph_data_to_graph_matrix_matrix(current_graph)
    print("weight matrix:")
    for row in graph_matrix:
        print(row)

    #makes parent matrix
    parent_matrix = generate_parent_matrix(graph_matrix)
    print("\nparent matrix:")
    for row in parent_matrix:
        print(row)

    print("\nperforming floyd-warshall")

    #floyd-warshall
    updated_graph_matrix, updated_parent_matrix = floyd_warshall(current_graph)

    print("\nnew weight matrix:")
    for row in updated_graph_matrix:
        print(row)

    print("\nnew parent matrix:")
    for row in updated_parent_matrix:
        print(row)

    #finding paths
    all_paths = floyd_warshall_paths(updated_graph_matrix, updated_parent_matrix)
    print("\nall shortest paths:")
    for start, end, path in all_paths:
        if path:
            print(f"shortest path from {start} to {end}: {path}")
        else:
            print(f"no path exists from {start} to {end}.")

if __name__ == "__main__":
    main()
