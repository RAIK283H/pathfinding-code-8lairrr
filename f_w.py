import graph_data
import global_game_data

graph = graph_data.graph_data[global_game_data.current_graph_index]

#transforms graph data to n * n matrix for input
def graph_data_to_graph_matrix_matrix(graph):
    n = len(graph) #number of nodes
    graph_matrix = [[float('inf')] * n for _ in range(n)] #initialize n x n matrix with infinity as the values

    for i, (_, adjacency_list) in enumerate(graph):
        graph_matrix[i][i] = 0 #distance from a node to itself is 0
        for neighbor, weight in adjacency_list:
            graph_matrix[i][neighbor] = weight #update with the given edge weight

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
def floyd_warshall():
    graph_matrix = graph_data_to_graph_matrix_matrix() #graph_matrix is a weighted graph matrix
    V = len(graph_matrix) #V is the number of nodes aka the height/weight of the graph_matrix matrix
    parent_matrix = generate_parent_matrix(graph_matrix) #parent_matrix is the matrix that holds parents of the graph_matrix matrix

    for k in range(V - 1):
        for i in range(V - 1):
            for j in range(V - 1):
                if graph_matrix[i,k] + graph_matrix[k,j] < graph_matrix[i,j]:
                    graph_matrix[i,j] = graph_matrix[i,k] + graph_matrix[k,j] #children matrix
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
                while current != j:
                    if current == -1: #if there's no valid path
                        path = []
                        break
                    path.append(current)
                    current = parent_matrix[current][j]
                if current != -1: #add the last node if path reconstruction was successful
                    path.append(j)
                all_paths.append((i, j, path)) #add the path for (i, j)

    return all_paths
