import math

from scipy.spatial import distance_matrix
import graph_data

#finds shortest paths from floyd warshall matrices
def floyd_warshall_paths(distance_matrix, parent_matrix):
    for k in range(len(distance_matrix)):
        for i in range(len(distance_matrix)):
            for j in range(len(distance_matrix)):
                current_distance = distance_matrix[i][k] + distance_matrix[k][j]
                if distance_matrix[i][j] > current_distance:
                    distance_matrix[i][j] = current_distance
                    parent_matrix[i][j] = parent_matrix[k][j]

#builds path from u (start node) to v (target node)
def build_path(u, v, parent_matrix):
    path = []
    if parent_matrix[u][v] is None:
        return path
    path.insert(0, v)
    while u != v:
        v = parent_matrix[u][v]
        path.insert(0, v)
    return path

#removes infinity and replaces it with "no path" for sake of output
def clean_output(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] >= math.inf:
                matrix[i][j] = "no path"

    return matrix

#helps for the main to find winner
def find_distance(nodeA, nodeB):
    xA = nodeA[0]
    yA = nodeA[1]
    xB = nodeB[0]
    yB = nodeB[1]
    diffOfX = math.pow(xA-xB, 2)
    diffOfY = math.pow(yA-yB, 2)

    distance = math.sqrt(diffOfX + diffOfY)
    return distance

#builds matrices and performs floyd warshall algorithm. returns all-pairs shortest path.
def floyd_warshall(graph, nodes):
    numOfVertices = len(graph)
    distanceMatrix = [[math.inf] * numOfVertices for _ in range(numOfVertices)] #set all to big number (infinity)
    parentMatrix = [[None] * numOfVertices for _ in range(numOfVertices)] #set parents to blank

    #finding distance between nodes
    for i in range(numOfVertices):
        for j in graph[i][1]: #goes through adjacency list
            if i == j: #put all vertices in distance matrix as 0 if it is pointing to itself
                distanceMatrix[i][j] = 0

            else:
                distanceMatrix[i][j] = find_distance(nodes[i][0], nodes[j][0])
                parentMatrix[i][j] = i

    for i in range(numOfVertices):
        parentMatrix[i][i] = i

    paths = floyd_warshall_paths(distanceMatrix, parentMatrix)
    return paths
