import math

#transforms graph data to n * n matrix for input
def array_to_matrix(vertexList):
    vertices = len(vertexList)
    matrix = [[0] * vertices for _ in range(vertices)]

    for vertexNumber, vertexInfo in enumerate(vertexList):
        neighbors = vertexInfo[vertexNumber]
        for neighbor in neighbors:
            matrix[vertexNumber][neighbor] = 1
    return matrix

#finds shortest paths from floyd warshall matrices
def floyd_warshall_paths(graph_matrix):
    graph = array_to_matrix(graph_matrix)
    distances, parents = floyd_warshall(graph, graph_matrix)
    return clean_output(distances), clean_output(parents)

#removes "sys.maxsize" and replaces it with "no path" for sake of output
def clean_output(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] >= math.inf:
                matrix[i][j] = "no path"

    return matrix

#helps for the main to find winner
def find_distance(nodeA, nodeB):
    xA = nodeA[0][0]
    yA = nodeA[0][1]
    xB = nodeB[0][0]
    yB = nodeB[0][1]
    diffOfX = math.pow(xA-xB, 2)
    diffOfY = math.pow(yA-yB, 2)

    distance = math.sqrt(diffOfX + diffOfY)
    return distance

#performs floyd warshall algorithm and returns all-pairs shortest path
def floyd_warshall(graph, nodes):
    numOfVertices = len(graph)
    distanceMatrix = [[math.inf] * numOfVertices for _ in range(numOfVertices)] #set all to big number (infinity)
    parentMatrix = [[None] * numOfVertices for _ in range(numOfVertices)] #set parents to blank

    #put all vertices in distance matrix as 0 if it is pointing to itself
    for vertex in range(numOfVertices):
        distanceMatrix[vertex][vertex] = 0

    #finding distance between nodes
    for i in range(numOfVertices):
        for j in range(numOfVertices):
            if i < len(graph) and j < len(graph[i]):
                if graph[i][j] == 1:
                    distanceMatrix[i][j] = find_distance(nodes[i], nodes[j])

    #updates distance matrix
    for x in range(numOfVertices):
        for y in range(numOfVertices):
            for z in range(numOfVertices):
                if distanceMatrix[y][z] > distanceMatrix[y][x] + distanceMatrix[x][z]:
                    distanceMatrix[y][z] = distanceMatrix[y][x] + distanceMatrix[x][z]
                    parentMatrix[y][z] = x

    print("\n")
    for row in parentMatrix:
        print(row)

    return distanceMatrix, parentMatrix

