import graph_data
import global_game_data
from numpy import random

def set_current_graph_paths():
    global_game_data.graph_paths.clear()
    global_game_data.graph_paths.append(get_test_path())
    global_game_data.graph_paths.append(get_random_path())
    global_game_data.graph_paths.append(get_dfs_path())
    global_game_data.graph_paths.append(get_bfs_path())
    global_game_data.graph_paths.append(get_dijkstra_path())


def get_test_path():
    return graph_data.test_path[global_game_data.current_graph_index]

def get_random_path(): #generates a path randomly from start to target and target to end
    graphIndex = global_game_data.current_graph_index #stores index for the graph we are accessing
    graphStuff = graph_data.graph_data #the list of graphs

    startNodeIndex = 0 #get start node (the first index in the graph)
    targetNodeIndex = global_game_data.target_node[graphIndex] #get target node index
    endNodeIndex = len(graphStuff[graphIndex]) - 1 #get exit node (the last index in the graph)

    #initialize the start node
    currentNodeIndex = startNodeIndex
    path = [currentNodeIndex] #keeps track of each index visited

    #get current graph
    graph = graphStuff[graphIndex]
    visited = {currentNodeIndex} #for visited neighbors

    max_iterations = 1000
    iterations = 0

    #generating the random path from start to target
    while currentNodeIndex != targetNodeIndex and iterations < max_iterations:
        neighbors = list(graph[currentNodeIndex][1])

        # ensure the current node has neighbors
        assert len(neighbors) > 0, f"Node {currentNodeIndex} has no neighbors!"

        # create a list of unvisited neighbors
        frontier = [n for n in neighbors if n not in visited]

        # if no unvisited neighbors, fall back to any available neighbor
        if len(frontier) == 0:
            frontier = neighbors

        # choose a random neighbor and update the path
        nextNodeIndex = int(random.choice(frontier))
        path.append(nextNodeIndex)
        visited.add(nextNodeIndex)
        global_game_data.nodesVisited += 1

        # move to the next node
        currentNodeIndex = nextNodeIndex

        iterations += 1 #avoiding infinite loop

        if iterations >= max_iterations:
            print("Max iterations reached while finding path to target.")
            return path

    # path from target to end
    while currentNodeIndex != endNodeIndex and iterations < max_iterations:
        neighbors = list(graph[currentNodeIndex][1])

        # ensure the current node has neighbors
        assert len(neighbors) > 0, f"Node {currentNodeIndex} has no neighbors!"

        # create a list of unvisited neighbors
        frontier = [n for n in neighbors if n not in visited]

        # if no unvisited neighbors, fall back to any available neighbor
        if len(frontier) == 0:
            frontier = neighbors

        # choose a random neighbor and update the path
        nextNodeIndex = int(random.choice(frontier))
        path.append(nextNodeIndex)
        visited.add(nextNodeIndex)
        global_game_data.nodesVisited += 1

        # move to the next node
        currentNodeIndex = nextNodeIndex

        iterations += 1 #infinite loop prevention

        if iterations >= max_iterations:
            print("Max iterations reached while finding path to end.")
            return path

    #checking that path doesn't end at a node it isn't supposed to
    assert path[-1] == endNodeIndex, "path does not end at the correct exit node"

    return path


def get_dfs_path():
    graphIndex = global_game_data.current_graph_index  # stores index for the graph we are accessing
    graphStuff = graph_data.graph_data  # the list of graphs

    startNodeIndex = 0  # get start node (the first index in the graph)
    targetNodeIndex = global_game_data.target_node[graphIndex]  # get target node index
    endNodeIndex = len(graphStuff[graphIndex]) - 1  # get exit node (the last index in the graph)

    # initialize the start node
    currentNodeIndex = startNodeIndex
    path = []  # keeps track of the final path
    visited = {currentNodeIndex}  # tracks visited nodes

    # DFS data structures
    stack = [currentNodeIndex]  # Stack for DFS, starting with the start node
    parents = {currentNodeIndex: None}  # Parent mapping for backtracking

    while stack:
        currentNodeIndex = stack.pop()  # get the last node from the stack

        # check if we reached the target node
        if currentNodeIndex == targetNodeIndex:
            break

        # get neighbors
        neighbors = graphStuff[graphIndex][currentNodeIndex][1]

        for neighbor in neighbors:
            if neighbor not in visited:
                visited.add(neighbor)  # mark as visited
                parents[neighbor] = currentNodeIndex  # track its parent
                stack.append(neighbor)  # add to the stack

    # backtrack to form first part of the path
    path_to_target = []
    currentNodeIndex = targetNodeIndex
    while currentNodeIndex is not None:
        path_to_target.append(currentNodeIndex)
        currentNodeIndex = parents[currentNodeIndex]
    path_to_target.reverse()  # reverse path

    visited = {targetNodeIndex}  # reset visited set
    stack = [targetNodeIndex]  # reset stack
    parents = {targetNodeIndex: None}  # reset parents

    # DFS for target to end
    while stack:
        currentNodeIndex = stack.pop()

        # check if we've reached the end node
        if currentNodeIndex == endNodeIndex:
            break

        # get neighbors
        neighbors = graphStuff[graphIndex][currentNodeIndex][1]

        for neighbor in neighbors:
            if neighbor not in visited:
                visited.add(neighbor)
                parents[neighbor] = currentNodeIndex
                stack.append(neighbor)

    # backtrack to form the second part of the path
    path_to_end = []
    currentNodeIndex = endNodeIndex
    while currentNodeIndex is not None:
        path_to_end.append(currentNodeIndex)
        currentNodeIndex = parents[currentNodeIndex]
    path_to_end.reverse()

    # combine both paths into path variable
    path = path_to_target[:-1] + path_to_end  # exclude the duplicated target node

    # post-conditions
    assert path[-1] == endNodeIndex, "Path does not end at the correct exit node"
    assert targetNodeIndex in path, "PResult path does not include the target node."

    for index in range(len(path) - 1):  # checking that all nodes have paths to each other
        nodeA = path[index]
        nodeB = path[index + 1]
        nodeA_neighbors = graphStuff[graphIndex][nodeA][1]  # get neighbors of nodeA
        assert nodeB in nodeA_neighbors, f"No edge exists between {nodeA} and {nodeB}."

    return path


def get_bfs_path():
    graphIndex = global_game_data.current_graph_index  # stores index for the graph we are accessing
    graphStuff = graph_data.graph_data  # the list of graphs

    startNodeIndex = 0  # get start node (the first index in the graph)
    targetNodeIndex = global_game_data.target_node[graphIndex]  # get target node index
    endNodeIndex = len(graphStuff[graphIndex]) - 1  # get exit node (the last index in the graph)

    # initialize start node
    currentNodeIndex = startNodeIndex
    path = []  # keeps track of the final path
    visited = {currentNodeIndex}  # tracks visited nodes

    # for BFS data structures
    frontier = [currentNodeIndex]  # queue
    parents = {currentNodeIndex: None}  # parent mapping for back tracking

    # path from start to target
    while frontier:
        currentNodeIndex = frontier.pop(0)  # get next node from the queue

        # check if we've reached the target node
        if currentNodeIndex == targetNodeIndex:
            break

        # get the neighbors of the current node
        neighbors = graphStuff[graphIndex][currentNodeIndex][1]

        for neighbor in neighbors:
            if neighbor not in visited:
                visited.add(neighbor)  # mark as visited
                parents[neighbor] = currentNodeIndex  # track its parent
                frontier.append(neighbor)  # add the neighbor to the frontier queue

    # backtrack from target to start to form the path
    path_to_target = []
    currentNodeIndex = targetNodeIndex
    while currentNodeIndex is not None:
        path_to_target.append(currentNodeIndex)
        currentNodeIndex = parents[currentNodeIndex]
    path_to_target.reverse()  # reverse the path

    # path from target to end
    visited = {targetNodeIndex}  # reset visited set
    frontier = [targetNodeIndex]  # reset frontier
    parents = {targetNodeIndex: None}  # reset parents

    while frontier:
        currentNodeIndex = frontier.pop(0)

        # check if we've reached the end node
        if currentNodeIndex == endNodeIndex:
            break

        # get neighbors
        neighbors = graphStuff[graphIndex][currentNodeIndex][1]

        for neighbor in neighbors:
            if neighbor not in visited:
                visited.add(neighbor)
                parents[neighbor] = currentNodeIndex
                frontier.append(neighbor)

    # backtrack from end to target to get second part of path
    path_to_end = []
    currentNodeIndex = endNodeIndex
    while currentNodeIndex is not None:
        path_to_end.append(currentNodeIndex)
        currentNodeIndex = parents[currentNodeIndex]
    path_to_end.reverse()

    # combine the two paths into path variable
    path = path_to_target[:-1] + path_to_end  # exclude the duplicated target node

    # post-conditions
    assert path[-1] == endNodeIndex, "Path does not end at the correct exit node"
    assert targetNodeIndex in path, "PResult path does not include the target node."

    for index in range(len(path) - 1): #checking that all nodes have paths to each other
        nodeA = path[index]
        nodeB = path[index + 1]
        nodeA_neighbors = graphStuff[graphIndex][nodeA][1]  # get neighbors of nodeA
        assert nodeB in nodeA_neighbors, f"No edge exists between {nodeA} and {nodeB}."

    return path


def get_dijkstra_path():
    return [1,2]
