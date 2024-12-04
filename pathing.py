import heapq
import math
import f_w

import graph_data
import global_game_data
from numpy import random

from config_data import player_data


def set_current_graph_paths():
    global_game_data.graph_paths.clear()
    global_game_data.graph_paths.append(get_test_path())
    global_game_data.graph_paths.append(get_random_path())
    global_game_data.graph_paths.append(get_dfs_path())
    global_game_data.graph_paths.append(get_bfs_path())

    if player_data[4][0] == "Floyd-Warshall":
        #use floyd-warshall's algorithm to calculate paths
        graph = graph_data.graph_data[global_game_data.current_graph_index]

        graph_matrix, parent_matrix = f_w.floyd_warshall(graph, graph)
        paths = f_w.floyd_warshall_paths(graph_matrix)
        global_game_data.graph_paths.append(paths)
    else:
        # default to dijkstra
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
    graphIndex = global_game_data.current_graph_index
    graphStuff = graph_data.graph_data

    startNodeIndex = 0
    targetNodeIndex = global_game_data.target_node[graphIndex]
    endNodeIndex = len(graphStuff[graphIndex]) - 1

    #resetting variables for scoreboard purposes
    global_game_data.distance_traveled = 0
    global_game_data.dijkstra_nodes_visited = 0

    def heuristic(node, goal): #for extra credit
        node_coords = graphStuff[graphIndex][node][0]
        goal_coords = graphStuff[graphIndex][goal][0]
        return math.sqrt((node_coords[0] - goal_coords[0]) ** 2 + (node_coords[1] - goal_coords[1]) ** 2)

    def a_star(start, goal): #for extra credit
        pq = []  #priority queue
        heapq.heappush(pq, (0, start))  #priority, node
        g_scores = {node: float('inf') for node in range(len(graphStuff[graphIndex]))}
        g_scores[start] = 0
        parents = {start: None}

        while pq:
            current_priority, current_node = heapq.heappop(pq)
            global_game_data.dijkstra_nodes_visited += 1

            #stop if reached the goal
            if current_node == goal:
                break

            #looking through neighbors
            for neighbor in graphStuff[graphIndex][current_node][1]:
                tentative_g_score = g_scores[current_node] + 1  #assuming edge weight is 1
                if tentative_g_score < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(pq, (f_score, neighbor))
                    parents[neighbor] = current_node

        #backtracking to construct path
        path = []
        current_node = goal
        while current_node is not None:
            path.append(current_node)
            if parents[current_node] is not None:
                global_game_data.distance_traveled += 1
            current_node = parents[current_node]
        path.reverse()
        return path

    #getting paths from start to target and then target to end
    path_to_target = a_star(startNodeIndex, targetNodeIndex)
    path_to_end = a_star(targetNodeIndex, endNodeIndex)

    #combine the two paths and remove the duplicate target node
    full_path = path_to_target[:-1] + path_to_end

    #updating global variables
    global_game_data.dijkstra_nodes_visited = global_game_data.dijkstra_nodes_visited
    global_game_data.distance_traveled = global_game_data.distance_traveled

    #postcondition checks
    assert full_path[0] == startNodeIndex, "Path does not start at the correct start node."
    assert full_path[-1] == endNodeIndex, "Path does not end at the correct exit node."
    for i in range(len(full_path) - 1):
        nodeA = full_path[i]
        nodeB = full_path[i + 1]
        assert nodeB in graphStuff[graphIndex][nodeA][1], f"No edge exists between {nodeA} and {nodeB}."

    return full_path
