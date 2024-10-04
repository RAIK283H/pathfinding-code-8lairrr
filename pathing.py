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

    #generating the random path from start to target
    while currentNodeIndex != targetNodeIndex:
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

    # path from target to end
    while currentNodeIndex != endNodeIndex:
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

    #checking that path doesn't end at a node it isn't supposed to
    assert path[-1] == endNodeIndex, "path does not end at the correct exit node"

    return path


def get_dfs_path():
    return [1,2]


def get_bfs_path():
    return [1,2]


def get_dijkstra_path():
    return [1,2]
