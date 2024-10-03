import graph_data
import global_game_data
from numpy import random

'''
from graph_data:
graph_data[a] = gives you graph at index a
graph_data[a][0] = start node of graph a
graph_data[a][length-1] = exit node of graph a
graph_data[a][b][0] = x-y coordinates as tuple of point b in graph a
graph_data[a][b][1] = adjacency list of point b in graph a
'''

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

    startNode = 0 #get start node (the first node in the graph)
    targetNode = global_game_data.target_node[graphIndex] #get target node
    endNode = graph_data[graphIndex][len(graph_data[graphIndex]) - 1] #get exit node (the last node in the graph)
    '''issues on above line for some reason'''
    #initialize the start node
    currentNode = startNode
    path = [currentNode]

    #get current graph
    graph = graph_data[graphIndex]
    visited = {currentNode} #for visited neighbors

    #generating the random path
    while currentNode != endNode:
        if currentNode == targetNode:
            newTarget = endNode #once target node is reached, then the new target is the end node
        else:
            newTarget = targetNode
        
        #get neighbors of the node we are on
        neighbors = graph[currentNode][1]

        #checking that there are no neighbors before proceeding
        assert len(neighbors) > 0, "node {} has no neighbors".format(currentNode)
        
        #creating frontier of unvisited neighbors
        frontier = [] #for unvisited neighbors
        for n in neighbors:
            if n not in visited:  #check if the neighbor has not been visited yet
                frontier.append(n) #appends to non-visited neigbors
        
        #break loop if no more neighbors to visit
        if len(frontier) == 0:
            break

        #checking that the frontier is not empty before proceeding
        assert len(frontier) > 0, "frontier is empty and shouldn't be"
        
        #choose a random neighbor to go to
        nextNode = random.choice(frontier)
        
        #update path and visited nodes
        path.append(nextNode)
        visited.add(nextNode)
        
        #move to the next node
        currentNode = nextNode
    
    #checking that path doesn't end at a node it isn't supposed to
    assert path[-1] == endNode, "path does not end at the correct exit node"

    return path


def get_dfs_path():
    #i did this because i thought we were supposed to but on further reading we dont...
    #but i dont want to completely start from scratch next time lol sorry about that
    '''
    graphIndex = global_game_data.current_graph_index  # stores the graph index of the graph we are using

    startNode = 0  # start node is the first node
    targetNode = global_game_data.target_node[graphIndex]  # getting target node
    endNode = len(graph_data[graphIndex]) - 1  # exit node is the last node

    # frontier as stack of neighbors
    frontier = [startNode]  # use a list to simulate stack behavior
    visited = {startNode}  # set to track visited nodes
    parents = {startNode: None}  # map to track parents of the nodes

    # the actual searching part
    while frontier:
        currentNode = frontier.pop()  # getting the next node in stack (LIFO)

        # check if we've reached the target node
        if currentNode == targetNode:
            newTarget = endNode  # once target node is reached, then the new target is the end node
        else:
            newTarget = targetNode

        # get neighbors of the current node
        neighbors = graph_data[graphIndex][currentNode][1]

        # go through the neighbors
        for neighbor in neighbors:
            if neighbor not in visited:
                visited.add(neighbor)  # mark neighbor as visited
                parents[neighbor] = currentNode  # set the parent
                frontier.append(neighbor)  # add neighbor to the stack

        # check if we've reached the end node
        if currentNode == endNode:
            break

    # go back to find the path
    path = []
    current = endNode  # begin going back from end node
    while current is not None:
        path.append(current) 
        current = parents[current]  # move to parent node

    path.reverse()  # reverse path to get it right

    return path
    '''


def get_bfs_path():
    #i did this because i thought we were supposed to but on further reading we dont...
    #but i dont want to completely start from scratch next time lol sorry about that
    '''
    graphIndex = global_game_data.current_graph_index  # stores the graph index of the graph we are using

    startNode = 0  #start node is first node
    targetNode = global_game_data.target_node[graphIndex]  #getting target node
    endNode = len(graph_data[graphIndex]) - 1  #exit node is last node
    
    #frontier as queue of neighbors
    frontier = [startNode]  
    visited = {startNode}  
    parents = {startNode: None}  #map to track parents of the nodes

    #the actual searching part
    while frontier:
        currentNode = frontier.pop(0)  #getting next node in queue (frontier)

        #have we reached target node?
        if currentNode == targetNode:
            newTarget = endNode  #once target node is reached, then the new target is the end node
        else:
            newTarget = targetNode

        #get neighbors of the current node
        neighbors = graph_data[graphIndex][currentNode][1]

        #go through the neighbors
        for neighbor in neighbors:
            if neighbor not in visited: 
                visited.add(neighbor)  #mark neighbor as visited
                parents[neighbor] = currentNode  #set the parent
                frontier.append(neighbor)

        if currentNode == endNode:
            break

    #go back to find the path
    path = []
    current = endNode  #begin going back from end node
    while current is not None:
        path.append(current) 
        current = parents[current]  #move to parent node

    path.reverse()  #reverse path to get it right

    return path
    '''


def get_dijkstra_path():
    return [1,2]
