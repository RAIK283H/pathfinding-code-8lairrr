# Pathfinding Starter Code

my statistic:
    the "nodes visited" statistic tracks the number of unique nodes each player has passed through during their path in the game. the aim of this statistic is to give players a clear view of how much of the graph they've explored. the count is updated as players go through new areas, helping them gauge how efficient their exploration has been in finding the target or exit.

my random pathing algorithm:
    my random pathing algorithm generates a path through the graph by selecting a random neighbor at each step from the start node to the target node, and then from the target node to the exit node. after initializing all the nodes needed and identifying which nodes have been visited, the algorithm enters a loop where it selects unvisited neighboring nodes to continue the path. once the target node is reached, the algorithm aims to reach the exit node. the algorithm also prevents revisiting nodes by ensuring there are no loops. 

did extra credit on homework 6 (implementing A* using any admissible heuristic)