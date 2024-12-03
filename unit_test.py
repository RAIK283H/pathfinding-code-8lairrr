import math
import unittest

from debugpy.common.timestamp import current
from numpy.random import permutation

import f_w
import global_game_data
import graph_data
import pathing
from f_w import floyd_warshall
from pathing import get_dfs_path, get_bfs_path
from permutation import find_permutations, validate_hamiltonian_cycle


class TestPathFinding(unittest.TestCase):

    def test_upper(self):
        self.assertEqual('test'.upper(), 'TEST')

    def test_isupper(self):
        self.assertTrue('TEST'.isupper())
        self.assertFalse('Test'.isupper())

    def test_floating_point_estimation(self):
        first_value = 0
        for x in range(1000):
            first_value += 1/100
        second_value = 10
        almost_pi = 3.1
        pi = math.pi
        self.assertNotEqual(first_value,second_value)
        self.assertAlmostEqual(first=first_value,second=second_value,delta=1e-9)
        self.assertNotEqual(almost_pi, pi)
        self.assertAlmostEqual(first=almost_pi, second=pi, delta=1e-1)

    def setUp(self):
        #saving the original values to reset after
        self.original_graph_data = graph_data.graph_data
        self.original_target_node = global_game_data.target_node
        self.original_current_graph_index = global_game_data.current_graph_index

        #setting up "mocks for the tests"
        graph_data.graph_data = [
            [(0, [1]), (1, [0, 2]), (2, [1, 3]), (3, [2])]
        ]
        global_game_data.target_node = [2]  # Example target node
        global_game_data.current_graph_index = 0

    #for resetting the values after testing
    def tearDown(self):
        graph_data.graph_data = self.original_graph_data
        global_game_data.target_node = self.original_target_node
        global_game_data.current_graph_index = self.original_current_graph_index

    def test_dfs_path_includes_target(self):
        path = get_dfs_path()
        targetNodeIndex = global_game_data.target_node[global_game_data.current_graph_index]
        self.assertIn(targetNodeIndex, path, "DFS path does not include the target node.")

    def test_dfs_path_ends_at_exit(self):
        path = get_dfs_path()
        endNodeIndex = len(graph_data.graph_data[global_game_data.current_graph_index]) - 1
        self.assertEqual(path[-1], endNodeIndex, "DFS path does not end at the exit node.")

    def test_dfs_path_has_valid_edges(self):
        path = get_dfs_path()
        graphIndex = global_game_data.current_graph_index
        graph = graph_data.graph_data[graphIndex]

        for i in range(len(path) - 1):
            nodeA = path[i]
            nodeB = path[i + 1]
            neighbors = graph[nodeA][1]  #getting neighbors of nodeA
            self.assertIn(nodeB, neighbors, f"DFS: No edge exists between {nodeA} and {nodeB}.")

    def test_bfs_path_includes_target(self):
        path = get_bfs_path()
        targetNodeIndex = global_game_data.target_node[global_game_data.current_graph_index]
        self.assertIn(targetNodeIndex, path, "BFS path does not include the target node.")

    def test_bfs_path_ends_at_exit(self):
        path = get_bfs_path()
        endNodeIndex = len(graph_data.graph_data[global_game_data.current_graph_index]) - 1
        self.assertEqual(path[-1], endNodeIndex, "BFS path does not end at the exit node.")

    def test_bfs_path_has_valid_edges(self):
        path = get_bfs_path()
        graphIndex = global_game_data.current_graph_index
        graph = graph_data.graph_data[graphIndex]

        for i in range(len(path) - 1):
            nodeA = path[i]
            nodeB = path[i + 1]
            neighbors = graph[nodeA][1]  #getting neighbors of nodeA
            self.assertIn(nodeB, neighbors, f"BFS: No edge exists between {nodeA} and {nodeB}.")

    '''
    def test_permutations_are_correct(self):
    graph = graph_data.graph_data[0]

    all_perms = find_permutations(graph)
    expected_perms = [[0, 1, 2], [0, 2, 1], [2, 0, 1], [2, 1, 0], [1, 2, 0], [1, 0, 2], [2, 0, 1]]
    self.assertEqual(all_perms, expected_perms, "Permutations do not match the expected list.")
    '''

    def test_there_are_no_hamiltonian(self):
        graph = graph_data.graph_data[0]
        permutations = find_permutations(graph)

        has_hamiltonian_cycle, hamiltonian_cycles = validate_hamiltonian_cycle(graph, permutations)

        self.assertFalse(has_hamiltonian_cycle, "Expected no Hamiltonian cycle, but one was found.")

    '''
    def test_dijkstra_path_includes_target(self):
        path = pathing.get_dijkstra_path()
        target_node_index = global_game_data.target_node[global_game_data.current_graph_index]
        self.assertIn(target_node_index, path, "Dijkstra path does not include the target node.")

    def test_dijkstra_path_is_shortest(self):
        path = pathing.get_dijkstra_path()
        shortest_path = [0, 1, 3]
        self.assertEqual(path, shortest_path, "Dijkstra path is not the shortest path.")
    '''

    def test_floyd_warshall_graph_0(self):
        graph = graph_data.graph_data[0]
        graph_matrix, parent_matrix = f_w.floyd_warshall(graph)
        # For graph 0, all nodes are directly connected
        expected_result = [
            [0, 1, 2],  # Distances from Node 0
            [1, 0, 1],  # Distances from Node 1
            [2, 1, 0]  # Distances from Node 2
        ]
        self.assertEqual(graph_matrix, expected_result, "Floyd-Warshall result for Graph 0 is incorrect.")

    def test_floyd_warshall_disconnected_graph(self):
        current = graph_data.graph_data[1]
        graph, parent = f_w.floyd_warshall(current)

        expected = [
            [0, 1, float('inf'), float('inf')],
            [1, 0, float('inf'), float('inf')],
            [float('inf'), float('inf'), 0, 2],
            [float('inf'), float('inf'), 2, 0]
        ]
        self.assertEqual(graph, expected, "Floyd-Warshall result for Graph 1 is incorrect.")

    def test_floyd_warshall_weighted_graph(self):
        current = graph_data.graph_data[2]
        graph, parent = f_w.floyd_warshall(current())

        expected = [
            [0, 4, 1, 6],
            [4, 0, 5, 2],
            [1, 5, 0, 3],
            [6, 2, 3, 0]
        ]
        self.assertEqual(graph, expected, "Floyd-Warshall result for Graph 2 is incorrect.")

if __name__ == '__main__':
    unittest.main()
