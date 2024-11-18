import math
import unittest

from numpy.random import permutation

import global_game_data
import graph_data
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

    def test_permutations_are_correct(self):
        graph = graph_data.graph_data[0]

        all_perms = find_permutations(graph)
        expected_perms = [[0, 1, 2], [0, 2, 1], [2, 0, 1], [2, 1, 0], [1, 2, 0], [1, 0, 2], [2, 0, 1]]
        self.assertEqual(all_perms, expected_perms, "Permutations do not match the expected list.")

    def test_there_are_no_hamiltonian(self):
        graph = graph_data.graph_data[0]
        permutations = find_permutations(graph)

        has_hamiltonian_cycle, hamiltonian_cycles = validate_hamiltonian_cycle(graph, permutations)

        self.assertFalse(has_hamiltonian_cycle, "Expected no Hamiltonian cycle, but one was found.")


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
            neighbors = graph[nodeA][1]  # Get neighbors of nodeA
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
            neighbors = graph[nodeA][1]  # Get neighbors of nodeA
            self.assertIn(nodeB, neighbors, f"BFS: No edge exists between {nodeA} and {nodeB}.")

if __name__ == '__main__':
    unittest.main()
