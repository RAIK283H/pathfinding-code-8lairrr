import math
import pyglet
import colors
import config_data
import global_game_data
import graph_data


class Scoreboard:
    player_name_display = []
    player_traveled_display = []
    player_excess_distance_display = []
    player_path_display = []
    player_nodes_visited_display = [] #my statistic yay
    winner_display = None #stores the winner
    total_distances = []  # keeps track of total distance for each player

    def __init__(self, batch, group):
        self.batch = batch
        self.group = group
        self.stat_height = 20
        self.stat_width = 400
        self.number_of_stats = 6
        self.base_height_offset = 20
        self.font_size = 16
        self.distance_to_exit_label = pyglet.text.Label('Direct Distance To Exit : 0', x=0, y=0,
                                                        font_name='Arial', font_size=self.font_size, batch=batch, group=group)
        self.distance_to_exit = 0
        for index, player in enumerate(config_data.player_data):
            player_name_label = pyglet.text.Label(str(index + 1) + " " + player[0],
                                                  x=0,
                                                  y=0,
                                                  font_name='Arial',
                                                  font_size=self.font_size, batch=batch, group=group, color=player[2][colors.TEXT_INDEX])
            self.player_name_display.append((player_name_label, player))
            traveled_distance_label = pyglet.text.Label("Distance Traveled:",
                                                        x=0,
                                                        y=0,
                                                        font_name='Arial',
                                                        font_size=self.font_size, batch=batch, group=group, color=player[2][colors.TEXT_INDEX])
            self.player_traveled_display.append(
                (traveled_distance_label, player))
            excess_distance_label = pyglet.text.Label("Excess Distance Traveled:",
                                                      x=0,
                                                      y=0,
                                                      font_name='Arial',
                                                      font_size=self.font_size, batch=batch, group=group, color=player[2][colors.TEXT_INDEX])

            self.player_excess_distance_display.append(
                (excess_distance_label, player))
            path_label = pyglet.text.Label("",
                                   x=0,
                                   y=0,
                                   font_name='Arial',
                                   font_size=self.font_size, batch=batch, group=group, color=player[2][colors.TEXT_INDEX])
            self.player_path_display.append(
                (path_label, player))
            
            #label for my statistic
            nodes_visited_label = pyglet.text.Label("Nodes Visited: 0",
                                                    x=0,
                                                    y=0,
                                                    font_name='Arial',
                                                    font_size=self.font_size, batch=batch, group=group, color=player[2][colors.TEXT_INDEX])
            self.player_nodes_visited_display.append((nodes_visited_label, player))

        self.winner_display = pyglet.text.Label("Winner: None", x=0, y=0,
                                                font_name='Arial', font_size=self.font_size, batch=batch, group=group)

    def update_elements_locations(self):
        self.distance_to_exit_label.x = config_data.window_width - self.stat_width
        self.distance_to_exit_label.y = config_data.window_height - self.stat_height;
        for index, (display_element, player) in enumerate(self.player_name_display):
            display_element.x = config_data.window_width - self.stat_width
            display_element.y = config_data.window_height - self.base_height_offset - self.stat_height * 2 - self.stat_height * (index * self.number_of_stats)
        for index, (display_element, player) in enumerate(self.player_traveled_display):
            display_element.x = config_data.window_width - self.stat_width
            display_element.y = config_data.window_height - self.base_height_offset - self.stat_height * 3 - self.stat_height * (index * self.number_of_stats)
        for index, (display_element, player) in enumerate(self.player_excess_distance_display):
            display_element.x = config_data.window_width - self.stat_width
            display_element.y = config_data.window_height - self.base_height_offset - self.stat_height * 4 - self.stat_height * (index * self.number_of_stats)
        for index, (display_element, player) in enumerate(self.player_path_display):
            display_element.x = config_data.window_width - self.stat_width
            display_element.y = config_data.window_height - self.base_height_offset - self.stat_height * 5 - self.stat_height * (index * self.number_of_stats)
        for index, (display_element, player) in enumerate(self.player_nodes_visited_display): #updating the element for all of my nodes
            display_element.x = config_data.window_width - self.stat_width
            display_element.y = config_data.window_height - self.base_height_offset - self.stat_height * 6 - self.stat_height * (index * self.number_of_stats)

        # update the winner display location
        self.winner_display.x = config_data.window_width - self.stat_width
        self.winner_display.y = config_data.window_height - self.base_height_offset - self.stat_height * 7 - self.stat_height * (
                    len(self.player_name_display) * self.number_of_stats)

    def update_paths(self):
        for index in range(len(config_data.player_data)):
            self.player_path_display[index][0].text = self.wrap_text(str(global_game_data.graph_paths[index]))

    def update_distance_to_exit(self):
        start_x = graph_data.graph_data[global_game_data.current_graph_index][0][0][0]
        start_y = graph_data.graph_data[global_game_data.current_graph_index][0][0][1]
        end_x = graph_data.graph_data[global_game_data.current_graph_index][-1][0][0]
        end_y = graph_data.graph_data[global_game_data.current_graph_index][-1][0][1]
        self.distance_to_exit = math.sqrt(pow(start_x - end_x, 2) + pow(start_y - end_y, 2))
        self.distance_to_exit_label.text = 'Direct Distance To Exit : ' + "{0:.0f}".format(self.distance_to_exit)


    def wrap_text(self, input):
        wrapped_text = (input[:44] + ', ...]') if len(input) > 44 else input
        return wrapped_text

    def update_distance_traveled(self):
        self.total_distances = []  #reset the total distances list
        index = 0

        for display_element, player_configuration_info in self.player_traveled_display:
            for player_object in global_game_data.player_objects:
                if player_object.player_config_data == player_configuration_info:
                    path = global_game_data.graph_paths[index]
                    graph = graph_data.graph_data[global_game_data.current_graph_index]
                    distance_traveled = 0

                    if path and len(path) > 1:
                        for i in range(len(path) - 1):
                            node_a = graph[path[i]][0]  #coords of current node
                            node_b = graph[path[i + 1]][0]  #coords of next node
                            distance_traveled += math.sqrt(
                                (node_a[0] - node_b[0]) ** 2 + (node_a[1] - node_b[1]) ** 2
                            )

                    # calculating total distance
                    total_distance = distance_traveled + self.distance_to_exit
                    self.total_distances.append(total_distance)

                    #updating display
                    player_object.distance_traveled = total_distance
                    display_element.text = f"Distance Traveled: {int(distance_traveled)}"

                    index += 1 #incrementing

        for display_element, player_configuration_info in self.player_excess_distance_display:
            for player_object in global_game_data.player_objects:
                if player_object.player_config_data == player_configuration_info:
                    display_element.text = "Excess Distance Traveled: " + str(max(0, int(player_object.distance_traveled-self.distance_to_exit)))

        # updating nodes visited for each player
        for display_element, player_configuration_info in self.player_nodes_visited_display:
            for player_object in global_game_data.player_objects:
                if player_object.player_config_data == player_configuration_info:
                    display_element.text = "Nodes Visited: " + str(player_object.nodes_visited)

        self.update_winner(self.total_distances)

    def update_winner(self, total_distances):
        if self.winner_display.text != "Winner: None": #if there is already a winner displayed, then dont reset the display
            return

        min_distance = math.inf
        winner_index = None
        for index, distance in enumerate(total_distances):
            if distance < min_distance:  #compare each distance
                min_distance = distance
                winner_index = index

        winner_name = config_data.player_data[winner_index][0]  #get the winners name
        self.winner_display.text = f"Winner: {winner_name} (Distance: {min_distance:.2f})"

    def update_scoreboard(self):
        self.update_elements_locations()
        self.update_paths()
        self.update_distance_to_exit()
        self.update_distance_traveled()
