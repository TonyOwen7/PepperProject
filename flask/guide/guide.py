import sys
import os

move_path = os.path.join(os.path.dirname(__file__), '../move')
mymap_path = os.path.join(os.path.dirname(__file__), '../mymap')
shortest_path_path = os.path.join(os.path.dirname(__file__), '../shortest_path')
search_pattern_path = os.path.join(os.path.dirname(__file__), '../search_pattern')

sys.path.append(move_path)
sys.path.append(mymap_path)
sys.path.append(shortest_path_path)
sys.path.append(search_pattern_path)

from move import *
from mymap import university_matrix, DIRECTIONS, location_queries, pepper_direction, pepper_position
from shortest_path import *
from search_pattern import *


def clearup_sequence_of_moves(commands):
    file_of_commandes = deque()
    file_of_commandes.append([commands[0], 2])
    for command in commands[1:]:
        command_ , time = file_of_commandes.popleft()
        if command == command_:
            file_of_commandes.append([command_, time + 1])
        else:
            file_of_commandes.append([command_, time])
            file_of_commandes.append([command, 2])
        

def upgrade_position_and_direction(path):
    global pepper_position, pepper_direction
    commands = []
    for step in path:
        if pepper_direction[0] == "up":
            if step[0] < pepper_position[0]:
                commands.append("avancer")
            if step[0] > pepper_position[0]:
                commands.append("demi-tour gauche")
                commands.append("avancer")
            if step[1] < pepper_position[1]:
                pepper_direction[0] = "left"
                commands.append("tourner à gauche")
                commands.append("avancer")
            if step[1] > pepper_position[1]:
                pepper_direction[0] = "right"
                commands.append("tourner à droite")
                commands.append("aller à droite")

        elif pepper_direction[0] == "bottom":
            if step[0] < pepper_position[0]:
                commands.append("demi-tour gauche")
                commands.append("avancer")
            if step[0] > pepper_position[0]:
                commands.append("avancer")
            if step[1] < pepper_position[1]:
                pepper_direction[0] = "left"
                commands.append("tourner à droite")
                commands.append("avancer")
            if step[1] > pepper_position[1]:
                pepper_direction[0] = "right"
                commands.append("tourner à gauche")
                commands.append("avancer")

        elif pepper_direction[0] == "left":
            if step[0] < pepper_position[0]:
                pepper_direction[0] = "up"
                commands.append("tourner à droite")
                commands.append("avancer")
            if step[0] > pepper_position[0]:
                pepper_direction[0] = "bottom"
                commands.append("tourner à gauche")
                commands.append("avancer")
            if step[1] < pepper_position[1]:
                commands.append("avancer")
            if step[1] > pepper_position[1]:
                commands.append("demi-tour gauche")
                commands.append("avancer")

        elif pepper_direction[0] == "right":
            if step[0] < pepper_position[0]:
                pepper_direction[0] = "up"
                commands.append("tourner à gauche")
                commands.append("avancer")
            if step[0] > pepper_position[0]:
                pepper_direction[0] = "bottom"
                commands.append("tourner à droite")
                commands.append("avancer")
            if step[1] < pepper_position[1]:
                commands.append("demi-tour gauche")
                commands.append("avancer")
            if step[1] > pepper_position[1]:
                commands.append("avancer")

        pepper_position[0] = step[0]
        pepper_position[1] = step[1]
                                                  
    return commands

def guide(driver, location_queries):
    global university_matrix
    for location in location_queries:
        location_regex = re.sub(r"\s+", r"\\s+", location)

        if re.search(location_regex, normalized_input, re.IGNORECASE):
            row, col = location_queries[location]
            print(f"Location found: {location}, at row {row}, column {col}")
            location_found = True
            path = bfs(university_matrix, tuple(pepper_position), (row, col))
            if path:
                commands = upgrade_position_and_direction(path[0])
                
                commands = clearup_sequence_of_moves(commands)
                     
                for command in commands:
                    move(driver, command[0], command[1])
            else:
                print("Désolé, je ne peux pas atteindre cette destination.")
            break