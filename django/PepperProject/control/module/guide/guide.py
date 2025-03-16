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

# from move import *
from mymap import university_matrix, DIRECTIONS, location_queries, pepper_direction, pepper_position
from shortest_path import *
from search_pattern import *
from collections import deque

def clearup_sequence_of_moves(commands):
    if not commands:
        return deque()  

    file_of_commandes = deque()
    file_of_commandes.append([commands[0], 0.5])  

    for command in commands[1:]:
        last_command, last_time = file_of_commandes[-1]  
        if command == last_command:
            file_of_commandes[-1][1] += 0.5
        else:
            file_of_commandes.append([command, 0.5])

    return file_of_commandes

def upgrade_position_and_direction(path):
    global pepper_position, pepper_direction
    commands = []
    for i in range(path):
        if i != 0 and path[i] == path[i - 1]:
            if pepper_direction[0] == "up":
                if path[i][1] < pepper_position[1]:
                    commands.append("avancer")
                if path[i][1] > pepper_position[1]:
                    pepper_direction[0] = "bottom"
                    commands.append("demi-tour gauche")
                    commands.append("avancer")
                if path[i][2] < pepper_position[2]:
                    pepper_direction[0] = "left"
                    commands.append("tourner à gauche")
                    commands.append("avancer")
                if path[i][2] > pepper_position[2]:
                    pepper_direction[0] = "right"
                    commands.append("tourner à droite")
                    commands.append("avancer")

            elif pepper_direction[0] == "bottom":
                if path[i][1] < pepper_position[1]:
                    pepper_direction[0] = "up"
                    commands.append("demi-tour gauche")
                    commands.append("avancer")
                if path[i][1] > pepper_position[1]:
                    commands.append("avancer")
                if path[i][2] < pepper_position[2]:
                    pepper_direction[0] = "left"
                    commands.append("tourner à droite")
                    commands.append("avancer")
                if path[i][2] > pepper_position[2]:
                    pepper_direction[0] = "right"
                    commands.append("tourner à gauche")
                    commands.append("avancer")

            elif pepper_direction[0] == "left":
                if path[i][1] < pepper_position[1]:
                    pepper_direction[0] = "up"
                    commands.append("tourner à droite")
                    commands.append("avancer")
                if path[i][1] > pepper_position[1]:
                    pepper_direction[0] = "bottom"
                    commands.append("tourner à gauche")
                    commands.append("avancer")
                if path[i][2] < pepper_position[2]:
                    commands.append("avancer")
                if path[i][2] > pepper_position[2]:
                    pepper_direction[0] = "right"
                    commands.append("demi-tour gauche")
                    commands.append("avancer")

            elif pepper_direction[0] == "right":
                if path[i][1] < pepper_position[1]:
                    pepper_direction[0] = "up"
                    commands.append("tourner à gauche")
                    commands.append("avancer")
                if path[i][1] > pepper_position[1]:
                    pepper_direction[0] = "bottom"
                    commands.append("tourner à droite")
                    commands.append("avancer")
                if path[i][2] < pepper_position[2]:
                    pepper_direction[0] = "left"
                    commands.append("demi-tour gauche")
                    commands.append("avancer")
                if path[i][2] > pepper_position[2]:
                    commands.append("avancer")

            pepper_position[1] = path[i][1]
            pepper_position[2] = path[i][2]
        else:
            commands.append("monter")
            pepper_position[0] = path[i][0]

                                                  
    return commands

def guide(driver, chosen_location, myMap):
    global university_matrix, location_queries, pepper_position, pepper_direction
    for location in location_queries:
        location_regex = re.sub(r"\s+", r"\\s+", location)

        if re.search(location_regex, chosen_location, re.IGNORECASE):
            floor, row, col = location_queries[location]
            print(f"Location found: {location}, at floor {floor} row {row}, column {col}")
            location_found = True
            path = bfs(myMap, tuple(pepper_position), (floor, row, col))

            if path:
                commands = upgrade_position_and_direction(path)
                
                commands = clearup_sequence_of_moves(commands)
                for command in commands:
                    move(driver, command[0], command[1])
            else:
                print("Désolé, je ne peux pas atteindre cette destination.")
            break
