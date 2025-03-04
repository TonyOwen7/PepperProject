university_matrix = [
    [0, 2, 2, 0, 1, 0],
    [0, 1, 0, 0, 1, 0],
    [0, 1, 0, 2, 1, 2],
    [0, 0, 0, 0, 0, 0],
    [2, 1, 0, 1, 1, 2]
]

location_queries = {
            "classe 1": (0, 1),
            "classe 2": (0, 2),
            "classe 3": (2, 3),
            "toilette": (2, 5),
            "escalier": (4, 0),
            "bureau des enseignants": (4, 5)
            "Accueil" : (0 ,0)
        }

DIRECTIONS = {
    'down': (1, 0),
    'up': (-1, 0),
    'left': (0, -1),
   'right': (0, 1),  
}

pepper_position = [4, 5]
pepper_direction = ["left"]