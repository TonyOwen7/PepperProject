from collections import deque
from mymap import DIRECTIONS
import numpy as np
from numpy import inf 

university_matrix = [
    [0, 2, 2, 0, 1, 0],
    [0, 1, 0, 0, 1, 0],
    [0, 1, 0, 2, 1, 0],
    [0, 0, 0, 0, 0, 0],
    [1, 1, 0, 1, 1, 2]
]

def create_adjacency_matrix(mymap, start, destination):
    nb_lines = len(mymap)
    nb_columns = len(mymap[0])
    
    adjacency_matrix =[[inf for x in range(nb_lines * nb_columns)] for x in range(nb_lines * nb_columns)]
    
    nb_nodes = len(adjacency_matrix)
    
    i = 0
    j = 0
    for i in range(nb_nodes):
       
        for j in range(nb_nodes):   
            if i == j:
                adjacency_matrix[i][j] = 0
                continue
            
            
            if mymap[i//nb_columns][i%nb_columns] == 0 or (i//nb_columns == start[0] and i%nb_columns== start[1]) or (i//nb_columns == destination[0] and i%nb_columns== destination[1]):
                if mymap[j//nb_columns][j%nb_columns] == 0 or (j//nb_columns == start[0] and j%nb_columns== start[1]) or (j//nb_columns == destination[0] and j%nb_columns== destination[1]):
                    if abs(i//nb_columns - j//nb_columns) + abs(i%nb_columns - j%nb_columns) == 1:
                        adjacency_matrix[i][j] = 1
                
         
        

    return adjacency_matrix            
                        
def initialisation(nb_nodes, start):
    matrix_shortest_path = [[inf] * nb_nodes]
    matrix_shortest_path.append([-1 for x in range(nb_nodes)])
    matrix_shortest_path[0][start] = 0
    
    return matrix_shortest_path
    
    
    
def update(adjacency_matrix, matrix_shortest_path, predecessor, sucessor):
    if matrix_shortest_path[0][sucessor] > (matrix_shortest_path[0][predecessor] + adjacency_matrix[sucessor][predecessor]):
        matrix_shortest_path[0][sucessor] = (matrix_shortest_path[0][predecessor] + adjacency_matrix[sucessor][predecessor])
        matrix_shortest_path[1][sucessor] = predecessor

        
def find_shortests_path(mymap, matrix_shortest_path, shortest_paths, path, goal, nb_nodes):
    if path[-1][0]  == goal[0] and path[-1][1]  == goal[1]:   
        shortest_paths.append(path)
    else:
        current_predecessor = path[-1][0]*len(mymap[0]) + path[-1][1]
        for node in range(nb_nodes):
                if matrix_shortest_path[1][node] == current_predecessor:
                    find_shortests_path(mymap, matrix_shortest_path, shortest_paths, path + [((node//len(mymap[0]), node%len(mymap[0])))], goal, nb_nodes)
        
def bfs(mymap, start, goal):
    index_start = start[0]*len(mymap[0]) + start[1]
    
    current_node = index_start
    
    adjacency_matrix = create_adjacency_matrix(mymap, start, goal)
    
    nb_nodes = len(adjacency_matrix) 
    
    mark = [False for x in range(nb_nodes)]
    mark[index_start] = True
    
    matrix_shortest_path = initialisation(nb_nodes, index_start)
    
    nb_nodes_marked = 0
    
    while nb_nodes_marked < nb_nodes:
        mark[current_node] = True
        nb_nodes_marked +=1
        
        for current_node_sucessor in range(nb_nodes):
            if mark[current_node_sucessor] == False:
                min = current_node_sucessor
                break
            
        for current_node_sucessor in range(nb_nodes):
            if adjacency_matrix[current_node][current_node_sucessor] == 1:
                if mark[current_node_sucessor] == False:
                    update(adjacency_matrix, matrix_shortest_path, current_node, current_node_sucessor)
                    
        for current_node_sucessor in range(nb_nodes):
            if mark[current_node_sucessor] == False:
                if matrix_shortest_path[0][min] > matrix_shortest_path[0][current_node_sucessor]:
                    min = current_node_sucessor
                
                        
        current_node = min
        
    shortest_paths = []
    
    find_shortests_path(mymap, matrix_shortest_path, shortest_paths, [start], goal, nb_nodes)
                                
    return shortest_paths
