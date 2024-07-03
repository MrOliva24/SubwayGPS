# This file contains all the required routines to make an A* search algorithm.
#
__author__ = '1668300'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Curs 2023 - 2024
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy
from itertools import permutations


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """
    cons = map.connections.get(path.last)
    path_list = []

    for station in cons.keys():
        newRoute = path.route.copy()
        newRoute.append(station)
        newPath = Path(newRoute)
        newPath.update_g(path.g)
        path_list.append(newPath)
    
    return path_list

        
def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """
    goodList = []
    length = len(path_list[0].route)
    for path in path_list:
        if len(set(path.route)) == length:
            goodList.append(path)
    return goodList 


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """

    list_of_path.pop(0)
    return expand_paths + list_of_path


def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """
    
    list_of_path = [Path([origin_id])]
    
    while len(list_of_path) > 0 and (list_of_path[0].last != destination_id) :
        C = list_of_path[0]
        E = expand(C, map)
        E = remove_cycles(E)
        list_of_path = insert_depth_first_search(E, list_of_path)

    if(len(list_of_path) !=0): return (list_of_path[0])
    else: return "Solution does not exist"



def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    
    """
    if len(list_of_path) == 1:
        return expand_paths
    list_of_path.pop(0)
    return list_of_path + expand_paths
    
    

def breadth_first_search(origin_id, destination_id, map : Map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_path = [Path(origin_id)]
    while len(list_of_path) > 0 and list_of_path[0].last != destination_id:
        C = list_of_path[0]
        E = expand(C, map)
        E = remove_cycles(E)
        list_of_path = insert_breadth_first_search(E, list_of_path)
    
    if(len(list_of_path) !=0): return (list_of_path[0])
    else: return "Solution does not exist"
    

def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """
    match type_preference:
        case 0:
            for path in expand_paths:
                path.update_g(1)
        case 1:
            for path in expand_paths:
                path.update_g(map.connections[path.penultimate][path.last])
        case 2:
            for path in expand_paths:
                if (map.stations[path.last]['name'] != map.stations[path.penultimate]['name']):
                    path.update_g(
                        map.stations[path.penultimate]['velocity'] * map.connections[path.last][path.penultimate]
                    )
        case 3:
            for path in expand_paths:
                if (map.stations[path.penultimate]['line'] != map.stations[path.last]['line']):
                    path.update_g(1)
    return expand_paths


def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """
    if len(list_of_path) == 0:
        path = expand_paths.copy()
    else:
        path = expand_paths + list_of_path
    
    path.sort(key=lambda x: [x.g, len(x.route)])
    return path


def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    
    list_of_path = [Path([origin_id])]
    while (list_of_path[0].last != destination_id) and list_of_path != None:
        C = list_of_path.pop(0)
        E = expand(C, map)
        E = remove_cycles(E)
        E = calculate_cost(E, map, type_preference)
        list_of_path = insert_cost(E, list_of_path)
    if len(list_of_path) != 0:
        return list_of_path[0]
    else:
        return "Solution does not exist"


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            destination_id (int): Final station id
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """
    match type_preference:
        case 0:
            for path in expand_paths:
                if path.last != destination_id:
                    path.update_h(1)
        case 1:
            a = (map.stations[destination_id]['x'], map.stations[destination_id]['y'])
            maximizer = -INF
            for key in map.stations.keys():
                v = map.stations[key]["velocity"]
                if maximizer < v:
                    maximizer = v
            v = maximizer
            
            for path in expand_paths:
                b = (map.stations[path.last]['x'], map.stations[path.last]['y'])
                distance = euclidean_dist(a, b)
                path.update_h(distance/v)
        case 2:
            a = (map.stations[destination_id]['x'], map.stations[destination_id]['y'])
            for path in expand_paths:
                b = (map.stations[path.last]['x'], map.stations[path.last]['y'])
                distance = euclidean_dist(a, b)
                path.update_h(distance)            
        case 3:
            for path in expand_paths:
                if map.stations[destination_id]['line'] != map.stations[path.last]['line']:
                    path.update_h(1)
    return expand_paths
            
        


def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    for path in expand_paths:
        path.update_f()
    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g-cost at this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
             visited_stations_cost (dict): Updated visited stations cost
    """

    new_paths = [path for path in expand_paths if path.last not in visited_stations_cost or path.g < visited_stations_cost.get(path.last, INF)]
    visited_stations_cost.update({path.last: path.g for path in new_paths if path.last not in visited_stations_cost or path.g < visited_stations_cost[path.last]})

    list_of_path = [path for path in list_of_path if path.last not in visited_stations_cost or path.g <= visited_stations_cost[path.last]]

    return new_paths, list_of_path, visited_stations_cost



def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    new_path = expand_paths + list_of_path
    new_path.sort(key=lambda x: [x.f, len(x.route)])
    return new_path


def distance_to_stations(coord, map):
    """
        From coordinates, it computes the distance to all stations in map.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            (dict): Dictionary containing as keys, all the Indexes of all the stations in the map, and as values, the
            distance between each station and the coord point
    """
    distances = { k: math.sqrt((v['x'] - coord[0])**2 + (v['y'] - coord[1])**2) for k, v in map.stations.items()}
    return dict(sorted(distances.items(), key=lambda item: (item[1], item[0])))


def Astar(origin_id, destination_id, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    
    list_of_path = [Path([origin_id])]
    
    visited_station_cost = {}
           

    while (list_of_path[0].last != destination_id and len(list_of_path) !=0):
        C = list_of_path.pop(0)
        E = expand(C,map)
        E = remove_cycles(E)
  
        E = calculate_cost(E, map, type_preference)
        E = calculate_heuristics(E, map, destination_id,type_preference)
        E = update_f(E)
        
        E, list_of_path, visited_station_cost = remove_redundant_paths(E, list_of_path,visited_station_cost)
        list_of_path = insert_cost_f(E, list_of_path)
        
    
    if(len(list_of_path) !=0): return (list_of_path[0])
    else: return "Solution does not exist"

def Astar_improved(origin_coord, destination_coord, map):
    
    best_time = euclidean_dist(origin_coord, destination_coord) / 5

    all_possible_paths = [
        (origin_station, destination_station, Astar(origin_station, destination_station, map, 1))
        for destination_station, destination_coords in map.stations.items()
        for origin_station, origin_coords in map.stations.items()
        if destination_station != origin_station
    ]

    for orig_station, dest_station, current_path in all_possible_paths:
        destination_coordinates = [map.stations[dest_station]['x'], map.stations[dest_station]['y']]
        destination_cost = euclidean_dist(destination_coordinates, destination_coord) / 5
        origin_coordinates = [map.stations[orig_station]['x'], map.stations[orig_station]['y']]
        origin_cost = euclidean_dist(origin_coord, origin_coordinates) / 5
        current_path.update_g(destination_cost + origin_cost)
        current_path.update_h(0)
        current_path.update_f()

    filtered_paths = [path for _, _, path in all_possible_paths]
    optimal_path = min(filtered_paths, key=lambda path: path.f, default=None)

    if optimal_path and optimal_path.f < best_time:
        final_route_sequence = [0] + optimal_path.route + [-1]
        final_path_result = Path(final_route_sequence)
        final_path_result.f = optimal_path.f
    else:
        final_path_result = Path([0, -1])
        final_path_result.f = best_time

    return final_path_result


    

