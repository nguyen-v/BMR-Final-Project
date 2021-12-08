import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
import math
from create_map import *
from locate_thymio_goal import *
from rdp import rdp
from img_utils import *

## All nodes on the path that are within this range of initial position of Thymio are ignored
NODE_DIST_THR = 50

def _get_movements_4n():
    """
    Get all possible 4-connectivity movements (up, down, left right).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0)]

def _get_movements_8n():
    """
    Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1)
    (up, down, left, right and the 4 diagonals).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]



def reconstruct_path(cameFrom, current):
    """
    Recurrently reconstructs the path from start node to the current node
    :param cameFrom: map (dictionary) containing for each node n the node immediately 
                     preceding it on the cheapest path from start to n 
                     currently known.
    :param current: current node (x, y)
    :return: list of nodes from start to current node
    """
    total_path = [current]
    while current in cameFrom.keys():
        # Add where the current node came from to the start of the list
        total_path.insert(0, cameFrom[current]) 
        current=cameFrom[current]
    return total_path

def A_Star(start, goal, h, coords, occupancy_grid, movement_type="4N", map_width = MAP_WIDTH_CELL, map_height = MAP_HEIGHT_CELL):
    """
    A* for 2D occupancy grid. Finds a path from start to goal.
    h is the heuristic function. h(n) estimates the cost to reach goal from node n.
    :param start: start node (x, y)
    :param goal_m: goal node (x, y)
    :param occupancy_grid: the grid map
    :param movement: select between 4-connectivity ('4N') and 8-connectivity ('8N', default)
    :return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
    """
    
    # -----------------------------------------
    # DO NOT EDIT THIS PORTION OF CODE
    # -----------------------------------------
    
    # Check if the start and goal are within the boundaries of the map
    for point in [start, goal]:
        # for coord in point:
        #     assert coord>=0 and coord<max_val, "start or end goal not contained in the map"
        if point[0] < 0 and point[0] >= map_height and point[1] < 0 and point[1] >= map_width:
            print("Start or end goal not contained in the map")
            return [], []
    
    # check if start and goal nodes correspond to free spaces
    if occupancy_grid[start[0], start[1]]:
        print("Start node is not traversable")
        return [], []
    if occupancy_grid[goal[0], goal[1]]:
        print("Goal node is not traversable")
        return [], []
    
    # get the possible movements corresponding to the selected connectivity
    if movement_type == '4N':
        movements = _get_movements_4n()
    elif movement_type == '8N':
        movements = _get_movements_8n()
    else:
        raise ValueError('Unknown movement')
    
    # --------------------------------------------------------------------------------------------
    # A* Algorithm implementation - feel free to change the structure / use another pseudo-code
    # --------------------------------------------------------------------------------------------
    
    # The set of visited nodes that need to be (re-)expanded, i.e. for which the neighbors need to be explored
    # Initially, only the start node is known.
    openSet = [start]
    
    # The set of visited nodes that no longer need to be expanded.
    closedSet = []

    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
    cameFrom = dict()

    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    gScore[start] = 0

    # For node n, fScore[n] := gScore[n] + h(n). map with default value of Infinity
    fScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    fScore[start] = h[start]

    # while there are still elements to investigate
    while openSet != []:
        
        #the node in openSet having the lowest fScore[] value
        fScore_openSet = {key:val for (key,val) in fScore.items() if key in openSet}
        current = min(fScore_openSet, key=fScore_openSet.get)
        del fScore_openSet
        
        #If the goal is reached, reconstruct and return the obtained path
        if current == goal:
            return reconstruct_path(cameFrom, current), closedSet

        openSet.remove(current)
        closedSet.append(current)
        
        #for each neighbor of current:
        for dx, dy, deltacost in movements:
            
            neighbor = (current[0]+dx, current[1]+dy)
            
            # if the node is not in the map, skip
            if (neighbor[0] >= occupancy_grid.shape[0]) or (neighbor[1] >= occupancy_grid.shape[1]) or (neighbor[0] < 0) or (neighbor[1] < 0):
                continue
            
            # if the node is occupied or has already been visited, skip
            if (occupancy_grid[neighbor[0], neighbor[1]]) or (neighbor in closedSet): 
                continue
                
            # d(current,neighbor) is the weight of the edge from current to neighbor
            # tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore = gScore[current] + deltacost
            
            if neighbor not in openSet:
                openSet.append(neighbor)
                
            if tentative_gScore < gScore[neighbor]:
                # This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + h[neighbor]

    # Open set is empty but goal was never reached
    print("No path found to goal")
    return [], closedSet

## Returns global path to goal
#  @param       map_enlarged    Enlarged map (cells).
#  @param       start           Start cell (line, colum).
#  @param       goal            Goal cell (line column).
#  @return      path            A list of cells [(line, column), (line, column), ...] defining the global path.
#  @return      path_found      True if path is found, false if it is not.
def get_global_path(map_enlarged, start, goal):
    occupancy_grid = map_enlarged
    # List of all coordinates in the grid
    x,y = np.mgrid[0:MAP_HEIGHT_CELL:1, 0:MAP_WIDTH_CELL:1]
    pos = np.empty(x.shape + (2,))
    pos[:, :, 0] = x; pos[:, :, 1] = y
    pos = np.reshape(pos, (x.shape[0]*x.shape[1], 2))
    coords = list([(int(x[0]), int(x[1])) for x in pos])

    # Define the heuristic, here = distance to goal ignoring obstacles
    h = np.linalg.norm(pos - goal, axis=-1)
    h = dict(zip(coords, h))

    # Run the A* algorithm
    path, visitedNodes = A_Star(start, goal, h, coords, occupancy_grid, movement_type="8N", map_width = MAP_WIDTH_CELL, map_height = MAP_HEIGHT_CELL)
    if path == []:
        return [], False
    path = np.array(path)
    visitedNodes = np.array(visitedNodes)
    return path, True

def lin_refine_implicit(x, n):
    """
    Given a 2D ndarray (npt, m) of npt coordinates in m dimension, insert 2**(n-1) additional points on each trajectory segment
    Returns an (npt*2**(n-1), m) ndarray
    """
    if n > 1:
        m = 0.5*(x[:-1] + x[1:])
        if x.ndim == 2:
            msize = (x.shape[0] + m.shape[0], x.shape[1])
        else:
            raise NotImplementedError

        x_new = np.empty(msize, dtype=np.float64)
        x_new[0::2] = x
        x_new[1::2] = m
        return lin_refine_implicit(x_new, n-1)
    elif n == 1:
        return x
    else:
        raise ValueError


def simplify_path(path, rect_height, rect_width, thymio_pos):
    # # Simplify path
    # path = rdp(path, epsilon=0.5)

    # # Add intermediate points to path 
    # path = lin_refine_implicit(path, n=5)

    # Convert path to a list of (x, y) positions
    path_temp = path + 1/2
    path[:, 0] = path_temp[:, 1]*(rect_height/MAP_HEIGHT_CELL)
    path[:, 1] = path_temp[:, 0]*(rect_width/MAP_WIDTH_CELL)

    # Delete first nodes if too close to Thymio
    for idx, node in enumerate(path):
        if dist(node, thymio_pos[0:2]) < NODE_DIST_THR:
            path = np.delete(path, idx, 0)
    return path


def init_path(cam, thymio, clear_start_node = False):
    M, rect_width, rect_height, map, map_enlarged = init_map(cam)
    img, img_taken = take_picture(cam)

    found_path = False
    thymio_pos = []
    obj_pos = []
    path = []
    while not found_path:
        # Find Thymio
        thymio_found = False
        while not thymio_found:
            img, img_taken = take_picture(cam)
            if img_taken:
                img_rect = get_rectified_img(img, M, rect_width, rect_height)
                thymio_pos, thymio_found = locate_thymio_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
        thymio.set_last_angle(thymio_pos[2])
        print("Thymio found.")

        # Find objective
        obj_found = False
        while not obj_found:
            img, img_taken = take_picture(cam)
            if img_taken:
                img_rect = get_rectified_img(img, M, rect_width, rect_height)
                obj_pos, obj_found = locate_goal_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))

        print("Objective found")
        # Compute global path to objective
        print("Computing global path")
        thymio_pos_grid = cartesian_to_grid(thymio_pos[0:2], (rect_width, rect_height), (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
        obj_pos_grid = cartesian_to_grid(obj_pos, (rect_width, rect_height), (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
        if clear_start_node == True: # this option is useful when recalculating path because thyimo might be close to an obstacle
            map_enlarged[thymio_pos_grid[0]][thymio_pos_grid[1]] = 0
        path, found_path = get_global_path(map_enlarged, thymio_pos_grid, obj_pos_grid)
        if not found_path:
            time.sleep(1)
            M, rect_width, rect_height, map, map_enlarged = init_map(cam)
            plt.figure()
            plt.imshow(map_enlarged, origin = 'lower', cmap = 'Greys', interpolation = 'nearest')
            plt.title("Original Map")
            plt.gca().invert_yaxis()
            plt.show()
    path = simplify_path(path, rect_height, rect_width, thymio_pos)
    return path, thymio_pos, obj_pos