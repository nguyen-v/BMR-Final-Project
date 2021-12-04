## 
# @file compute_global_path.py
#
# @brief Global navigation for Thymio.
# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

## Recurrently reconstructs the path from start node to the current node
# @param    cameFrom                map (dictionary) containing for each node n the node immediately 
#                                   preceding it on the cheapest path from start to n 
#                                   currently known.
# @param    current                 current node (x, y)
# @return   total_path              list of nodes from start to current node
def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom.keys():
        # Add where the current node came from to the start of the list
        total_path.insert(0, cameFrom[current]) 
        current=cameFrom[current]
    return total_path


## A* for 2D occupancy grid. Finds a path from start to goal. h is the heuristic function.
#                                   h(n) estimates the cost to reach goal from node n.
# @param     start                  start node (x, y)
# @param     goal_m                 goal node (x, y)
# @param     occupancy_grid         the grid map
# @return    thymio_path            a tuple that contains the resulting path in data array indices (list of tuples)
# @return    path_found             a boolean that indicates if a path was found
def A_Star(start, goal, occupancy_grid):

    max_val_x, max_val_y = occupancy_grid.shape

    # Check if the start and goal are allowed points
    for point in [start, goal]:
        assert point[0] >= 0 and point[0] < max_val_x and point[1] >= 0 and point[1] < max_val_y , "start or end goal not contained in the map"
    if occupancy_grid[start[0], start[1]]:
        raise Exception('Start node is not traversable')
    if occupancy_grid[goal[0], goal[1]]:
        raise Exception('Goal node is not traversable')
    
    # Movements for 8-connectivity 
    s2 = math.sqrt(2)
    movements = [(1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0), (1, 1, s2), (-1, 1, s2), (-1, -1, s2), (1, -1, s2)]
    
    # The set of visited nodes that need to be (re-)expanded
    openSet = [start]
    # The set of visited nodes that have been expanded.
    closedSet = []
    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
    cameFrom = dict()
    
    # Create the set of all points in the map, used as keys for the score dicts
    x,y = np.mgrid[0:max_val_x:1, 0:max_val_y:1]
    pos = np.empty(x.shape + (2,))
    pos[:, :, 0] = x
    pos[:, :, 1] = y
    pos = np.reshape(pos, (x.shape[0]*x.shape[1], 2))
    coords = list([(int(x[0]), int(x[1])) for x in pos])

    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    # Heuristic function; here, L2-distance to the goal ignoring obstacles
    # Values will be computed only if necessary
    # hScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    hScore = dict.fromkeys(coords)
    # For node n, fScore[n] := gScore[n] + h(n). map with default value of Infinity
    fScore = dict(zip(coords, [np.inf for x in range(len(coords))]))

    # Initialize the start scores:
    gScore[start] = 0
    hScore[start] = np.linalg.norm(np.asarray(start) - np.asarray(goal), axis=-1)
    fScore[start] = hScore[start]

    # while there are still elements to investigate
    while openSet != []:
        # pick the node in openSet with the lowest fScore[] value
        fScore_openSet = {key:val for (key,val) in fScore.items() if key in openSet}
        current = min(fScore_openSet, key=fScore_openSet.get)
        del fScore_openSet

        #If the goal is reached, reconstruct and return the obtained path
        if current == goal:
            return reconstruct_path(cameFrom, current), True
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
            # tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore = gScore[current] + deltacost
            if neighbor not in openSet:
                openSet.append(neighbor)
            if tentative_gScore < gScore[neighbor]:
                # This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                if hScore[neighbor] is None: # calculate hScore only if hasn't been done before
                    hScore[neighbor] = np.linalg.norm(np.asarray(neighbor) - np.asarray(goal), axis=-1)
                fScore[neighbor] = gScore[neighbor] + hScore[neighbor]
                
    # Open set is empty but goal was never reached
    print("No path found to goal")
    return [], False
