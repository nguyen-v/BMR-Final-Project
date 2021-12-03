
import math 
import numpy as np

from img_utils import get_color_dots


#BGR2HSV gives value from 0 to 180 fro the first component

GREEN_THR_HSV_HIGH = (60, 255, 255)
GREEN_THR_HSV_LOW = (45, 110, 110)

BLUE_THR_HSV_HIGH = (100, 255, 255)
BLUE_THR_HSV_LOW = (80, 110, 110)

PURPLE_THR_HSV_HIGH = (145, 255, 255)
PURPLE_THR_HSV_LOW = (125, 50, 50)


#thymio's rotaion point is blue, thymio's "direction point" is purple, goal is green;

# transform (x,y) coordinates into grid coordinates
#  @param coords         cartesian coordinates
#  @param map_size       size of the map in pixels (size(x),size(y))
#                        cartesian will return (x,y) coordinates
#                        grid will return (column, line) coordinates
#  @param grid_size      size of the grid (nb_columns,nb_lines)
#  @return grid_coords   pos in the grid
def cartesian_to_grid(coords,map_size,grid_size):
    grid_coords = (math.floor(coords[0]*grid_size[0]/map_size[0]),math.floor(coords[1]*grid_size[1]/map_size[1]))
    return grid_coords


## Returns position of the thymio if found
#  @param rectified_img  Array containing each pixels of the rectified image from the camera 
#  @param coord_type     string variable that can contain "cartesian" or "grid"
#                        cartesian will return (x,y) coordinates
#                        grid will return (column,line) coordinates
#  @param grid_size      size of the grid (nb_colums,nb_lines)
#  @return thymio_pose   gives thymio (x,y, angle) or (column,line, angle) coordinates
#  @return found_thymio  (bool) returns true if thymio was found, false otherwise
def locate_thymio_camera(rectified_img,coord_type, grid_size):

    thymio_rot_point, found_rot_point = get_color_dots(rectified_img, BLUE_THR_HSV_LOW, BLUE_THR_HSV_HIGH, 1)
    thymio_dir_point, found_dir_point = get_color_dots(rectified_img, PURPLE_THR_HSV_LOW, PURPLE_THR_HSV_HIGH, 1)

    if(found_rot_point and found_dir_point):
        thymio_rot_point = thymio_rot_point[0]
        thymio_dir_point = thymio_dir_point[0]
        # angle is zero on the right, goes to pi counter clockwise, goes to -pi counter clockwise
        angle = math.atan2(-(thymio_dir_point[1]-thymio_rot_point[1]), thymio_dir_point[0] - thymio_rot_point[0]) 
        if(coord_type == 'cartesian'):
            thymio_pose = np.append(thymio_rot_point,angle)
            return thymio_pose, True
        else:
            map_size = (np.size(rectified_img, 1),np.size(rectified_img, 0))
            thymio_coords  = cartesian_to_grid(thymio_rot_point,map_size,grid_size)
            thymio_pose = np.append(thymio_coords,angle)
            return thymio_pose, True   
    else:
        return [], False
    
## Returns position of the goal if found
#  @param rectified_img  Array containing each pixels of the rectified image from the camera 
#  @param coord_type     string variable that can contain "cartesian" or "grid"
#                        cartesian will return (x,y) coordinates
#                        grid will return (column,line) coordinates
#  @param grid_size      size of the grid (nb_columns,nb_lines)
#  @return goal_coords   gives goal (x,y) or (column,line, angle) coordinates
#  @return found_goal   (bool) returns true if goal was found, false otherwise 
def locate_goal_camera(rectified_img,coord_type, grid_size):
    
    goal_coords, found_goal = get_color_dots(rectified_img, GREEN_THR_HSV_LOW, GREEN_THR_HSV_HIGH, 1)
    goal_coords = goal_coords[0]
    
    if(found_goal):
        if(coord_type == 'cartesian'):
            return goal_coords, True
        else:
            map_size = (np.size(rectified_img, 1),np.size(rectified_img, 0))
            goal_coords  = cartesian_to_grid(goal_coords,map_size,grid_size)
            return goal_coords, True   
    else:
        return [], False 

    