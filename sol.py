from waypoint import Waypoint
from collections import defaultdict

class PathFinder(object):
    def __init__(self):
        #Transition representing the 4 directions and for each direction the possible 6 movements
        self._movements = {0: [(-1,1,3),(0,1,0),(1,1,1),(-1,-1,1),(0,-1,0),(1,-1,3)], 
                            1: [(1,1,0),(1,0,1),(1,-1,2),(-1,1,2),(-1,0,1),(-1,-1,0)], 
                            2: [(-1,-1,3),(0,-1,2),(1,-1,1),(-1,1,1),(0,1,2),(1,1,3)],  
                            3: [(-1,-1,2),(-1,0,3),(-1,1,0),(1,-1,0),(1,0,3),(1,1,2)]}

    def get_path(self, grid, start_wp, end_wp):
        """Returns a list of Waypoints from the start Waypoint to the end Waypoint. 
        :param grid: Grid is a 2D numpy ndarray of boolean values. grid[x, y] == True if the cell contains an obstacle. The grid dimensions are exposed via grid.shape
        :param start_wp: The Waypoint that the path should start from.
        :param end_wp: The Waypoint that the path should end on.
        :return: The path from the start waypoint to the end waypoint that follows the movement model without going
        off the grid or intersecting an obstacle.
        :rtype: A list of Waypoints.
        """

        #Keeping track of the visied paths so the loader doesn't revisit previous areas or go in a loop
        visited = defaultdict(lambda:defaultdict(bool))
        visited[(start_wp.x,start_wp.y)][start_wp.orientation] = True

        #List to store the final WayPoints
        result = []
        #Queue to keep track of the paths currently being explored
        queue = [[start_wp]]
        
        while queue:
            path = queue.pop(0)
            cell = path[-1]

            #If last visited area of the path is the end way point. Save this as the result path and exit loop
            if cell == end_wp:
                result = path
                break
           
            #Retrieving the possible movements for the loader's current orientationu
            moves = self._movements[cell.orientation]
            for x,y,o in moves:
                newX,newY = cell.x + x, cell.y + y 
                if 0 <= newX < grid.shape[0] and 0 <=  newY < grid.shape[1] and not grid[newX, newY]:
                    if not visited[(newX,newY)].get(o,False):
                        newPath = list(path)
                        newPath.append(Waypoint(newX,newY,o))
                        queue.append(newPath)
                        visited[(newX,newY)][o] = True

        return result
        
'''
Time Complexity: O(RC)
R being the Rows in the grid and C being the Columns in the grid, since it is a 2D matrix.
This is because in the worst case the get_path function would have to traverse through the whole grid in order to find the path between the two spoil piles.
'''