import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Define a class to represent the 3D map
class Map:
    def __init__(self, obstacles):
        # Initialize the map with the given obstacles
        self.obstacles = obstacles

    def is_free(self, x, y, z):
        # Check if the given coordinates are free of obstacles
        for obstacle in self.obstacles:
            if x >= obstacle[0] and x <= obstacle[1] and y >= obstacle[2] and y <= obstacle[3] and z >= obstacle[
                4] and z <= obstacle[5]:
                return False
        return True

    def heuristic(self, x1, y1, z1, x2, y2, z2):
        # Calculate the heuristic cost between two points
        return abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2)


# Define a function to perform the A* search
def a_star_search(map, start, goal):
    # Create a list to store the visited nodes
    visited = []
    # Create a list of the remaining nodes to be explored
    remaining = [start]

    # Keep looping until there are no more nodes to explore
    while remaining:
        # Sort the remaining nodes by their heuristic cost
        remaining.sort(key=lambda x: map.heuristic(x[0], x[1], x[2], goal[0], goal[1], goal[2]))

        # Get the node with the lowest heuristic cost
        current = remaining.pop(0)

        # Check if we have reached the goal
        if current == goal:
            # Return the list of visited nodes if the goal is reached
            return visited

        # Check the neighboring nodes of the current node
        for x in [current[0] - 1, current[0] + 1]:
            for y in [current[1] - 1, current[1] + 1]:
                for z in [current[2] - 1, current[2] + 1]:
                    # Check if the neighboring node is free of obstacles and has not been visited
                    if map.is_free(x, y, z) and (x, y, z) not in visited:
                        # Add the node to the list of remaining nodes to be explored
                        remaining.append((x, y, z))

        # Add the current node to the list of visited nodes
        visited.append(current)

    # Return an empty list if the goal could not be reached
    return []

fig = plt.figure()
ax = plt.axes(projection='3d')



# Define the start and goal coordinates
start = (0, 0, 0)
goal1 = (15, 15, 15)
goal2 = (9, 9, 15)

# Define the obstacles on the map
obstacles = [(0, 3, 0, 3, 0, 3), (0, 4, 6, 9, 0, 10), (6, 9, 6, 8, 0, 12)]



# Create the map
map = Map(obstacles)

# Perform the A* search to find the shortest path
path1 = a_star_search(map, start, goal1)

path2 = a_star_search(map, start, goal2)
print(path2)

path_arr1 = np.asarray(path1)

path_arr2 = np.asarray(path2)




obstacles_coordinates = []

for obst in obstacles:
    for x in range(obst[0], obst[1] + 1):
        #print(x)
        for y in range(obst[2], obst[3] + 1):
            #print(y)
            for z in range(obst[4], obst[5] + 1):
                #print(z)
                obstacles_coordinates.append([x, y, z])



obstacles_coordinates_np = np.asarray(obstacles_coordinates)


ax.scatter(obstacles_coordinates_np[:,0], obstacles_coordinates_np[:,1], obstacles_coordinates_np[:,2], color='red')

ax.scatter(path_arr1[:,0], path_arr1[:,1], path_arr1[:,2], color = 'blue')
ax.scatter(path_arr2[:,0], path_arr2[:,1], path_arr2[:,2], color = 'green')

plt.show()