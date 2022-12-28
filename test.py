import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
class Map:
    def __init__(self, obstacles):
        # Initialize the map with the given obstacles
        self.obstacles = obstacles

    def is_free(self, x, y, z):
        # Check if the given coordinates are free of obstacles
        for obstacle in self.obstacles:
            if x >= obstacle[0] and x <= obstacle[1] and y >= obstacle[2] and y <= obstacle[3] and z >= obstacle[4] and z <= obstacle[5]:
                return False
        return True

    def heuristic(self, x1, y1, z1, x2, y2, z2, wind_speed, wind_direction):
        # Calculate the heuristic cost between two points
        distance = abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2)

        # Calculate the angle between the direction of the wind and the line connecting the two points
        wind_angle = math.atan2(y2 - y1, x2 - x1) - wind_direction

        # Calculate the component of the wind speed in the direction of the line connecting the two points
        wind_component = wind_speed * math.cos(wind_angle)

        # Return the total heuristic cost, taking into account the wind speed and direction
        return distance + wind_component

# Define a function to perform the A* search
def a_star_search(map, start, goal, wind_speed, wind_direction):
    # Create a list to store the visited nodes
    visited = []
    # Create a list of the remaining nodes to be explored
    remaining = [start]

    # Keep looping until there are no more nodes to explore
    while remaining:
        # Sort the remaining nodes by their heuristic cost
        remaining.sort(key=lambda x: map.heuristic(x[0], x[1], x[2], goal[0], goal[1], goal[2], wind_speed, wind_direction))

        # Get the node with the lowest heuristic cost
        current = remaining.pop(0)
        print(current)
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

obstacles = [[0, 3, 2, 5, 0, 6], [6, 14, 3, 8, 0, 12], [3, 8, 11, 15, 0, 15]]
# Define a map with some obstacles
map = Map(obstacles)

# Define the start and goal coordinates
start = (0, 0, 0)
start2 = (0, 6, 0)
start3 = (8, -2, 2)

goal = (13, 15, 13)
goal2 = (13, 13, 13)
goal3 = (15, 15, 15)

# Define the wind speed and direction
wind_speed1 = 0
wind_direction1 = math.pi / 2

wind_speed2 = 2
wind_direction2 = math.pi / 3

# Perform the A* search, taking into account the wind speed and direction
visited = a_star_search(map, start, goal, wind_speed1, wind_direction1)

visited2 = a_star_search(map, start2, goal2, wind_speed1, wind_direction2)

visited3 = a_star_search(map, start3, goal3, wind_speed2, wind_direction2)
# Create a figure and a 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot the visited nodes
ax.plot([x[0] for x in visited], [x[1] for x in visited], [x[2] for x in visited], "ro-")
ax.plot([x[0] for x in visited2], [x[1] for x in visited2], [x[2] for x in visited2], "go-")
ax.plot([x[0] for x in visited3], [x[1] for x in visited3], [x[2] for x in visited3], "yo-")

#Plot obstacles
obstacles_coordinates = []

for obst in obstacles:
    for x in range(obst[0], obst[1] + 1):
        for y in range(obst[2], obst[3] + 1):
            for z in range(obst[4], obst[5] + 1):
                obstacles_coordinates.append([x, y, z])

ax.scatter3D([x[0] for x in obstacles_coordinates], [x[1] for x in obstacles_coordinates], [x[2] for x in obstacles_coordinates], "b")

# Set the axes labels
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# Show the plot
plt.show()