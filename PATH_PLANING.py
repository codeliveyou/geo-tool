import cv2
import numpy as np

# Load image (assuming white is free and black is obstacle)
img = cv2.imread('./Assets/F4.png', cv2.IMREAD_GRAYSCALE)
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)

# Convert obstacles to 1 and free space to 0
occupancy = (binary > 0).astype(np.uint8)

# Inflate obstacles (dilate)
robot_size = 10  # adjust as needed (in pixels)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (robot_size, robot_size))
inflated = cv2.dilate(occupancy, kernel, iterations=1)

# Optionally, invert back so free=1 (or keep as is depending on planner)
free_space = (inflated == 0).astype(np.uint8)






import heapq
import math

def heuristic(a, b):
    # Use Euclidean or Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
    # For Euclidean:
    # return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def astar(grid, start, goal):
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:  # 4-connected grid
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                # Check if neighbor is free (1 means free space if using free_space mask)
                if grid[neighbor[0], neighbor[1]] == 0:
                    continue
                
                tentative_g = g_score[current] + 1  # assumes uniform cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

# Define your start and goal in grid coordinates (row, col)
start = (10, 10)
goal = (200, 300)

path = astar(free_space, start, goal)
if path is None:
    print("No path found!")
else:
    print("Path found:", path)



# Visualize on the image:
color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
for point in path:
    cv2.circle(color_img, (point[1], point[0]), 2, (0, 0, 255), -1)
cv2.imshow("Path", color_img)
cv2.waitKey(0)