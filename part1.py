import numpy as np
import time
import cv2
import math
import heapq as hq

scale1 = 10 # To convert radius of wheel and robot and the distance between wheels to cms
scale2=0.1 # To convert the map and obstacles to cms

r = 0.220 * scale1 # Radius of the robot in cms
R = 0.033 * scale1  # Radius of the wheel in cms
L = 0.287 * scale1 # Distance between the wheels in cms

map_xbounds, map_ybounds = 600, 200   # Map dimensions in cms
threshold = 5 # Threshold for the goal node


def all_obstacles(clearance):
    # Define the Obstacle Equations and Map Parameters
    obs = {
        "Rectangle1": lambda x, y: np.logical_and(0 <= y, y <= 1000*scale2) & np.logical_and(1500*scale2 <= x, x <= 1750*scale2),
        "Rectangle2": lambda x, y: np.logical_and(1000*scale2 <= y, y <= 2000*scale2) & np.logical_and(2500*scale2 <= x, x <= 2750*scale2),
        "Circle": lambda x, y: (x - 4200*scale2) ** 2 + (y - 800*scale2) ** 2 <= (120 / 2) ** 2
    }

    clearance = clearance + r
    y_coords, x_coords = np.meshgrid(np.arange(map_ybounds), np.arange(map_xbounds), indexing='ij')
    is_obstacle = np.zeros_like(x_coords, dtype=bool)
    for eqn in obs.values():  # Check if the point is an obstacle
        is_obstacle |= eqn(x_coords, y_coords)

    is_clearance = np.zeros_like(x_coords, dtype=bool)  # Check if the point is within the clearance
    for eqn in obs.values():
        for c_x in np.arange(-clearance, clearance + 0.5, 0.5):
            for c_y in np.arange(-clearance, clearance + 0.5, 0.5):
                if (c_x ** 2 + c_y ** 2) <= clearance ** 2:
                    is_clearance |= eqn(x_coords + c_x, y_coords + c_y)

    pixels = np.full((map_ybounds, map_xbounds, 3), 255, dtype=np.uint8)
    pixels[is_obstacle] = [0, 0, 255]  # obstacle
    pixels[np.logical_not(is_obstacle) & np.logical_or.reduce(
        (y_coords < clearance, y_coords >= map_ybounds - clearance, x_coords < clearance, x_coords >= map_xbounds - clearance), axis=0)] = [0, 255, 255]  # boundary
    pixels[np.logical_not(is_obstacle) & np.logical_not(
        np.logical_or.reduce((y_coords < clearance, y_coords >= map_ybounds - clearance, x_coords < clearance, x_coords >= map_xbounds - clearance),
                             axis=0)) & is_clearance] = [0, 255, 255]  # clearance
    pixels[np.logical_not(is_obstacle) & np.logical_not(np.logical_or.reduce((y_coords < clearance, y_coords >= map_ybounds - clearance, x_coords < clearance, x_coords >= map_xbounds - clearance),
                             axis=0)) & np.logical_not(is_clearance)] = [255, 255, 255]  # free space

    return pixels


# Define a function to check if current node is in range
def check_range(node):
    if node is None:
        return False
    if len(node) == 3:
        x, y, _ = node
    else:
        x, y = node
    y = map_ybounds - y - 1
    return 0 <= x < map_xbounds and 0 <= y < map_ybounds and (pixels[int(y), int(x)] == [255, 255, 255]).all()


def check_valid_node(node, visited):
    if node is None:
        return False
    if not check_range(node):
        return False  # out of range
    x, y, _ = node
    y = map_ybounds - y - 1
    if not (pixels[int(y), int(x)] == [255, 255, 255]).all():
        return False  # in obstacle space
    threshold_theta = math.radians(30)
    for i in range(-1, 2):
        for j in range(-1, 2):
            for k in range(-1, 2):
                neighbor_node = (x + i * threshold, y + j * threshold, k * threshold_theta)
                if neighbor_node in visited:
                    return False  # Too close to a visited node
    return True


# Define a function to check if current node is the goal node
def check_goal(current_node, goal_node):
    return np.sqrt((goal_node[0] - current_node[0]) ** 2 + (goal_node[1] - current_node[1]) ** 2) <= 5


# Define a function to find the optimal path
def backtrack_path(parents, start_node, goal_node):
    path, current_node = [goal_node], goal_node
    while current_node != start_node:
        path.append(current_node)
        current_node = parents[current_node]
    path.append(start_node)
    return path[::-1]


# Define a function to calculate the euclidean distance
def euclidean_distance(node1, node2):
    x1, y1, _ = node1
    x2, y2 = node2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def move_func(input_node, UL, UR, plot=False):
    t, dt = 0, 0.1  # Time step
    Xi, Yi, Thetai = input_node  # Input point's coordinates
    Thetan = 3.14 * Thetai / 180  # Convert end point angle to radian
    Xn, Yn = Xi, Yi  # End point coordinates
    Cost = 0
    valid = True  # Flag to indicate if all points in the curve are valid nodes
    while t < 1:
        t += dt
        X_prev, Y_prev = Xn, Yn
        Dx = 0.5 * R * (UL + UR) * math.cos(Thetan) * dt
        Dy = 0.5 * R * (UL + UR) * math.sin(Thetan) * dt
        Xn += Dx
        Yn += Dy
        Thetan += (R / L) * (UR - UL) * dt
        if Thetan < 0:
            Thetan += 2 * math.pi
        Cost += math.sqrt(math.pow(Dx, 2) + math.pow(Dy, 2))
        node = (Xn, Yn, Thetan)
        if (pixels[int(map_ybounds - Yn - 1), int(Xn)] == [0, 0, 0]).all() or (
                pixels[int(map_ybounds - Yn - 1), int(Xn)] == [192, 192, 192]).all():
            valid = False  # Mark as invalid
            break
        if plot: cv2.arrowedLine(pixels, (int(X_prev), map_ybounds - 1 - int(Y_prev)),
                                 (int(Xn), map_ybounds - 1 - int(Yn)), (255, 0, 0), thickness=1)
    Thetan = (180 * (Thetan) / 3.14) % 360  # Convert back to degrees
    if valid:
        return (Xn, Yn, Thetan), Cost
    else:
        return None, float('inf')


# Define the A* algorithm
def a_star(start_node, goal_node, display_animation=True):
    rows = int(map_ybounds / threshold)  # number of rows
    cols = int(map_xbounds / threshold)  # number of columns
    angles = int(360 / 30)  # number of angles
    visited_node_matrix = [[[False for _ in range(angles)] for _ in range(cols)] for _ in range(rows)]  # visited nodes matrix

    olist = []
    hq.heapify(olist)
    clist = set()
    c2c = {start_node: 0}
    c2g = {start_node: euclidean_distance(start_node, goal_node)}
    cost = {start_node: c2c[start_node] + c2g[start_node]}
    parent = {start_node: None}
    hq.heappush(olist,(cost[start_node],start_node))
    visited = set([start_node])
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('animation.mp4', fourcc, 15.0, (map_xbounds, map_ybounds))

    while olist:
        _, current_node = hq.heappop(olist)
        clist.add(current_node)
        x, y, theta = current_node
        visited_node_matrix[int(y / threshold)][int(x / threshold)][int(theta / 30)] = True  # Mark current node as visited
        out.write(pixels)
        if display_animation:
            cv2.imshow('Path', pixels)
            cv2.waitKey(1)
        # Check if current node is the goal node
        if check_goal(current_node, goal_node):
            approx_goal_node = current_node  # Approximate goal node (within threshold distance)
            cost[goal_node] = cost[current_node]  # Cost of goal node
            path = backtrack_path(parent, start_node, approx_goal_node)  # Backtrack the path
            if display_animation:
                for i in range(len(path) - 1):
                    x1, y1, _ = path[i]
                    x2, y2, _ = path[i + 1]
                    cv2.line(pixels, (int(x1), map_ybounds - 1 - int(y1)), (int(x2), map_ybounds - 1 - int(y2)),
                             (0, 0, 255), thickness=2)
                cv2.imshow('Path', pixels)
                cv2.waitKey(0)
                out.write(pixels)

            print("Final Cost: ", cost[goal_node])
            out.release()
            cv2.destroyAllWindows()
            return path

        for action in actions:  # Iterate through all possible moves
            new_node, move_cost = move_func(current_node, action[0], action[1])
            if check_valid_node(new_node, visited):  # Check if the node is valid
                i, j, k = int(new_node[1] / threshold), int(new_node[0] / threshold), int(
                    new_node[2] / 30)  # Get the index of the node in the 3D array
                if not visited_node_matrix[i][j][k]:  # Check if the node is in closed list
                    new_c2c = c2c[current_node] + move_cost
                    new_c2g = euclidean_distance(new_node, goal_node)
                    new_cost = new_c2c + new_c2g*5  # Update cost
                    if new_node not in c2c or new_c2c < c2c[new_node]:
                        c2c[new_node] = new_c2c  # Update cost to come
                        c2g[new_node] = new_c2g  # Update cost to go
                        cost[new_node] = new_cost  # Update cost
                        parent[new_node] = current_node  # Update parent
                        visited_node_matrix[i][j][k] = True
                        hq.heappush(olist,(new_cost,new_node))  # Add to open list
                        visited.add(new_node)  # Add to visited list
                        _, _ = move_func(current_node, action[0], action[1], plot=display_animation)  # Plot the path

        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            break

    out.release()
    cv2.destroyAllWindows()
    return None


# Get valid start and goal nodes from user input
while True:
    print("Note:")
    print("1) Please give clearance in cms")
    print("2) All the values are in cms")
    print("3) Enter the each value of the start node position and orientation after a space. For ex. X_coordinate Y_coordinate orientation angle")
    print("4) Enter the each value of the goal node position and orientation after a space. For ex. X_coordinate Y_coordinate")
    print("5) Please be patient, it may take some time to get the output\n")

    clearance = int(input("Clearance from the obstacles: "))
    pixels = all_obstacles(clearance)
    start_node = tuple(map(int, input("Enter the start node : ").split()))
    if not check_range(start_node):
        print("Please reenter a new node!")
        continue
    goal_node = tuple(map(int, input("Enter the goal node: ").split()))
    if not check_range(goal_node):
        print("Please reenter a new node!")
        continue
    RPM1 = int(input("RPM1 for the robot: "))
    RPM2 = int(input("RPM2 for the robot: "))
    break

actions = [[RPM1, RPM1], [0, RPM1], [RPM1, 0], [RPM2, RPM2], [0, RPM2], [RPM2, 0], [RPM1, RPM2],
           [RPM2, RPM1]]  # List of actions

# Run A* algorithm
start_time = time.time()
path = a_star(start_node, goal_node)
if path is None:
    print("\npath not found")
else:
    print("\n reached the goal")
end_time = time.time()
print("Runtime:", end_time - start_time, "seconds\n")