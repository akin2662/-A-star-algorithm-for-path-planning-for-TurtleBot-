#!/usr/bin/env python3

#importing important libraries
import numpy as np
import time
import cv2
import math
from queue import PriorityQueue
import heapq as hq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



scale = 100  # Scale of the map to convert from m to pixels
R = 0.033*scale  # Radius of the wheel in m
r = 0.220*scale  # Radius of the robot in m
L = 0.287*scale   # Distance between the wheels in m
map_xbounds, map_ybounds = (6 * scale), (2 * scale)  # Map dimensions
threshold = 0.05 * scale  # Threshold 


def all_obstacles(clearance):
    # defining the obstacles
    obs = {
        "Rectangle1": lambda x, y: np.logical_and(1 * scale <= map_ybounds - y - 1,
                                                  map_ybounds - y - 1 <= 2 * scale) & np.logical_and(1.5 * scale <= x,
                                                                                                    x <= 1.75 * scale),
        "Rectangle2": lambda x, y: np.logical_and(0 * scale <= map_ybounds - y - 1,
                                                  map_ybounds - y - 1 <= 1.0 * scale) & np.logical_and(
            2.50 * scale <= x, x <= 2.75 * scale),
        "Circle": lambda x, y: (x - (4 * scale)) ** 2 + (map_ybounds - y - 1 - (1.2 * scale)) ** 2 <= (0.6 * scale) ** 2
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
        (y_coords < clearance, y_coords >= map_ybounds - clearance, x_coords < clearance, x_coords >= map_xbounds - clearance), axis=0)] = [0, 255,
                                                                                                             255]  # boundary
    pixels[np.logical_not(is_obstacle) & np.logical_not(
        np.logical_or.reduce((y_coords < clearance, y_coords >= map_ybounds - clearance, x_coords < clearance, x_coords >= map_xbounds - clearance),
                             axis=0)) & is_clearance] = [0, 255, 255]  # clearance
    pixels[np.logical_not(is_obstacle) & np.logical_not(
        np.logical_or.reduce((y_coords < clearance, y_coords >= map_ybounds - clearance, x_coords < clearance, x_coords >= map_xbounds - clearance),
                             axis=0)) & np.logical_not(is_clearance)] = [255, 255, 255]  # free space

    return pixels


# check if node is in range
def check_range(node):
    if node is None:
        return False
    if len(node) == 3:
        x, y, _ = node
    else:
        x, y = node
    y = map_ybounds - y - 1
    return 0 <= x < map_xbounds and 0 <= y < map_ybounds and (pixels[int(y), int(x)] == [255, 255, 255]).all()

#check if node is valid
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


# check if the current node is the goal node
def check_goal(current_node, goal_node):
    return np.sqrt((goal_node[0] - current_node[0]) ** 2 + (goal_node[1] - current_node[1]) ** 2) <= threshold


# Backtrack
def backtrack_path(parents, start_node, goal_node):
    path, actions, current_node = [], [], goal_node
    action = parents[goal_node][1]
    while current_node != start_node:
        path.append(current_node)
        actions.append(action)
        current_node, action = parents[current_node]
    path.append(start_node)
    actions.append(parents[start_node][1])
    return path[::-1], actions[::-1]


# Find the euclidian distance as a heuristic
def euclidean_distance(node1, node2):
    x1, y1, _ = node1
    x2, y2 = node2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Function to make the movements
def move_func(input_node, UL, UR, plot=False):
    t, dt = 0, 0.1  # Time step
    Xi, Yi, Thetai = input_node  # Input point's coordinates
    Thetan = math.pi * Thetai / 180  # Convert end point angle to radian
    Xn, Yn = Xi, Yi  # End point coordinates
    Cost = 0
    valid = True  # Flag to indicate if all points in the curve are valid nodes
    while t < 1:
        t += dt
        X_prev, Y_prev = Xn, Yn
        linearx = 0.5 * R * (UL + UR) * math.cos(Thetan) * dt
        lineary = 0.5 * R * (UL + UR) * math.sin(Thetan) * dt
        lineartheta = (R / L) * (UR - UL) * dt
        Xn += linearx
        Yn += lineary
        Thetan += lineartheta
        if Thetan < 0:
            Thetan += 2 * math.pi
        Cost += math.sqrt(math.pow(linearx, 2) + math.pow(lineary, 2))
        if (pixels[int(map_ybounds - Yn - 1), int(Xn)] == [0, 0, 255]).all() or (
                pixels[int(map_ybounds - Yn - 1), int(Xn)] == [0, 255, 255]).all():
            valid = False  # Mark as invalid
            break
        if plot: cv2.arrowedLine(pixels, (int(X_prev), map_ybounds - 1 - int(Y_prev)),
                                 (int(Xn), map_ybounds - 1 - int(Yn)), (255, 0, 0), thickness=1)
    Thetan = (180 * (Thetan) / math.pi) % 360  # Convert back to degrees
    if valid:
        return (Xn, Yn, Thetan), Cost
    else:
        return None, float('inf')


# Astar algorithm
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
    parent = {start_node: (None, None)}
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

        if check_goal(current_node, goal_node):
            approx_goal_node = current_node  # Approximate goal node (within threshold distance)
            cost[goal_node] = cost[current_node]  # Cost of goal node
            path, final_actions = backtrack_path(parent, start_node, approx_goal_node)  # Backtrack the path
            if display_animation:
                for i in range(len(path) - 1):
                    x1, y1, _ = path[i]
                    x2, y2, _ = path[i + 1]
                    cv2.line(pixels, (int(x1), map_ybounds - 1 - int(y1)), (int(x2), map_ybounds - 1 - int(y2)),
                             (0, 0, 255), thickness=2)
                cv2.imshow('Path', pixels)
                cv2.waitKey(2000)
                out.write(pixels)
            print("Final Cost: ", cost[goal_node])
            out.release()
            return path, final_actions

        for action in actions:  # Iterate through all possible moves
            new_node, move_cost = move_func(current_node, action[0], action[1])
            if check_valid_node(new_node, visited):  # Check if the node is valid
                i, j, k = int(new_node[1] / threshold), int(new_node[0] / threshold), int(
                    new_node[2] / 30)  # Get the index of the node in the 3D array
                if not visited_node_matrix[i][j][k]:  # Check if the node is in closed list
                    new_c2c = c2c[current_node] + move_cost
                    new_c2g = euclidean_distance(new_node, goal_node)
                    new_cost = new_c2c + new_c2g*5 # Update cost
                    if new_node not in c2c or new_c2c < c2c[new_node]:
                        c2c[new_node] = new_c2c  # Update cost to come
                        c2g[new_node] = new_c2g  # Update cost to go
                        cost[new_node] = new_cost  # Update cost
                        parent[new_node] = (current_node, action)  # Update parent
                        visited_node_matrix[i][j][k] = True
                        hq.heappush(olist,(new_cost,new_node))   # Add to open list
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
    print("1) All the values are in m")
    print("2) Enter the each value of the start node position and orientation after a space. For ex. X_coordinate Y_coordinate orientation angle")
    print("3) Enter the each value of the goal node position and orientation after a space. For ex. X_coordinate Y_coordinate")
    print("4) Enter the each value of the RPM's after a space. For ex. RPM1 RPM2")
    print("5) Please be patient, it may take some time to get the output\n")

    clearance = float(input("Clearance from the obstacles: "))
    pixels = all_obstacles(clearance * scale)
    start_node = tuple(map(float, input("Enter the start node : ").split()))
    gazebostart_node = start_node
    start_node = ((start_node[0] + 0.5) * scale, (start_node[1] + 1) * scale, start_node[2] % 360)
    if not check_range(start_node):
        print("Please reenter a new node!")
        continue
    goal_node = tuple(map(float, input("Enter the goal node: ").split()))
    goal_node = ((goal_node[0] + 0.5) * scale, (goal_node[1] + 1) * scale)
    if not check_range(goal_node):
        print("Please reenter a new node!")
        continue
    RPM1, RPM2 = map(int, input("RPM1 and RPM2 for the robot: ").split())
    break

actions = [[RPM1, RPM1], [0, RPM1], [RPM1, 0], [RPM2, RPM2], [0, RPM2], [RPM2, 0], [RPM1, RPM2],
           [RPM2, RPM1]]  # List of actions


#converting the rpm values from the algortihm to wheel velocities
def RPM2Wheel_Velocities(act):
    wheel_velocities = []
    for i in act:
        if i is not None:
            for j in range(len(i)-1):
                vl = (2*math.pi*i[j])/60
                vr = (2*math.pi*i[j+1])/60
            wheel_velocities.append([vl,vr])


    return wheel_velocities

#Using the wheel velocities to get the Linear and Angular velocity of the robot
def getRobotVel(wheel_velocities):
    robot_vel_linear = []
    robot_vel_angular = []

    for i in wheel_velocities:
        for j in range(len(i)-1):
            V = (R/2)*(i[j]+i[j+1])
            W = (R/L)*(i[j+1]-i[j])

            robot_vel_linear.append(V)
            robot_vel_angular.append(W)



    return robot_vel_linear,robot_vel_angular

# Descriptipn for the node
class RobotControl(Node):

    def __init__(self, linear_vel, angular_vel):
        super().__init__('robot_path')
        self.actuate = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        self.current_index = 0
        self.timer = self.create_timer(1.75, self.move2goal)

# To move the node to the goal according the the velocities
    def move2goal(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        twist.linear.x = self.linear_vel[self.current_index] * 0.058
        twist.angular.z = self.angular_vel[self.current_index] * (3.15)
        self.actuate.publish(twist)
        self.current_index += 1
        if self.current_index >= len(self.linear_vel)-2:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.actuate.publish(twist)
            self.timer.cancel()

# Main function where the algorithm is run and simulated
def main(args = None):
    start_time = time.time()
    path, act = a_star(start_node, goal_node)
    for i in range(len(path)):
        x, y, theta = path[i]
        path[i] = (x / scale - 0.5, y / scale - 1, theta)
    if path is None:
        print("\nError: No path found.")
    else:
        print("\nGoal Node Reached!\n\nShortest Path: ", path, "\n\nActions: ", act, "\n")
    end_time = time.time()
    print("Runtime:", end_time - start_time, "seconds\n\nStarting ROS simulation...")
    print(act)
    wheel_velocities = RPM2Wheel_Velocities(act)
    print(wheel_velocities)
    linear_vel,angular_vel = getRobotVel(wheel_velocities)
    print(linear_vel)
    print(angular_vel)
    rclpy.init(args=args)
    node = RobotControl(linear_vel,angular_vel)
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()


