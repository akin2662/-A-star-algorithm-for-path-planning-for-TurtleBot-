# A-star-algorithm-for-2D-point-robot-path-planning
**Note:** This project was a requirement for the course ENPM 661- Planning for Autonomous Robots at University of Maryland, College Park and was done in collaboration with Raajith Gadam (raajithg@umd.edu)

## Project Description:
This script contains an implementation of the A-star algorithm for a point robot for navigation around obstacles in a 2D space. It also contains a script, which after finding the path, sends the appropriate velocity commands to the TurtleBot3 Waffle, which will then move according to the vel commands.

## Dependencies:

* python 3.11 (any version above 3 should work)
* Python running IDE (We used Pycharm)
  
## Libraries Used:
* numpy
* time
* cv2
* heapq
* matplotlib
* Time
* Math
* rclpy
* geometry_msgs 

## Instructions:

### Part1:
1) Run python3 part1.py
2) Input the values as asked in the terminal

**Test case:**
**Input:**
Clearance from the obstacles: 5
Enter the start node : 10 10 30
Enter the goal node: 580 180
RPM1 for the robot: 10
RPM2 for the robot: 10
Final Cost:  1001.3822195135446

**Output:**
 reached the goal
Runtime: 25.256956100463867 seconds
note:a video will be saved in the same directory as the code!

### Part2:
1. Place the turtlebot3_project3 package in source folder of your workspace
2. colcon build
3. source install/setup.bash
4. ros2 launch turtlebot3_project3 competition_world.launch.py
5. In new terminal,source install/setup.bash
6. ros2 run turtlebot3_project3 part2_new.py
7. Input the values as asked in the terminal

**Test case:**
**Input:**
1. Clearance: 0.01
2. Start coordinates and angle: 0 0 0
3. Goal coordinates: 5.1 0
4. RPM1 and RPM2: 10 16

## Demo videos:
Following are the links to the recorded simulation
part1: https://drive.google.com/file/d/14HxtT1DbY8zS777Tv_Qrf_5FtBZ3wkad/view?usp=sharing
part2: https://drive.google.com/file/d/1IWEEaPTUh4zzWwEZcqSbLXrzmgNFMoEE/view?usp=sharing

***NOTE:** The simulation environment (world) in gazebo is taken from https://github.com/shantanuparabumd/turtlebot3_project3.git as required for ENPM 661 Project3 Phase2



