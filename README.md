# Astar_Algorithm_Simulation

student1: Raajith Gadam (UID:119461167)(Directory ID: raajithg)
student2: Advait Kinikar (UID: 120097997 )(Directory ID:kinikara ) 

Github Link: https://github.com/akin2662/Astar_Algorithm_Simulation.git

Demo videos:
part1: https://drive.google.com/file/d/14HxtT1DbY8zS777Tv_Qrf_5FtBZ3wkad/view?usp=sharing
part2: https://drive.google.com/file/d/1IWEEaPTUh4zzWwEZcqSbLXrzmgNFMoEE/view?usp=sharing

# Dependencies 
 numpy
time
cv2
heapq
matplotlib
Time
Math

Part1 instructions:

Run python3 part1.py

Input:
Clearance from the obstacles: 5
Enter the start node : 10 10 30
Enter the goal node: 580 180
RPM1 for the robot: 10
RPM2 for the robot: 10
Final Cost:  1001.3822195135446

Output:
 reached the goal
Runtime: 25.256956100463867 seconds
note:a video will be saved in the same directory as the code!

Part2 instructions:
1.Place the turtlebot3_project3 package in source folder of your workspace
2.colcon build
3.source install/setup.bash
4.ros2 launch turtlebot3_project3 competition_world.launch.py
5.In new terminal,source install/setup.bash
6.ros2 run turtlebot3_project3 part2_new.py

Test case:
Input:
1. Clearance: 0.01
2. Start coordinates and angle: 0 0 0
3. Goal coordinates: 5.1 0
4. RPM1 and RPM2: 10 16
