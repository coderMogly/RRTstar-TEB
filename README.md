# RRTstar-TEB
Code for my Bachelors Thesis. Simulation and comparison of various path planing algorithms for autonomous vehicle. 

It implements RRT* as a global planner using OMPL with TEB local planner. ROS navigation stack is used with move base being the executive coordinator between the local and global planner. Th

# TODO:
1. Add comments in code.
2. Create a doxy documentation of the project.

# Instruction
1. Create a catkin workspace in Ubuntu home directory.
2. Clone the source code.
3. Build the workspace and enjoy.

# Requirements
1. ROS indigo 
2. Ubuntu 14.04 

# dependencies 
1. geometry_msgs
2. nav_msgs 
3. rviz physics engine 
4. Stage simulator
5. Ompl
6. ROS nav_stack 
