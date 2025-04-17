# robotics_navigation


# 1. Path Planning Algorithms

Now that you understand how robots map their environments, we'll take a look at how they navigate those maps. This is done using "path planning" algorithms. These are the same types of algorithms your favorite map app uses on your phone. The two algorithms we will discuss in lecture are Dijkstra's (pronounced: dike-struh) algorithm and A\* (pronounced: A-star). Both of these algorithms have been implemented for us in the nav2-stack. Start by using both planning algorithms in simulation to compare their performance: 

1. Launch the `turtlebot3_world` Gazebo simulation:

	```bash
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
	```

2. Launch `nav2` to use this package's path planners. Now when you use the `Nav2 Goal` in Rviz2, this package will use predefined path planning algorithms to navigate the map towards that goal. The default planner is Dijkstra's algorithm. Use the map you made of the simulation environment in lab 2:

	```bash
	ros2 launch nav2_bringup bringup_launch.py map:=/path/to/your-map.yaml
	```

3. Start Rviz2:

	```bash
	ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
	```

4. Set the initial position with the `2D Pose Estimate` in Rviz2.

5. Publish a series of poses and record how long the robot takes to navigate between them. You can publish these poses using a C++ script, a python script, or from the command line. You can time the movement using the script or your phone. The poses are given below in the format (x, y, z, qx, qy, qz, qw) where 'q' values represent quaternion orientations: 

	| p0 | p1 | p2 | p3 |
	| - | - | - | - |
	| (-2. -0.5, 0, 0, 0, 0, 1) | (0, -2, 0, 0, 0, 0.342, 0.939) | (2, 0, 0, 0, 0, 0.472, 0.882) | (0.55, 1.25, 0, 0, 0, -0.707, 0.707) |  

	**Deliverables**

	**1.1:** Fill in the following table with time values moving between positions using different path planning algorithms:

	| path | time with Dijkstra | time with A\* |
	| - | - | - |
	| p0 -> p1 | | |
	| p1 -> p2 | | |
	| p2 -> p3 | | | 
	| p3 -> p0 | | |

	Note: you can change which algorithm the path planner uses with the parameter reconfiguration tool in rqt, or by relaunching the nav2 stack with the following command: 

	```bash
	ros2 launch nav2_bringup bringup_launch.py map:=/path/to/your-map.yaml planner_server.GridBased.use_astar:=True
	```

	**1.2:** Describe qualitatively which path length was shorter, Dijkstra or A\*?

	**1.3:** Discuss the advantages and disadvantages of the two path planning algorithms.

# 2. Path Planning With the Physical Robot

This portion of the lab assignment will entail repeating the same process for testing the two global planners, now using the GIX map. You have already completed a map for this environment. If you do not have the .pgm and .yaml files associated with the GIX map you will have to run the mapping demo and teleoperate the robot around the map before getting started.

1. Instead of providing a sequence of 4 points as navigation goals, the physical implementation will include only 2 points, one in each enclosed area of the maze.

	**Deliverables**

	**2.1:** Fill in the following table with the time values using different path planning algorithms:

	| path | time with Dijkstra | time with A\* | 
	| - | - | - |
	| p0 -> p1 | | |
	| p1 -> p0 | | |

	**2.2:** If any collisions occurred, comment on possible causes of the collision. 

2. Reset the robot to position p0, then add an obstacle in the map environment. In Rviz2, add a `path` display and set the topic to `/local_plan`. Attempt the navigation again with both planner and comment on the following: 

	**Deliverables** 
	
	**2.3:** Does the robot reach its intended goal with both planners?

	**2.4:** What do you observe from the global vs. local paths?
