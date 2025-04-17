# robotics_navigation


## 1. Path Planning Algorithms

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


## 2. Path Planning With the Physical Robot

This portion of the lab assignment will entail repeating the same process for testing the two global planners, now using the GIX map. You have already completed a map for this environment. If you do not have the .pgm and .yaml files associated with the GIX map you will have to run the mapping demo and teleoperate the robot around the map before getting started.

1. Instead of providing a sequence of 4 points as navigation goals, the physical implementation will include only 2 points, one in each enclosed area of the maze.

2. Reset the robot to position p0, then add an obstacle in the map environment. In Rviz2, add a `path` display and set the topic to `/local_plan`. Attempt the navigation again with both planner and comment on the following: 
