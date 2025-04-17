# robotics_navigation

## 1. Path Planning Algorithms

**path planning** algorithms like Dijkstra's and A\* (A-star) are implemented in the `nav2` stack.

### Step-by-step Instructions (Simulation):

1. Launch the TurtleBot3 world in Gazebo:

    ```bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```

2. Launch the `nav2` stack with a known map:

    ```bash
    ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/path/to/your_map.yaml
    ```

3. Start RViz2:

    ```bash
    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
    ```

4. Set the initial pose with the `2D Pose Estimate` tool in RViz2.

5. Publish a navigation goal using the `Nav2 Goal` button in RViz2. You can record the time between goals manually or via a script.

6. To switch between Dijkstra and A*:

    Edit this parameter in your local `nav2_params.yaml`:

    ```yaml
    global_planner:
      ros__parameters:
        planner_plugin: "GridBased"
        use_astar: true  # Set to false for Dijkstra, true for A*
    ```

    Then relaunch `nav2` with that config:

    ```bash
    ros2 launch nav2_bringup bringup_launch.py map:=/path/to/your_map.yaml params_file:=/path/to/nav2_params.yaml
    ```

---

## 2. Path Planning With the Physical TurtleBot

This portion of the lab involves testing the global planners (Dijkstra and A*) using a physical TurtleBot in the Robotics Lab.

### Step-by-step Instructions (Physical TurtleBot):

1. **Map the lab** using SLAM Toolbox:

    ```bash
    ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=false
    ```

    Start teleoperation in a new terminal:

    ```bash
    ros2 run turtlebot3_teleop teleop_keyboard
    ```

    Save the map after exploration:

    ```bash
    ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
    ```

2. **Launch navigation stack** using your saved map:

    ```bash
    ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false map:=~/maps/lab_map.yaml
    ```

3. **Start RViz2** and set the robot’s initial pose:

    ```bash
    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
    ```

4. **Set navigation goals** between two key points in the lab using the `Nav2 Goal` tool.

5. To **compare Dijkstra vs A***:

    Modify your `nav2_params.yaml` as above and repeat navigation for both planners.

6. **Add an obstacle** after the first run (e.g., chair or box in the path), and try navigating again with both planners. 

7. In RViz2, add a `Path` display and set the topic to:

    ```text
    /local_plan
    ```

    This lets you visually compare how each planner adapts to obstacles in real-time.

---

### Evaluation Notes:

- Time the robot from initial pose to goal.
- Observe differences in chosen path lengths and reactivity to obstacles.
- Use your findings to answer: How does A\* compare to Dijkstra in real-world navigation?

