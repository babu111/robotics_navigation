# robotics_navigation

## 1. Path Planning with GUI (Zeyi)

**path planning** algorithms like Dijkstra's and A\* (A-star) are implemented in the `nav2` stack.

### Installation
The ROS2 env given by the IT shop boys is buggy. Consider reinstalling [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

Run `turtlebot_ws/install_turtlebot_dependencies.sh` for turtlebot workspace installation.

### Step-by-step Instructions:

Run the following blocks of commands each in a separate terminal.

1. Open fastdds discovery in a terminal:

    ```bash
    fastdds discovery --server-id 0
    ```

2. Launch the turtlebot:
   ```bash
   # ssh to the robot
    ssh ubuntu@10.18.3.94
    
    
    # both on the robot and laptop, run the following 2 commands
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
    # export ROS_DISCOVERY_SERVER=<ip_of_your_laptop>:11811
    echo "export ROS_DISCOVERY_SERVER=10.18.239.105:11811" >> ~/.bashrc
    # then restart the terminal or souce bashrc
    
    # inside the robot, run
    ros2 launch turtlebot3_bringup robot.launch.py
   ```

3. Run teleop keyboard.
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

4. Launch the `nav2` stack with Jason's map:

    ```bash
    cd /home/gixadmin/robotics_navigation
    ros2 launch nav2_bringup bringup_launch.py map:=map_kitchen.yaml
    # use A* algorithm
    ros2 launch nav2_bringup bringup_launch.py map:=map_kitchen.yaml planner_server.GridBased.use_astar:=True
    ```

5. Start RViz2:

    ```bash
    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
    ```

6. Set the initial pose with the `2D Pose Estimate` tool in RViz2.

7. Publish a navigation goal using the `Nav2 Goal` button in RViz2. You can set intermediate waypoints by clicking the waypoints button at bottom left and put multiple nav2 goals on the map.

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

### Progress (milestone 1):

- At least Nav2Goal function is reponsive, but it doesn't work well.
- Tried adjusting occupied_thresh: 0.5, free_thresh: 0.35, no effects.
- Tried using Turtlebot08 instead of Turtlebot07 and it worked. Verified that it's a hardware problem.
---

## 2. Mapping With the Physical TurtleBot

Testing the global planners (Dijkstra and A*) using a physical TurtleBot in the Robotics Lab.

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

4. In RViz2, add a `Path` display and set the topic to:

    ```text
    /local_plan
    ```
    
    This lets you visually compare how each planner adapts to obstacles in real-time.


## 3. Path Planning with code (Zeyi)

Use the `send_goal.py` in `nav_ws`.
```
ros2 run nav_ws send_goal
```
