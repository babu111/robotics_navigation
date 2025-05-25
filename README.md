# robotics_navigation


## Progress:

### Milestone 1:

- At least Nav2Goal function is reponsive, but it doesn't work well.
- Tried adjusting occupied_thresh: 0.5, free_thresh: 0.35, no effects.
- Tried using Turtlebot08 instead of Turtlebot07 and it worked. Verified that it's a hardware problem.


### Milestone 2:

- Concatenated the kitchen map and the robotics workshop map together.
- Able to use python code to give nav goal in coordinates and let turtlebot navigate to that spot.
---

## Configuring Environment


### Installation
The ROS2 env given by the IT shop boys is buggy. Consider reinstalling [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

```
git clone git@github.com:babu111/robotics_navigation.git
cd robotics_navigation
colcon build
```

(Optional) Run `turtlebot_ws/install_turtlebot_dependencies.sh` for turtlebot workspace installation if previous installation fails.


## Path Planning with GUI (Zeyi)

**path planning** algorithms like Dijkstra's and A\* (A-star) are implemented in the `nav2` stack.

### Step-by-step Instructions:
(for turtlebot)
Install raspi-config: If it's not already installed, run: sudo apt install raspi-config. 
Open raspi-config: Run: sudo raspi-config. 
Select "Interfacing Options": Choose the relevant option in the menu. 
Enable Camera: Locate the camera interface option (usually labeled "Camera" or "Legacy Camera") and enable it. 
Exit raspi-config: Follow the prompts to save the changes and exi

Run the following blocks of commands each in a separate terminal.


0. Run the tf transform publisher:
   ```bash
   ros2 run tf2_ros static_transform_publisher 0 0 0.35 0 0 0 base_link laser
~~0. Open fastdds discovery in a terminal:~~

1. Launch the turtlebot:
   ```bash
   # ssh to the robot
    ssh ubuntu@192.168.3.186
    
    
    # both on the robot and laptop, run the following
    # ROS_DOMAIN_ID=30 is occupied by the hardware & software team
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
    # then restart the terminal or souce bashrc
    
    # inside the robot, run
    # ros2 launch turtlebot3_bringup robot.launch.py

    # for create bot, run:
      ros2 launch techin517 techin517.launch.py
   ```

2. Run teleop keyboard.
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

3. Launch the `nav2` stack with Jason's map from the /maps folder:

    ```bash
    cd /home/gixadmin/robotics_navigation
    # In Cathy's laptop: cd /home/gixstudent/Desktop/Final_lab/robotics_navigation
    ros2 launch nav2_bringup bringup_launch.py map:=maps/map_v4.yaml

    # We found that using the nav2_params and the a* was worse
    ```

4. Start RViz2:

    ```bash
    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
    ```

5. Set the initial pose with the `2D Pose Estimate` tool in RViz2.

6. Publish a navigation goal using the `Nav2 Goal` button in RViz2. You can set intermediate waypoints by clicking the waypoints button at bottom left and put multiple nav2 goals on the map.

#### ⚠️ !! IMPORTANT !!: **Turn off teleop before you give Nav2 Goal command.**


7. To switch between Dijkstra and A*:

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

### How to get ip of a robot:

This is to help you get the ip of the robot when you can't ssh onto it.

1. Connect a hdmi to a micro-hdmi adapter. Plug the micro-hdmi to the turtlebot. Plug a usb of keyboard to the turtlebot.

2. Turn on the turtlebot. ONLY do so after you connected properly in step 1.

3. Login with user=ubuntu, password=robot1234

4. Run `ifconfig -a`.


---

## Path Planning with code (Zeyi)

First configure the environment.
```
cd ~/robotics_navigation
colcon build --packages-select c8nav
source install/setup.bash
```

To use path planning, first follow step 1-5 of section "Path Planning with GUI".

Then use the `send_goal.py` in `nav_ws`.
```
ros2 run c8nav send_goal --x -8.0 --y 1.5 --theta 1.57
```

## Save current robot position and navigate to saved position(Yuxin)
First configure the environment.
```
cd ~/robotics_navigation
colcon build --packages-select c8nav
source install/setup.bash
```

To save the robot's current position on the map(eg. sink):
```
ros2 run c8nav save_current_pose --name sink
```
The position will be saved to robotics_navigation/saved_goals.yaml (currently there are pick_up/wall/sink/elevator/sofa/lab)

To navigate to a saved position(eg. sink):
```
ros2 run c8nav get_goal --name sink
```


## Publishing current robot position, distance to goal and estimated time to goal to a topic (Zeyi)

Run:
```
cd /robotics_navigation
ros2 run c8nav nav2_status_publisher.py
```

View the topic at
```
ros2 topic echo /nav2_status
```

---

## Mapping With the Physical TurtleBot

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
