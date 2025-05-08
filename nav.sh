# set up turtlebot3
./install_turtle_dependencies.sh


echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /home/gixadmin/robotics_navigation/turtlebot_ws/install/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

# open fastdds discovery in a terminal
fastdds discovery --server-id 0

# check ip with ifconfig
ifconfig -a

# ssh to the robot
ssh ubuntu@10.18.3.92


# both on the robot and laptop, run the following 2 commands
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
# export ROS_DISCOVERY_SERVER=<ip_of_your_laptop>:11811
echo "export ROS_DISCOVERY_SERVER=10.18.239.105:11811" >> ~/.bashrc


# inside the robot, run
ros2 launch turtlebot3_bringup robot.launch.py



ros2 run turtlebot3_teleop teleop_keyboard

# ros2 launch nav2_bringup bringup_launch.py map:=/path/to/your-map.yaml

cd /home/gixadmin/robotics_navigation
ros2 launch nav2_bringup bringup_launch.py map:=map_kitchen.yaml
# use A* algorithm
ros2 launch nav2_bringup bringup_launch.py map:=map_kitchen.yaml planner_server.GridBased.use_astar:=True


ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
