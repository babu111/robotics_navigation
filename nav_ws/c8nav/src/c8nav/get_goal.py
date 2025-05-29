import yaml
import os
import subprocess
import argparse
import sys
import time

from c8nav.qr_code_aruco_create import docking
from c8nav.pose_estimate import publish_initial_pose, move_back
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def delivery_and_return (name_of_location):
    yaml_path = "maps/saved_goals.yaml"
    if not os.path.exists(yaml_path):
        print(f'Error: Goal file not found at {yaml_path}')
        sys.exit(1)

    with open(yaml_path, 'r') as f:
        goals = yaml.safe_load(f)

    print(f)

    if name_of_location not in goals:
        print(f'Error: Goal name "{name_of_location}" not found in file.')
        print(f'Available goals: {list(goals.keys())}')
        sys.exit(1)

    goal = goals[name_of_location]
    x, y, theta = goal['x'], goal['y'], goal['theta']
    print(f'Sending goal "{name_of_location}": x={x}, y={y}, theta={theta}')

    pick_up_goal = goals['pick_up']
    pick_up_x, pick_up_y, pick_up_theta = pick_up_goal['x'], pick_up_goal['y'], pick_up_goal['theta']
    try:
        move_back(1)
        publish_initial_pose(pick_up_x, pick_up_y, pick_up_theta)

        subprocess.run([
            'ros2', 'run', 'c8nav', 'send_goal',
            '--x', str(x), '--y', str(y), '--theta', str(theta)
        ], check=True)
        #  ## OPEN LID
        # print('Sleeping for 10 seconds to allow for pickup.')
        # time.sleep(5)
        # ## CLOSE LID
        publish_initial_pose(x, y, theta)

        print('Go back to pick up goal')
        subprocess.run([
            'ros2', 'run', 'c8nav', 'send_goal',
            '--x', str(pick_up_x), '--y', str(pick_up_y), '--theta', str(pick_up_theta), ], check=True)
        docking()
         ## OPEN LID
         ## CLOSE LID

       

    except subprocess.CalledProcessError as e:
        print(f'Failed to execute send_goal.py: {e}')
        sys.exit(1)

class LocationSubscriber(Node):
    def __init__(self):
        super().__init__('location_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/requested_location',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received location request: {msg.data}')
        delivery_and_return(msg.data.name)


def main(args=None):
    delivery_and_return("sink")
    return
    rclpy.init(args=args)
    location_subscriber = LocationSubscriber()
    rclpy.spin(location_subscriber)
    location_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
