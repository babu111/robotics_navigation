#!/usr/bin/env python3

import yaml
import os
import subprocess
import argparse
import sys
import time

from qr_code_aruco_create import docking
from pose_estimate import publish_initial_pose, move_back
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

def open_lid():
    for i in range(5):
        # subprocess.run('ros2 topic pub --once /lid_cmd std_msgs/Bool "data: true"', shell=True)  old version
        subprocess.run('ros2 topic pub --once /lid_cmd std_msgs/msg/ String "data: open"', shell=True)
        time.sleep(0.4)
    print("Lid opened.")

def close_lid():
    for i in range(5):
        subprocess.run('ros2 topic pub --once /lid_cmd std_msgs/msg/ String "data: close"', shell=True)
        time.sleep(0.4)
    print("Lid closed.")


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

    pre_pick_up = goals['pre_pick_up']
    pick_up_goal = goals['pick_up']
    pick_up_x, pick_up_y, pick_up_theta = pick_up_goal['x'], pick_up_goal['y'], pick_up_goal['theta']
    try:
        print("Set ready status to false.")
        subprocess.run([
            'ros2', 'param', 'set', '/nav2_status_publisher', 'ready', 'false'
        ])

        close_lid()

        move_back(1)
        publish_initial_pose(pick_up_x, pick_up_y, pick_up_theta)

        command = (
            "ros2 run c8nav send_goal.py "
            f"--x {x} --y {y} --theta {theta}"
        )

        subprocess.run(command, shell=True, check=True)
        open_lid()
        time.sleep(5)
        close_lid()
        
        
        publish_initial_pose(x, y, theta)

        
        print("Got to pre pickup pose")
        command = (
            "ros2 run c8nav send_goal.py "
            f"--x {pre_pick_up['x']} --y {pre_pick_up['y']} --theta {pre_pick_up['theta']}"
        )
        subprocess.run(command, shell=True, check=True)
        print('Go back to pick up goal')
        publish_initial_pose(pre_pick_up['x'], pre_pick_up['y'], pre_pick_up['theta'])
        
        command = (
            "ros2 run c8nav send_goal.py "
            f"--x {pick_up_x} --y {pick_up_y} --theta {pick_up_theta}"
        )

        subprocess.run(command, shell=True, check=True)

        docking()
        try:
            rclpy.init()
        except:
            print("init error")
            pass
        open_lid()

        print("Set ready status to true.")
        subprocess.run([
            'ros2', 'param', 'set', '/nav2_status_publisher', 'ready', 'true'
        ])

       

    except subprocess.CalledProcessError as e:
        print(f'Failed to execute send_goal.py: {e}')
        sys.exit(1)



class DestinationClient(Node):
    def __init__(self):
        super().__init__("destination_client")
        self.cli = self.create_client(Trigger, "get_destination")
        self.in_progress = False

        self.get_logger().info("⏳ Waiting for /get_destination …")
        self.cli.wait_for_service()
        self.get_logger().info("🟢 Service available")

        # Poll every 5 seconds
        self.timer = self.create_timer(5.0, self.request_destination)

    def request_destination(self):
        self.get_logger().info("Requesting destination...")
        if self.in_progress:
            self.get_logger().info("⏳ Delivery in progress, skipping request.")
            return

        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.destination_callback)

    def destination_callback(self, future):
        self.get_logger().info("Destination callback received")
        try:
            resp = future.result()
            self.get_logger().info(f"Resp: {resp}")
            if resp.success:
                self.get_logger().info(f"📦 New destination: {resp.message!r}")
                self.in_progress = True

                # Run delivery (blocking call)
                delivery_and_return(resp.message)

                self.get_logger().info("✅ Delivery complete")
                self.in_progress = False
            else:
                self.get_logger().warn("⚠️ No destination stored yet.")
        except Exception as e:
            self.get_logger().error(f"❌ Service call failed: {e}")

def main(args=None):

    rclpy.init(args=args)
    delivery_and_return("sink")
    return
    node = DestinationClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
