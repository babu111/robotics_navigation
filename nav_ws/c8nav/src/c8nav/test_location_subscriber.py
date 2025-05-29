#!/usr/bin/env python3

import time

from qr_code_aruco_create import docking
from pose_estimate import publish_initial_pose, move_back
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def publish_locations():
    rclpy.init()
    node = rclpy.create_node('location_publisher')
    publisher = node.create_publisher(String, '/requested_location', 10)
    msg = String()

    try:
        while rclpy.ok():
            msg.data = "sink"
            node.get_logger().info(f'Publishing location: {msg.data}')
            publisher.publish(msg)
            time.sleep(5)  # Wait for 5 seconds

            msg.data = "elevator"
            node.get_logger().info(f'Publishing location: {msg.data}')
            publisher.publish(msg)
            time.sleep(5)  # Wait for 5 seconds
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down location publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    publish_locations()
    
if __name__ == '__main__':
    main()