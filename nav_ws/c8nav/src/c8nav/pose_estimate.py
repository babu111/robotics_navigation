#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time

def publish_initial_pose(x, y, theta, frame_id='map'):
    """
    Publishes an initial 2D pose estimate to /initialpose for Nav2 localization.
    :param x: X coordinate in map frame
    :param y: Y coordinate in map frame
    :param theta: Orientation in radians (yaw)
    :param frame_id: Coordinate frame, usually 'map'
    """
    class PosePublisher(Node):
        def __init__(self):
            super().__init__('initial_pose_publisher')
            self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        def publish_pose(self):
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0.0

            # Convert yaw (theta) to quaternion
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw

            # Simple low uncertainty for testing
            msg.pose.covariance = [
                0.25, 0.0,  0.0, 0.0, 0.0, 0.0,
                0.0,  0.25, 0.0, 0.0, 0.0, 0.0,
                0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                0.0,  0.0,  0.0, 0.0, 0.0, 0.0685
            ]


            self.publisher.publish(msg)
            self.get_logger().info(f"Published initial pose: x={x:.2f}, y={y:.2f}, theta={theta:.2f} rad")

    rclpy.init()
    node = PosePublisher()
    node.publish_pose()
    print("Initial pose published. Waiting for subscribers...")

    # Give time for message to go out before shutdown
    time.sleep(3.0)
    node.destroy_node()
    rclpy.shutdown()
    print("Node shutdown. Exiting.")



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def move_back(distance, speed=0.2):
    """
    Move the robot backward by a specified distance.

    :param distance: Distance to move back in meters
    :param speed: Linear speed in m/s (default 0.2)
    """
    # Initialize the ROS2 node
    # rclpy.init()
    node = rclpy.create_node('simple_backward_mover')

    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    twist = Twist()
    twist.linear.x = -abs(speed)  # Ensure it's moving backward

    duration = distance / speed  # Total time to move
    rate = 20  # Hz
    interval = 1.0 / rate
    ticks = int(duration * rate)

    # Start publishing
    for _ in range(ticks):
        pub.publish(twist)
        time.sleep(interval)

    # Stop the robot
    twist.linear.x = 0.0
    pub.publish(twist)

    # Give some time to ensure the stop command is received
    time.sleep(0.2)

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()