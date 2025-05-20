#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from c8nav.msg import Nav2Status
import math
import time
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage



class Nav2StatusPublisher(Node):
    def __init__(self):
        super().__init__('nav2_status_publisher')

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(
            NavigateToPose_FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.feedback_callback,
            10
        )

        # Publisher
        self.status_pub = self.create_publisher(Nav2Status, '/nav2_status', 10)

        # Timer to publish status periodically
        self.create_timer(0.5, self.publish_status)

        self.current_position = None
        self.goal_position = Point()
        self.distance_remaining = 0.0
        self.estimated_time_remaining = 0.0
        self.last_time = self.get_clock().now().seconds_nanoseconds()[0]

    def odom_callback(self, msg: Odometry):
        self.get_logger().info("Received odom")
        self.current_position = msg.pose.pose.position

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info("Received goal")
        self.goal_position = msg.pose.position

    def feedback_callback(self, msg: NavigateToPose_FeedbackMessage):
        self.distance_remaining = msg.feedback.distance_remaining
        self.estimated_time_remaining = msg.feedback.estimated_time_remaining
        self.get_logger().info(
            f"[Nav2 ETA] Remaining: {self.distance_remaining:.2f} m, "
            f"ETA: {self.estimated_time_remaining:.1f} s"
        )

    def compute_distance(self, p1: Point, p2: Point):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def publish_status(self):
        if self.current_position is None:
            return

        msg = Nav2Status()
        msg.position = self.current_position
        msg.distance_to_goal = self.distance_remaining
        msg.estimated_time_to_goal = self.estimated_time_remaining

        self.status_pub.publish(msg)
        self.get_logger().info(
            f"Published: Pos({msg.position.x:.2f},{msg.position.y:.2f}), "
            f"Dist={msg.distance_to_goal:.2f}m, ETA={msg.estimated_time_to_goal:.1f}s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Nav2StatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
