#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(
            NavigateToPose_FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.feedback_callback,
            10
        )

        # Publisher
        self.status_pub = self.create_publisher(Nav2Status, '/nav2_status', 10)

        # Timer to publish status periodically
        self.create_timer(1, self.publish_status)

        self.current_position = Point()
        self.goal_position = Point()
        self.distance_remaining = 0.0
        self.estimated_time_remaining = 0.0
        self.last_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.declare_parameter('ready', False, ParameterDescriptor(description='Robot navigation readiness'))


    def goal_callback(self, msg: PoseStamped):
        # self.get_logger().info("Received goal")
        self.goal_position = msg.pose.position

    def status_callback(self, msg: Nav2Status):
        pass  # Empty callback; does nothing

    def feedback_callback(self, msg: NavigateToPose_FeedbackMessage):
        self.distance_remaining = msg.feedback.distance_remaining

        duration = msg.feedback.estimated_time_remaining
        self.estimated_time_remaining = duration.sec + duration.nanosec * 1e-9

    def compute_distance(self, p1: Point, p2: Point):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def publish_status(self):

        try:
            now = rclpy.time.Time()
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=2.0))

            self.current_position.x = transform.transform.translation.x
            self.current_position.y = transform.transform.translation.y

        except Exception as ex:
            self.get_logger().warn(f"Transform unavailable: {ex}")

        msg = Nav2Status()
        msg.position = self.current_position
        msg.distance_to_goal = self.distance_remaining
        msg.estimated_time_to_goal = self.estimated_time_remaining
        msg.ready = True

        self.status_pub.publish(msg)
        self.get_logger().info(
            f"Published: Pos({msg.position.x:.2f},{msg.position.y:.2f}), "
            f"Dist={msg.distance_to_goal:.2f}m, ETA={msg.estimated_time_to_goal:.1f}s, "
            f"Ready={msg.ready}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Nav2StatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
