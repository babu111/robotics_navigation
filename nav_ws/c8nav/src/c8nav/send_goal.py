#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import cv2

class NavClient(Node):
    def __init__(self):
        super().__init__('nav_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self.qr_detected = False

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()

    def send_goal(self, x, y, theta=0.0):
        if self.qr_detected:
            self.get_logger().info("QR detected, skipping goal send.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._client.wait_for_server()
        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def image_callback(self, msg):
        if self.qr_detected:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detector = cv2.QRCodeDetector()
            data, points, _ = detector.detectAndDecode(cv_image)

            if data:
                self.qr_detected = True
                self.get_logger().info(f'QR Code detected: {data}')

                if self._goal_handle and self._goal_handle.accepted:
                    self._goal_handle.cancel_goal_async()
                    self.get_logger().info("Navigation canceled due to QR code.")

                # TODO: compute the position 0.3m in front of QR and issue a new goal
                # e.g., use TF to get relative pose, then transform it into map frame

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
