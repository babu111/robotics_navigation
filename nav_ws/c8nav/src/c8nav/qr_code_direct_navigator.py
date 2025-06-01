#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import math
import time

class TimeBasedQRNavigator(Node):
    def __init__(self):
        super().__init__('time_based_qr_navigator')
        
        # Publisher for /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Last pose timestamp
        self.last_qr_time = 0.0

        # Speed configurations
        self.linear_speed = 0.1  # m/s
        self.angular_speed = 0.1 # rad/s

        # Target pose container
        self.qr_pose = None

        # State machine variables
        self.state = 'waiting_for_pose'
        self.state_start_time = None
        self.motion_duration = 0.0


        # Map states to handler methods
        self.handlers = {
            'rotate_to_target': self._handle_rotate_to_target,
            'move_forward': self._handle_move_forward,
            'rotate_to_face_qr': self._handle_rotate_to_face_qr
        }

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def set_qr_pose(self, x: float, y: float, theta: float):
        """Set QR Pose directly instead of subscribing to a topic"""
        current_time = time.time()
        self.last_qr_time = current_time

        msg = Pose2D(x=x, y=y, theta=theta)
        self.qr_pose = msg
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.angle_tan = math.atan2(y, x) *0.2
        self.get_logger().info(f"✅ Received QR Pose: x={x:.2f}, y={y:.2f}, θ={math.degrees(theta):.1f}°, "
                               f"angle_tan={math.degrees(self.angle_tan):.1f}°")



        if self.state == 'waiting_for_pose':
            self.motion_duration = abs(self.angle_tan) / self.angular_speed #############=======================
            self.turn_direction = 1 if self.angle_tan > 0 else -1
            self.state = 'rotate_to_target'
            self.state_start_time = self._now()
            self.get_logger().info(
                f"🌀 Rotate to angle {math.degrees(self.angle_tan):.1f}°, "
                f"duration {self.motion_duration:.2f}s"
            )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _handle_rotate_to_target(self):
        elapsed = self._now() - self.state_start_time
        if elapsed < self.motion_duration:
            cmd = Twist()
            cmd.angular.z = self.angular_speed * self.turn_direction
            self.cmd_pub.publish(cmd)
        else:
            self._stop()
            self.get_logger().info("✅ Rotation complete. Start moving forward.")
            distance = math.hypot(self.x, self.y) + 0.8  # add a small buffer to ensure we reach the target
            self.motion_duration = distance / self.linear_speed
            self.state = 'move_forward'
            self.state_start_time = self._now()
            self.get_logger().info(
                f"🚶 Move forward {distance:.2f}m, duration {self.motion_duration:.2f}s"
            )

    def _handle_move_forward(self):
        elapsed = self._now() - self.state_start_time
        if elapsed < self.motion_duration:
            cmd = Twist()
            cmd.linear.x = self.linear_speed
            self.cmd_pub.publish(cmd)
        else:
            self._stop()

            self.get_logger().info("🎯 Mission complete. Robot is aligned and positioned.")
            self.state = 'done'
            rclpy.shutdown()
            # self.get_logger().info("✅ Arrived in front of QR. Rotating to face QR.")
            # alpha = -1 * (math.pi + self.theta)
            # beta = self.angle_tan
            # self.turn_angle = beta - alpha
            # self.motion_duration = abs(self.turn_angle) / self.angular_speed  # add a small buffer
            # self.turn_direction = 1 if self.turn_angle > 0 else -1
            # self.state = 'rotate_to_face_qr'
            # self.state_start_time = self._now()
            # self.get_logger().info(
            #     f"🔁 Final rotation to face QR {math.degrees(self.turn_angle):.1f}°, "
            #     f"duration {self.motion_duration:.2f}s"
            # )

    def _handle_rotate_to_face_qr(self):
        elapsed = self._now() - self.state_start_time
        if elapsed < self.motion_duration:
            cmd = Twist()
            cmd.angular.z = self.angular_speed * self.turn_direction
            self.cmd_pub.publish(cmd)
        else:
            self._stop()
            self.get_logger().info("🎯 Mission complete. Robot is aligned and positioned.")
            self.state = 'done'
            rclpy.shutdown()

    def control_loop(self):
        if self.state == 'waiting_for_pose' or not self.qr_pose:
            return
        handler = self.handlers.get(self.state)
        if handler:
            handler()