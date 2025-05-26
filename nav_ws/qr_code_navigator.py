#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from tf_transformations import euler_from_quaternion
import math
import time 

class TimeBasedQRNavigator(Node):
    def __init__(self):
        super().__init__('time_based_qr_navigator')

        # Publisher for /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to QR code pose
        self.last_qr_time = 0.0
        self.qr_sub = self.create_subscription(Pose2D, '/qr_pose', self.qr_pose_callback, 10)



        # Speed configs
        self.linear_speed = 0.1  # m/s
        self.angular_speed = 0.3  # rad/s

        # Thresholds
        self.threshold_angle = 0.05  # rad

        # Target from /qr_pose
        self.qr_pose = None

        # Internal state machine
        self.state = 'waiting_for_pose'
        self.state_start_time = None
        self.motion_duration = 0.0

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def qr_pose_callback(self, msg):
        current_time = time.time()
        # if current_time - self.last_qr_time < 20.0:
        #     return  # Skip this message
        if self.qr_sub is not None:
            self.destroy_subscription(self.qr_sub)
            self.qr_sub = None
        self.last_qr_time = current_time

        self.qr_pose = msg
        self.get_logger().info(f"✅ Received QR Pose: x={msg.x:.2f}, y={msg.y:.2f}, θ={math.degrees(msg.theta):.1f}°")
        if self.state == 'waiting_for_pose':
            # Step 1: Rotate to face target
            self.angle_to_qr = math.atan2(msg.y, msg.x)
            # self.motion_duration = abs(self.angle_to_qr) / self.angular_speed
            self.motion_duration = abs(self.qr_pose.theta) / self.angular_speed
            self.state = 'rotate_to_target'
            self.state_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"🌀 Rotate to angle {math.degrees(self.angle_to_qr):.1f}°, duration {self.motion_duration:.2f}s")

    def control_loop(self):
        if self.state == 'waiting_for_pose' or self.qr_pose is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.state_start_time
        cmd = Twist()

        if self.state == 'rotate_to_target':
            if elapsed < self.motion_duration:
                # cmd.angular.z = self.angular_speed if self.angle_to_qr > 0 else -self.angular_speed
                cmd.angular.z = self.angular_speed if self.qr_pose.theta > 0 else -self.angular_speed
                self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())  # stop
                self.get_logger().info("✅ Rotation complete. Start moving forward.")
                # Step 2: Translate forward
                dx = self.qr_pose.x
                dy = self.qr_pose.y
                dist = math.sqrt(dx**2 + dy**2)
                self.motion_duration = dist / self.linear_speed
                self.state = 'move_forward'
                self.state_start_time = current_time
                self.get_logger().info(f"🚶 Move forward {dist:.2f}m, duration {self.motion_duration:.2f}s")

        elif self.state == 'move_forward':
            if elapsed < self.motion_duration:
                cmd.linear.x = self.linear_speed
                self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info("✅ Arrived in front of QR. Rotating to face QR.")
                # Step 3: Rotate to face QR
                self.motion_duration = abs(self.angle_to_qr - self.qr_pose.theta) / self.angular_speed
                # self.motion_duration = abs(self.qr_pose.theta) / self.angular_speed
                self.state = 'rotate_to_face_qr'
                self.state_start_time = current_time 
                self.get_logger().info(f"🔁 Final rotation to face QR θ={math.degrees(self.qr_pose.theta):.1f}°, duration {self.motion_duration:.2f}s")

        elif self.state == 'rotate_to_face_qr':
            if elapsed < self.motion_duration:
                cmd.angular.z = self.angular_speed if (self.qr_pose.theta - self.qr_pose.theta) > 0 else -self.angular_speed
                self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info("🎯 Mission complete. Robot is aligned and positioned.")
                self.state = 'done'
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TimeBasedQRNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
