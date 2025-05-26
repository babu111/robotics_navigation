import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import math

class FaceMapSouthNode(Node):
    def __init__(self):
        super().__init__('face_map_south')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_yaw = -math.pi / 2  # 朝向 map 坐标系的负 y 轴（南）
        self.yaw_threshold = 0.05       # 允许误差（约 3°）
        self.kp = 0.8                   # 比例控制系数
        self.max_angular_speed = 1.0

        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        try:
            tf_time = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_link', tf_time)
            q = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            yaw_error = self.target_yaw - yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

            if abs(yaw_error) > self.yaw_threshold:
                twist = Twist()
                twist.angular.z = max(-self.max_angular_speed, min(self.kp * yaw_error, self.max_angular_speed))
                self.cmd_pub.publish(twist)
                self.get_logger().info(f"🔄 Turning to south, yaw error: {math.degrees(yaw_error):.1f}°")
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info("✅ Facing map south complete")
                self.timer.cancel()

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")


def main():
    rclpy.init()
    node = FaceMapSouthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
