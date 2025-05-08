import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math
import yaml
import os
import argparse

class PoseSaver(Node):
    def __init__(self, name):
        super().__init__('pose_saver')
        self.name = name
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.save_pose)

    def save_pose(self):
        try:
            now = rclpy.time.Time()
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=2.0))

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            theta = math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz)

            save_path = "saved_goals.yaml"
            if os.path.exists(save_path):
                with open(save_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
            else:
                data = {}

            data[self.name] = {'x': x, 'y': y, 'theta': theta}
            with open(save_path, 'w') as f:
                yaml.dump(data, f)

            self.get_logger().info(f'Saved pose for "{self.name}": x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
            self.destroy_node()
        except Exception as e:
            self.get_logger().warn(f'Failed to get transform: {e}')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True, help='Name of the goal (e.g. sofa)')
    args = parser.parse_args()

    rclpy.init()
    node = PoseSaver(args.name)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
