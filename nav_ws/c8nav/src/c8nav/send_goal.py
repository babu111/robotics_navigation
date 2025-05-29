import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import argparse

class NavClient(Node):
    def __init__(self):
        super().__init__('nav_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, theta=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Simplified for facing forward

        # Convert yaw (theta) to quaternion
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._client.wait_for_server()
        self._send_future = self._client.send_goal_async(goal_msg)
        self._send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description='Send navigation goal to Nav2')
    parser.add_argument('--x', type=float, required=True, help='X coordinate of goal')
    parser.add_argument('--y', type=float, required=True, help='Y coordinate of goal')
    parser.add_argument('--theta', type=float, default=0.0, help='Yaw angle in radians')
    args = parser.parse_args()

    rclpy.init()
    node = NavClient()
    node.send_goal(x=args.x, y=args.y, theta=args.theta)  # Change coordinates as needed
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()


# ros2 run c8nav send_goal.py --x 2.0 --y 1.5 --theta 1.57