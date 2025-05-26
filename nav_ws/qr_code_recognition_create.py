import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import time

class QRCodePoseEstimator(Node):
    def __init__(self):
        super().__init__('qr_code_pose_estimator')

        self.qr_detector = cv2.QRCodeDetector()
        self.bridge = CvBridge()
        self.last_print_time = time.time()

        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            10
        )

        self.QR_CODE_SIZE = 0.147  # 14.7cm
        self.camera_matrix = np.array([[640, 0, 320],
                                       [0, 640, 240],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1))  # assume no distortion

    def rotationMatrixToEulerAngles(self, R):
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0
        return np.degrees(np.array([x, y, z]))

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        data, bbox, _ = self.qr_detector.detectAndDecode(frame)

        # Only proceed if bbox is valid
        self.get_logger().info("=" * 50)
        if bbox is not None and bbox.shape[1] == 4:
            area = cv2.contourArea(bbox.astype(np.float32))
            if area > 0 and data:
                bbox = bbox.astype(np.float32)
                obj_points = np.array([
                    [0, 0, 0],
                    [self.QR_CODE_SIZE, 0, 0],
                    [self.QR_CODE_SIZE, self.QR_CODE_SIZE, 0],
                    [0, self.QR_CODE_SIZE, 0]
                ], dtype=np.float32)
                img_points = bbox[0]

                success, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)
                if success:
                    now = time.time()
                    if now - self.last_print_time >= 2.0:
                        self.last_print_time = now

                        R, _ = cv2.Rodrigues(rvec)
                        euler = self.rotationMatrixToEulerAngles(R)
                        yaw = (euler[2] + 180) % 360 - 180

                        T = np.eye(4)
                        T[:3, :3] = R
                        T[:3, 3] = tvec.T

                        self.get_logger().info(f"\n✅ QR Code Detected: {data}")
                        self.get_logger().info(f"📍 Translation vector (tvec):\n{tvec}")
                        self.get_logger().info(f"🔄 Rotation matrix (R):\n{R}")
                        self.get_logger().info(f"🔁 4x4 Transformation matrix (QR → Camera):\n{T}")
                        self.get_logger().info(f"📐 Euler angles [roll, pitch, yaw] (degrees): {euler}")
                        self.get_logger().info(f"🤖 Final angle robot should rotate (Yaw): {yaw:.2f} degrees")
                else:
                    self.get_logger().warn("⚠️ Pose estimation failed.")
            else:
                self.get_logger().warn("⚠️ Invalid QR bbox or no data.")
        else:
            self.get_logger().info("📷 No QR code detected in this frame.")

def main(args=None):
    rclpy.init(args=args)
    node = QRCodePoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
