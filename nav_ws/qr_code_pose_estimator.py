
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D, PoseStamped, TransformStamped
from cv_bridge import CvBridge

import cv2
import numpy as np
import math
import time

from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
import tf2_geometry_msgs
from tf_transformations import quaternion_from_matrix


class QRCodePoseEstimator(Node):
    def __init__(self):
        super().__init__('qr_code_pose_estimator')

        # 初始化二维码检测器与图像桥
        self.qr_detector = cv2.QRCodeDetector()
        self.bridge = CvBridge()
        self.last_print_time = time.time()

        # 订阅图像数据
        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            10
        )

        # 发布 QR 相对于 base_link 的二维位置与朝向
        self.qr_pose_pub = self.create_publisher(Pose2D, '/qr_pose', 10)

        # 相机内参与二维码尺寸
        self.QR_CODE_SIZE = 0.147  # 14.7cm
        self.camera_matrix = np.array([[640, 0, 320],
                                       [0, 640, 240],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1))  # assume no distortion

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_tf()

        self.get_logger().info("✅ QRCodePoseEstimator 节点已启动")

    def broadcast_static_tf(self):
        """广播 base_link → camera_link 静态 TF"""
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'camera_link'
        static_tf.transform.translation.x = 0.10
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_tf)
        self.get_logger().info("📡 已广播 base_link → camera_link 静态 TF")

    def convert_tvec_opencv_to_ros(self, tvec_cv):
        """OpenCV 坐标 → ROS 坐标转换（位置）"""
        return np.array([
            tvec_cv[2],     # ROS x = OpenCV z
            -tvec_cv[0],    # ROS y = -OpenCV x
            -tvec_cv[1]     # ROS z = -OpenCV y
        ])

    # def convert_rvec_to_quaternion_ros(self, rvec_cv):
    #     """OpenCV rvec → ROS 坐标下四元数"""
    #     R_cv, _ = cv2.Rodrigues(rvec_cv)
    #     T = np.eye(4)
    #     T[:3, :3] = R_cv
    #     quat = quaternion_from_matrix(T)
    #     return quat  # [x, y, z, w]
    def convert_rvec_to_quaternion_ros(self, rvec_cv):
        """Convert OpenCV rvec to ROS-style quaternion (camera_link)"""
        R_cv, _ = cv2.Rodrigues(rvec_cv)
    
        # OpenCV -> ROS camera_link axis conversion
        # R_cv_to_ros: converts OpenCV pose to ROS camera_link pose
        R_cv_to_ros = np.array([
            [0, 0, 1],   # ROS x = OpenCV z
            [-1, 0, 0],  # ROS y = -OpenCV x
            [0, -1, 0]   # ROS z = -OpenCV y
        ])
    
        R_ros = R_cv_to_ros @ R_cv @ R_cv_to_ros.T
    
        T = np.eye(4)
        T[:3, :3] = R_ros
        quat = quaternion_from_matrix(T)
        return quat  # [x, y, z, w]



    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        data, bbox, _ = self.qr_detector.detectAndDecode(frame)

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

                        # ========== 坐标轴转换 ==========
                        tvec_ros = self.convert_tvec_opencv_to_ros(tvec)
                        quat_ros = self.convert_rvec_to_quaternion_ros(rvec)
                        self.get_logger().info(f"✅ QR in camera_link:\n tvec_ros: {tvec_ros.flatten()}\n rvec: {rvec.flatten()}")

                        # 构造 PoseStamped（camera_link 下）
                        pose_in_camera = PoseStamped()
                        pose_in_camera.header.frame_id = 'camera_link'
                        pose_in_camera.header.stamp = self.get_clock().now().to_msg()
                        pose_in_camera.pose.position.x = float(tvec_ros[0])
                        pose_in_camera.pose.position.y = float(tvec_ros[1])
                        pose_in_camera.pose.position.z = float(tvec_ros[2])
                        pose_in_camera.pose.orientation.x = quat_ros[0]
                        pose_in_camera.pose.orientation.y = quat_ros[1]
                        pose_in_camera.pose.orientation.z = quat_ros[2]
                        pose_in_camera.pose.orientation.w = quat_ros[3]

                        try:
                            # 使用 TF2 转换为 base_link 下的 Pose
                            pose_in_base = self.tf_buffer.transform(
                                pose_in_camera,
                                'base_link',
                                timeout=rclpy.duration.Duration(seconds=0.3)
                            )

                            x = pose_in_base.pose.position.x
                            y = pose_in_base.pose.position.y
                            z = pose_in_base.pose.position.z

                            q = pose_in_base.pose.orientation
                            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))

                            msg = Pose2D()
                            msg.x = x
                            msg.y = y
                            msg.theta = yaw
                            self.qr_pose_pub.publish(msg)

                            self.get_logger().info(f"📍 QR(base_link): x={x:.2f} m, y={y:.2f} m, z= {z:.2f}, yaw={math.degrees(yaw):.1f}°")

                        except Exception as e:
                            self.get_logger().warn(f"⚠️ 坐标变换失败: {e}")
                else:
                    self.get_logger().warn("⚠️ 位姿解算失败")
            else:
                self.get_logger().warn("⚠️ 非法二维码 bbox 或未检测到数据")
        else:
            self.get_logger().info("📷 当前帧未检测到二维码")


def main(args=None):
    rclpy.init(args=args)
    node = QRCodePoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
