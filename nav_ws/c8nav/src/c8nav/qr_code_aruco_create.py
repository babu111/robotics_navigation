
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose2D, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from qr_code_direct_navigator import TimeBasedQRNavigator

def get_homogeneous_matrix(rvec, tvec):
    """Convert rotation and translation vectors into 4x4 transformation matrix"""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T

class ArucoDetectorNode(Node):
    def __init__(self, navigator: TimeBasedQRNavigator):
        super().__init__('aruco_detector_node')
        self.navigator = navigator
        # Subscriber for camera images
        self.subscription = self.create_subscription(
                Image,
                '/oak_camera/rgb/image_raw',
                #'/oak_camera/rgb/image_raw/compressed',
                self.image_callback,
                10
            )

            
        # # 发布 QR 相对于 base_link 的二维位置与朝向
        # self.qr_pose_pub = self.create_publisher(Pose2D, '/qr_pose', 10)

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_matrix = np.array([
            [994.3858642578125, 0.0,               641.6470336914062],
            [0.0,               994.3858642578125, 360.0758972167969],
            [0.0,               0.0,               1.0]
        ], dtype=np.float64)
        
        # For OpenCV functions you can pass all 8 coefficients:
        self.dist_coeffs = np.array([
            -5.1219635009765625,
            14.857661247253418,
            0.0024553341791033745,
            -6.054222831153311e-05,
            -18.969715118408203,
            -5.192310333251953,
            15.136564254760742,
            -19.2453670501709
        ], dtype=np.float64)

        # Marker size in meters
        self.marker_length = 0.183

        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        # Camera-to-base_link transform (adjust based on your robot's setup)
        R_bc = np.array([
            [ 0,  0,  1],
            [-1,  0,  0],
            [ 0, -1,  0],
        ])

        # Translation (camera origin in base_link)
        t_bc = np.array([0.1, 0.0, 0.0])  # 0.1 m forward

        self.T_camera_base = np.eye(4)
        self.T_camera_base[:3, :3] = R_bc
        self.T_camera_base[:3,  3] = t_bc

        # Throttle control (process at most 2 Hz to reduce CPU load)
        self.last_process_time = 0.0

        self.get_logger().info("🚀 ArUco Detector Node Initialized!")

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_process_time < 0.5:  # Throttle to 2 Hz
            return
        self.last_process_time = current_time

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Detect markers
        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is not None:
            # Estimate pose for each detected marker
            #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                rvec = rvecs[i]
                tvec = tvecs[i]

                # Get marker-to-camera transform
                T_marker_camera = get_homogeneous_matrix(rvec, tvec)
                #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                # Compute marker-to-base_link transform
                T_marker_base = np.dot(self.T_camera_base, T_marker_camera)

                # Extract position (x, y, z)
                position = T_marker_base[:3, 3]

                # Extract orientation as Euler angles (roll, pitch, yaw)
                roll, pitch, yaw = self.rotation_matrix_to_euler(T_marker_base[:3, :3])

                self.get_logger().info(
                    f"\n🟢 Marker ID {marker_id}:\n"
                    f"   Position (x, y, z): {position}\n"
                    f"   Orientation (roll, pitch, yaw): [{roll:.3f}, {pitch:.3f}, {yaw:.3f}] rad\n"
                    f"   Distance to base_link: {np.linalg.norm(position):.2f} m"
                )
                
                # Publish TF for visualization in RViz
                #self.publish_tf(T_marker_base, marker_id)

                self.destroy_subscription(self.subscription)
                self.navigator.set_qr_pose(position[0], position[1], yaw)




    def rotation_matrix_to_euler(self, R):
        """Convert rotation matrix to Euler angles (roll, pitch, yaw)"""
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0.0

        return roll, pitch, yaw

    def publish_tf(self, T, marker_id):
        """Publish transform from base_link to marker"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = f'marker_{marker_id}'
        
        # Set translation
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]
        
        # Convert rotation matrix to quaternion
        q = self.rotation_matrix_to_quaternion(T[:3, :3])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion (x, y, z, w)"""
        # More numerically stable implementation
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
            
        return np.array([qx, qy, qz, qw])

def docking(args=None):
    # rclpy.init(args=args)
    # node = ArucoDetectorNode()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    rclpy.init(args=args)

    # 1) create both nodes
    navigator = TimeBasedQRNavigator()
    detector  = ArucoDetectorNode(navigator)  # pass navigator into detector

    # 2) use a MultiThreadedExecutor so both timers & subscriptions run
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    executor.add_node(detector)

    try:
        executor.spin()
    finally:
        pass
    #     detector.destroy_node()
    #     navigator.destroy_node()
    #     rclpy.shutdown()

def main(args=None):
    docking()

if __name__ == '__main__':
    main()