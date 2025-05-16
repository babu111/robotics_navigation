import cv2
import numpy as np
import time

# ==== Parameters ====
QR_CODE_SIZE = 0.147  # meters (14.7 cm QR code size)

# Replace with your real calibrated camera intrinsics
camera_matrix = np.array([[640, 0, 320],
                          [0, 640, 240],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1))  # Assume no distortion

# ==== Helper: Convert rotation matrix to Euler angles (ZYX) ====
def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])  # Roll
        y = np.arctan2(-R[2, 0], sy)      # Pitch
        z = np.arctan2(R[1, 0], R[0, 0])  # Yaw
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.degrees(np.array([x, y, z]))  # Convert to degrees

# ==== Start camera and QR detection ====
cap = cv2.VideoCapture(0)
qr_detector = cv2.QRCodeDetector()

print("🔍 Starting QR code detection. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame from camera.")
        break

    data, bbox, _ = qr_detector.detectAndDecode(frame)

    if data and bbox is not None:
        print(f"\n✅ QR Code Detected: {data}")
        bbox = bbox.astype(np.float32)

        # Define 3D points of the QR code corners
        obj_points = np.array([
            [0, 0, 0],
            [QR_CODE_SIZE, 0, 0],
            [QR_CODE_SIZE, QR_CODE_SIZE, 0],
            [0, QR_CODE_SIZE, 0]
        ], dtype=np.float32)

        # 2D image points
        img_points = bbox[0]

        success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
        if success:
            print("📍 Translation (QR to Camera):\n", tvec)

            R, _ = cv2.Rodrigues(rvec)
            print("🔄 Rotation matrix:\n", R)

            # 4x4 transformation matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = tvec.T
            print("🔁 4x4 Transformation matrix (QR → Camera):\n", T)

            # Euler angles
            euler_angles = rotationMatrixToEulerAngles(R)
            print("📐 Euler Angles [Roll, Pitch, Yaw] (degrees):", euler_angles)

            # Calculate final yaw rotation needed for robot
            yaw = euler_angles[2]
            yaw = (yaw + 180) % 360 - 180  # Normalize to [-180, 180]
            print(f"🤖 Final angle robot should rotate (Yaw): {yaw:.2f} degrees")

            # Draw axes
            axis = np.float32([[0.05, 0, 0], [0, 0.05, 0], [0, 0, -0.05]])
            imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
            corner = tuple(img_points[0].ravel().astype(int))
            frame = cv2.line(frame, corner, tuple(imgpts[0].ravel().astype(int)), (255,0,0), 2)
            frame = cv2.line(frame, corner, tuple(imgpts[1].ravel().astype(int)), (0,255,0), 2)
            frame = cv2.line(frame, corner, tuple(imgpts[2].ravel().astype(int)), (0,0,255), 2)

            time.sleep(0.01)  # Prevent console spam
        else:
            print("❌ Pose estimation failed.")

    cv2.imshow("QR Code Detector with Pose", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
