# -*- coding: utf-8 -*-
import cv2
import time
import numpy as np

# ----------- 新增函数：获取齐次变换矩阵 -----------
def get_homogeneous_matrix(rvec, tvec):
    """将旋转向量和平移向量转换为4x4齐次变换矩阵"""
    # 将旋转向量转换为旋转矩阵
    R, _ = cv2.Rodrigues(rvec)
    
    # 构建4x4变换矩阵
    T = np.eye(4)
    T[:3, :3] = R          # 旋转部分
    T[:3, 3] = tvec.flatten()  # 平移部分
    return T

# ----------- 参数配置区 -----------
CAMERA_INDEX = 0          # 摄像头设备索引（通常0为默认摄像头）
MARKER_LENGTH = 0.183      # ArUco标记的实际物理边长（单位：米）
DICT_TYPE = cv2.aruco.DICT_7X7_1000  # 使用的ArUco字典类型

# 相机标定参数（已配置高精度参数）
camera_matrix = np.array([
    [2983.1575,    0.0,       1924.9410],
    [0.0,          2983.1575, 1080.2277],
    [0.0,          0.0,       1.0]
], dtype=np.float64)

dist_coeffs = np.array([
    -5.1219635,   # k1
    14.8576612,   # k2
    0.00245533,   # p1
    -0.00006054,  # p2
    -18.9697151   # k3
], dtype=np.float64)

# ----------- 初始化模块 -----------
aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_TYPE)
detector_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# ----------- 主循环 -----------
start_time = time.time()
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    corners, ids, rejected = detector.detectMarkers(frame)
    
    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, MARKER_LENGTH, camera_matrix, dist_coeffs
        )
        
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        for i in range(len(ids)):
            # ----------- 新增变换矩阵获取 -----------
            current_rvec = rvecs[i]
            current_tvec = tvecs[i]
            
            # 获取4x4变换矩阵
            T = get_homogeneous_matrix(current_rvec, current_tvec)
            
            # 终端输出矩阵信息
            marker_id = ids[i][0]
            np.set_printoptions(precision=4, suppress=True)
            print(f"\nMarker ID {marker_id} Transform Matrix:")
            print(T)
            
            # 在图像显示关键参数
            text = f"ID {marker_id}: T [{T[0,3]:.2f}, {T[1,3]:.2f}, {T[2,3]:.2f}]"
            center = tuple(map(int, corners[i][0].mean(axis=0)))
            cv2.putText(frame, text, (center[0]-200, center[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,255), 2)
            
            # 保持原有坐标系绘制
            cv2.drawFrameAxes(
                frame, camera_matrix, dist_coeffs,
                current_rvec, current_tvec, MARKER_LENGTH*0.5
            )

    # 显示每两秒一次的帧率
    current_time = time.time()
    elapsed_time = current_time - start_time
    print(f"Elapsed Time: {elapsed_time:.2f} seconds")
    if elapsed_time > 1:  # 检查是否接近两秒的间隔
        fps = cap.get(cv2.CAP_PROP_FPS)
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.imshow("ArUco Tracking", frame)
        start_time = current_time  # 重置计时器
        
    
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
