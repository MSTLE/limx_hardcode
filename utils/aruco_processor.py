"""
ArUco标记处理模块
包含ArUco检测和坐标变换计算
"""
import cv2
import cv2.aruco as aruco
import numpy as np
from .coordinate_transforms import (
    quaternion_to_rotation_matrix, 
    rotation_matrix_to_quaternion,
    quaternion_to_euler,
    transform_aruco_to_reference
)


class ArucoProcessor:
    def __init__(self, camera_matrix, dist_coeffs):
        """
        初始化ArUco处理器
        
        参数:
            camera_matrix: 相机内参矩阵
            dist_coeffs: 畸变系数
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        
        # ArUco字典和检测器
        self.aruco_dict_6x6 = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters_6x6 = aruco.DetectorParameters()
        self.detector_6x6 = aruco.ArucoDetector(self.aruco_dict_6x6, self.parameters_6x6)
        
        self.aruco_dict_4x4 = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters_4x4 = aruco.DetectorParameters()
        self.detector_4x4 = aruco.ArucoDetector(self.aruco_dict_4x4, self.parameters_4x4)
        
        # 标记尺寸
        self.marker_length_66 = 0.10  # 10cm
        self.marker_length_44 = 0.04  # 4cm
        
        # 标记点坐标
        self.marker_points_66 = np.array([
            [-self.marker_length_66/2,  self.marker_length_66/2, 0],
            [ self.marker_length_66/2,  self.marker_length_66/2, 0],
            [ self.marker_length_66/2, -self.marker_length_66/2, 0],
            [-self.marker_length_66/2, -self.marker_length_66/2, 0]
        ], dtype=np.float32)
        
        self.marker_points_44 = np.array([
            [-self.marker_length_44/2,  self.marker_length_44/2, 0],
            [ self.marker_length_44/2,  self.marker_length_44/2, 0],
            [ self.marker_length_44/2, -self.marker_length_44/2, 0],
            [-self.marker_length_44/2, -self.marker_length_44/2, 0]
        ], dtype=np.float32)
        
        # 相机到头部的变换矩阵（从URDF获取）
        from .coordinate_transforms import homogeneous_transform
        T_head_to_camera = homogeneous_transform(
            xyz=np.array([0.07453, 0.0175, 0.065]),
            rpy=np.array([0.0, 1.5708, 0.0])
        )
        self.CAMERA_TO_HEAD = np.linalg.inv(T_head_to_camera)
    
    def detect_markers(self, frame):
        """
        检测图像中的ArUco标记
        
        参数:
            frame: 输入图像
            
        返回:
            dict: 检测结果，包含corners, ids等信息
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 检测6x6标记
        corners66, ids66, _ = self.detector_6x6.detectMarkers(gray)
        
        # 检测4x4标记
        corners44, ids44, _ = self.detector_4x4.detectMarkers(gray)
        
        return {
            '6x6': {'corners': corners66, 'ids': ids66},
            '4x4': {'corners': corners44, 'ids': ids44}
        }
    
    def draw_markers(self, frame, detection_results):
        """
        在图像上绘制检测到的标记
        
        参数:
            frame: 输入图像
            detection_results: 检测结果
            
        返回:
            frame: 绘制后的图像
        """
        # 绘制6x6标记
        if detection_results['6x6']['ids'] is not None:
            aruco.drawDetectedMarkers(frame, detection_results['6x6']['corners'], detection_results['6x6']['ids'])
        
        # 绘制4x4标记
        if detection_results['4x4']['ids'] is not None:
            aruco.drawDetectedMarkers(frame, detection_results['4x4']['corners'], detection_results['4x4']['ids'])
        
        return frame
    
    def calculate_aruco_transforms(self, rvec, tvec, head_quat, roll_offset=0.0, offset=np.array([0.11, 0.0, -0.03])):
        """
        计算ArUco码在base_link坐标系中的位置和姿态，以及左臂末端的目标位置和姿态
        
        参数:
        rvec: ArUco码的旋转向量
        tvec: ArUco码的平移向量
        head_quat: 头部姿态四元数
        roll_offset: 末端roll角度偏移（弧度）
        offset: 末端相对于ArUco码的偏移量（米）
        
        返回:
        tuple: (marker_pos_base, marker_quat_base, target_left_hand_pos, target_left_hand_quat)
        """
        # 计算ArUco码在相机坐标系中的变换矩阵
        R, _ = cv2.Rodrigues(rvec)
        
        # OpenCV solvePnP返回的坐标系：x向右，y向下，z向前
        # 转换为相机坐标系：x向前，y向左，z向上
        # 坐标系变换：x→z, y→-x, z→-y
        tvec_original = tvec.flatten()
        # 对位置进行坐标系变换
        marker_pos_camera = np.array([tvec_original[2], -tvec_original[0], -tvec_original[1]])
        
        # 对旋转矩阵也进行同样的坐标系变换
        # 定义坐标系变换矩阵
        R_coord_transform = np.array([
            [0, 0, 1],   # x' = z
            [-1, 0, 0],  # y' = -x
            [0, -1, 0]   # z' = -y
        ])
        # 应用坐标系变换到旋转矩阵
        R = R_coord_transform @ R @ R_coord_transform.T
        
        T_camera_to_marker = np.eye(4)
        T_camera_to_marker[:3, :3] = R
        T_camera_to_marker[:3, 3] = marker_pos_camera
        
        # 从head_quat中提取pitch和yaw角度
        head_roll, head_pitch, head_yaw = quaternion_to_euler(head_quat)
        
        # 使用transform_aruco_to_reference函数将相机坐标系下的位置转换为base_link坐标系下的位置
        # 相机相对于base_link的偏移量：从URDF中获取的固定位置
        cam_offset = np.array([-0.013, 0, 0.58709])
        
        # 调用transform_aruco_to_reference函数进行坐标转换
        marker_pos_base_x, marker_pos_base_y, marker_pos_base_z = transform_aruco_to_reference(
            marker_pos_camera[0], marker_pos_camera[1], marker_pos_camera[2],
            head_pitch, head_yaw,
            cam_offset
        )
        
        # 组装转换后的位置
        marker_pos_base = np.array([marker_pos_base_x, marker_pos_base_y, marker_pos_base_z])
        
        # 保留原来的旋转矩阵计算，用于姿态转换
        R_head_to_base = quaternion_to_rotation_matrix(head_quat)
        
        # 头部位置（从URDF中获取）
        head_pos_base = np.array([-0.013, 0, 0.58709])
        
        # 构建头部到base_link的变换矩阵
        T_head_to_base = np.eye(4)
        T_head_to_base[:3, :3] = R_head_to_base
        T_head_to_base[:3, 3] = head_pos_base
        
        # 计算相机到base_link的变换矩阵，用于姿态转换
        T_camera_to_base = T_head_to_base @ self.CAMERA_TO_HEAD
        
        # 计算ArUco码在base_link坐标系中的姿态（四元数）
        # 首先计算ArUco码在base_link坐标系中的旋转矩阵
        R_marker_to_camera = R
        R_marker_to_base = T_camera_to_base[:3, :3] @ R_marker_to_camera
        
        # 将旋转矩阵转换为四元数
        marker_quat_base = rotation_matrix_to_quaternion(R_marker_to_base)
        
        # 计算带有偏移的目标位置：aruco中心偏移offset
        # base_link坐标系：x向前，y向左，z向上
        target_left_hand_pos = marker_pos_base + offset
        
        # 应用roll_offset到末端姿态
        if roll_offset != 0.0:
            # 将四元数转换为旋转矩阵
            R_target = quaternion_to_rotation_matrix(marker_quat_base)
            # 创建roll旋转矩阵
            R_roll = np.array([
                [np.cos(roll_offset), -np.sin(roll_offset), 0],
                [np.sin(roll_offset), np.cos(roll_offset), 0],
                [0, 0, 1]
            ])
            # 应用roll偏移
            R_target_rolled = R_target @ R_roll
            # 转换回四元数
            target_left_hand_quat = rotation_matrix_to_quaternion(R_target_rolled)
        else:
            # 使用aruco码的姿态作为末端姿态
            target_left_hand_quat = marker_quat_base
        
        return marker_pos_base, marker_quat_base, target_left_hand_pos, target_left_hand_quat
    
    def process_marker(self, frame, target_id=0, marker_type='6x6'):
        """
        处理特定ID的标记
        
        参数:
            frame: 输入图像
            target_id: 目标标记ID
            marker_type: 标记类型 ('6x6' 或 '4x4')
            
        返回:
            tuple: (success, rvec, tvec) 或 (False, None, None)
        """
        detection_results = self.detect_markers(frame)
        
        if detection_results[marker_type]['ids'] is not None:
            ids = detection_results[marker_type]['ids']
            corners = detection_results[marker_type]['corners']
            
            for i in range(len(ids)):
                if ids[i] == target_id:
                    current_corners = corners[i].reshape((4, 2))
                    
                    # 选择对应的标记点
                    marker_points = self.marker_points_66 if marker_type == '6x6' else self.marker_points_44
                    
                    success, rvec, tvec = cv2.solvePnP(
                        marker_points, 
                        current_corners, 
                        self.camera_matrix, 
                        self.dist_coeffs
                    )
                    
                    if success:
                        return True, rvec, tvec
        
        return False, None, None