"""
坐标变换工具模块
包含四元数、旋转矩阵、欧拉角之间的转换函数
"""
import numpy as np


def quaternion_to_rotation_matrix(q):
    """将四元数转换为旋转矩阵"""
    x, y, z, w = q
    R = np.array([
        [1-2*y**2-2*z**2, 2*x*y-2*z*w, 2*x*z+2*y*w],
        [2*x*y+2*z*w, 1-2*x**2-2*z**2, 2*y*z-2*x*w],
        [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x**2-2*y**2]
    ])
    return R


def rotation_matrix(rpy):
    """根据roll-pitch-yaw角度生成旋转矩阵（使用XYZ欧拉角旋转顺序，符合URDF规范）"""
    roll, pitch, yaw = rpy
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    # 使用XYZ欧拉角旋转顺序：先绕x轴roll，再绕y轴pitch，最后绕z轴yaw
    # 这符合ROS URDF规范
    return Rz @ Ry @ Rx


def homogeneous_transform(xyz, rpy):
    """根据位置和姿态生成齐次变换矩阵"""
    R = rotation_matrix(rpy)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def rotation_matrix_to_quaternion(R):
    """将旋转矩阵转换为四元数"""
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [x, y, z, w]


def quaternion_to_euler(quat):
    """将四元数转换为欧拉角（roll, pitch, yaw），单位：弧度"""
    x, y, z, w = quat
    
    # 计算roll (x轴旋转)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # 计算pitch (y轴旋转)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # 避免奇点
    else:
        pitch = np.arcsin(sinp)
    
    # 计算yaw (z轴旋转)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def convert_to_float_list(arr):
    """将 numpy 数组转换为 Python 列表，并确保元素类型为 float"""
    if hasattr(arr, 'tolist'):
        return [float(x) for x in arr.tolist()]
    elif isinstance(arr, list):
        return [float(x) for x in arr]
    else:
        return arr


def transform_aruco_to_reference(x_cam, y_cam, z_cam, pitch, yaw, cam_offset=[-0.013, 0, 0.58709]):
    """
    将aruco码在相机坐标系下的位置转换为参考点坐标系下的位置
    
    参数:
        x_cam, y_cam, z_cam: aruco码在相机坐标系下的坐标 [m]
        pitch: 相机的俯仰角 [弧度]
        yaw: 相机的偏航角 [弧度]
        cam_offset: 相机相对于参考点的偏移量 [x, y, z] [m]
        
    返回:
        x_ref, y_ref, z_ref: aruco码在参考点坐标系下的坐标 [m]
        
    坐标系定义:
        前方为+x，左方为+y，上方为+z
    """
    # 1. 将aruco码坐标转换为齐次坐标
    aruco_cam = np.array([x_cam, y_cam, z_cam, 1])
    
    # 2. 计算旋转矩阵 (pitch是绕y轴旋转，yaw是绕z轴旋转)
    # 旋转顺序：先绕y轴旋转pitch，再绕z轴旋转yaw
    # 绕y轴旋转pitch
    R_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch), 0],
        [0, 1, 0, 0],
        [-np.sin(pitch), 0, np.cos(pitch), 0],
        [0, 0, 0, 1]
    ])
    
    # 绕z轴旋转yaw
    R_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, 0],
        [np.sin(yaw), np.cos(yaw), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # 组合旋转矩阵：先pitch后yaw
    R = np.dot(R_yaw, R_pitch)
    
    # 3. 计算平移矩阵
    T = np.eye(4)
    T[0:3, 3] = cam_offset
    
    # 4. 计算从相机坐标系到参考点坐标系的变换矩阵：先旋转后平移
    # 注意：变换顺序是先旋转相机坐标系中的点，再平移到参考点坐标系
    transform_matrix = np.dot(T, R)
    
    # 5. 应用变换
    aruco_ref = np.dot(transform_matrix, aruco_cam)
    
    return aruco_ref[0], aruco_ref[1], aruco_ref[2]