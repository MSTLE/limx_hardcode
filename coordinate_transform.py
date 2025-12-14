import numpy as np

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

# 示例使用
def example():
    # 示例参数
    x_cam = 1.0  # aruco在相机前方1m
    y_cam = 0.5  # aruco在相机左方0.5m
    z_cam = 0.0  # aruco与相机同高
    pitch = np.radians(10)  # 相机俯仰10度
    yaw = np.radians(15)    # 相机偏航15度
    
    x_ref, y_ref, z_ref = transform_aruco_to_reference(x_cam, y_cam, z_cam, pitch, yaw)
    
    print(f"相机坐标系下的aruco位置: [{x_cam}, {y_cam}, {z_cam}]")
    print(f"相机姿态: pitch={np.degrees(pitch):.1f}度, yaw={np.degrees(yaw):.1f}度")
    print(f"参考点坐标系下的aruco位置: [{x_ref:.4f}, {y_ref:.4f}, {z_ref:.4f}]")

if __name__ == "__main__":
    example()