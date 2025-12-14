import numpy as np
from all import transform_aruco_to_reference

# 测试示例
def test_transform():
    # 示例参数
    x_cam = 1.0  # aruco在相机前方1m
    y_cam = 0.5  # aruco在相机左方0.5m
    z_cam = 0.0  # aruco与相机同高
    pitch = np.radians(10)  # 相机俯仰10度
    yaw = np.radians(15)    # 相机偏航15度
    cam_offset = [-0.013, 0, 0.58709]  # 相机相对于参考点的偏移量
    
    print("=== 坐标转换测试 ===")
    print(f"相机坐标系下的aruco位置: [{x_cam}, {y_cam}, {z_cam}] m")
    print(f"相机姿态: pitch={np.degrees(pitch):.1f}度, yaw={np.degrees(yaw):.1f}度")
    print(f"相机相对于参考点的偏移量: {cam_offset} m")
    
    # 调用转换函数
    x_ref, y_ref, z_ref = transform_aruco_to_reference(x_cam, y_cam, z_cam, pitch, yaw, cam_offset)
    
    print(f"参考点坐标系下的aruco位置: [{x_ref:.4f}, {y_ref:.4f}, {z_ref:.4f}] m")
    print("====================")

if __name__ == "__main__":
    test_transform()