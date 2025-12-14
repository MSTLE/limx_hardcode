import cv2
import cv2.aruco as aruco
import numpy as np
import time
import robot_control_api

import camera_api

camera_reveiver = camera_api.RobotImageReceiver()

robot_controller = robot_control_api.RobotClient()

"""
======
矩阵变化函数
======
"""
# 定义四元数到旋转矩阵的转换函数
def quaternion_to_rotation_matrix(q):
    """将四元数转换为旋转矩阵"""
    x, y, z, w = q
    R = np.array([
        [1-2*y**2-2*z**2, 2*x*y-2*z*w, 2*x*z+2*y*w],
        [2*x*y+2*z*w, 1-2*x**2-2*z**2, 2*y*z-2*x*w],
        [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x**2-2*y**2]
    ])
    return R

# 定义旋转矩阵生成函数
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

# 定义齐次变换矩阵生成函数
def homogeneous_transform(xyz, rpy):
    """根据位置和姿态生成齐次变换矩阵"""
    R = rotation_matrix(rpy)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T

# 定义旋转矩阵到四元数的转换函数
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

# 定义四元数到欧拉角的转换函数
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

# 从URDF注释中提取相机相对于头部的变换矩阵
# 根据URDF注释: <joint name="head_camera_joint" type="fixed">
#               <origin rpy="0.0 1.5708 0.0" xyz="0.07453 0.0175 0.065"/>
# 注意：URDF中的origin表示的是子坐标系(相机)相对于父坐标系(头部)的变换
# 即：T_head_to_camera，而我们需要的是T_camera_to_head
# 所以需要取逆矩阵
T_head_to_camera = homogeneous_transform(
    xyz=np.array([0.07453, 0.0175, 0.065]),
    rpy=np.array([0.0, 1.5708, 0.0])
)

# 计算相机到头部的变换矩阵：T_camera_to_head = T_head_to_camera的逆矩阵
CAMERA_TO_HEAD = np.linalg.inv(T_head_to_camera)

# 打印变换矩阵，用于调试
print(f"\n=== 变换矩阵调试信息 ===")
print(f"T_head_to_camera:\n{T_head_to_camera}")
print(f"CAMERA_TO_HEAD (逆矩阵):\n{CAMERA_TO_HEAD}")
print("=============================")

# 机器人头部相对于base_link的变换矩阵（初始状态，无旋转）
def head_to_base_transform(head_pitch, head_yaw):
    """根据头部的pitch和yaw角度计算头部到base_link的变换矩阵"""
    # 头部yaw和pitch对应的旋转矩阵
    # 注意：URDF中head_yaw_joint的axis是xyz="0 0 1"，head_pitch_joint的axis是xyz="0 1 0"
    rpy = np.array([0.0, head_pitch, head_yaw])
    return homogeneous_transform(
        xyz=np.array([-0.013, 0, 0.58709]),  # 从URDF中head_pitch_joint的origin获取
        rpy=rpy
    )

# 将 numpy 数组转换为 Python 列表，并确保元素类型为 float
def convert_to_float_list(arr):
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

def calculate_aruco_transforms(rvec, tvec, head_quat, CAMERA_TO_HEAD, quaternion_to_rotation_matrix, rotation_matrix_to_quaternion, roll_offset=0.0, offset=np.array([0.11, 0.0, -0.03])):
    """
    计算ArUco码在base_link坐标系中的位置和姿态，以及左臂末端的目标位置和姿态
    
    参数:
    rvec: ArUco码的旋转向量
    tvec: ArUco码的平移向量
    head_quat: 头部姿态四元数
    CAMERA_TO_HEAD: 相机到头部的变换矩阵
    quaternion_to_rotation_matrix: 四元数到旋转矩阵的转换函数
    rotation_matrix_to_quaternion: 旋转矩阵到四元数的转换函数
    roll_offset: 末端roll角度偏移（弧度）
    offset: 末端相对于ArUco码的偏移量（米）
    
    返回:
    tuple: (marker_pos_base, marker_quat_base, target_left_hand_pos, target_left_hand_quat)
        marker_pos_base: ArUco码在base_link坐标系中的位置
        marker_quat_base: ArUco码在base_link坐标系中的姿态（四元数）
        target_left_hand_pos: 左臂末端目标位置
        target_left_hand_quat: 左臂末端目标姿态
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
    
    # ArUco码在相机坐标系中的位置已经在上面转换过了
    
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
    T_camera_to_base = T_head_to_base @ CAMERA_TO_HEAD
    

    
    # 计算ArUco码在base_link坐标系中的姿态（四元数）
    # 首先计算ArUco码在base_link坐标系中的旋转矩阵
    R_marker_to_camera = R
    R_marker_to_base = T_camera_to_base[:3, :3] @ R_marker_to_camera
    
    # 将旋转矩阵转换为四元数
    marker_quat_base = rotation_matrix_to_quaternion(R_marker_to_base)
    
    # 计算带有偏移的目标位置：aruco中心偏移offset
    # base_link坐标系：x向前，y向左，z向上
    target_left_hand_pos = marker_pos_base + offset

    # print(marker_pos_camera)
    # print(marker_pos_base)

    
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



## =============== main ===============
if __name__ == "__main__":

    print("识别aruco 6*6 and 4*4")
    print("="*60)

    target_marker_id_66 = 1
    approach_distance = 0.11  # 接近到距离标记11cm
    aruco_dict_type_66 = aruco.DICT_6X6_250
    aruco_dict_type_44 = aruco.DICT_4X4_250
    marker_length_66 = 0.10  # 10cm
    marker_length_44 = 0.04  # 7cm
    
    # 初始化变量
    latest_marker_pos = None
    latest_marker_quat = None
    latest_head_quat = None
    
    # 相机控制参数
    camera_pitch = np.radians(-180)  # 初始pitch角度（弧度）
    camera_yaw = np.radians(-180)    # 初始yaw角度（弧度）
    angle_step = 5.0    # 每次按键移动的角度（度）
    angle_step_rad = np.radians(angle_step)  # 转换为弧度
    
    # 左臂末端跟随模式
    left_arm_follow_mode = False  # 初始为退出跟随模式
    
    # 末端offset调整（单位：米）
    offset = np.array([-0.02, 0.0, -0.10])  # 初始偏移：前方11cm，上方3cm
    offset_step = 0.02  # 每次按键调整3cm
    
    # 末端roll角度offset调整（单位：弧度）
    roll_offset = np.radians(6.0)  # 初始roll偏移为0
    roll_step = np.radians(3.0)  # 每次按键调整5°
    
    # 末端pitch和yaw角度offset调整（单位：弧度）
    pitch_offset = np.radians(-111)  # 初始pitch偏移为0
    yaw_offset = 0.0    # 初始yaw偏移为0
    pitch_step = np.radians(3.0)  # 每次按键调整3°
    yaw_step = np.radians(3.0)    # 每次按键调整3°


    # 相机内参 (RealSense标定)
    camera_matrix = np.array([
        [907.719360351562, 0.0, 651.812194824219],
        [0.0, 908.330444335938, 387.889526367188],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)


    camera_reveiver.start()
    print("等待图像...")
    for i in range(50):
        if camera_reveiver.get_frame() is not None:
            print(f"✅ 图像接收成功!")
            break
        time.sleep(0.1)
    else:
        print("❌ 未接收到图像")
        exit(1)

    aruco_dict_6x6 = aruco.getPredefinedDictionary(aruco_dict_type_66)
    parameters_6x6 = aruco.DetectorParameters()
    detector_6x6 = aruco.ArucoDetector(aruco_dict_6x6, parameters_6x6)

    aruco_dict_4x4 = aruco.getPredefinedDictionary(aruco_dict_type_44)
    parameters_4x4 = aruco.DetectorParameters()
    detector_4x4 = aruco.ArucoDetector(aruco_dict_4x4, parameters_4x4)

    marker_points_66 = np.array([
        [-marker_length_66/2,  marker_length_66/2, 0],
        [ marker_length_66/2,  marker_length_66/2, 0],
        [ marker_length_66/2, -marker_length_66/2, 0],
        [-marker_length_66/2, -marker_length_66/2, 0]
    ], dtype=np.float32)

    marker_points_44 = np.array([
        [-marker_length_44/2,  marker_length_44/2, 0],
        [ marker_length_44/2,  marker_length_44/2, 0],
        [ marker_length_44/2, -marker_length_44/2, 0],
        [-marker_length_44/2, -marker_length_44/2, 0]
    ], dtype=np.float32)


    robot_controller.connect()
    cv2.waitKey(500)
    robot_controller.set_damping()
    cv2.waitKey(500)
    robot_controller.set_stand_mode()
    cv2.waitKey(500)
    input("请放下robot enter pprush")

    robot_controller.set_manip_mode(0)
    time.sleep(3)
    robot_controller.set_manip_mode(1)
    time.sleep(3)

    send_frame_init = 0
    get_frame_init = 0
    
    time.sleep(3)
    head_pose = robot_controller.get_manip_ee_pose()
    if head_pose is None:
        print("❌ 无法获取头部姿态")
        exit(1)


    print(head_pose)
    # 从头部姿态中提取四元数
    head_quat = head_pose.get('head_quat', [0.0, 0.0, 0.0, 1.0])
    print(f"初始head_quat: {head_quat}")
    
    # 将初始头部四元数转换为欧拉角
    head_roll, head_pitch, head_yaw = quaternion_to_euler(head_quat)
    print(f"初始头部角度 (单位: 度):")
    print(f"  roll: {np.degrees(head_roll):.2f}°")
    print(f"  pitch: {np.degrees(head_pitch):.2f}°")
    print(f"  yaw: {np.degrees(head_yaw):.2f}°")
    while True:
        begin_open_statu = False
        frame = camera_reveiver.get_frame()

        if frame is None:
            time.sleep(0.01)
            continue
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners66, ids66, rejectedImgPoints66 = detector_6x6.detectMarkers(gray)
        corners44, ids44, rejectedImgPoints44 = detector_4x4.detectMarkers(gray)

        ## 6*6 aruco识别
        if ids66 is not None:
            aruco.drawDetectedMarkers(frame, corners66, ids66)
            for i in range(len(ids66)):
                # 只处理id为1的aruco码
                if ids66[i] == 0:
                    current_corners = corners66[i].reshape((4, 2))
                    success, rvec, tvec = cv2.solvePnP(
                        marker_points_66, 
                        current_corners, 
                        camera_matrix, 
                        dist_coeffs
                    )
                    if not success:
                        continue
                    
                    get_frame_init = get_frame_init+1
                    # 每隔 15 帧 进行一次运算
                    if get_frame_init >= 15:
                        get_frame_init = 0
                        # 获取机器人头部当前姿态    
                        head_pose = robot_controller.get_manip_ee_pose()
                        if head_pose is None:
                            print("❌ 无法获取头部姿态")
                            continue

                        # 从头部姿态中提取四元数
                        head_quat = head_pose.get('head_quat', [0.0, 0.0, 0.0, 1.0])
                        
                        # 保存最新的左臂末端位置
                        latest_left_hand_pos = head_pose.get('left_hand_pos', [0.0, 0.0, 0.0])
                        
                        # # 打印左右臂末端位置
                        # print(f"\n=== 机器人状态信息 ===")
                        # print(f"左臂末端位置: {head_pose.get('left_hand_pos', [0.0, 0.0, 0.0])}")
                        # print(f"右臂末端位置: {head_pose.get('right_hand_pos', [0.0, 0.0, 0.0])}")
                        
                        # # 将头部四元数转换为角度（roll, pitch, yaw）
                        # if head_quat is not None and len(head_quat) == 4:
                        #     # 使用新添加的quaternion_to_euler函数进行转换
                        #     roll, pitch, yaw = quaternion_to_euler(head_quat)
                        #     print(f"当前头部角度 - roll: {np.degrees(roll):.2f}°, pitch: {np.degrees(pitch):.2f}°, yaw: {np.degrees(yaw):.2f}°")
                        
                        # print("=======================")

                    # 调用函数计算ArUco码的变换矩阵和目标位置
                    marker_pos_base, marker_quat_base, target_left_hand_pos, target_left_hand_quat = calculate_aruco_transforms(
                        rvec, tvec, head_quat, CAMERA_TO_HEAD, quaternion_to_rotation_matrix, rotation_matrix_to_quaternion, roll_offset, offset
                    )
                    
                    if marker_pos_base is None:
                        continue
                    
                    # 保存最新的ArUco码信息
                    latest_marker_pos = marker_pos_base.copy()  # 保存原始ArUco码位置
                    latest_marker_quat = marker_quat_base.copy()
                    latest_head_quat = head_quat.copy()
                    latest_target_left_hand_pos = target_left_hand_pos.copy()  # 保存带有offset的目标位置
                    latest_target_left_hand_quat = target_left_hand_quat.copy()
                    
                    # 打印ArUco码在base_link坐标系中的位置信息
                    # print("=== ArUco码（ID: 1）在base_link坐标系中的位置信息 ===")
                    # print(f"x坐标（前后方向）: {latest_marker_pos[0]:.3f} 米")
                    # print(f"y坐标（左右方向）: {latest_marker_pos[1]:.3f} 米")
                    # print(f"z坐标（上下方向）: {latest_marker_pos[2]:.3f} 米")
                    # print("==================================================")

        ## 4*4 aruco识别 略过
        cv2.imshow("asdasd",frame)

        # 获取键盘输入
        key = cv2.waitKey(1) & 0xFF
        
        # 退出程序
        if key == ord('q'):
            robot_controller.set_manip_mode(2)
            cv2.waitKey(500)
            robot_controller.set_damping()
            break
        
        # 摄像头控制：上下左右按键控制pitch和yaw
        if key == ord('w'):  # 上箭头：pitch减小（抬头）
            camera_pitch += angle_step_rad
            if(camera_pitch> np.radians(-155)):
                camera_pitch = np.radians(-155)
            print(f"摄像头向上移动5°，当前pitch: {np.degrees(camera_pitch):.1f}°")
            # 发送相机角度给机器人
            rpy = np.array([0.0, camera_pitch, camera_yaw])
            R_head = rotation_matrix(rpy)
            head_quat = rotation_matrix_to_quaternion(R_head)
            response = robot_controller.set_manip_ee_pose(head_quat=convert_to_float_list(head_quat))
            print(f"相机角度控制响应: {response}")
        elif key == ord('s'):  # 下箭头：pitch增大（低头）
            camera_pitch -= angle_step_rad
            if(camera_pitch< np.radians(-225)):
                camera_pitch = np.radians(-225)
            print(f"摄像头向下移动5°，当前pitch: {np.degrees(camera_pitch):.1f}°")
            # 发送相机角度给机器人
            rpy = np.array([0.0, camera_pitch, camera_yaw])
            R_head = rotation_matrix(rpy)
            head_quat = rotation_matrix_to_quaternion(R_head)
            response = robot_controller.set_manip_ee_pose(head_quat=convert_to_float_list(head_quat))
            print(f"相机角度控制响应: {response}")
        elif key == ord('a'):  # 左箭头：yaw减小（向左转）
            camera_yaw += angle_step_rad
            if(camera_yaw> np.radians(-150)):
                camera_yaw = np.radians(-150)
            print(f"摄像头向左移动5°，当前yaw: {np.degrees(camera_yaw):.1f}°")
            # 发送相机角度给机器人
            rpy = np.array([0.0, camera_pitch, camera_yaw])
            R_head = rotation_matrix(rpy)
            head_quat = rotation_matrix_to_quaternion(R_head)
            response = robot_controller.set_manip_ee_pose(head_quat=convert_to_float_list(head_quat))
            print(f"相机角度控制响应: {response}")
        elif key == ord('d'):  # 右箭头：yaw增大（向右转）
            camera_yaw -= angle_step_rad
            if(camera_yaw< np.radians(-210)):
                camera_yaw = np.radians(-210)
            print(f"摄像头向右移动5°，当前yaw: {np.degrees(camera_yaw):.1f}°")
            # 发送相机角度给机器人
            rpy = np.array([0.0, camera_pitch, camera_yaw])
            R_head = rotation_matrix(rpy)
            head_quat = rotation_matrix_to_quaternion(R_head)
            response = robot_controller.set_manip_ee_pose(head_quat=convert_to_float_list(head_quat))
            print(f"相机角度控制响应: {response}")
        elif key == ord('x'):  # 右箭头：yaw增大（向右转）
            camera_yaw = np.radians(-180)
            camera_pitch = np.radians(-180)
            print(f"云台复位")
            # 发送相机角度给机器人
            rpy = np.array([0.0, camera_pitch, camera_yaw])
            R_head = rotation_matrix(rpy)
            head_quat = rotation_matrix_to_quaternion(R_head)
            response = robot_controller.set_manip_ee_pose(head_quat=convert_to_float_list(head_quat))
            print(f"相机角度控制响应: {response}")
        
        # 左臂末端跟随模式控制
        elif key == ord('n'):  # 退出左臂末端跟随模式
            left_arm_follow_mode = False
            print("已退出左臂末端跟随模式")
        elif key == ord('m'):  # 进入左臂末端跟随模式
            left_arm_follow_mode = True
            print("已进入左臂末端跟随模式")
        
        # 末端offset调整（只有在跟随模式下可用）
        elif left_arm_follow_mode:
            if key == ord('i'):  # 上抬3cm
                offset[2] += offset_step
                print(f"末端offset上抬3cm，当前offset: {offset}")
            elif key == ord('j'):  # 左移3cm
                offset[1] += offset_step
                print(f"末端offset左移3cm，当前offset: {offset}")
            elif key == ord('k'):  # 下降3cm
                offset[2] -= offset_step
                print(f"末端offset下降3cm，当前offset: {offset}")
            elif key == ord('l'):  # 右移3cm
                offset[1] -= offset_step
                print(f"末端offset右移3cm，当前offset: {offset}")
            elif key == ord('u'):  # roll角度左旋转5°
                roll_offset += roll_step
                print(f"末端roll角度左旋转5°，当前roll_offset: {np.degrees(roll_offset):.1f}°")
            elif key == ord('o'):  # roll角度右旋转5°
                roll_offset -= roll_step
                print(f"末端roll角度右旋转5°，当前roll_offset: {np.degrees(roll_offset):.1f}°")
            elif key == ord('y'):  # pitch角度增加（抬头）
                pitch_offset += pitch_step
                print(f"末端pitch角度增加3°，当前pitch_offset: {np.degrees(pitch_offset):.1f}°")
            elif key == ord('p'):  # pitch角度减小（低头）
                pitch_offset -= pitch_step
                print(f"末端pitch角度减小3°，当前pitch_offset: {np.degrees(pitch_offset):.1f}°")
            elif key == ord('g'):  # yaw角度增加（左转）
                yaw_offset += yaw_step
                print(f"末端yaw角度增加3°，当前yaw_offset: {np.degrees(yaw_offset):.1f}°")
            elif key == ord('h'):  # yaw角度减小（右转）
                yaw_offset -= yaw_step
                print(f"末端yaw角度减小3°，当前yaw_offset: {np.degrees(yaw_offset):.1f}°")
            elif key == ord('r'):  # x轴offset增加（向前移动）
                offset[0] += offset_step
                print(f"末端x轴offset增加2cm，当前offset: [{offset[0]*100:.1f}, {offset[1]*100:.1f}, {offset[2]*100:.1f}] cm")
            elif key == ord('t'):  # x轴offset减小（向后移动）
                offset[0] -= offset_step
                print(f"末端x轴offset减小2cm，当前offset: [{offset[0]*100:.1f}, {offset[1]*100:.1f}, {offset[2]*100:.1f}] cm")
            elif key == ord('z'):  # 重置offset为初始值
                offset = np.array([0.10, 0.08, -0.10]) 
                roll_offset = np.radians(6.0)  # 初始roll偏移为0
                pitch_offset = np.radians(-111)  # 初始pitch偏移为0
                yaw_offset = 0.0    # 初始yaw偏移为0
            elif key == ord('c'):
                aruco_roll_11, aruco_pitch_11, aruco_yaw_11= quaternion_to_euler(latest_marker_quat)
                offset= np.array([0.08, 0.10, -0.15]) 
                roll_offset = np.radians(6.0) 
                yaw_offset = aruco_pitch_11 - np.radians(90) 
                pitch_offset = -1 * aruco_yaw_11
            elif key == ord('v'):
                offset= np.array([0.15, 0.12, -0.35]) 
        

        # 在左臂跟随模式下，实时发送移动命令
        if left_arm_follow_mode and latest_marker_pos is not None:
            # 使用最新检测到的ArUco码信息和当前offset计算目标位置
            target_pos = latest_marker_pos + offset
            
            # 将四元数转换为旋转矩阵
            R_aruco = quaternion_to_rotation_matrix(latest_marker_quat)
            
            # 获取ArUco码的z轴方向（垂直于码面）
            aruco_z_axis = R_aruco[:, 2]
            
            # 计算与ArUco码z轴垂直的方向，作为末端的x轴方向
            # 假设我们希望末端x轴指向ArUco码的前方，z轴指向ArUco码的左侧
            # 这里我们使用一个参考向量，然后计算垂直方向
            ref_vector = np.array([1.0, 0.0, 0.0])  # 参考向量
            
            # 计算垂直于aruco_z_axis的向量作为末端x轴
            # 如果aruco_z_axis接近ref_vector，使用另一个参考向量
            if abs(np.dot(aruco_z_axis, ref_vector)) > 0.9:  # 接近平行
                ref_vector = np.array([0.0, 1.0, 0.0])  # 换一个参考向量
            
            # 计算末端x轴：垂直于aruco_z_axis和ref_vector的叉积
            end_effector_x = np.cross(ref_vector, aruco_z_axis)
            end_effector_x /= np.linalg.norm(end_effector_x)  # 归一化
            
            # 计算末端y轴：垂直于end_effector_x和aruco_z_axis的叉积
            end_effector_y = np.cross(aruco_z_axis, end_effector_x)
            end_effector_y /= np.linalg.norm(end_effector_y)  # 归一化
            
            # 计算末端z轴：与aruco_z_axis相同方向（指向ArUco码前方）
            end_effector_z = aruco_z_axis
            
            # 构建末端的旋转矩阵
            R_end_effector = np.column_stack([end_effector_x, end_effector_y, end_effector_z])
            
            # 应用roll_offset到末端姿态
            if roll_offset != 0.0:
                # 创建roll旋转矩阵
                R_roll = np.array([
                    [np.cos(roll_offset), -np.sin(roll_offset), 0],
                    [np.sin(roll_offset), np.cos(roll_offset), 0],
                    [0, 0, 1]
                ])
                # 应用roll偏移
                R_end_effector = R_end_effector @ R_roll
            
            # 应用pitch_offset到末端姿态
            if pitch_offset != 0.0:
                # 创建pitch旋转矩阵（绕y轴旋转）
                R_pitch = np.array([
                    [np.cos(pitch_offset), 0, np.sin(pitch_offset)],
                    [0, 1, 0],
                    [-np.sin(pitch_offset), 0, np.cos(pitch_offset)]
                ])
                # 应用pitch偏移
                R_end_effector = R_end_effector @ R_pitch
            
            # 应用yaw_offset到末端姿态
            if yaw_offset != 0.0:
                # 创建yaw旋转矩阵（绕z轴旋转）
                R_yaw = np.array([
                    [np.cos(yaw_offset), -np.sin(yaw_offset), 0],
                    [np.sin(yaw_offset), np.cos(yaw_offset), 0],
                    [0, 0, 1]
                ])
                # 应用yaw偏移
                R_end_effector = R_end_effector @ R_yaw
            
            # 将旋转矩阵转换为四元数
            target_quat = rotation_matrix_to_quaternion(R_end_effector)

            send_frame_init = send_frame_init + 1
            if send_frame_init >= 15:
                print(begin_open_statu)
                if begin_open_statu == True:
                    offset[2] = offset[2] - 0.02
                    print(offset[2])
                    if offset[2] < -0.35:
                        begin_open_statu = False
                print(f"\n=== 左臂跟随模式信息 ===")
                print(f"ArUco码位置（base_link）: [{latest_marker_pos[0]:.3f}, {latest_marker_pos[1]:.3f}, {latest_marker_pos[2]:.3f}] 米")
                print(f"当前offset: [{offset[0]*100:.1f}, {offset[1]*100:.1f}, {offset[2]*100:.1f}] cm")
                print(f"目标位置: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] 米")
                print(f"roll偏移: {np.degrees(roll_offset):.1f}°")
                print(f"yaw偏移: {np.degrees(yaw_offset):.1f}°")
                print(f"pitch偏移: {np.degrees(pitch_offset):.1f}°")
                
                # 打印左臂末端当前位置
                if latest_left_hand_pos is not None:
                    print(f"\n=== 左臂末端当前位置 ===")
                    print(f"x坐标（前后方向）: {latest_left_hand_pos[0]:.3f} 米")
                    print(f"y坐标（左右方向）: {latest_left_hand_pos[1]:.3f} 米")
                    print(f"z坐标（上下方向）: {latest_left_hand_pos[2]:.3f} 米")
                
                # 打印ArUco码角度信息
                aruco_roll, aruco_pitch, aruco_yaw = quaternion_to_euler(latest_marker_quat)
                print(f"\n=== ArUco码角度信息 ===")
                print(f"roll: {np.degrees(aruco_roll):.1f}°")
                print(f"pitch: {np.degrees(aruco_pitch):.1f}°")
                print(f"yaw: {np.degrees(aruco_yaw):.1f}°")
                
                # 打印末端角度信息
                end_roll, end_pitch, end_yaw = quaternion_to_euler(target_quat)
                print(f"\n=== 末端角度信息 ===")
                print(f"roll: {np.degrees(end_roll):.1f}°")
                print(f"pitch: {np.degrees(end_pitch):.1f}°")
                print(f"yaw: {np.degrees(end_yaw):.1f}°")
                
                print("========================")
                send_frame_init = 0
                response = robot_controller.set_manip_ee_pose(
                    head_quat=convert_to_float_list(latest_head_quat),  # 保持当前头部姿态
                    left_pos=convert_to_float_list(target_pos),
                    left_quat=convert_to_float_list(target_quat)
                )
                # print(f"机器人响应: {response}")

    camera_reveiver.stop()
    cv2.destroyAllWindows()