"""
机器人控制器模块
封装机器人控制逻辑和状态管理
"""
import numpy as np
import time
from .coordinate_transforms import (
    rotation_matrix, 
    rotation_matrix_to_quaternion, 
    quaternion_to_euler,
    quaternion_to_rotation_matrix,
    convert_to_float_list
)


class RobotController:
    def __init__(self, robot_api, camera_api):
        """
        初始化机器人控制器
        
        参数:
            robot_api: 机器人控制API实例
            camera_api: 相机API实例
        """
        self.robot_api = robot_api
        self.camera_api = camera_api
        
        # 相机控制参数
        self.camera_pitch = np.radians(-180)  # 初始pitch角度（弧度）
        self.camera_yaw = np.radians(-180)    # 初始yaw角度（弧度）
        self.angle_step = 5.0    # 每次按键移动的角度（度）
        self.angle_step_rad = np.radians(self.angle_step)  # 转换为弧度
        
        # 左臂末端跟随模式
        self.left_arm_follow_mode = False  # 初始为退出跟随模式
        
        # 末端offset调整（单位：米）
        self.offset = np.array([-0.02, 0.0, -0.10])  # 初始偏移
        self.offset_step = 0.02  # 每次按键调整2cm
        
        # 末端角度offset调整（单位：弧度）
        self.roll_offset = np.radians(6.0)  # 初始roll偏移
        self.pitch_offset = np.radians(-111)  # 初始pitch偏移
        self.yaw_offset = 0.0    # 初始yaw偏移
        self.roll_step = np.radians(3.0)  # 每次按键调整3°
        self.pitch_step = np.radians(3.0)
        self.yaw_step = np.radians(3.0)
        
        # 状态变量
        self.latest_marker_pos = None
        self.latest_marker_quat = None
        self.latest_head_quat = None
        self.latest_left_hand_pos = None
        self.latest_target_left_hand_pos = None
        self.latest_target_left_hand_quat = None
        
        # 帧计数器
        self.send_frame_init = 0
        self.get_frame_init = 0
        
    def initialize_robot(self):
        """初始化机器人状态"""
        print("初始化机器人...")
        self.robot_api.connect()
        time.sleep(0.5)
        self.robot_api.set_damping()
        time.sleep(0.5)
        self.robot_api.set_stand_mode()
        time.sleep(0.5)
        
        input("请放下robot，按Enter继续...")
        
        self.robot_api.set_manip_mode(0)
        time.sleep(3)
        self.robot_api.set_manip_mode(1)
        time.sleep(3)
        
        # 获取初始头部姿态
        head_pose = self.robot_api.get_manip_ee_pose()
        if head_pose is None:
            print("❌ 无法获取头部姿态")
            return False
            
        self.latest_head_quat = head_pose.get('head_quat', [0.0, 0.0, 0.0, 1.0])
        print(f"初始head_quat: {self.latest_head_quat}")
        
        # 将初始头部四元数转换为欧拉角
        head_roll, head_pitch, head_yaw = quaternion_to_euler(self.latest_head_quat)
        print(f"初始头部角度 (单位: 度):")
        print(f"  roll: {np.degrees(head_roll):.2f}°")
        print(f"  pitch: {np.degrees(head_pitch):.2f}°")
        print(f"  yaw: {np.degrees(head_yaw):.2f}°")
        
        return True
    
    def update_camera_angle(self, direction):
        """
        更新相机角度
        
        参数:
            direction: 方向 ('up', 'down', 'left', 'right', 'reset')
        """
        if direction == 'up':  # w键：pitch减小（抬头）
            self.camera_pitch += self.angle_step_rad
            if self.camera_pitch > np.radians(-155):
                self.camera_pitch = np.radians(-155)
            print(f"摄像头向上移动{self.angle_step}°，当前pitch: {np.degrees(self.camera_pitch):.1f}°")
        elif direction == 'down':  # s键：pitch增大（低头）
            self.camera_pitch -= self.angle_step_rad
            if self.camera_pitch < np.radians(-225):
                self.camera_pitch = np.radians(-225)
            print(f"摄像头向下移动{self.angle_step}°，当前pitch: {np.degrees(self.camera_pitch):.1f}°")
        elif direction == 'left':  # a键：yaw减小（向左转）
            self.camera_yaw += self.angle_step_rad
            if self.camera_yaw > np.radians(-150):
                self.camera_yaw = np.radians(-150)
            print(f"摄像头向左移动{self.angle_step}°，当前yaw: {np.degrees(self.camera_yaw):.1f}°")
        elif direction == 'right':  # d键：yaw增大（向右转）
            self.camera_yaw -= self.angle_step_rad
            if self.camera_yaw < np.radians(-210):
                self.camera_yaw = np.radians(-210)
            print(f"摄像头向右移动{self.angle_step}°，当前yaw: {np.degrees(self.camera_yaw):.1f}°")
        elif direction == 'reset':  # x键：复位
            self.camera_yaw = np.radians(-180)
            self.camera_pitch = np.radians(-180)
            print("云台复位")
        
        # 发送相机角度给机器人
        rpy = np.array([0.0, self.camera_pitch, self.camera_yaw])
        R_head = rotation_matrix(rpy)
        head_quat = rotation_matrix_to_quaternion(R_head)
        response = self.robot_api.set_manip_ee_pose(head_quat=convert_to_float_list(head_quat))
        print(f"相机角度控制响应: {response}")
    
    def set_follow_mode(self, enabled):
        """设置跟随模式"""
        self.left_arm_follow_mode = enabled
        if enabled:
            print("已进入左臂末端跟随模式")
        else:
            print("已退出左臂末端跟随模式")
    
    def adjust_offset(self, adjustment_type, direction):
        """
        调整末端偏移
        
        参数:
            adjustment_type: 调整类型 ('position', 'roll', 'pitch', 'yaw')
            direction: 方向或轴 ('up', 'down', 'left', 'right', 'forward', 'backward', 'positive', 'negative')
        """
        if not self.left_arm_follow_mode:
            return
            
        if adjustment_type == 'position':
            if direction == 'up':  # i键
                self.offset[2] += self.offset_step
                print(f"末端offset上抬2cm，当前offset: [{self.offset[0]*100:.1f}, {self.offset[1]*100:.1f}, {self.offset[2]*100:.1f}] cm")
            elif direction == 'down':  # k键
                self.offset[2] -= self.offset_step
                print(f"末端offset下降2cm，当前offset: [{self.offset[0]*100:.1f}, {self.offset[1]*100:.1f}, {self.offset[2]*100:.1f}] cm")
            elif direction == 'left':  # j键
                self.offset[1] += self.offset_step
                print(f"末端offset左移2cm，当前offset: [{self.offset[0]*100:.1f}, {self.offset[1]*100:.1f}, {self.offset[2]*100:.1f}] cm")
            elif direction == 'right':  # l键
                self.offset[1] -= self.offset_step
                print(f"末端offset右移2cm，当前offset: [{self.offset[0]*100:.1f}, {self.offset[1]*100:.1f}, {self.offset[2]*100:.1f}] cm")
            elif direction == 'forward':  # r键
                self.offset[0] += self.offset_step
                print(f"末端x轴offset增加2cm，当前offset: [{self.offset[0]*100:.1f}, {self.offset[1]*100:.1f}, {self.offset[2]*100:.1f}] cm")
            elif direction == 'backward':  # t键
                self.offset[0] -= self.offset_step
                print(f"末端x轴offset减小2cm，当前offset: [{self.offset[0]*100:.1f}, {self.offset[1]*100:.1f}, {self.offset[2]*100:.1f}] cm")
        elif adjustment_type == 'roll':
            if direction == 'positive':  # u键
                self.roll_offset += self.roll_step
                print(f"末端roll角度左旋转3°，当前roll_offset: {np.degrees(self.roll_offset):.1f}°")
            elif direction == 'negative':  # o键
                self.roll_offset -= self.roll_step
                print(f"末端roll角度右旋转3°，当前roll_offset: {np.degrees(self.roll_offset):.1f}°")
        elif adjustment_type == 'pitch':
            if direction == 'positive':  # y键
                self.pitch_offset += self.pitch_step
                print(f"末端pitch角度增加3°，当前pitch_offset: {np.degrees(self.pitch_offset):.1f}°")
            elif direction == 'negative':  # p键
                self.pitch_offset -= self.pitch_step
                print(f"末端pitch角度减小3°，当前pitch_offset: {np.degrees(self.pitch_offset):.1f}°")
        elif adjustment_type == 'yaw':
            if direction == 'positive':  # g键
                self.yaw_offset += self.yaw_step
                print(f"末端yaw角度增加3°，当前yaw_offset: {np.degrees(self.yaw_offset):.1f}°")
            elif direction == 'negative':  # h键
                self.yaw_offset -= self.yaw_step
                print(f"末端yaw角度减小3°，当前yaw_offset: {np.degrees(self.yaw_offset):.1f}°")
    
    def reset_offsets(self, preset_type='default'):
        """重置偏移量"""
        if preset_type == 'default':  # z键
            self.offset = np.array([0.10, 0.08, -0.10]) 
            self.roll_offset = np.radians(6.0)
            self.pitch_offset = np.radians(-111)
            self.yaw_offset = 0.0
            print("重置offset为默认值")
        elif preset_type == 'aruco_aligned' and self.latest_marker_quat is not None:  # c键
            aruco_roll, aruco_pitch, aruco_yaw = quaternion_to_euler(self.latest_marker_quat)
            self.offset = np.array([0.08, 0.10, -0.15]) 
            self.roll_offset = np.radians(6.0) 
            self.yaw_offset = aruco_pitch - np.radians(90) 
            self.pitch_offset = -1 * aruco_yaw
            print("设置为ArUco对齐模式")
        elif preset_type == 'extended':  # v键
            self.offset = np.array([0.15, 0.12, -0.35])
            print("设置为扩展模式")
    
    def update_robot_pose(self):
        """更新机器人姿态信息"""
        self.get_frame_init += 1
        
        # 每隔15帧进行一次运算
        if self.get_frame_init >= 15:
            self.get_frame_init = 0
            
            # 获取机器人头部当前姿态    
            head_pose = self.robot_api.get_manip_ee_pose()
            if head_pose is None:
                print("❌ 无法获取头部姿态")
                return False
            
            # 从头部姿态中提取四元数
            self.latest_head_quat = head_pose.get('head_quat', [0.0, 0.0, 0.0, 1.0])
            
            # 保存最新的左臂末端位置
            self.latest_left_hand_pos = head_pose.get('left_hand_pos', [0.0, 0.0, 0.0])
            
            return True
        
        return False
    
    def update_marker_info(self, marker_pos_base, marker_quat_base, target_left_hand_pos, target_left_hand_quat):
        """更新标记信息"""
        self.latest_marker_pos = marker_pos_base.copy()
        self.latest_marker_quat = marker_quat_base.copy()
        self.latest_target_left_hand_pos = target_left_hand_pos.copy()
        self.latest_target_left_hand_quat = target_left_hand_quat.copy()
    
    def execute_follow_mode(self):
        """执行跟随模式"""
        if not self.left_arm_follow_mode or self.latest_marker_pos is None:
            return
        
        # 使用最新检测到的ArUco码信息和当前offset计算目标位置
        target_pos = self.latest_marker_pos + self.offset
        
        # 将四元数转换为旋转矩阵
        R_aruco = quaternion_to_rotation_matrix(self.latest_marker_quat)
        
        # 获取ArUco码的z轴方向（垂直于码面）
        aruco_z_axis = R_aruco[:, 2]
        
        # 计算与ArUco码z轴垂直的方向，作为末端的x轴方向
        ref_vector = np.array([1.0, 0.0, 0.0])  # 参考向量
        
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
        
        # 应用各种角度偏移
        if self.roll_offset != 0.0:
            R_roll = np.array([
                [np.cos(self.roll_offset), -np.sin(self.roll_offset), 0],
                [np.sin(self.roll_offset), np.cos(self.roll_offset), 0],
                [0, 0, 1]
            ])
            R_end_effector = R_end_effector @ R_roll
        
        if self.pitch_offset != 0.0:
            R_pitch = np.array([
                [np.cos(self.pitch_offset), 0, np.sin(self.pitch_offset)],
                [0, 1, 0],
                [-np.sin(self.pitch_offset), 0, np.cos(self.pitch_offset)]
            ])
            R_end_effector = R_end_effector @ R_pitch
        
        if self.yaw_offset != 0.0:
            R_yaw = np.array([
                [np.cos(self.yaw_offset), -np.sin(self.yaw_offset), 0],
                [np.sin(self.yaw_offset), np.cos(self.yaw_offset), 0],
                [0, 0, 1]
            ])
            R_end_effector = R_end_effector @ R_yaw
        
        # 将旋转矩阵转换为四元数
        target_quat = rotation_matrix_to_quaternion(R_end_effector)
        
        self.send_frame_init += 1
        if self.send_frame_init >= 15:
            self.send_frame_init = 0
            
            # 发送控制命令
            response = self.robot_api.set_manip_ee_pose(
                head_quat=convert_to_float_list(self.latest_head_quat),
                left_pos=convert_to_float_list(target_pos),
                left_quat=convert_to_float_list(target_quat)
            )
            
            return True
        
        return False
    
    def print_status_info(self):
        """打印状态信息"""
        if not self.left_arm_follow_mode or self.latest_marker_pos is None:
            return
            
        target_pos = self.latest_marker_pos + self.offset
        
        print(f"\n=== 左臂跟随模式信息 ===")
        print(f"ArUco码位置（base_link）: [{self.latest_marker_pos[0]:.3f}, {self.latest_marker_pos[1]:.3f}, {self.latest_marker_pos[2]:.3f}] 米")
        print(f"当前offset: [{self.offset[0]*100:.1f}, {self.offset[1]*100:.1f}, {self.offset[2]*100:.1f}] cm")
        print(f"目标位置: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] 米")
        print(f"roll偏移: {np.degrees(self.roll_offset):.1f}°")
        print(f"yaw偏移: {np.degrees(self.yaw_offset):.1f}°")
        print(f"pitch偏移: {np.degrees(self.pitch_offset):.1f}°")
        
        # 打印左臂末端当前位置
        if self.latest_left_hand_pos is not None:
            print(f"\n=== 左臂末端当前位置 ===")
            print(f"x坐标（前后方向）: {self.latest_left_hand_pos[0]:.3f} 米")
            print(f"y坐标（左右方向）: {self.latest_left_hand_pos[1]:.3f} 米")
            print(f"z坐标（上下方向）: {self.latest_left_hand_pos[2]:.3f} 米")
        
        # 打印ArUco码角度信息
        if self.latest_marker_quat is not None:
            aruco_roll, aruco_pitch, aruco_yaw = quaternion_to_euler(self.latest_marker_quat)
            print(f"\n=== ArUco码角度信息 ===")
            print(f"roll: {np.degrees(aruco_roll):.1f}°")
            print(f"pitch: {np.degrees(aruco_pitch):.1f}°")
            print(f"yaw: {np.degrees(aruco_yaw):.1f}°")
        
        print("========================")
    
    def shutdown(self):
        """关闭机器人"""
        self.robot_api.set_manip_mode(2)
        time.sleep(0.5)
        self.robot_api.set_damping()