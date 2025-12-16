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
from .trajectory_interpolator import AdaptiveTrajectoryInterpolator


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
        
        # 初始位置保存
        self.initial_left_hand_pos = None
        self.initial_left_hand_quat = None
        self.initial_right_hand_pos = None
        self.initial_right_hand_quat = None
        self.initial_head_quat = None
        
        # 帧计数器
        self.send_frame_init = 0
        self.get_frame_init = 0
        
        # 轨迹插值器
        self.trajectory_interpolator = AdaptiveTrajectoryInterpolator(
            interpolation_steps=15,
            max_velocity=0.35,  # 8cm/s
            max_angular_velocity=0.6  # 约17度/s
        )
        
        # 平滑控制参数
        self.smooth_control_enabled = True
        self.last_command_time = 0
        self.min_command_interval = 1.0 / 30.0  # 30Hz最大发送频率
        
        # 操作模式状态跟踪
        self.manipulation_mode_active = False
        
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
        
        # 标记移动操作模式已激活
        self.manipulation_mode_active = True
        
        # 获取初始姿态（头部和左臂）
        initial_pose = self.robot_api.get_manip_ee_pose()
        if initial_pose is None:
            print("❌ 无法获取初始姿态")
            return False
        
        # 保存初始位置和姿态
        self.initial_head_quat = initial_pose.get('head_quat', [0.0, 0.0, 0.0, 1.0])
        self.initial_left_hand_pos = initial_pose.get('left_hand_pos', [0.0, 0.0, 0.0])
        self.initial_left_hand_quat = initial_pose.get('left_hand_quat', [0.0, 0.0, 0.0, 1.0])
        self.initial_right_hand_pos = initial_pose.get('right_hand_pos', [0.0, 0.0, 0.0])
        self.initial_right_hand_quat = initial_pose.get('right_hand_quat', [0.0, 0.0, 0.0, 1.0])
        
        # 设置当前状态
        self.latest_head_quat = self.initial_head_quat.copy()
        
        print(f"✅ 保存初始姿态:")
        print(f"  头部四元数: {self.initial_head_quat}")
        print(f"  左臂位置: {self.initial_left_hand_pos}")
        print(f"  左臂四元数: {self.initial_left_hand_quat}")
        print(f"  右臂位置: {self.initial_right_hand_pos}")
        print(f"  右臂四元数: {self.initial_right_hand_quat}")
        
        # 将初始头部四元数转换为欧拉角
        head_roll, head_pitch, head_yaw = quaternion_to_euler(self.initial_head_quat)
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
        
        # 发送相机角度给机器人，确保右臂保持初始位置
        rpy = np.array([0.0, self.camera_pitch, self.camera_yaw])
        R_head = rotation_matrix(rpy)
        head_quat = rotation_matrix_to_quaternion(R_head)
        
        # 构建完整的姿态控制命令，保持右臂初始位置
        pose_params = {"head_quat": convert_to_float_list(head_quat)}
        
        # 如果有右臂初始位置，则保持它
        if self.initial_right_hand_pos is not None and self.initial_right_hand_quat is not None:
            pose_params["right_pos"] = convert_to_float_list(self.initial_right_hand_pos)
            pose_params["right_quat"] = convert_to_float_list(self.initial_right_hand_quat)
        
        response = self.robot_api.set_manip_ee_pose(**pose_params)
        print(f"相机角度控制响应: {response}")
    
    def set_follow_mode(self, enabled):
        """设置跟随模式"""
        self.left_arm_follow_mode = enabled
        if enabled:
            print("已进入左臂末端跟随模式")
            # 重置帧计数器，确保状态同步
            self.send_frame_init = 0
            self.get_frame_init = 0
        else:
            print("已退出左臂末端跟随模式")
            # 停止轨迹插值运动
            if hasattr(self, 'trajectory_interpolator'):
                self.trajectory_interpolator.stop_motion()
    
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
            self.offset = np.array([0.00, 0.02, -0.10]) 
            self.roll_offset = np.radians(6.0)
            self.pitch_offset = np.radians(-111)
            self.yaw_offset = 0.0
            print("重置offset为默认值")
        elif preset_type == 'aruco_aligned' and self.latest_marker_quat is not None:  # c键
            aruco_roll, aruco_pitch, aruco_yaw = quaternion_to_euler(self.latest_marker_quat)
            self.offset = np.array([0.02, 0.08, -0.15]) 
            self.roll_offset = np.radians(6.0) 
            self.yaw_offset = aruco_pitch - np.radians(90) 
            self.pitch_offset = -1 * aruco_yaw
            print("设置为ArUco对齐模式")
        elif preset_type == 'extended':  # v键
            self.offset = np.array([0.10, 0.12, -0.28])
            print("设置为扩展模式")
        elif preset_type == 'opened':  # e键
            self.offset = np.array([0.12, -0.15, -0.30])
            print("设置为扩展模式")
        elif preset_type == 'open_ac':  # e键
            self.offset = np.array([0.45, -0.15, 0.00])
            print("设置为扩展模式")
    
    def toggle_smooth_control(self):
        """切换平滑控制模式"""
        self.smooth_control_enabled = not self.smooth_control_enabled
        if self.smooth_control_enabled:
            print("✅ 启用平滑插帧控制")
        else:
            print("❌ 禁用平滑插帧控制")
            # 停止当前插值运动
            self.trajectory_interpolator.stop_motion()
    
    def adjust_smooth_parameters(self, param_type, direction):
        """
        调整平滑控制参数
        
        参数:
            param_type: 参数类型 ('speed', 'steps', 'angular_speed')
            direction: 方向 ('increase', 'decrease')
        """
        if param_type == 'speed':
            current_speed = self.trajectory_interpolator.max_velocity
            if direction == 'increase':
                new_speed = min(0.15, current_speed + 0.01)  # 最大15cm/s
            else:
                new_speed = max(0.02, current_speed - 0.01)  # 最小2cm/s
            
            self.trajectory_interpolator.set_interpolation_parameters(max_vel=new_speed)
            print(f"🎛️ 线性速度: {new_speed*100:.1f} cm/s")
            
        elif param_type == 'angular_speed':
            current_angular_speed = self.trajectory_interpolator.max_angular_velocity
            if direction == 'increase':
                new_angular_speed = min(1.0, current_angular_speed + 0.05)  # 最大约57度/s
            else:
                new_angular_speed = max(0.1, current_angular_speed - 0.05)  # 最小约6度/s
            
            self.trajectory_interpolator.set_interpolation_parameters(max_ang_vel=new_angular_speed)
            print(f"🎛️ 角速度: {np.degrees(new_angular_speed):.1f} deg/s")
            
        elif param_type == 'steps':
            current_steps = self.trajectory_interpolator.interpolation_steps
            if direction == 'increase':
                new_steps = min(30, current_steps + 2)
            else:
                new_steps = max(5, current_steps - 2)
            
            self.trajectory_interpolator.set_interpolation_parameters(steps=new_steps)
            print(f"🎛️ 插值步数: {new_steps}")
    
    def get_smooth_control_status(self):
        """获取平滑控制状态信息"""
        if not self.smooth_control_enabled:
            return "OFF"
        
        progress = self.trajectory_interpolator.get_motion_progress()
        is_moving = not self.trajectory_interpolator.is_motion_complete()
        
        status = "ON"
        if is_moving:
            status += f" (运动中 {progress*100:.0f}%)"
        else:
            status += " (静止)"
        
        return status
    
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
        """执行跟随模式（带平滑插帧）"""
        if not self.left_arm_follow_mode or self.latest_marker_pos is None:
            return
        
        # 使用最新检测到的ArUco码信息和当前offset计算目标位置
        target_pos = self.latest_marker_pos + self.offset
        
        # 计算目标姿态
        target_quat = self._calculate_target_orientation()
        
        # 更新轨迹插值器的目标
        if self.smooth_control_enabled:
            self.trajectory_interpolator.set_target(target_pos, target_quat)
            
            # 获取插值后的路径点
            interpolated_pos, interpolated_quat, is_moving = self.trajectory_interpolator.get_next_waypoint()
            
            if interpolated_pos is not None and interpolated_quat is not None:
                # 使用插值后的位置和姿态
                return self._send_smooth_command(interpolated_pos, interpolated_quat)
            else:
                # 如果没有新的路径点，检查是否需要发送当前状态
                current_pos, current_quat = self.trajectory_interpolator.get_current_state()
                if current_pos is not None:
                    return self._send_smooth_command(current_pos, current_quat)
        else:
            # 传统控制模式
            return self._send_traditional_command(target_pos, target_quat)
        
        return False
    
    def _calculate_target_orientation(self):
        """计算目标姿态"""
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
        return rotation_matrix_to_quaternion(R_end_effector)
    
    def _send_smooth_command(self, target_pos, target_quat):
        """发送平滑控制命令"""
        current_time = time.time()
        
        # 频率控制
        if current_time - self.last_command_time < self.min_command_interval:
            return False
        
        self.last_command_time = current_time
        
        try:
            # 发送控制命令，确保右臂保持初始位置
            response = self.robot_api.set_manip_ee_pose(
                head_quat=convert_to_float_list(self.latest_head_quat),
                left_pos=convert_to_float_list(target_pos),
                left_quat=convert_to_float_list(target_quat),
                right_pos=convert_to_float_list(self.initial_right_hand_pos) if self.initial_right_hand_pos is not None else None,
                right_quat=convert_to_float_list(self.initial_right_hand_quat) if self.initial_right_hand_quat is not None else None
            )
            
            # 检查响应
            if response and response.get('result') == 'success':
                return True
            else:
                print(f"⚠️ 平滑控制命令响应异常: {response}")
                return False
                
        except Exception as e:
            print(f"❌ 发送平滑控制命令失败: {e}")
            return False
    
    def _send_traditional_command(self, target_pos, target_quat):
        """发送传统控制命令"""
        self.send_frame_init += 1
        if self.send_frame_init >= 15:
            self.send_frame_init = 0
            
            # 发送控制命令，确保右臂保持初始位置
            response = self.robot_api.set_manip_ee_pose(
                head_quat=convert_to_float_list(self.latest_head_quat),
                left_pos=convert_to_float_list(target_pos),
                left_quat=convert_to_float_list(target_quat),
                right_pos=convert_to_float_list(self.initial_right_hand_pos) if self.initial_right_hand_pos is not None else None,
                right_quat=convert_to_float_list(self.initial_right_hand_quat) if self.initial_right_hand_quat is not None else None
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
    
    def return_to_initial_position(self):
        """恢复到初始位置"""
        if (self.initial_left_hand_pos is None or self.initial_left_hand_quat is None or 
            self.initial_right_hand_pos is None or self.initial_right_hand_quat is None):
            print("❌ 未保存完整的初始位置，无法恢复")
            return False
        
        print("🔄 恢复到初始位置...")
        
        try:
            # 先禁用跟随模式
            was_following = self.left_arm_follow_mode
            if was_following:
                self.set_follow_mode(False)
                print("  暂时禁用跟随模式")
            
            # 恢复到初始位置和姿态（包括左臂和右臂）
            response = self.robot_api.set_manip_ee_pose(
                head_quat=convert_to_float_list(self.initial_head_quat),
                left_pos=convert_to_float_list(self.initial_left_hand_pos),
                left_quat=convert_to_float_list(self.initial_left_hand_quat),
                right_pos=convert_to_float_list(self.initial_right_hand_pos),
                right_quat=convert_to_float_list(self.initial_right_hand_quat)
            )
            
            if response and response.get('result') == 'success':
                print("✅ 成功恢复到初始位置")
                print(f"  左臂位置: {self.initial_left_hand_pos}")
                print(f"  左臂姿态: {self.initial_left_hand_quat}")
                print(f"  右臂位置: {self.initial_right_hand_pos}")
                print(f"  右臂姿态: {self.initial_right_hand_quat}")
                
                # 如果之前是跟随模式，询问是否重新启用
                if was_following:
                    print("  跟随模式已暂停，按M键重新启用")
                
                return True
            else:
                print(f"❌ 恢复初始位置失败: {response}")
                return False
                
        except Exception as e:
            print(f"❌ 恢复初始位置时发生错误: {e}")
            return False
    
    def ensure_manipulation_mode(self):
        """确保机器人处于移动操作模式"""
        if not self.manipulation_mode_active:
            print("🔧 重新进入移动操作模式...")
            try:
                # 进入准备模式
                result0 = self.robot_api.set_manip_mode(0)
                time.sleep(3)
                
                # 进入操作模式
                result1 = self.robot_api.set_manip_mode(1)
                time.sleep(3)
                
                if result1 and result1.get('result') == 'success':
                    self.manipulation_mode_active = True
                    print("✅ 成功重新进入移动操作模式")
                    return True
                else:
                    print(f"❌ 重新进入移动操作模式失败: {result1}")
                    return False
                    
            except Exception as e:
                print(f"❌ 重新进入移动操作模式时出错: {e}")
                return False
        return True
    
    def exit_manipulation_mode(self):
        """退出移动操作模式"""
        if self.manipulation_mode_active:
            try:
                result = self.robot_api.set_manip_mode(2)
                time.sleep(1)
                
                if result and result.get('result') == 'success':
                    self.manipulation_mode_active = False
                    print("✅ 成功退出移动操作模式")
                else:
                    print(f"⚠️ 退出移动操作模式响应: {result}")
                    
            except Exception as e:
                print(f"❌ 退出移动操作模式时出错: {e}")
    
    def shutdown(self):
        """关闭机器人"""
        self.exit_manipulation_mode()
        self.robot_api.set_damping()