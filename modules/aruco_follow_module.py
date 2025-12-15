"""
ArUco跟随模块
实现基于ArUco标记的机械臂跟随功能
"""
import cv2
import numpy as np
from utils.base_module import BaseModule


class ArucoFollowModule(BaseModule):
    """ArUco跟随功能模块"""
    
    def __init__(self, robot_controller, aruco_processor):
        super().__init__("ArUco Follow", robot_controller, aruco_processor)
        self.target_marker_id = 0
        self.key_bindings = self.get_key_bindings()
    
    def process_frame(self, frame):
        """处理图像帧"""
        if not self.active:
            return frame
        
        # 检测ArUco标记
        detection_results = self.aruco_processor.detect_markers(frame)
        
        # 绘制检测到的标记
        frame = self.aruco_processor.draw_markers(frame, detection_results)
        
        # 处理6x6标记
        success, rvec, tvec = self.aruco_processor.process_marker(frame, self.target_marker_id, '6x6')
        
        if success:
            # 更新机器人姿态信息
            if self.robot_controller.update_robot_pose():
                pass  # 每15帧更新一次
            
            # 计算ArUco变换
            marker_pos_base, marker_quat_base, target_left_hand_pos, target_left_hand_quat = \
                self.aruco_processor.calculate_aruco_transforms(
                    rvec, tvec, self.robot_controller.latest_head_quat, 
                    self.robot_controller.roll_offset, self.robot_controller.offset
                )
            
            if marker_pos_base is not None:
                # 更新标记信息
                self.robot_controller.update_marker_info(
                    marker_pos_base, marker_quat_base, 
                    target_left_hand_pos, target_left_hand_quat
                )
        
        # 执行跟随模式
        if self.robot_controller.execute_follow_mode():
            self.robot_controller.print_status_info()
        
        # 绘制状态信息
        self.draw_status(frame)
        
        return frame
    
    def get_key_bindings(self):
        """获取键盘绑定"""
        return {
            # 基础控制
            ord('n'): lambda: self.robot_controller.set_follow_mode(False),
            ord('m'): lambda: self.robot_controller.set_follow_mode(True),
            ord('b'): lambda: self.robot_controller.return_to_initial_position(),
            
            # 位置调整
            ord('i'): lambda: self.robot_controller.adjust_offset('position', 'up'),
            ord('k'): lambda: self.robot_controller.adjust_offset('position', 'down'),
            ord('j'): lambda: self.robot_controller.adjust_offset('position', 'left'),
            ord('l'): lambda: self.robot_controller.adjust_offset('position', 'right'),
            ord('r'): lambda: self.robot_controller.adjust_offset('position', 'forward'),
            ord('t'): lambda: self.robot_controller.adjust_offset('position', 'backward'),
            
            # 角度调整
            ord('u'): lambda: self.robot_controller.adjust_offset('roll', 'positive'),
            ord('o'): lambda: self.robot_controller.adjust_offset('roll', 'negative'),
            ord('y'): lambda: self.robot_controller.adjust_offset('pitch', 'positive'),
            ord('p'): lambda: self.robot_controller.adjust_offset('pitch', 'negative'),
            ord('g'): lambda: self.robot_controller.adjust_offset('yaw', 'positive'),
            ord('h'): lambda: self.robot_controller.adjust_offset('yaw', 'negative'),
            
            # 预设配置
            ord('z'): lambda: self.robot_controller.reset_offsets('default'),
            ord('c'): lambda: self.robot_controller.reset_offsets('aruco_aligned'),
            ord('v'): lambda: self.robot_controller.reset_offsets('extended'),
            
            # 平滑控制
            ord('f'): lambda: self.robot_controller.toggle_smooth_control(),
            ord('='): lambda: self.robot_controller.adjust_smooth_parameters('speed', 'increase'),
            ord('-'): lambda: self.robot_controller.adjust_smooth_parameters('speed', 'decrease'),
            ord(']'): lambda: self.robot_controller.adjust_smooth_parameters('angular_speed', 'increase'),
            ord('['): lambda: self.robot_controller.adjust_smooth_parameters('angular_speed', 'decrease'),
            ord('.'): lambda: self.robot_controller.adjust_smooth_parameters('steps', 'increase'),
            ord(','): lambda: self.robot_controller.adjust_smooth_parameters('steps', 'decrease'),
        }
    
    def draw_status(self, frame):
        """绘制状态信息"""
        super().draw_status(frame)
        
        # 显示跟随模式状态
        follow_status = "ON" if self.robot_controller.left_arm_follow_mode else "OFF"
        cv2.putText(frame, f"Follow: {follow_status}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 显示平滑控制状态
        smooth_status = self.robot_controller.get_smooth_control_status()
        smooth_color = (0, 255, 255) if self.robot_controller.smooth_control_enabled else (128, 128, 128)
        cv2.putText(frame, f"Smooth: {smooth_status}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, smooth_color, 1)
        
        # 显示平滑控制参数
        if self.robot_controller.smooth_control_enabled:
            speed = self.robot_controller.trajectory_interpolator.max_velocity * 100
            ang_speed = np.degrees(self.robot_controller.trajectory_interpolator.max_angular_velocity)
            steps = self.robot_controller.trajectory_interpolator.interpolation_steps
            cv2.putText(frame, f"Speed: {speed:.1f}cm/s Ang: {ang_speed:.0f}deg/s Steps: {steps}", 
                       (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # 显示控制提示
        cv2.putText(frame, "M-Enable N-Disable B-InitPos F-Smooth IJKLRT-Position UOYPGH-Rotation", 
                   (10, frame.shape[0] - 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(frame, "Z-Default C-Aligned V-Extended +/-Speed [/]AngSpeed ,/.Steps", 
                   (10, frame.shape[0] - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    def deactivate(self):
        """停用模块时关闭跟随模式"""
        if hasattr(self.robot_controller, 'set_follow_mode'):
            self.robot_controller.set_follow_mode(False)
        super().deactivate()