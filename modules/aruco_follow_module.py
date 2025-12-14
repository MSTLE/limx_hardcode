"""
ArUco跟随模块
实现基于ArUco标记的机械臂跟随功能
"""
import cv2
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
            ord('n'): lambda: self.robot_controller.set_follow_mode(False),
            ord('m'): lambda: self.robot_controller.set_follow_mode(True),
            ord('b'): lambda: self.robot_controller.return_to_initial_position(),
            ord('i'): lambda: self.robot_controller.adjust_offset('position', 'up'),
            ord('k'): lambda: self.robot_controller.adjust_offset('position', 'down'),
            ord('j'): lambda: self.robot_controller.adjust_offset('position', 'left'),
            ord('l'): lambda: self.robot_controller.adjust_offset('position', 'right'),
            ord('r'): lambda: self.robot_controller.adjust_offset('position', 'forward'),
            ord('t'): lambda: self.robot_controller.adjust_offset('position', 'backward'),
            ord('u'): lambda: self.robot_controller.adjust_offset('roll', 'positive'),
            ord('o'): lambda: self.robot_controller.adjust_offset('roll', 'negative'),
            ord('y'): lambda: self.robot_controller.adjust_offset('pitch', 'positive'),
            ord('p'): lambda: self.robot_controller.adjust_offset('pitch', 'negative'),
            ord('g'): lambda: self.robot_controller.adjust_offset('yaw', 'positive'),
            ord('h'): lambda: self.robot_controller.adjust_offset('yaw', 'negative'),
            ord('z'): lambda: self.robot_controller.reset_offsets('default'),
            ord('c'): lambda: self.robot_controller.reset_offsets('aruco_aligned'),
            ord('v'): lambda: self.robot_controller.reset_offsets('extended'),
        }
    
    def draw_status(self, frame):
        """绘制状态信息"""
        super().draw_status(frame)
        
        # 显示跟随模式状态
        follow_status = "ON" if self.robot_controller.left_arm_follow_mode else "OFF"
        cv2.putText(frame, f"Follow: {follow_status}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 显示控制提示
        cv2.putText(frame, "M-Enable N-Disable B-InitPos IJKLRT-Position UOYPGH-Rotation", (10, frame.shape[0] - 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(frame, "Z-Default C-Aligned V-Extended", (10, frame.shape[0] - 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def deactivate(self):
        """停用模块时关闭跟随模式"""
        if hasattr(self.robot_controller, 'set_follow_mode'):
            self.robot_controller.set_follow_mode(False)
        super().deactivate()