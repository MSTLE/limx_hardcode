"""
ArUco行走控制模块
基于ArUco标记（编号2）控制机器人行走对正
"""
import cv2
import numpy as np
from utils.base_module import BaseModule
from api.camera_api import WalkCameraReceiver


class ArucoWalkModule(BaseModule):
    """ArUco行走控制功能模块"""
    
    def __init__(self, robot_controller, aruco_processor):
        super().__init__("ArUco Walk", robot_controller, aruco_processor)
        self.target_marker_id = 2  # 目标ArUco码编号
        self.key_bindings = self.get_key_bindings()
        
        # 行走控制参数
        self.walk_enabled = False
        self.walk_speed = 0.3  # 降低基础行走速度，便于精确控制
        self.turn_speed = 0.3  # 降低基础转向速度，便于精确对正
        
        # 控制参数 - 三个独立的对正条件
        self.target_distance = 0.7  # 目标距离（米）
        self.distance_tolerance = 0.15  # 条件3：距离容差（米）
        self.origin_tolerance = 50.0  # 条件1：原点与中线重合容差（像素）
        self.z_axis_angle_tolerance = 10.0  # 条件2：Z轴与水平方向角度容差（度）
        
        # 5555端口相机
        self.walk_camera = WalkCameraReceiver()
        self.walk_camera_initialized = False
        
        # PID控制参数 - 进一步降低增益，让运动更平稳
        self.distance_kp = 0.5  # 进一步降低距离控制增益，减少运动幅度
        self.angle_kp = 0.4     # 进一步降低角度控制增益，让旋转更加平稳
        
        # 最小控制阈值（避免控制值太小机器人不响应）
        self.min_walk_vel = 0.15   # 降低最小行走速度，便于精细调整
        self.min_turn_vel = 0.18   # 降低最小转向速度，便于精细对正
        
        # 状态变量
        self.last_marker_detected = False
        self.marker_center_x = 0
        self.marker_distance = 0
        self.frame_width = 640  # 假设的图像宽度
        self.robot_mode_checked = False  # 机器人模式检查标志
        
        # 控制频率限制
        self.control_frame_count = 0
        self.control_interval = 5  # 每5帧发送一次控制命令（提高频率）
    
    def activate(self):
        """激活模块时启动5555端口相机"""
        super().activate()
        if not self.walk_camera_initialized:
            self.walk_camera.start()
            self.walk_camera_initialized = True
            print("✅ 5555端口相机已启动")
        
        # 确保机器人处于正确的行走模式
        self._ensure_walk_mode()
    
    def deactivate(self):
        """停用模块时停止相机和行走"""
        if self.walk_enabled:
            self.set_walk_mode(False)
        if self.walk_camera_initialized:
            self.walk_camera.stop()
            self.walk_camera_initialized = False
            print("✅ 5555端口相机已停止")
        super().deactivate()
    
    def process_frame(self, frame):
        """处理图像帧"""
        if not self.active:
            return frame
        
        # 获取5555端口的图像
        walk_frame = self.walk_camera.get_frame()
        if walk_frame is not None:
            # 使用5555端口的图像进行ArUco检测
            frame = self._process_walk_frame(walk_frame)
        else:
            # 如果没有5555端口图像，在原图像上显示状态
            cv2.putText(frame, "Waiting for 5555 camera...", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 绘制状态信息
        self.draw_status(frame)
        
        return frame
    
    def _process_walk_frame(self, frame):
        """处理行走控制帧"""
        self.frame_width = frame.shape[1]
        frame_center_x = self.frame_width // 2
        
        # 检测ArUco标记
        detection_results = self.aruco_processor.detect_markers(frame)
        
        # 绘制检测到的标记
        frame = self.aruco_processor.draw_markers(frame, detection_results)
        
        # 处理目标标记（编号2）
        success, rvec, tvec = self.aruco_processor.process_marker(frame, self.target_marker_id, '6x6')
        
        if success:
            self.last_marker_detected = True
            
            # 计算距离（使用tvec的z分量）
            self.marker_distance = tvec[2][0]
            
            # 绘制ArUco坐标系并获取Z轴端点
            z_axis_end = self._draw_aruco_axes(frame, rvec, tvec)
            
            # 绘制目标线
            cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame.shape[0]), (0, 255, 0), 2)
            
            # 实现三个独立的对正条件检测
            origin_error = 0
            z_axis_angle_error = 0
            
            if z_axis_end is not None:
                # 计算ArUco原点
                aruco_origin = self._get_aruco_origin(rvec, tvec)
                if aruco_origin is not None:
                    # 绘制Z轴投影线
                    cv2.line(frame, tuple(aruco_origin.astype(int)), tuple(z_axis_end.astype(int)), (255, 255, 0), 2)
                    
                    # 条件1：绿色中线与二维码原点重合
                    origin_error = aruco_origin[0] - frame_center_x  # 原点与中线的偏差
                    
                    # 条件2：绿色中线与ArUco码的蓝色Z轴的角度在一定范围内
                    # 计算Z轴的方向向量
                    z_axis_vector = z_axis_end - aruco_origin
                    
                    # 计算Z轴与垂直方向（绿色中线方向）的角度偏差
                    # 绿色中线是垂直的，所以理想的Z轴向量应该是水平的
                    # 计算Z轴向量与水平方向的角度偏差
                    z_axis_angle_rad = np.arctan2(abs(z_axis_vector[1]), abs(z_axis_vector[0]))
                    z_axis_angle_deg = np.degrees(z_axis_angle_rad)
                    
                    # 理想情况下，Z轴应该与水平方向平行（角度接近0度或90度）
                    # 我们希望Z轴尽可能水平，所以计算与水平方向的偏差
                    if z_axis_angle_deg > 45:
                        z_axis_angle_error = 90 - z_axis_angle_deg  # 如果角度大于45度，计算与90度的差值
                    else:
                        z_axis_angle_error = z_axis_angle_deg  # 如果角度小于45度，直接使用角度值
                    
                    z_axis_angle_error = abs(z_axis_angle_error)  # 取绝对值
                    
                    # 获取X轴端点用于显示
                    x_axis_end = self._get_x_axis_end(rvec, tvec)
                    if x_axis_end is not None:
                        # 绘制X轴端点
                        cv2.circle(frame, tuple(x_axis_end.astype(int)), 3, (0, 0, 255), -1)
                        cv2.putText(frame, 'X-end', (int(x_axis_end[0]) + 5, int(x_axis_end[1])), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                    
                    # 绘制Z轴端点和原点
                    cv2.circle(frame, tuple(z_axis_end.astype(int)), 5, (255, 255, 0), -1)
                    cv2.circle(frame, tuple(aruco_origin.astype(int)), 3, (255, 255, 255), -1)
                    
                    # 显示详细的对正信息
                    cv2.putText(frame, f"Origin Error: {origin_error:.1f}px", (10, 90), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(frame, f"Z-Axis Angle: {z_axis_angle_error:.1f}deg", (10, 110), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            distance_error = self.marker_distance - self.target_distance
            
            # 显示距离信息
            cv2.putText(frame, f"Distance: {self.marker_distance:.2f}m", (10, 130), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 执行行走控制 - 传递三个独立的误差值
            if self.walk_enabled:
                self._execute_walk_control(origin_error, z_axis_angle_error, distance_error)
        else:
            # 如果检测不到标记，停止行走（但只在状态改变时调用一次）
            if self.last_marker_detected and self.walk_enabled:
                try:
                    # 优先使用set_walk_vel方法
                    result = self.robot_controller.robot_api.set_walk_vel(0, 0, 0)
                    print(f"🛑 检测不到ArUco码，停止行走: {result}")
                except Exception as e:
                    print(f"❌ 停止行走命令异常: {e}")
                    # 备用方法
                    try:
                        data = {"x": 0, "y": 0, "yaw": 0}
                        self.robot_controller.robot_api.send_command("request_set_walk_vel", data, wait_for_response=False)
                    except Exception as e2:
                        print(f"❌ 备用停止命令也失败: {e2}")
            self.last_marker_detected = False
        
        return frame
    
    def _draw_aruco_axes(self, frame, rvec, tvec):
        """绘制ArUco坐标系，返回Z轴端点"""
        try:
            # 定义坐标轴长度（米）
            axis_length = 0.05  # 5cm
            
            # 定义坐标轴端点（在ArUco坐标系中）
            axis_points = np.array([
                [0, 0, 0],              # 原点
                [axis_length, 0, 0],    # X轴（红色）
                [0, axis_length, 0],    # Y轴（绿色）
                [0, 0, axis_length]     # Z轴（蓝色）
            ], dtype=np.float32)
            
            # 投影到图像平面
            projected_points, _ = cv2.projectPoints(
                axis_points, 
                rvec, 
                tvec, 
                self.aruco_processor.camera_matrix, 
                self.aruco_processor.dist_coeffs
            )
            
            # 转换为浮点坐标
            projected_points = projected_points.reshape(-1, 2)
            
            # 绘制坐标轴
            origin = projected_points[0].astype(int)
            x_end = projected_points[1].astype(int)
            y_end = projected_points[2].astype(int)
            z_end = projected_points[3].astype(int)
            
            # X轴 - 红色
            cv2.arrowedLine(frame, tuple(origin), tuple(x_end), (0, 0, 255), 3, tipLength=0.3)
            cv2.putText(frame, 'X', (x_end[0] + 5, x_end[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Y轴 - 绿色
            cv2.arrowedLine(frame, tuple(origin), tuple(y_end), (0, 255, 0), 3, tipLength=0.3)
            cv2.putText(frame, 'Y', (y_end[0] + 5, y_end[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Z轴 - 蓝色
            cv2.arrowedLine(frame, tuple(origin), tuple(z_end), (255, 0, 0), 3, tipLength=0.3)
            cv2.putText(frame, 'Z', (z_end[0] + 5, z_end[1]), 

                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # 在原点绘制小圆圈
            cv2.circle(frame, tuple(origin), 3, (255, 255, 255), -1)
            
            # 返回Z轴端点坐标
            return projected_points[3]
            
        except Exception as e:
            print(f"绘制ArUco坐标系失败: {e}")
            return None
    
    def _get_aruco_origin(self, rvec, tvec):
        """获取ArUco原点在图像中的位置"""
        try:
            # 原点坐标
            origin_point = np.array([[0, 0, 0]], dtype=np.float32)
            
            # 投影到图像平面
            projected_origin, _ = cv2.projectPoints(
                origin_point,
                rvec,
                tvec,
                self.aruco_processor.camera_matrix,
                self.aruco_processor.dist_coeffs
            )
            
            return projected_origin.reshape(-1, 2)[0]
            
        except Exception as e:
            print(f"获取ArUco原点失败: {e}")
            return None
    
    def _get_x_axis_end(self, rvec, tvec):
        """获取ArUco X轴端点在图像中的位置"""
        try:
            # X轴端点坐标
            axis_length = 0.05  # 5cm，与_draw_aruco_axes中的长度一致
            x_axis_point = np.array([[axis_length, 0, 0]], dtype=np.float32)
            
            # 投影到图像平面
            projected_x_axis, _ = cv2.projectPoints(
                x_axis_point,
                rvec,
                tvec,
                self.aruco_processor.camera_matrix,
                self.aruco_processor.dist_coeffs
            )
            
            return projected_x_axis.reshape(-1, 2)[0]
            
        except Exception as e:
            print(f"获取ArUco X轴端点失败: {e}")
            return None
    
    def _execute_walk_control(self, origin_error, z_axis_angle_error, distance_error):
        """执行行走控制 - 基于三个独立的对正条件"""
        # 频率控制：每隔几帧才发送一次控制命令
        self.control_frame_count += 1
        if self.control_frame_count < self.control_interval:
            return
        
        self.control_frame_count = 0  # 重置计数器
        
        # 检查三个独立的对正条件
        # 条件1：绿色中线与二维码原点重合
        origin_ok = abs(origin_error) < self.origin_tolerance
        
        # 条件2：绿色中线与ArUco码的蓝色Z轴角度在容差范围内
        z_axis_ok = abs(z_axis_angle_error) < self.z_axis_angle_tolerance
        
        # 条件3：距离误差在容差范围内
        distance_ok = abs(distance_error) < self.distance_tolerance
        
        # 计算控制量
        # 旋转控制：主要基于原点偏差，Z轴角度作为辅助
        combined_angle_error = origin_error + 0.3 * z_axis_angle_error
        raw_yaw_vel = -self.angle_kp * (combined_angle_error / (self.frame_width / 2))
        
        # 距离控制 - 如果距离太远就前进，太近就后退
        raw_x_vel = self.distance_kp * distance_error
        
        # 只有三个条件都满足才认为对正成功
        if origin_ok and z_axis_ok and distance_ok:
            x_vel = 0
            yaw_vel = 0
            print("✅ 已对正目标 - 三个条件都满足！")
        else:
            # 应用最小阈值，确保控制值足够大让机器人响应
            if abs(raw_yaw_vel) > 0.01:  # 如果需要转向
                if abs(raw_yaw_vel) < self.min_turn_vel:
                    yaw_vel = self.min_turn_vel if raw_yaw_vel > 0 else -self.min_turn_vel
                    print(f"🔧 应用最小转向速度: 原始={raw_yaw_vel:.3f} -> 最小={yaw_vel:.3f}")
                else:
                    yaw_vel = raw_yaw_vel
            else:
                yaw_vel = 0
            
            if abs(raw_x_vel) > 0.01:  # 如果需要前后移动
                if abs(raw_x_vel) < self.min_walk_vel:
                    x_vel = self.min_walk_vel if raw_x_vel > 0 else -self.min_walk_vel
                    print(f"🔧 应用最小行走速度: 原始={raw_x_vel:.3f} -> 最小={x_vel:.3f}")
                else:
                    x_vel = raw_x_vel
            else:
                x_vel = 0
        
        # 限制速度范围
        yaw_vel = np.clip(yaw_vel, -self.turn_speed, self.turn_speed)
        x_vel = np.clip(x_vel, -self.walk_speed, self.walk_speed)
        
        # 调试信息 - 显示三个独立条件的状态
        print(f"条件1-原点偏差: {origin_error:.1f}px (容差:{self.origin_tolerance:.1f}) {'✅' if origin_ok else '❌'}")
        print(f"条件2-Z轴角度: {z_axis_angle_error:.1f}deg (容差:{self.z_axis_angle_tolerance:.1f}) {'✅' if z_axis_ok else '❌'}")
        print(f"条件3-距离误差: {distance_error:.2f}m (容差:{self.distance_tolerance:.2f}) {'✅' if distance_ok else '❌'}")
        print(f"综合状态: {'✅ 完全对正' if (origin_ok and z_axis_ok and distance_ok) else '🔄 调整中'}")
        if not (origin_ok and z_axis_ok and distance_ok):
            print(f"原始控制值: yaw={raw_yaw_vel:.3f}, x={raw_x_vel:.3f}")
        print(f"最终控制值: yaw={yaw_vel:.3f}, x={x_vel:.3f}")
        
        # 发送行走命令 - 尝试多种方法
        try:
            print(f"🚶 发送行走命令: x={x_vel:.3f}, y=0, yaw={yaw_vel:.3f}")
            
            # 方法1: 使用set_walk_vel方法（推荐）
            result1 = self.robot_controller.robot_api.set_walk_vel(x_vel, 0, yaw_vel)
            print(f"✅ set_walk_vel结果: {result1}")
            
            # 如果方法1失败，尝试方法2
            if result1 is not None and "fail" in str(result1):
                print("⚠️ set_walk_vel失败，尝试send_command方法")
                data = {"x": x_vel, "y": 0, "yaw": yaw_vel}
                result2 = self.robot_controller.robot_api.send_command("request_set_walk_vel", data, wait_for_response=False)
                print(f"✅ send_command结果: {result2}")
            
        except Exception as e:
            print(f"❌ 发送行走命令异常: {e}")
            # 如果出现异常，尝试备用方法
            try:
                print("🔄 尝试备用发送方法...")
                data = {"x": x_vel, "y": 0, "yaw": yaw_vel}
                backup_result = self.robot_controller.robot_api.send_command("request_set_walk_vel", data, wait_for_response=False)
                print(f"✅ 备用方法结果: {backup_result}")
            except Exception as e2:
                print(f"❌ 备用方法也失败: {e2}")
    
    def get_key_bindings(self):
        """获取键盘绑定"""
        return {
            ord('e'): self.toggle_walk_mode,
            ord('f'): self.increase_walk_speed,
            ord('g'): self.decrease_walk_speed,
            ord('r'): self.increase_target_distance,
            ord('t'): self.decrease_target_distance,
            ord('q'): self.emergency_stop,
            ord('1'): self.test_forward,
            ord('2'): self.test_backward,
            ord('3'): self.test_turn_left,
            ord('4'): self.test_turn_right,
        }
    
    def toggle_walk_mode(self):
        """切换行走模式"""
        self.walk_enabled = not self.walk_enabled
        if self.walk_enabled:
            print("🚶 启用ArUco行走控制")
        else:
            print("🛑 禁用ArUco行走控制")
            # 停止行走
            try:
                result = self.robot_controller.robot_api.set_walk_vel(0, 0, 0)
                print(f"停止行走结果: {result}")
            except Exception as e:
                print(f"❌ 停止行走命令异常: {e}")
    
    def set_walk_mode(self, enabled):
        """设置行走模式"""
        self.walk_enabled = enabled
        if enabled:
            print("🚶 启用ArUco行走控制")
        else:
            print("🛑 禁用ArUco行走控制")
            # 停止行走
            try:
                result = self.robot_controller.robot_api.set_walk_vel(0, 0, 0)
                print(f"停止行走结果: {result}")
            except Exception as e:
                print(f"❌ 停止行走命令异常: {e}")
    
    def increase_walk_speed(self):
        """增加行走速度"""
        self.walk_speed = min(0.8, self.walk_speed + 0.1)
        print(f"行走速度: {self.walk_speed:.1f}")
    
    def decrease_walk_speed(self):
        """减少行走速度"""
        self.walk_speed = max(0.1, self.walk_speed - 0.1)
        print(f"行走速度: {self.walk_speed:.1f}")
    
    def increase_target_distance(self):
        """增加目标距离"""
        self.target_distance = min(3.0, self.target_distance + 0.1)
        print(f"目标距离: {self.target_distance:.1f}m")
    
    def decrease_target_distance(self):
        """减少目标距离"""
        self.target_distance = max(0.5, self.target_distance - 0.1)
        print(f"目标距离: {self.target_distance:.1f}m")
    
    def test_forward(self):
        """测试前进"""
        try:
            print("🧪 测试前进 (0.2m/s, 2秒)")
            result = self.robot_controller.robot_api.set_walk_vel(0.2, 0, 0)
            print(f"前进命令结果: {result}")
        except Exception as e:
            print(f"❌ 测试前进失败: {e}")
    
    def test_backward(self):
        """测试后退"""
        try:
            print("🧪 测试后退 (-0.2m/s, 2秒)")
            result = self.robot_controller.robot_api.set_walk_vel(-0.2, 0, 0)
            print(f"后退命令结果: {result}")
        except Exception as e:
            print(f"❌ 测试后退失败: {e}")
    
    def test_turn_left(self):
        """测试左转"""
        try:
            print("🧪 测试左转 (0.3rad/s, 2秒)")
            result = self.robot_controller.robot_api.set_walk_vel(0, 0, 0.3)
            print(f"左转命令结果: {result}")
        except Exception as e:
            print(f"❌ 测试左转失败: {e}")
    
    def test_turn_right(self):
        """测试右转"""
        try:
            print("🧪 测试右转 (-0.3rad/s, 2秒)")
            result = self.robot_controller.robot_api.set_walk_vel(0, 0, -0.3)
            print(f"右转命令结果: {result}")
        except Exception as e:
            print(f"❌ 测试右转失败: {e}")
    
    def emergency_stop(self):
        """紧急停止"""
        try:
            result = self.robot_controller.robot_api.set_walk_vel(0, 0, 0)
            self.walk_enabled = False
            print(f"🚨 紧急停止！结果: {result}")
        except Exception as e:
            print(f"❌ 紧急停止命令异常: {e}")
            self.walk_enabled = False
    
    def draw_status(self, frame):
        """绘制状态信息"""
        super().draw_status(frame)
        
        # 显示行走模式状态
        walk_status = "ON" if self.walk_enabled else "OFF"
        color = (0, 255, 0) if self.walk_enabled else (0, 0, 255)
        cv2.putText(frame, f"Walk Mode: {walk_status}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # 显示检测状态
        detect_status = "DETECTED" if self.last_marker_detected else "NOT FOUND"
        detect_color = (0, 255, 0) if self.last_marker_detected else (0, 0, 255)
        cv2.putText(frame, f"ArUco-2: {detect_status}", (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, detect_color, 2)
        
        # 显示参数
        cv2.putText(frame, f"Speed: {self.walk_speed:.1f} Target: {self.target_distance:.1f}m", (10, 180), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 显示控制提示
        cv2.putText(frame, "E-Walk F/G-Speed R/T-Distance Q-Stop", (10, frame.shape[0] - 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, "Test: 1-Forward 2-Back 3-Left 4-Right", (10, frame.shape[0] - 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
    def _ensure_walk_mode(self):
        """确保机器人处于正确的行走模式"""
        if self.robot_mode_checked:
            return
            
        try:
            print("🔧 检查机器人行走模式...")
            
            # 确保机器人处于站立模式（可以接受行走命令）
            result = self.robot_controller.robot_api.set_stand_mode()
            if result and result.get('result') == 'success':
                print("✅ 机器人已进入站立模式")
            else:
                print(f"⚠️ 设置站立模式响应: {result}")
            
            # 发送一个测试命令确保通信正常
            test_result = self.robot_controller.robot_api.send_command(
                "request_set_walk_vel", 
                {"x": 0, "y": 0, "yaw": 0}, 
                wait_for_response=True,
                timeout=2.0
            )
            
            if test_result is not None:
                print("✅ 行走控制接口测试成功")
            else:
                print("⚠️ 行走控制接口测试无响应（可能正常）")
            
            self.robot_mode_checked = True
            
        except Exception as e:
            print(f"❌ 设置机器人行走模式时出错: {e}")
    
    def cleanup(self):
        """清理资源"""
        if self.walk_enabled:
            self.emergency_stop()
        if self.walk_camera_initialized:
            self.walk_camera.stop()
        super().cleanup()