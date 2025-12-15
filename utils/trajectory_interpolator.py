"""
轨迹插值器模块
实现机械臂运动的平滑插帧控制
"""
import numpy as np
import time
from collections import deque
from .coordinate_transforms import (
    quaternion_to_rotation_matrix, 
    rotation_matrix_to_quaternion,
    convert_to_float_list
)


class TrajectoryInterpolator:
    """轨迹插值器 - 实现平滑的机械臂运动"""
    
    def __init__(self, interpolation_steps=10, max_velocity=0.1, max_angular_velocity=0.5):
        """
        初始化轨迹插值器
        
        参数:
            interpolation_steps: 插值步数
            max_velocity: 最大线性速度 (m/s)
            max_angular_velocity: 最大角速度 (rad/s)
        """
        self.interpolation_steps = interpolation_steps
        self.max_velocity = max_velocity
        self.max_angular_velocity = max_angular_velocity
        
        # 当前状态
        self.current_position = None
        self.current_quaternion = None
        self.target_position = None
        self.target_quaternion = None
        
        # 插值队列
        self.position_queue = deque()
        self.quaternion_queue = deque()
        
        # 时间控制
        self.last_update_time = time.time()
        self.interpolation_dt = 1.0 / 30.0  # 30Hz更新频率
        
        # 运动状态
        self.is_moving = False
        self.motion_start_time = 0
        
        # 平滑参数
        self.position_smoothing = 0.8  # 位置平滑系数
        self.orientation_smoothing = 0.8  # 姿态平滑系数
        
    def set_target(self, target_position, target_quaternion):
        """
        设置目标位置和姿态
        
        参数:
            target_position: 目标位置 [x, y, z]
            target_quaternion: 目标四元数 [x, y, z, w]
        """
        target_position = np.array(target_position)
        target_quaternion = np.array(target_quaternion)
        
        # 如果是第一次设置目标，直接设为当前位置
        if self.current_position is None:
            self.current_position = target_position.copy()
            self.current_quaternion = target_quaternion.copy()
            self.target_position = target_position.copy()
            self.target_quaternion = target_quaternion.copy()
            return
        
        # 检查目标是否发生显著变化
        position_change = np.linalg.norm(target_position - self.target_position)
        quaternion_change = self._quaternion_distance(target_quaternion, self.target_quaternion)
        
        # 只有变化足够大时才更新目标
        if position_change > 0.005 or quaternion_change > 0.05:  # 5mm或约3度
            self.target_position = target_position.copy()
            self.target_quaternion = target_quaternion.copy()
            self._generate_trajectory()
    
    def _quaternion_distance(self, q1, q2):
        """计算两个四元数之间的角度距离"""
        dot_product = np.abs(np.dot(q1, q2))
        # 确保点积在有效范围内
        dot_product = np.clip(dot_product, 0.0, 1.0)
        return 2 * np.arccos(dot_product)
    
    def _generate_trajectory(self):
        """生成平滑轨迹"""
        if self.current_position is None or self.target_position is None:
            return
        
        # 清空当前队列
        self.position_queue.clear()
        self.quaternion_queue.clear()
        
        # 计算运动距离和时间
        position_distance = np.linalg.norm(self.target_position - self.current_position)
        quaternion_distance = self._quaternion_distance(self.target_quaternion, self.current_quaternion)
        
        # 根据最大速度计算所需时间
        time_for_position = position_distance / self.max_velocity
        time_for_rotation = quaternion_distance / self.max_angular_velocity
        total_time = max(time_for_position, time_for_rotation, 0.1)  # 最少0.1秒
        
        # 计算插值步数
        steps = max(int(total_time / self.interpolation_dt), self.interpolation_steps)
        
        # 生成位置插值轨迹（使用三次样条插值）
        for i in range(steps + 1):
            t = i / steps
            # 使用平滑的S曲线插值
            smooth_t = self._smooth_step(t)
            
            # 位置插值
            interpolated_position = self.current_position + smooth_t * (self.target_position - self.current_position)
            
            # 四元数球面线性插值 (SLERP)
            interpolated_quaternion = self._slerp(self.current_quaternion, self.target_quaternion, smooth_t)
            
            self.position_queue.append(interpolated_position.copy())
            self.quaternion_queue.append(interpolated_quaternion.copy())
        
        self.is_moving = True
        self.motion_start_time = time.time()
    
    def _smooth_step(self, t):
        """平滑步进函数 - 生成S型曲线"""
        # 使用smoothstep函数: 3t² - 2t³
        return t * t * (3.0 - 2.0 * t)
    
    def _slerp(self, q1, q2, t):
        """球面线性插值 (Spherical Linear Interpolation)"""
        # 确保使用最短路径
        dot = np.dot(q1, q2)
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # 如果四元数非常接近，使用线性插值
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # 计算角度
        theta_0 = np.arccos(np.abs(dot))
        sin_theta_0 = np.sin(theta_0)
        
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        
        return s0 * q1 + s1 * q2
    
    def get_next_waypoint(self):
        """
        获取下一个路径点
        
        返回:
            tuple: (position, quaternion, is_moving) 或 (None, None, False)
        """
        current_time = time.time()
        
        # 检查是否需要更新
        if current_time - self.last_update_time < self.interpolation_dt:
            return None, None, self.is_moving
        
        self.last_update_time = current_time
        
        # 如果队列为空，停止运动
        if not self.position_queue or not self.quaternion_queue:
            self.is_moving = False
            return None, None, False
        
        # 获取下一个路径点
        next_position = self.position_queue.popleft()
        next_quaternion = self.quaternion_queue.popleft()
        
        # 更新当前状态
        self.current_position = next_position.copy()
        self.current_quaternion = next_quaternion.copy()
        
        return next_position, next_quaternion, True
    
    def get_current_state(self):
        """
        获取当前状态
        
        返回:
            tuple: (position, quaternion)
        """
        return self.current_position, self.current_quaternion
    
    def is_motion_complete(self):
        """检查运动是否完成"""
        return not self.is_moving and len(self.position_queue) == 0
    
    def stop_motion(self):
        """停止当前运动"""
        self.position_queue.clear()
        self.quaternion_queue.clear()
        self.is_moving = False
    
    def set_interpolation_parameters(self, steps=None, max_vel=None, max_ang_vel=None):
        """
        设置插值参数
        
        参数:
            steps: 插值步数
            max_vel: 最大线性速度
            max_ang_vel: 最大角速度
        """
        if steps is not None:
            self.interpolation_steps = steps
        if max_vel is not None:
            self.max_velocity = max_vel
        if max_ang_vel is not None:
            self.max_angular_velocity = max_ang_vel
    
    def get_motion_progress(self):
        """
        获取运动进度
        
        返回:
            float: 进度百分比 (0.0 - 1.0)
        """
        if not self.is_moving:
            return 1.0
        
        total_steps = len(self.position_queue) + len(self.quaternion_queue)
        if total_steps == 0:
            return 1.0
        
        remaining_steps = len(self.position_queue)
        return 1.0 - (remaining_steps / (total_steps / 2))


class AdaptiveTrajectoryInterpolator(TrajectoryInterpolator):
    """自适应轨迹插值器 - 根据运动复杂度自动调整参数"""
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # 自适应参数
        self.min_steps = 5
        self.max_steps = 30
        self.velocity_scale_factor = 1.0
        
        # 运动历史记录
        self.motion_history = deque(maxlen=10)
        
    def _generate_trajectory(self):
        """生成自适应轨迹"""
        if self.current_position is None or self.target_position is None:
            return
        
        # 分析运动复杂度
        position_distance = np.linalg.norm(self.target_position - self.current_position)
        quaternion_distance = self._quaternion_distance(self.target_quaternion, self.current_quaternion)
        
        # 记录运动历史
        self.motion_history.append({
            'position_distance': position_distance,
            'quaternion_distance': quaternion_distance,
            'timestamp': time.time()
        })
        
        # 根据运动复杂度调整参数
        complexity_factor = self._calculate_complexity_factor(position_distance, quaternion_distance)
        
        # 自适应调整插值步数
        adaptive_steps = int(self.interpolation_steps * complexity_factor)
        adaptive_steps = np.clip(adaptive_steps, self.min_steps, self.max_steps)
        
        # 自适应调整速度
        adaptive_velocity = self.max_velocity * self.velocity_scale_factor
        adaptive_angular_velocity = self.max_angular_velocity * self.velocity_scale_factor
        
        # 临时保存原始参数
        original_steps = self.interpolation_steps
        original_velocity = self.max_velocity
        original_angular_velocity = self.max_angular_velocity
        
        # 设置自适应参数
        self.interpolation_steps = adaptive_steps
        self.max_velocity = adaptive_velocity
        self.max_angular_velocity = adaptive_angular_velocity
        
        # 生成轨迹
        super()._generate_trajectory()
        
        # 恢复原始参数
        self.interpolation_steps = original_steps
        self.max_velocity = original_velocity
        self.max_angular_velocity = original_angular_velocity
        
        print(f"🎯 自适应轨迹: 步数={adaptive_steps}, 复杂度={complexity_factor:.2f}")
    
    def _calculate_complexity_factor(self, position_distance, quaternion_distance):
        """计算运动复杂度因子"""
        # 基础复杂度
        base_complexity = 1.0
        
        # 距离因子
        if position_distance > 0.1:  # 大于10cm
            base_complexity *= 1.5
        elif position_distance < 0.02:  # 小于2cm
            base_complexity *= 0.7
        
        # 角度因子
        if quaternion_distance > 0.5:  # 大角度旋转
            base_complexity *= 1.3
        elif quaternion_distance < 0.1:  # 小角度旋转
            base_complexity *= 0.8
        
        # 运动频率因子
        if len(self.motion_history) >= 3:
            recent_motions = list(self.motion_history)[-3:]
            time_span = recent_motions[-1]['timestamp'] - recent_motions[0]['timestamp']
            if time_span < 1.0:  # 1秒内多次运动
                base_complexity *= 0.6  # 减少步数，提高响应速度
        
        return np.clip(base_complexity, 0.5, 2.0)