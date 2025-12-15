"""
功能模块基类
为不同的机器人任务提供统一的接口
"""
from abc import ABC, abstractmethod
import cv2


class BaseModule(ABC):
    """功能模块基类"""
    
    def __init__(self, name, robot_controller, aruco_processor=None):
        """
        初始化模块
        
        参数:
            name: 模块名称
            robot_controller: 机器人控制器
            aruco_processor: ArUco处理器（可选）
        """
        self.name = name
        self.robot_controller = robot_controller
        self.aruco_processor = aruco_processor
        self.active = False
        self.key_bindings = {}
    
    @abstractmethod
    def process_frame(self, frame):
        """
        处理图像帧
        
        参数:
            frame: 输入图像
            
        返回:
            处理后的图像
        """
        pass
    
    @abstractmethod
    def get_key_bindings(self):
        """
        获取键盘绑定
        
        返回:
            dict: 键盘映射字典 {key_code: function}
        """
        pass
    
    def activate(self):
        """激活模块"""
        self.active = True
        print(f"✅ 模块 {self.name} 已激活")
    
    def deactivate(self):
        """停用模块"""
        self.active = False
        print(f"❌ 模块 {self.name} 已停用")
    
    def handle_key(self, key):
        """
        处理按键
        
        参数:
            key: 按键码
        """
        if key in self.key_bindings:
            self.key_bindings[key]()
    
    def draw_status(self, frame):
        """
        在图像上绘制状态信息
        
        参数:
            frame: 图像帧
        """
        cv2.putText(frame, f"Module: {self.name}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    def cleanup(self):
        """清理资源"""
        pass