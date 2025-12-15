"""
主程序入口
ArUco标记检测与机器人控制系统
"""
import cv2
import numpy as np
import time
from api import robot_control_api
from api import camera_api
from utils import ArucoProcessor, RobotController
from modules import ArucoFollowModule, ArucoWalkModule


class RobotSystem:
    """机器人系统主类"""
    
    def __init__(self):
        """初始化系统"""
        self.camera_receiver = None
        self.robot_api = None
        self.aruco_processor = None
        self.robot_controller = None
        self.running = False
        
        # 功能模块
        self.modules = {}
        self.current_module = None
        
        # 相机内参 (RealSense标定)
        self.camera_matrix = np.array([
            [907.719360351562, 0.0, 651.812194824219],
            [0.0, 908.330444335938, 387.889526367188],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
        # 系统级键盘映射
        self.system_key_handlers = self._setup_system_key_handlers()
    
    def _setup_system_key_handlers(self):
        """设置系统级键盘处理映射"""
        return {
            # 系统控制
            ord('q'): self._quit_system,
            ord('1'): lambda: self._switch_module("aruco_follow"),
            ord('2'): lambda: self._switch_module("aruco_walk"),
            
            # 相机控制 (所有模块通用)
            ord('w'): lambda: self.robot_controller.update_camera_angle('up'),
            ord('s'): lambda: self.robot_controller.update_camera_angle('down'),
            ord('a'): lambda: self.robot_controller.update_camera_angle('left'),
            ord('d'): lambda: self.robot_controller.update_camera_angle('right'),
            ord('x'): lambda: self.robot_controller.update_camera_angle('reset'),
        }
    
    def initialize(self):
        """初始化系统组件"""
        print("ArUco机器人视觉伺服控制系统")
        print("="*60)
        print("模块切换: 1-ArUco跟随 2-ArUco行走")
        print("相机控制: WASD-移动 X-复位")
        print("="*60)
        
        # 初始化API
        self.camera_receiver = camera_api.RobotImageReceiver()
        self.robot_api = robot_control_api.RobotClient()
        
        # 初始化处理器
        self.aruco_processor = ArucoProcessor(self.camera_matrix, self.dist_coeffs)
        self.robot_controller = RobotController(self.robot_api, self.camera_receiver)
        
        # 启动相机
        if not self._initialize_camera():
            return False
        
        # 初始化机器人
        if not self.robot_controller.initialize_robot():
            return False
        
        # 初始化功能模块
        self._initialize_modules()
        
        # 默认激活ArUco跟随模块
        self._switch_module("aruco_follow")
        
        return True
    
    def _initialize_modules(self):
        """初始化功能模块"""
        self.modules = {
            "aruco_follow": ArucoFollowModule(self.robot_controller, self.aruco_processor),
            "aruco_walk": ArucoWalkModule(self.robot_controller, self.aruco_processor),
            # 可以在这里添加更多模块
        }
    
    def _initialize_camera(self):
        """初始化相机"""
        self.camera_receiver.start()
        print("等待图像...")
        for i in range(50):
            if self.camera_receiver.get_frame() is not None:
                print(f"✅ 图像接收成功!")
                return True
            time.sleep(0.1)
        
        print("❌ 未接收到图像")
        return False
    
    def run(self):
        """运行主循环"""
        self.running = True
        
        try:
            while self.running:
                # 获取图像帧
                frame = self.camera_receiver.get_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue
                
                # 使用当前模块处理
                frame = self._process_current_module(frame)
                
                # 显示图像
                self._display_frame(frame)
                
                # 处理键盘输入
                self._handle_keyboard_input()
        
        except KeyboardInterrupt:
            print("\n程序被用户中断")
        
        finally:
            self._cleanup()
    
    def _process_current_module(self, frame):
        """使用当前模块处理帧"""
        if self.current_module and self.current_module.active:
            return self.current_module.process_frame(frame)
        else:
            # 默认处理：显示无活动模块
            cv2.putText(frame, "No Active Module", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return frame
    
    def _display_frame(self, frame):
        """显示帧并添加状态信息"""
        # 添加当前模块信息
        module_name = self.current_module.name if self.current_module else "None"
        cv2.putText(frame, f"Module: {module_name}", (10, frame.shape[0] - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 添加系统控制提示
        cv2.putText(frame, "1-Follow 2-Walk Q-Quit WASD-Camera X-Reset", (10, frame.shape[0] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow("Robot Control System", frame)
    
    def _handle_keyboard_input(self):
        """处理键盘输入"""
        key = cv2.waitKey(1) & 0xFF
        
        # 优先处理系统级按键
        if key in self.system_key_handlers:
            self.system_key_handlers[key]()
        # 然后处理当前模块的按键
        elif self.current_module and self.current_module.active:
            self.current_module.handle_key(key)
    
    def _switch_module(self, module_name):
        """切换功能模块"""
        if module_name in self.modules:
            print(f"🔄 正在切换到模块: {module_name}")
            
            # 停用当前模块
            if self.current_module:
                print(f"🔄 停用当前模块: {self.current_module.name}")
                self.current_module.deactivate()
                
                # 等待一小段时间确保资源完全释放
                import time
                time.sleep(0.1)
            
            # 激活新模块
            self.current_module = self.modules[module_name]
            print(f"🔄 激活新模块: {self.current_module.name}")
            self.current_module.activate()
            
            # 重置机器人控制器的状态
            self._reset_robot_controller_state()
            
            print(f"✅ 成功切换到模块: {module_name}")
        else:
            print(f"❌ 未找到模块: {module_name}")
    
    def _reset_robot_controller_state(self):
        """重置机器人控制器状态，避免模块间冲突"""
        if hasattr(self, 'robot_controller'):
            # 重置帧计数器
            self.robot_controller.send_frame_init = 0
            self.robot_controller.get_frame_init = 0
            
            # 重置最后命令时间
            if hasattr(self.robot_controller, 'last_command_time'):
                self.robot_controller.last_command_time = 0
            
            # 停止轨迹插值运动
            if hasattr(self.robot_controller, 'trajectory_interpolator'):
                self.robot_controller.trajectory_interpolator.stop_motion()
            
            print("🔄 已重置机器人控制器状态")
    

    
    def _quit_system(self):
        """退出系统"""
        self.running = False
    
    def _cleanup(self):
        """清理资源"""
        # 停用当前模块
        if self.current_module:
            self.current_module.deactivate()
        
        # 清理所有模块
        for module in self.modules.values():
            module.cleanup()
        
        # 清理系统资源
        if self.robot_controller:
            self.robot_controller.shutdown()
        if self.camera_receiver:
            self.camera_receiver.stop()
        cv2.destroyAllWindows()
        print("程序结束")


def main():
    """主函数"""
    system = RobotSystem()
    
    if system.initialize():
        system.run()
    else:
        print("系统初始化失败")


if __name__ == "__main__":
    main()