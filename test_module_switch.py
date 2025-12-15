#!/usr/bin/env python3
"""
简单的模块切换测试
"""
import sys
import os

# 添加项目路径
sys.path.append(os.path.dirname(__file__))

from utils.robot_controller import RobotController
from api.robot_control_api import RobotClient
from api.camera_api import RobotImageReceiver

def test_robot_controller_methods():
    """测试机器人控制器的新方法"""
    print("🧪 测试机器人控制器方法...")
    
    # 创建模拟的API对象
    robot_api = RobotClient()
    camera_api = RobotImageReceiver()
    
    # 创建机器人控制器
    controller = RobotController(robot_api, camera_api)
    
    # 测试属性是否存在
    print(f"manipulation_mode_active 属性: {hasattr(controller, 'manipulation_mode_active')}")
    print(f"ensure_manipulation_mode 方法: {hasattr(controller, 'ensure_manipulation_mode')}")
    print(f"exit_manipulation_mode 方法: {hasattr(controller, 'exit_manipulation_mode')}")
    
    # 测试初始状态
    print(f"初始操作模式状态: {controller.manipulation_mode_active}")
    
    print("✅ 机器人控制器方法测试完成")

def main():
    """主函数"""
    print("🚀 模块切换修复验证")
    print("=" * 40)
    
    try:
        test_robot_controller_methods()
        print("\n✅ 所有测试通过！")
        print("现在可以运行 main.py 进行实际测试")
        
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()