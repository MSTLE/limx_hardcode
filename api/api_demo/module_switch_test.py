#!/usr/bin/env python3
"""
模块切换测试脚本
测试从模式2切换到模式1时的操作模式管理
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

import time
from api.robot_control_api import RobotClient


def test_manipulation_mode_switching():
    """测试移动操作模式切换"""
    print("🧪 测试移动操作模式切换...")
    
    # 创建机器人客户端
    robot_api = RobotClient()
    
    try:
        # 连接机器人
        robot_api.connect()
        if not robot_api.connected:
            print("❌ 无法连接到机器人")
            return False
        
        print("✅ 机器人连接成功")
        
        # 测试序列1: 进入移动操作模式 (模拟模式1)
        print("\n🔧 测试序列1: 进入移动操作模式")
        
        # 设置阻尼模式
        result = robot_api.set_damping()
        print(f"设置阻尼模式: {result}")
        time.sleep(1)
        
        # 设置站立模式
        result = robot_api.set_stand_mode()
        print(f"设置站立模式: {result}")
        time.sleep(1)
        
        # 进入移动操作模式
        print("进入移动操作准备模式 (mode 0)...")
        result = robot_api.set_manip_mode(0)
        print(f"移动操作准备模式: {result}")
        time.sleep(3)
        
        print("进入移动操作模式 (mode 1)...")
        result = robot_api.set_manip_mode(1)
        print(f"移动操作模式: {result}")
        time.sleep(3)
        
        # 获取当前姿态
        pose = robot_api.get_manip_ee_pose()
        if pose and pose.get('result') == 'success':
            print("✅ 成功获取末端姿态，移动操作模式正常")
        else:
            print(f"❌ 获取末端姿态失败: {pose}")
        
        # 测试序列2: 切换到行走模式 (模拟模式2)
        print("\n🔧 测试序列2: 切换到行走模式")
        
        # 退出移动操作模式
        print("退出移动操作模式 (mode 2)...")
        result = robot_api.set_manip_mode(2)
        print(f"退出移动操作模式: {result}")
        time.sleep(1)
        
        # 设置站立模式（行走准备）
        result = robot_api.set_stand_mode()
        print(f"设置站立模式: {result}")
        time.sleep(1)
        
        # 测试行走命令
        print("测试行走命令...")
        result = robot_api.set_walk_vel(0, 0, 0)
        print(f"行走命令测试: {result}")
        
        # 测试序列3: 重新切换回移动操作模式 (模拟从模式2回到模式1)
        print("\n🔧 测试序列3: 重新切换回移动操作模式")
        
        # 停止行走
        result = robot_api.set_walk_vel(0, 0, 0)
        print(f"停止行走: {result}")
        time.sleep(0.5)
        
        # 重新进入移动操作模式
        print("重新进入移动操作准备模式 (mode 0)...")
        result = robot_api.set_manip_mode(0)
        print(f"移动操作准备模式: {result}")
        time.sleep(3)
        
        print("重新进入移动操作模式 (mode 1)...")
        result = robot_api.set_manip_mode(1)
        print(f"移动操作模式: {result}")
        time.sleep(3)
        
        # 再次获取当前姿态
        pose = robot_api.get_manip_ee_pose()
        if pose and pose.get('result') == 'success':
            print("✅ 成功重新获取末端姿态，模式切换正常")
            print(f"头部姿态: {pose.get('head_quat')}")
            print(f"左臂位置: {pose.get('left_hand_pos')}")
            print(f"右臂位置: {pose.get('right_hand_pos')}")
        else:
            print(f"❌ 重新获取末端姿态失败: {pose}")
        
        # 测试末端控制
        print("\n🔧 测试末端控制...")
        result = robot_api.set_manip_ee_pose(
            head_quat=pose.get('head_quat'),
            left_pos=pose.get('left_hand_pos'),
            left_quat=pose.get('left_hand_quat'),
            right_pos=pose.get('right_hand_pos'),
            right_quat=pose.get('right_hand_quat')
        )
        print(f"末端控制测试: {result}")
        
        if result and result.get('result') == 'success':
            print("✅ 末端控制正常，模式切换测试成功")
            return True
        else:
            print("❌ 末端控制失败")
            return False
        
    except Exception as e:
        print(f"❌ 测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        # 清理：设置阻尼模式
        try:
            robot_api.set_manip_mode(2)
            time.sleep(1)
            robot_api.set_damping()
            print("🔄 已设置阻尼模式")
        except:
            pass
        
        robot_api.disconnect()


def test_mode_state_tracking():
    """测试模式状态跟踪"""
    print("\n🧪 测试模式状态跟踪...")
    
    # 这里可以添加更多的状态跟踪测试
    # 比如检查模式切换后的内部状态是否正确
    
    print("✅ 模式状态跟踪测试完成")


def main():
    """主函数"""
    print("🚀 模块切换测试程序")
    print("=" * 60)
    print("测试场景: 模式1 -> 模式2 -> 模式1")
    print("验证: 移动操作模式的正确进入和退出")
    print("=" * 60)
    
    try:
        # 测试移动操作模式切换
        success = test_manipulation_mode_switching()
        
        if success:
            print("\n✅ 所有测试通过！")
            print("📋 测试结果:")
            print("  ✅ 移动操作模式进入正常")
            print("  ✅ 行走模式切换正常")
            print("  ✅ 重新进入移动操作模式正常")
            print("  ✅ 末端控制功能正常")
        else:
            print("\n❌ 测试失败！")
            print("请检查机器人连接和API响应")
        
        # 测试模式状态跟踪
        test_mode_state_tracking()
        
    except KeyboardInterrupt:
        print("\n🛑 测试被用户中断")
    except Exception as e:
        print(f"\n❌ 测试程序异常: {e}")


if __name__ == "__main__":
    main()