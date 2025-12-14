      
import time
from robot_control_api import RobotClient

# 配置
ROBOT_IP = "10.192.1.2"  # 或 127.0.0.1

def print_menu():
    """打印测试菜单"""
    print("\n" + "="*50)
    print("           机器人接口测试菜单")
    print("="*50)
    print("基础控制:")
    print("  1. 阻尼模式 (set_damping)")
    print("  2. 站立模式 (set_stand_mode)")
    print()
    print("操作模式:")
    print("  3. 进入操作模式 (set_manip_mode)")
    print("  4. 操作模式 (set_manip_mode)")
    print("  5. 退出操作模式 (set_manip_mode)")
    print()
    print("末端控制:")
    print("  6. 获取末端位姿 (get_manip_ee_pose)")
    print("  7. 设置末端位姿 (set_manip_ee_pose)")
    print()
    print("运动控制:")
    print("  8. 控制行走 (set_walk_vel)")
    print()
    print("状态获取:")
    print("  9. 获取关节状态 (get_joint_state)")
    print("  10. 获取IMU数据 (get_imu_data)")
    print()
    print("  0. 退出程序")
    print("="*50)

def test_damping(robot):
    """测试阻尼模式"""
    print("\n--- 测试：阻尼模式 ---")
    result = robot.set_damping()
    print(f"结果: {result}")
    if result and result.get("result") == "success":
        print("✅ 机器人已进入阻尼模式，可以手动摆动")
    else:
        print("❌ 进入阻尼模式失败")

def test_stand_mode(robot):
    """测试站立模式"""
    print("\n--- 测试：站立模式 ---")
    result = robot.set_stand_mode()
    print(f"结果: {result}")
    if result and result.get("result") == "success":
        print("✅ 机器人已进入站立模式")
    else:
        print("❌ 进入站立模式失败")

def test_manip_mode_enter(robot):
    """测试进入操作模式"""
    print("\n--- 测试：进入操作模式 ---")
    result = robot.set_manip_mode(0)  # 准备进入模式
    print(f"结果: {result}")
    if result and result.get("result") == "success":
        print("✅ 准备进入操作模式成功")
    else:
        print("❌ 准备进入操作模式失败")

def test_manip_mode_active(robot):
    """测试激活操作模式"""
    print("\n--- 测试：激活操作模式 ---")
    result = robot.set_manip_mode(1)  # 操作模式
    print(f"结果: {result}")
    if result and result.get("result") == "success":
        print("✅ 操作模式已激活，开始跟踪末端位置")
    else:
        print("❌ 激活操作模式失败")

def test_manip_mode_exit(robot):
    """测试退出操作模式"""
    print("\n--- 测试：退出操作模式 ---")
    result = robot.set_manip_mode(2)  # 准备退出模式
    print(f"结果: {result}")
    if result and result.get("result") == "success":
        print("✅ 准备退出操作模式成功")
    else:
        print("❌ 准备退出操作模式失败")

def test_get_ee_pose(robot):
    """测试获取末端位姿"""
    print("\n--- 测试：获取末端位姿 ---")
    result = robot.get_manip_ee_pose()
    print(f"结果: {result}")
    if result and result.get("result") == "success":
        print("✅ 获取末端位姿成功:")
        print(f"  头部位置: {result.get('head_pos')}")
        print(f"  头部姿态: {result.get('head_quat')}")
        print(f"  左手位置: {result.get('left_hand_pos')}")
        print(f"  左手姿态: {result.get('left_hand_quat')}")
        print(f"  右手位置: {result.get('right_hand_pos')}")
        print(f"  右手姿态: {result.get('right_hand_quat')}")
    else:
        print("❌ 获取末端位姿失败")

def test_set_ee_pose(robot):
    """测试设置末端位姿"""
    print("\n--- 测试：设置末端位姿 ---")
    print("使用当前位姿，只微调左手位置...")
    
    # 只改变左手位置，其他保持当前值
    result = robot.set_manip_ee_pose(
        left_pos=[0.25, 0.25, 0.20]  # 示例位置
    )
    print(f"结果: {result}")
    if result and result.get("result") == "success":
        print("✅ 设置末端位姿成功")
    else:
        print("❌ 设置末端位姿失败")


def test_walk_vel(robot):
    """测试控制行走"""
    print("\n--- 测试：控制行走 ---")
    print("测试前进...")
    
    # 前进
    result = robot.set_walk_vel(x=0.3)
    if result is None:
        print("✅ 前进指令发送成功")
    else:
        print(f"❌ 前进指令失败: {result}")
    
    time.sleep(2)
    
    # 停止
    print("停止...")
    result = robot.set_walk_vel(x=0.0)
    if result is None:
        print("✅ 停止指令发送成功")
    else:
        print(f"❌ 停止指令失败: {result}")



def test_joint_state(robot):
    """测试获取关节状态"""
    print("\n--- 测试：获取关节状态 ---")
    result = robot.get_joint_state()
    print(f"结果: {result}")
    if result and result.get("result") == "success":
        names = result.get("names", [])
        positions = result.get("q", [])
        velocities = result.get("dq", [])
        torques = result.get("tau", [])
        
        print("✅ 获取关节状态成功:")
        print(f"  关节数量: {len(names)}")
        if names:
            print("  前3个关节状态:")
            for i in range(min(3, len(names))):
                print(f"    {names[i]}: 位置={positions[i]:.3f}, 速度={velocities[i]:.3f}, 扭矩={torques[i]:.3f}")
    else:
        print("❌ 获取关节状态失败")

def test_imu_data(robot):
    """测试获取IMU数据"""
    print("\n--- 测试：获取IMU数据 ---")
    result = robot.get_imu_data()
    print(f"结果: {result}")
    if result and result.get("result") == "success":
        euler = result.get("euler", [])
        acc = result.get("acc", [])
        gyro = result.get("gyro", [])
        quat = result.get("quat", [])
        
        print("✅ 获取IMU数据成功:")
        print(f"  欧拉角 (度): Roll={euler[0]:.2f}, Pitch={euler[1]:.2f}, Yaw={euler[2]:.2f}")
        print(f"  加速度 (m/s²): X={acc[0]:.3f}, Y={acc[1]:.3f}, Z={acc[2]:.3f}")
        print(f"  角速度 (rad/s): X={gyro[0]:.3f}, Y={gyro[1]:.3f}, Z={gyro[2]:.3f}")
        print(f"  四元数: W={quat[0]:.3f}, X={quat[1]:.3f}, Y={quat[2]:.3f}, Z={quat[3]:.3f}")
    else:
        print("❌ 获取IMU数据失败")

def test_connection():
    """测试网络连接"""
    import socket
    try:
        # 测试TCP连接
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        result = sock.connect_ex((ROBOT_IP, 5000))
        sock.close()
        
        if result == 0:
            print("✅ 网络连接正常")
            return True
        else:
            print("❌ 无法连接到机器人端口 5000")
            return False
    except Exception as e:
        print(f"❌ 网络测试失败: {e}")
        return False

def main():
    """主函数"""
    print("机器人接口测试程序")
    print(f"连接地址: {ROBOT_IP}")
    
    # 先测试网络连接
    print("测试网络连接...")
    if not test_connection():
        print("请检查:")
        print("1. 机器人IP地址是否正确")
        print("2. 机器人是否开机并运行WebSocket服务")
        print("3. 网络连接是否正常")
        return
    
    # 初始化机器人客户端
    robot = RobotClient(ROBOT_IP)
    robot.connect()
    
    if not robot.connected:
        print("❌ WebSocket连接失败")
        print("可能的原因:")
        print("1. 机器人WebSocket服务未启动")
        print("2. 端口5000被占用或防火墙阻止")
        print("3. WebSocket协议版本不兼容")
        return
    
    print(f"✅ 机器人连接成功! ACCID: {robot.accid}")
    
    # 测试函数映射
    test_functions = {
        1: test_damping,
        2: test_stand_mode,
        3: test_manip_mode_enter,
        4: test_manip_mode_active,
        5: test_manip_mode_exit,
        6: test_get_ee_pose,
        7: test_set_ee_pose,
        8: test_walk_vel,
        9: test_joint_state,
        10: test_imu_data,
    }
    
    try:
        while True:
            print_menu()
            try:
                choice = int(input("请选择测试项目 (0-10): "))
                
                if choice == 0:
                    print("退出程序...")
                    break
                elif choice in test_functions:
                    test_functions[choice](robot)
                    input("\n按回车键继续...")
                else:
                    print("❌ 无效选择，请输入 0-10")
                    
            except ValueError:
                print("❌ 请输入有效数字")
            except KeyboardInterrupt:
                print("\n用户中断程序")
                break
                
    finally:
        robot.disconnect()
        print("连接已断开，程序结束")

if __name__ == "__main__":
    main()

    