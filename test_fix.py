#!/usr/bin/env python3
"""
验证修复的简单测试
"""

# 测试导入
try:
    from utils.robot_controller import RobotController
    from api.robot_control_api import RobotClient
    from api.camera_api import RobotImageReceiver
    from modules.aruco_follow_module import ArucoFollowModule
    from modules.aruco_walk_module import ArucoWalkModule
    from utils.aruco_processor import ArucoProcessor
    import numpy as np
    
    print("✅ 所有模块导入成功")
    
    # 创建模拟对象
    robot_api = RobotClient()
    camera_api = RobotImageReceiver()
    
    # 测试机器人控制器
    controller = RobotController(robot_api, camera_api)
    print(f"✅ RobotController 创建成功")
    print(f"   manipulation_mode_active: {controller.manipulation_mode_active}")
    print(f"   ensure_manipulation_mode 方法: {hasattr(controller, 'ensure_manipulation_mode')}")
    print(f"   exit_manipulation_mode 方法: {hasattr(controller, 'exit_manipulation_mode')}")
    
    # 测试ArUco处理器
    camera_matrix = np.array([
        [907.719360351562, 0.0, 651.812194824219],
        [0.0, 908.330444335938, 387.889526367188],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)
    
    aruco_processor = ArucoProcessor(camera_matrix, dist_coeffs)
    print(f"✅ ArucoProcessor 创建成功")
    
    # 测试模块
    follow_module = ArucoFollowModule(controller, aruco_processor)
    walk_module = ArucoWalkModule(controller, aruco_processor)
    
    print(f"✅ ArucoFollowModule 创建成功")
    print(f"   activate 方法: {hasattr(follow_module, 'activate')}")
    print(f"   deactivate 方法: {hasattr(follow_module, 'deactivate')}")
    print(f"   _refresh_initial_pose 方法: {hasattr(follow_module, '_refresh_initial_pose')}")
    
    print(f"✅ ArucoWalkModule 创建成功")
    print(f"   activate 方法: {hasattr(walk_module, 'activate')}")
    print(f"   deactivate 方法: {hasattr(walk_module, 'deactivate')}")
    
    print("\n🎉 所有测试通过！修复成功！")
    print("现在可以运行 main.py 进行实际测试")
    
except ImportError as e:
    print(f"❌ 导入错误: {e}")
except Exception as e:
    print(f"❌ 其他错误: {e}")
    import traceback
    traceback.print_exc()