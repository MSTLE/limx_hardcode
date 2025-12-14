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


def main():
    """主函数"""
    print("识别aruco 6*6 and 4*4")
    print("="*60)
    
    # 初始化相机和机器人API
    camera_receiver = camera_api.RobotImageReceiver()
    robot_api = robot_control_api.RobotClient()
    
    # 相机内参 (RealSense标定)
    camera_matrix = np.array([
        [907.719360351562, 0.0, 651.812194824219],
        [0.0, 908.330444335938, 387.889526367188],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)
    
    # 初始化处理器
    aruco_processor = ArucoProcessor(camera_matrix, dist_coeffs)
    robot_controller = RobotController(robot_api, camera_receiver)
    
    # 启动相机
    camera_receiver.start()
    print("等待图像...")
    for i in range(50):
        if camera_receiver.get_frame() is not None:
            print(f"✅ 图像接收成功!")
            break
        time.sleep(0.1)
    else:
        print("❌ 未接收到图像")
        return
    
    # 初始化机器人
    if not robot_controller.initialize_robot():
        return
    
    # 主循环
    target_marker_id = 0  # 目标标记ID
    
    try:
        while True:
            # 获取图像帧
            frame = camera_receiver.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            
            # 检测ArUco标记
            detection_results = aruco_processor.detect_markers(frame)
            
            # 绘制检测到的标记
            frame = aruco_processor.draw_markers(frame, detection_results)
            
            # 处理6x6标记
            success, rvec, tvec = aruco_processor.process_marker(frame, target_marker_id, '6x6')
            
            if success:
                # 更新机器人姿态信息
                if robot_controller.update_robot_pose():
                    pass  # 每15帧更新一次
                
                # 计算ArUco变换
                marker_pos_base, marker_quat_base, target_left_hand_pos, target_left_hand_quat = \
                    aruco_processor.calculate_aruco_transforms(
                        rvec, tvec, robot_controller.latest_head_quat, 
                        robot_controller.roll_offset, robot_controller.offset
                    )
                
                if marker_pos_base is not None:
                    # 更新标记信息
                    robot_controller.update_marker_info(
                        marker_pos_base, marker_quat_base, 
                        target_left_hand_pos, target_left_hand_quat
                    )
            
            # 执行跟随模式
            if robot_controller.execute_follow_mode():
                robot_controller.print_status_info()
            
            # 显示图像
            cv2.imshow("ArUco Detection", frame)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):  # 退出程序
                break
            elif key == ord('w'):  # 摄像头控制
                robot_controller.update_camera_angle('up')
            elif key == ord('s'):
                robot_controller.update_camera_angle('down')
            elif key == ord('a'):
                robot_controller.update_camera_angle('left')
            elif key == ord('d'):
                robot_controller.update_camera_angle('right')
            elif key == ord('x'):
                robot_controller.update_camera_angle('reset')
            elif key == ord('n'):  # 跟随模式控制
                robot_controller.set_follow_mode(False)
            elif key == ord('m'):
                robot_controller.set_follow_mode(True)
            elif key == ord('i'):  # 位置调整
                robot_controller.adjust_offset('position', 'up')
            elif key == ord('k'):
                robot_controller.adjust_offset('position', 'down')
            elif key == ord('j'):
                robot_controller.adjust_offset('position', 'left')
            elif key == ord('l'):
                robot_controller.adjust_offset('position', 'right')
            elif key == ord('r'):
                robot_controller.adjust_offset('position', 'forward')
            elif key == ord('t'):
                robot_controller.adjust_offset('position', 'backward')
            elif key == ord('u'):  # 角度调整
                robot_controller.adjust_offset('roll', 'positive')
            elif key == ord('o'):
                robot_controller.adjust_offset('roll', 'negative')
            elif key == ord('y'):
                robot_controller.adjust_offset('pitch', 'positive')
            elif key == ord('p'):
                robot_controller.adjust_offset('pitch', 'negative')
            elif key == ord('g'):
                robot_controller.adjust_offset('yaw', 'positive')
            elif key == ord('h'):
                robot_controller.adjust_offset('yaw', 'negative')
            elif key == ord('z'):  # 重置偏移
                robot_controller.reset_offsets('default')
            elif key == ord('c'):
                robot_controller.reset_offsets('aruco_aligned')
            elif key == ord('v'):
                robot_controller.reset_offsets('extended')
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    
    finally:
        # 清理资源
        robot_controller.shutdown()
        camera_receiver.stop()
        cv2.destroyAllWindows()
        print("程序结束")


if __name__ == "__main__":
    main()