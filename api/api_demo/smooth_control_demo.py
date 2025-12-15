#!/usr/bin/env python3
"""
平滑控制演示脚本
测试轨迹插值器的功能
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

import numpy as np
import time
import matplotlib.pyplot as plt
from utils.trajectory_interpolator import AdaptiveTrajectoryInterpolator
from utils.coordinate_transforms import rotation_matrix_to_quaternion, rotation_matrix


def test_position_interpolation():
    """测试位置插值"""
    print("🧪 测试位置插值...")
    
    # 创建插值器
    interpolator = AdaptiveTrajectoryInterpolator(
        interpolation_steps=20,
        max_velocity=0.1,  # 10cm/s
        max_angular_velocity=0.5
    )
    
    # 设置起始位置和姿态
    start_pos = np.array([0.3, 0.1, 0.5])
    start_quat = np.array([0, 0, 0, 1])
    
    # 设置目标位置和姿态
    target_pos = np.array([0.5, 0.3, 0.4])
    target_quat = rotation_matrix_to_quaternion(rotation_matrix([0, 0, np.pi/4]))
    
    # 初始化插值器
    interpolator.set_target(start_pos, start_quat)
    interpolator.set_target(target_pos, target_quat)
    
    # 记录轨迹
    positions = []
    quaternions = []
    timestamps = []
    
    start_time = time.time()
    
    while not interpolator.is_motion_complete():
        pos, quat, is_moving = interpolator.get_next_waypoint()
        if pos is not None:
            positions.append(pos.copy())
            quaternions.append(quat.copy())
            timestamps.append(time.time() - start_time)
        
        time.sleep(0.033)  # 30Hz
    
    # 绘制结果
    positions = np.array(positions)
    timestamps = np.array(timestamps)
    
    plt.figure(figsize=(12, 8))
    
    # 位置轨迹
    plt.subplot(2, 2, 1)
    plt.plot(timestamps, positions[:, 0], 'r-', label='X')
    plt.plot(timestamps, positions[:, 1], 'g-', label='Y')
    plt.plot(timestamps, positions[:, 2], 'b-', label='Z')
    plt.xlabel('时间 (s)')
    plt.ylabel('位置 (m)')
    plt.title('位置轨迹')
    plt.legend()
    plt.grid(True)
    
    # 速度曲线
    plt.subplot(2, 2, 2)
    if len(positions) > 1:
        dt = np.diff(timestamps)
        velocities = np.diff(positions, axis=0) / dt[:, np.newaxis]
        plt.plot(timestamps[1:], np.linalg.norm(velocities, axis=1), 'k-', label='速度大小')
        plt.axhline(y=0.1, color='r', linestyle='--', label='最大速度')
        plt.xlabel('时间 (s)')
        plt.ylabel('速度 (m/s)')
        plt.title('速度曲线')
        plt.legend()
        plt.grid(True)
    
    # 3D轨迹
    ax = plt.subplot(2, 2, 3, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2)
    ax.scatter(start_pos[0], start_pos[1], start_pos[2], color='g', s=100, label='起点')
    ax.scatter(target_pos[0], target_pos[1], target_pos[2], color='r', s=100, label='终点')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D轨迹')
    ax.legend()
    
    # 四元数变化
    plt.subplot(2, 2, 4)
    quaternions = np.array(quaternions)
    plt.plot(timestamps, quaternions[:, 0], 'r-', label='qx')
    plt.plot(timestamps, quaternions[:, 1], 'g-', label='qy')
    plt.plot(timestamps, quaternions[:, 2], 'b-', label='qz')
    plt.plot(timestamps, quaternions[:, 3], 'k-', label='qw')
    plt.xlabel('时间 (s)')
    plt.ylabel('四元数分量')
    plt.title('姿态变化')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('smooth_control_test.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    print(f"✅ 插值完成，总时间: {timestamps[-1]:.2f}s，总步数: {len(positions)}")


def test_multiple_targets():
    """测试多目标连续运动"""
    print("🧪 测试多目标连续运动...")
    
    interpolator = AdaptiveTrajectoryInterpolator(
        interpolation_steps=15,
        max_velocity=0.08,
        max_angular_velocity=0.3
    )
    
    # 定义多个目标点
    targets = [
        (np.array([0.3, 0.1, 0.5]), np.array([0, 0, 0, 1])),
        (np.array([0.4, 0.2, 0.6]), rotation_matrix_to_quaternion(rotation_matrix([0, 0, np.pi/6]))),
        (np.array([0.5, 0.1, 0.5]), rotation_matrix_to_quaternion(rotation_matrix([0, 0, np.pi/3]))),
        (np.array([0.4, 0.0, 0.4]), rotation_matrix_to_quaternion(rotation_matrix([0, 0, 0]))),
        (np.array([0.3, 0.1, 0.5]), np.array([0, 0, 0, 1])),
    ]
    
    # 初始化到第一个目标
    interpolator.set_target(targets[0][0], targets[0][1])
    
    positions = []
    timestamps = []
    target_changes = []
    
    start_time = time.time()
    target_index = 1
    
    while target_index < len(targets):
        current_time = time.time() - start_time
        
        # 检查是否需要设置新目标
        if interpolator.is_motion_complete() and target_index < len(targets):
            print(f"🎯 设置目标 {target_index}: {targets[target_index][0]}")
            interpolator.set_target(targets[target_index][0], targets[target_index][1])
            target_changes.append(current_time)
            target_index += 1
        
        # 获取当前路径点
        pos, quat, is_moving = interpolator.get_next_waypoint()
        if pos is not None:
            positions.append(pos.copy())
            timestamps.append(current_time)
        
        time.sleep(0.033)  # 30Hz
    
    # 等待最后一个运动完成
    while not interpolator.is_motion_complete():
        pos, quat, is_moving = interpolator.get_next_waypoint()
        if pos is not None:
            positions.append(pos.copy())
            timestamps.append(time.time() - start_time)
        time.sleep(0.033)
    
    # 绘制结果
    positions = np.array(positions)
    timestamps = np.array(timestamps)
    
    plt.figure(figsize=(10, 6))
    
    # 3D轨迹
    ax = plt.subplot(1, 2, 1, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2, alpha=0.7)
    
    # 标记目标点
    for i, (target_pos, _) in enumerate(targets):
        color = 'g' if i == 0 else 'r'
        ax.scatter(target_pos[0], target_pos[1], target_pos[2], 
                  color=color, s=100, label=f'目标{i}')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('多目标连续运动轨迹')
    ax.legend()
    
    # 时间序列
    plt.subplot(1, 2, 2)
    plt.plot(timestamps, positions[:, 0], 'r-', label='X')
    plt.plot(timestamps, positions[:, 1], 'g-', label='Y')
    plt.plot(timestamps, positions[:, 2], 'b-', label='Z')
    
    # 标记目标切换时间
    for i, change_time in enumerate(target_changes):
        plt.axvline(x=change_time, color='k', linestyle='--', alpha=0.5)
        plt.text(change_time, plt.ylim()[1]*0.9, f'T{i+1}', rotation=90)
    
    plt.xlabel('时间 (s)')
    plt.ylabel('位置 (m)')
    plt.title('位置随时间变化')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('multi_target_test.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    print(f"✅ 多目标运动完成，总时间: {timestamps[-1]:.2f}s")


def test_speed_comparison():
    """测试不同速度设置的效果"""
    print("🧪 测试不同速度设置...")
    
    start_pos = np.array([0.3, 0.1, 0.5])
    target_pos = np.array([0.5, 0.3, 0.4])
    start_quat = np.array([0, 0, 0, 1])
    target_quat = rotation_matrix_to_quaternion(rotation_matrix([0, 0, np.pi/4]))
    
    speeds = [0.05, 0.1, 0.2]  # 不同的最大速度
    colors = ['r', 'g', 'b']
    
    plt.figure(figsize=(12, 4))
    
    for i, speed in enumerate(speeds):
        interpolator = AdaptiveTrajectoryInterpolator(
            interpolation_steps=15,
            max_velocity=speed,
            max_angular_velocity=speed * 5
        )
        
        interpolator.set_target(start_pos, start_quat)
        interpolator.set_target(target_pos, target_quat)
        
        positions = []
        timestamps = []
        start_time = time.time()
        
        while not interpolator.is_motion_complete():
            pos, quat, is_moving = interpolator.get_next_waypoint()
            if pos is not None:
                positions.append(pos.copy())
                timestamps.append(time.time() - start_time)
            time.sleep(0.033)
        
        positions = np.array(positions)
        timestamps = np.array(timestamps)
        
        # 绘制轨迹
        plt.subplot(1, 3, 1)
        plt.plot(timestamps, np.linalg.norm(positions - start_pos, axis=1), 
                colors[i], label=f'{speed*100:.0f}cm/s')
        
        plt.subplot(1, 3, 2)
        if len(positions) > 1:
            dt = np.diff(timestamps)
            velocities = np.diff(positions, axis=0) / dt[:, np.newaxis]
            plt.plot(timestamps[1:], np.linalg.norm(velocities, axis=1), 
                    colors[i], label=f'{speed*100:.0f}cm/s')
        
        plt.subplot(1, 3, 3)
        plt.plot(positions[:, 0], positions[:, 1], colors[i], 
                label=f'{speed*100:.0f}cm/s', linewidth=2)
    
    plt.subplot(1, 3, 1)
    plt.xlabel('时间 (s)')
    plt.ylabel('距离起点 (m)')
    plt.title('运动距离')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(1, 3, 2)
    plt.xlabel('时间 (s)')
    plt.ylabel('速度 (m/s)')
    plt.title('瞬时速度')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(1, 3, 3)
    plt.scatter(start_pos[0], start_pos[1], color='g', s=100, label='起点')
    plt.scatter(target_pos[0], target_pos[1], color='r', s=100, label='终点')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('XY平面轨迹')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('speed_comparison_test.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    print("✅ 速度对比测试完成")


def main():
    """主函数"""
    print("🚀 平滑控制演示程序")
    print("=" * 50)
    
    try:
        # 测试基础插值
        test_position_interpolation()
        
        # 测试多目标运动
        test_multiple_targets()
        
        # 测试速度对比
        test_speed_comparison()
        
        print("\n✅ 所有测试完成！")
        print("📊 生成的图片:")
        print("  - smooth_control_test.png: 基础插值测试")
        print("  - multi_target_test.png: 多目标运动测试")
        print("  - speed_comparison_test.png: 速度对比测试")
        
    except Exception as e:
        print(f"❌ 测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()