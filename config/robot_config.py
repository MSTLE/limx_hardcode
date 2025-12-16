"""
机器人配置参数
"""
import numpy as np

# URDF中的变换参数
URDF_TRANSFORMS = {
    # 头部相机关节参数 (从URDF获取)
    'head_camera_joint': {
        'xyz': [0.07453, 0.0175, 0.065],
        'rpy': [0.0, 1.5708, 0.0]
    },
    # 头部到base_link的偏移
    'head_to_base': {
        'xyz': [-0.013, 0, 0.58709],
        'rpy': [0.0, 0.0, 0.0]
    }
}

# 相机控制参数
CAMERA_CONTROL = {
    'initial_pitch': np.radians(-180),
    'initial_yaw': np.radians(-180),
    'angle_step': 5.0,  # 度
    'pitch_limits': {
        'min': np.radians(-225),
        'max': np.radians(-155)
    },
    'yaw_limits': {
        'min': np.radians(-210),
        'max': np.radians(-150)
    }
}

# 末端执行器控制参数
END_EFFECTOR_CONFIG = {
    'initial_offset': np.array([0.07, 0.02, -0.10]),  # 米
    'offset_step': 0.02,  # 米
    'initial_roll_offset': np.radians(6.0),
    'initial_pitch_offset': np.radians(-111),
    'initial_yaw_offset': 0.0,
    'angle_step': np.radians(3.0)  # 弧度
}

# 预设偏移配置
PRESET_OFFSETS = {
    'default': {
        'offset': np.array([-0.20, 0.02, -0.10]),
        'roll_offset': np.radians(6.0),
        'pitch_offset': np.radians(-111),
        'yaw_offset': 0.0
    },
    'aruco_aligned': {
        'offset': np.array([0.05, 0.08 -0.15]),
        'roll_offset': np.radians(6.0)
    },
    'extended': {
        'offset': np.array([0.05, 0.10, -0.35])
    },
    'opened': {
        'offset': np.array([0.45, 0.10, -0.40])
    },
    'open_ac': {
        'offset': np.array([0.45, -0.25, -0.20])
    }
}

# 控制频率参数
CONTROL_FREQUENCY = {
    'pose_update_interval': 15,  # 每15帧更新一次姿态
    'command_send_interval': 15  # 每15帧发送一次控制命令
}