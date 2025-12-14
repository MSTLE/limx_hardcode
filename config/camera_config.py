"""
相机配置参数
"""
import numpy as np

# RealSense相机内参矩阵
CAMERA_MATRIX = np.array([
    [907.719360351562, 0.0, 651.812194824219],
    [0.0, 908.330444335938, 387.889526367188],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

# 畸变系数
DIST_COEFFS = np.zeros((5, 1), dtype=np.float32)

# ArUco标记尺寸配置
MARKER_CONFIG = {
    '6x6': {
        'dict_type': 'DICT_6X6_250',
        'marker_length': 0.10,  # 10cm
    },
    '4x4': {
        'dict_type': 'DICT_4X4_250', 
        'marker_length': 0.04,  # 4cm
    }
}

# 目标标记ID
TARGET_MARKER_ID = 0