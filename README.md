# ArUco机器人视觉伺服控制系统

基于ArUco标记检测的人形机器人视觉伺服控制系统，支持实时标记检测、坐标变换和机械臂跟随控制。

## 项目结构

```
├── api/                    # API接口模块
│   ├── camera_api.py      # 相机API
│   ├── robot_control_api.py # 机器人控制API
│   └── api_demo/          # API演示代码
├── utils/                 # 工具模块
│   ├── coordinate_transforms.py # 坐标变换工具
│   ├── aruco_processor.py      # ArUco处理器
│   ├── robot_controller.py     # 机器人控制器
│   └── __init__.py
├── config/                # 配置模块
│   ├── camera_config.py   # 相机配置
│   ├── robot_config.py    # 机器人配置
│   └── __init__.py
├── assets/                # 资源文件
│   └── HU_D04_01.urdf    # 机器人URDF文件
├── md/                    # 文档
├── tools/                 # 工具脚本
├── main.py               # 主程序入口
└── all.py                # 原始单文件版本
```

## 功能特性

### 核心功能
- **ArUco标记检测**: 支持6x6和4x4标记检测
- **坐标系变换**: 相机坐标系到机器人base_link坐标系的完整变换链
- **实时视觉伺服**: 基于标记位置的机械臂实时跟随控制
- **头部相机控制**: 支持俯仰和偏航角度控制

### 控制接口
- **相机控制** (WASD键):
  - W/S: 俯仰控制 (±5°)
  - A/D: 偏航控制 (±5°)
  - X: 复位到中心位置

- **跟随模式控制**:
  - M: 启用跟随模式
  - N: 禁用跟随模式

- **位置偏移调整** (跟随模式下):
  - I/K: Z轴上下 (±2cm)
  - J/L: Y轴左右 (±2cm)
  - R/T: X轴前后 (±2cm)

- **姿态偏移调整** (跟随模式下):
  - U/O: Roll角度 (±3°)
  - Y/P: Pitch角度 (±3°)
  - G/H: Yaw角度 (±3°)

- **预设配置**:
  - Z: 默认配置
  - C: ArUco对齐模式
  - V: 扩展模式

## 安装依赖

```bash
pip install opencv-python numpy
```

## 使用方法

### 1. 运行主程序
```bash
python main.py
```

### 2. 或运行原始版本
```bash
python all.py
```

## 模块说明

### utils/coordinate_transforms.py
坐标变换核心函数:
- 四元数 ↔ 旋转矩阵 ↔ 欧拉角转换
- 齐次变换矩阵生成
- ArUco坐标到参考坐标系变换

### utils/aruco_processor.py
ArUco处理器类:
- 标记检测和绘制
- 位姿估计 (solvePnP)
- 坐标系变换计算

### utils/robot_controller.py
机器人控制器类:
- 机器人初始化和状态管理
- 相机角度控制
- 跟随模式执行
- 偏移量调整

### config/
配置参数模块:
- 相机内参和标记配置
- 机器人URDF参数
- 控制参数和预设值

## 坐标系定义

- **Base_link坐标系**: X向前，Y向左，Z向上
- **相机坐标系**: X向前，Y向左，Z向上 (经过OpenCV坐标系转换)
- **ArUco标记坐标系**: 遵循OpenCV ArUco标准

## 技术特点

1. **模块化设计**: 清晰的功能分离和接口定义
2. **配置化参数**: 所有关键参数可配置
3. **实时性能**: 帧跳跃处理，优化计算频率
4. **鲁棒性**: 完善的错误处理和状态检查
5. **可扩展性**: 易于添加新的标记类型和控制模式

## 注意事项

1. 确保相机已正确标定
2. 检查URDF变换参数的准确性
3. 根据实际机器人调整控制参数
4. 注意安全距离和工作空间限制