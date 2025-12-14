#!/bin/bash

# SSH远程连接到摄像头服务器并启动图像服务
# 服务器信息: 10.192.1.3, 用户名: guest, 密码: 123456
# 依赖  sudo apt-get install sshpassserver

echo "正在连接到摄像头服务器 10.192.1.3..."

# 使用sshpass进行自动密码输入的SSH连接
sshpass -p '123456' ssh -o StrictHostKeyChecking=no guest@10.192.1.3 << 'EOF'
echo "已连接到摄像头服务器"
echo "切换到工作目录..."
cd /home/guest/xianpeng/teleoperate/

echo "初始化conda环境..."
# 初始化conda (通常conda安装在/opt/conda或~/miniconda3)
if [ -f "/opt/conda/etc/profile.d/conda.sh" ]; then
    source /opt/conda/etc/profile.d/conda.sh
elif [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source $HOME/miniconda3/etc/profile.d/conda.sh
elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    source $HOME/anaconda3/etc/profile.d/conda.sh
else
    echo "⚠️  警告: 未找到conda安装路径，尝试使用PATH中的conda"
fi

echo "激活conda环境..."
conda activate teleoperte

echo "检查RealSense相机连接..."
lsusb | grep Intel || echo "⚠️  警告: 未检测到Intel RealSense设备"

echo "启动图像服务器..."
bash start_image_server.sh --stats
EOF

echo "摄像头服务器连接已断开"