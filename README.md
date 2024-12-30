# 机械臂矩形绘制项目

本项目实现了一个机械臂控制系统，可以在RVIZ中可视化并通过真实机械臂绘制矩形。系统提供轨迹可视化和实时控制功能。

## 项目概述

该项目包含以下主要功能：
- 在RVIZ中可视化计划的矩形轨迹
- 控制真实机械臂绘制矩形
- 通过GUI界面实时调整矩形位置和大小
- 轨迹生成和运动控制
- 关节位置控制和监控

## 环境要求

- ROS 2（Robot Operating System 2）
- Python 3
- RVIZ2 可视化工具
- PyQt5（用于GUI界面）
- 所需Python包
- 兼容的机械臂硬件（STS3215系列舵机）

## 项目结构

```
.
├── ros2_ws/                 # ROS 2工作空间
│   ├── src/
│   │   ├── arm_description/    # 机械臂描述包
│   │   └── arm_visualization/  # 可视化和控制包
│   │       └── arm_visualization/
│   │           ├── rectangle_visualizer.py    # 矩形绘制节点
│   │           ├── rectangle_gui_controller.py # GUI控制界面
│   │           ├── arm_controller.py          # 机械臂控制
│   │           └── trajectory_generator.py    # 轨迹生成
├── arm_controller.py         # 机械臂控制实现
├── motion_controller.py      # 运动控制算法
├── trajectory_generator.py   # 轨迹生成工具
└── motors/                  # 电机控制实现
```

## 安装和配置

1. 确保系统已安装ROS 2和PyQt5
   ```bash
   sudo apt install python3-pyqt5  # 安装PyQt5
   ```

2. 进入工作空间并编译：
   ```bash
   cd /home/ubuntu22/so_arm100_write/ros2_ws
   colcon build
   source install/setup.bash
   ```

## 使用方法

### 1. 启动ROS 2环境
打开新终端，进入工作空间并source环境：
```bash
cd /home/ubuntu22/so_arm100_write/ros2_ws
source install/setup.bash
```

### 2. 启动机械臂描述和可视化
在终端中运行：
```bash
ros2 launch arm_description display.launch.py
```

### 3. 启动矩形绘制节点
打开新终端，source环境后运行：
```bash
ros2 run arm_visualization rectangle_visualizer
```

### 4. 启动GUI控制界面（可选）
如果需要实时调整矩形的位置和大小，打开新终端，source环境后运行：
```bash
ros2 run arm_visualization rectangle_gui_controller
```

这将打开一个GUI窗口，您可以通过以下控制：
- 使用滑块调整矩形的X、Y位置
- 调整矩形的宽度和高度
- 实时预览RVIZ中的变化
- 确认后机械臂将执行新的轨迹

此时，您将看到：
- RVIZ中显示机械臂模型
- 可视化计划的矩形轨迹
- GUI界面用于调整矩形参数
- 真实机械臂执行绘制动作

### 主要功能

- RVIZ中的实时轨迹可视化
- 可配置的矩形尺寸和位置
- 图形界面实时调整参数
- 关节位置监控和限位保护
- 平滑的运动控制
- 安全检查和位置验证

### 参数配置

可以通过以下方式调整参数：
1. 使用GUI界面实时调整
2. 修改`rectangle_visualizer.py`中的默认参数：
   - 矩形尺寸（宽度、高度）
   - 位置（中心点x、y坐标）
   - 绘制高度（z坐标）
   - 运动速度和加速度

## 安全注意事项

- 系统包含关节限位检查，防止过度旋转
- 位置监控确保安全运动
- 提供紧急停止功能
- 操作前确保工作空间畅通

## 故障排除

1. 如果机械臂不动：
   - 检查电机连接
   - 验证电源供应
   - 确保关节限位设置正确

2. 如果可视化不显示：
   - 检查是否正确source环境
   - 确保RVIZ正常运行
   - 检查是否有错误信息输出

3. 如果GUI界面无法启动：
   - 确认PyQt5已正确安装
   - 检查Python环境
   - 查看终端错误输出

## 贡献

欢迎提交问题和改进建议。

## 许可证

[在此指定许可证]
