# 机械臂写字项目

## 项目概述
本项目实现了使用机械臂（ARM100）进行汉字书写的功能。通过精确的运动控制和轨迹规划，使机械臂能够模拟人类书写汉字的过程。目前已实现"二"和"四"两个汉字的书写功能。

## 主要功能
1. 汉字笔画轨迹规划
2. 机械臂运动控制
3. 实时轨迹可视化
4. 写字区域参数自定义

## 系统架构

### 核心模块
- **运动控制器** (`motion_controller.py`)
  - 基于DH参数的运动学计算
  - 机械臂关节角度控制
  - 笔尖位置精确控制

- **轨迹生成器** (`trajectory_generator.py`)
  - 笔画轨迹点生成
  - 路径插值算法
  - 平滑运动控制

- **可视化模块** (`rectangle_visualizer.py`)
  - RVIZ轨迹可视化
  - 实时运动状态显示
  - 用户交互界面

### 写字参数配置
- 写字区域大小：100mm × 100mm
- 基座距离：150mm
- 写字平面高度：30mm
- 抬笔高度：50mm

## 运行环境
- ROS2
- Python 3
- MoveIt2

## 使用说明

### 1. 环境配置
```bash
# 配置ROS2环境
source /home/ubuntu22/so_arm100_write/ros2_ws/install/setup.bash
```

### 2. 启动可视化
```bash
# 运行可视化节点
ros2 run arm_visualization rectangle_visualizer
```

### 3. 执行写字任务
- 启动后系统会在RVIZ中显示预览轨迹
- 确认轨迹无误后，可执行实际写字动作
- 系统会自动控制抬笔和落笔动作

## 项目特点
1. 精确的运动学控制
   - 使用DH参数进行位置计算
   - 考虑笔尖偏移量
   - 关节限位保护

2. 平滑的轨迹生成
   - 路径插值算法
   - 合理的过渡时间
   - 自然的运动效果

3. 友好的用户交互
   - 实时轨迹预览
   - 状态提示
   - 操作确认机制

## 注意事项
1. 运行前确保机械臂已正确连接
2. 检查写字平面高度设置
3. 确认笔尖安装位置正确
4. 注意观察RVIZ中的轨迹预览

## 后续开发计划
1. 支持更多汉字的书写
2. 优化笔画轨迹生成算法
3. 添加字体风格定制功能
4. 提升写字速度和稳定性
