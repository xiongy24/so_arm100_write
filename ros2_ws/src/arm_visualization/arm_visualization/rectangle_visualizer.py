#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
from .arm_controller import ArmController
import time
from sensor_msgs.msg import JointState

class RectangleVisualizer(Node):
    def __init__(self):
        super().__init__('rectangle_visualizer')
        self.arm = ArmController()
        
        # 创建发布者
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.current_path_pub = self.create_publisher(Marker, 'current_path_marker', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # 初始化消息
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.001  # 线宽
        self.marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # 红色
        
        self.current_path_marker = Marker()
        self.current_path_marker.header.frame_id = "base_link"
        self.current_path_marker.type = Marker.LINE_STRIP
        self.current_path_marker.action = Marker.ADD
        self.current_path_marker.scale.x = 0.001  # 线宽
        self.current_path_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # 绿色
        
        # 创建关节状态消息
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
            'jaw_joint'
        ]

    def publish_joint_states(self, joint_angles):
        """发布关节状态以在 RVIZ 中更新机械臂位置"""
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = joint_angles
        self.joint_state_pub.publish(self.joint_state_msg)

    def preview_trajectory(self, points):
        """在 RVIZ 中预览轨迹"""
        print("\n开始轨迹预览...")
        for point in points:
            # 添加姿态信息（保持笔尖朝下）
            target_pose = np.append(point, [0.0, np.pi, 0.0])  # [x, y, z, roll, pitch, yaw]
            
            # 计算逆运动学
            joint_angles = self.arm.get_joint_angles(target_pose)
            if joint_angles is None:
                print(f"警告：无法到达点 {point}")
                continue
            
            # 发布关节状态以更新 RVIZ 中的机械臂位置
            self.publish_joint_states(joint_angles)
            
            # 等待一小段时间以便观察
            time.sleep(0.1)
        
        print("轨迹预览完成。按回车键执行实际运动，或按 Ctrl+C 取消...")
        input()

    def move_rectangle(self):
        """执行矩形轨迹运动"""
        # 从文件读取位置信息
        try:
            with open('/home/ubuntu22/so_arm100_write/ros2_ws/src/arm_visualization/rectangle_position.txt', 'r') as f:
                position_data = {}
                for line in f:
                    key, value = line.strip().split(': ')
                    position_data[key] = float(value)
            
            # 设置矩形参数（单位：米）
            width = 0.08    # 宽度
            height = 0.08   # 高度
            x_offset = position_data['x']  # 从文件读取的X偏移
            y_offset = position_data['y']  # 从文件读取的Y偏移
            z_offset = position_data['z']  # 从文件读取的Z偏移
        except (FileNotFoundError, KeyError, ValueError) as e:
            self.get_logger().error(f'Error reading position file: {str(e)}')
            return
        
        # 定义矩形的四个角点（基于基座坐标系）
        points = [
            np.array([x_offset, y_offset, z_offset]),               # 左下角
            np.array([x_offset + width, y_offset, z_offset]),             # 右下角
            np.array([x_offset + width, y_offset + width, z_offset]),     # 右上角
            np.array([x_offset, y_offset + width, z_offset]),       # 左上角
            np.array([x_offset, y_offset, z_offset])                # 回到起点
        ]
        
        # 发布预期轨迹（红色）
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points]
        print("\n正在发布轨迹可视化消息...")
        
        # 持续发布5秒，确保RVIZ能接收到消息
        start_time = time.time()
        while time.time() - start_time < 5.0:
            self.marker_pub.publish(self.marker)
            time.sleep(0.1)
        
        print("\n已持续发布5秒消息，现在应该可以在RVIZ的 'By topic' 中看到 '/visualization_marker'。")
        print("完成Marker添加后，按回车键继续...")
        input()
        
        print("\n继续发布轨迹可视化消息...")
        
        # 预览轨迹
        self.preview_trajectory(points)
        
        # 执行实际运动
        print("\n开始执行实际运动...")
        self.current_path_marker.points = []  # 清空当前路径
        
        for point in points:
            # 移动到目标点
            success = self.arm.move_to(point)
            if not success:
                print(f"无法到达点 {point}")
                continue
            
            # 记录实际路径
            current_pos = self.arm.get_current_position()
            self.current_path_marker.points.append(Point(x=current_pos[0], y=current_pos[1], z=current_pos[2]))
            self.current_path_marker.header.stamp = self.get_clock().now().to_msg()
            self.current_path_pub.publish(self.current_path_marker)
        
        print("运动完成！")

def main(args=None):
    rclpy.init(args=args)
    
    print("\n=== 矩形轨迹可视化程序 ===")
    print("请确保已经完成以下步骤：")
    print("1. 在另一个终端运行：ros2 launch arm_description display.launch.py")
    print("2. 在RVIZ中添加两个Marker显示：")
    print("   - 点击 'Add' 按钮")
    print("   - 选择 'By topic'")
    print("   - 等待 '/visualization_marker' 和 '/current_path_marker' 出现（大约需要3-5秒）")
    print("   - 分别选择这两个话题")
    print("   - 点击 'OK' 确认\n")
    
    visualizer = RectangleVisualizer()
    visualizer.move_rectangle()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
