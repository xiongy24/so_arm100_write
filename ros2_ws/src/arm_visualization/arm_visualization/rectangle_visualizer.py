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
        
        # 创建机械臂控制器
        self.arm = ArmController()
        
        # 连接机械臂
        print("正在连接机械臂...")
        if not self.arm.connect():
            print("警告：无法连接到机械臂，将在仿真模式下运行")
            self.arm.simulation_mode = True
        else:
            self.arm.simulation_mode = False
            
        # 创建Marker发布器
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.current_path_pub = self.create_publisher(Marker, '/current_path_marker', 10)
        
        # 创建关节状态发布器
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # 创建定时器，10Hz发布关节状态
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # 初始化Marker
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.ns = "expected_path"
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.005  # 线宽
        self.marker.color.r = 1.0  # 红色
        self.marker.color.a = 1.0  # 不透明度
        
        self.current_path_marker = Marker()
        self.current_path_marker.header.frame_id = "base_link"
        self.current_path_marker.ns = "current_path"
        self.current_path_marker.id = 1
        self.current_path_marker.type = Marker.LINE_STRIP
        self.current_path_marker.action = Marker.ADD
        self.current_path_marker.pose.orientation.w = 1.0
        self.current_path_marker.scale.x = 0.005  # 线宽
        self.current_path_marker.color.g = 1.0  # 绿色
        self.current_path_marker.color.a = 1.0  # 不透明度
        
        # 初始化关节状态消息
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
            'jaw_joint'
        ]
        # 初始化速度和加速度字段为空列表
        self.joint_state_msg.velocity = []
        self.joint_state_msg.effort = []
        
        # 存储当前关节角度
        self.current_angles = [0.0] * 6

    def publish_joint_states(self):
        """发布关节状态以在 RVIZ 中更新机械臂位置"""
        now = self.get_clock().now()
        self.joint_state_msg.header.stamp = now.to_msg()
        self.joint_state_msg.position = self.current_angles
        self.joint_state_pub.publish(self.joint_state_msg)

    def update_joint_angles(self, joint_angles):
        """更新当前关节角度"""
        self.current_angles = [float(angle) for angle in joint_angles]

    def interpolate_path(self, start_point, end_point, num_points=150):
        """在两点之间插值生成平滑路径"""
        path = []
        for i in range(num_points):
            t = i / (num_points - 1)
            point = start_point * (1 - t) + end_point * t
            path.append(point)
        return path

    def is_point_in_list(self, point, points_list, tolerance=1e-6):
        """检查点是否在点列表中"""
        for p in points_list:
            if np.allclose(point, p, atol=tolerance):
                return True
        return False

    def preview_trajectory(self, points):
        """预览轨迹"""
        print("\n开始轨迹预览...")
        
        # 更新轨迹标记点
        self.marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points]
        self.marker.header.stamp = self.get_clock().now().to_msg()
        
        # 持续发布轨迹消息，直到确认RVIZ已接收
        print("正在发布轨迹可视化消息...")
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < 5.0:  # 发布5秒
            self.marker_pub.publish(self.marker)
            self.current_path_pub.publish(self.current_path_marker)
            time.sleep(0.1)
        
        # 预览每个点的关节角度
        print("\n开始轨迹预览（每段插值150个点）...")
        for i in range(len(points) - 1):
            print(f"\n预览第 {i+1}/{len(points)-1} 段轨迹")
            
            # 在相邻两点之间生成平滑路径
            interpolated_points = self.interpolate_path(points[i], points[i + 1])
            
            # 计算这段轨迹的总长度
            segment_length = np.linalg.norm(points[i+1] - points[i])
            # 根据轨迹长度动态调整延时，使速度保持恒定
            point_delay = 0.02  # 基础延时20ms
            
            for point in interpolated_points:
                # 设置目标姿态（笔尖垂直向下）
                target_pose = np.append(point, [0.0, -np.pi/2, 0.0])  # [x, y, z, roll, pitch, yaw]
                
                # 计算逆运动学
                joint_angles = self.arm.get_joint_angles(target_pose)
                if joint_angles is None:
                    print(f"警告：无法计算点 {point} 的逆运动学解")
                    continue
                
                # 更新当前关节角度（用于可视化）
                self.current_angles = joint_angles
                
                # 显示关节角度（角度制）
                if self.is_point_in_list(point, points):  # 只显示关键点的角度
                    print(f"目标点 {point} 的关节角度: {np.degrees(joint_angles)}")
                
                # 更新当前路径标记
                self.current_path_marker.points.append(Point(x=point[0], y=point[1], z=point[2]))
                self.current_path_marker.header.stamp = self.get_clock().now().to_msg()
                self.current_path_pub.publish(self.current_path_marker)
                
                # 发布关节状态
                self.publish_joint_states()
                time.sleep(point_delay)  # 使用更小的延时实现连续运动

    def move_rectangle(self):
        """执行矩形轨迹运动"""
        try:
            # 从文件读取位置信息
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
            
            # 生成矩形轨迹点
            points = [
                np.array([x_offset, y_offset, z_offset]),  # 左下角
                np.array([x_offset + width, y_offset, z_offset]),  # 右下角
                np.array([x_offset + width, y_offset + height, z_offset]),  # 右上角
                np.array([x_offset, y_offset + height, z_offset]),  # 左上角
                np.array([x_offset, y_offset, z_offset])  # 回到起点
            ]
            
            # 等待用户完成RViz设置
            print("\n请在RViz中添加两个Marker显示：")
            print("1. 点击 'Add' 按钮")
            print("2. 选择 'By topic'")
            print("3. 等待 '/visualization_marker' 和 '/current_path_marker' 出现")
            print("4. 分别选择这两个话题")
            print("5. 点击 'OK' 确认")
            input("\n完成RViz设置后，按回车键继续...")
            
            # 清空当前路径标记
            self.current_path_marker.points = []
            self.current_path_marker.header.stamp = self.get_clock().now().to_msg()
            self.current_path_pub.publish(self.current_path_marker)
            
            # 预览轨迹
            self.preview_trajectory(points)
            
            # 等待用户确认
            input("\n轨迹预览完成。按回车键执行实际运动，或按 Ctrl+C 取消...\n")
            
            # 执行实际运动
            print("\n开始执行实际运动...")
            
            # 如果是实际硬件模式，只在第一次移动到初始位置
            if not self.arm.simulation_mode:
                print("正在移动到初始位置...")
                if not self.arm.move_to_zero():
                    print("错误：无法移动到初始位置")
                    return
                time.sleep(2)  # 等待稳定
            
            # 清空实际路径标记
            self.current_path_marker.points = []
            self.current_path_marker.header.stamp = self.get_clock().now().to_msg()
            self.current_path_pub.publish(self.current_path_marker)
            
            # 记录上一个点的关节角度，用于下一次运动的初始位置
            last_joint_angles = None
            
            for i in range(len(points) - 1):
                print(f"\n执行第 {i+1}/{len(points)-1} 段轨迹")
                
                # 在相邻两点之间生成平滑路径
                interpolated_points = self.interpolate_path(points[i], points[i + 1])
                
                # 计算这段轨迹的总长度
                segment_length = np.linalg.norm(points[i+1] - points[i])
                # 根据轨迹长度动态调整延时，使速度保持恒定
                point_delay = 0.02  # 基础延时20ms
                
                for point in interpolated_points:
                    # 设置目标姿态（笔尖垂直向下）
                    target_pose = np.append(point, [0.0, -np.pi/2, 0.0])  # [x, y, z, roll, pitch, yaw]
                    
                    # 计算逆运动学，使用上一个点的关节角度作为初始位置
                    joint_angles = self.arm.get_joint_angles(target_pose, initial_position=last_joint_angles)
                    if joint_angles is None:
                        print(f"警告：无法到达点 {point}")
                        continue
                    
                    # 移动到目标位置
                    success = self.arm.move_to_joint_angles(joint_angles)
                    if not success:
                        print(f"警告：移动到点 {point} 失败")
                        continue
                    
                    # 更新实际路径标记
                    self.current_path_marker.points.append(Point(x=point[0], y=point[1], z=point[2]))
                    self.current_path_marker.header.stamp = self.get_clock().now().to_msg()
                    self.current_path_pub.publish(self.current_path_marker)
                    
                    # 记录当前关节角度，用于下一次运动
                    last_joint_angles = joint_angles
                    
                    # 等待运动完成
                    time.sleep(point_delay)  # 使用更小的延时实现连续运动
            
            print("矩形轨迹执行完成！")
            
        except Exception as e:
            self.get_logger().error(f'Error executing rectangle trajectory: {str(e)}')
            
        finally:
            # 如果是实际硬件模式，断开连接
            if not self.arm.simulation_mode:
                self.arm.disconnect()

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
