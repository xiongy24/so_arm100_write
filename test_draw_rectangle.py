#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from arm_controller import ArmController
import numpy as np
import time
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class RectangleVisualizer(Node):
    def __init__(self):
        super().__init__('rectangle_visualization')
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        time.sleep(0.5)  # 等待发布者初始化
        
    def visualize_rectangle(self, points, z_height):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rectangle_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 设置标记的尺寸和颜色
        marker.scale.x = 0.002  # 线宽2mm
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # 添加路径点
        marker.points = []
        for x, y in points:
            point = Point()
            point.x = x
            point.y = y
            point.z = z_height
            marker.points.append(point)
            
        self.marker_pub.publish(marker)

def generate_rectangle_points(center_x, center_y, width, height, num_points_per_side=20):
    """生成矩形的路径点"""
    points = []
    
    # 计算矩形四个角的坐标
    half_w = width / 2
    half_h = height / 2
    corners = [
        (center_x - half_w, center_y - half_h),  # 左下
        (center_x + half_w, center_y - half_h),  # 右下
        (center_x + half_w, center_y + half_h),  # 右上
        (center_x - half_w, center_y + half_h),  # 左上
        (center_x - half_w, center_y - half_h)   # 回到起点
    ]
    
    # 在每条边上生成插值点
    for i in range(len(corners)-1):
        start = corners[i]
        end = corners[i+1]
        for t in range(num_points_per_side):
            alpha = t / (num_points_per_side - 1)
            x = start[0] + (end[0] - start[0]) * alpha
            y = start[1] + (end[1] - start[1]) * alpha
            points.append((x, y))
    
    return points

def wait_for_position(arm, target_angles, timeout=5.0, threshold=0.1):
    """等待机械臂到达目标位置"""
    start_time = time.time()
    while time.time() - start_time < timeout:
        current_angles = arm.get_current_joints()
        if current_angles is None:
            return False
        
        # 检查所有关节是否都接近目标位置
        max_diff = max(abs(c - t) for c, t in zip(current_angles, target_angles))
        if max_diff < threshold:
            return True
        
        time.sleep(0.1)
    
    return False

def check_joint_limits(angles, joint_limits):
    """检查关节角度是否在限制范围内"""
    for angle, (min_angle, max_angle) in zip(angles, joint_limits):
        if angle < min_angle or angle > max_angle:
            return False
    return True

def main():
    # 初始化ROS2节点
    rclpy.init()
    
    arm = ArmController()
    visualizer = RectangleVisualizer()

    try:
        # 连接机械臂
        print("正在连接机械臂...")
        if not arm.connect():
            print("错误：无法连接到机械臂")
            return
        
        # 移动到初始位置
        print("\n移动到初始位置...")
        if not arm.move_to_urdf_init():
            print("错误：无法移动到初始位置")
            return
        
        # 等待到达初始位置
        print("等待到达初始位置...")
        init_angles = [0.0, 0.785398, 0.0, -0.785398, 1.5708, -0.698132]  # URDF初始位置的关节角度
        if not wait_for_position(arm, init_angles):
            print("错误：未能到达初始位置")
            return
        
        print("已到达初始位置")
        input("按Enter键开始规划矩形轨迹...")
        
        # 定义矩形参数（在机械臂基座前方的平面上）
        center_x = 0.0     # 基座中心线
        center_y = 0.10    # 基座前方10cm
        width = 0.04      # 矩形宽度4cm
        height = 0.03     # 矩形高度3cm
        z_height = 0.05   # 写字平面高度5cm
        
        # 生成矩形轨迹点
        print("\n生成矩形轨迹...")
        points = generate_rectangle_points(center_x, center_y, width, height)
        
        # 在RVIZ中可视化矩形轨迹
        print("在RVIZ中显示矩形轨迹...")
        visualizer.visualize_rectangle(points, z_height)
        
        # 等待用户确认
        input("轨迹已在RVIZ中显示，按回车键继续执行实际运动...")
        
        # 预检查所有位置
        print("\n预检查轨迹点...")
        for x, y in points:
            # 检查抬笔位置
            target_pos = np.array([x, y, z_height + 0.03])
            angles = arm.motion_controller.inverse_kinematics(target_pos)
            if angles is None:
                print(f"错误：无法求解逆运动学 - 点 ({x:.3f}, {y:.3f}, {z_height + 0.03:.3f})")
                return
            if not check_joint_limits(angles, arm.motion_controller.joint_limits):
                print(f"错误：关节角度超出限制 - 点 ({x:.3f}, {y:.3f}, {z_height + 0.03:.3f})")
                print("超出限制的关节角度：")
                for i, (angle, (min_angle, max_angle)) in enumerate(zip(angles, arm.motion_controller.joint_limits)):
                    if angle < min_angle or angle > max_angle:
                        print(f"关节 {i}: {angle:.3f} rad (限制: {min_angle:.3f} ~ {max_angle:.3f} rad)")
                return
                
            # 检查落笔位置
            target_pos = np.array([x, y, z_height])
            angles = arm.motion_controller.inverse_kinematics(target_pos)
            if angles is None:
                print(f"错误：无法求解逆运动学 - 点 ({x:.3f}, {y:.3f}, {z_height:.3f})")
                return
            if not check_joint_limits(angles, arm.motion_controller.joint_limits):
                print(f"错误：关节角度超出限制 - 点 ({x:.3f}, {y:.3f}, {z_height:.3f})")
                print("超出限制的关节角度：")
                for i, (angle, (min_angle, max_angle)) in enumerate(zip(angles, arm.motion_controller.joint_limits)):
                    if angle < min_angle or angle > max_angle:
                        print(f"关节 {i}: {angle:.3f} rad (限制: {min_angle:.3f} ~ {max_angle:.3f} rad)")
                return
        
        print("轨迹预检查通过")
        input("按Enter键移动到起始点上方...")
        
        # 移动到起始点上方
        print("\n移动到起始点上方...")
        start_x, start_y = points[0]
        target_pos = np.array([start_x, start_y, z_height + 0.03])  # 抬高3cm
        angles = arm.motion_controller.inverse_kinematics(target_pos)
        if not arm.move_joints(dict(zip(arm.motors.keys(), angles))):
            print("错误：无法移动到起始点上方")
            return
        
        # 等待到达起始点上方
        print("等待到达起始点上方...")
        if not wait_for_position(arm, angles):
            print("错误：未能到达起始点上方")
            return
        
        print("已到达起始点上方")
        input("按Enter键开始落笔...")
        
        # 落笔（降低高度）
        print("\n落笔...")
        target_pos = np.array([start_x, start_y, z_height])
        angles = arm.motion_controller.inverse_kinematics(target_pos)
        if not arm.move_joints(dict(zip(arm.motors.keys(), angles))):
            print("错误：无法落笔")
            return
        
        # 等待落笔完成
        if not wait_for_position(arm, angles):
            print("错误：落笔未完成")
            return
        
        print("已落笔")
        
        # 开始画矩形
        print("\n开始画矩形...")
        for i, (x, y) in enumerate(points):
            # 显示进度
            print(f"\n正在绘制第 {i+1}/{len(points)} 个点")
            input(f"按Enter键移动到点 ({x:.3f}, {y:.3f})...")
            
            target_pos = np.array([x, y, z_height])
            angles = arm.motion_controller.inverse_kinematics(target_pos)
            if not arm.move_joints(dict(zip(arm.motors.keys(), angles))):
                print(f"错误：无法移动到点 ({x:.3f}, {y:.3f})")
                return
            
            # 等待到达当前点
            if not wait_for_position(arm, angles):
                print(f"错误：未能到达点 ({x:.3f}, {y:.3f})")
                return
            
            print(f"已到达点 ({x:.3f}, {y:.3f})")
        
        print("\n矩形绘制完成")
        input("按Enter键抬笔...")
        
        # 抬笔
        print("\n抬笔...")
        last_x, last_y = points[-1]
        target_pos = np.array([last_x, last_y, z_height + 0.03])  # 抬高3cm
        angles = arm.motion_controller.inverse_kinematics(target_pos)
        if not arm.move_joints(dict(zip(arm.motors.keys(), angles))):
            print("错误：无法抬笔")
            return
        
        # 等待抬笔完成
        if not wait_for_position(arm, angles):
            print("错误：抬笔未完成")
            return
        
        print("抬笔完成")
        input("按Enter键返回初始位置...")
        
        # 返回初始位置
        print("\n返回初始位置...")
        if not arm.move_to_urdf_init():
            print("错误：无法返回初始位置")
            return
        
        # 等待返回初始位置
        if not wait_for_position(arm, init_angles):
            print("错误：未能返回初始位置")
            return
        
        print("\n绘制完成！")
        
    except KeyboardInterrupt:
        print("\n收到中断信号，正在停止...")
    except Exception as e:
        print(f"\n发生错误: {e}")
    finally:
        # 清理资源
        print("\n正在断开连接...")
        arm.disconnect()
        print("程序结束")

if __name__ == "__main__":
    main()
