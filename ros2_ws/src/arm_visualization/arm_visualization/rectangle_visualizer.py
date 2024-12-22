#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from .arm_controller import ArmController
import numpy as np
import time

class RectangleVisualizer(Node):
    def __init__(self):
        super().__init__('rectangle_visualization')
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.arm = ArmController()
        
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
            
        # 闭合路径
        if len(points) > 0:
            point = Point()
            point.x = points[0][0]
            point.y = points[0][1]
            point.z = z_height
            marker.points.append(point)
            
        self.marker_pub.publish(marker)

    def wait_for_position(self, target_angles, timeout=5.0, threshold=0.1):
        """等待机械臂到达目标位置"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_angles = self.arm.get_current_joints()
            if current_angles is None:
                return False
            
            # 检查所有关节是否都接近目标位置
            max_diff = max(abs(c - t) for c, t in zip(current_angles, target_angles))
            if max_diff < threshold:
                return True
            
            time.sleep(0.1)
        
        return False

    def move_rectangle(self, points, z_height, pen_up_height):
        """执行矩形轨迹运动"""
        try:
            # 连接机械臂
            if not self.arm.connect():
                print("无法连接到机械臂，请检查连接。")
                return False
                
            print("\n开始执行运动...")
            
            # 移动到初始位置（抬笔高度）
            print("1. 移动到初始位置...")
            initial_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.arm.move_to_joint_angles(initial_joint_angles)
            if not self.wait_for_position(initial_joint_angles):
                print("移动到初始位置失败")
                return False
            
            # 移动到起点上方
            print("2. 移动到起点上方...")
            start_point = points[0]
            self.arm.move_to_pose(start_point[0], start_point[1], pen_up_height, 0, 180, 0)
            time.sleep(2.0)
            
            # 落笔
            print("3. 落笔...")
            self.arm.move_to_pose(start_point[0], start_point[1], z_height, 0, 180, 0)
            time.sleep(1.0)
            
            # 执行轨迹
            print("4. 执行轨迹...")
            for i, (x, y) in enumerate(points):
                print(f"   执行第 {i+1}/{len(points)} 个点")
                self.arm.move_to_pose(x, y, z_height, 0, 180, 0)
                time.sleep(0.5)
            
            # 回到起点（闭合矩形）
            print("5. 闭合轨迹...")
            self.arm.move_to_pose(points[0][0], points[0][1], z_height, 0, 180, 0)
            time.sleep(1.0)
            
            # 抬笔
            print("6. 抬笔...")
            self.arm.move_to_pose(points[0][0], points[0][1], pen_up_height, 0, 180, 0)
            time.sleep(1.0)
            
            # 回到初始位置
            print("7. 返回初始位置...")
            self.arm.move_to_joint_angles(initial_joint_angles)
            if not self.wait_for_position(initial_joint_angles):
                print("返回初始位置失败")
                return False
            
            print("运动执行完成！")
            return True
            
        except Exception as e:
            print(f'执行运动时出错: {str(e)}')
            return False
        finally:
            # 断开连接
            self.arm.disconnect()

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
    ]
    
    # 在每条边上生成插值点
    for i in range(len(corners)):
        start = corners[i]
        end = corners[(i + 1) % len(corners)]
        for t in range(num_points_per_side):
            alpha = t / (num_points_per_side - 1)
            x = start[0] + (end[0] - start[0]) * alpha
            y = start[1] + (end[1] - start[1]) * alpha
            points.append((x, y))
    
    return points

def main():
    print("\n=== 矩形轨迹可视化程序 ===")
    print("请确保已经完成以下步骤：")
    print("1. 在另一个终端运行：ros2 launch arm_description display.launch.py")
    print("2. 在RVIZ中添加Marker显示：")
    print("   - 点击 'Add' 按钮")
    print("   - 选择 'By topic'")
    print("   - 等待 '/visualization_marker' 出现（大约需要3-5秒）")
    print("   - 选择 '/visualization_marker'")
    print("   - 点击 'OK' 确认\n")
    
    rclpy.init()
    visualizer = RectangleVisualizer()
    
    # 设置矩形参数（根据机器人工作区域调整）
    center_x = 0.25  # 矩形中心x坐标，距离基座25cm
    center_y = 0.0   # 矩形中心y坐标，在基座正前方
    width = 0.1      # 矩形宽度10cm
    height = 0.1     # 矩形高度10cm
    z_height = 0.03  # 写字高度3cm
    pen_up_height = 0.05  # 抬笔高度5cm
    
    # 生成矩形路径点
    points = generate_rectangle_points(center_x, center_y, width, height)
    
    try:
        print("\n正在发布轨迹可视化消息...")
        # 持续发布一段时间以确保RVIZ能够接收到消息
        start_time = time.time()
        while time.time() - start_time < 5.0:  # 发布5秒
            visualizer.visualize_rectangle(points, z_height)
            time.sleep(0.1)
            rclpy.spin_once(visualizer, timeout_sec=0)
        
        input("\n已持续发布5秒消息，现在应该可以在RVIZ的 'By topic' 中看到 '/visualization_marker'。\n完成Marker添加后，按回车键继续...")
        
        print("\n继续发布轨迹可视化消息...")
        # 再次持续发布以显示轨迹
        for _ in range(10):
            visualizer.visualize_rectangle(points, z_height)
            time.sleep(0.1)
            rclpy.spin_once(visualizer, timeout_sec=0)
        
        # 等待用户确认
        input("\n轨迹已在RVIZ中显示（红色线条），按回车键开始执行实际运动...")
        
        # 执行实际运动
        if visualizer.move_rectangle(points, z_height, pen_up_height):
            print("矩形轨迹执行完成！")
        else:
            print("执行失败，请检查机器人状态。")
            
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
