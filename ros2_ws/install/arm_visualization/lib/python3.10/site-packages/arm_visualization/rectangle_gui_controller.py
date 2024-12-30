#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                           QHBoxLayout, QLabel, QSlider, QPushButton)
from PyQt5.QtCore import Qt
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class RectangleControlGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.width = 0.08  # 矩形宽度
        self.height = 0.08  # 矩形高度
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle('矩形轨迹控制器')
        self.setGeometry(100, 100, 400, 300)
        
        # 创建中心部件和布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # X轴控制
        x_layout = QHBoxLayout()
        x_label = QLabel('X位置:')
        self.x_slider = QSlider(Qt.Horizontal)
        self.x_slider.setMinimum(-200)
        self.x_slider.setMaximum(200)
        self.x_slider.setValue(0)
        self.x_value = QLabel('0.00')
        x_layout.addWidget(x_label)
        x_layout.addWidget(self.x_slider)
        x_layout.addWidget(self.x_value)
        layout.addLayout(x_layout)
        
        # Y轴控制
        y_layout = QHBoxLayout()
        y_label = QLabel('Y位置:')
        self.y_slider = QSlider(Qt.Horizontal)
        self.y_slider.setMinimum(-800)  # 调整最小值以允许更大的负值
        self.y_slider.setMaximum(0)
        self.y_slider.setValue(-600)  # 设置默认值为-0.6
        self.y_value = QLabel('-0.60')
        y_layout.addWidget(y_label)
        y_layout.addWidget(self.y_slider)
        y_layout.addWidget(self.y_value)
        layout.addLayout(y_layout)
        
        # Z轴控制
        z_layout = QHBoxLayout()
        z_label = QLabel('Z位置:')
        self.z_slider = QSlider(Qt.Horizontal)
        self.z_slider.setMinimum(0)
        self.z_slider.setMaximum(200)
        self.z_slider.setValue(50)  # 默认值0.05
        self.z_value = QLabel('0.05')
        z_layout.addWidget(z_label)
        z_layout.addWidget(self.z_slider)
        z_layout.addWidget(self.z_value)
        layout.addLayout(z_layout)
        
        # 保存按钮
        save_btn = QPushButton('保存位置')
        layout.addWidget(save_btn)
        
        # 连接信号和槽
        self.x_slider.valueChanged.connect(self.update_position)
        self.y_slider.valueChanged.connect(self.update_position)
        self.z_slider.valueChanged.connect(self.update_position)
        save_btn.clicked.connect(self.save_position)
        
        self.update_position()  # 初始化位置
        
    def update_position(self):
        # 更新标签显示
        x = self.x_slider.value() / 1000.0
        y = self.y_slider.value() / 1000.0
        z = self.z_slider.value() / 1000.0
        
        self.x_value.setText(f'{x:.2f}')
        self.y_value.setText(f'{y:.2f}')
        self.z_value.setText(f'{z:.2f}')
        
        # 更新可视化标记
        self.publish_rectangle(x, y, z)
        
    def publish_rectangle(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.001  # 线宽
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # 红色
        
        # 定义矩形的四个角点
        points = [
            Point(x=x, y=y, z=z),  # 左下角
            Point(x=x + self.width, y=y, z=z),  # 右下角
            Point(x=x + self.width, y=y + self.width, z=z),  # 右上角
            Point(x=x, y=y + self.width, z=z),  # 左上角
            Point(x=x, y=y, z=z)  # 回到起点
        ]
        marker.points = points
        
        self.node.marker_pub.publish(marker)
        
    def save_position(self):
        x = self.x_slider.value() / 1000.0
        y = self.y_slider.value() / 1000.0
        z = self.z_slider.value() / 1000.0
        
        # 保存位置到文件
        with open('/home/ubuntu22/so_arm100_write/ros2_ws/src/arm_visualization/rectangle_position.txt', 'w') as f:
            f.write(f'x: {x}\ny: {y}\nz: {z}')
        self.node.get_logger().info(f'Position saved: x={x}, y={y}, z={z}')

class RectangleControlNode(Node):
    def __init__(self):
        super().__init__('rectangle_control_gui')
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

def main(args=None):
    rclpy.init(args=args)
    node = RectangleControlNode()
    
    app = QApplication(sys.argv)
    gui = RectangleControlGUI(node)
    gui.show()
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        app.processEvents()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
