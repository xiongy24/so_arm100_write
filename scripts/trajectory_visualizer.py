#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from trajectory_generator import TrajectoryGenerator, StrokePoint
from typing import List

class TrajectoryVisualizer:
    def __init__(self):
        self.trajectory_generator = TrajectoryGenerator()
        
    def plot_stroke(self, ax: plt.Axes, stroke: List[StrokePoint], color: str = 'blue'):
        """绘制单个笔画的轨迹
        
        Args:
            ax: matplotlib轴对象
            stroke: 笔画点列表
            color: 轨迹颜色
        """
        # 提取坐标
        x_coords = [point.x for point in stroke]
        y_coords = [point.y for point in stroke]
        z_coords = [point.z for point in stroke]
        
        # 绘制3D轨迹
        ax.plot(x_coords, y_coords, z_coords, color=color, linewidth=2)
        
        # 标记起点和终点
        ax.scatter(x_coords[0], y_coords[0], z_coords[0], color='green', marker='o', label='Start')
        ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], color='red', marker='o', label='End')

    def plot_character(self, strokes: List[List[StrokePoint]], title: str):
        """绘制单个汉字的所有笔画
        
        Args:
            strokes: 笔画列表的列表
            title: 图表标题
        """
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制每个笔画
        colors = plt.cm.rainbow(np.linspace(0, 1, len(strokes)))
        for stroke, color in zip(strokes, colors):
            self.plot_stroke(ax, stroke, color)
        
        # 设置图表属性
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title(title)
        
        # 添加图例
        ax.legend(['Stroke {}'.format(i+1) for i in range(len(strokes))])
        
        # 设置视角
        ax.view_init(elev=30, azim=45)
        
        return fig, ax

    def plot_writing_area(self, ax: plt.Axes):
        """绘制写字区域"""
        center = self.trajectory_generator.writing_area_center
        size = self.trajectory_generator.char_size * 2
        
        # 创建写字平面的四个角点
        corners = np.array([
            [center[0] - size/2, center[1] - size/2, center[2]],
            [center[0] + size/2, center[1] - size/2, center[2]],
            [center[0] + size/2, center[1] + size/2, center[2]],
            [center[0] - size/2, center[1] + size/2, center[2]],
            [center[0] - size/2, center[1] - size/2, center[2]]  # 闭合多边形
        ])
        
        # 绘制写字平面边界
        ax.plot(corners[:, 0], corners[:, 1], corners[:, 2], 'k--', alpha=0.5, label='Writing Area')
        
        # 添加中心点标记
        ax.scatter(center[0], center[1], center[2], color='black', marker='+', s=100, label='Center')

    def visualize_full_trajectory(self):
        """可视化完整的"二四"轨迹"""
        # 生成完整轨迹
        trajectory = self.trajectory_generator.generate_full_trajectory()
        
        # 创建图表
        fig = plt.figure(figsize=(15, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制写字区域
        self.plot_writing_area(ax)
        
        # 计算"二"和"四"的笔画数量
        er_strokes = len(self.trajectory_generator.generate_er_strokes())
        
        # 使用不同的颜色绘制"二"和"四"
        colors_er = plt.cm.autumn(np.linspace(0, 1, er_strokes))
        colors_si = plt.cm.winter(np.linspace(0, 1, len(trajectory) - er_strokes))
        colors = np.vstack((colors_er, colors_si))
        
        # 绘制所有笔画
        for stroke, color in zip(trajectory, colors):
            self.plot_stroke(ax, stroke, color)
        
        # 设置图表属性
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title('Trajectory for Writing "Er" and "Si"')
        
        # 添加图例
        stroke_labels = ([f'Er-Stroke{i+1}' for i in range(er_strokes)] + 
                        [f'Si-Stroke{i+1}' for i in range(len(trajectory) - er_strokes)])
        ax.legend(stroke_labels, bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # 设置视角
        ax.view_init(elev=30, azim=45)
        
        # 调整图表布局
        plt.tight_layout()
        
        return fig, ax

def main():
    """主函数，用于测试可视化功能"""
    visualizer = TrajectoryVisualizer()
    
    # 可视化完整轨迹
    fig, ax = visualizer.visualize_full_trajectory()
    
    # 显示图表
    plt.show()

if __name__ == '__main__':
    main()
