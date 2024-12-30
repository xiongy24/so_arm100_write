#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from scipy.interpolate import CubicSpline
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class StrokePoint:
    """笔画点的数据类"""
    x: float  # X坐标
    y: float  # Y坐标
    z: float  # Z坐标（笔尖高度）
    speed: float  # 该点处的期望速度

class TrajectoryGenerator:
    def __init__(self):
        # 基本参数设置
        self.writing_speed = 50.0  # 书写速度，单位：mm/s
        self.transition_speed = 100.0  # 过渡移动速度
        self.pen_down_height = 0.0  # 落笔高度
        self.pen_up_height = 10.0  # 抬笔高度
        self.transition_height = 20.0  # 过渡移动高度
        
        # 写字区域参数（单位：毫米）
        self.writing_area = {
            'center_x': 150,    # 减小距离，避免关节角度过大
            'center_y': 0,
            'width': 100,      # 减小写字区域大小
            'height': 100,
            'z_surface': 30,   # 降低写字平面高度
            'z_up': 50,        # 降低抬笔高度
        }
        
        # 当前字的位置偏移
        self.current_char_offset = np.array([0.0, 0.0, 0.0])

    def set_current_char_position(self, char_index: int):
        """设置当前要写的字的位置"""
        # 目前支持两个字的位置
        if char_index == 0:  # "二"
            self.current_char_offset = np.array([-self.writing_area['width']/2, 0, 0])
        elif char_index == 1:  # "四"
            self.current_char_offset = np.array([self.writing_area['width']/2, 0, 0])

    def generate_stroke_points(self, stroke_type: str, start_point: Tuple[float, float], 
                             end_point: Tuple[float, float], control_points: Optional[List[Tuple[float, float]]] = None) -> List[StrokePoint]:
        """生成单个笔画的轨迹点
        
        Args:
            stroke_type: 笔画类型（横、竖、撇、点等）
            start_point: 起始点坐标 (x, y)
            end_point: 结束点坐标 (x, y)
            control_points: 控制点列表，用于非直线笔画
        
        Returns:
            List[StrokePoint]: 笔画轨迹点列表
        """
        points = []
        
        # 添加抬笔到起始点的轨迹
        start_transition = StrokePoint(
            x=start_point[0],
            y=start_point[1],
            z=self.writing_area['z_up'],
            speed=self.transition_speed
        )
        points.append(start_transition)
        
        # 添加落笔点
        start_down = StrokePoint(
            x=start_point[0],
            y=start_point[1],
            z=self.writing_area['z_surface'],
            speed=self.writing_speed
        )
        points.append(start_down)
        
        # 根据笔画类型生成中间点
        if control_points:
            # 使用控制点生成样条曲线
            t = np.linspace(0, 1, len(control_points) + 2)
            x_points = [start_point[0]] + [p[0] for p in control_points] + [end_point[0]]
            y_points = [start_point[1]] + [p[1] for p in control_points] + [end_point[1]]
            
            cs_x = CubicSpline(t, x_points)
            cs_y = CubicSpline(t, y_points)
            
            # 生成更密集的点
            t_dense = np.linspace(0, 1, 50)
            for t_val in t_dense[1:-1]:
                points.append(StrokePoint(
                    x=float(cs_x(t_val)),
                    y=float(cs_y(t_val)),
                    z=self.writing_area['z_surface'],
                    speed=self.writing_speed
                ))
        else:
            # 直线笔画，只需要终点
            points.append(StrokePoint(
                x=end_point[0],
                y=end_point[1],
                z=self.writing_area['z_surface'],
                speed=self.writing_speed
            ))
        
        # 添加抬笔点
        end_up = StrokePoint(
            x=end_point[0],
            y=end_point[1],
            z=self.writing_area['z_up'],
            speed=self.transition_speed
        )
        points.append(end_up)
        
        # 转换到实际坐标系
        for point in points:
            point.x += self.writing_area['center_x'] + self.current_char_offset[0]
            point.y += self.writing_area['center_y'] + self.current_char_offset[1]
            point.z += self.writing_area['z_surface'] + self.current_char_offset[2]
        
        return points

    def generate_er_strokes(self) -> List[List[StrokePoint]]:
        """生成"二"字的所有笔画轨迹"""
        strokes = []
        char_size = self.writing_area['width']
        
        # 设置当前字的位置
        self.set_current_char_position(0)
        
        # 笔画1：第一横
        strokes.append(self.generate_stroke_points(
            "横",
            (-0.3*char_size, 0.2*char_size),
            (0.3*char_size, 0.2*char_size)
        ))
        
        # 笔画2：第二横
        strokes.append(self.generate_stroke_points(
            "横",
            (-0.3*char_size, -0.2*char_size),
            (0.3*char_size, -0.2*char_size)
        ))
        
        return strokes

    def generate_si_strokes(self) -> List[List[StrokePoint]]:
        """生成"四"字的所有笔画轨迹"""
        strokes = []
        char_size = self.writing_area['width']
        
        # 设置当前字的位置
        self.set_current_char_position(1)
        
        # 笔画1：横
        strokes.append(self.generate_stroke_points(
            "横",
            (-0.3*char_size, 0.3*char_size),
            (0.3*char_size, 0.3*char_size)
        ))
        
        # 笔画2：竖
        strokes.append(self.generate_stroke_points(
            "竖",
            (0*char_size, 0.4*char_size),
            (0*char_size, -0.4*char_size)
        ))
        
        # 笔画3：左撇
        strokes.append(self.generate_stroke_points(
            "撇",
            (-0.2*char_size, 0.1*char_size),
            (-0.3*char_size, -0.2*char_size)
        ))
        
        # 笔画4：右捺
        strokes.append(self.generate_stroke_points(
            "捺",
            (0.2*char_size, 0.1*char_size),
            (0.3*char_size, -0.2*char_size)
        ))
        
        # 笔画5：横（底）
        strokes.append(self.generate_stroke_points(
            "横",
            (-0.3*char_size, -0.3*char_size),
            (0.3*char_size, -0.3*char_size)
        ))
        
        return strokes

    def generate_full_trajectory(self) -> List[List[StrokePoint]]:
        """生成完整的"二四"轨迹"""
        trajectory = []
        trajectory.extend(self.generate_er_strokes())
        trajectory.extend(self.generate_si_strokes())
        return trajectory
