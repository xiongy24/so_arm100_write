#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from motors.feetech import FeetechMotorsBus
import numpy as np
import time
from typing import List, Dict, Tuple, Optional
from trajectory_generator import StrokePoint
from motion_controller import MotionController

class ArmController:
    def __init__(self):
        # 定义电机配置
        self.motors = {
            "shoulder_pan_joint": (1, "sts3215"),
            "shoulder_lift_joint": (2, "sts3215"),
            "elbow_joint": (3, "sts3215"),
            "wrist_pitch_joint": (4, "sts3215"),
            "wrist_roll_joint": (5, "sts3215"),
            "jaw_joint": (6, "sts3215")
        }
        
        # 关节限制（弧度）
        self.joint_limits = {
            "shoulder_pan_joint": (-1.57, 1.57),
            "shoulder_lift_joint": (-1.57, 1.57),
            "elbow_joint": (-1.57, 1.57),
            "wrist_pitch_joint": (-1.57, 1.57),
            "wrist_roll_joint": (-1.57, 1.57),
            "jaw_joint": (-0.7, 0.7)
        }
        
        # 运动速度参数
        self.max_speed = 200   # 最大速度限制
        
        # 初始化运动控制器
        self.motion_controller = MotionController()
        
        # 初始化总线连接
        self.bus = None

    def rad_to_steps(self, rad: float) -> int:
        """将弧度转换为电机步数"""
        degree = rad * 180.0 / np.pi
        steps = int(2048 + (degree * 4096 / 360.0))
        return steps

    def steps_to_rad(self, steps: int) -> float:
        """将电机步数转换为弧度"""
        degree = (steps - 2048) * 360.0 / 4096
        rad = degree * np.pi / 180.0
        return rad

    def connect(self, port: str = "/dev/ttyACM0"):
        """连接到机械臂"""
        try:
            print("Connecting to motors bus...")
            self.bus = FeetechMotorsBus(port=port, motors=self.motors)
            self.bus.connect()
            time.sleep(1)  # 等待连接稳定
            
            # 启用扭矩
            print("Enabling torque...")
            self.bus.write("Torque_Enable", [1] * len(self.motors))
            time.sleep(0.5)
            
            # 设置P系数（控制精度和响应速度）
            print("Setting P coefficient...")
            self.bus.write("P_Coefficient", [8] * len(self.motors))  # 使用较小的P系数使运动更平滑
            time.sleep(0.5)
            
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def disconnect(self):
        """断开与机械臂的连接"""
        if self.bus:
            try:
                # 禁用扭矩
                print("Disabling torque...")
                self.bus.write("Torque_Enable", [0] * len(self.motors))
                time.sleep(0.5)
                
                # 断开连接
                print("Disconnecting...")
                self.bus.disconnect()
                print("Disconnected successfully")
            except Exception as e:
                print(f"Disconnection error: {e}")

    def interpolate_path(self, start_positions, end_positions, steps=50):
        """在起始位置和目标位置之间进行线性插值，生成平滑路径"""
        paths = []
        for i in range(steps):
            t = i / (steps - 1)  # 时间参数，从0到1
            current_pos = []
            for start, end in zip(start_positions, end_positions):
                pos = start + (end - start) * t
                current_pos.append(int(pos))
            paths.append(current_pos)
        return paths

    def move_to_zero(self):
        """将机械臂移动到零位"""
        if not self.bus:
            print("Not connected to the arm")
            return False
        
        try:
            print("Moving to motor zero position...")
            # 获取当前位置
            current_positions = self.bus.read("Present_Position")
            target_positions = [2048] * len(self.motors)
            
            # 生成平滑路径
            path = self.interpolate_path(current_positions, target_positions)
            
            # 执行平滑运动
            for positions in path:
                self.bus.write("Goal_Position", positions)
                time.sleep(0.02)  # 50Hz的更新频率
            
            time.sleep(0.5)  # 最后等待稳定
            return True
        except Exception as e:
            print(f"Move to zero error: {e}")
            return False

    def move_to_urdf_init(self):
        """移动到URDF文件中定义的初始位置"""
        if not self.bus:
            print("Not connected to the arm")
            return False
        
        try:
            print("Moving to URDF initial position...")
            # 计算每个关节的目标位置（弧度）
            init_angles = {
                "shoulder_pan_joint": 0.0,           # 0度
                "shoulder_lift_joint": 0.785398,     # 45度
                "elbow_joint": 0.0,                 # 0度
                "wrist_pitch_joint": -0.785398,     # -45度
                "wrist_roll_joint": 1.5708,         # 90度
                "jaw_joint": -0.698132              # -40度
            }
            
            # 获取当前位置
            current_positions = self.bus.read("Present_Position")
            
            # 计算目标位置
            target_positions = []
            for joint_name in self.motors.keys():
                angle = init_angles[joint_name]
                steps = self.rad_to_steps(angle)
                target_positions.append(steps)
            
            # 生成平滑路径
            path = self.interpolate_path(current_positions, target_positions)
            
            # 执行平滑运动
            for positions in path:
                self.bus.write("Goal_Position", positions)
                time.sleep(0.02)  # 50Hz的更新频率
            
            time.sleep(0.5)  # 最后等待稳定
            return True
            
        except Exception as e:
            print(f"Move to URDF init error: {e}")
            return False

    def move_joints(self, joint_angles):
        """移动机械臂到指定的关节角度"""
        if not self.bus:
            print("Not connected to the arm")
            return False
        
        try:
            # 获取当前位置
            current_positions = self.bus.read("Present_Position")
            
            # 计算目标位置
            target_positions = []
            for joint_name, angle in joint_angles.items():
                steps = self.rad_to_steps(angle)
                target_positions.append(steps)
            
            # 生成平滑路径
            path = self.interpolate_path(current_positions, target_positions)
            
            # 执行平滑运动
            for positions in path:
                self.bus.write("Goal_Position", positions)
                time.sleep(0.02)  # 50Hz的更新频率
            
            time.sleep(0.5)  # 最后等待稳定
            return True
        except Exception as e:
            print(f"Move joints error: {e}")
            return False

    def get_current_joints(self) -> Optional[List[float]]:
        """获取当前关节角度（弧度）"""
        if not self.bus:
            print("Not connected to the arm")
            return None
        
        try:
            positions = self.bus.read("Present_Position")
            return [self.steps_to_rad(pos) for pos in positions]
        except Exception as e:
            print(f"Get current joints error: {e}")
            return None

    def execute_trajectory(self, trajectory: List[List[float]], dt: float = 0.01) -> bool:
        """执行轨迹
        
        Args:
            trajectory: 关节角度序列
            dt: 时间步长
        """
        if not self.bus:
            print("Not connected to the arm")
            return False
        
        try:
            for joint_angles in trajectory:
                if not self.move_joints({joint_name: angle for joint_name, angle in zip(self.motors.keys(), joint_angles)}):
                    return False
                time.sleep(dt)
            return True
        except Exception as e:
            print(f"Execute trajectory error: {e}")
            return False

    def write_stroke(self, stroke_points: List[StrokePoint]) -> bool:
        """执行单个笔画
        
        Args:
            stroke_points: 笔画轨迹点列表
        """
        try:
            # 规划轨迹
            trajectory = self.motion_controller.plan_trajectory(stroke_points)
            
            # 插值得到更密集的轨迹点
            dense_trajectory = self.motion_controller.interpolate_trajectory(trajectory)
            
            # 执行轨迹
            return self.execute_trajectory(dense_trajectory)
        except Exception as e:
            print(f"Write stroke error: {e}")
            return False
