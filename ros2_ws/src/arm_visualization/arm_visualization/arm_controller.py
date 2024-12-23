#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from .motors.feetech import FeetechMotorsBus
import numpy as np
import time
from typing import List, Dict, Tuple, Optional
from .trajectory_generator import StrokePoint
from .motion_controller import MotionController

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
            "shoulder_pan_joint": (-np.pi, np.pi),
            "shoulder_lift_joint": (-np.pi/2, np.pi/2),
            "elbow_joint": (-np.pi/2, np.pi/2),
            "wrist_pitch_joint": (-np.pi/2, np.pi/2),
            "wrist_roll_joint": (-np.pi, np.pi),
            "jaw_joint": (-np.pi/4, np.pi/4)
        }
        
        # 运动速度参数
        self.max_speed = 200   # 最大速度限制
        
        # 初始化运动控制器
        self.motion_controller = MotionController()
        
        # 初始化总线连接
        self.bus = None

        # 当前关节角度
        self.current_joints = [0.0] * 6

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
            self.bus.write("P_Coefficient", [32] * len(self.motors))
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

    def get_current_joints(self) -> Optional[List[float]]:
        """获取当前关节角度（弧度）"""
        try:
            if not self.bus:
                return None
            
            # 读取当前位置
            positions = []
            for joint_name in self.motors.keys():
                pos = self.bus.read("Present_Position", joint_name)
                if pos is None:
                    return None
                positions.append(pos)
            
            # 转换为弧度
            current_angles = [self.steps_to_rad(pos) for pos in positions]
            return current_angles
            
        except Exception as e:
            print(f"Error getting current joints: {e}")
            return None

    def get_joint_angles(self, target_position):
        """计算到达目标位置所需的关节角度"""
        try:
            # 确保目标位置是正确的格式
            target_position = np.array(target_position)
            
            # 如果包含姿态信息，将其转换为变换矩阵
            if len(target_position) == 6:  # [x, y, z, roll, pitch, yaw]
                x, y, z = target_position[:3]
                roll, pitch, yaw = target_position[3:]
                
                # 创建旋转矩阵
                Rx = np.array([[1, 0, 0],
                             [0, np.cos(roll), -np.sin(roll)],
                             [0, np.sin(roll), np.cos(roll)]])
                
                Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                             [0, 1, 0],
                             [-np.sin(pitch), 0, np.cos(pitch)]])
                
                Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                             [np.sin(yaw), np.cos(yaw), 0],
                             [0, 0, 1]])
                
                R = Rz @ Ry @ Rx
                target_orientation = R
            else:  # 只有位置信息 [x, y, z]
                x, y, z = target_position
                # 默认姿态：笔尖朝下
                target_orientation = np.array([
                    [1, 0, 0],
                    [0, 0, -1],
                    [0, 1, 0]
                ])
            
            # 使用位置和姿态进行逆运动学计算
            initial_position = [0.0] * 8  # 从零位置开始计算
            
            # 创建目标变换矩阵
            target_matrix = np.eye(4)
            target_matrix[:3, :3] = target_orientation
            target_matrix[:3, 3] = [x, y, z]
            
            # 使用 IKPy 的 inverse_kinematics 方法
            joint_angles = self.motion_controller.chain.inverse_kinematics_frame(
                target_matrix,
                initial_position=initial_position,
                orientation_mode="all",
                max_iter=100
            )
            
            # 只返回活动关节的角度（去掉原点链接和笔尖固定关节）
            active_joint_angles = [joint_angles[i] for i in range(1, 7)]
            
            # 检查关节限制
            if self.check_joint_limits(active_joint_angles):
                return active_joint_angles
            else:
                print(f"警告：计算出的关节角度超出限制：{active_joint_angles}")
                return None
                
        except Exception as e:
            print(f"计算逆运动学时出错：{e}")
            import traceback
            traceback.print_exc()
            return None

    def check_joint_limits(self, joint_angles):
        """检查关节角度是否在限制范围内"""
        joint_limits = [
            [-2.0, 2.0],       # shoulder_pan_joint  (±120度)
            [-1.57, 1.57],     # shoulder_lift_joint (±90度)
            [-1.57, 1.57],     # elbow_joint        (±90度)
            [-1.57, 1.57],     # wrist_pitch_joint  (±90度)
            [-3.14, 3.14],     # wrist_roll_joint   (±180度)
            [-0.78, 0.78],     # jaw_joint          (±45度)
        ]
        
        for angle, limits in zip(joint_angles, joint_limits):
            if angle < limits[0] or angle > limits[1]:
                return False
        return True

    def move_to_joint_angles(self, target_angles: List[float]) -> bool:
        """移动到指定的关节角度（弧度）"""
        try:
            if not self.bus:
                return False
                
            # 检查输入参数
            if len(target_angles) != len(self.motors):
                print(f"Expected {len(self.motors)} angles, got {len(target_angles)}")
                return False
            
            # 检查关节限制
            if not self.check_joint_limits(target_angles):
                print(f"警告：目标关节角度超出限制：{target_angles}")
                return False
            
            # 将目标角度转换为电机步数
            target_positions = [self.rad_to_steps(angle) for angle in target_angles]
            
            # 获取当前位置
            current_positions = []
            for joint_name in self.motors.keys():
                pos = self.bus.read("Present_Position", joint_name)
                if pos is None:
                    return False
                current_positions.append(pos)
            
            # 生成平滑路径
            path_points = self.interpolate_path(current_positions, target_positions)
            
            # 执行运动
            joint_names = list(self.motors.keys())
            for positions in path_points:
                for i, joint_name in enumerate(joint_names):
                    self.bus.write("Goal_Position", positions[i], joint_name)
                time.sleep(0.02)  # 50Hz控制频率
            
            # 更新当前关节角度
            self.current_joints = target_angles
            
            return True
            
        except Exception as e:
            print(f"Error moving to joint angles: {e}")
            return False

    def move_to_pose(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> bool:
        """移动末端执行器到指定位姿"""
        try:
            # 使用运动控制器计算逆运动学
            target_angles = self.get_joint_angles([x, y, z, roll, pitch, yaw])
            if target_angles is None:
                print("无法计算逆运动学解")
                return False
            
            # 移动到目标关节角度
            return self.move_to_joint_angles(target_angles)
            
        except Exception as e:
            print(f"Error moving to pose: {e}")
            return False

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

    def move_to_zero(self) -> bool:
        """将机械臂移动到电机零位（所有关节角度为0）"""
        if not self.bus:
            print("Not connected to the arm")
            return False
        
        try:
            print("Moving to motor zero position...")
            # 获取当前位置
            current_positions = []
            for joint_name in self.motors.keys():
                pos = self.bus.read("Present_Position", joint_name)
                if pos is None:
                    return False
                current_positions.append(pos)
            
            # 目标位置：所有电机的零位是2048
            target_positions = [2048] * len(self.motors)
            
            # 生成平滑路径
            path = self.interpolate_path(current_positions, target_positions)
            
            # 执行平滑运动
            joint_names = list(self.motors.keys())
            for positions in path:
                for i, joint_name in enumerate(joint_names):
                    self.bus.write("Goal_Position", positions[i], joint_name)
                time.sleep(0.02)  # 50Hz的更新频率
            
            time.sleep(0.5)  # 最后等待稳定
            return True
            
        except Exception as e:
            print(f"Move to zero error: {e}")
            return False

    def move_to_urdf_init(self) -> bool:
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
            current_positions = []
            for joint_name in self.motors.keys():
                pos = self.bus.read("Present_Position", joint_name)
                if pos is None:
                    return False
                current_positions.append(pos)
            
            # 计算目标位置
            target_positions = []
            for joint_name in self.motors.keys():
                angle = init_angles[joint_name]
                steps = self.rad_to_steps(angle)
                target_positions.append(steps)
            
            # 生成平滑路径
            path = self.interpolate_path(current_positions, target_positions)
            
            # 执行平滑运动
            joint_names = list(self.motors.keys())
            for positions in path:
                for i, joint_name in enumerate(joint_names):
                    self.bus.write("Goal_Position", positions[i], joint_name)
                time.sleep(0.02)  # 50Hz的更新频率
            
            time.sleep(0.5)  # 最后等待稳定
            return True
            
        except Exception as e:
            print(f"Move to URDF init error: {e}")
            return False
