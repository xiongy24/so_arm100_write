#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from typing import List, Tuple, Optional
from .trajectory_generator import StrokePoint
import math

class MotionController:
    def __init__(self):
        # DH参数（根据URDF文件中的信息）
        self.dh_params = [
            # [a, alpha, d, theta]
            [0, np.pi/2, 0.0305, 0],     # Joint 1: shoulder_pan_joint
            [0, 0, 0, 0],                # Joint 2: shoulder_lift_joint
            [0.0317, 0, 0.10423, 0],     # Joint 3: elbow_joint
            [0, np.pi/2, 0.09070, 0],    # Joint 4: wrist_pitch_joint
            [0, -np.pi/2, 0.0586, 0],    # Joint 5: wrist_roll_joint
            [0, 0, 0.0259, 0],           # Joint 6: jaw_joint
        ]
        
        # 关节限制
        self.joint_limits = [
            [-1.57, 1.57],  # shoulder_pan_joint
            [-1.57, 1.57],  # shoulder_lift_joint
            [-1.57, 1.57],  # elbow_joint
            [-1.57, 1.57],  # wrist_pitch_joint
            [-1.57, 1.57],  # wrist_roll_joint
            [-0.7, 0.7],    # jaw_joint
        ]
        
        # 当前关节角度
        self.current_joints = [0.0] * 6
        
        # 笔尖相对于末端执行器的偏移
        self.pen_offset = np.array([0.0, 0.0, 0.040525])  # 简化笔尖偏移，只保留Z轴偏移

    def transform_matrix(self, a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        """计算DH参数对应的变换矩阵"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_angles: List[float]) -> np.ndarray:
        """正向运动学：计算给定关节角度下的末端执行器位姿"""
        T = np.eye(4)
        
        for i, theta in enumerate(joint_angles):
            a, alpha, d, _ = self.dh_params[i]
            T = T @ self.transform_matrix(a, alpha, d, theta)
        
        # 添加笔尖偏移
        T[0:3, 3] += T[0:3, 0:3] @ self.pen_offset
        
        return T

    def inverse_kinematics(self, target_pos: np.ndarray, current_joints: List[float] = None) -> List[float]:
        """逆运动学：计算达到目标位置所需的关节角度
        
        使用数值迭代法（雅可比矩阵）求解逆运动学，并添加关节优先级和姿态约束
        """
        if current_joints is None:
            current_joints = self.current_joints.copy()
        
        max_iterations = 100
        epsilon = 1e-3
        damping = 0.5  # 阻尼因子，用于避免奇异点
        
        # 设置目标姿态：保持笔尖垂直
        target_orientation = np.array([0, 0, -1])  # 笔尖朝下
        
        for _ in range(max_iterations):
            # 计算当前位置和姿态
            current_transform = self.forward_kinematics(current_joints)
            current_pos = current_transform[0:3, 3]
            current_orientation = current_transform[0:3, 2]  # z轴方向
            
            # 计算位置和姿态误差
            pos_error = target_pos - current_pos
            orientation_error = np.cross(current_orientation, target_orientation)
            
            # 组合误差向量
            error = np.concatenate([pos_error, 0.5 * orientation_error])
            
            if np.linalg.norm(error) < epsilon:
                break
            
            # 计算雅可比矩阵
            J = np.zeros((6, 6))
            delta = 1e-6
            
            for i in range(6):
                joints_plus = current_joints.copy()
                joints_plus[i] += delta
                transform_plus = self.forward_kinematics(joints_plus)
                pos_plus = transform_plus[0:3, 3]
                orientation_plus = transform_plus[0:3, 2]
                
                # 位置雅可比矩阵
                J[0:3, i] = (pos_plus - current_pos) / delta
                # 姿态雅可比矩阵
                J[3:6, i] = np.cross(current_orientation, orientation_plus - current_orientation) / delta
            
            # 使用阻尼最小二乘法求解
            J_T = J.T
            lambda_square = damping * damping
            delta_theta = J_T @ np.linalg.inv(J @ J_T + lambda_square * np.eye(6)) @ error
            
            # 更新关节角度，使用不同的权重
            weights = [1.0, 1.0, 0.8, 0.6, 0.4, 0.2]  # 调整关节权重
            for i in range(6):
                current_joints[i] += weights[i] * delta_theta[i]
                # 限制关节角度在允许范围内
                current_joints[i] = np.clip(current_joints[i],
                                          self.joint_limits[i][0],
                                          self.joint_limits[i][1])
        
        return current_joints

    def calculate_ik(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> Optional[List[float]]:
        """计算逆运动学解

        Args:
            x: 目标位置x坐标（米）
            y: 目标位置y坐标（米）
            z: 目标位置z坐标（米）
            roll: 目标姿态roll角（弧度）
            pitch: 目标姿态pitch角（弧度）
            yaw: 目标姿态yaw角（弧度）

        Returns:
            List[float]: 关节角度列表，如果无解则返回None
        """
        try:
            # 创建目标位置向量
            target_pos = np.array([x, y, z])
            
            # 计算逆运动学解
            joint_angles = self.inverse_kinematics(target_pos)
            if joint_angles is None:
                return None
            
            # 检查解是否在关节限制范围内
            for i, (angle, (min_angle, max_angle)) in enumerate(zip(joint_angles, self.joint_limits)):
                if angle < min_angle or angle > max_angle:
                    print(f"Joint {i} angle {angle:.2f} exceeds limits [{min_angle:.2f}, {max_angle:.2f}]")
                    return None
            
            return joint_angles
            
        except Exception as e:
            print(f"Error calculating inverse kinematics: {e}")
            return None

    def plan_trajectory(self, stroke_points: List[StrokePoint]) -> List[Tuple[List[float], float]]:
        """规划轨迹，返回关节角度序列和对应的时间点
        
        Args:
            stroke_points: 笔画轨迹点列表
        
        Returns:
            List[Tuple[List[float], float]]: 关节角度和对应时间点的列表
        """
        trajectory = []
        current_time = 0.0
        
        for i, point in enumerate(stroke_points):
            # 计算目标位置
            target_pos = np.array([point.x, point.y, point.z]) / 1000.0  # 转换为米
            
            # 计算关节角度
            joint_angles = self.inverse_kinematics(target_pos)
            
            # 计算时间点
            if i > 0:
                # 计算与上一个点的距离
                prev_point = stroke_points[i-1]
                distance = np.sqrt((point.x - prev_point.x)**2 +
                                 (point.y - prev_point.y)**2 +
                                 (point.z - prev_point.z)**2)
                # 根据速度计算时间增量
                time_increment = distance / point.speed
                current_time += time_increment
            
            trajectory.append((joint_angles, current_time))
        
        return trajectory

    def interpolate_trajectory(self, trajectory: List[Tuple[List[float], float]], dt: float = 0.01) -> List[List[float]]:
        """对轨迹进行插值，生成更密集的路径点
        
        Args:
            trajectory: 关节角度和时间点的列表
            dt: 时间步长
        
        Returns:
            List[List[float]]: 插值后的关节角度序列
        """
        if not trajectory:
            return []
        
        # 提取时间点和关节角度
        times = [t for _, t in trajectory]
        joint_angles = [angles for angles, _ in trajectory]
        
        # 创建插值时间点
        t_interp = np.arange(times[0], times[-1], dt)
        
        # 对每个关节进行插值
        interpolated = []
        for i in range(6):  # 6个关节
            joint_i = [angles[i] for angles in joint_angles]
            # 使用三次样条插值
            interpolated_i = np.interp(t_interp, times, joint_i)
            interpolated.append(interpolated_i)
        
        # 重新组织数据格式
        return [[joint[i] for joint in interpolated] for i in range(len(t_interp))]
