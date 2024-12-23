#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from typing import List, Tuple, Optional
from .trajectory_generator import StrokePoint
import math
import ikpy.chain
import ikpy.utils.plot as plot_utils
import os
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET

class MotionController:
    def __init__(self):
        # 获取 URDF 文件路径
        arm_description_dir = get_package_share_directory('arm_description')
        urdf_file = os.path.join(arm_description_dir, 'urdf', 'so_arm100_write.urdf')
        
        # 如果找不到安装目录中的文件，尝试源代码目录
        if not os.path.exists(urdf_file):
            src_urdf_file = "/home/ubuntu22/so_arm100_write/ros2_ws/src/arm_description/urdf/so_arm100_write.urdf"
            if os.path.exists(src_urdf_file):
                urdf_file = src_urdf_file
            else:
                raise FileNotFoundError(f"找不到 URDF 文件，已尝试以下路径：\n1. {urdf_file}\n2. {src_urdf_file}")
        
        print(f"使用 URDF 文件：{urdf_file}")
        
        # 解析 URDF 文件以获取关节信息
        tree = ET.parse(urdf_file)
        root = tree.getroot()
        
        # 计算活动关节掩码（注意：IKPy 会自动添加一个原点链接）
        active_links_mask = [False]  # 原点链接设为非活动的
        joint_count = 0
        
        for joint in root.findall(".//joint"):
            joint_type = joint.get("type")
            joint_name = joint.get("name")
            print(f"找到关节: {joint_name} (类型: {joint_type})")
            
            # 只有非固定关节才计入
            if joint_type != "fixed":
                joint_count += 1
                active_links_mask.append(True)  # 将所有非固定关节设为活动的
            else:
                active_links_mask.append(False)  # 固定关节设为非活动的
        
        print(f"总链接数: {len(active_links_mask)} (包括原点链接)")
        print(f"活动关节数: {joint_count}")
        print(f"活动关节掩码: {active_links_mask}")
        
        # 创建机械臂运动链
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_file,
            base_elements=["base_link"],
            active_links_mask=active_links_mask,
            name="arm"
        )
        
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

    def forward_kinematics(self, joint_angles: List[float]) -> np.ndarray:
        """正向运动学：计算给定关节角度下的末端执行器位姿"""
        # 使用 IKPy 的正向运动学
        fk_matrix = self.chain.forward_kinematics(joint_angles)
        
        # 添加笔尖偏移
        fk_matrix[0:3, 3] += fk_matrix[0:3, 0:3] @ self.pen_offset
        
        return fk_matrix

    def calculate_ik(self, x: float, y: float, z: float, roll: float = 0.0, pitch: float = -np.pi/2, yaw: float = 0.0) -> Optional[List[float]]:
        """计算逆运动学解

        Args:
            x: 目标位置x坐标（米）
            y: 目标位置y坐标（米）
            z: 目标位置z坐标（米）
            roll: 目标姿态roll角（弧度），默认0
            pitch: 目标姿态pitch角（弧度），默认-pi/2（笔尖朝下）
            yaw: 目标姿态yaw角（弧度），默认0

        Returns:
            List[float]: 关节角度列表，如果无解则返回None
        """
        try:
            print(f"\n=== 计算逆运动学 ===")
            print(f"目标位置: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            print(f"目标姿态: roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}")
            
            # 考虑笔尖偏移，调整目标位置
            target_pos = np.array([x, y, z]) - np.array([0, 0, self.pen_offset[2]])
            print(f"考虑笔尖偏移后的目标位置: {target_pos}")
            
            # 设置目标姿态矩阵
            target_orientation = np.array([
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)]
            ]) @ np.array([
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)]
            ]) @ np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw), np.cos(yaw), 0],
                [0, 0, 1]
            ])
            
            # 构建目标变换矩阵
            target_matrix = np.eye(4)
            target_matrix[0:3, 0:3] = target_orientation
            target_matrix[0:3, 3] = target_pos
            
            # 使用当前关节角度作为初始猜测
            initial_position = self.current_joints if any(self.current_joints) else None
            
            # 计算逆运动学
            joint_angles = self.chain.inverse_kinematics(
                target_matrix,
                initial_position=initial_position,
                orientation_mode="all",  # 考虑完整的姿态约束
                max_iter=100
            )
            
            if joint_angles is None:
                print("无法找到逆运动学解")
                return None
            
            print("\n计算得到的关节角度（弧度）:")
            for i, angle in enumerate(joint_angles[1:]):  # 跳过第一个元素（base）
                print(f"Joint {i}: {angle:.3f} rad ({math.degrees(angle):.1f} deg)")
            
            # 检查关节限制
            for i, (angle, (min_angle, max_angle)) in enumerate(zip(joint_angles[1:], self.joint_limits)):
                if angle < min_angle or angle > max_angle:
                    print(f"警告: Joint {i} 角度 {angle:.2f} rad ({math.degrees(angle):.1f} deg) 超出限制范围 [{min_angle:.2f}, {max_angle:.2f}]")
                    return None
                print(f"Joint {i} 在限制范围内: {min_angle:.2f} <= {angle:.2f} <= {max_angle:.2f}")
            
            # 验证解的准确性
            fk_matrix = self.chain.forward_kinematics(joint_angles)
            actual_pos = fk_matrix[0:3, 3]
            pos_error = np.linalg.norm(actual_pos - target_pos)
            print(f"\n位置误差: {pos_error*1000:.2f} mm")
            
            if pos_error > 0.001:  # 误差大于1mm
                print("警告: 位置误差过大")
                print(f"目标位置: {target_pos}")
                print(f"实际位置: {actual_pos}")
            
            # 更新当前关节角度
            self.current_joints = list(joint_angles[1:])  # 跳过第一个元素（base）
            return self.current_joints
            
        except Exception as e:
            print(f"计算逆运动学时出错: {e}")
            import traceback
            traceback.print_exc()
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
            # 计算目标位置（单位转换：毫米->米）
            x = point.x / 1000.0
            y = point.y / 1000.0
            z = point.z / 1000.0
            
            # 计算关节角度
            joint_angles = self.calculate_ik(x, y, z)
            if joint_angles is None:
                print(f"警告: 无法为轨迹点 {i} 找到逆运动学解")
                continue
            
            # 计算时间点
            if i > 0:
                # 计算与上一个点的距离
                prev_point = stroke_points[i-1]
                distance = np.sqrt((point.x - prev_point.x)**2 +
                                 (point.y - prev_point.y)**2 +
                                 (point.z - prev_point.z)**2)
                # 根据速度计算时间增量（考虑单位转换）
                time_increment = (distance / 1000.0) / point.speed  # 距离转换为米
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
        
        interpolated = []
        for i in range(len(trajectory) - 1):
            current_joints, t1 = trajectory[i]
            next_joints, t2 = trajectory[i + 1]
            
            # 计算这段轨迹需要的步数
            steps = int((t2 - t1) / dt)
            
            for step in range(steps):
                # 线性插值
                t = step / steps
                joints = [current_joints[j] + t * (next_joints[j] - current_joints[j])
                         for j in range(len(current_joints))]
                interpolated.append(joints)
        
        # 添加最后一个点
        interpolated.append(trajectory[-1][0])
        
        return interpolated
