#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from .motors.feetech import FeetechMotorsBus
import numpy as np
import time
from typing import List, Dict, Tuple, Optional
from .trajectory_generator import StrokePoint
from .motion_controller import MotionController
from scipy.spatial.transform import Rotation as R

class ArmController:
    def __init__(self):
        """初始化机械臂控制器"""
        # 定义电机配置
        self.motors = {
            "shoulder_pan_joint": (1, "sts3215"),
            "shoulder_lift_joint": (2, "sts3215"),
            "elbow_joint": (3, "sts3215"),
            "wrist_pitch_joint": (4, "sts3215"),
            "wrist_roll_joint": (5, "sts3215"),
            "jaw_joint": (6, "sts3215")
        }
        
        # URDF初始位置对应的实际关节角度（弧度）
        # 这些角度表示从电机零点位置（步数2048）到达URDF初始位置需要转动的角度
        self.urdf_init_angles = {
            "shoulder_pan_joint": 0.0,  # 保持不变
            "shoulder_lift_joint": 0.7854,  # 45度
            "elbow_joint": 0.0,  # 保持不变
            "wrist_pitch_joint": -0.7854,  # -45度
            "wrist_roll_joint": -1.5708,  # -90度
            "jaw_joint": -0.69813  # -40度
        }
        
        # 创建运动控制器
        self.motion_controller = MotionController()
        
        # 获取机械臂运动链
        self.chain = self.motion_controller.chain
        
        # 设置关节限制
        self.joint_limits = [
            [-1.57, 1.57],       # shoulder_pan_joint  
            [-1.57, 1.57],     # shoulder_lift_joint (±90度)
            [-1.57, 1.57],     # elbow_joint        (±90度)
            [-1.57, 1.57],     # wrist_pitch_joint  (±90度)
            [-1.57, 1.57],     # wrist_roll_joint   
            [-0.78, 0.78],     # jaw_joint          (±45度)
        ]
        
        # 运动速度参数
        self.max_speed = 200   # 最大速度限制
        
        # 初始化总线连接
        self.bus = None
        
        # 初始化当前关节角度
        self.current_joints = [0.0] * 6  # 6个关节的初始角度都设为0
        
        # 是否处于仿真模式
        self.simulation_mode = True

    def rad_to_steps(self, rad: float) -> int:
        """将弧度转换为电机步数
        
        STS3215舵机的步数范围是0-4096
        0对应-180度
        2048对应0度（电机零点位置）
        4096对应180度
        """
        degree = rad * 180.0 / np.pi
        # 限制角度范围在-180到180度之间
        degree = np.clip(degree, -180, 180)
        # 将角度映射到步数范围：-180度对应0步，0度对应2048步，180度对应4096步
        steps = int(2048 + (degree * 2048 / 180.0))
        return steps

    def steps_to_rad(self, steps: int) -> float:
        """将电机步数转换为弧度
        
        STS3215舵机的步数范围是0-4096
        0对应-180度
        2048对应0度（电机零点位置）
        4096对应180度
        """
        # 将步数转换为角度：0步对应-180度，2048步对应0度，4096步对应180度
        degree = (steps - 2048) * 180.0 / 2048.0
        # 将角度转换为弧度
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

    def get_joint_angles(self, target_pose, initial_position=None):
        """计算给定目标位姿的逆运动学解
        
        Args:
            target_pose: 目标位姿 [x, y, z, roll, pitch, yaw]
            initial_position: 可选的初始关节角度，如果不提供则使用当前关节角度
        """
        try:
            # 如果没有提供初始位置，使用当前关节角度
            if initial_position is None:
                # 创建完整的初始位置（包括固定关节）
                initial_position = [0.0]  # base_link (固定)
                # 当前关节角度已经是相对于URDF初始位置的角度，无需转换
                initial_position.extend(self.current_joints[:5])  # 前5个活动关节
                initial_position.append(0.0)  # jaw_joint固定为0
                initial_position.append(0.0)  # pen_point_joint (固定)
            else:
                # 确保提供的初始位置是完整的（8个关节）
                if len(initial_position) != 8:
                    # 如果提供的是只有活动关节的角度，添加固定关节
                    if len(initial_position) == 6:
                        initial_position = [0.0] + list(initial_position) + [0.0]
                    else:
                        raise ValueError(f"初始位置应该包含6个或8个关节角度，得到了 {len(initial_position)} 个")
            
            # 使用IKPY计算逆运动学
            joint_angles = self.chain.inverse_kinematics(
                target_position=target_pose[:3],
                target_orientation=R.from_euler('xyz', target_pose[3:]).as_matrix(),
                orientation_mode="all",  # 考虑所有轴的方向
                max_iter=10000,  # 增加最大迭代次数
                initial_position=initial_position
            )
            
            # 只取活动关节的角度（跳过第一个和最后两个固定关节）
            active_joint_angles = list(joint_angles[1:6])  # 只取前5个活动关节
            active_joint_angles.append(0.0)  # jaw_joint固定为0
            
            # 确保所有角度都是浮点数
            active_joint_angles = [float(angle) for angle in active_joint_angles]
            
            # 检查关节限制（只检查前5个关节）
            for i, angle in enumerate(active_joint_angles[:5]):
                if angle < self.joint_limits[i][0] or angle > self.joint_limits[i][1]:
                    print(f"警告：关节 {i+1} 角度 {np.degrees(angle):.2f}° 超出限制范围 [{np.degrees(self.joint_limits[i][0]):.2f}°, {np.degrees(self.joint_limits[i][1]):.2f}°]")
                    return None
            
            return active_joint_angles
            
        except Exception as e:
            print(f"逆运动学计算错误: {str(e)}")
            return None

    def move_to_joint_angles(self, target_angles: List[float]) -> bool:
        """移动到指定的关节角度（弧度）
        
        注意：target_angles是相对于URDF初始位置的角度，需要加上URDF初始位置的偏移才是实际电机需要转到的角度
        """
        try:
            # 检查输入参数
            if len(target_angles) != len(self.motors):
                print(f"Expected {len(self.motors)} angles, got {len(target_angles)}")
                return False
            
            # 检查关节限制
            for i, angle in enumerate(target_angles):
                if angle < self.joint_limits[i][0] or angle > self.joint_limits[i][1]:
                    print(f"警告：关节 {i+1} 角度 {np.degrees(angle):.2f}° 超出限制范围 [{np.degrees(self.joint_limits[i][0]):.2f}°, {np.degrees(self.joint_limits[i][1]):.2f}°]")
                    return False
            
            if self.simulation_mode:
                # 在仿真模式下，直接更新关节角度
                self.current_joints = target_angles
                return True
            else:
                # 实际硬件控制代码
                if not self.bus:
                    return False
                
                # 将目标角度转换为实际电机角度（加上URDF初始位置的偏移）
                actual_target_angles = []
                for i, (joint_name, _) in enumerate(self.motors.items()):
                    actual_angle = target_angles[i] + self.urdf_init_angles[joint_name]
                    actual_target_angles.append(actual_angle)
                
                # 将实际目标角度转换为电机步数
                target_positions = [self.rad_to_steps(angle) for angle in actual_target_angles]
                
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
                
                # 更新当前关节角度（保存的是相对于URDF初始位置的角度）
                self.current_joints = target_angles
                return True
            
        except Exception as e:
            print(f"移动关节时出错: {str(e)}")
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
        """移动到URDF初始位置"""
        try:
            if self.simulation_mode:
                self.current_joints = [0.0] * len(self.motors)  # 仿真模式下直接设为0
                return True
                
            # 实际硬件控制代码
            if not self.bus:
                print("错误：串口未连接")
                return False
                
            # 将URDF初始位置的关节角度转换为电机步数
            init_positions = []
            for joint_name in self.motors.keys():
                init_angle = self.urdf_init_angles[joint_name]
                init_positions.append(self.rad_to_steps(init_angle))
                
            # 获取当前位置
            current_positions = []
            for joint_name in self.motors.keys():
                pos = self.bus.read("Present_Position", joint_name)
                if pos is None:
                    return False
                current_positions.append(pos)
                
            # 生成平滑路径
            path_points = self.interpolate_path(current_positions, init_positions)
            
            # 执行运动
            joint_names = list(self.motors.keys())
            for positions in path_points:
                for i, joint_name in enumerate(joint_names):
                    self.bus.write("Goal_Position", positions[i], joint_name)
                time.sleep(0.02)  # 50Hz控制频率
                
            # 更新当前关节角度为URDF中的0位置
            self.current_joints = [0.0] * len(self.motors)
            return True
            
        except Exception as e:
            print(f"移动到URDF初始位置时出错: {str(e)}")
            return False
