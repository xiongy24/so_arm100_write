#!/usr/bin/env python3
# 该程序用于直接控制机械臂，测试每个关节的运动。

from motors.feetech import FeetechMotorsBus
import time

def main():
    # 定义电机配置和限制
    motors = {
        "shoulder_pan_joint": (1, "sts3215"),
        "shoulder_lift_joint": (2, "sts3215"),
        "elbow_joint": (3, "sts3215"),
        "wrist_pitch_joint": (4, "sts3215"),
        "wrist_roll_joint": (5, "sts3215"),
        "jaw_joint": (6, "sts3215")
    }

    # 定义关节限制（弧度）
    joint_limits = {
        "shoulder_pan_joint": (-1.57, 1.57),
        "shoulder_lift_joint": (-1.57, 1.57),
        "elbow_joint": (-1.57, 1.57),
        "wrist_pitch_joint": (-1.57, 1.57),
        "wrist_roll_joint": (-1.57, 1.57),
        "jaw_joint": (-0.7, 0.7)
    }

    # 将弧度转换为步数的函数
    def rad_to_steps(rad):
        # 2048对应0度，4096步对应360度，0步对应-180度
        degree = rad * 180.0 / 3.14159  # 转换为角度
        steps = int(2048 + (degree * 4096 / 360.0))  # 每度对应 4096/360 步
        return steps

    # 将步数转换为弧度的函数
    def steps_to_rad(steps):
        # 2048对应0度，4096步对应360度，0步对应-180度
        degree = (steps - 2048) * 360.0 / 4096  # 转换为角度
        rad = degree * 3.14159 / 180.0  # 转换为弧度
        return rad

    try:
        # 初始化总线
        bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors)
        print("Connecting to motors bus...")
        bus.connect()
        
        # 等待连接稳定
        time.sleep(1)
        
        # 启用扭矩
        print("\nEnabling torque...")
        bus.write("Torque_Enable", [1] * len(motors))
        time.sleep(0.5)

        # 设置P系数（控制精度）
        print("\nSetting P coefficient...")
        bus.write("P_Coefficient", [32] * len(motors))
        time.sleep(0.5)

        # 移动到中心位置（零位）
        print("\n将机械臂移动到零位（中心位置）...")
        bus.write("Goal_Position", [2048] * len(motors))
        time.sleep(3)  # 等待所有关节到达零位位置

        # 读取当前位置
        print("\nReading current positions...")
        positions = bus.read("Present_Position")
        print(f"Current positions: {positions}")

        # 测试每个关节
        for joint_name in motors.keys():
            print(f"\nTesting {joint_name}...")
            
            # 获取当前位置
            current_pos = bus.read("Present_Position", joint_name)
            current_rad = steps_to_rad(current_pos)
            print(f"Current position: {current_pos} steps ({current_rad:.2f} rad)")
            
            # 计算目标位置（当前位置增加10度，约0.175弧度）
            target_rad = current_rad + 0.175
            # 确保在限制范围内
            min_rad, max_rad = joint_limits[joint_name]
            target_rad = min(max(target_rad, min_rad), max_rad)
            target_pos = rad_to_steps(target_rad)
            
            print(f"Moving to position: {target_pos} steps ({target_rad:.2f} rad)")
            bus.write("Goal_Position", target_pos, joint_name)
            time.sleep(1)
            
            # 读取新位置
            new_pos = bus.read("Present_Position", joint_name)
            new_rad = steps_to_rad(new_pos)
            print(f"New position: {new_pos} steps ({new_rad:.2f} rad)")
            
            # 返回原位
            print("Moving back...")
            bus.write("Goal_Position", current_pos, joint_name)
            time.sleep(1)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'bus' in locals():
            # 禁用扭矩
            print("\nDisabling torque...")
            bus.write("Torque_Enable", [0] * len(motors))
            time.sleep(0.5)
            
            # 断开连接
            print("Disconnecting...")
            bus.disconnect()
            print("Test completed")

if __name__ == "__main__":
    main()
