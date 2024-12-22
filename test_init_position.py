#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from arm_controller import ArmController
import time

def main():
    arm = ArmController()
    
    try:
        # 连接机械臂
        print("正在连接机械臂...")
        if not arm.connect():
            print("错误：无法连接到机械臂")
            return
        
        # 先移动到电机零位
        print("\n正在移动到电机零位...")
        if not arm.move_to_zero():
            print("错误：无法移动到电机零位")
            return
        
        time.sleep(2)  # 等待稳定
        
        # 移动到URDF初始位置
        print("\n正在移动到URDF初始位置...")
        if not arm.move_to_urdf_init():
            print("错误：无法移动到URDF初始位置")
            return
        
        print("\n已到达URDF初始位置")
        print("按Ctrl+C退出...")
        
        # 保持在初始位置
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n收到中断信号，正在停止...")
    except Exception as e:
        print(f"\n发生错误: {e}")
    finally:
        # 清理资源
        print("\n正在断开连接...")
        arm.disconnect()
        print("程序结束")

if __name__ == "__main__":
    main()
