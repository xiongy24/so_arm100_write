#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
from typing import List, Optional
from trajectory_generator import TrajectoryGenerator, StrokePoint
from motion_controller import MotionController
from arm_controller import ArmController

class CharacterWriter:
    def __init__(self):
        self.trajectory_generator = TrajectoryGenerator()
        self.motion_controller = MotionController()
        self.arm_controller = ArmController()
        
        # 写字区域参数（单位：毫米）
        self.writing_area = {
            'center_x': 150,    # 减小距离，避免关节角度过大
            'center_y': 0,
            'width': 100,      # 减小写字区域大小
            'height': 100,
            'z_surface': 30,   # 降低写字平面高度
            'z_up': 50,        # 降低抬笔高度
        }

    def initialize(self) -> bool:
        """初始化系统"""
        try:
            print("正在初始化写字系统...")
            
            # 连接机械臂
            if not self.arm_controller.connect():
                print("错误：无法连接到机械臂")
                return False
            
            # 先移动到电机零位
            if not self.arm_controller.move_to_zero():
                print("错误：无法移动到电机零位")
                return False
            
            time.sleep(1)  # 等待稳定
            
            # 移动到URDF初始位置
            if not self.arm_controller.move_to_urdf_init():
                print("错误：无法移动到URDF初始位置")
                return False
            
            print("初始化完成")
            return True
            
        except Exception as e:
            print(f"初始化错误: {e}")
            return False

    def cleanup(self):
        """清理系统资源"""
        try:
            print("正在清理系统...")
            self.arm_controller.disconnect()
            print("清理完成")
        except Exception as e:
            print(f"清理错误: {e}")

    def write_stroke(self, stroke_points: List[StrokePoint]) -> bool:
        """执行单个笔画"""
        try:
            return self.arm_controller.write_stroke(stroke_points)
        except Exception as e:
            print(f"笔画执行错误: {e}")
            return False

    def write_character(self, char: str) -> bool:
        """写一个汉字"""
        try:
            print(f"正在写字：{char}")
            
            # 根据不同的字生成轨迹
            if char == "二":
                strokes = self.trajectory_generator.generate_er_strokes()
            elif char == "四":
                strokes = self.trajectory_generator.generate_si_strokes()
            else:
                print(f"错误：不支持的字符 {char}")
                return False
            
            # 执行每个笔画
            for i, stroke in enumerate(strokes, 1):
                print(f"正在写第 {i} 笔...")
                if not self.write_stroke(stroke):
                    print(f"错误：第 {i} 笔写失败")
                    return False
                time.sleep(0.5)  # 笔画之间的停顿
            
            return True
            
        except Exception as e:
            print(f"写字错误: {e}")
            return False

    def write_characters(self, chars: str = "二四") -> bool:
        """写多个汉字"""
        try:
            print(f"开始写字：{chars}")
            
            for i, char in enumerate(chars, 1):
                print(f"\n正在写第 {i} 个字：{char}")
                if not self.write_character(char):
                    print(f"错误：第 {i} 个字 {char} 写失败")
                    return False
                time.sleep(1)  # 字之间的停顿
            
            print("\n写字完成")
            return True
            
        except Exception as e:
            print(f"写字错误: {e}")
            return False

def main():
    writer = CharacterWriter()
    
    try:
        # 初始化系统
        if not writer.initialize():
            print("系统初始化失败")
            return
        
        # 写字
        if not writer.write_characters():
            print("写字失败")
            return
        
        print("任务完成")
        
    except KeyboardInterrupt:
        print("\n收到中断信号，正在停止...")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        writer.cleanup()

if __name__ == "__main__":
    main()
