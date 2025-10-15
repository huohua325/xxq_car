#!/usr/bin/env python3
"""
电机PWM标定脚本（独立PWM控制版本）
自动测试不同PWM值对应的轮速，建立线性关系
支持左右轮独立PWM控制

使用方法：
python pwm_calibration.py

协议格式：
TURN,direction,left_pwm,right_pwm
- direction: 0=左转, 1=右转
- left_pwm: 左轮PWM (0.0-1.0)
- right_pwm: 右轮PWM (0.0-1.0)
"""

import time
import csv
from datetime import datetime
import sys
import os

# 直接导入SimpleBLERobotComm类（避免执行main函数）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 只导入必要的部分
import asyncio
import json
import threading
from typing import Optional, Callable

try:
    from bleak import BleakClient
except ImportError:
    print("❌ 未安装bleak库")
    print("请运行: pip install bleak")
    sys.exit(1)

# 数据类（从ble_robot_control.py复制）
class LidarData:
    def __init__(self, timestamp: int, total_points: int, angle_coverage: float, sectors: list):
        self.timestamp = timestamp
        self.total_points = total_points
        self.angle_coverage = angle_coverage
        self.sectors = sectors

class OdometryData:
    def __init__(self, timestamp: int, left_speed: float, right_speed: float, left_count: int, right_count: int):
        self.timestamp = timestamp
        self.left_speed = left_speed
        self.right_speed = right_speed
        self.left_count = left_count
        self.right_count = right_count

# 导入SimpleBLERobotComm类（需要读取文件但不执行main）
with open('ble_robot_control.py', encoding='utf-8') as f:
    code = f.read()
    # 只执行到main()之前
    code_lines = code.split('\n')
    class_code = []
    for line in code_lines:
        if line.startswith('if __name__'):
            break
        class_code.append(line)
    exec('\n'.join(class_code), globals())

def calibrate_motor_pwm():
    """PWM标定主程序"""
    address = "C4:25:01:20:02:8E"
    
    print("=" * 90)
    print("🔧 电机PWM标定程序")
    print("=" * 90)
    print("功能：测试多个PWM值，记录对应的轮速")
    print("输出：CSV文件，用于分析PWM-速度线性关系")
    print("=" * 90)
    
    # 配置标定参数
    print("\n⚙️ 标定配置：")
    print("  1. PWM范围：0.1 - 0.9")
    print("  2. PWM步进：0.05（即每次增加5%）")
    print("  3. 每个点测试时间：2秒")
    print("  4. 测试内容：左转、右转")
    
    confirm = input("\n是否开始标定？(y/n): ").strip().lower()
    if confirm != 'y':
        print("已取消")
        return
    
    # 连接BLE
    print("\n🔵 连接BLE...")
    robot = SimpleBLERobotComm(address, verbose=False)
    
    # 用于收集速度数据
    speed_data = {
        'left_wheel': {},  # {pwm: speed}
        'right_wheel': {}
    }
    
    # 回调函数：收集ODO数据（如果STM32发送的话）
    current_speeds = {'left': 0.0, 'right': 0.0}
    
    def on_odo(odo):
        current_speeds['left'] = odo.left_speed
        current_speeds['right'] = odo.right_speed
        print(f"\r  轮速: L={odo.left_speed:.2f} R={odo.right_speed:.2f} RPS", end='')
    
    robot.on_odom_update = on_odo
    robot.on_message = lambda msg: None  # 静默模式，不显示消息
    
    if not robot.connect():
        print("❌ 连接失败")
        return
    
    try:
        # 生成PWM测试点
        pwm_values = []
        pwm = 0.10
        while pwm <= 0.90:
            pwm_values.append(round(pwm, 2))
            pwm += 0.05
        
        print(f"\n📊 将测试 {len(pwm_values)} 个PWM点")
        print(f"预计耗时: {len(pwm_values) * 2 * 2} 秒（每个点2秒 × 左右转）")
        print()
        
        results = []
        
        # 测试左转（测试左右轮在不同PWM下的表现）
        print("\n" + "="*90)
        print("📍 开始测试 - 左转模式（独立PWM控制）")
        print("="*90)
        
        for i, pwm in enumerate(pwm_values, 1):
            print(f"\n[{i}/{len(pwm_values)}] 测试PWM = {pwm:.2f}")
            
            # 发送独立PWM命令（新格式：TURN,direction,left_pwm,right_pwm）
            # 左转：左轮后退，右轮前进，使用相同PWM观察两轮速度差异
            cmd = f"TURN,0,{pwm:.2f},{pwm:.2f}\n"  # 0=左转, left_pwm, right_pwm
            robot.send_command(cmd)
            print(f"  已发送命令: {cmd.strip()}")
            
            # 等待速度稳定
            print(f"  ⏳ 等待2秒...")
            time.sleep(2)
            
            # 记录速度（如果有ODO数据）
            left_speed = current_speeds['left']
            right_speed = current_speeds['right']
            
            print(f"\n  📊 结果: 左轮PWM={pwm:.2f} (后退) → {left_speed:.2f} RPS")
            print(f"          右轮PWM={pwm:.2f} (前进) → {right_speed:.2f} RPS")
            
            # 保存数据
            results.append({
                'mode': 'left_turn',
                'left_pwm': pwm,
                'right_pwm': pwm,
                'left_speed': left_speed,
                'right_speed': right_speed,
                'left_direction': 'backward',
                'right_direction': 'forward'
            })
            
            # 停止
            robot.stop_robot()
            time.sleep(0.5)
        
        # 测试右转
        print("\n" + "="*90)
        print("📍 开始测试 - 右转模式（独立PWM控制）")
        print("="*90)
        
        for i, pwm in enumerate(pwm_values, 1):
            print(f"\n[{i}/{len(pwm_values)}] 测试PWM = {pwm:.2f}")
            
            # 发送独立PWM命令（新格式：TURN,direction,left_pwm,right_pwm）
            # 右转：左轮前进，右轮后退，使用相同PWM观察两轮速度差异
            cmd = f"TURN,1,{pwm:.2f},{pwm:.2f}\n"  # 1=右转, left_pwm, right_pwm
            robot.send_command(cmd)
            print(f"  已发送命令: {cmd.strip()}")
            
            # 等待稳定
            print(f"  ⏳ 等待2秒...")
            time.sleep(2)
            
            # 记录速度
            left_speed = current_speeds['left']
            right_speed = current_speeds['right']
            
            print(f"\n  📊 结果: 左轮PWM={pwm:.2f} (前进) → {left_speed:.2f} RPS")
            print(f"          右轮PWM={pwm:.2f} (后退) → {right_speed:.2f} RPS")
            
            results.append({
                'mode': 'right_turn',
                'left_pwm': pwm,
                'right_pwm': pwm,
                'left_speed': left_speed,
                'right_speed': right_speed,
                'left_direction': 'forward',
                'right_direction': 'backward'
            })
            
            # 停止
            robot.stop_robot()
            time.sleep(0.5)
        
        # 保存结果到CSV
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"pwm_calibration_{timestamp}.csv"
        
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'mode', 'left_pwm', 'right_pwm', 'left_speed', 'right_speed',
                'left_direction', 'right_direction'
            ])
            writer.writeheader()
            writer.writerows(results)
        
        print("\n" + "="*90)
        print("✅ 标定完成！")
        print("="*90)
        print(f"数据已保存到: {filename}")
        print(f"总测试点: {len(results)}")
        
        # 显示摘要
        print("\n📊 数据摘要（左转模式 - 左轮后退）:")
        print("-"*90)
        print(f"{'PWM':<8} {'左轮速度':<12} {'备注':<20}")
        print("-"*90)
        
        for r in results[:len(pwm_values)]:  # 左转数据
            if abs(r['left_speed']) > 0.1:  # 只显示有明显速度的
                print(f"{r['left_pwm']:<8.2f} {r['left_speed']:<12.2f} 后退({r['left_direction']})")
        
        print("\n📊 数据摘要（左转模式 - 右轮前进）:")
        print("-"*90)
        print(f"{'PWM':<8} {'右轮速度':<12} {'备注':<20}")
        print("-"*90)
        
        for r in results[:len(pwm_values)]:  # 左转数据
            if abs(r['right_speed']) > 0.1:  # 只显示有明显速度的
                print(f"{r['right_pwm']:<8.2f} {r['right_speed']:<12.2f} 前进({r['right_direction']})")
        
        print("\n📊 数据摘要（右转模式 - 左轮前进）:")
        print("-"*90)
        print(f"{'PWM':<8} {'左轮速度':<12} {'备注':<20}")
        print("-"*90)
        
        for r in results[len(pwm_values):]:  # 右转数据
            if abs(r['left_speed']) > 0.1:  # 只显示有明显速度的
                print(f"{r['left_pwm']:<8.2f} {r['left_speed']:<12.2f} 前进({r['left_direction']})")
        
        print("\n📊 数据摘要（右转模式 - 右轮后退）:")
        print("-"*90)
        print(f"{'PWM':<8} {'右轮速度':<12} {'备注':<20}")
        print("-"*90)
        
        for r in results[len(pwm_values):]:  # 右转数据
            if abs(r['right_speed']) > 0.1:  # 只显示有明显速度的
                print(f"{r['right_pwm']:<8.2f} {r['right_speed']:<12.2f} 后退({r['right_direction']})")
        
        print("\n💡 下一步：")
        print(f"  1. 用Excel打开 {filename}")
        print(f"  2. 分析每个轮子在前进/后退时的PWM-速度关系")
        print(f"  3. 找到最佳PWM组合（考虑左右轮不同特性）")
        print(f"  4. 更新Motor.c中的Car_TurnLeft_Continuous/Car_TurnRight_Continuous")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断标定")
        robot.stop_robot()
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()

def quick_test_turn():
    """快速测试独立PWM值"""
    address = "C4:25:01:20:02:8E"
    
    print("=" * 90)
    print("🔧 快速转向PWM测试（独立PWM控制）")
    print("=" * 90)
    
    # 获取参数
    try:
        left_pwm = float(input("请输入左轮PWM值 (0.1-0.9): ").strip())
        if left_pwm < 0.1 or left_pwm > 0.9:
            print("❌ 左轮PWM值超出范围")
            return
            
        right_pwm = float(input("请输入右轮PWM值 (0.1-0.9): ").strip())
        if right_pwm < 0.1 or right_pwm > 0.9:
            print("❌ 右轮PWM值超出范围")
            return
    except:
        print("❌ 输入无效")
        return
    
    direction = input("方向 (a=左转, d=右转): ").strip().lower()
    if direction not in ['a', 'd']:
        print("❌ 方向无效")
        return
    
    # 连接
    robot = SimpleBLERobotComm(address, verbose=False)
    
    # 显示ODO数据
    def on_odo(odo):
        print(f"\r  实时速度: 左轮={odo.left_speed:.2f} RPS, 右轮={odo.right_speed:.2f} RPS", end='')
    
    robot.on_odom_update = on_odo
    robot.on_message = lambda msg: print(f"\n📨 {msg}") if "[OK]" in msg or "[ERROR]" in msg else None
    
    if not robot.connect():
        return
    
    try:
        direction_num = 0 if direction == 'a' else 1
        direction_name = "左转" if direction == 'a' else "右转"
        
        if direction == 'a':
            print(f"\n🎯 测试: {direction_name}")
            print(f"  左轮PWM={left_pwm:.2f} (后退)")
            print(f"  右轮PWM={right_pwm:.2f} (前进)")
        else:
            print(f"\n🎯 测试: {direction_name}")
            print(f"  左轮PWM={left_pwm:.2f} (前进)")
            print(f"  右轮PWM={right_pwm:.2f} (后退)")
        
        cmd = f"TURN,{direction_num},{left_pwm:.2f},{right_pwm:.2f}\n"
        robot.send_command(cmd)
        
        print("\n⏳ 运行3秒，观察转速...")
        time.sleep(3)
        
        print("\n\n⏹️ 停止")
        robot.stop_robot()
        time.sleep(1)
        
        print("\n✅ 测试完成")
        
    except KeyboardInterrupt:
        print("\n⚠️ 中断")
        robot.stop_robot()
    finally:
        robot.disconnect()

if __name__ == '__main__':
    print("\n🤖 电机PWM标定工具")
    print("选择模式:")
    print("  1. 完整标定（测试多个PWM点，约5-10分钟）")
    print("  2. 快速测试（测试单个PWM值）")
    print("  3. 退出")
    
    choice = input("\n请选择 (1/2/3): ").strip()
    
    if choice == '1':
        calibrate_motor_pwm()
    elif choice == '2':
        quick_test_turn()
    elif choice == '3':
        print("再见！")
    else:
        print("无效选择")

