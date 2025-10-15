#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单独测试转向功能
使用calibrate_turn.py的完全相同的参数和流程
"""

import sys
import os
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ble_robot_control import SimpleBLERobotComm
from simple_pose_estimator import SimplePoseEstimator

BLE_ADDRESS = "C4:25:01:20:02:8E"


def test_turn_90():
    """测试90度转向（与calibrate_turn.py完全一致）"""
    
    print("\n" + "="*80)
    print("🔄 转向功能单独测试")
    print("="*80)
    print("\n测试内容：")
    print("  1. 左转90度（TURN,0,0.85,0.18, 1.0秒）")
    print("  2. 右转90度（TURN,1,0.50,0.45, 0.73秒）")
    print("\n参数来源：calibrate_turn.py 经过验证的标定结果")
    print("="*80 + "\n")
    
    robot = SimpleBLERobotComm(address=BLE_ADDRESS, verbose=False)
    pose_estimator = SimplePoseEstimator()
    
    def on_odo_update(data):
        """ODO数据回调，更新位姿"""
        pose_estimator.update(data)
        x, y, theta = pose_estimator.get_pose_degrees()
        print(f"\r  [位姿] x={x:6.3f}m, y={y:6.3f}m, θ={theta:6.1f}°", end='', flush=True)
    
    robot.on_odom_update = on_odo_update
    
    try:
        # 连接BLE
        print("🔵 连接BLE...")
        if not robot.connect():
            print("❌ 连接失败")
            return
        print("✅ BLE已连接\n")
        time.sleep(1)
        
        # ========== 测试1: 左转90度 ==========
        print("\n" + "="*80)
        print("测试1: 左转90度")
        print("="*80)
        print("  参数: direction=0, left_pwm=0.85, right_pwm=0.18")
        print("  持续时间: 1.0秒")
        
        print("\n  ⏸️  3秒后开始左转...")
        time.sleep(3)
        
        # 记录初始位姿
        pose_before = pose_estimator.get_pose_degrees()
        print(f"\n  初始位姿: ({pose_before[0]:.3f}, {pose_before[1]:.3f}, {pose_before[2]:.1f}°)")
        
        # 发送转向命令（完全模仿calibrate_turn.py）
        direction = 0
        left_pwm = 0.85
        right_pwm = 0.18
        duration = 1.0
        
        turn_cmd = f"TURN,{direction},{left_pwm:.2f},{right_pwm:.2f}\n"
        print(f"\n  📤 发送命令: {turn_cmd.strip()}")
        robot.send_command(turn_cmd)
        
        # 等待转向完成
        print(f"  ⏳ 转向中...（{duration}秒）")
        time.sleep(duration)
        
        # 停止
        print(f"  📤 发送停止命令: MODE,0")
        robot.send_command("MODE,0\n")
        time.sleep(0.5)
        
        # 记录最终位姿
        pose_after = pose_estimator.get_pose_degrees()
        print(f"\n  最终位姿: ({pose_after[0]:.3f}, {pose_after[1]:.3f}, {pose_after[2]:.1f}°)")
        
        # 计算角度变化
        angle_change = pose_after[2] - pose_before[2]
        print(f"\n  📊 结果:")
        print(f"     角度变化: {angle_change:+.1f}°")
        print(f"     期望值: +90.0°")
        print(f"     误差: {abs(angle_change - 90.0):.1f}°")
        
        if abs(angle_change - 90.0) < 10.0:
            print(f"     ✅ 转向准确（误差<10°）")
        else:
            print(f"     ⚠️ 转向不准确（误差≥10°）")
        
        time.sleep(2)
        
        # ========== 测试2: 右转90度 ==========
        print("\n" + "="*80)
        print("测试2: 右转90度")
        print("="*80)
        print("  参数: direction=1, left_pwm=0.50, right_pwm=0.45")
        print("  持续时间: 0.73秒")
        
        print("\n  ⏸️  3秒后开始右转...")
        time.sleep(3)
        
        # 记录初始位姿
        pose_before = pose_estimator.get_pose_degrees()
        print(f"\n  初始位姿: ({pose_before[0]:.3f}, {pose_before[1]:.3f}, {pose_before[2]:.1f}°)")
        
        # 发送转向命令
        direction = 1
        left_pwm = 0.50
        right_pwm = 0.45
        duration = 0.73
        
        turn_cmd = f"TURN,{direction},{left_pwm:.2f},{right_pwm:.2f}\n"
        print(f"\n  📤 发送命令: {turn_cmd.strip()}")
        robot.send_command(turn_cmd)
        
        # 等待转向完成
        print(f"  ⏳ 转向中...（{duration}秒）")
        time.sleep(duration)
        
        # 停止
        print(f"  📤 发送停止命令: MODE,0")
        robot.send_command("MODE,0\n")
        time.sleep(0.5)
        
        # 记录最终位姿
        pose_after = pose_estimator.get_pose_degrees()
        print(f"\n  最终位姿: ({pose_after[0]:.3f}, {pose_after[1]:.3f}, {pose_after[2]:.1f}°)")
        
        # 计算角度变化
        angle_change = pose_after[2] - pose_before[2]
        print(f"\n  📊 结果:")
        print(f"     角度变化: {angle_change:+.1f}°")
        print(f"     期望值: -90.0°")
        print(f"     误差: {abs(angle_change + 90.0):.1f}°")
        
        if abs(angle_change + 90.0) < 10.0:
            print(f"     ✅ 转向准确（误差<10°）")
        else:
            print(f"     ⚠️ 转向不准确（误差≥10°）")
        
        # ========== 测试总结 ==========
        print("\n" + "="*80)
        print("📊 测试总结")
        print("="*80)
        print("\n如果两个测试都通过，说明：")
        print("  ✅ TURN命令正常工作")
        print("  ✅ 参数设置正确")
        print("  ✅ 可以在test_all_modules.py中使用相同逻辑")
        print("\n如果测试失败，可能原因：")
        print("  ❌ 固件端TURN命令实现有问题")
        print("  ❌ 电机接线或PWM配置不对")
        print("  ❌ 位姿估计算法有误")
        print("="*80 + "\n")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断测试")
        robot.send_command("MODE,0\n")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        robot.send_command("MODE,0\n")
    finally:
        robot.disconnect()


if __name__ == '__main__':
    test_turn_90()

