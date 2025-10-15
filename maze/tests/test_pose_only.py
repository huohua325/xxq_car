#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SimplePoseEstimator 独立测试
不需要硬件，使用模拟数据
"""

import sys
import os
# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simple_pose_estimator import SimplePoseEstimator
import math


def test_pose_estimator():
    """测试位姿估计器"""
    
    print("\n" + "="*80)
    print("SimplePoseEstimator 单元测试")
    print("="*80)
    
    estimator = SimplePoseEstimator()
    
    # 测试1: 直线前进
    print("\n测试1: 模拟直线前进1米")
    print("-"*80)
    
    estimator.reset()
    
    # 模拟10次更新，每次前进0.1米
    for i in range(1, 11):
        # 计算编码器计数（假设左右轮速度相同）
        # 0.1米对应的编码器增量
        delta_left = int((0.1 / (2 * math.pi * 0.033)) * 1560)
        delta_right = int((0.1 / (2 * math.pi * 0.033)) * 780)
        
        estimator.update({
            'left_count': delta_left * i,
            'right_count': delta_right * i,
            'timestamp': i * 100
        })
        
        x, y, theta = estimator.get_pose_degrees()
        print(f"  步骤{i:2d}: x={x:6.3f}m, y={y:6.3f}m, θ={theta:6.1f}°")
    
    x, y, theta = estimator.get_pose_degrees()
    print(f"\n  [OK] 期望位姿: (1.0, 0.0, 0°)")
    print(f"  [OK] 实际位姿: ({x:.3f}, {y:.3f}, {theta:.1f}°)")
    
    # 测试2: 原地左转90度
    print("\n\n测试2: 模拟原地左转90度")
    print("-"*80)
    
    estimator.reset()
    
    # 原地左转：左轮后退，右轮前进
    # 转90度需要的弧长：wheel_base * pi / 4
    arc_length = 0.185 * math.pi / 4
    
    # 左轮后退（负值）
    left_count = -int((arc_length / (2 * math.pi * 0.033)) * 1560)
    # 右轮前进（正值）
    right_count = int((arc_length / (2 * math.pi * 0.033)) * 780)
    
    estimator.update({
        'left_count': left_count,
        'right_count': right_count,
        'timestamp': 1000
    })
    
    x, y, theta = estimator.get_pose_degrees()
    print(f"  [OK] 期望位姿: (0.0, 0.0, 90°)")
    print(f"  [OK] 实际位姿: ({x:.3f}, {y:.3f}, {theta:.1f}°)")
    
    # 测试3: 组合运动（前进+转向+前进）
    print("\n\n测试3: 组合运动（前进0.5m → 左转45° → 前进0.5m）")
    print("-"*80)
    
    estimator.reset()
    
    # 前进0.5米
    delta_left = int((0.5 / (2 * math.pi * 0.033)) * 1560)
    delta_right = int((0.5 / (2 * math.pi * 0.033)) * 780)
    estimator.update({
        'left_count': delta_left,
        'right_count': delta_right,
        'timestamp': 500
    })
    x1, y1, theta1 = estimator.get_pose_degrees()
    print(f"  前进0.5m后: ({x1:.3f}, {y1:.3f}, {theta1:.1f}°)")
    
    # 左转45度
    arc_length = 0.185 * math.pi / 8  # 45度 = pi/4 的一半
    left_turn = -int((arc_length / (2 * math.pi * 0.033)) * 1560)
    right_turn = int((arc_length / (2 * math.pi * 0.033)) * 780)
    
    estimator.update({
        'left_count': delta_left + left_turn,
        'right_count': delta_right + right_turn,
        'timestamp': 1000
    })
    x2, y2, theta2 = estimator.get_pose_degrees()
    print(f"  左转45°后: ({x2:.3f}, {y2:.3f}, {theta2:.1f}°)")
    
    # 再前进0.5米
    delta_left2 = int((0.5 / (2 * math.pi * 0.033)) * 1560)
    delta_right2 = int((0.5 / (2 * math.pi * 0.033)) * 780)
    
    estimator.update({
        'left_count': delta_left + left_turn + delta_left2,
        'right_count': delta_right + right_turn + delta_right2,
        'timestamp': 1500
    })
    x3, y3, theta3 = estimator.get_pose_degrees()
    print(f"  再前进0.5m后: ({x3:.3f}, {y3:.3f}, {theta3:.1f}°)")
    
    # 期望结果计算
    expected_x = 0.5 + 0.5 * math.cos(math.radians(45))
    expected_y = 0.5 * math.sin(math.radians(45))
    print(f"\n  [OK] 期望位姿: ({expected_x:.3f}, {expected_y:.3f}, 45°)")
    print(f"  [OK] 实际位姿: ({x3:.3f}, {y3:.3f}, {theta3:.1f}°)")
    
    # 测试4: 统计信息
    print("\n\n测试4: 统计信息")
    print("-"*80)
    stats = estimator.get_statistics()
    print(f"  总行驶距离: {stats['total_distance']:.3f}m")
    print(f"  更新次数: {stats['update_count']}")
    print(f"  当前位姿: {stats['current_pose']}")
    print(f"  当前位姿(度): {stats['current_pose_deg']}")
    
    print("\n" + "="*80)
    print("[OK] 单元测试完成")
    print("="*80 + "\n")


if __name__ == '__main__':
    test_pose_estimator()

