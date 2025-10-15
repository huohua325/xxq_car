#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
集成测试：测试所有三个模块
位姿估计器 + 雷达包装器 + 运动控制器
"""

import sys
import os
import time
# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ble_robot_control import SimpleBLERobotComm
from simple_pose_estimator import SimplePoseEstimator
from simple_lidar import SimpleLidarWrapper
from simple_motion import SimpleMotionController

# 配置
BLE_ADDRESS = "C4:25:01:20:02:8E"


def test_integrated():
    """集成测试"""
    
    print("\n" + "="*80)
    print("🧪 阶段二模块集成测试")
    print("="*80)
    print("\n测试内容：")
    print("  1. SimplePoseEstimator - 位姿估计器")
    print("  2. SimpleLidarWrapper - 雷达数据包装器")
    print("  3. SimpleMotionController - 运动控制包装器")
    print("\n测试序列：")
    print("  前进 → 扫描 → 左转45° → 前进 → 扫描 → 右转45° → 前进 → 扫描")
    print("="*80 + "\n")
    
    choice = input("⚠️ 此测试需要真实硬件，是否继续？(y/n): ").strip().lower()
    if choice != 'y':
        print("测试取消")
        return
    
    robot = SimpleBLERobotComm(address=BLE_ADDRESS, verbose=False)
    
    # 创建三个模块
    pose_estimator = SimplePoseEstimator()
    lidar = SimpleLidarWrapper(robot)
    motion = SimpleMotionController(robot)
    
    # 位姿历史
    pose_history = []
    
    # ODO数据回调
    def on_odo_update(data):
        """ODO数据回调，更新位姿"""
        pose_estimator.update(data)
        # 实时显示
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
        
        # ========== 测试序列 ==========
        
        # 步骤1: 前进
        print("\n" + "="*80)
        print("步骤1: 前进（medium步长）")
        print("="*80)
        dist = motion.forward('medium')
        print(f"\n  ✅ 前进完成，距离: {dist:.3f}m")
        
        # ⭐ 关键：增加稳定时间，确保PID模式完全清除
        print("  ⏳ 稳定中...（1秒）")
        time.sleep(1.0)
        
        # 记录位姿
        pose = pose_estimator.get_pose_degrees()
        pose_history.append(('前进', pose))
        print(f"  当前位姿: {pose}")
        
        # 扫描
        print("\n  🔄 雷达扫描...")
        print(f"  📤 发送扫描命令: A")
        scan = lidar.request_scan(timeout=6.0)  # ⭐ 增加到10秒
        if scan:
            print(f"  ✅ 扫描成功，点数: {len(scan)}")
            obstacles = lidar.get_obstacles_summary()
            print(f"  障碍物: 前={obstacles['front']:.2f}m, 左={obstacles['left']:.2f}m, 右={obstacles['right']:.2f}m")
        else:
            print(f"  ⚠️ 扫描超时（6秒）")
            print(f"  💡 提示: 固件端可能未响应'A'命令，或雷达硬件未连接")
        
        # ⭐ 关键：转向前需要更长时间，确保PID完全停止
        print("  ⏳ 准备转向...（2秒）")
        time.sleep(2.0)
        
        # 步骤2: 左转45度
        print("\n" + "="*80)
        print("步骤2: 左转45度")
        print("="*80)
        # 使用calibrate_turn.py验证过的参数
        # 左转90度: TURN,0,0.85,0.18, 1.0秒
        # 左转45度: 时间减半 = 0.5秒
        duration = 0.5
        turn_cmd = "TURN,0,0.85,0.18\n"
        print(f"  📤 发送命令: {turn_cmd.strip()}")
        robot.send_command(turn_cmd)
        print(f"  ⏳ 转向中...（{duration}秒）")
        time.sleep(duration)  # 一次性等待，与test_turn_only.py一致
        print(f"  📤 发送停止命令: MODE,0")
        robot.send_command("MODE,0\n")
        time.sleep(0.5)  # 停止后稳定时间
        print(f"\n  ✅ 左转完成，角度: 45°")
        
        # ⭐ 等待惯性消失和位姿稳定
        print(f"  ⏳ 等待位姿稳定...（1秒）")
        time.sleep(1.0)
        
        # 记录位姿
        pose = pose_estimator.get_pose_degrees()
        pose_history.append(('左转45°', pose))
        print(f"  当前位姿: {pose}")
        
        time.sleep(1)
        
        # 步骤3: 前进
        print("\n" + "="*80)
        print("步骤3: 前进（medium步长）")
        print("="*80)
        dist = motion.forward('medium')
        print(f"\n  ✅ 前进完成，距离: {dist:.3f}m")
        
        # ⭐ 关键：增加稳定时间
        print("  ⏳ 稳定中...（1秒）")
        time.sleep(1.0)
        
        # 记录位姿
        pose = pose_estimator.get_pose_degrees()
        pose_history.append(('前进', pose))
        print(f"  当前位姿: {pose}")
        
        # 扫描
        print("\n  🔄 雷达扫描...")
        print(f"  📤 发送扫描命令: A")
        scan = lidar.request_scan(timeout=6.0)  # ⭐ 增加到10秒
        if scan:
            print(f"  ✅ 扫描成功，点数: {len(scan)}")
            lidar.visualize_scan()
        else:
            print(f"  ⚠️ 扫描超时（6秒）")
            print(f"  💡 提示: 固件端可能未响应'A'命令，或雷达硬件未连接")
        
        # ⭐ 关键：转向前需要更长时间，确保PID完全停止
        print("  ⏳ 准备转向...（2秒）")
        time.sleep(2.0)
        
        # 步骤4: 右转45度
        print("\n" + "="*80)
        print("步骤4: 右转45度")
        print("="*80)
        # 使用calibrate_turn.py验证过的参数
        # 右转90度: TURN,1,0.50,0.45, 0.73秒
        # 右转45度: 时间减半 = 0.365秒
        duration = 0.365
        turn_cmd = "TURN,1,0.50,0.45\n"
        print(f"  📤 发送命令: {turn_cmd.strip()}")
        robot.send_command(turn_cmd)
        print(f"  ⏳ 转向中...（{duration}秒）")
        time.sleep(duration)  # 一次性等待，与test_turn_only.py一致
        print(f"  📤 发送停止命令: MODE,0")
        robot.send_command("MODE,0\n")
        time.sleep(0.5)  # 停止后稳定时间
        print(f"\n  ✅ 右转完成，角度: 45°")
        
        # ⭐ 等待惯性消失和位姿稳定
        print(f"  ⏳ 等待位姿稳定...（1秒）")
        time.sleep(1.0)
        
        # 记录位姿
        pose = pose_estimator.get_pose_degrees()
        pose_history.append(('右转45°', pose))
        print(f"  当前位姿: {pose}")
        
        time.sleep(1)
        
        # 步骤5: 前进
        print("\n" + "="*80)
        print("步骤5: 前进（long步长）")
        print("="*80)
        dist = motion.forward('long')
        print(f"\n  ✅ 前进完成，距离: {dist:.3f}m")
        
        # ⭐ 关键：增加稳定时间
        print("  ⏳ 稳定中...（1秒）")
        time.sleep(1.0)
        
        # 记录位姿
        pose = pose_estimator.get_pose_degrees()
        pose_history.append(('前进', pose))
        print(f"  当前位姿: {pose}")
        
        # 扫描
        print("\n  🔄 雷达扫描...")
        print(f"  📤 发送扫描命令: A")
        scan = lidar.request_scan(timeout=6.0)  # ⭐ 增加到6秒
        if scan:
            print(f"  ✅ 扫描成功，点数: {len(scan)}")
            obstacles = lidar.get_obstacles_summary()
            print(f"  障碍物: 前={obstacles['front']:.2f}m, 左={obstacles['left']:.2f}m, 右={obstacles['right']:.2f}m")
        else:
            print(f"  ⚠️ 扫描超时（6秒）")
            print(f"  💡 提示: 如果一直超时，可能是雷达硬件问题，可以跳过扫描步骤")
        
        # ========== 测试总结 ==========
        print("\n" + "="*80)
        print("📊 测试总结")
        print("="*80)
        
        # 位姿历史
        print("\n位姿历史:")
        print(f"  {'动作':<10} {'X (m)':<10} {'Y (m)':<10} {'θ (°)':<10}")
        print(f"  {'-'*40}")
        for action, (x, y, theta) in pose_history:
            print(f"  {action:<10} {x:<10.3f} {y:<10.3f} {theta:<10.1f}")
        
        # 位姿估计统计
        print("\n位姿估计统计:")
        stats = pose_estimator.get_statistics()
        print(f"  总行驶距离: {stats['total_distance']:.3f}m")
        print(f"  更新次数: {stats['update_count']}")
        
        # 运动统计
        print("\n运动统计:")
        motion_stats = motion.get_statistics()
        print(f"  总运动次数: {motion_stats['total_motions']}")
        for action, count in motion_stats['motion_count'].items():
            if count > 0:
                print(f"  {action}: {count}")
        
        # 雷达统计
        print("\n雷达统计:")
        print(f"  扫描次数: {lidar.get_scan_count()}")
        print(f"  最新扫描点数: {len(lidar.get_latest_scan()) if lidar.has_scan() else 0}")
        
        # 模块状态
        print("\n模块状态:")
        print(f"  位姿估计器: {pose_estimator}")
        print(f"  雷达包装器: {lidar}")
        print(f"  运动控制器: {motion}")
        
        print("\n" + "="*80)
        print("✅ 集成测试完成！")
        print("="*80)
        print("\n💡 说明:")
        print("  - 位姿估计可能有累积误差（正常）")
        print("  - 转向角度可能不精确（±5度范围内可接受）")
        print("  - 雷达数据取决于周围环境")
        print("\n下一步: 开发墙跟随算法（阶段三）")
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
    test_integrated()

