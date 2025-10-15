#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达数据测试脚本
测试：360°激光雷达扫描功能
"""

import sys
import time
import json
sys.path.insert(0, '.')
from ble_robot_control import SimpleBLERobotComm

# 配置
BLE_ADDRESS = "C4:25:01:20:02:8E"
SCAN_COUNT = 5  # 测试扫描次数


def test_lidar():
    """雷达功能测试"""
    
    print("\n" + "="*80)
    print("📡 激光雷达测试")
    print("="*80)
    print(f"\n测试内容：")
    print(f"  - 发送'A'命令请求雷达扫描")
    print(f"  - 接收并解析JSON格式的雷达数据")
    print(f"  - 显示8个扇区的距离信息")
    print(f"  - 测试次数：{SCAN_COUNT}")
    print("="*80 + "\n")
    
    robot = SimpleBLERobotComm(address=BLE_ADDRESS, verbose=False)
    
    # 用于收集雷达数据
    lidar_data_received = []
    scan_results = []
    
    def on_lidar_update(data):
        """雷达数据回调"""
        lidar_data_received.append(data)
        
        print(f"\n📡 雷达扫描完成 (时间戳: {data.timestamp}ms)")
        print(f"  总点数: {data.total_points}")
        print(f"  角度覆盖: {data.angle_coverage}°")
        print(f"\n  扇区详情:")
        print(f"  {'扇区':<6} {'中心角':<8} {'点数':<6} {'最小距离':<10} {'平均距离':<10}")
        print(f"  {'-'*50}")
        
        for sector in data.sectors:
            sector_id = sector.get('sector_id', -1)
            angle = sector.get('angle_center', 0)
            count = sector.get('count', 0)
            min_dist = sector.get('min_dist', 0)
            avg_dist = sector.get('avg_dist', 0)
            
            # 根据距离显示警告
            warning = ""
            if min_dist > 0 and min_dist < 0.3:
                warning = " ⚠️ 近距离障碍"
            elif count == 0:
                warning = " ❌ 无数据"
            
            print(f"  {sector_id:<6} {angle:<8}° {count:<6} {min_dist:<10.2f}m {avg_dist:<10.2f}m{warning}")
        
        # 保存结果
        scan_results.append({
            'timestamp': data.timestamp,
            'total_points': data.total_points,
            'sectors': data.sectors
        })
    
    robot.on_lidar_update = on_lidar_update
    
    try:
        # 连接BLE
        print("🔵 连接BLE...")
        if not robot.connect():
            print("❌ 连接失败")
            return
        print("✅ BLE已连接\n")
        time.sleep(1)
        
        # 执行多次扫描测试
        for i in range(SCAN_COUNT):
            print("\n" + "="*80)
            print(f"📍 第 {i+1}/{SCAN_COUNT} 次扫描")
            print("="*80)
            
            lidar_data_received.clear()
            
            # 发送雷达扫描请求
            print("  发送命令: 'A\\n' (雷达扫描请求)")
            robot.send_command("A\n")  # 所有命令都需要\n才能被STM32识别
            
            # 等待雷达数据
            print("  ⏳ 等待雷达数据...")
            timeout = 5.0  # 5秒超时
            start_time = time.time()
            
            while len(lidar_data_received) == 0:
                if time.time() - start_time > timeout:
                    print("\n  ❌ 超时：未收到雷达数据（5秒）")
                    break
                time.sleep(0.1)
            
            if len(lidar_data_received) > 0:
                print("\n  ✅ 雷达数据接收成功")
            
            # 间隔1秒再进行下次扫描
            if i < SCAN_COUNT - 1:
                time.sleep(1)
        
        # ========== 总结报告 ==========
        print("\n" + "="*80)
        print("📊 测试总结")
        print("="*80)
        
        if len(scan_results) == 0:
            print("\n❌ 未接收到任何雷达数据")
            print("\n可能原因：")
            print("  1. 雷达硬件未连接或故障")
            print("  2. 固件中雷达扫描功能未启用")
            print("  3. 通信超时或BLE连接不稳定")
        else:
            print(f"\n✅ 成功接收 {len(scan_results)}/{SCAN_COUNT} 次扫描数据")
            
            # 统计分析
            total_points_list = [r['total_points'] for r in scan_results]
            avg_points = sum(total_points_list) / len(total_points_list)
            min_points = min(total_points_list)
            max_points = max(total_points_list)
            
            print(f"\n点数统计:")
            print(f"  平均点数: {avg_points:.1f}")
            print(f"  最少点数: {min_points}")
            print(f"  最多点数: {max_points}")
            
            # 扇区统计（统计有障碍物的扇区）
            sector_obstacle_count = [0] * 8  # 8个扇区
            
            for result in scan_results:
                for sector in result['sectors']:
                    sector_id = sector.get('sector_id', -1)
                    count = sector.get('count', 0)
                    if 0 <= sector_id < 8 and count > 0:
                        sector_obstacle_count[sector_id] += 1
            
            print(f"\n扇区障碍物检测率:")
            for i, count in enumerate(sector_obstacle_count):
                rate = (count / len(scan_results)) * 100
                angle_range = f"{i*45}°-{(i+1)*45}°"
                print(f"  扇区{i} ({angle_range:>10}): {count}/{len(scan_results)} ({rate:5.1f}%)")
            
            # 最近障碍物
            print(f"\n最近障碍物距离:")
            all_min_dists = []
            for result in scan_results:
                for sector in result['sectors']:
                    min_dist = sector.get('min_dist', 0)
                    if min_dist > 0:
                        all_min_dists.append(min_dist)
            
            if all_min_dists:
                print(f"  最近: {min(all_min_dists):.2f}m")
                print(f"  最远: {max(all_min_dists):.2f}m")
                print(f"  平均: {sum(all_min_dists)/len(all_min_dists):.2f}m")
            else:
                print(f"  ⚠️ 未检测到障碍物")
        
        print("\n" + "="*80)
        print("✅ 测试完成！")
        print("="*80 + "\n")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断测试")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()


def test_lidar_continuous():
    """连续雷达扫描测试（实时监控）"""
    
    print("\n" + "="*80)
    print("📡 雷达连续扫描测试（按Ctrl+C停止）")
    print("="*80 + "\n")
    
    robot = SimpleBLERobotComm(address=BLE_ADDRESS, verbose=False)
    
    scan_count = 0
    
    def on_lidar_update(data):
        """雷达数据回调"""
        nonlocal scan_count
        scan_count += 1
        
        # 简洁显示
        obstacle_sectors = []
        for sector in data.sectors:
            if sector.get('count', 0) > 0:
                sector_id = sector.get('sector_id', -1)
                min_dist = sector.get('min_dist', 0)
                obstacle_sectors.append(f"S{sector_id}:{min_dist:.2f}m")
        
        obstacles_str = ", ".join(obstacle_sectors) if obstacle_sectors else "无障碍物"
        
        print(f"[{scan_count:3d}] 点数:{data.total_points:3d} | 障碍: {obstacles_str}")
    
    robot.on_lidar_update = on_lidar_update
    
    try:
        print("🔵 连接BLE...")
        if not robot.connect():
            print("❌ 连接失败")
            return
        print("✅ BLE已连接\n")
        print("开始连续扫描...\n")
        time.sleep(1)
        
        # 连续发送扫描请求
        while True:
            robot.send_command("A\n")
            time.sleep(1)  # 每秒扫描一次
        
    except KeyboardInterrupt:
        print(f"\n\n⚠️ 停止扫描（共扫描{scan_count}次）")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
    finally:
        robot.disconnect()


if __name__ == '__main__':
    print("\n📡 激光雷达测试工具")
    print("选择模式:")
    print("  1. 基础测试（扫描5次，详细显示）")
    print("  2. 连续扫描（实时监控，按Ctrl+C停止）")
    print("  3. 退出")
    
    choice = input("\n请选择 (1/2/3): ").strip()
    
    if choice == '1':
        test_lidar()
    elif choice == '2':
        test_lidar_continuous()
    elif choice == '3':
        print("再见！")
    else:
        print("无效选择")

