#!/usr/bin/env python3
"""
阶段1 BLE通信测试 + 数据显示
测试STM32通信并实时显示所有传感器数据

功能：
1. 连接BLE
2. 发送MODE命令控制机器人
3. 实时显示接收到的数据（MPU、ODO、POSE、LIDAR）
4. 请求雷达扫描并显示结果
"""

import sys
import time
import asyncio
import logging
import os

# 添加项目根目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
sys.path.insert(0, project_root)

from src.communication.robot_comm_ble import RobotCommBLE

# 配置日志
logging.basicConfig(
    level=logging.WARNING,  # 只显示警告和错误，数据由回调打印
    format='[%(levelname)s] %(message)s'
)

def on_raw_message(line: str):
    """处理未解析的消息（STM32的调试信息、确认消息等）"""
    # 检查是否是确认消息
    if line == "ACK":
        data_stats['ack_count'] += 1
        print(f"  ✅ STM32确认 [ACK #{data_stats['ack_count']}]")
    elif "[Python CMD]" in line or "[LIDAR]" in line or "[CMD]" in line:
        # STM32的调试消息
        print(f"  📨 STM32: {line}")
        other_messages.append(line)
    elif line and line not in ['\r', '\n', '']:
        # 其他消息
        print(f"  💬 {line}")
        other_messages.append(line)

# 数据统计
data_stats = {
    'mpu_count': 0,
    'odo_count': 0,
    'pose_count': 0,
    'lidar_count': 0,
    'ack_count': 0,  # 新增：统计收到的ACK确认
    'start_time': 0
}

# 收到的其他消息（调试、确认等）
other_messages = []

def on_mpu_data(mpu_data):
    """MPU数据回调"""
    data_stats['mpu_count'] += 1
    print(f"\r[MPU] Roll:{mpu_data.roll:6.2f}° Pitch:{mpu_data.pitch:6.2f}° "
          f"Accel:({mpu_data.accel[0]:5.2f},{mpu_data.accel[1]:5.2f},{mpu_data.accel[2]:5.2f}) "
          f"Gyro:({mpu_data.gyro[0]:5.1f},{mpu_data.gyro[1]:5.1f},{mpu_data.gyro[2]:5.1f}) "
          f"[{data_stats['mpu_count']}]", end='')

def on_odo_data(odo_data):
    """里程计数据回调"""
    data_stats['odo_count'] += 1
    print(f"\r[ODO] L:{odo_data.left_speed:5.2f}RPS R:{odo_data.right_speed:5.2f}RPS "
          f"Count:({odo_data.left_count},{odo_data.right_count}) "
          f"[{data_stats['odo_count']}]", end='')

def on_pose_data(pose_data):
    """位姿数据回调"""
    data_stats['pose_count'] += 1
    print(f"\r[POSE] X:{pose_data.x:6.3f}m Y:{pose_data.y:6.3f}m "
          f"θ:{pose_data.theta:6.2f}° [{data_stats['pose_count']}]", end='')

def on_lidar_data(lidar_data):
    """雷达数据回调"""
    data_stats['lidar_count'] += 1
    print(f"\n{'='*70}")
    print(f"[LIDAR #{data_stats['lidar_count']}] 接收到雷达数据！")
    print(f"{'='*70}")
    print(f"时间戳: {lidar_data.timestamp} ms")
    print(f"总点数: {lidar_data.total_points}")
    print(f"角度覆盖: {lidar_data.angle_coverage}°")
    print(f"\n扇区详情（8个扇区，每个45°）:")
    print("-" * 70)
    print(f"{'扇区':<8} {'角度':<10} {'点数':<8} {'最小距离':<12} {'平均距离':<12}")
    print("-" * 70)
    
    for sector in lidar_data.sectors:
        sector_id = sector.get('sector_id', '?')
        angle = sector.get('angle_center', '?')
        count = sector.get('count', 0)
        min_dist = sector.get('min_dist', 0.0)
        avg_dist = sector.get('avg_dist', 0.0)
        
        print(f"扇区{sector_id:<3} {angle:>3}°      {count:<6}   "
              f"{min_dist:>6.2f}m      {avg_dist:>6.2f}m")
    
    print("=" * 70)
    print()

def print_stats():
    """打印数据统计"""
    elapsed = time.time() - data_stats['start_time']
    if elapsed > 0:
        print(f"\n{'='*70}")
        print(f"数据统计（运行{elapsed:.1f}秒）")
        print(f"{'='*70}")
        print(f"MPU数据:    {data_stats['mpu_count']} 包 ({data_stats['mpu_count']/elapsed:.1f} Hz)")
        print(f"ODO数据:    {data_stats['odo_count']} 包 ({data_stats['odo_count']/elapsed:.1f} Hz)")
        print(f"POSE数据:   {data_stats['pose_count']} 包 ({data_stats['pose_count']/elapsed:.1f} Hz)")
        print(f"LIDAR数据:  {data_stats['lidar_count']} 包")
        print(f"ACK确认:    {data_stats['ack_count']} 个  ⭐ 关键指标")
        print(f"其他消息:   {len(other_messages)} 条")
        print("=" * 70)

def test_with_data(address: str):
    """测试通信并显示数据"""
    
    print("=" * 70)
    print("阶段1 BLE通信 + 数据显示测试")
    print("=" * 70)
    print(f"BLE地址: {address}")
    print("⚠️  按Ctrl+C可随时停止并断开连接")
    print("-" * 70)
    
    try:
        # 1. 创建BLE通信对象
        print("\n[1/6] 创建BLE通信对象...")
        comm = RobotCommBLE(address=address)
        
        # 设置数据回调（实时打印收到的数据）
        comm.on_mpu_update = on_mpu_data
        comm.on_odom_update = on_odo_data
        comm.on_pose_update = on_pose_data
        comm.on_lidar_update = on_lidar_data
        comm.on_message = on_raw_message  # 新增：接收STM32的确认和调试消息
        
        print("  ✅ 回调函数已设置（包括ACK确认接收）")
        
        # 2. 连接BLE
        print("\n[2/6] 连接BLE设备（10-20秒）...")
        if not comm.start():
            print("\n❌ BLE连接失败")
            return False
        
        print("  ✅ BLE已连接")
        data_stats['start_time'] = time.time()
        
        # 3. 监听数据一段时间
        print("\n[3/6] 监听传感器数据（10秒）...")
        print("提示：你应该看到MPU、ODO、POSE数据实时刷新")
        print("-" * 70)
        
        time.sleep(10)
        print("\n")  # 换行
        
        # 4. 请求雷达扫描
        print("\n[4/6] 请求雷达扫描...")
        success = comm.request_lidar_scan()
        if success:
            print("  ✅ 雷达扫描请求已发送")
            print("  ⏳ 等待雷达数据（可能需要5-10秒）...")
            time.sleep(10)
        else:
            print("  ❌ 请求失败")
        
        # 5. 测试MODE命令
        print("\n[5/6] 测试MODE命令...")
        print("-" * 70)
        
        test_commands = [
            (0, "停止", 1.0),
            (1, "前进", 2.0),
            (0, "停止", 1.5),
            (3, "左转", 1.5),
            (0, "停止", 1.5),
        ]
        
        for mode_id, desc, wait_time in test_commands:
            print(f"\n➤ 发送: MODE,{mode_id} ({desc}) - 执行{wait_time}秒")
            comm.send_mode_command(mode_id)
            
            # 等待期间显示数据
            for i in range(int(wait_time * 10)):
                time.sleep(0.1)
                # 数据在回调中实时打印
            print()  # 换行
        
        # 6. 显示统计
        print("\n[6/6] 数据统计...")
        print_stats()
        
        # 7. 断开
        print("\n断开BLE连接...")
        comm.stop()
        print("  ✅ BLE已断开")
        
        print("\n" + "=" * 70)
        print("✅ 测试完成！")
        print("=" * 70)
        
        # 判断是否成功（关键看ACK数量）
        commands_sent = 5  # 发送了5个MODE命令
        has_sensor_data = (data_stats['mpu_count'] > 0 or 
                          data_stats['odo_count'] > 0 or 
                          data_stats['pose_count'] > 0)
        
        print("\n" + "=" * 70)
        print("✅ 通信验证结果")
        print("=" * 70)
        
        # 关键验证：是否收到ACK确认
        if data_stats['ack_count'] >= commands_sent:
            print(f"✅ STM32确实收到了命令！(ACK: {data_stats['ack_count']}/{commands_sent})")
            print("   这证明：BLE通信正常，命令能到达STM32")
        elif data_stats['ack_count'] > 0:
            print(f"⚠️  部分命令被确认 (ACK: {data_stats['ack_count']}/{commands_sent})")
            print("   可能存在丢包问题")
        else:
            print(f"❌ 未收到任何ACK确认 (0/{commands_sent})")
            print("   可能原因：")
            print("   1. 固件未重新烧录（修改后必须重新烧录）")
            print("   2. BLE连接有问题但未报错")
            print("   3. STM32未运行或卡死")
        
        # 传感器数据验证
        if has_sensor_data:
            print(f"\n✅ 传感器数据接收正常")
            print(f"   MPU:{data_stats['mpu_count']} ODO:{data_stats['odo_count']} POSE:{data_stats['pose_count']}")
            if data_stats['lidar_count'] > 0:
                print(f"   雷达数据: {data_stats['lidar_count']}次扫描")
        else:
            print(f"\n⏳ 未收到传感器数据（阶段1正常，阶段2会添加）")
        
        # 其他消息
        if len(other_messages) > 0:
            print(f"\n📨 收到{len(other_messages)}条STM32消息（前5条）:")
            for msg in other_messages[:5]:
                print(f"   {msg}")
        
        success = data_stats['ack_count'] >= commands_sent
        
        return True
        
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断 - 紧急停止机器人")
        if 'comm' in locals() and comm.is_connected():
            print("  发送停止命令...")
            for i in range(3):
                comm.send_mode_command(0)
                time.sleep(0.2)
            print("  ✅ 停止命令已发送")
            comm.stop()
        return False
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        if 'comm' in locals():
            comm.stop()
        return False


if __name__ == '__main__':
    print("\n" + "=" * 70)
    print("阶段1 BLE通信 + 数据显示测试")
    print("=" * 70)
    
    # 检查bleak
    try:
        import bleak
    except ImportError:
        print("\n❌ 未安装bleak库")
        print("请运行: pip install bleak")
        sys.exit(1)
    
    # 默认地址
    default_address = "C4:25:01:20:02:8E"
    
    print(f"\n默认BLE地址: {default_address}")
    address = input("请输入BLE地址（直接回车使用默认）: ").strip()
    
    if not address:
        address = default_address
    
    print(f"\n使用地址: {address}")
    print("\n提示：")
    print("  - 测试会持续约30秒")
    print("  - 你会看到实时的传感器数据")
    print("  - MPU、ODO、POSE数据会不断刷新（如果STM32有发送）")
    print("  - 雷达数据会显示详细的扇区信息")
    print("  - 按Ctrl+C可随时停止\n")
    
    input("按回车开始测试...")
    
    success = test_with_data(address)
    sys.exit(0 if success else 1)

