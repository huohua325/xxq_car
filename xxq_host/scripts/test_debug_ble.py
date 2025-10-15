#!/usr/bin/env python3
"""
BLE通信调试脚本
深入诊断BLE数据接收问题
"""

import sys
import time
import os
import logging

# 添加项目根目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
sys.path.insert(0, project_root)

from src.communication.robot_comm_ble import RobotCommBLE

# 配置详细日志
logging.basicConfig(
    level=logging.DEBUG,  # 显示所有调试信息
    format='[%(asctime)s] %(name)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)

# 统计
received_raw_data = []
received_lines = []

def debug_notification_handler(sender, data: bytearray):
    """调试用：记录所有原始BLE数据"""
    decoded = data.decode('utf-8', errors='ignore')
    received_raw_data.append(decoded)
    print(f"\n🔵 BLE收到数据（{len(data)}字节）: {repr(decoded)}")

def on_any_message(line: str):
    """接收所有解析后的行"""
    received_lines.append(line)
    print(f"📨 解析行: {line}")

def main():
    address = "C4:25:01:20:02:8E"
    
    print("=" * 70)
    print("BLE通信深度调试")
    print("=" * 70)
    print(f"BLE地址: {address}")
    print("本脚本会显示所有BLE原始数据和解析结果")
    print("=" * 70)
    
    try:
        # 创建通信对象
        print("\n[1/4] 创建BLE通信对象...")
        comm = RobotCommBLE(address=address)
        
        # 设置所有回调
        comm.on_message = on_any_message
        comm.on_mpu_update = lambda d: print(f"📊 MPU数据: {d}")
        comm.on_odo_update = lambda d: print(f"📊 ODO数据: {d}")
        comm.on_pose_update = lambda d: print(f"📊 POSE数据: {d}")
        comm.on_lidar_update = lambda d: print(f"📊 LIDAR数据: {d.total_points}点")
        
        print("  ✅ 回调已设置")
        
        # 连接
        print("\n[2/4] 连接BLE...")
        if not comm.start():
            print("❌ BLE连接失败")
            return
        
        print("  ✅ BLE已连接")
        
        # 监听数据
        print("\n[3/4] 监听原始数据（10秒）...")
        print("（任何从STM32发来的数据都会显示）")
        print("-" * 70)
        
        time.sleep(10)
        
        # 发送测试命令
        print("\n[4/4] 发送测试命令...")
        print("-" * 70)
        
        print("\n➤ 发送: MODE,0")
        success = comm.send_mode_command(0)
        print(f"  发送结果: {'成功' if success else '失败'}")
        time.sleep(2)
        
        print("\n➤ 发送: A (雷达扫描)")
        success = comm.request_lidar_scan()
        print(f"  发送结果: {'成功' if success else '失败'}")
        time.sleep(5)
        
        # 统计
        print("\n" + "=" * 70)
        print("接收统计")
        print("=" * 70)
        print(f"原始BLE数据块: {len(received_raw_data)} 次")
        print(f"解析后的行: {len(received_lines)} 行")
        
        if len(received_raw_data) > 0:
            print(f"\n✅ 有收到数据！")
            print(f"\n前10块原始数据:")
            for i, data in enumerate(received_raw_data[:10], 1):
                print(f"  {i}. {repr(data)}")
        else:
            print(f"\n❌ 完全没收到数据！")
            print(f"\n可能原因：")
            print(f"  1. STM32没有通过UART4发送数据")
            print(f"  2. BLE模块TX线未连接到STM32 UART4")
            print(f"  3. STM32固件未运行或卡死")
            print(f"  4. BLE连接是假连接（配对了但没真正通信）")
        
        if len(received_lines) > 0:
            print(f"\n解析后的行:")
            for i, line in enumerate(received_lines[:10], 1):
                print(f"  {i}. {line}")
        
        # 断开
        print("\n断开BLE...")
        comm.stop()
        print("  ✅ 已断开")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断")
        if 'comm' in locals():
            comm.stop()
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        if 'comm' in locals():
            comm.stop()

if __name__ == '__main__':
    try:
        import bleak
    except ImportError:
        print("❌ 未安装bleak")
        print("请运行: pip install bleak")
        sys.exit(1)
    
    main()

