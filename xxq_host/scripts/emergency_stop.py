#!/usr/bin/env python3
"""
🚨 紧急停止脚本
快速停止正在运动的机器人
"""

import sys
import os
import time

# 添加项目根目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
sys.path.insert(0, project_root)

from src.communication.robot_comm_ble import RobotCommBLE

# 你的BLE地址
DEFAULT_ADDRESS = "C4:25:01:20:02:8E"

def emergency_stop(address: str):
    """紧急停止机器人"""
    print("\n" + "=" * 60)
    print("🚨 紧急停止机器人")
    print("=" * 60)
    print(f"BLE地址: {address}")
    print()
    
    try:
        print("[1/4] 连接BLE...")
        comm = RobotCommBLE(address=address)
        
        if not comm.start():
            print("  ❌ BLE连接失败")
            print("\n请检查：")
            print("  1. HC-04BLE是否已上电")
            print("  2. BLE地址是否正确")
            print("  3. 是否被其他设备连接")
            return False
        
        print("  ✅ BLE已连接")
        
        print("\n[2/4] 发送紧急停止命令...")
        # 连续发送5次停止命令（确保至少一次成功）
        for i in range(5):
            success = comm.send_mode_command(0)
            print(f"  → 停止命令 {i+1}/5: {'✅' if success else '❌'}")
            time.sleep(0.2)
        
        print("\n[3/4] 等待机器人停止...")
        time.sleep(1.5)
        
        print("\n[4/4] 断开BLE连接...")
        comm.stop()
        print("  ✅ BLE已断开")
        
        print("\n" + "=" * 60)
        print("✅ 紧急停止完成！")
        print("=" * 60)
        print("\n机器人应该已经停止。")
        print("如果仍在运动，请：")
        print("  1. 断开STM32电源")
        print("  2. 或重新运行此脚本")
        
        return True
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        print("\n如果机器人仍在运动，请立即断开STM32电源！")
        return False

if __name__ == '__main__':
    print("\n" + "=" * 60)
    print("紧急停止工具")
    print("=" * 60)
    
    # 检查bleak
    try:
        import bleak
    except ImportError:
        print("\n❌ 未安装bleak库")
        print("请运行: pip install bleak")
        sys.exit(1)
    
    # 获取地址
    print(f"\n默认地址: {DEFAULT_ADDRESS}")
    address = input("请输入BLE地址（直接回车使用默认）: ").strip()
    
    if not address:
        address = DEFAULT_ADDRESS
    
    # 执行停止
    success = emergency_stop(address)
    sys.exit(0 if success else 1)




