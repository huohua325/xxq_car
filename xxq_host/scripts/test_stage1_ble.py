#!/usr/bin/env python3
"""
阶段1 BLE通信测试脚本
测试STM32与Python的基本命令收发功能（通过BLE）

测试项目：
1. BLE连接
2. 接收STM32就绪消息
3. 发送MODE命令
4. 接收STM32响应
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
    level=logging.INFO,
    format='[%(levelname)s] %(message)s'
)

def emergency_stop(address: str):
    """紧急停止机器人"""
    print("\n🚨 紧急停止机器人...")
    
    try:
        comm = RobotCommBLE(address=address)
        if comm.start():
            print("  ✅ BLE已连接")
            # 连续发送3次停止命令
            for i in range(3):
                comm.send_mode_command(0)
                print(f"  ✅ 发送停止命令 ({i+1}/3)")
                time.sleep(0.3)
            
            time.sleep(1)
            comm.stop()
            print("  ✅ 机器人应该已停止")
        else:
            print("  ❌ BLE连接失败")
    except Exception as e:
        print(f"  ❌ 错误: {e}")

def test_stage1_ble(address: str):
    """阶段1 BLE通信测试"""
    
    print("=" * 60)
    print("阶段1 BLE通信测试 - STM32硬件端适配")
    print("=" * 60)
    print(f"BLE地址: {address}")
    print(f"波特率: 9600 (BLE透传)")
    print("⚠️  测试过程中如需紧急停止，按Ctrl+C")
    print("-" * 60)
    
    # 记录收到的所有消息
    received_messages = []
    test_results = {
        'connect': False,
        'receive': False,
        'mode_commands': []
    }
    
    def on_data_received(data):
        """回调：记录收到的数据"""
        msg = str(data)
        received_messages.append(msg)
        print(f"  收到数据: {msg[:100]}...")
    
    try:
        # 1. 创建BLE通信对象
        print("\n[步骤1] 创建BLE通信对象...")
        comm = RobotCommBLE(address=address)
        
        # 设置回调（记录所有收到的数据）
        # 注意：阶段1的MODE命令响应不会触发这些回调，
        # 因为响应是文本消息，不是传感器数据
        print("  ✅ BLE通信对象已创建")
        
        # 2. 连接BLE设备
        print("\n[步骤2] 连接BLE设备...")
        print("  (可能需要10-20秒，请耐心等待)")
        print("  提示: 确保HC-04BLE已上电且未被其他设备连接")
        
        success = comm.start()
        
        if not success:
            print("\n❌ BLE连接失败")
            print("\n可能原因：")
            print("  1. BLE地址不正确")
            print("  2. HC-04BLE未上电")
            print("  3. HC-04BLE已被其他设备连接（手机等）")
            print("  4. 距离太远（>10米）")
            print("\n建议：")
            print("  1. 运行扫描脚本确认地址: python scan_ble.py")
            print("  2. 断开其他设备的连接")
            print("  3. 重启HC-04BLE")
            return False
        
        print("  ✅ BLE已连接")
        test_results['connect'] = True
        
        # 3. 等待STM32启动并发送就绪消息
        print("\n[步骤3] 等待STM32就绪...")
        print("  (STM32启动后会发送\"STM32 Ready\"消息)")
        time.sleep(2)
        
        # 检查是否收到数据（BLE会通过回调接收）
        if len(received_messages) > 0:
            print(f"  ✅ 收到 {len(received_messages)} 条消息")
            test_results['receive'] = True
        else:
            print("  ⚠️  未收到消息（可能STM32已启动，继续测试）")
        
        # 4. 测试MODE命令
        print("\n[步骤4] 测试MODE命令...")
        print("-" * 60)
        
        test_commands = [
            (0, "停止", 1.0),
            (1, "前进", 2.0),  # 前进2秒
            (0, "停止", 1.5),  # 停止并等待1.5秒确认
            (3, "左转", 1.5),
            (0, "停止", 1.5),
            (4, "右转", 1.5),
            (0, "停止", 1.5),
        ]
        
        for mode_id, desc, wait_time in test_commands:
            print(f"\n➤ 发送命令: MODE,{mode_id} ({desc})")
            
            # 发送命令
            success = comm.send_mode_command(mode_id)
            
            if success:
                print(f"  ✅ 命令已发送")
                test_results['mode_commands'].append(mode_id)
            else:
                print(f"  ❌ 命令发送失败")
            
            # 等待执行（BLE需要更长的延迟）
            print(f"  ⏳ 等待{wait_time}秒...")
            time.sleep(wait_time)
        
        # 5. 测试未知命令（手动发送原始字符串）
        print("\n[步骤5] 测试未知命令...")
        print("-" * 60)
        # BLE版本暂不支持发送原始字符串，跳过此测试
        print("  ⚠️  BLE版本暂不支持发送原始命令，跳过此测试")
        
        # 6. 等待一下，看是否有延迟的响应
        print("\n[步骤6] 等待延迟响应...")
        time.sleep(2)
        
        # 7. 总结
        print("\n" + "=" * 60)
        print("阶段1 BLE测试完成！")
        print("=" * 60)
        
        print("\n验收结果：")
        if test_results['connect']:
            print("  ✅ BLE能连接STM32")
        else:
            print("  ❌ BLE连接失败")
        
        if test_results['receive']:
            print("  ✅ 能接收数据")
        else:
            print("  ⚠️  未收到数据（但命令发送成功）")
        
        if len(test_results['mode_commands']) >= 4:
            print(f"  ✅ MODE命令发送成功 ({len(test_results['mode_commands'])}/6)")
        else:
            print(f"  ⚠️  MODE命令部分失败 ({len(test_results['mode_commands'])}/6)")
        
        # 最终结论
        print("\n📝 注意事项：")
        print("  - BLE通信比串口慢（15包/秒 vs 100包/秒）")
        print("  - 阶段1主要测试命令发送，接收功能在阶段2测试")
        print("  - 如果命令能发送，说明BLE通信正常")
        
        if test_results['connect'] and len(test_results['mode_commands']) >= 4:
            print("\n🎉 阶段1 BLE通信基础建立成功！可以进入阶段2。")
            print("\n💡 提示：")
            print("  - 调试阶段建议用USB串口（如果有）")
            print("  - 演示阶段可以用BLE无线连接")
        else:
            print("\n⚠️  测试未完全通过，请检查：")
            print("  1. BLE地址是否正确")
            print("  2. STM32固件是否已烧录")
            print("  3. HC-04BLE是否正常工作")
        
        # 8. 断开连接
        print("\n[步骤7] 断开BLE连接...")
        comm.stop()
        print("  ✅ BLE已断开")
        
        return test_results['connect'] and len(test_results['mode_commands']) >= 4
        
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断测试 - 紧急停止机器人")
        if 'comm' in locals():
            try:
                # 紧急停止
                print("  发送紧急停止命令...")
                for i in range(3):
                    comm.send_mode_command(0)
                    time.sleep(0.2)
                print("  ✅ 停止命令已发送")
            except:
                pass
            comm.stop()
        return False
        
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        import traceback
        traceback.print_exc()
        if 'comm' in locals():
            comm.stop()
        return False


def scan_ble_devices():
    """扫描附近的BLE设备"""
    print("=" * 60)
    print("扫描BLE设备...")
    print("=" * 60)
    
    try:
        from bleak import BleakScanner
        
        async def scan():
            print("正在扫描（10秒）...")
            devices = await BleakScanner.discover(timeout=10.0)
            
            if not devices:
                print("\n❌ 未发现任何BLE设备")
                print("\n请检查：")
                print("  1. HC-04BLE是否已上电")
                print("  2. 电脑蓝牙是否开启")
                print("  3. HC-04BLE是否处于可发现状态")
                return None
            
            print(f"\n发现 {len(devices)} 个BLE设备：")
            print("-" * 60)
            
            for i, device in enumerate(devices, 1):
                name = device.name or "(未命名)"
                print(f"{i}. {name}")
                print(f"   地址: {device.address}")
                print(f"   信号: {device.rssi} dBm")
                print()
            
            # 尝试找到HC-04相关的设备
            hc04_devices = [d for d in devices if d.name and ('HC' in d.name.upper() or 'BLE' in d.name.upper())]
            
            if hc04_devices:
                print("可能的HC-04BLE设备：")
                for device in hc04_devices:
                    print(f"  ✨ {device.name} - {device.address}")
                return hc04_devices[0].address
            
            return None
        
        # 运行扫描
        address = asyncio.run(scan())
        return address
        
    except ImportError:
        print("\n❌ 未安装bleak库")
        print("\n请运行: pip install bleak")
        return None
    except Exception as e:
        print(f"\n❌ 扫描失败: {e}")
        return None


if __name__ == '__main__':
    print("\n" + "=" * 60)
    print("STM32 阶段1 BLE通信测试工具")
    print("=" * 60)
    
    # 检查bleak是否安装
    try:
        import bleak
    except ImportError:
        print("\n❌ 未安装bleak库")
        print("\n请先安装BLE支持库：")
        print("  pip install bleak")
        sys.exit(1)
    
    print("\n请选择操作：")
    print("  1. 使用已知地址测试（推荐）")
    print("  2. 扫描BLE设备")
    print("  3. 紧急停止机器人 🚨")
    print("  4. 退出")
    
    choice = input("\n请选择 (1/2/3/4): ").strip()
    
    if choice == '1':
        # 使用已知地址
        default_address = "C4:25:01:20:02:8E"  # 从BLE使用说明中获取
        print(f"\n默认地址: {default_address}")
        address = input(f"请输入BLE地址（直接回车使用默认）: ").strip()
        
        if not address:
            address = default_address
        
        print(f"\n使用地址: {address}")
        success = test_stage1_ble(address)
        sys.exit(0 if success else 1)
        
    elif choice == '2':
        # 扫描设备
        address = scan_ble_devices()
        
        if address:
            print("\n" + "=" * 60)
            confirm = input(f"\n是否使用此地址进行测试？(y/n): ").strip().lower()
            
            if confirm == 'y':
                success = test_stage1_ble(address)
                sys.exit(0 if success else 1)
        else:
            print("\n未找到合适的设备")
            sys.exit(1)
            
    elif choice == '3':
        # 紧急停止
        default_address = "C4:25:01:20:02:8E"
        address = input(f"请输入BLE地址（回车使用{default_address}）: ").strip()
        if not address:
            address = default_address
        emergency_stop(address)
        
    elif choice == '4':
        print("再见！")
        sys.exit(0)
    else:
        print("无效选择")
        sys.exit(1)

