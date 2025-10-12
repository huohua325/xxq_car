#!/usr/bin/env python3
"""
自动查找STM32连接的COM口
扫描所有蓝牙串口，找到响应"STM32 Ready"的那个
"""

import serial
import serial.tools.list_ports
import time

def find_bluetooth_ports():
    """查找所有蓝牙串口"""
    ports = serial.tools.list_ports.comports()
    bluetooth_ports = []
    
    for port in ports:
        # 查找蓝牙相关的串口
        if 'Bluetooth' in port.description or 'BT' in port.description:
            bluetooth_ports.append(port)
    
    return bluetooth_ports

def test_port(port_name, baudrate=9600, timeout=2):
    """测试指定端口是否是STM32"""
    try:
        print(f"  正在测试 {port_name}...", end=' ')
        ser = serial.Serial(port_name, baudrate, timeout=timeout)
        time.sleep(0.5)  # 等待连接稳定
        
        # 清空缓冲区
        ser.reset_input_buffer()
        
        # 发送测试命令
        ser.write(b'MODE,0\n')
        time.sleep(0.5)
        
        # 读取响应
        response = ""
        if ser.in_waiting:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        
        ser.close()
        
        # 检查是否是STM32的响应
        if "[Python CMD]" in response or "Robot" in response or "STM32" in response:
            print("✅ 找到了！")
            return True, response
        else:
            print("❌ 无响应")
            return False, ""
            
    except serial.SerialException as e:
        print(f"❌ 无法打开 ({e})")
        return False, ""
    except Exception as e:
        print(f"❌ 错误 ({e})")
        return False, ""

def main():
    print("=" * 60)
    print("自动查找STM32连接的COM口")
    print("=" * 60)
    
    # 1. 列出所有串口
    print("\n[步骤1] 扫描所有串口...")
    all_ports = serial.tools.list_ports.comports()
    
    if not all_ports:
        print("❌ 未找到任何串口设备")
        print("\n请检查：")
        print("  1. 蓝牙是否已配对")
        print("  2. 蓝牙设备是否已连接（不只是配对）")
        return
    
    print(f"找到 {len(all_ports)} 个串口设备：")
    for port in all_ports:
        print(f"  - {port.device}: {port.description}")
    
    # 2. 筛选蓝牙串口
    print("\n[步骤2] 筛选蓝牙串口...")
    bt_ports = find_bluetooth_ports()
    
    if not bt_ports:
        print("⚠️  未找到蓝牙串口")
        print("\n建议：")
        print("  1. 在蓝牙设置中点击HC-04")
        print("  2. 点击'连接'按钮")
        print("  3. 等待几秒后重新运行此脚本")
        
        # 尝试所有串口
        print("\n尝试测试所有串口...")
        bt_ports = all_ports
    else:
        print(f"找到 {len(bt_ports)} 个蓝牙串口：")
        for port in bt_ports:
            print(f"  - {port.device}: {port.description}")
    
    # 3. 测试每个蓝牙串口
    print("\n[步骤3] 测试每个串口...")
    print("（发送MODE,0命令，看哪个有STM32响应）")
    print()
    
    found_port = None
    found_response = ""
    
    for port in bt_ports:
        is_stm32, response = test_port(port.device)
        if is_stm32:
            found_port = port.device
            found_response = response
            break
    
    # 4. 显示结果
    print("\n" + "=" * 60)
    if found_port:
        print("🎉 找到STM32了！")
        print("=" * 60)
        print(f"\n✅ 正确的COM口: {found_port}")
        print(f"\nSTM32响应内容:")
        print("-" * 60)
        print(found_response)
        print("-" * 60)
        
        print(f"\n接下来请使用: {found_port}")
        print("\n在测试脚本中使用此COM口：")
        print(f"  python scripts/test_stage1_communication.py")
        print(f"  然后输入端口号: {found_port}")
        
    else:
        print("❌ 未找到STM32响应")
        print("=" * 60)
        print("\n可能的原因：")
        print("  1. STM32固件未烧录或未启动")
        print("  2. 蓝牙已配对但未连接")
        print("  3. 波特率不匹配（应为9600）")
        print("  4. STM32 UART4接线错误")
        
        print("\n排查步骤：")
        print("  1. 确认STM32已上电且运行")
        print("  2. 在蓝牙设置中点击HC-04 → 连接")
        print("  3. 检查TX/RX接线是否交叉")
        print("  4. 重新烧录固件")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")


