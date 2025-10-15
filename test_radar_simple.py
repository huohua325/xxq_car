#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达三区域避障系统 - 简化测试脚本（纯串口版）
无需任何外部依赖，直接用pyserial测试

用法:
    python test_radar_simple.py
    
作者: AI Assistant
日期: 2025-10-15
版本: v2.0
"""

import serial
import time
import json
import re

# 配置
SERIAL_PORT = "COM5"        # 修改为您的串口号
SERIAL_BAUDRATE = 115200
TIMEOUT = 2


def test_raw_data(ser):
    """测试1：原始数据解析（命令R）"""
    print("\n" + "="*80)
    print("🧪 测试1：原始数据解析")
    print("="*80)
    
    # 清空缓冲区
    ser.reset_input_buffer()
    
    # 发送命令
    print("\n📤 发送命令: R")
    ser.write(b"R\n")
    ser.flush()
    
    # 接收数据
    print("\n📥 接收数据:\n")
    lines = []
    start_time = time.time()
    
    while time.time() - start_time < 15:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
                lines.append(line)
                
                # 检查结束标记
                if '========' in line and len(lines) > 10:
                    break
        time.sleep(0.01)
    
    # 分析结果
    valid_count = 0
    invalid_count = 0
    
    for line in lines:
        if 'SUMMARY' in line:
            match = re.search(r'Valid:\s*(\d+)\s*\|\s*Invalid:\s*(\d+)', line)
            if match:
                valid_count = int(match.group(1))
                invalid_count = int(match.group(2))
    
    if valid_count > 0:
        total = valid_count + invalid_count
        valid_rate = (valid_count / total) * 100 if total > 0 else 0
        
        print(f"\n📊 结果分析:")
        print(f"  有效点: {valid_count}/{total} ({valid_rate:.1f}%)")
        
        if valid_rate >= 80:
            print("\n✅ 测试通过：原始数据解析准确")
            return True
        else:
            print("\n❌ 测试失败：有效点比例过低")
            return False
    else:
        print("\n⚠️  未找到统计数据")
        return False


def test_obstacle_detection(ser):
    """测试2：三区域避障（命令O）"""
    print("\n" + "="*80)
    print("🧪 测试2：三区域避障数据")
    print("="*80)
    
    # 清空缓冲区
    ser.reset_input_buffer()
    
    # 发送命令
    print("\n📤 发送命令: O")
    ser.write(b"O\n")
    ser.flush()
    
    # 接收数据
    print("\n📥 接收数据:\n")
    lines = []
    start_time = time.time()
    json_lines = []
    in_json = False
    
    while time.time() - start_time < 20:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
                lines.append(line)
                
                # 提取JSON
                if line.startswith('{'):
                    in_json = True
                    json_lines = [line]
                elif in_json:
                    json_lines.append(line)
                    if line == '}':
                        in_json = False
                
                # 检查结束标记
                if '========' in line and len(lines) > 20:
                    break
        time.sleep(0.01)
    
    # 解析JSON
    if json_lines:
        json_str = '\n'.join(json_lines)
        try:
            data = json.loads(json_str)
            
            if data.get('type') == 'OBSTACLE':
                print(f"\n📊 结果分析:")
                print(f"  总点数: {data['total_points']}")
                print(f"  覆盖率: {data['coverage']:.1f}%")
                print(f"\n  前方: {data['front']['count']} 点, 最近 {data['front']['min_dist']:.2f}m")
                print(f"  左侧: {data['left']['count']} 点, 最近 {data['left']['min_dist']:.2f}m")
                print(f"  右侧: {data['right']['count']} 点, 最近 {data['right']['min_dist']:.2f}m")
                
                # 判断成功条件
                success = (
                    data['coverage'] >= 90 and
                    data['total_points'] >= 200 and
                    data['front']['count'] > 0 and
                    data['left']['count'] > 0 and
                    data['right']['count'] > 0
                )
                
                if success:
                    print("\n✅ 测试通过：三区域数据准确")
                    return True
                else:
                    print("\n❌ 测试失败：数据不符合要求")
                    return False
        except json.JSONDecodeError as e:
            print(f"\n⚠️  JSON解析错误: {e}")
    else:
        print("\n⚠️  未找到JSON数据")
    
    return False


def test_scan(ser):
    """测试3：正常扫描（命令A）"""
    print("\n" + "="*80)
    print("🧪 测试3：正常扫描（Python端使用）")
    print("="*80)
    
    # 清空缓冲区
    ser.reset_input_buffer()
    
    # 发送命令
    print("\n📤 发送命令: A")
    ser.write(b"A\n")
    ser.flush()
    
    # 接收数据
    print("\n📥 接收数据:\n")
    start_time = time.time()
    json_found = False
    
    while time.time() - start_time < 20:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                # 只打印JSON数据
                if line.startswith('{') or json_found:
                    print(line)
                    json_found = True
                    if line == '}':
                        print("\n✅ JSON数据接收完成")
                        return True
        time.sleep(0.01)
    
    if not json_found:
        print("\n⚠️  未收到JSON数据")
    
    return json_found


def main():
    """主程序"""
    print("\n" + "="*80)
    print("📡 雷达三区域避障系统 - 快速测试工具")
    print("="*80)
    
    # 输入串口号
    port = input(f"\n串口号 (回车使用默认 {SERIAL_PORT}): ").strip() or SERIAL_PORT
    
    try:
        # 打开串口
        print(f"\n🔌 连接串口: {port}...")
        ser = serial.Serial(port, SERIAL_BAUDRATE, timeout=TIMEOUT)
        print("✅ 串口已连接")
        time.sleep(1)  # 等待设备准备
        
        # 运行测试
        print("\n开始测试...\n")
        time.sleep(1)
        
        test1 = test_raw_data(ser)
        time.sleep(2)
        
        test2 = test_obstacle_detection(ser)
        time.sleep(2)
        
        # 可选：测试Python端接收
        choice = input("\n是否测试Python端JSON接收？(y/n): ").strip().lower()
        test3 = False
        if choice == 'y':
            test3 = test_scan(ser)
        
        # 总结
        print("\n" + "="*80)
        print("📊 测试总结")
        print("="*80)
        print(f"\n测试1 - 原始数据解析: {'✅ 通过' if test1 else '❌ 失败'}")
        print(f"测试2 - 三区域避障:   {'✅ 通过' if test2 else '❌ 失败'}")
        if choice == 'y':
            print(f"测试3 - JSON接收:      {'✅ 通过' if test3 else '❌ 失败'}")
        
        all_pass = test1 and test2
        
        if all_pass:
            print("\n🎉 所有测试通过！雷达系统工作正常")
        else:
            print("\n⚠️  部分测试失败，请检查硬件连接和固件")
        
        print("\n" + "="*80 + "\n")
        
        # 关闭串口
        ser.close()
        print("✅ 串口已关闭")
        
    except serial.SerialException as e:
        print(f"\n❌ 串口错误: {e}")
        print(f"\n请检查：")
        print(f"  1. 串口号是否正确（{port}）")
        print(f"  2. 设备是否已连接")
        print(f"  3. 串口是否被其他程序占用")
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断测试")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 再见！")

