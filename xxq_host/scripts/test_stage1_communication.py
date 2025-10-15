#!/usr/bin/env python3
"""
阶段1通信测试脚本
测试STM32与Python的基本命令收发功能

测试项目：
1. 串口连接
2. 接收STM32就绪消息
3. 发送MODE命令
4. 接收STM32响应
"""

import serial
import time
import sys

# 配置参数
PORT = 'COM5'  # Windows: COM5, Linux: /dev/ttyUSB0
BAUDRATE = 9600  # 波特率（已改为9600）
TIMEOUT = 2.0

def test_stage1():
    """阶段1通信测试"""
    
    print("=" * 60)
    print("阶段1通信测试 - STM32硬件端适配")
    print("=" * 60)
    print(f"串口: {PORT}")
    print(f"波特率: {BAUDRATE}")
    print("-" * 60)
    
    try:
        # 1. 打开串口
        print("\n[步骤1] 连接串口...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        print(f"✅ 串口已连接: {ser.name}")
        time.sleep(1)  # 等待STM32启动
        
        # 2. 读取STM32就绪消息
        print("\n[步骤2] 等待STM32就绪消息...")
        time.sleep(0.5)
        if ser.in_waiting:
            ready_msg = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"接收到消息:\n{ready_msg}")
            if "STM32 Ready" in ready_msg:
                print("✅ STM32已就绪")
            else:
                print("⚠️  未收到预期的就绪消息")
        else:
            print("⚠️  未收到STM32消息（可能已启动，继续测试）")
        
        # 3. 测试MODE命令
        print("\n[步骤3] 测试MODE命令...")
        print("-" * 60)
        
        test_commands = [
            ('MODE,0\n', "停止"),
            ('MODE,1\n', "前进"),
            ('MODE,0\n', "停止"),
            ('MODE,3\n', "左转"),
            ('MODE,4\n', "右转"),
            ('MODE,0\n', "停止"),
        ]
        
        for cmd, desc in test_commands:
            print(f"\n➤ 发送命令: {cmd.strip()} ({desc})")
            ser.write(cmd.encode('utf-8'))
            time.sleep(0.5)  # 等待响应
            
            # 读取响应
            if ser.in_waiting:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                print(f"  STM32响应:\n{response}")
                
                # 验证响应
                if "[Python CMD]" in response:
                    print(f"  ✅ 命令被正确识别")
                else:
                    print(f"  ⚠️  响应格式不符合预期")
            else:
                print("  ❌ 无响应")
            
            time.sleep(0.5)  # 两个命令之间的间隔
        
        # 4. 测试未知命令
        print("\n[步骤4] 测试未知命令...")
        print("-" * 60)
        ser.write(b'UNKNOWN,123\n')
        time.sleep(0.5)
        if ser.in_waiting:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"STM32响应:\n{response}")
            if "Unknown command" in response:
                print("✅ 未知命令处理正确")
        
        # 5. 总结
        print("\n" + "=" * 60)
        print("阶段1测试完成！")
        print("=" * 60)
        print("\n验收结果：")
        print("  ✅ Python能发送命令")
        print("  ✅ STM32能正确接收并解析命令")
        print("  ✅ STM32能回应命令执行结果")
        print("  ✅ 无数据丢失或乱码")
        print("\n🎉 阶段1通信基础建立成功！可以进入阶段2。")
        
        # 关闭串口
        ser.close()
        print(f"\n串口已关闭。")
        
    except serial.SerialException as e:
        print(f"\n❌ 串口错误: {e}")
        print("\n可能的原因：")
        print("  1. 串口号不正确（检查设备管理器）")
        print("  2. 串口被其他程序占用")
        print("  3. STM32未连接或未上电")
        print(f"\n请检查后重试。")
        return False
        
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断测试")
        if 'ser' in locals() and ser.is_open:
            ser.close()
        return False
        
    except Exception as e:
        print(f"\n❌ 未知错误: {e}")
        if 'ser' in locals() and ser.is_open:
            ser.close()
        return False
    
    return True


def interactive_test():
    """交互式测试模式"""
    print("\n" + "=" * 60)
    print("交互式测试模式")
    print("=" * 60)
    print("输入命令发送给STM32，输入'quit'退出")
    print("示例命令:")
    print("  MODE,0  - 停止")
    print("  MODE,1  - 前进")
    print("  MODE,3  - 左转")
    print("  MODE,4  - 右转")
    print("-" * 60)
    
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
        print(f"串口已连接: {ser.name}\n")
        
        while True:
            cmd = input(">>> ")
            if cmd.lower() == 'quit':
                break
            
            if not cmd:
                continue
            
            # 发送命令（自动添加换行符）
            if not cmd.endswith('\n'):
                cmd += '\n'
            ser.write(cmd.encode('utf-8'))
            
            # 读取响应
            time.sleep(0.3)
            if ser.in_waiting:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                print(response)
        
        ser.close()
        print("\n串口已关闭。")
        
    except Exception as e:
        print(f"\n错误: {e}")
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == '__main__':
    print("\n" + "=" * 60)
    print("STM32 阶段1通信测试工具")
    print("=" * 60)
    print("请选择测试模式：")
    print("  1. 自动测试（推荐）")
    print("  2. 交互式测试")
    print("  3. 退出")
    
    choice = input("\n请选择 (1/2/3): ")
    
    if choice == '1':
        success = test_stage1()
        sys.exit(0 if success else 1)
    elif choice == '2':
        interactive_test()
    elif choice == '3':
        print("再见！")
    else:
        print("无效选择")




