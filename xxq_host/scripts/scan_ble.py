#!/usr/bin/env python3
"""
BLE设备扫描工具
扫描附近的BLE设备，找到HC-04BLE的地址
"""

import asyncio
import sys

try:
    from bleak import BleakScanner
except ImportError:
    print("❌ 未安装bleak库")
    print("\n请运行: pip install bleak")
    sys.exit(1)


async def scan_ble():
    """扫描BLE设备"""
    print("=" * 70)
    print("BLE设备扫描工具")
    print("=" * 70)
    print("\n正在扫描附近的BLE设备（15秒）...")
    print("请确保HC-04BLE已上电...\n")
    
    try:
        devices = await BleakScanner.discover(timeout=15.0)
        
        if not devices:
            print("\n❌ 未发现任何BLE设备")
            print("\n可能原因：")
            print("  1. 电脑蓝牙未开启")
            print("  2. HC-04BLE未上电")
            print("  3. HC-04BLE已被其他设备连接")
            print("  4. 距离太远")
            return
        
        print(f"✅ 发现 {len(devices)} 个BLE设备：")
        print("=" * 70)
        
        # 按信号强度排序（兼容不同版本的bleak）
        try:
            devices = sorted(devices, key=lambda d: getattr(d, 'rssi', -100), reverse=True)
        except:
            pass  # 如果排序失败，保持原顺序
        
        hc04_found = False
        
        for i, device in enumerate(devices, 1):
            name = device.name or "(未命名)"
            address = device.address
            # 兼容不同版本的bleak
            rssi = getattr(device, 'rssi', None)
            rssi_str = f"{rssi} dBm" if rssi is not None else "N/A"
            
            # 检查是否是HC-04相关设备
            is_hc04 = False
            if device.name:
                name_upper = device.name.upper()
                if any(keyword in name_upper for keyword in ['HC', 'BLE', 'UART', 'SERIAL']):
                    is_hc04 = True
                    hc04_found = True
            
            # 标记可能的HC-04设备
            marker = "  ⭐ [可能是HC-04BLE]" if is_hc04 else ""
            
            print(f"\n{i}. {name}{marker}")
            print(f"   地址:  {address}")
            print(f"   信号:  {rssi_str}")
            
            # 如果是可能的HC-04设备，显示使用提示
            if is_hc04:
                print(f"   💡 使用: python test_stage1_ble.py")
                print(f"      然后输入地址: {address}")
        
        print("\n" + "=" * 70)
        
        if hc04_found:
            print("\n✅ 找到可能的HC-04BLE设备（标记为⭐）")
            print("\n下一步：")
            print("  1. 记住设备地址")
            print("  2. 运行测试: python scripts/test_stage1_ble.py")
            print("  3. 输入设备地址")
        else:
            print("\n⚠️  未找到明显的HC-04设备")
            print("\n如果你的HC-04在列表中但名称不明显，请：")
            print("  1. 查看设备信号强度（越大越近）")
            print("  2. 暂时关闭HC-04电源")
            print("  3. 重新扫描，看哪个设备消失了")
            print("  4. 那个消失的设备就是HC-04")
        
        print("\n💡 提示：")
        print("  - 信号强度越大（如-40 dBm）说明设备越近")
        print("  - 如果看到多个设备，把HC-04靠近电脑再扫描")
        print("  - HC-04可能显示为: HC-04, BLE, UART, Serial等名称")
        
    except Exception as e:
        print(f"\n❌ 扫描失败: {e}")
        print("\n请检查：")
        print("  1. 电脑蓝牙是否开启")
        print("  2. 是否有权限访问蓝牙")
        print("  3. bleak库是否正确安装")


def main():
    """主函数"""
    print("\n" + "=" * 70)
    print("提示：此工具会扫描附近所有BLE设备，可能需要15秒")
    print("=" * 70)
    
    try:
        asyncio.run(scan_ble())
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断扫描")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

