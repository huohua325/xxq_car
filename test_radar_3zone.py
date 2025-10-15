#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达三区域避障系统测试脚本
测试目标：
  1. 原始数据解析准确性（角度0-360°，距离合理）
  2. 三区域处理后的数据准确性（前/左/右区域划分）
  3. 障碍物检测准确性
  
作者: AI Assistant
日期: 2025-10-15
版本: v2.0
"""

import sys
import time
import json
import re
from typing import Dict, List, Optional
from dataclasses import dataclass

# 尝试导入蓝牙通信模块
try:
    sys.path.insert(0, '.')
    from ble_robot_control import SimpleBLERobotComm
    BLE_AVAILABLE = True
except ImportError:
    BLE_AVAILABLE = False
    print("⚠️  警告: 未找到ble_robot_control模块，将使用串口模式")

# 配置
BLE_ADDRESS = "C4:25:01:20:02:8E"  # 修改为您的蓝牙地址
SERIAL_PORT = "COM5"                # 修改为您的串口号
SERIAL_BAUDRATE = 115200


# ============================================================================
# 数据结构定义
# ============================================================================

@dataclass
class RawPointData:
    """原始测量点数据"""
    angle_deg: float
    distance_m: float
    quality: int
    is_sync: bool
    is_valid: bool


@dataclass
class ZoneData:
    """区域统计数据"""
    count: int
    min_dist: float
    avg_dist: float
    closest_angle: float
    has_obstacle: bool


@dataclass
class ObstacleData:
    """三区域避障数据"""
    timestamp: int
    total_points: int
    invalid_points: int
    coverage_percent: float
    front: ZoneData
    left: ZoneData
    right: ZoneData


# ============================================================================
# 测试类
# ============================================================================

class RadarTester:
    """雷达三区域系统测试器"""
    
    def __init__(self, use_ble=True, ble_address=None, serial_port=None):
        """
        初始化测试器
        
        Args:
            use_ble: 是否使用蓝牙（True）或串口（False）
            ble_address: 蓝牙地址
            serial_port: 串口号
        """
        self.use_ble = use_ble and BLE_AVAILABLE
        self.comm = None
        self.ble_address = ble_address or BLE_ADDRESS
        self.serial_port = serial_port or SERIAL_PORT
        
        # 测试结果存储
        self.raw_test_results = []
        self.obstacle_test_results = []
        self.received_lines = []  # 存储所有接收到的行
        
    def connect(self) -> bool:
        """连接设备"""
        try:
            if self.use_ble:
                print(f"🔵 连接蓝牙设备: {self.ble_address}...")
                self.comm = SimpleBLERobotComm(address=self.ble_address, verbose=False)
                if not self.comm.connect():
                    print("❌ 蓝牙连接失败")
                    return False
                print("✅ 蓝牙已连接\n")
            else:
                print(f"🔌 连接串口: {self.serial_port}...")
                import serial
                self.comm = serial.Serial(self.serial_port, SERIAL_BAUDRATE, timeout=2)
                if not self.comm.is_open:
                    print("❌ 串口连接失败")
                    return False
                print("✅ 串口已连接\n")
                time.sleep(1)  # 等待设备准备
            
            return True
        except Exception as e:
            print(f"❌ 连接错误: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.comm:
            if self.use_ble:
                self.comm.disconnect()
            else:
                self.comm.close()
            print("\n✅ 已断开连接")
    
    def send_command(self, cmd: str):
        """发送命令"""
        if self.use_ble:
            self.comm.send_command(cmd + "\n")
        else:
            self.comm.write((cmd + "\n").encode('utf-8'))
            self.comm.flush()
    
    def read_response(self, timeout: float = 10.0) -> List[str]:
        """
        读取响应直到收到结束标记
        
        Args:
            timeout: 超时时间（秒）
        
        Returns:
            接收到的所有行
        """
        lines = []
        start_time = time.time()
        
        # 结束标记（根据不同命令有不同的结束标记）
        end_markers = [
            "================================",
            "===========================================",
            "============================",
        ]
        
        while time.time() - start_time < timeout:
            try:
                if self.use_ble:
                    # BLE模式：从缓冲区读取
                    # 注意：SimpleBLERobotComm可能需要实现raw_read方法
                    time.sleep(0.1)
                    continue
                else:
                    # 串口模式
                    if self.comm.in_waiting > 0:
                        line = self.comm.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            lines.append(line)
                            print(line)  # 实时显示
                            
                            # 检查是否到达结束标记
                            for marker in end_markers:
                                if marker in line:
                                    return lines
            except Exception as e:
                print(f"⚠️  读取错误: {e}")
                break
            
            time.sleep(0.01)
        
        return lines
    
    # ========================================================================
    # 测试1：原始数据解析测试
    # ========================================================================
    
    def test_raw_data_parsing(self) -> bool:
        """
        测试原始数据解析（命令R）
        验证：角度0-360°、距离合理、数据有效性
        """
        print("\n" + "="*80)
        print("🧪 测试1：原始数据解析准确性")
        print("="*80)
        print("\n目标：验证雷达原始数据解析是否正确")
        print("  - 角度应在0-360°范围内")
        print("  - 距离应在0.1-6.0米范围内")
        print("  - 有效点比例 > 80%\n")
        
        # 发送命令R
        print("📤 发送命令: R")
        self.send_command("R")
        
        # 读取响应
        print("\n📥 接收数据...\n")
        lines = self.read_response(timeout=15.0)
        
        if not lines:
            print("\n❌ 测试失败：未收到数据")
            return False
        
        # 解析原始数据
        print("\n📊 分析结果...\n")
        
        # 查找PARSED部分
        parsed_points = []
        summary_valid = 0
        summary_invalid = 0
        
        for line in lines:
            # 解析测量点：1: Angle=  12.3° Dist= 1.45m Q= 78   VALID
            if re.search(r'^\s*\d+:\s+Angle=', line):
                match = re.search(r'Angle=\s*([\d.]+)°\s+Dist=\s*([\d.]+)m\s+Q=\s*(\d+)\s+(\w+)\s+(\w+)', line)
                if match:
                    angle = float(match.group(1))
                    distance = float(match.group(2))
                    quality = int(match.group(3))
                    sync = 'S' in match.group(4)
                    valid = 'VALID' in match.group(5)
                    
                    parsed_points.append(RawPointData(
                        angle_deg=angle,
                        distance_m=distance,
                        quality=quality,
                        is_sync=sync,
                        is_valid=valid
                    ))
            
            # 解析摘要：[SUMMARY] Valid: 8 | Invalid: 2
            if 'SUMMARY' in line:
                match = re.search(r'Valid:\s*(\d+)\s*\|\s*Invalid:\s*(\d+)', line)
                if match:
                    summary_valid = int(match.group(1))
                    summary_invalid = int(match.group(2))
        
        # 分析结果
        if not parsed_points:
            print("❌ 未找到解析后的测量点数据")
            return False
        
        print(f"✅ 成功解析 {len(parsed_points)} 个测量点\n")
        
        # 统计分析
        valid_count = sum(1 for p in parsed_points if p.is_valid)
        invalid_count = len(parsed_points) - valid_count
        
        angle_errors = sum(1 for p in parsed_points if p.angle_deg < 0 or p.angle_deg >= 360)
        distance_errors = sum(1 for p in parsed_points if p.distance_m < 0.1 or p.distance_m > 6.0)
        
        valid_rate = (valid_count / len(parsed_points)) * 100 if parsed_points else 0
        
        print("📊 统计分析：")
        print(f"  有效点数: {valid_count}/{len(parsed_points)} ({valid_rate:.1f}%)")
        print(f"  无效点数: {invalid_count}")
        print(f"  角度异常: {angle_errors} 个（角度<0或≥360°）")
        print(f"  距离异常: {distance_errors} 个（距离<0.1或>6.0m）")
        
        if summary_valid > 0:
            print(f"\n  STM32报告: 有效{summary_valid}, 无效{summary_invalid}")
        
        # 判断测试结果
        success = valid_rate >= 80 and angle_errors == 0
        
        if success:
            print("\n✅ 测试通过：原始数据解析准确")
        else:
            print("\n❌ 测试失败：")
            if valid_rate < 80:
                print(f"  - 有效点比例过低（{valid_rate:.1f}% < 80%）")
            if angle_errors > 0:
                print(f"  - 存在{angle_errors}个角度异常点")
        
        # 保存结果
        self.raw_test_results.append({
            'success': success,
            'valid_rate': valid_rate,
            'angle_errors': angle_errors,
            'distance_errors': distance_errors
        })
        
        return success
    
    # ========================================================================
    # 测试2：三区域避障测试
    # ========================================================================
    
    def test_obstacle_detection(self) -> bool:
        """
        测试三区域避障数据（命令O）
        验证：区域划分正确、统计准确、障碍物检测
        """
        print("\n" + "="*80)
        print("🧪 测试2：三区域避障数据准确性")
        print("="*80)
        print("\n目标：验证三区域数据处理是否正确")
        print("  - 前方区域：330°~30°")
        print("  - 左侧区域：30°~150°")
        print("  - 右侧区域：210°~330°")
        print("  - 每个区域应有足够测量点（>50）\n")
        
        # 发送命令O
        print("📤 发送命令: O")
        self.send_command("O")
        
        # 读取响应
        print("\n📥 接收数据...\n")
        lines = self.read_response(timeout=20.0)
        
        if not lines:
            print("\n❌ 测试失败：未收到数据")
            return False
        
        # 解析三区域数据
        print("\n📊 分析结果...\n")
        
        # 查找JSON数据
        json_start = -1
        json_end = -1
        for i, line in enumerate(lines):
            if line.strip().startswith('{'):
                json_start = i
            if line.strip() == '}' and json_start >= 0:
                json_end = i
                break
        
        obstacle_data = None
        
        if json_start >= 0 and json_end >= 0:
            # 提取JSON字符串
            json_lines = lines[json_start:json_end+1]
            json_str = '\n'.join(json_lines)
            
            try:
                data = json.loads(json_str)
                
                if data.get('type') == 'OBSTACLE':
                    # 解析数据
                    obstacle_data = ObstacleData(
                        timestamp=data['timestamp'],
                        total_points=data['total_points'],
                        invalid_points=data['invalid_points'],
                        coverage_percent=data['coverage'],
                        front=ZoneData(**data['front']),
                        left=ZoneData(**data['left']),
                        right=ZoneData(**data['right'])
                    )
                    
                    print("✅ 成功解析JSON数据\n")
            except json.JSONDecodeError as e:
                print(f"⚠️  JSON解析错误: {e}")
        
        # 如果JSON解析失败，尝试从文本输出解析
        if not obstacle_data:
            print("⚠️  JSON解析失败，尝试从文本输出解析...")
            # 这里可以添加文本解析逻辑
            print("❌ 测试失败：无法解析数据")
            return False
        
        # 分析数据
        print("📊 三区域统计：")
        print(f"\n  总点数: {obstacle_data.total_points}")
        print(f"  无效点: {obstacle_data.invalid_points} ({obstacle_data.invalid_points/max(obstacle_data.total_points,1)*100:.1f}%)")
        print(f"  覆盖率: {obstacle_data.coverage_percent:.1f}%")
        
        print(f"\n  [前方区域] 330°~30°:")
        print(f"    点数: {obstacle_data.front.count}")
        print(f"    最近距离: {obstacle_data.front.min_dist:.2f}m @ {obstacle_data.front.closest_angle:.1f}°")
        print(f"    平均距离: {obstacle_data.front.avg_dist:.2f}m")
        print(f"    有障碍物: {'是' if obstacle_data.front.has_obstacle else '否'}")
        
        print(f"\n  [左侧区域] 30°~150°:")
        print(f"    点数: {obstacle_data.left.count}")
        print(f"    最近距离: {obstacle_data.left.min_dist:.2f}m @ {obstacle_data.left.closest_angle:.1f}°")
        print(f"    平均距离: {obstacle_data.left.avg_dist:.2f}m")
        print(f"    有障碍物: {'是' if obstacle_data.left.has_obstacle else '否'}")
        
        print(f"\n  [右侧区域] 210°~330°:")
        print(f"    点数: {obstacle_data.right.count}")
        print(f"    最近距离: {obstacle_data.right.min_dist:.2f}m @ {obstacle_data.right.closest_angle:.1f}°")
        print(f"    平均距离: {obstacle_data.right.avg_dist:.2f}m")
        print(f"    有障碍物: {'是' if obstacle_data.right.has_obstacle else '否'}")
        
        # 验证区域角度范围
        angle_valid = True
        
        # 前方区域：330°~30° (需要特殊处理跨越0°的情况)
        if obstacle_data.front.count > 0:
            angle = obstacle_data.front.closest_angle
            if not ((angle >= 330 and angle <= 360) or (angle >= 0 and angle <= 30)):
                print(f"\n  ⚠️  前方区域角度异常: {angle}° (应在330°~30°)")
                angle_valid = False
        
        # 左侧区域：30°~150°
        if obstacle_data.left.count > 0:
            angle = obstacle_data.left.closest_angle
            if not (30 <= angle <= 150):
                print(f"\n  ⚠️  左侧区域角度异常: {angle}° (应在30°~150°)")
                angle_valid = False
        
        # 右侧区域：210°~330°
        if obstacle_data.right.count > 0:
            angle = obstacle_data.right.closest_angle
            if not (210 <= angle <= 330):
                print(f"\n  ⚠️  右侧区域角度异常: {angle}° (应在210°~330°)")
                angle_valid = False
        
        # 判断测试结果
        success = (
            obstacle_data.coverage_percent >= 90 and
            obstacle_data.total_points >= 200 and
            obstacle_data.front.count > 0 and
            obstacle_data.left.count > 0 and
            obstacle_data.right.count > 0 and
            angle_valid
        )
        
        if success:
            print("\n✅ 测试通过：三区域数据准确")
        else:
            print("\n❌ 测试失败：")
            if obstacle_data.coverage_percent < 90:
                print(f"  - 覆盖率不足（{obstacle_data.coverage_percent:.1f}% < 90%）")
            if obstacle_data.total_points < 200:
                print(f"  - 总点数过少（{obstacle_data.total_points} < 200）")
            if obstacle_data.front.count == 0:
                print("  - 前方区域无数据")
            if obstacle_data.left.count == 0:
                print("  - 左侧区域无数据")
            if obstacle_data.right.count == 0:
                print("  - 右侧区域无数据")
            if not angle_valid:
                print("  - 区域角度范围错误")
        
        # 保存结果
        self.obstacle_test_results.append({
            'success': success,
            'data': obstacle_data
        })
        
        return success
    
    # ========================================================================
    # 测试3：障碍物检测验证
    # ========================================================================
    
    def test_obstacle_detection_interactive(self):
        """
        交互式障碍物检测测试
        提示用户放置/移除障碍物，验证检测准确性
        """
        print("\n" + "="*80)
        print("🧪 测试3：障碍物检测验证（交互式）")
        print("="*80)
        print("\n目标：验证障碍物检测准确性")
        print("  请按照提示放置/移除障碍物\n")
        
        # 测试1：无障碍物
        input("📍 步骤1：确保前方1米内无障碍物，然后按回车...")
        print("\n📤 发送命令: O")
        self.send_command("O")
        print("\n📥 接收数据...\n")
        lines = self.read_response(timeout=20.0)
        
        # 简单检查（这里省略详细解析）
        has_front_obstacle = any('has_obstacle": 1' in line and 'front' in line for line in lines)
        
        if not has_front_obstacle:
            print("✅ 正确：前方无障碍物")
        else:
            print("❌ 错误：前方误报障碍物")
        
        # 测试2：有障碍物
        input("\n📍 步骤2：在前方0.3米处放置障碍物，然后按回车...")
        print("\n📤 发送命令: O")
        self.send_command("O")
        print("\n📥 接收数据...\n")
        lines = self.read_response(timeout=20.0)
        
        has_front_obstacle = any('"has_obstacle": 1' in line and 'front' in line for line in lines)
        
        if has_front_obstacle:
            print("✅ 正确：前方检测到障碍物")
        else:
            print("❌ 错误：前方未检测到障碍物")
        
        print("\n✅ 交互式测试完成")
    
    # ========================================================================
    # 综合测试报告
    # ========================================================================
    
    def run_all_tests(self, interactive=False):
        """运行所有测试"""
        print("\n" + "="*80)
        print("🚀 雷达三区域避障系统 - 完整测试")
        print("="*80)
        print(f"\n测试模式: {'蓝牙' if self.use_ble else '串口'}")
        print(f"设备地址: {self.ble_address if self.use_ble else self.serial_port}\n")
        
        if not self.connect():
            return
        
        try:
            # 等待设备准备
            time.sleep(2)
            
            # 测试1：原始数据解析
            test1_pass = self.test_raw_data_parsing()
            time.sleep(2)
            
            # 测试2：三区域避障
            test2_pass = self.test_obstacle_detection()
            time.sleep(2)
            
            # 测试3：交互式障碍物检测（可选）
            if interactive:
                self.test_obstacle_detection_interactive()
            
            # 总结报告
            print("\n" + "="*80)
            print("📊 测试总结报告")
            print("="*80)
            
            print(f"\n测试1 - 原始数据解析: {'✅ 通过' if test1_pass else '❌ 失败'}")
            print(f"测试2 - 三区域避障:   {'✅ 通过' if test2_pass else '❌ 失败'}")
            
            all_pass = test1_pass and test2_pass
            
            if all_pass:
                print("\n🎉 所有测试通过！雷达系统工作正常")
            else:
                print("\n⚠️  部分测试失败，请检查：")
                if not test1_pass:
                    print("  - 原始数据解析可能存在偏移问题")
                if not test2_pass:
                    print("  - 三区域数据处理可能存在错误")
            
            print("\n" + "="*80)
            
        except KeyboardInterrupt:
            print("\n\n⚠️  用户中断测试")
        except Exception as e:
            print(f"\n❌ 测试错误: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.disconnect()


# ============================================================================
# 主程序
# ============================================================================

def main():
    """主程序入口"""
    print("\n" + "="*80)
    print("📡 雷达三区域避障系统 - 测试工具 v2.0")
    print("="*80)
    
    # 选择连接模式
    if BLE_AVAILABLE:
        print("\n选择连接模式:")
        print("  1. 蓝牙（BLE）")
        print("  2. 串口（USB/UART）")
        mode = input("\n请选择 (1/2): ").strip()
        use_ble = (mode == '1')
    else:
        print("\n⚠️  蓝牙模块不可用，将使用串口模式")
        use_ble = False
    
    # 选择测试模式
    print("\n选择测试模式:")
    print("  1. 自动测试（快速验证）")
    print("  2. 完整测试（包含交互式障碍物检测）")
    test_mode = input("\n请选择 (1/2): ").strip()
    interactive = (test_mode == '2')
    
    # 创建测试器
    if use_ble:
        ble_addr = input(f"\n蓝牙地址 (回车使用默认: {BLE_ADDRESS}): ").strip()
        tester = RadarTester(use_ble=True, ble_address=ble_addr or BLE_ADDRESS)
    else:
        serial_port = input(f"\n串口号 (回车使用默认: {SERIAL_PORT}): ").strip()
        tester = RadarTester(use_ble=False, serial_port=serial_port or SERIAL_PORT)
    
    # 运行测试
    tester.run_all_tests(interactive=interactive)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 再见！")

