#!/usr/bin/env python3
"""
STM32智能小车BLE控制程序
功能：通过BLE连接控制机器人，接收传感器数据

使用方法：
python ble_robot_control.py
"""

import asyncio
import json
import time
import threading
from typing import Optional, Callable
import sys

# 检查依赖
try:
    from bleak import BleakClient
except ImportError:
    print("❌ 未安装bleak库")
    print("请运行: pip install bleak")
    sys.exit(1)

# ============================================================================
# 数据结构定义
# ============================================================================

class LidarData:
    """激光雷达数据"""
    def __init__(self, timestamp: int, total_points: int, angle_coverage: float, sectors: list):
        self.timestamp = timestamp
        self.total_points = total_points
        self.angle_coverage = angle_coverage
        self.sectors = sectors

class MPUData:
    """MPU6500传感器数据"""
    def __init__(self, timestamp: int, roll: float, pitch: float, accel: list, gyro: list):
        self.timestamp = timestamp
        self.roll = roll
        self.pitch = pitch
        self.accel = accel  # [x, y, z]
        self.gyro = gyro    # [x, y, z]

class OdometryData:
    """里程计数据"""
    def __init__(self, timestamp: int, left_speed: float, right_speed: float, left_count: int, right_count: int):
        self.timestamp = timestamp
        self.left_speed = left_speed
        self.right_speed = right_speed
        self.left_count = left_count
        self.right_count = right_count

class PoseData:
    """机器人位姿数据"""
    def __init__(self, timestamp: int, x: float, y: float, theta: float):
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.theta = theta

# ============================================================================
# 简化版BLE通信类
# ============================================================================

class SimpleBLERobotComm:
    """简化的BLE机器人通信类"""
    
    # BLE UART服务UUID（HC-04BLE常用）
    UART_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
    UART_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
    
    def __init__(self, address: str, verbose: bool = False):
        """
        初始化BLE通信
        
        Args:
            address: BLE设备地址（如 'C4:25:01:20:02:8E'）
            verbose: 是否显示详细日志
        """
        self.address = address
        self.verbose = verbose
        self.client: Optional[BleakClient] = None
        self.running = False
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        self._connect_event = threading.Event()
        self._stop_requested = False
        
        # 数据缓冲区（处理BLE分包）
        self.buffer = ""
        self._json_collecting = False  # JSON收集状态
        self._json_buffer = ""         # JSON重组缓冲区
        
        # 最新数据缓存
        self.latest_lidar: Optional[LidarData] = None
        self.latest_mpu: Optional[MPUData] = None
        self.latest_odom: Optional[OdometryData] = None
        self.latest_pose: Optional[PoseData] = None
        
        # 回调函数
        self.on_lidar_update: Optional[Callable[[LidarData], None]] = None
        self.on_mpu_update: Optional[Callable[[MPUData], None]] = None
        self.on_odom_update: Optional[Callable[[OdometryData], None]] = None
        self.on_pose_update: Optional[Callable[[PoseData], None]] = None
        self.on_message: Optional[Callable[[str], None]] = None
        
        # 统计
        self.stats = {
            'mpu_count': 0,
            'odo_count': 0,
            'pose_count': 0,
            'lidar_count': 0,
            'ack_count': 0,
            'other_messages': 0
        }
    
    def connect(self) -> bool:
        """连接BLE设备"""
        if self.verbose:
            print(f"🔵 连接BLE设备: {self.address}")
        
        self.running = True
        self._stop_requested = False
        self._connect_event.clear()
        
        # 在独立线程中运行BLE事件循环
        self.thread = threading.Thread(
            target=self._run_ble_thread,
            daemon=True,
            name="SimpleBLERobotComm"
        )
        self.thread.start()
        
        # 等待连接完成（最多15秒）
        if self._connect_event.wait(timeout=15.0):
            if self.client and self.client.is_connected:
                print("✅ BLE已连接")
                return True
        
        print("❌ BLE连接超时")
        self.disconnect()
        return False
    
    def _run_ble_thread(self):
        """在独立线程中运行BLE事件循环"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            self.loop.run_until_complete(self._ble_main_loop())
        except Exception as e:
            if self.verbose:
                print(f"❌ BLE线程异常: {e}")
        finally:
            # 清理事件循环
            try:
                pending = asyncio.all_tasks(self.loop)
                for task in pending:
                    task.cancel()
                if pending:
                    self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except:
                pass
            finally:
                self.loop.close()
    
    async def _ble_main_loop(self):
        """BLE主循环"""
        try:
            self.client = BleakClient(self.address, timeout=20.0)
            await self.client.connect()
            
            # 订阅通知
            await self.client.start_notify(
                self.UART_CHAR_UUID,
                self._notification_handler
            )
            
            # 通知连接成功
            self._connect_event.set()
            
            # 保持连接
            while not self._stop_requested:
                if not self.client.is_connected:
                    break
                await asyncio.sleep(0.1)
            
        except Exception as e:
            if self.verbose:
                print(f"❌ BLE连接失败: {e}")
            self._connect_event.set()
        finally:
            if self.client and self.client.is_connected:
                try:
                    await self.client.stop_notify(self.UART_CHAR_UUID)
                    await self.client.disconnect()
                except:
                    pass
    
    def _notification_handler(self, sender, data: bytearray):
        """BLE数据接收回调"""
        if self._stop_requested or not self.running:
            return
            
        try:
            decoded = data.decode('utf-8', errors='ignore')
            self.buffer += decoded
            
            # 按行处理（BLE会分包，需要重组）
            while '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)
                self._parse_line(line.strip())
                
        except Exception as e:
            if self.verbose:
                print(f"❌ 数据处理异常: {e}")
    
    def _parse_line(self, line: str):
        """解析一行数据"""
        if not line:
            return
        
        try:
            # JSON重组状态机 - 完全修复版
            if self._json_collecting:
                # 正在收集JSON，所有行都加入
                self._json_buffer += line + '\n'
                
                # 检查是否结束（大括号数量相等）
                open_braces = self._json_buffer.count('{')
                close_braces = self._json_buffer.count('}')
                
                if open_braces > 0 and open_braces == close_braces:
                    # JSON完整了
                    self._parse_complete_json()
                    self._json_collecting = False
                    self._json_buffer = ""
                return
            
            # 检测JSON开始（以{开头，可能前面有空格）
            if line.strip().startswith('{'):
                # JSON开始
                self._json_collecting = True
                self._json_buffer = line + '\n'
                return
                
            # CSV格式（MPU/ODO/POSE）
            if line.startswith('MPU,') or line.startswith('ODO,') or line.startswith('POSE,'):
                self._parse_csv(line)
            # 其他消息
            else:
                self._parse_other_message(line)
                
        except Exception as e:
            if self.verbose:
                print(f"⚠️ 解析失败: {line[:50]}... 错误: {e}")
            # 重置JSON收集状态
            self._json_collecting = False
            self._json_buffer = ""
    
    def _parse_complete_json(self):
        """解析完整的JSON"""
        try:
            data = json.loads(self._json_buffer)
            
            if data.get('type') == 'LIDAR':
                lidar_info = data.get('data', {})
                self.latest_lidar = LidarData(
                    timestamp=data.get('timestamp', 0),
                    total_points=lidar_info.get('total_points', 0),
                    angle_coverage=lidar_info.get('angle_coverage', 0.0),
                    sectors=lidar_info.get('sectors', [])
                )
                
                self.stats['lidar_count'] += 1
                
                if self.on_lidar_update:
                    self.on_lidar_update(self.latest_lidar)
                else:
                    # 如果没有回调，直接打印
                    print(f"\n📡 雷达数据: {self.latest_lidar.total_points}点，覆盖{self.latest_lidar.angle_coverage:.1f}°")
                    
        except json.JSONDecodeError as e:
            # JSON解析失败，尝试简单的文本解析作为备选
            print(f"⚠️ JSON解析失败，原始内容:")
            print(self._json_buffer[:200] + "..." if len(self._json_buffer) > 200 else self._json_buffer)
    
    def _parse_csv(self, line: str):
        """解析CSV格式数据"""
        parts = line.split(',')
        data_type = parts[0]
        
        try:
            if data_type == 'MPU' and len(parts) >= 10:
                self.latest_mpu = MPUData(
                    timestamp=int(parts[1]),
                    roll=float(parts[2]),
                    pitch=float(parts[3]),
                    accel=[float(parts[4]), float(parts[5]), float(parts[6])],
                    gyro=[float(parts[7]), float(parts[8]), float(parts[9])]
                )
                self.stats['mpu_count'] += 1
                if self.on_mpu_update:
                    self.on_mpu_update(self.latest_mpu)
            
            elif data_type == 'ODO' and len(parts) >= 6:
                self.latest_odom = OdometryData(
                    timestamp=int(parts[1]),
                    left_speed=float(parts[2]),
                    right_speed=float(parts[3]),
                    left_count=int(parts[4]),
                    right_count=int(parts[5])
                )
                self.stats['odo_count'] += 1
                if self.on_odom_update:
                    self.on_odom_update(self.latest_odom)
            
            elif data_type == 'POSE' and len(parts) >= 5:
                self.latest_pose = PoseData(
                    timestamp=int(parts[1]),
                    x=float(parts[2]),
                    y=float(parts[3]),
                    theta=float(parts[4])
                )
                self.stats['pose_count'] += 1
                if self.on_pose_update:
                    self.on_pose_update(self.latest_pose)
                    
        except (ValueError, IndexError):
            if self.verbose:
                print(f"⚠️ CSV解析错误: {line}")
    
    def _parse_other_message(self, line: str):
        """解析其他消息"""
        if line == "ACK":
            self.stats['ack_count'] += 1
        else:
            self.stats['other_messages'] += 1
        
        if self.on_message:
            self.on_message(line)
    
    def send_command(self, command: str) -> bool:
        """发送命令"""
        if not self.running or self._stop_requested:
            return False
            
        if not self.client or not self.client.is_connected:
            return False
        
        try:
            if self.loop and not self.loop.is_closed():
                future = asyncio.run_coroutine_threadsafe(
                    self._async_send(command.encode('utf-8')),
                    self.loop
                )
                future.result(timeout=1.0)
                return True
            else:
                return False
                
        except Exception as e:
            if self.verbose:
                print(f"❌ 发送失败: {e}")
            return False
    
    async def _async_send(self, data: bytes):
        """异步发送数据"""
        if self.client and self.client.is_connected:
            await self.client.write_gatt_char(self.UART_CHAR_UUID, data)
    
    # 便捷的命令方法
    def stop_robot(self) -> bool:
        """停止机器人"""
        return self.send_command("MODE,0\n")
    
    def forward(self) -> bool:
        """前进"""
        return self.send_command("MODE,1\n")
    
    def backward(self) -> bool:
        """后退"""
        return self.send_command("MODE,2\n")
    
    def turn_left(self) -> bool:
        """左转"""
        return self.send_command("MODE,3\n")
    
    def turn_right(self) -> bool:
        """右转"""
        return self.send_command("MODE,4\n")
    
    def scan_lidar(self) -> bool:
        """雷达扫描"""
        return self.send_command("A\n")
    
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.client is not None and self.client.is_connected
    
    def disconnect(self):
        """断开连接"""
        if not self.running:
            return
        
        self.running = False
        self._stop_requested = True
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=3.0)
        
        print("🔵 BLE已断开")
        
        # 重要：等待BLE完全释放（避免重连失败）
        time.sleep(3.0)

# ============================================================================
# 主程序
# ============================================================================

def on_lidar_data(lidar: LidarData):
    """雷达数据回调 - 详细显示版本"""
    print(f"\n{'='*90}")
    print(f"📡 雷达扫描完成！时间戳: {lidar.timestamp}ms")
    print(f"{'='*90}")
    print(f"🎯 扫描概要: 总点数 {lidar.total_points}, 角度覆盖 {lidar.angle_coverage:.1f}°")
    
    if len(lidar.sectors) > 0:
        print(f"\n📋 8个扇区详情 (每扇区45°):")
        print("-" * 90)
        print(f"{'扇区':<10} {'中心角度':<10} {'点数':<10} {'最近障碍物':<12} {'平均距离':<12} {'状态'}")
        print("-" * 90)
        
        # 方向名称映射
        direction_names = ['正前', '右前', '正右', '右后', '正后', '左后', '正左', '左前']
        
        for i, sector in enumerate(lidar.sectors):
            sid = sector.get('sector_id', i)
            angle = sector.get('angle_center', i * 45)
            count = sector.get('count', 0)
            min_d = sector.get('min_dist', 0.0)
            avg_d = sector.get('avg_dist', 0.0)
            
            # 状态判断
            if count == 0:
                status = "无数据"
            elif min_d < 0.3:
                status = "🚫 有障碍"
            elif min_d < 1.0:
                status = "⚠️ 较近"
            else:
                status = "✅ 通畅"
            
            direction = direction_names[i] if i < len(direction_names) else f"方向{i}"
            print(f"{direction:<10} {angle:>3}°      {count:<8} {min_d:>8.2f}m   {avg_d:>8.2f}m   {status}")
        
        print("-" * 90)
        
        # 安全分析
        safe_sectors = [s for s in lidar.sectors if s.get('min_dist', 0) > 0.5]
        print(f"🛡️ 安全分析: {len(safe_sectors)}/8 个方向安全(>0.5m)")
        
        if len(safe_sectors) > 0:
            best_sector = max(safe_sectors, key=lambda s: s.get('avg_dist', 0))
            best_angle = best_sector.get('angle_center', 0)
            best_dist = best_sector.get('avg_dist', 0)
            best_dir = direction_names[best_sector.get('sector_id', 0)]
            print(f"🎯 推荐方向: {best_dir}({best_angle}°), 平均距离{best_dist:.2f}m")
        else:
            print("⚠️ 警告: 周围障碍物较多，建议谨慎移动")
    else:
        print("⚠️ 未收到扇区数据")
    
    print("=" * 90)

def on_mpu_data(mpu: MPUData):
    """MPU数据回调"""
    print(f"\r[MPU] Roll:{mpu.roll:6.2f}° Pitch:{mpu.pitch:6.2f}° "
          f"Acc:({mpu.accel[0]:5.2f},{mpu.accel[1]:5.2f},{mpu.accel[2]:5.2f}) "
          f"Gyro:({mpu.gyro[0]:4.1f},{mpu.gyro[1]:4.1f},{mpu.gyro[2]:4.1f})", end='')

def on_odo_data(odo: OdometryData):
    """里程计数据回调"""
    print(f"\r[ODO] L:{odo.left_speed:5.2f} R:{odo.right_speed:5.2f} RPS "
          f"Count:({odo.left_count},{odo.right_count})", end='')

def on_pose_data(pose: PoseData):
    """位姿数据回调"""
    print(f"\r[POSE] X:{pose.x:6.3f} Y:{pose.y:6.3f} θ:{pose.theta:6.2f}°", end='')

def on_stm32_message(msg: str):
    """STM32消息回调"""
    if msg == "ACK":
        print("✅", end='', flush=True)  # 简洁的确认
    elif "Starting full LiDAR scan" in msg:
        print("\n📡 雷达开始扫描...")
    elif "Scan completed" in msg:
        print("📡 雷达扫描完成")
    elif "Robot FORWARD" in msg:
        print("⬆️ 机器人前进中")
    elif "Robot STOPPED" in msg:
        print("\n⏹️ 机器人已停止")  # 加换行，更明显
    elif "Robot TURN LEFT" in msg:
        print("\n⬅️ 机器人左转中") 
    elif "Robot TURN RIGHT" in msg:
        print("\n➡️ 机器人右转中")
    elif "Robot BACKWARD" in msg:
        print("⬇️ 机器人后退中")
    elif "MODE,3" in msg or "MODE,4" in msg:
        # 显示转向相关的所有消息
        print(f"\n💬 {msg}")
    elif "[DEBUG]" in msg:
        # 显示所有DEBUG消息
        print(f"💬 {msg}")
    # 隐藏其他调试消息，保持界面清洁

# 详细版本的消息回调（调试用）
def on_stm32_message_verbose(msg: str):
    """STM32消息回调 - 详细版本"""
    on_stm32_message(msg)  # 先调用简洁版本
    
    # 额外显示调试信息
    if "[Python CMD]" in msg or "[LIDAR]" in msg:
        if "Received:" not in msg:  # 不显示"Received:"消息，太冗余
            print(f"💬 {msg}")

def interactive_control():
    """交互式控制"""
    address = "C4:25:01:20:02:8E"
    
    print("=" * 80)
    print("🤖 STM32智能小车 BLE控制程序")
    print("=" * 80)
    print(f"BLE地址: {address}")
    
    # 连接
    print("\n🔵 正在连接BLE...")
    robot = SimpleBLERobotComm(address, verbose=False)
    
    # 设置回调
    robot.on_lidar_update = on_lidar_data
    robot.on_mpu_update = on_mpu_data
    robot.on_odom_update = on_odo_data
    robot.on_pose_update = on_pose_data
    robot.on_message = on_stm32_message
    
    if not robot.connect():
        print("❌ 连接失败")
        return
    
    try:
        print("\n" + "=" * 80)
        print("🎮 控制命令:")
        print("  w - 前进    s - 后退    a - 左转    d - 右转")
        print("  x - 停止    r - 雷达扫描    q - 退出")
        print("  i - 显示统计")
        print("=" * 80)
        
        while True:
            command = input("\n>> ").lower().strip()
            
            if command == 'q':
                break
            elif command == 'w':
                print("⬆️ 前进")
                robot.forward()
            elif command == 's':
                print("⬇️ 后退")
                robot.backward()
            elif command == 'a':
                print("⬅️ 左转")
                robot.turn_left()
            elif command == 'd':
                print("➡️ 右转")
                robot.turn_right()
            elif command == 'x':
                print("⏹️ 停止")
                robot.stop_robot()
            elif command == 'r':
                print("📡 雷达扫描...")
                robot.scan_lidar()
            elif command == 'i':
                stats = robot.stats
                print(f"\n📊 统计:")
                print(f"  MPU: {stats['mpu_count']} | ODO: {stats['odo_count']} | POSE: {stats['pose_count']}")
                print(f"  LIDAR: {stats['lidar_count']} | ACK: {stats['ack_count']} | 消息: {stats['other_messages']}")
            else:
                print("❓ 未知命令")
    
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    finally:
        print("\n🔵 断开连接...")
        robot.disconnect()

def demo_mode():
    """演示模式"""
    address = "C4:25:01:20:02:8E"
    
    print("=" * 80)
    print("🎬 演示模式")
    print("=" * 80)
    
    robot = SimpleBLERobotComm(address)
    
    # 设置回调
    robot.on_lidar_update = on_lidar_data
    robot.on_mpu_update = on_mpu_data
    robot.on_odo_update = on_odo_data
    robot.on_pose_update = on_pose_data
    robot.on_message = on_stm32_message
    
    if not robot.connect():
        return
    
    try:
        demo_steps = [
            ("停止", robot.stop_robot, 1),
            ("雷达扫描", robot.scan_lidar, 8),
            ("前进", robot.forward, 3),
            ("停止", robot.stop_robot, 1),
            ("左转", robot.turn_left, 2),
            ("停止", robot.stop_robot, 1),
            ("右转", robot.turn_right, 2),
            ("停止", robot.stop_robot, 1),
        ]
        
        for desc, action, wait_time in demo_steps:
            print(f"\n🎯 {desc}...")
            action()
            
            for i in range(wait_time):
                print(f"  ⏳ {i+1}/{wait_time}秒", end='\r')
                time.sleep(1)
            print()
        
        # 显示最终统计
        stats = robot.stats
        print(f"\n📊 演示统计:")
        print(f"  传感器数据: MPU:{stats['mpu_count']} ODO:{stats['odo_count']} POSE:{stats['pose_count']}")
        print(f"  雷达扫描: {stats['lidar_count']}次")
        print(f"  命令确认: {stats['ack_count']}个ACK")
    
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    finally:
        robot.disconnect()

def debug_mode():
    """调试模式 - 显示所有详细信息"""
    address = "C4:25:01:20:02:8E"
    
    print("=" * 80)
    print("🔍 调试模式（显示所有数据）")
    print("=" * 80)
    
    robot = SimpleBLERobotComm(address, verbose=True)  # 开启详细日志
    
    # 设置回调
    robot.on_lidar_update = on_lidar_data
    robot.on_mpu_update = on_mpu_data
    robot.on_odo_update = on_odo_data 
    robot.on_pose_update = on_pose_data
    robot.on_message = on_stm32_message_verbose  # 使用详细版本
    
    if not robot.connect():
        return
    
    try:
        print("\n🎯 调试测试序列...")
        
        # 1. 停止
        print("\n1️⃣ 发送停止命令...")
        robot.stop_robot()
        print("   等待STM32响应...")
        time.sleep(2)
        
        # 2. 雷达扫描（现在只需3秒）
        print("\n2️⃣ 请求雷达扫描...")
        robot.scan_lidar()
        print("   等待雷达数据（5秒）...")
        time.sleep(5)  # 从10秒减到5秒
        
        # 3. 前进测试
        print("\n3️⃣ 前进2秒...")
        robot.forward()
        print("   前进中...")
        time.sleep(2)
        
        print("\n   🛑 发送停止命令...")
        robot.stop_robot()
        print("   等待停止生效...")
        time.sleep(2)
        
        # 4. 左转测试
        print("\n4️⃣ 左转1.5秒...")
        robot.turn_left()
        print("   左转中...")
        time.sleep(1.5)
        
        print("\n   🛑 发送停止命令...")
        robot.stop_robot()
        print("   等待停止生效...")
        time.sleep(2)
        
        # 5. 右转测试
        print("\n5️⃣ 右转1.5秒...")
        robot.turn_right()
        print("   右转中...")
        time.sleep(1.5)
        
        print("\n   🛑 发送停止命令...")
        robot.stop_robot()
        print("   等待停止生效...")
        time.sleep(2)
        
        # 6. 显示详细统计
        stats = robot.stats
        print(f"\n📊 详细统计:")
        print(f"  传感器数据:")
        print(f"    MPU: {stats['mpu_count']} 包")
        print(f"    ODO: {stats['odo_count']} 包")
        print(f"    POSE: {stats['pose_count']} 包")
        print(f"  雷达扫描: {stats['lidar_count']} 次  ⭐ 关键")
        print(f"  通信:")
        print(f"    ACK确认: {stats['ack_count']} 个")
        print(f"    其他消息: {stats['other_messages']} 条")
        
        if stats['lidar_count'] == 0:
            print(f"\n⚠️ 雷达数据解析问题诊断:")
            print(f"  1. JSON重组可能失败")
            print(f"  2. 或者STM32发送的不是完整JSON")
            print(f"  3. 建议检查STM32固件的Radar_SendJSON()函数")
    
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    finally:
        robot.disconnect()

def main():
    """主菜单"""
    print("\n🤖 STM32智能小车 BLE控制程序")
    print("选择模式:")
    print("  1. 交互式控制（手动控制）")
    print("  2. 演示模式（自动演示）")
    print("  3. 调试模式（显示所有数据）⭐")
    print("  4. 退出")
    
    choice = input("\n请选择 (1/2/3/4): ").strip()
    
    if choice == '1':
        interactive_control()
    elif choice == '2':
        demo_mode()
    elif choice == '3':
        debug_mode()
    elif choice == '4':
        print("再见！👋")
    else:
        print("无效选择")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 再见！")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
