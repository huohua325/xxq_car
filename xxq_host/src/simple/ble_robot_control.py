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
    """激光雷达数据（增强版）"""
    def __init__(self, timestamp: int, total_points: int, angle_coverage: float, sectors: list):
        self.timestamp = timestamp
        self.total_points = total_points
        self.angle_coverage = angle_coverage
        self.sectors = sectors  # 现在包含增强统计字段
        self.format = "sectors_enhanced"  # 标识增强格式

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
            # JSON重组状态机
            if self._json_collecting:
                self._json_buffer += line + '\n'
                
                # 检查是否结束
                open_braces = self._json_buffer.count('{')
                close_braces = self._json_buffer.count('}')
                
                if open_braces > 0 and open_braces == close_braces:
                    self._parse_complete_json()
                    self._json_collecting = False
                    self._json_buffer = ""
                return
            
            # 检测JSON开始
            if line.strip().startswith('{'):
                self._json_collecting = True
                self._json_buffer = line + '\n'
                return
                
            # CSV格式
            if line.startswith('MPU,') or line.startswith('ODO,') or line.startswith('POSE,'):
                self._parse_csv(line)
            else:
                self._parse_other_message(line)
                
        except Exception as e:
            if self.verbose:
                print(f"⚠️ 解析失败: {line[:50]}... 错误: {e}")
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
                    
        except json.JSONDecodeError as e:
            if self.verbose:
                print(f"⚠️ JSON解析失败")
    
    def _parse_csv(self, line: str):
        """解析CSV格式数据"""
        line = line.strip()
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
    
    def stop_robot(self) -> bool:
        """停止机器人"""
        return self.send_command("MODE,0\n")
    
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
        time.sleep(3.0)
