"""
机器人BLE通信模块
负责与STM32固件的BLE通信，数据收发和解析
"""

import asyncio
import json
import time
import threading
from typing import Optional, Callable
import logging
import numpy as np
from bleak import BleakClient

from .protocol import (
    LidarData, MPUData, OdometryData, PoseData,
    CommandType, RobotMode
)


class RobotCommBLE:
    """机器人BLE通信类
    
    实现与STM32固件的BLE双向通信（通过HC-04BLE等BLE模块）
    接口与RobotComm完全兼容，可直接替换
    
    Example:
        >>> comm = RobotCommBLE(address='C4:25:01:20:02:8E')
        >>> comm.on_lidar_update = lambda data: print(f"收到雷达数据: {data.total_points}点")
        >>> comm.start()
        >>> comm.request_lidar_scan()
        >>> time.sleep(2)
        >>> comm.stop()
    """
    
    # BLE UART服务UUID（HC-04BLE常用）
    UART_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
    UART_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
    
    def __init__(self, address: str):
        """初始化BLE通信对象
        
        Args:
            address: BLE设备地址 (如 'C4:25:01:20:02:8E')
        """
        self.address = address
        self.client: Optional[BleakClient] = None
        self.running = False
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        self._connect_event = threading.Event()
        self._stop_requested = False
        
        # 数据缓冲区（处理BLE分包）
        self.buffer = ""
        
        # 最新数据缓存（与RobotComm保持一致）
        self.latest_lidar: Optional[LidarData] = None
        self.latest_mpu: Optional[MPUData] = None
        self.latest_odom: Optional[OdometryData] = None
        self.latest_pose: Optional[PoseData] = None
        
        # 回调函数（与RobotComm保持一致）
        self.on_lidar_update: Optional[Callable[[LidarData], None]] = None
        self.on_mpu_update: Optional[Callable[[MPUData], None]] = None
        self.on_odom_update: Optional[Callable[[OdometryData], None]] = None
        self.on_pose_update: Optional[Callable[[PoseData], None]] = None
        
        # 阶段1新增：通用消息回调（用于接收STM32的ACK、调试信息等）
        self.on_message: Optional[Callable[[str], None]] = None
        
        # 日志
        self.logger = logging.getLogger(__name__)
    
    def start(self) -> bool:
        """启动BLE通信（连接并启动接收线程）
        
        Returns:
            启动成功返回True
        """
        if self.running:
            return True
        
        self.running = True
        self._stop_requested = False
        self._connect_event.clear()
        
        # 在独立线程中运行BLE事件循环
        self.thread = threading.Thread(
            target=self._run_ble_thread,
            daemon=True,
            name="RobotCommBLE"
        )
        self.thread.start()
        
        # 等待连接完成
        if self._connect_event.wait(timeout=10.0):
            if self.client and self.client.is_connected:
                self.logger.info(f"BLE已连接: {self.address}")
                self.logger.info("BLE接收线程已启动")
                return True
        
        self.logger.error("BLE连接超时")
        self.stop()
        return False
    
    def _run_ble_thread(self):
        """在独立线程中运行BLE事件循环"""
        # 创建新的事件循环
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            # 运行主循环
            self.loop.run_until_complete(self._ble_main_loop())
        except Exception as e:
            self.logger.error(f"BLE线程异常: {e}")
        finally:
            # 关闭事件循环
            try:
                # 取消所有未完成的任务
                pending = asyncio.all_tasks(self.loop)
                for task in pending:
                    task.cancel()
                # 运行直到所有任务取消
                if pending:
                    self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except:
                pass
            finally:
                self.loop.close()
    
    async def _ble_main_loop(self):
        """BLE主循环（连接、保持活跃、断开）"""
        try:
            # 连接BLE设备
            self.client = BleakClient(self.address, timeout=20.0)
            await self.client.connect()
            
            # 订阅通知
            await self.client.start_notify(
                self.UART_CHAR_UUID,
                self._notification_handler
            )
            
            # 通知主线程连接成功
            self._connect_event.set()
            
            # 保持连接，直到停止请求
            while not self._stop_requested:
                if not self.client.is_connected:
                    self.logger.warning("BLE连接断开")
                    break
                await asyncio.sleep(0.1)
            
        except Exception as e:
            self.logger.error(f"BLE连接失败: {e}")
            self._connect_event.set()  # 通知连接完成（失败）
        finally:
            # 清理资源
            if self.client and self.client.is_connected:
                try:
                    await self.client.stop_notify(self.UART_CHAR_UUID)
                    await self.client.disconnect()
                except:
                    pass
    
    def stop(self):
        """停止通信并断开BLE连接"""
        if not self.running:
            return
        
        self.running = False
        self._stop_requested = True
        
        # 等待BLE线程结束
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=3.0)
            if self.thread.is_alive():
                self.logger.warning("BLE线程未能正常停止")
            else:
                self.logger.info("BLE线程已停止")
                self.logger.info("BLE已断开")
    
    def _notification_handler(self, sender, data: bytearray):
        """BLE通知处理器（接收数据回调）
        
        Args:
            sender: BLE特征对象
            data: 接收到的数据
        """
        # 如果已经停止，忽略新数据
        if self._stop_requested or not self.running:
            return
            
        try:
            # 解码数据
            decoded = data.decode('utf-8', errors='ignore')
            self.buffer += decoded
            
            # 按行处理（BLE会分包，需要重组）
            while '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)
                self._parse_line(line.strip())
                
        except Exception as e:
            if not self._stop_requested:  # 只在运行时记录错误
                self.logger.error(f"数据处理异常: {e}")
    
    def _parse_line(self, line: str):
        """解析一行数据（与RobotComm相同）
        
        Args:
            line: 接收到的一行数据（已去除换行符）
        """
        if not line:
            return
        
        try:
            # JSON格式（雷达数据）
            if line.startswith('{'):
                self._parse_json(line)
            # CSV格式（MPU/ODO/POSE）
            elif line.startswith('MPU,') or line.startswith('ODO,') or line.startswith('POSE,'):
                self._parse_csv(line)
            # 阶段1：其他消息（ACK、调试信息等）
            else:
                if self.on_message:
                    self.on_message(line)
                
        except Exception as e:
            self.logger.warning(f"数据解析失败: {line[:50]}... 错误: {e}")
    
    def _parse_json(self, line: str):
        """解析JSON格式数据（雷达）
        
        Args:
            line: JSON字符串
        """
        try:
            data = json.loads(line)
            
            if data.get('type') == 'LIDAR':
                lidar_info = data.get('data', {})
                self.latest_lidar = LidarData(
                    timestamp=data.get('timestamp', 0),
                    total_points=lidar_info.get('total_points', 0),
                    angle_coverage=lidar_info.get('angle_coverage', 0.0),
                    sectors=lidar_info.get('sectors', [])
                )
                
                if self.on_lidar_update:
                    self.on_lidar_update(self.latest_lidar)
                    
        except json.JSONDecodeError as e:
            self.logger.warning(f"JSON解析错误: {e}")
    
    def _parse_csv(self, line: str):
        """解析CSV格式数据（MPU/ODO/POSE）
        
        Args:
            line: CSV字符串
        """
        parts = line.split(',')
        
        if len(parts) < 2:
            return
        
        data_type = parts[0]
        
        try:
            if data_type == 'MPU':
                self._parse_mpu(parts)
            elif data_type == 'ODO':
                self._parse_odom(parts)
            elif data_type == 'POSE':
                self._parse_pose(parts)
        except Exception as e:
            self.logger.warning(f"CSV解析错误 {data_type}: {e}")
    
    def _parse_mpu(self, parts: list):
        """解析MPU数据: MPU,timestamp,roll,pitch,ax,ay,az,gx,gy,gz"""
        if len(parts) >= 10:
            self.latest_mpu = MPUData(
                timestamp=int(parts[1]),
                roll=float(parts[2]),
                pitch=float(parts[3]),
                accel=[float(parts[4]), float(parts[5]), float(parts[6])],
                gyro=[float(parts[7]), float(parts[8]), float(parts[9])]
            )
            
            if self.on_mpu_update:
                self.on_mpu_update(self.latest_mpu)
    
    def _parse_odom(self, parts: list):
        """解析里程计数据: ODO,timestamp,left_speed,right_speed,left_count,right_count"""
        if len(parts) >= 6:
            self.latest_odom = OdometryData(
                timestamp=int(parts[1]),
                left_speed=float(parts[2]),
                right_speed=float(parts[3]),
                left_count=int(parts[4]),
                right_count=int(parts[5])
            )
            
            if self.on_odom_update:
                self.on_odom_update(self.latest_odom)
    
    def _parse_pose(self, parts: list):
        """解析位姿数据: POSE,timestamp,x,y,theta"""
        if len(parts) >= 5:
            self.latest_pose = PoseData(
                timestamp=int(parts[1]),
                x=float(parts[2]),
                y=float(parts[3]),
                theta=float(parts[4])
            )
            
            if self.on_pose_update:
                self.on_pose_update(self.latest_pose)
    
    def _send_command(self, cmd: str) -> bool:
        """发送命令到BLE设备（内部方法）
        
        Args:
            cmd: 命令字符串
            
        Returns:
            发送成功返回True
        """
        if not self.running or self._stop_requested:
            self.logger.warning("BLE未运行，无法发送命令")
            return False
            
        if not self.client or not self.client.is_connected:
            self.logger.warning("BLE未连接，无法发送命令")
            return False
        
        try:
            # 在BLE线程的事件循环中异步发送
            if self.loop and not self.loop.is_closed():
                future = asyncio.run_coroutine_threadsafe(
                    self._async_send(cmd.encode('utf-8')),
                    self.loop
                )
                # 等待发送完成（最多1秒）
                future.result(timeout=1.0)
                self.logger.debug(f"发送命令: {cmd.strip()}")
                return True
            else:
                self.logger.warning("事件循环不可用")
                return False
                
        except Exception as e:
            self.logger.error(f"命令发送失败: {e}")
            return False
    
    async def _async_send(self, data: bytes):
        """异步发送数据"""
        try:
            if self.client and self.client.is_connected:
                await self.client.write_gatt_char(self.UART_CHAR_UUID, data)
        except Exception as e:
            self.logger.error(f"异步发送失败: {e}")
    
    # ========== 发送指令方法（与RobotComm保持一致）==========
    
    def send_navigation_command(self, x: float, y: float, theta: float, speed: float) -> bool:
        """发送导航指令
        
        Args:
            x: 目标X坐标（米）
            y: 目标Y坐标（米）
            theta: 目标朝向（度）
            speed: 目标速度（RPS）
        """
        cmd = f"NAV,{x:.3f},{y:.3f},{theta:.2f},{speed:.2f}\n"
        return self._send_command(cmd)
    
    def send_speed_command(self, left_speed: float, right_speed: float) -> bool:
        """发送速度指令
        
        Args:
            left_speed: 左轮速度（RPS）
            right_speed: 右轮速度（RPS）
        """
        cmd = f"SPD,{left_speed:.2f},{right_speed:.2f}\n"
        return self._send_command(cmd)
    
    def send_mode_command(self, mode: int) -> bool:
        """发送模式指令
        
        Args:
            mode: 运动模式（参见RobotMode）
        """
        cmd = f"MODE,{mode}\n"
        return self._send_command(cmd)
    
    def request_lidar_scan(self) -> bool:
        """请求雷达扫描"""
        return self._send_command("A\n")  # 修复：添加换行符
    
    def stop_robot(self) -> bool:
        """紧急停止机器人"""
        return self.send_mode_command(RobotMode.STOP)
    
    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> bool:
        """重置机器人位姿
        
        Args:
            x: X坐标（米），默认0
            y: Y坐标（米），默认0
            theta: 航向角（弧度），默认0 - 内部自动转换为度发送
        """
        # ✅ 转换为度（STM32接收度）
        theta_deg = np.rad2deg(theta)
        cmd = f"RESET,{x:.3f},{y:.3f},{theta_deg:.3f}\n"
        return self._send_command(cmd)
    
    # ========== 辅助方法（与RobotComm保持一致）==========
    
    def is_connected(self) -> bool:
        """检查BLE是否已连接
        
        Returns:
            已连接返回True
        """
        return self.client is not None and self.client.is_connected
    
    def get_latest_data(self) -> dict:
        """获取所有最新数据
        
        Returns:
            包含所有传感器最新数据的字典
        """
        return {
            'lidar': self.latest_lidar,
            'mpu': self.latest_mpu,
            'odom': self.latest_odom,
            'pose': self.latest_pose
        }
    
    def clear_buffers(self):
        """清空数据缓存"""
        self.latest_lidar = None
        self.latest_mpu = None
        self.latest_odom = None
        self.latest_pose = None
        self.logger.info("数据缓存已清空")
    
    def __del__(self):
        """析构函数，确保资源释放"""
        self.stop()

