"""
机器人通信模块
负责与STM32固件的串口通信，数据收发和解析
"""

import serial
import json
import time
import threading
from typing import Optional, Callable
import logging
import numpy as np

from .protocol import (
    LidarData, MPUData, OdometryData, PoseData,
    CommandType, RobotMode
)


class RobotComm:
    """机器人通信类
    
    实现与STM32固件的双向通信：
    - 接收：雷达、MPU、里程计、位姿数据
    - 发送：导航指令、速度控制、模式切换
    
    Example:
        >>> comm = RobotComm(port='/dev/ttyUSB0', baudrate=115200)
        >>> comm.on_lidar_update = lambda data: print(f"收到雷达数据: {data.total_points}点")
        >>> comm.start()
        >>> comm.request_lidar_scan()
        >>> time.sleep(2)
        >>> comm.stop()
    """
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        """初始化通信对象
        
        Args:
            port: 串口设备路径 (WSL/Linux: '/dev/ttyUSB0', Windows: 'COM5')
            baudrate: 波特率，默认115200
        """
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None
        
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
        
        # 日志
        self.logger = logging.getLogger(__name__)
    
    def connect(self) -> bool:
        """连接串口
        
        Returns:
            连接成功返回True，失败返回False
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=1.0
            )
            self.logger.info(f"串口已连接: {self.port} @ {self.baudrate}")
            return True
        except serial.SerialException as e:
            self.logger.error(f"串口连接失败: {e}")
            return False
    
    def start(self) -> bool:
        """启动接收线程
        
        Returns:
            启动成功返回True
        """
        if not self.serial or not self.serial.is_open:
            if not self.connect():
                return False
        
        self.running = True
        self.receive_thread = threading.Thread(
            target=self._receive_loop,
            daemon=True,
            name="RobotComm-Receiver"
        )
        self.receive_thread.start()
        self.logger.info("接收线程已启动")
        return True
    
    def stop(self):
        """停止通信并关闭串口"""
        self.running = False
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
            self.logger.info("接收线程已停止")
        
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.logger.info("串口已关闭")
    
    def _receive_loop(self):
        """接收循环（在独立线程中运行）"""
        buffer = ""
        
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    # 读取可用数据
                    data = self.serial.read(self.serial.in_waiting)
                    decoded = data.decode('utf-8', errors='ignore')
                    buffer += decoded
                    
                    # 按行处理
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self._parse_line(line.strip())
                else:
                    # 没有数据时短暂休眠，避免CPU占用过高
                    time.sleep(0.001)
                    
            except serial.SerialException as e:
                self.logger.error(f"串口读取错误: {e}")
                time.sleep(0.1)
            except Exception as e:
                self.logger.error(f"接收循环异常: {e}")
                time.sleep(0.1)
    
    def _parse_line(self, line: str):
        """解析一行数据
        
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
            else:
                self._parse_csv(line)
                
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
            if data_type == 'MPU' and len(parts) == 10:
                self.latest_mpu = MPUData(
                    timestamp=int(parts[1]),
                    roll=float(parts[2]),
                    pitch=float(parts[3]),
                    accel=[float(parts[4]), float(parts[5]), float(parts[6])],
                    gyro=[float(parts[7]), float(parts[8]), float(parts[9])]
                )
                if self.on_mpu_update:
                    self.on_mpu_update(self.latest_mpu)
            
            elif data_type == 'ODO' and len(parts) == 6:
                self.latest_odom = OdometryData(
                    timestamp=int(parts[1]),
                    left_speed=float(parts[2]),
                    right_speed=float(parts[3]),
                    left_count=int(parts[4]),
                    right_count=int(parts[5])
                )
                if self.on_odom_update:
                    self.on_odom_update(self.latest_odom)
            
            elif data_type == 'POSE' and len(parts) == 5:
                self.latest_pose = PoseData(
                    timestamp=int(parts[1]),
                    x=float(parts[2]),
                    y=float(parts[3]),
                    theta=float(parts[4])
                )
                if self.on_pose_update:
                    self.on_pose_update(self.latest_pose)
                    
        except (ValueError, IndexError) as e:
            self.logger.warning(f"CSV解析错误: {line} - {e}")
    
    # ========== 发送指令方法 ==========
    
    def send_navigation_command(self, x: float, y: float, theta: float, speed: float):
        """发送导航指令
        
        Args:
            x: 目标X坐标（米）
            y: 目标Y坐标（米）
            theta: 目标朝向（度）
            speed: 目标速度（RPS）
        """
        cmd = f"NAV,{x:.3f},{y:.3f},{theta:.2f},{speed:.2f}\n"
        self._send_command(cmd)
    
    def send_speed_command(self, left_speed: float, right_speed: float):
        """发送速度指令
        
        Args:
            left_speed: 左轮速度（RPS）
            right_speed: 右轮速度（RPS）
        """
        cmd = f"SPD,{left_speed:.2f},{right_speed:.2f}\n"
        self._send_command(cmd)
    
    def send_mode_command(self, mode: int):
        """发送模式指令
        
        Args:
            mode: 运动模式（参见RobotMode）
        """
        cmd = f"MODE,{mode}\n"
        self._send_command(cmd)
    
    def request_lidar_scan(self):
        """请求雷达扫描"""
        self._send_command("A")
    
    def stop_robot(self):
        """紧急停止机器人"""
        self.send_mode_command(RobotMode.STOP)
    
    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """重置机器人位姿
        
        Args:
            x: X坐标（米），默认0
            y: Y坐标（米），默认0
            theta: 航向角（弧度），默认0 - 内部自动转换为度发送
        """
        # ✅ 转换为度（STM32接收度）
        theta_deg = np.rad2deg(theta)
        cmd = f"RESET,{x:.3f},{y:.3f},{theta_deg:.3f}\n"
        self._send_command(cmd)
    
    def _send_command(self, cmd: str):
        """发送命令到串口
        
        Args:
            cmd: 命令字符串
        """
        try:
            if self.serial and self.serial.is_open:
                self.serial.write(cmd.encode('utf-8'))
                self.logger.debug(f"发送命令: {cmd.strip()}")
            else:
                self.logger.warning("串口未打开，无法发送命令")
        except serial.SerialException as e:
            self.logger.error(f"发送命令失败: {e}")
    
    # ========== 辅助方法 ==========
    
    def is_connected(self) -> bool:
        """检查串口是否已连接
        
        Returns:
            已连接返回True
        """
        return self.serial is not None and self.serial.is_open
    
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
        
        if self.serial and self.serial.is_open:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.logger.info("串口缓冲区已清空")


# 便捷的日志配置函数
def setup_comm_logging(level=logging.INFO):
    """配置通信模块日志
    
    Args:
        level: 日志级别
    """
    logging.basicConfig(
        level=level,
        format='[%(asctime)s] %(name)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )




