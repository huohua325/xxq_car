"""
通信协议数据类定义
定义STM32固件与Python主机端之间传输的数据结构
"""

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class LidarData:
    """激光雷达数据
    
    Attributes:
        timestamp: 毫秒时间戳
        total_points: 有效测量点总数
        angle_coverage: 角度覆盖范围（度）
        sectors: 8个扇区的统计数据列表
    """
    timestamp: int
    total_points: int
    angle_coverage: float
    sectors: List[dict]  # 每个扇区包含: sector_id, angle_center, count, min_dist, avg_dist
    
    def get_sector(self, sector_id: int) -> Optional[dict]:
        """获取指定扇区数据
        
        Args:
            sector_id: 扇区编号 (0-7)
            
        Returns:
            扇区数据字典，如果不存在返回None
        """
        for sector in self.sectors:
            if sector.get('sector_id') == sector_id:
                return sector
        return None


@dataclass
class MPUData:
    """MPU6500姿态传感器数据
    
    Attributes:
        timestamp: 毫秒时间戳
        roll: 横滚角（度）
        pitch: 俯仰角（度）
        accel: 三轴加速度 [x, y, z] (m/s²)
        gyro: 三轴角速度 [x, y, z] (°/s)
    """
    timestamp: int
    roll: float
    pitch: float
    accel: List[float]  # [x, y, z]
    gyro: List[float]   # [x, y, z]
    
    @property
    def accel_magnitude(self) -> float:
        """加速度向量的模"""
        return (self.accel[0]**2 + self.accel[1]**2 + self.accel[2]**2) ** 0.5


@dataclass
class OdometryData:
    """里程计数据
    
    Attributes:
        timestamp: 毫秒时间戳
        left_speed: 左轮速度（RPS - 圈/秒）
        right_speed: 右轮速度（RPS - 圈/秒）
        left_count: 左轮编码器累计计数
        right_count: 右轮编码器累计计数
    """
    timestamp: int
    left_speed: float
    right_speed: float
    left_count: int
    right_count: int
    
    @property
    def average_speed(self) -> float:
        """平均速度（RPS）"""
        return (self.left_speed + self.right_speed) / 2.0
    
    @property
    def speed_diff(self) -> float:
        """左右轮速度差"""
        return abs(self.left_speed - self.right_speed)


@dataclass
class PoseData:
    """机器人位姿数据
    
    Attributes:
        timestamp: 毫秒时间戳
        x: X坐标（米）
        y: Y坐标（米）
        theta: 朝向角（度，0°为正前方）
    """
    timestamp: int
    x: float
    y: float
    theta: float
    
    def distance_to(self, target_x: float, target_y: float) -> float:
        """计算到目标点的距离
        
        Args:
            target_x: 目标X坐标
            target_y: 目标Y坐标
            
        Returns:
            欧氏距离（米）
        """
        return ((self.x - target_x)**2 + (self.y - target_y)**2) ** 0.5


# 命令类型枚举
class CommandType:
    """命令类型定义"""
    NAVIGATION = "NAV"  # 导航控制
    SPEED = "SPD"       # 速度控制
    MODE = "MODE"       # 模式控制
    LIDAR_SCAN = "A"    # 请求雷达扫描
    REQUEST_LIDAR = "A" # 请求雷达扫描（别名）
    RESET_POSE = "RESET_POSE"  # 重置位姿


class RobotMode:
    """机器人运动模式"""
    STOP = 0            # 停止
    FORWARD = 1         # PID前进
    BACKWARD = 2        # PID后退
    TURN_LEFT = 3       # 左转
    TURN_RIGHT = 4      # 右转
    AUTO_NAV = 5        # 自动导航模式

