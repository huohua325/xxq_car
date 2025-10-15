"""
maze通信层到xxq_host控制器的适配器
"""

import numpy as np
from typing import Tuple, List, Optional
import sys
import os

# 添加项目根目录
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# maze模块
from src.simple.ble_robot_control import SimpleBLERobotComm
from src.simple.simple_pose_estimator import SimplePoseEstimator
from src.simple.simple_lidar import SimpleLidarWrapper
from src.simple.simple_motion import SimpleMotionController

# xxq_host配置
import config


class MazeAdapter:
    """maze通信层适配器"""
    
    def __init__(self, ble_address: str = None):
        """
        初始化适配器
        
        Args:
            ble_address: BLE地址，None则使用config中的默认值
        """
        # BLE通信
        address = ble_address or getattr(config, 'BLE_ADDRESS', 'C4:25:01:20:02:8E')
        self.robot = SimpleBLERobotComm(address=address, verbose=False)
        
        # maze三大模块
        self.pose_estimator = SimplePoseEstimator(
            wheel_base=config.WHEEL_BASE,
            wheel_radius=config.WHEEL_RADIUS,
            left_ppr=config.LEFT_ENCODER_PPR,
            right_ppr=config.RIGHT_ENCODER_PPR
        )
        self.lidar_wrapper = SimpleLidarWrapper(self.robot)
        self.motion_controller = SimpleMotionController(self.robot)
        
        # 注册ODO回调
        self.robot.on_odom_update = self._on_odo_update
        
        # 当前位姿缓存
        self._current_pose = (0.0, 0.0, 0.0)
        
        print(f"[MazeAdapter] 初始化完成，BLE地址: {address}")
    
    def _on_odo_update(self, data):
        """ODO数据回调"""
        self._current_pose = self.pose_estimator.update(data)
    
    def connect(self) -> bool:
        """连接BLE"""
        return self.robot.connect()
    
    def disconnect(self):
        """断开BLE"""
        self.robot.disconnect()
    
    def get_pose(self) -> Tuple[float, float, float]:
        """
        获取当前位姿
        
        Returns:
            (x, y, theta): 位姿（米，米，弧度）
        """
        return self._current_pose
    
    def scan_lidar(self, timeout: float = 6.0) -> Optional[List[Tuple[float, float]]]:
        """
        请求雷达扫描
        
        Args:
            timeout: 超时时间（秒）
        
        Returns:
            [(angle_deg, distance_m), ...] 或 None
        """
        scan = self.lidar_wrapper.request_scan(timeout=timeout)
        if not scan:
            return None
        
        # 转换为(angle, distance)元组列表
        return [(p['angle'], p['distance']) for p in scan]
    
    def move_forward(self, distance: float = 0.26):
        """
        前进指定距离
        
        Args:
            distance: 目标距离（米），默认0.26m
        """
        if distance < 0.30:
            self.motion_controller.forward('medium')  # 26cm
        else:
            self.motion_controller.forward('long')    # 32cm
    
    def turn_angle(self, angle_deg: float):
        """
        转向指定角度
        
        Args:
            angle_deg: 转向角度（度），正数=左转，负数=右转
        """
        if angle_deg > 0:
            # 左转
            if abs(angle_deg) <= 50:
                self.motion_controller.turn_left(45)
            else:
                self.motion_controller.turn_left(90)
        else:
            # 右转
            if abs(angle_deg) <= 50:
                self.motion_controller.turn_right(45)
            else:
                self.motion_controller.turn_right(90)
    
    def stop(self):
        """停止运动"""
        self.motion_controller.stop()
    
    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """
        重置位姿
        
        Args:
            x, y: 坐标（米）
            theta: 航向角（弧度）
        """
        self.pose_estimator.reset(x, y, theta)
        self._current_pose = (x, y, theta)


