#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化的位姿估计器
仅使用编码器数据，采用差分驱动运动学模型
"""

import math


class SimplePoseEstimator:
    """基于编码器的2D位姿估计器"""
    
    def __init__(self, wheel_base=0.185, wheel_radius=0.033, left_ppr=1560, right_ppr=780):
        """
        初始化位姿估计器
        
        Args:
            wheel_base: 轮距（米），默认0.185
            wheel_radius: 轮半径（米），默认0.033
            left_ppr: 左轮编码器分辨率（脉冲/圈），默认1560（四倍频）
            right_ppr: 右轮编码器分辨率（脉冲/圈），默认780（双倍频）
        """
        # 当前位姿
        self.x = 0.0        # X坐标（米）
        self.y = 0.0        # Y坐标（米）
        self.theta = 0.0    # 航向角（弧度）
        
        # 历史编码器值
        self.last_left_count = None
        self.last_right_count = None
        self.last_timestamp = None
        
        # 机器人参数
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.left_ppr = left_ppr
        self.right_ppr = right_ppr
        
        # 统计信息
        self.total_distance = 0.0  # 总行驶距离
        self.update_count = 0
    
    def update(self, odo_data):
        """
        从里程计数据更新位姿
        
        Args:
            odo_data: 字典，包含以下字段：
                - left_count: 左轮编码器计数
                - right_count: 右轮编码器计数
                - timestamp: 时间戳（毫秒）
                或者SimpleBLERobotComm的ODO回调数据对象
        
        Returns:
            (x, y, theta): 当前位姿（米，米，弧度）
        """
        # 处理数据对象（支持字典或对象）
        if hasattr(odo_data, 'left_count'):
            left_count = odo_data.left_count
            right_count = odo_data.right_count
            timestamp = odo_data.timestamp
        else:
            left_count = odo_data.get('left_count', 0)
            right_count = odo_data.get('right_count', 0)
            timestamp = odo_data.get('timestamp', 0)
        
        # 初始化
        if self.last_left_count is None:
            self.last_left_count = left_count
            self.last_right_count = right_count
            self.last_timestamp = timestamp
            return self.get_pose()
        
        # 计算编码器增量
        delta_left = left_count - self.last_left_count
        delta_right = right_count - self.last_right_count
        
        # 异常值过滤
        if abs(delta_left) > 10000 or abs(delta_right) > 10000:
            self.last_left_count = left_count
            self.last_right_count = right_count
            self.last_timestamp = timestamp
            return self.get_pose()
        
        # 更新历史值
        self.last_left_count = left_count
        self.last_right_count = right_count
        self.last_timestamp = timestamp
        
        # 转换为轮子行驶距离（米）
        left_distance = (delta_left / self.left_ppr) * (2 * math.pi * self.wheel_radius)
        right_distance = (delta_right / self.right_ppr) * (2 * math.pi * self.wheel_radius)
        
        # 差分驱动运动学更新
        center_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base
        
        # 更新位姿（使用中点法）
        if abs(delta_theta) < 1e-6:
            # 直线运动
            self.x += center_distance * math.cos(self.theta)
            self.y += center_distance * math.sin(self.theta)
        else:
            # 曲线运动
            mid_theta = self.theta + delta_theta / 2.0
            self.x += center_distance * math.cos(mid_theta)
            self.y += center_distance * math.sin(mid_theta)
            self.theta += delta_theta
        
        # 归一化角度到 [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # 更新统计
        self.total_distance += abs(center_distance)
        self.update_count += 1
        
        return self.get_pose()
    
    def get_pose(self):
        """获取当前位姿"""
        return (self.x, self.y, self.theta)
    
    def get_pose_degrees(self):
        """获取当前位姿（角度用度表示）"""
        return (self.x, self.y, math.degrees(self.theta))
    
    def reset(self, x=0.0, y=0.0, theta=0.0):
        """重置位姿"""
        self.x = x
        self.y = y
        self.theta = theta
        self.last_left_count = None
        self.last_right_count = None
        self.last_timestamp = None
        self.total_distance = 0.0
        self.update_count = 0
    
    def get_statistics(self):
        """获取统计信息"""
        return {
            'total_distance': self.total_distance,
            'update_count': self.update_count,
            'current_pose': self.get_pose(),
            'current_pose_deg': self.get_pose_degrees()
        }
    
    def __repr__(self):
        """字符串表示"""
        x, y, theta_deg = self.get_pose_degrees()
        return f"SimplePoseEstimator(x={x:.3f}m, y={y:.3f}m, θ={theta_deg:.1f}°)"
