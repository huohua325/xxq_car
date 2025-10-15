#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达数据包装器
支持原始点云数据（360个点）+ 兼容旧的8扇区JSON格式
"""

import time


class SimpleLidarWrapper:
    """激光雷达数据包装器（支持点云格式 + 向后兼容）"""
    
    def __init__(self, robot):
        """
        初始化雷达包装器
        
        Args:
            robot: SimpleBLERobotComm实例
        """
        self.robot = robot
        self.latest_scan = None
        self.scan_count = 0
        self.format_type = None  # 'pointcloud' or 'sectors'
        
        # 注册回调
        self.robot.on_lidar_update = self._on_lidar_callback
    
    def _on_lidar_callback(self, lidar_data):
        """雷达数据回调（支持点云 + 扇区格式）"""
        # 判断数据类型
        if hasattr(lidar_data, 'points'):
            # 新格式：点云数据（360点）
            self.latest_scan = lidar_data.points
            self.format_type = 'pointcloud'
        elif hasattr(lidar_data, 'sectors'):
            # 旧格式：扇区数据（8点）
            self.latest_scan = self._convert_sectors_to_points(lidar_data)
            self.format_type = 'sectors'
        
        self.scan_count += 1
    
    def _convert_sectors_to_points(self, lidar_data):
        """将扇区数据转换为点集（保留增强统计字段）"""
        points = []
        
        for sector in lidar_data.sectors:
            sector_id = sector.get('sector_id', -1)
            angle = sector.get('angle_center', 0)
            count = sector.get('count', 0)
            
            # 只添加有效数据的扇区
            if count > 0:
                # 🔧 方向补偏修正：基于实时验证数据，需要-225度偏移
                # 分析结果：正前方检测到扇区5，需要-225度补偿
                corrected_sector_id = (sector_id - 5) % 8  # -5个扇区 = -225度
                
                points.append({
                    'sector_id': corrected_sector_id,  # ⭐ 使用修正后的扇区ID
                    'angle': angle,
                    'count': count,
                    # 基础统计
                    'distance': sector.get('min_dist', 999.0),
                    'min_dist': sector.get('min_dist', 999.0),
                    'max_dist': sector.get('max_dist', 999.0),
                    'avg_dist': sector.get('avg_dist', 999.0),
                    'avg_distance': sector.get('avg_dist', 999.0),
                    # 增强统计
                    'median_dist': sector.get('median_dist', 999.0),
                    'median_distance': sector.get('median_dist', 999.0),
                    'p10_dist': sector.get('p10_dist', 999.0),
                    'p10_distance': sector.get('p10_dist', 999.0),
                    'p90_dist': sector.get('p90_dist', 999.0),
                    'p90_distance': sector.get('p90_dist', 999.0),
                    'std_dist': sector.get('std_dist', 0.0),
                    'std_distance': sector.get('std_dist', 0.0),
                    'quality_avg': sector.get('quality_avg', 0),
                    'outlier_count': sector.get('outlier_count', 0)
                })
        
        return points
    
    def request_scan(self, timeout=3.0):
        """请求雷达扫描并等待结果"""
        self.latest_scan = None
        self.robot.send_command('A\n')
        
        start_time = time.time()
        while self.latest_scan is None:
            if time.time() - start_time > timeout:
                return None
            time.sleep(0.05)
        
        return self.latest_scan
    
    def get_distance_at_angle(self, target_angle, tolerance=15):
        """获取指定角度的距离"""
        if not self.latest_scan:
            return 999.0
        
        # 自动调整容差
        if self.format_type == 'pointcloud':
            tolerance = 5
        
        distances = []
        for p in self.latest_scan:
            angle_diff = abs(p['angle'] - target_angle)
            if angle_diff > 180:
                angle_diff = 360 - angle_diff
            
            if angle_diff <= tolerance:
                distances.append(p['distance'])
        
        return min(distances) if distances else 999.0
    
    def get_distance_in_range(self, angle_min, angle_max):
        """获取角度范围内的最小距离"""
        if not self.latest_scan:
            return 999.0
        
        distances = []
        for p in self.latest_scan:
            angle = p['angle']
            if angle_min <= angle_max:
                if angle_min <= angle <= angle_max:
                    distances.append(p['distance'])
            else:
                if angle >= angle_min or angle <= angle_max:
                    distances.append(p['distance'])
        
        return min(distances) if distances else 999.0
    
    def get_obstacles_summary(self):
        """获取障碍物概览（使用增强统计）"""
        return {
            'front': self.get_distance_in_range(345, 15),
            'left': self.get_distance_in_range(75, 105),
            'right': self.get_distance_in_range(255, 285),
            'back': self.get_distance_in_range(165, 195)
        }
    
    def get_sector_stats(self, sector_id: int):
        """
        获取扇区完整统计信息（增强版）
        
        Args:
            sector_id: 扇区编号（0-7）
        
        Returns:
            dict: 包含所有统计指标的字典，如果无数据返回None
        """
        if not self.latest_scan:
            return None
        
        for p in self.latest_scan:
            if p.get('sector_id') == sector_id:
                return {
                    'min': p.get('distance', p.get('min_dist', 999.0)),
                    'max': p.get('max_distance', p.get('max_dist', 999.0)),
                    'avg': p.get('avg_distance', p.get('avg_dist', 999.0)),
                    'median': p.get('median_distance', p.get('median_dist', 999.0)),
                    'p10': p.get('p10_distance', p.get('p10_dist', 999.0)),
                    'p90': p.get('p90_distance', p.get('p90_dist', 999.0)),
                    'std': p.get('std_distance', p.get('std_dist', 0.0)),
                    'quality': p.get('quality_avg', 0),
                    'outliers': p.get('outlier_count', 0),
                    'count': p.get('count', 0)
                }
        
        return None
    
    def get_sector_median(self, sector_id: int) -> float:
        """
        获取扇区中位数距离（推荐用于避障决策）
        
        中位数比平均值更可靠，不受离群值影响
        """
        stats = self.get_sector_stats(sector_id)
        return stats['median'] if stats else 999.0
    
    def get_sector_p10(self, sector_id: int) -> float:
        """
        获取扇区10%分位数（最近10%的障碍物距离）
        
        适用于紧急避障，代表最近的一小部分障碍物
        """
        stats = self.get_sector_stats(sector_id)
        return stats['p10'] if stats else 999.0
    
    def check_critical_obstacle(self, sector_id: int, threshold: float = 0.3) -> bool:
        """
        检查扇区是否有危险障碍物（使用p10）
        
        Args:
            sector_id: 扇区编号
            threshold: 危险距离阈值（米），默认0.3m
        
        Returns:
            True: 有危险障碍物
            False: 安全
        """
        p10 = self.get_sector_p10(sector_id)
        return p10 < threshold
    
    def get_best_direction_enhanced(self) -> int:
        """
        获取最佳移动方向（基于中位数）
        
        Returns:
            扇区编号（0-7），-1表示无有效数据
        """
        if not self.latest_scan:
            return -1
        
        best_sector = -1
        max_median = 0.0
        
        for i in range(8):
            median = self.get_sector_median(i)
            if median > max_median and median < 900:  # 排除无效数据
                max_median = median
                best_sector = i
        
        return best_sector
    
    def get_latest_scan(self):
        """获取最新的扫描数据"""
        return self.latest_scan
    
    def has_scan(self):
        """检查是否有扫描数据"""
        return self.latest_scan is not None
    
    def get_scan_count(self):
        """获取扫描次数"""
        return self.scan_count
    
    def get_point_cloud(self):
        """获取完整点云数据（用于SLAM建图）"""
        if not self.latest_scan:
            return None
        
        return [(p['angle'], p['distance']) for p in self.latest_scan]
    
    def __repr__(self):
        """字符串表示"""
        if self.latest_scan:
            return f"SimpleLidarWrapper(format={self.format_type}, points={len(self.latest_scan)}, scans={self.scan_count})"
        else:
            return f"SimpleLidarWrapper(no_data, scans={self.scan_count})"
