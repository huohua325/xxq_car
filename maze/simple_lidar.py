#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达数据包装器
支持原始点云数据（360个点）+ 兼容旧的8扇区JSON格式
"""

import time
import math


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
        self.format_type = None  # 'pointcloud' or 'sectors'（新增）
        
        # 注册回调
        self.robot.on_lidar_update = self._on_lidar_callback
    
    def _on_lidar_callback(self, lidar_data):
        """
        雷达数据回调（支持点云 + 扇区格式）
        
        Args:
            lidar_data: SimpleBLERobotComm的雷达数据对象
        """
        # 🆕 判断数据类型（自动兼容新旧格式）
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
        """
        将扇区数据转换为点集
        
        Args:
            lidar_data: 雷达数据对象
        
        Returns:
            list: [{"angle": 0, "distance": 1.2, "avg_distance": 1.5}, ...]
        """
        points = []
        
        for sector in lidar_data.sectors:
            angle = sector.get('angle_center', 0)
            min_dist = sector.get('min_dist', 0)
            avg_dist = sector.get('avg_dist', 0)
            count = sector.get('count', 0)
            
            # 只保留有效数据
            if count > 0 and min_dist > 0:
                points.append({
                    'angle': angle,
                    'distance': min_dist,       # 使用最小距离（保守策略）
                    'avg_distance': avg_dist,
                    'count': count
                })
        
        return points
    
    def request_scan(self, timeout=3.0):
        """
        请求雷达扫描并等待结果
        
        Args:
            timeout: 超时时间（秒），默认3秒
        
        Returns:
            list: 扫描点集，或None如果超时
        """
        # 清空之前的扫描
        self.latest_scan = None
        
        # 发送扫描请求
        self.robot.send_command('A\n')
        
        # 等待扫描结果
        start_time = time.time()
        while self.latest_scan is None:
            if time.time() - start_time > timeout:
                return None
            time.sleep(0.05)  # 50ms轮询间隔
        
        return self.latest_scan
    
    def get_distance_at_angle(self, target_angle, tolerance=15):
        """
        获取指定角度的距离
        
        Args:
            target_angle: 目标角度（度，0=正前方，90=左侧，270=右侧）
            tolerance: 角度容差（度），默认15度（扇区）或5度（点云）
        
        Returns:
            float: 距离（米），如果无数据返回999.0
        """
        if not self.latest_scan:
            return 999.0
        
        # 🆕 自动调整容差（点云模式更精确）
        if self.format_type == 'pointcloud':
            tolerance = 5  # 点云模式：1°分辨率，使用更小容差
        
        # 查找角度范围内的所有点
        distances = []
        for p in self.latest_scan:
            angle_diff = abs(p['angle'] - target_angle)
            # 处理角度环绕（例如350度和10度的差应该是20度）
            if angle_diff > 180:
                angle_diff = 360 - angle_diff
            
            if angle_diff <= tolerance:
                distances.append(p['distance'])
        
        # 返回最小距离（保守策略）
        return min(distances) if distances else 999.0
    
    def get_distance_in_range(self, angle_min, angle_max):
        """
        获取角度范围内的最小距离
        
        Args:
            angle_min: 最小角度（度）
            angle_max: 最大角度（度）
        
        Returns:
            float: 最小距离（米），如果无数据返回999.0
        """
        if not self.latest_scan:
            return 999.0
        
        distances = []
        for p in self.latest_scan:
            angle = p['angle']
            # 处理角度环绕
            if angle_min <= angle_max:
                if angle_min <= angle <= angle_max:
                    distances.append(p['distance'])
            else:
                # 例如：330-30度范围
                if angle >= angle_min or angle <= angle_max:
                    distances.append(p['distance'])
        
        return min(distances) if distances else 999.0
    
    def get_obstacles_summary(self):
        """
        获取障碍物概览
        
        Returns:
            dict: {
                'front': 前方最小距离,
                'left': 左侧最小距离,
                'right': 右侧最小距离,
                'back': 后方最小距离
            }
        """
        return {
            'front': self.get_distance_in_range(345, 15),    # -15° ~ +15°
            'left': self.get_distance_in_range(75, 105),     # 75° ~ 105°
            'right': self.get_distance_in_range(255, 285),   # 255° ~ 285°
            'back': self.get_distance_in_range(165, 195)     # 165° ~ 195°
        }
    
    def visualize_scan(self, width=60):
        """
        在终端可视化雷达扫描（简易版）
        
        Args:
            width: 显示宽度（字符数）
        """
        if not self.latest_scan:
            print("  [无雷达数据]")
            return
        
        # 按角度分8个扇区显示
        sectors = [[] for _ in range(8)]
        
        for p in self.latest_scan:
            sector_id = int(p['angle'] / 45) % 8
            sectors[sector_id].append(p['distance'])
        
        # 计算每个扇区的最小距离
        sector_dists = []
        for s in sectors:
            sector_dists.append(min(s) if s else 999.0)
        
        # ASCII可视化
        print("  雷达扫描 (8扇区):")
        print("         前方 (0°)")
        print("           ↑")
        print(f"  左 ({sector_dists[2]:.2f}m) ← + → ({sector_dists[6]:.2f}m) 右")
        print("           ↓")
        print(f"        后方 ({sector_dists[4]:.2f}m)")
        print()
        print(f"  详细: ", end="")
        for i, d in enumerate(sector_dists):
            angle = i * 45
            print(f"{angle}°:{d:.2f}m  ", end="")
        print()
    
    def get_latest_scan(self):
        """
        获取最新的扫描数据
        
        Returns:
            list: 扫描点集，或None
        """
        return self.latest_scan
    
    def has_scan(self):
        """
        检查是否有扫描数据
        
        Returns:
            bool: True如果有数据
        """
        return self.latest_scan is not None
    
    def get_scan_count(self):
        """
        获取扫描次数
        
        Returns:
            int: 扫描次数
        """
        return self.scan_count
    
    def get_point_cloud(self):
        """
        获取完整点云数据（用于SLAM建图）
        
        Returns:
            list: [(angle, distance), ...] 或 None
        
        Example:
            >>> cloud = lidar.get_point_cloud()
            >>> if cloud:
            ...     for angle, dist in cloud:
            ...         print(f"Angle: {angle}°, Distance: {dist}m")
        """
        if not self.latest_scan:
            return None
        
        return [(p['angle'], p['distance']) for p in self.latest_scan]
    
    def __repr__(self):
        """字符串表示"""
        if self.latest_scan:
            return f"SimpleLidarWrapper(format={self.format_type}, points={len(self.latest_scan)}, scans={self.scan_count})"
        else:
            return f"SimpleLidarWrapper(no_data, scans={self.scan_count})"


if __name__ == '__main__':
    # 单元测试（需要实际硬件）
    print("="*60)
    print("SimpleLidarWrapper 单元测试")
    print("="*60)
    print("\n⚠️ 此测试需要连接真实硬件")
    print("请确保小车已开机并且BLE已连接\n")
    
    choice = input("是否继续？(y/n): ").strip().lower()
    if choice != 'y':
        print("测试取消")
        exit()
    
    # 导入BLE通信模块
    import sys
    sys.path.insert(0, '..')
    from ble_robot_control import SimpleBLERobotComm
    
    BLE_ADDRESS = "C4:25:01:20:02:8E"
    
    robot = SimpleBLERobotComm(address=BLE_ADDRESS, verbose=False)
    
    try:
        print("\n🔵 连接BLE...")
        if not robot.connect():
            print("❌ 连接失败")
            exit()
        print("✅ BLE已连接\n")
        time.sleep(1)
        
        # 创建雷达包装器
        lidar = SimpleLidarWrapper(robot)
        
        # 测试1: 请求扫描
        print("\n测试1: 请求雷达扫描")
        print("-"*60)
        scan = lidar.request_scan(timeout=3.0)
        
        if scan:
            print(f"✅ 扫描成功，收到 {len(scan)} 个点")
            print(f"\n前5个点:")
            for i, p in enumerate(scan[:5]):
                print(f"  点{i+1}: 角度={p['angle']:3.0f}°, 距离={p['distance']:.2f}m")
        else:
            print("❌ 扫描失败（超时）")
        
        # 测试2: 查询特定角度距离
        print("\n\n测试2: 查询特定方向距离")
        print("-"*60)
        front = lidar.get_distance_at_angle(0)      # 正前方
        left = lidar.get_distance_at_angle(90)      # 左侧
        right = lidar.get_distance_at_angle(270)    # 右侧
        
        print(f"  正前方 (0°):   {front:.2f}m")
        print(f"  左侧 (90°):    {left:.2f}m")
        print(f"  右侧 (270°):   {right:.2f}m")
        
        # 测试3: 障碍物概览
        print("\n\n测试3: 障碍物概览")
        print("-"*60)
        obstacles = lidar.get_obstacles_summary()
        print(f"  前方: {obstacles['front']:.2f}m")
        print(f"  左侧: {obstacles['left']:.2f}m")
        print(f"  右侧: {obstacles['right']:.2f}m")
        print(f"  后方: {obstacles['back']:.2f}m")
        
        # 测试4: 可视化
        print("\n\n测试4: 终端可视化")
        print("-"*60)
        lidar.visualize_scan()
        
        print("\n" + "="*60)
        print("✅ 测试完成")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()

