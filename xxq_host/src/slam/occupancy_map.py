"""
占据栅格地图模块
实现SLAM中的2D占据栅格地图
"""

import numpy as np
from typing import Tuple, List, Optional
from dataclasses import dataclass


@dataclass
class MapConfig:
    """地图配置参数"""
    width: int = 500  # 栅格数量
    height: int = 500
    resolution: float = 0.1  # 米/栅格
    origin_x: int = 250  # 原点位置（栅格坐标）
    origin_y: int = 250
    
    # 占据概率阈值
    free_threshold: float = 0.3  # <0.3为空闲
    occupied_threshold: float = 0.7  # >0.7为占用
    
    # 概率更新参数
    prob_occupied: float = 0.9  # 检测到障碍物的概率
    prob_free: float = 0.3  # 空闲区域的概率


class OccupancyGridMap:
    """占据栅格地图
    
    使用占据概率表示每个栅格的状态：
    - 0.0: 完全空闲
    - 0.5: 未知
    - 1.0: 完全占用
    
    Attributes:
        grid: 概率地图矩阵 (height x width)
        config: 地图配置
        robot_path: 机器人运动轨迹
    
    Example:
        >>> map_obj = OccupancyGridMap(width=500, height=500, resolution=0.1)
        >>> map_obj.update_with_lidar(lidar_data, robot_pose)
        >>> grid_array = map_obj.get_map_array()
    """
    
    def __init__(self, config: MapConfig = None):
        """初始化占据栅格地图
        
        Args:
            config: 地图配置，None则使用默认配置
        """
        self.config = config if config else MapConfig()
        
        # 初始化地图（0.5=未知）
        self.grid = np.ones(
            (self.config.height, self.config.width),
            dtype=np.float32
        ) * 0.5
        
        # Log-odds表示（提高数值稳定性）
        self.log_odds = np.zeros_like(self.grid)
        
        # 机器人轨迹
        self.robot_path = []
        
        # 统计信息
        self.update_count = 0
        self.total_obstacles = 0
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """世界坐标转栅格坐标
        
        Args:
            x, y: 世界坐标（米）
        
        Returns:
            (grid_x, grid_y): 栅格坐标
        """
        grid_x = int(x / self.config.resolution) + self.config.origin_x
        grid_y = int(y / self.config.resolution) + self.config.origin_y
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """栅格坐标转世界坐标
        
        Args:
            grid_x, grid_y: 栅格坐标
        
        Returns:
            (x, y): 世界坐标（米）
        """
        x = (grid_x - self.config.origin_x) * self.config.resolution
        y = (grid_y - self.config.origin_y) * self.config.resolution
        return x, y
    
    def is_valid_grid(self, grid_x: int, grid_y: int) -> bool:
        """检查栅格坐标是否有效
        
        Args:
            grid_x, grid_y: 栅格坐标
        
        Returns:
            是否在地图范围内
        """
        return (0 <= grid_x < self.config.width and 
                0 <= grid_y < self.config.height)
    
    def update_with_lidar(self, lidar_data, robot_pose: Tuple[float, float, float]):
        """使用雷达数据更新地图
        
        Args:
            lidar_data: LidarData对象（包含扇区信息）
            robot_pose: 机器人位姿 (x, y, theta)
        """
        robot_x, robot_y, robot_theta = robot_pose
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        
        if not self.is_valid_grid(robot_gx, robot_gy):
            return
        
        # 记录轨迹
        self.robot_path.append((robot_x, robot_y))
        
        # 处理每个扇区
        for sector in lidar_data.sectors:
            if sector['count'] == 0:
                continue
            
            # 扇区中心角度
            angle_deg = sector['angle_center']
            min_dist = sector['min_dist']
            
            # 忽略无效距离
            if min_dist <= 0.01 or min_dist > 10.0:
                continue
            
            # 计算障碍物点的世界坐标
            angle_rad = np.deg2rad(angle_deg) + robot_theta
            obs_x = robot_x + min_dist * np.cos(angle_rad)
            obs_y = robot_y + min_dist * np.sin(angle_rad)
            
            obs_gx, obs_gy = self.world_to_grid(obs_x, obs_y)
            
            if not self.is_valid_grid(obs_gx, obs_gy):
                continue
            
            # 射线追踪：标记途径格子为空闲
            ray_cells = self._ray_trace(robot_gx, robot_gy, obs_gx, obs_gy)
            
            # 更新空闲区域（不包括终点）
            for cell_x, cell_y in ray_cells[:-1]:
                self._update_cell(cell_x, cell_y, is_occupied=False)
            
            # 更新障碍物点
            self._update_cell(obs_gx, obs_gy, is_occupied=True)
        
        self.update_count += 1
    
    def _update_cell(self, grid_x: int, grid_y: int, is_occupied: bool):
        """更新单个栅格的占据概率（贝叶斯更新）
        
        Args:
            grid_x, grid_y: 栅格坐标
            is_occupied: 是否观察到占用
        """
        if not self.is_valid_grid(grid_x, grid_y):
            return
        
        # Log-odds更新（数值更稳定）
        if is_occupied:
            # 增加占据概率
            log_update = np.log(self.config.prob_occupied / (1 - self.config.prob_occupied))
        else:
            # 减少占据概率（增加空闲概率）
            log_update = np.log(self.config.prob_free / (1 - self.config.prob_free))
        
        self.log_odds[grid_y, grid_x] += log_update
        
        # 限制log-odds范围，避免数值溢出
        self.log_odds[grid_y, grid_x] = np.clip(self.log_odds[grid_y, grid_x], -5, 5)
        
        # 转换回概率
        self.grid[grid_y, grid_x] = 1.0 / (1.0 + np.exp(-self.log_odds[grid_y, grid_x]))
    
    def _ray_trace(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham射线追踪算法
        
        生成从(x0, y0)到(x1, y1)的所有栅格坐标
        
        Args:
            x0, y0: 起点栅格坐标
            x1, y1: 终点栅格坐标
        
        Returns:
            途径的栅格坐标列表
        """
        cells = []
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            cells.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            
            if e2 > -dy:
                err -= dy
                x += sx
            
            if e2 < dx:
                err += dx
                y += sy
        
        return cells
    
    def get_map_array(self) -> np.ndarray:
        """获取地图概率矩阵
        
        Returns:
            (height x width)的概率矩阵
        """
        return self.grid.copy()
    
    def get_binary_map(self, threshold: float = 0.7) -> np.ndarray:
        """获取二值化地图
        
        Args:
            threshold: 占用阈值
        
        Returns:
            二值地图（0=空闲/未知，1=占用）
        """
        binary = np.zeros_like(self.grid, dtype=np.uint8)
        binary[self.grid > threshold] = 1
        return binary
    
    def get_free_space_map(self) -> np.ndarray:
        """获取空闲空间地图（用于路径规划）
        
        Returns:
            二值地图（0=占用/未知，1=空闲）
        """
        free_map = np.zeros_like(self.grid, dtype=np.uint8)
        free_map[self.grid < self.config.free_threshold] = 1
        return free_map
    
    def is_occupied(self, x: float, y: float) -> bool:
        """检查世界坐标是否被占用
        
        Args:
            x, y: 世界坐标（米）
        
        Returns:
            是否占用
        """
        gx, gy = self.world_to_grid(x, y)
        if not self.is_valid_grid(gx, gy):
            return True  # 边界外视为占用
        
        return self.grid[gy, gx] > self.config.occupied_threshold
    
    def is_free(self, x: float, y: float) -> bool:
        """检查世界坐标是否空闲
        
        Args:
            x, y: 世界坐标（米）
        
        Returns:
            是否空闲
        """
        gx, gy = self.world_to_grid(x, y)
        if not self.is_valid_grid(gx, gy):
            return False
        
        return self.grid[gy, gx] < self.config.free_threshold
    
    def get_statistics(self) -> dict:
        """获取地图统计信息
        
        Returns:
            统计信息字典
        """
        occupied_cells = np.sum(self.grid > self.config.occupied_threshold)
        free_cells = np.sum(self.grid < self.config.free_threshold)
        unknown_cells = np.sum(
            (self.grid >= self.config.free_threshold) & 
            (self.grid <= self.config.occupied_threshold)
        )
        
        total_cells = self.config.width * self.config.height
        
        return {
            'total_cells': total_cells,
            'occupied_cells': int(occupied_cells),
            'free_cells': int(free_cells),
            'unknown_cells': int(unknown_cells),
            'explored_ratio': (free_cells + occupied_cells) / total_cells,
            'update_count': self.update_count,
            'path_length': len(self.robot_path)
        }
    
    def save_map(self, filename: str):
        """保存地图到文件
        
        Args:
            filename: 文件路径（支持.npy或.png）
        """
        from pathlib import Path
        
        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        if path.suffix == '.npy':
            # 保存numpy数组
            np.save(path, self.grid)
        elif path.suffix == '.png':
            # 保存为图片
            import matplotlib.pyplot as plt
            plt.imsave(path, self.grid, cmap='gray_r')
        else:
            # 默认保存为numpy
            np.save(path.with_suffix('.npy'), self.grid)
        
        print(f"[地图] 已保存: {path}")
    
    def load_map(self, filename: str):
        """从文件加载地图
        
        Args:
            filename: 文件路径
        """
        self.grid = np.load(filename)
        
        # 重新计算log-odds
        # p = 1 / (1 + exp(-log_odds))
        # log_odds = log(p / (1-p))
        self.log_odds = np.log(self.grid / (1 - self.grid + 1e-10))
        
        print(f"[地图] 已加载: {filename}")

