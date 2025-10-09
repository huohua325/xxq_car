"""
路径规划模块
实现A*全局路径规划、路径平滑和障碍物膨胀
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
import heapq
from scipy.ndimage import binary_dilation


@dataclass
class PathPlannerConfig:
    """路径规划配置"""
    obstacle_threshold: float = 0.7  # 障碍物阈值
    inflation_radius: int = 2  # 障碍物膨胀半径（格子数）
    allow_diagonal: bool = True  # 是否允许斜向移动
    diagonal_cost: float = 1.414  # 斜向移动代价（√2）
    smoothing_max_angle: float = 30.0  # 路径平滑最大角度（度）
    smoothing_tolerance: float = 0.5  # Douglas-Peucker容差（米）


class PathPlanner:
    """A*路径规划器
    
    实现全局路径规划功能：
    1. A*搜索算法
    2. 障碍物膨胀（增加安全裕度）
    3. 路径平滑（Douglas-Peucker算法）
    4. 启发式函数（欧氏距离/曼哈顿距离）
    
    Attributes:
        map_obj: OccupancyGridMap对象
        config: PathPlannerConfig配置对象
        
    Example:
        >>> planner = PathPlanner(map_obj)
        >>> path = planner.plan_path((0, 0), (5, 5))
        >>> smoothed_path = planner.smooth_path(path)
    """
    
    def __init__(self, map_obj, config: PathPlannerConfig = None):
        """初始化路径规划器
        
        Args:
            map_obj: OccupancyGridMap对象
            config: 路径规划配置，None则使用默认配置
        """
        self.map = map_obj
        self.config = config if config else PathPlannerConfig()
        
        # 8邻域偏移（包含斜向）
        self.neighbors_8 = [
            (-1, -1, self.config.diagonal_cost),  # 左上
            (-1,  0, 1.0),                         # 左
            (-1,  1, self.config.diagonal_cost),  # 左下
            ( 0, -1, 1.0),                         # 下
            ( 0,  1, 1.0),                         # 上
            ( 1, -1, self.config.diagonal_cost),  # 右下
            ( 1,  0, 1.0),                         # 右
            ( 1,  1, self.config.diagonal_cost),  # 右上
        ]
        
        # 4邻域偏移（不含斜向）
        self.neighbors_4 = [
            (-1,  0, 1.0),  # 左
            ( 0, -1, 1.0),  # 下
            ( 0,  1, 1.0),  # 上
            ( 1,  0, 1.0),  # 右
        ]
        
        print(f"[PathPlanner] 路径规划器初始化完成 "
              f"(inflation={config.inflation_radius if config else 2}, "
              f"diagonal={config.allow_diagonal if config else True})")
    
    def plan_path(self, 
                  start: Tuple[float, float], 
                  goal: Tuple[float, float],
                  inflate_obstacles: bool = True,
                  smooth: bool = True) -> Optional[List[Tuple[float, float]]]:
        """规划从起点到终点的路径
        
        Args:
            start: 起点坐标 (x, y)，世界坐标，单位：米
            goal: 终点坐标 (x, y)，世界坐标，单位：米
            inflate_obstacles: 是否膨胀障碍物
            smooth: 是否平滑路径
            
        Returns:
            路径点列表 [(x1, y1), (x2, y2), ...]，世界坐标
            如果路径不存在，返回None
        """
        # 转换为栅格坐标
        start_grid = self.map.world_to_grid(*start)
        goal_grid = self.map.world_to_grid(*goal)
        
        # 检查起点和终点是否有效
        if not self.map.is_valid_grid(*start_grid):
            print(f"[PathPlanner] 起点无效: {start} -> {start_grid}")
            return None
        
        if not self.map.is_valid_grid(*goal_grid):
            print(f"[PathPlanner] 终点无效: {goal} -> {goal_grid}")
            return None
        
        # 准备障碍物地图
        obstacle_map = self._prepare_obstacle_map(inflate_obstacles)
        
        # 检查起点和终点是否被障碍物占据
        if obstacle_map[start_grid[1], start_grid[0]]:
            print(f"[PathPlanner] 起点被障碍物占据: {start}")
            return None
        
        if obstacle_map[goal_grid[1], goal_grid[0]]:
            print(f"[PathPlanner] 终点被障碍物占据: {goal}")
            return None
        
        # A*搜索
        path_grid = self._astar_search(start_grid, goal_grid, obstacle_map)
        
        if path_grid is None:
            print(f"[PathPlanner] 未找到路径: {start} -> {goal}")
            return None
        
        # 转换为世界坐标
        path_world = [self.map.grid_to_world(x, y) for x, y in path_grid]
        
        # 路径平滑
        if smooth and len(path_world) > 2:
            path_world = self.smooth_path(path_world)
        
        print(f"[PathPlanner] 找到路径: {len(path_world)} 个点")
        
        return path_world
    
    def _prepare_obstacle_map(self, inflate: bool) -> np.ndarray:
        """准备障碍物地图
        
        Args:
            inflate: 是否膨胀障碍物
            
        Returns:
            障碍物地图，True=障碍物，False=空闲
        """
        # 创建二值障碍物地图
        obstacle_map = self.map.grid >= self.config.obstacle_threshold
        
        # 膨胀障碍物
        if inflate and self.config.inflation_radius > 0:
            obstacle_map = binary_dilation(
                obstacle_map, 
                iterations=self.config.inflation_radius
            )
        
        return obstacle_map
    
    def _astar_search(self,
                     start: Tuple[int, int],
                     goal: Tuple[int, int],
                     obstacle_map: np.ndarray) -> Optional[List[Tuple[int, int]]]:
        """A*搜索算法
        
        Args:
            start: 起点栅格坐标 (x, y)
            goal: 终点栅格坐标 (x, y)
            obstacle_map: 障碍物地图
            
        Returns:
            路径点列表（栅格坐标），如果不存在则返回None
        """
        # 选择邻域
        neighbors = self.neighbors_8 if self.config.allow_diagonal else self.neighbors_4
        
        # 优先队列：(f_score, counter, node)
        # counter用于打破f_score相同的平局
        open_set = []
        counter = 0
        heapq.heappush(open_set, (0, counter, start))
        counter += 1
        
        # 来源字典：node -> parent_node
        came_from = {}
        
        # g_score: 从起点到当前节点的实际代价
        g_score = {start: 0}
        
        # f_score: g_score + h_score（启发式估计）
        f_score = {start: self._heuristic(start, goal)}
        
        # 已访问集合
        closed_set = set()
        
        while open_set:
            # 取出f_score最小的节点
            current_f, _, current = heapq.heappop(open_set)
            
            # 到达目标
            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            # 标记为已访问
            if current in closed_set:
                continue
            closed_set.add(current)
            
            # 遍历邻居
            for dx, dy, cost in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 检查是否在地图范围内
                if not self.map.is_valid_grid(*neighbor):
                    continue
                
                # 检查是否是障碍物
                if obstacle_map[neighbor[1], neighbor[0]]:
                    continue
                
                # 检查是否已访问
                if neighbor in closed_set:
                    continue
                
                # 计算g_score
                tentative_g_score = g_score[current] + cost
                
                # 如果找到更好的路径
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal)
                    
                    # 添加到open_set
                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                    counter += 1
        
        # 未找到路径
        return None
    
    def _heuristic(self, node: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """启发式函数（欧氏距离）
        
        Args:
            node: 当前节点
            goal: 目标节点
            
        Returns:
            估计代价
        """
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])
        
        if self.config.allow_diagonal:
            # 对角距离（Diagonal distance）
            # 允许斜向移动时的更准确估计
            return max(dx, dy) + (self.config.diagonal_cost - 1) * min(dx, dy)
        else:
            # 曼哈顿距离
            return dx + dy
    
    def _reconstruct_path(self, 
                         came_from: dict, 
                         current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """重建路径
        
        Args:
            came_from: 父节点字典
            current: 当前节点（目标节点）
            
        Returns:
            路径点列表
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """路径平滑（Douglas-Peucker算法）
        
        简化路径，移除不必要的中间点
        
        Args:
            path: 原始路径点列表
            
        Returns:
            平滑后的路径点列表
        """
        if len(path) <= 2:
            return path
        
        # Douglas-Peucker算法
        smoothed = self._douglas_peucker(path, self.config.smoothing_tolerance)
        
        # 确保起点和终点
        if smoothed[0] != path[0]:
            smoothed.insert(0, path[0])
        if smoothed[-1] != path[-1]:
            smoothed.append(path[-1])
        
        return smoothed
    
    def _douglas_peucker(self, 
                        points: List[Tuple[float, float]], 
                        tolerance: float) -> List[Tuple[float, float]]:
        """Douglas-Peucker算法实现
        
        Args:
            points: 点列表
            tolerance: 容差（米）
            
        Returns:
            简化后的点列表
        """
        if len(points) <= 2:
            return points
        
        # 找到距离起点-终点连线最远的点
        start = np.array(points[0])
        end = np.array(points[-1])
        
        max_dist = 0
        max_index = 0
        
        for i in range(1, len(points) - 1):
            point = np.array(points[i])
            dist = self._point_to_line_distance(point, start, end)
            if dist > max_dist:
                max_dist = dist
                max_index = i
        
        # 如果最大距离超过容差，递归分割
        if max_dist > tolerance:
            # 递归处理两段
            left = self._douglas_peucker(points[:max_index + 1], tolerance)
            right = self._douglas_peucker(points[max_index:], tolerance)
            
            # 合并（去除重复点）
            return left[:-1] + right
        else:
            # 否则只保留起点和终点
            return [points[0], points[-1]]
    
    def _point_to_line_distance(self, 
                                point: np.ndarray, 
                                line_start: np.ndarray, 
                                line_end: np.ndarray) -> float:
        """计算点到线段的距离
        
        Args:
            point: 点坐标
            line_start: 线段起点
            line_end: 线段终点
            
        Returns:
            距离
        """
        # 线段向量
        line_vec = line_end - line_start
        line_len_sq = np.dot(line_vec, line_vec)
        
        if line_len_sq == 0:
            # 起点和终点重合
            return np.linalg.norm(point - line_start)
        
        # 点到线段的投影参数 t
        t = np.dot(point - line_start, line_vec) / line_len_sq
        t = np.clip(t, 0, 1)
        
        # 投影点
        projection = line_start + t * line_vec
        
        # 距离
        return np.linalg.norm(point - projection)
    
    def inflate_obstacles(self, 
                         obstacle_map: np.ndarray, 
                         radius: int = None) -> np.ndarray:
        """膨胀障碍物
        
        增加安全裕度，防止机器人撞到障碍物
        
        Args:
            obstacle_map: 障碍物地图（二值）
            radius: 膨胀半径（格子数），None则使用配置
            
        Returns:
            膨胀后的障碍物地图
        """
        if radius is None:
            radius = self.config.inflation_radius
        
        if radius <= 0:
            return obstacle_map
        
        return binary_dilation(obstacle_map, iterations=radius)
    
    def get_path_length(self, path: List[Tuple[float, float]]) -> float:
        """计算路径长度
        
        Args:
            path: 路径点列表
            
        Returns:
            路径总长度（米）
        """
        if len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            length += np.hypot(dx, dy)
        
        return length
    
    def is_path_valid(self, path: List[Tuple[float, float]]) -> bool:
        """检查路径是否有效（不穿过障碍物）
        
        Args:
            path: 路径点列表
            
        Returns:
            是否有效
        """
        for point in path:
            # 转换为栅格坐标
            gx, gy = self.map.world_to_grid(*point)
            
            # 检查是否在地图范围内
            if not self.map.is_valid_grid(gx, gy):
                return False
            
            # 检查是否是障碍物
            if self.map.grid[gy, gx] >= self.config.obstacle_threshold:
                return False
        
        return True

