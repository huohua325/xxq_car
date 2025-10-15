"""
Frontier探索算法模块
实现前沿点检测、聚类和目标选择
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from collections import deque


class FrontierDetector:
    """Frontier探索算法
    
    用于检测地图中的前沿点（frontier points），即已探索区域与未探索区域的边界。
    前沿点是自主探索算法的目标点。
    
    前沿点定义：
    - 当前格子是空闲的（占据概率 < free_threshold）
    - 8邻域中至少有一个未知格子（占据概率在 free_threshold 和 occupied_threshold 之间）
    
    Attributes:
        map_obj: OccupancyGridMap对象
        min_frontier_size: 最小前沿簇大小
        cluster_distance: 聚类距离阈值（米）
        
    Example:
        >>> detector = FrontierDetector(map_obj)
        >>> frontiers = detector.find_frontiers()
        >>> target = detector.select_best_frontier(frontiers, robot_pose)
    """
    
    def __init__(self, map_obj, min_frontier_size: int = 5, cluster_distance: float = 0.3):
        """初始化Frontier检测器
        
        Args:
            map_obj: OccupancyGridMap对象
            min_frontier_size: 最小前沿簇大小（格子数）
            cluster_distance: 聚类距离阈值（米）
        """
        self.map = map_obj
        self.min_frontier_size = min_frontier_size
        self.cluster_distance = cluster_distance
        
        # 8邻域偏移
        self.neighbors = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]
        
        print(f"[Frontier] 检测器初始化完成 (min_size={min_frontier_size}, cluster_dist={cluster_distance}m)")
    
    def find_frontiers(self) -> List[Tuple[float, float]]:
        """查找所有前沿点
        
        Returns:
            前沿簇中心点列表 [(x1, y1), (x2, y2), ...]，世界坐标
        """
        # 步骤1: 找到所有原始前沿点（栅格坐标）
        raw_frontiers = []
        
        for y in range(1, self.map.config.height - 1):
            for x in range(1, self.map.config.width - 1):
                if self._is_frontier(x, y):
                    raw_frontiers.append((x, y))
        
        if not raw_frontiers:
            return []
        
        # 步骤2: 转换为世界坐标
        world_frontiers = [
            self.map.grid_to_world(x, y) for x, y in raw_frontiers
        ]
        
        # 步骤3: 聚类
        clusters = self._cluster_frontiers(world_frontiers)
        
        # 步骤4: 过滤小簇，计算簇中心
        frontier_centers = []
        for cluster in clusters:
            if len(cluster) >= self.min_frontier_size:
                # 计算簇中心
                center_x = np.mean([p[0] for p in cluster])
                center_y = np.mean([p[1] for p in cluster])
                frontier_centers.append((center_x, center_y))
        
        print(f"[Frontier] 检测到 {len(raw_frontiers)} 个原始前沿点, "
              f"{len(clusters)} 个簇, {len(frontier_centers)} 个有效前沿")
        
        return frontier_centers
    
    def _is_frontier(self, x: int, y: int) -> bool:
        """判断栅格是否为前沿点
        
        Args:
            x, y: 栅格坐标
            
        Returns:
            是否为前沿点
        """
        # 检查当前格子是否为空闲
        if self.map.grid[y, x] >= self.map.config.free_threshold:
            return False
        
        # 检查8邻域是否有未知区域
        has_unknown = False
        for dx, dy in self.neighbors:
            nx, ny = x + dx, y + dy
            
            # 检查邻域是否在地图范围内
            if not self.map.is_valid_grid(nx, ny):
                continue
            
            # 检查邻域是否为未知区域
            prob = self.map.grid[ny, nx]
            if (prob >= self.map.config.free_threshold and 
                prob <= self.map.config.occupied_threshold):
                has_unknown = True
                break
        
        return has_unknown
    
    def _cluster_frontiers(self, frontiers: List[Tuple[float, float]]) -> List[List[Tuple[float, float]]]:
        """对前沿点进行聚类
        
        使用简单的距离聚类算法（类似DBSCAN）
        
        Args:
            frontiers: 前沿点列表（世界坐标）
            
        Returns:
            簇列表，每个簇是前沿点的列表
        """
        if not frontiers:
            return []
        
        clusters = []
        used = set()
        
        for i, point in enumerate(frontiers):
            if i in used:
                continue
            
            # 创建新簇，使用BFS扩展
            cluster = [point]
            queue = deque([i])
            used.add(i)
            
            while queue:
                current_idx = queue.popleft()
                current_point = frontiers[current_idx]
                
                # 查找邻近点
                for j, other_point in enumerate(frontiers):
                    if j in used:
                        continue
                    
                    # 计算距离
                    dist = np.hypot(
                        other_point[0] - current_point[0],
                        other_point[1] - current_point[1]
                    )
                    
                    # 如果在聚类距离内，加入簇
                    if dist <= self.cluster_distance:
                        cluster.append(other_point)
                        queue.append(j)
                        used.add(j)
            
            clusters.append(cluster)
        
        return clusters
    
    def select_best_frontier(self, 
                            frontiers: List[Tuple[float, float]], 
                            robot_pose: Tuple[float, float, float],
                            strategy: str = 'nearest') -> Optional[Tuple[float, float]]:
        """选择最佳前沿点作为目标
        
        Args:
            frontiers: 前沿点列表
            robot_pose: 机器人位姿 (x, y, theta)
            strategy: 选择策略
                - 'nearest': 选择最近的前沿点
                - 'largest': 选择最大簇（需要传入簇信息）
                - 'information_gain': 基于信息增益选择（未实现）
                
        Returns:
            最佳前沿点坐标，如果没有前沿点则返回None
        """
        if not frontiers:
            return None
        
        robot_x, robot_y = robot_pose[0], robot_pose[1]
        
        if strategy == 'nearest':
            return self._select_nearest(frontiers, robot_x, robot_y)
        elif strategy == 'largest':
            # 暂时使用nearest策略，largest需要额外的簇大小信息
            return self._select_nearest(frontiers, robot_x, robot_y)
        elif strategy == 'information_gain':
            # 未实现，使用nearest
            return self._select_nearest(frontiers, robot_x, robot_y)
        else:
            # 默认使用nearest
            return self._select_nearest(frontiers, robot_x, robot_y)
    
    def _select_nearest(self, 
                       frontiers: List[Tuple[float, float]], 
                       robot_x: float, 
                       robot_y: float) -> Tuple[float, float]:
        """选择最近的前沿点
        
        Args:
            frontiers: 前沿点列表
            robot_x, robot_y: 机器人位置
            
        Returns:
            最近的前沿点
        """
        min_dist = float('inf')
        best_frontier = frontiers[0]
        
        for frontier in frontiers:
            dist = np.hypot(frontier[0] - robot_x, frontier[1] - robot_y)
            if dist < min_dist:
                min_dist = dist
                best_frontier = frontier
        
        print(f"[Frontier] 选择最近前沿点: ({best_frontier[0]:.2f}, {best_frontier[1]:.2f}), "
              f"距离: {min_dist:.2f}m")
        
        return best_frontier
    
    def select_best_frontier_goal_directed(self,
                                          frontiers: List[Tuple[float, float]], 
                                          robot_pose: Tuple[float, float, float],
                                          goal_position: Tuple[float, float],
                                          alpha: float = 0.6) -> Optional[Tuple[float, float]]:
        """
        选择朝向目标方向的最佳前沿点（考试专用）
        
        Args:
            frontiers: 前沿点列表 [(x1, y1), (x2, y2), ...]
            robot_pose: 机器人位姿 (x, y, theta)
            goal_position: 目标位置 (x, y)，例如Exit坐标
            alpha: 距离权重 (0.6=稍微偏向距离，0.4偏向目标方向)
            
        Returns:
            最佳前沿点坐标 (x, y) 或 None
        """
        if not frontiers:
            return None
        
        robot_x, robot_y = robot_pose[0], robot_pose[1]
        goal_x, goal_y = goal_position
        
        # 机器人到目标的方向向量
        goal_vec = np.array([goal_x - robot_x, goal_y - robot_y])
        goal_dist = np.linalg.norm(goal_vec)
        
        # 如果已经很接近目标，选择最近的frontier
        if goal_dist < 0.35:  # 半个格子
            print("[Frontier] 接近目标，选择最近frontier")
            return self._select_nearest(frontiers, robot_x, robot_y)
        
        goal_vec = goal_vec / goal_dist  # 单位向量
        
        best_score = -float('inf')
        best_frontier = None
        
        for fx, fy in frontiers:
            # 到frontier的距离
            dist_to_frontier = np.hypot(fx - robot_x, fy - robot_y)
            
            # 到frontier的方向向量
            frontier_vec = np.array([fx - robot_x, fy - robot_y])
            frontier_vec = frontier_vec / (np.linalg.norm(frontier_vec) + 1e-6)
            
            # 与目标方向的余弦相似度 (-1到1)
            direction_similarity = np.dot(frontier_vec, goal_vec)
            
            # 综合评分
            # 距离评分：1/(1+dist)，越近越好
            distance_score = 1.0 / (1.0 + dist_to_frontier)
            
            # 最终评分
            score = alpha * distance_score + (1 - alpha) * direction_similarity
            
            if score > best_score:
                best_score = score
                best_frontier = (fx, fy)
        
        print(f"[Frontier] 目标导向: frontier={best_frontier}, 评分={best_score:.2f}")
        return best_frontier
    
    def find_frontiers_with_info(self) -> List[Dict]:
        """查找前沿点并返回详细信息
        
        Returns:
            前沿簇信息列表，每个元素包含：
            - center: 簇中心点 (x, y)
            - size: 簇大小（点数）
            - points: 簇内所有点
        """
        # 步骤1: 找到所有原始前沿点（栅格坐标）
        raw_frontiers = []
        
        for y in range(1, self.map.config.height - 1):
            for x in range(1, self.map.config.width - 1):
                if self._is_frontier(x, y):
                    raw_frontiers.append((x, y))
        
        if not raw_frontiers:
            return []
        
        # 步骤2: 转换为世界坐标
        world_frontiers = [
            self.map.grid_to_world(x, y) for x, y in raw_frontiers
        ]
        
        # 步骤3: 聚类
        clusters = self._cluster_frontiers(world_frontiers)
        
        # 步骤4: 构建详细信息
        frontier_info = []
        for cluster in clusters:
            if len(cluster) >= self.min_frontier_size:
                center_x = np.mean([p[0] for p in cluster])
                center_y = np.mean([p[1] for p in cluster])
                
                frontier_info.append({
                    'center': (center_x, center_y),
                    'size': len(cluster),
                    'points': cluster
                })
        
        # 按簇大小降序排序
        frontier_info.sort(key=lambda x: x['size'], reverse=True)
        
        return frontier_info
    
    def select_best_frontier_advanced(self,
                                     frontiers_info: List[Dict],
                                     robot_pose: Tuple[float, float, float],
                                     strategy: str = 'nearest') -> Optional[Tuple[float, float]]:
        """基于详细信息选择最佳前沿点
        
        Args:
            frontiers_info: 前沿簇详细信息列表
            robot_pose: 机器人位姿
            strategy: 选择策略
                - 'nearest': 最近的
                - 'largest': 最大的簇
                - 'balanced': 综合考虑距离和大小
                
        Returns:
            最佳前沿点
        """
        if not frontiers_info:
            return None
        
        robot_x, robot_y = robot_pose[0], robot_pose[1]
        
        if strategy == 'nearest':
            # 选择最近的簇中心
            min_dist = float('inf')
            best_frontier = None
            
            for info in frontiers_info:
                center = info['center']
                dist = np.hypot(center[0] - robot_x, center[1] - robot_y)
                if dist < min_dist:
                    min_dist = dist
                    best_frontier = center
            
            return best_frontier
        
        elif strategy == 'largest':
            # 选择最大的簇
            max_size = -1
            best_frontier = None
            
            for info in frontiers_info:
                if info['size'] > max_size:
                    max_size = info['size']
                    best_frontier = info['center']
            
            return best_frontier
        
        elif strategy == 'balanced':
            # 综合考虑距离和大小
            best_score = -float('inf')
            best_frontier = None
            
            for info in frontiers_info:
                center = info['center']
                size = info['size']
                
                dist = np.hypot(center[0] - robot_x, center[1] - robot_y)
                
                # 评分：大小越大越好，距离越近越好
                # score = size / (dist + 1.0)  # 避免除零
                score = np.log(size + 1) * 10 - dist  # 对数权重
                
                if score > best_score:
                    best_score = score
                    best_frontier = center
            
            return best_frontier
        
        else:
            # 默认最近
            return frontiers_info[0]['center'] if frontiers_info else None
    
    def calculate_information_gain(self, 
                                  frontier: Tuple[float, float],
                                  robot_pose: Tuple[float, float, float]) -> float:
        """计算探索某个前沿点的信息增益
        
        估算探索该前沿点可以获得的新信息量
        
        Args:
            frontier: 前沿点坐标
            robot_pose: 机器人位姿
            
        Returns:
            信息增益值（越大越好）
        """
        # 简化实现：基于前沿点周围的未知区域面积
        fx, fy = frontier
        gx, gy = self.map.world_to_grid(fx, fy)
        
        # 计算半径内的未知格子数
        search_radius = 10  # 栅格单位
        unknown_count = 0
        
        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                nx, ny = gx + dx, gy + dy
                
                if not self.map.is_valid_grid(nx, ny):
                    continue
                
                prob = self.map.grid[ny, nx]
                if (prob >= self.map.config.free_threshold and 
                    prob <= self.map.config.occupied_threshold):
                    unknown_count += 1
        
        # 距离成本
        dist = np.hypot(frontier[0] - robot_pose[0], frontier[1] - robot_pose[1])
        
        # 信息增益 = 未知区域面积 - 距离成本
        gain = unknown_count - dist * 2  # 距离权重可调
        
        return gain

