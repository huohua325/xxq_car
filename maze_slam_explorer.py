import json
import tkinter as tk
import numpy as np
import math
import time
import random
from collections import deque
import heapq

class MazeSLAMExplorer:
    def __init__(self, json_file):
        self.root = tk.Tk()
        self.root.title("迷宫SLAM地图构建与探索")
        
        # 移动速度设置
        self.speed_factor = 2.5  # 速度因子，值越大移动越快
        
        # 读取JSON文件 - 注意这些数据只用于后台模拟，机器人并不知道完整地图
        with open(json_file, 'r') as f:
            data = json.load(f)
        
        self.segments = data['segments']
        self.start_point = tuple(data['start_point'])
        
        # 计算地图尺寸
        max_row, max_col = 0, 0
        min_row, min_col = float('inf'), float('inf')
        for segment in self.segments:
            start_row, start_col = segment['start']
            end_row, end_col = segment['end']
            max_row = max(max_row, start_row, end_row)
            max_col = max(max_col, start_col, end_col)
            min_row = min(min_row, start_row, end_row)
            min_col = min(min_col, start_col, end_col)
        self.max_row = max_row
        self.max_col = max_col
        self.min_row = min_row
        self.min_col = min_col
        # 设置有效地图边界为json边界
        self.valid_map_bounds = (min_row, max_row, min_col, max_col)
        
        # 设置单元格大小和边距
        self.cell_size = 30
        self.margin = 20
        
        # 创建画布
        canvas_width = (max_col + 1) * self.cell_size + 2 * self.margin
        canvas_height = (max_row + 1) * self.cell_size + 2 * self.margin
        self.canvas = tk.Canvas(self.root, width=canvas_width, height=canvas_height, bg="lightgray")
        self.canvas.pack(padx=10, pady=10)
        
        # 机器人参数
        self.robot_radius = 0.3
        self.robot_position = self.start_point
        self.robot_heading = 0.0  # 朝向（弧度）
        self.robot_trajectory = [self.robot_position]
        
        # 雷达参数
        self.lidar_range = 5.5 # 雷达范围（米）- 增加雷达范围
        self.lidar_angles = np.linspace(0, 2*math.pi, 120)  # 雷达扫描角度（增加到72个方向以提高圆形雷达的分辨率）
        self.scan_lines = []  # 存储雷达扫描线的引用
        
        # 动态地图构建参数
        self.grid_size = 0.3  # 栅格大小
        self.grid_rows = int(self.max_row / self.grid_size) + 20
        self.grid_cols = int(self.max_col / self.grid_size) + 20
        
        # 地图状态：-1=未知，0=空闲，1=障碍物，2=出口/终点
        self.map_grid = np.full((self.grid_rows, self.grid_cols), -1)
        
        # 墙面线段，用于模拟雷达扫描
        self.wall_segments = []
        for segment in self.segments:
            self.wall_segments.append((
                (segment['start'][0], segment['start'][1]),
                (segment['end'][0], segment['end'][1])
            ))
        
        # 探索相关参数
        self.is_exploring = False
        self.robot_marker = None
        self.frontier_points = []  # 前沿点（探索边界）
        self.current_target = None  # 当前探索目标
        self.exploration_path = []  # 当前探索路径
        self.path_markers = []  # 存储路径标记的引用
        self.exploration_stuck_counter = 0  # 卡住计数器
        self.last_position = self.start_point  # 上一个位置
        self.last_heading = 0.0  # 上一个朝向
        self.position_history = deque(maxlen=20)  # 记录最近的位置历史，用于检测往复运动
        self.local_stuck_regions = set()  # 记录局部卡住区域
        self.exit_points = []  # 可能的出口点
        self.has_found_exit = False  # 是否已找到出口
        self.returning_home = False  # 是否正在返回起点
        self.exploration_complete = False  # 标记是否完成了地图探索
        self.wall_coverage_threshold = 0.985  # 墙壁探索完成的阈值(99.3%)
        
        # 控件
        self.control_frame = tk.Frame(self.root)
        self.control_frame.pack(pady=10)
        
        self.start_button = tk.Button(self.control_frame, text="开始SLAM", command=self.start_exploration)
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        self.reset_button = tk.Button(self.control_frame, text="重置", command=self.reset)
        self.reset_button.pack(side=tk.LEFT, padx=5)
        
        self.quit_button = tk.Button(self.control_frame, text="退出", command=self.root.quit)
        self.quit_button.pack(side=tk.LEFT, padx=5)
        
        # 状态标签
        self.status_label = tk.Label(self.root, text="准备开始SLAM探索")
        self.status_label.pack(pady=5)
        
        # 机器人状态标签
        self.robot_status_label = tk.Label(self.root, text="状态: 待命中", fg="blue")
        self.robot_status_label.pack(pady=2)
        
        # 路径点标签
        self.path_label = tk.Label(self.root, text="")
        self.path_label.pack(pady=5)
        
        # 添加速度控制滑块
        self.speed_frame = tk.Frame(self.root)
        self.speed_frame.pack(pady=5)
        
        speed_label = tk.Label(self.speed_frame, text="速度控制:")
        speed_label.pack(side=tk.LEFT, padx=5)
        
        self.speed_slider = tk.Scale(self.speed_frame, from_=1.0, to=5.0, 
                                     orient=tk.HORIZONTAL, resolution=0.5,
                                     length=200, command=self.update_speed)
        self.speed_slider.set(self.speed_factor)
        self.speed_slider.pack(side=tk.LEFT)
        
        self.speed_value_label = tk.Label(self.speed_frame, text=f"速度: {self.speed_factor}x")
        self.speed_value_label.pack(side=tk.LEFT, padx=5)
        
        # 初始化显示
        self.init_display()
        
    def init_display(self):
        """初始化显示，绘制网格背景"""
        try:
            # 绘制网格背景（全部为未知区域）
            for r in range(self.grid_rows):
                for c in range(self.grid_cols):
                    x1 = c * self.grid_size * self.cell_size + self.margin
                    y1 = r * self.grid_size * self.cell_size + self.margin
                    x2 = x1 + self.grid_size * self.cell_size
                    y2 = y1 + self.grid_size * self.cell_size
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill="lightgray", outline="gray")
            
            # 标记起点
            row, col = self.start_point
            x = col * self.cell_size + self.margin
            y = row * self.cell_size + self.margin
            self.canvas.create_oval(x-5, y-5, x+5, y+5, fill="green", outline="")
        except Exception as e:
            print(f"初始化显示错误: {e}")
    
    def distance_point_to_segment(self, p, s1, s2):
        """计算点到线段的最短距离"""
        try:
            # 线段向量
            v = (s2[0] - s1[0], s2[1] - s1[1])
            # 点到起点向量
            w = (p[0] - s1[0], p[1] - s1[1])
            
            # 计算投影长度比例
            c1 = sum(i*j for i, j in zip(w, v))
            if c1 <= 0:  # 投影点在线段外，距离即为到起点距离
                return math.sqrt(sum((i-j)**2 for i, j in zip(p, s1)))
                
            c2 = sum(i*i for i in v)
            if c2 <= c1:  # 投影点在线段外，距离即为到终点距离
                return math.sqrt(sum((i-j)**2 for i, j in zip(p, s2)))
                
            # 投影点在线段上，计算点到线的距离
            b = c1 / c2
            pb = (s1[0] + b * v[0], s1[1] + b * v[1])
            return math.sqrt(sum((i-j)**2 for i, j in zip(p, pb)))
        except Exception as e:
            print(f"计算点到线段距离错误: {e}")
            return float('inf')  # 返回无穷大表示无法计算
    
    def lidar_scan(self):
        """使用扇形雷达模拟扫描，返回不同方向上的障碍物距离，考虑墙壁遮挡"""
        try:
            x, y = self.robot_position
            scan_results = []
            
            # 清除之前的雷达线
            for line in self.scan_lines:
                self.canvas.delete(line)
            self.scan_lines = []
            
            # 雷达参数
            radar_range = self.lidar_range
            
            # 对每个角度进行射线投射，而不是检查所有墙体
            # 这更接近实际雷达的工作原理
            for angle in self.lidar_angles:
                # 计算雷达射线的绝对角度和终点
                absolute_angle = self.robot_heading + angle
                end_x = x + radar_range * math.cos(absolute_angle)
                end_y = y + radar_range * math.sin(absolute_angle)
                
                # 初始化这个角度的最小距离和交点
                min_dist = radar_range
                intersection_point = (end_x, end_y)
                
                # 检查这条射线是否与任何墙壁相交
                for wall in self.wall_segments:
                    # 计算射线与墙壁的交点
                    intersect = self.ray_segment_intersection(
                        (x, y), (end_x, end_y), wall[0], wall[1]
                    )
                    
                    if intersect:
                        # 计算交点距离
                        dist = math.sqrt((intersect[0] - x)**2 + (intersect[1] - y)**2)
                        # 更新最近的交点
                        if dist < min_dist:
                            min_dist = dist
                            intersection_point = intersect
                
                # 添加这个角度的扫描结果
                scan_results.append((angle, min_dist, intersection_point))
            
            return scan_results
        except Exception as e:
            print(f"雷达扫描错误: {e}")
            return []  # 返回空列表表示扫描失败
    
    def ray_segment_intersection(self, ray_origin, ray_end, segment_start, segment_end):
        """计算射线与线段的交点"""
        try:
            # 射线向量
            ray_direction = (ray_end[0] - ray_origin[0], ray_end[1] - ray_origin[1])
            
            # 线段向量
            segment_direction = (segment_end[0] - segment_start[0], segment_end[1] - segment_start[1])
            
            # 计算行列式
            denominator = ray_direction[0] * segment_direction[1] - ray_direction[1] * segment_direction[0]
            
            # 如果行列式为0，则线段与射线平行
            if abs(denominator) < 1e-10:
                return None
            
            # 计算参数
            t1 = ((segment_start[0] - ray_origin[0]) * segment_direction[1] - 
                (segment_start[1] - ray_origin[1]) * segment_direction[0]) / denominator
            
            t2 = ((segment_start[0] - ray_origin[0]) * ray_direction[1] - 
                (segment_start[1] - ray_origin[1]) * ray_direction[0]) / denominator
            
            # 检查参数是否在有效范围内
            if t1 >= 0 and 0 <= t2 <= 1:
                # 计算交点
                intersection_x = ray_origin[0] + t1 * ray_direction[0]
                intersection_y = ray_origin[1] + t1 * ray_direction[1]
                return (intersection_x, intersection_y)
            
            return None
        except Exception as e:
            print(f"射线相交计算错误: {e}")
            return None
    
    def update_map(self, scan_results):
        """根据雷达扫描结果更新地图，同时检测可能的出口"""
        try:
            if not scan_results:
                return
                
            x, y = self.robot_position
            
            # 将机器人位置转换为网格坐标
            grid_x = int(y / self.grid_size)
            grid_y = int(x / self.grid_size)
            
            # 将机器人周围的区域标记为已知空闲区域
            radius_grids = int(self.robot_radius / self.grid_size) + 1
            for r in range(-radius_grids, radius_grids + 1):
                for c in range(-radius_grids, radius_grids + 1):
                    if 0 <= grid_y + r < self.grid_rows and 0 <= grid_x + c < self.grid_cols:
                        if self.map_grid[grid_y + r, grid_x + c] == -1:  # 如果是未知区域
                            self.map_grid[grid_y + r, grid_x + c] = 0  # 标记为空闲
                            
                            # 更新地图显示
                            cell_x1 = (grid_x + c) * self.grid_size * self.cell_size + self.margin
                            cell_y1 = (grid_y + r) * self.grid_size * self.cell_size + self.margin
                            cell_x2 = cell_x1 + self.grid_size * self.cell_size
                            cell_y2 = cell_y1 + self.grid_size * self.cell_size
                            self.canvas.create_rectangle(cell_x1, cell_y1, cell_x2, cell_y2, fill="white", outline="gray")
            
            # 创建扫描角度到扫描结果的映射，便于后续处理
            angle_to_scan = {}
            for scan in scan_results:
                angle_to_scan[scan[0]] = scan
            
            # 处理每个雷达扫描结果
            for angle, distance, intersection in scan_results:
                if distance < self.lidar_range:  # 如果检测到障碍物
                    # 将障碍物位置转换为网格坐标
                    obs_x = int(intersection[1] / self.grid_size)
                    obs_y = int(intersection[0] / self.grid_size)
                    
                    if 0 <= obs_y < self.grid_rows and 0 <= obs_x < self.grid_cols:
                        if self.map_grid[obs_y, obs_x] != 1:  # 如果不是已知障碍物
                            self.map_grid[obs_y, obs_x] = 1  # 标记为障碍物
                            
                            # 更新地图显示
                            cell_x1 = obs_x * self.grid_size * self.cell_size + self.margin
                            cell_y1 = obs_y * self.grid_size * self.cell_size + self.margin
                            cell_x2 = cell_x1 + self.grid_size * self.cell_size
                            cell_y2 = cell_y1 + self.grid_size * self.cell_size
                            self.canvas.create_rectangle(cell_x1, cell_y1, cell_x2, cell_y2, fill="black", outline="gray")
                    
                    # 标记从机器人到障碍物之间的空闲区域，但不穿透墙壁
                    # 计算到障碍物的方向
                    dx = intersection[0] - x
                    dy = intersection[1] - y
                    
                    # 归一化方向向量
                    ray_length = math.sqrt(dx*dx + dy*dy)
                    if ray_length > 0:
                        dx /= ray_length
                        dy /= ray_length
                    
                    # 沿射线标记空闲区域，但只标记到障碍物前方
                    # 根据距离调整扫描密度，近处密集，远处稀疏
                    step_size = self.grid_size * 0.6  # 基础步长
                    current_dist = 0
                    
                    # 沿射线方向逐步扫描
                    while current_dist < distance * 0.95:  # 留出一点距离，避免标记到障碍物上
                        # 计算当前点的位置
                        free_x = x + current_dist * dx
                        free_y = y + current_dist * dy
                        
                        # 根据距离调整下一步的步长
                        distance_factor = current_dist / distance
                        # 远处的点步长逐渐增大，使扫描更高效
                        adaptive_step = step_size * (1.0 + distance_factor * 0.8)
                        
                        # 标记主射线点
                        self.mark_free_cell(free_x, free_y)
                        
                        # 增加当前距离，进入下一个点
                        current_dist += adaptive_step
                        
                        # 添加周围的点，形成扇形扫描效果，但需要检查这些点是否被墙壁遮挡
                        # 随距离增加扫描半径，但在障碍物附近减小
                        remaining_factor = 1.0 - (current_dist / distance)
                        spread_factor = min(0.8, current_dist / distance * 1.2) * remaining_factor
                        base_radius = 0.3 * (1 + spread_factor)  # 随距离先增加后减小的半径
                        
                        # 在圆周上分布点，但需检查每个点是否在视线内
                        for j in range(8):
                            angle_offset = j * math.pi / 4
                            offset_x = base_radius * math.cos(angle_offset)
                            offset_y = base_radius * math.sin(angle_offset)
                            
                            # 计算周围点
                            side_x = free_x + offset_x
                            side_y = free_y + offset_y
                            
                            # 检查从机器人到该点的路径是否被墙壁遮挡
                            if self.is_point_visible(x, y, side_x, side_y):
                                self.mark_free_cell(side_x, side_y)
                        
                        # 如果距离中等，再添加一个较大半径的圈，但仍需检查遮挡
                        if 0.3 < spread_factor < 0.6:
                            outer_radius = base_radius * 1.4
                            for j in range(8):
                                angle_offset = j * math.pi / 4 + math.pi / 8  # 错开角度
                                offset_x = outer_radius * math.cos(angle_offset)
                                offset_y = outer_radius * math.sin(angle_offset)
                                
                                # 计算外圈点
                                outer_x = free_x + offset_x
                                outer_y = free_y + offset_y
                                
                                # 检查从机器人到该点的路径是否被墙壁遮挡
                                if self.is_point_visible(x, y, outer_x, outer_y):
                                    self.mark_free_cell(outer_x, outer_y)
            
        except Exception as e:
            print(f"更新地图错误: {e}")
    
    def is_point_visible(self, from_x, from_y, to_x, to_y):
        """检查从起点到终点的视线是否被墙壁阻挡"""
        try:
            # 如果点距离太近，直接返回可见
            dist = math.sqrt((to_x - from_x)**2 + (to_y - from_y)**2)
            if dist < 0.1:
                return True
                
            # 检查线段是否与任何墙壁相交
            for wall in self.wall_segments:
                if self.line_segments_intersect((from_x, from_y), (to_x, to_y), wall[0], wall[1]):
                    return False
                    
            return True
        except Exception as e:
            print(f"检查点可见性错误: {e}")
            return False  # 出错时保守处理，认为不可见
            
    def mark_free_cell(self, x, y):
        """将指定位置标记为空闲区域并更新显示"""
        try:
            # 转换为网格坐标
            grid_x = int(y / self.grid_size)
            grid_y = int(x / self.grid_size)
            
            # 检查是否在网格范围内
            if 0 <= grid_y < self.grid_rows and 0 <= grid_x < self.grid_cols:
                if self.map_grid[grid_y, grid_x] == -1:  # 如果是未知区域
                    self.map_grid[grid_y, grid_x] = 0  # 标记为空闲
                    
                    # 更新地图显示
                    cell_x1 = grid_x * self.grid_size * self.cell_size + self.margin
                    cell_y1 = grid_y * self.grid_size * self.cell_size + self.margin
                    cell_x2 = cell_x1 + self.grid_size * self.cell_size
                    cell_y2 = cell_y1 + self.grid_size * self.cell_size
                    self.canvas.create_rectangle(cell_x1, cell_y1, cell_x2, cell_y2, fill="white", outline="gray")
        except Exception as e:
            print(f"标记空闲单元格错误: {e}")

    def distance_between_points(self, p1, p2):
        """计算两点之间的距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        
    def detect_frontiers(self):
        """检测地图中的前沿点（已知空闲区域与未知区域的边界）"""
        try:
            frontiers = []
            
            # 获取有效地图边界 - 基于已知的迷宫墙体
            min_valid_x = float('inf')
            max_valid_x = float('-inf')
            min_valid_y = float('inf')
            max_valid_y = float('-inf')
            
            # 使用墙体段落确定有效边界
            for wall in self.wall_segments:
                min_valid_x = min(min_valid_x, wall[0][0], wall[1][0])
                max_valid_x = max(max_valid_x, wall[0][0], wall[1][0])
                min_valid_y = min(min_valid_y, wall[0][1], wall[1][1])
                max_valid_y = max(max_valid_y, wall[0][1], wall[1][1])
            
            # 为边界添加更大的余量，以确保不会超出边界
            margin = 3.0  # 增加为3个单位的边界余量
            min_valid_x = max(0, min_valid_x - margin)
            min_valid_y = max(0, min_valid_y - margin)
            # 确保边界不会超过地图实际大小，同时留有足够余量
            max_valid_x = min(self.max_row - 1.0, max_valid_x + margin)
            max_valid_y = min(self.max_col - 1.0, max_valid_y + margin)
            
            # 记录有效边界
            if not hasattr(self, 'valid_map_bounds'):
                self.valid_map_bounds = (min_valid_x, max_valid_x, min_valid_y, max_valid_y)
                print(f"有效地图边界: X[{min_valid_x:.1f}-{max_valid_x:.1f}], Y[{min_valid_y:.1f}-{max_valid_y:.1f}]")
            
            # 扫描地图中的所有已知空闲区域
            for r in range(self.grid_rows):
                for c in range(self.grid_cols):
                    if self.map_grid[r, c] == 0:  # 已知空闲区域
                        
                        # 检查是否有相邻的未知区域 - 增加到8个方向
                        has_unknown = False
                        for dr, dc in [(-1, -1), (-1, 0), (-1, 1), (0, -1), 
                                      (0, 1), (1, -1), (1, 0), (1, 1)]:
                            nr, nc = r + dr, c + dc
                            if 0 <= nr < self.grid_rows and 0 <= nc < self.grid_cols:
                                if self.map_grid[nr, nc] == -1:  # 未知区域
                                    has_unknown = True
                                    break
                        
                        # 如果有相邻的未知区域，则可能是前沿点
                        if has_unknown:
                            # 转换为实际坐标
                            frontier_x = r * self.grid_size
                            frontier_y = c * self.grid_size
                            
                            # 检查是否在有效地图边界内
                            if not (min_valid_x <= frontier_x <= max_valid_x and 
                                    min_valid_y <= frontier_y <= max_valid_y):
                                continue
                            
                            # 检查是否在墙壁附近（降低安全距离要求，使检测更敏感）
                            safety_distance = self.robot_radius * 1.0  # 降低安全距离要求
                            too_close = False
                            
                            for wall in self.wall_segments:
                                distance = self.distance_point_to_segment((frontier_x, frontier_y), wall[0], wall[1])
                                if distance <= safety_distance:
                                    too_close = True
                                    break
                            
                            if not too_close:
                                # 检查是否是边缘尖端
                                if self.is_frontier_edge_tip(frontier_x, frontier_y):
                                    # 边缘尖端优先级更高
                                    frontiers.append((frontier_x, frontier_y))
                                else:
                                    # 普通前沿点
                                    frontiers.append((frontier_x, frontier_y))
            
            # 聚类和合并邻近的前沿点
            merged_frontiers = []
            visited_indices = set()
            
            for i, (frontier_x, frontier_y) in enumerate(frontiers):
                if i in visited_indices:
                    continue
                
                # 创建一个新簇
                cluster = [(frontier_x, frontier_y)]
                visited_indices.add(i)
                
                # 减小聚类距离，保留更多独立的前沿点
                cluster_distance = 1.0  # 降低聚类距离
                
                for j, (other_x, other_y) in enumerate(frontiers):
                    if j != i and j not in visited_indices:
                        if self.distance_between_points((frontier_x, frontier_y), (other_x, other_y)) < cluster_distance:
                            cluster.append((other_x, other_y))
                            visited_indices.add(j)
                
                # 计算簇的中心点
                if len(cluster) > 0:
                    center_x = sum(p[0] for p in cluster) / len(cluster)
                    center_y = sum(p[1] for p in cluster) / len(cluster)
                    
                    # 确保中心点在有效边界内
                    center_x = max(min_valid_x, min(max_valid_x, center_x))
                    center_y = max(min_valid_y, min(max_valid_y, center_y))
                    
                    # 再次检查中心点是否与墙体保持安全距离
                    safety_distance = self.robot_radius * 1.2  # 降低中心点的安全距离要求
                    center_too_close = False
                    
                    for wall in self.wall_segments:
                        if self.distance_point_to_segment((center_x, center_y), wall[0], wall[1]) <= safety_distance:
                            center_too_close = True
                            break
                    
                    if not center_too_close:
                        merged_frontiers.append((center_x, center_y))
                    else:
                        # 如果中心点不安全，尝试找到簇中距离墙体最远的点
                        safest_point = max(cluster, key=lambda p: min(
                            self.distance_point_to_segment(p, w[0], w[1]) for w in self.wall_segments
                        ))
                        if min(self.distance_point_to_segment(safest_point, w[0], w[1]) 
                              for w in self.wall_segments) > self.robot_radius:
                            merged_frontiers.append(safest_point)
            
            # 如果找到的前沿点太少，降低安全距离要求再次尝试
            if len(merged_frontiers) < 2:
                # 再次尝试，但使用更低的安全距离要求
                frontiers = []
                for r in range(self.grid_rows):
                    for c in range(self.grid_cols):
                        if self.map_grid[r, c] == 0:  # 已知空闲区域
                            # 检查是否有相邻的未知区域
                            has_unknown = False
                            for dr in range(-1, 2):
                                for dc in range(-1, 2):
                                    if dr == 0 and dc == 0:
                                        continue
                                    nr, nc = r + dr, c + dc
                                    if 0 <= nr < self.grid_rows and 0 <= nc < self.grid_cols:
                                        if self.map_grid[nr, nc] == -1:  # 未知区域
                                            has_unknown = True
                                            break
                                if has_unknown:
                                    break
                            
                            if has_unknown:
                                # 转换为实际坐标
                                frontier_x = r * self.grid_size
                                frontier_y = c * self.grid_size
                                
                                # 确保在有效边界内
                                if not (min_valid_x <= frontier_x <= max_valid_x and 
                                        min_valid_y <= frontier_y <= max_valid_y):
                                    continue
                                
                                # 使用极低的安全距离要求
                                if min(self.distance_point_to_segment((frontier_x, frontier_y), w[0], w[1]) 
                                     for w in self.wall_segments) > self.robot_radius * 0.8:
                                    frontiers.append((frontier_x, frontier_y))
                
                # 如果仍然找到了一些前沿点，返回它们
                if frontiers:
                    return frontiers
            
            return merged_frontiers
        except Exception as e:
            print(f"检测前沿点错误: {e}")
            return []  # 返回空列表表示没有前沿点
    
    def visualize_frontiers(self):
        """可视化前沿点和出口点（但不再显示标记）"""
        # 不显示前沿点和出口点标记，只保留函数结构
        pass
    
    def select_next_frontier(self, frontiers):
        """基于BFS的全局探索策略，选择下一个要探索的前沿点"""
        try:
            if not frontiers:
                return None
            
            # 获取有效地图边界
            if hasattr(self, 'valid_map_bounds'):
                min_valid_x, max_valid_x, min_valid_y, max_valid_y = self.valid_map_bounds
            else:
                # 如果未定义边界，使用保守估计
                min_valid_x, min_valid_y = 0, 0
                max_valid_x, max_valid_y = self.max_row - 1, self.max_col - 1
            
            # 过滤掉超出有效地图边界的前沿点，并对边缘点进行调整
            valid_frontiers = []
            
            # 标记已探索区域 - 使用访问热力图判断区域是否已探索
            if not hasattr(self, 'explored_regions'):
                self.explored_regions = set()
            
            # 更新已探索区域
            current_x, current_y = self.robot_position
            grid_x = int(current_y / self.grid_size)
            grid_y = int(current_x / self.grid_size)
            
            # 将当前位置周围标记为已探索（较大范围）
            exploration_radius = 3  # 增大探索标记半径
            for r in range(-exploration_radius, exploration_radius + 1):
                for c in range(-exploration_radius, exploration_radius + 1):
                    if math.sqrt(r*r + c*c) <= exploration_radius:
                        self.explored_regions.add((grid_y + r, grid_x + c))
            
            for f in frontiers:
                # 检查是否在有效边界内
                if min_valid_x <= f[0] <= max_valid_x and min_valid_y <= f[1] <= max_valid_y:
                    # 检查该点是否在未探索区域
                    frontier_grid_y = int(f[0] / self.grid_size)
                    frontier_grid_x = int(f[1] / self.grid_size)
                    
                    # 检查该点周围是否有未探索区域
                    has_unexplored = False
                    check_radius = 2
                    for r in range(-check_radius, check_radius + 1):
                        for c in range(-check_radius, check_radius + 1):
                            if (frontier_grid_y + r, frontier_grid_x + c) not in self.explored_regions:
                                has_unexplored = True
                                break
                        if has_unexplored:
                            break
                    
                    if has_unexplored:
                        valid_frontiers.append(f)
                else:
                    # 对于边缘点，尝试调整到有效边界内
                    adjusted_x = max(min_valid_x, min(max_valid_x, f[0]))
                    adjusted_y = max(min_valid_y, min(max_valid_y, f[1]))
                    adjusted_point = (adjusted_x, adjusted_y)
                    
                    # 确保调整后的点在地图边界内且在未探索区域
                    if self.is_within_map_boundary(adjusted_point):
                        frontier_grid_y = int(adjusted_x / self.grid_size)
                        frontier_grid_x = int(adjusted_y / self.grid_size)
                        
                        # 检查调整后的点周围是否有未探索区域
                        has_unexplored = False
                        for r in range(-check_radius, check_radius + 1):
                            for c in range(-check_radius, check_radius + 1):
                                if (frontier_grid_y + r, frontier_grid_x + c) not in self.explored_regions:
                                    has_unexplored = True
                                    break
                            if has_unexplored:
                                break
                        
                        if has_unexplored:
                            valid_frontiers.append(adjusted_point)
            
            if not valid_frontiers:
                print("所有前沿点都超出有效地图边界或已探索过，无法选择目标")
                # 如果没有有效的前沿点，尝试在未探索区域中找一个点
                self.status_label.config(text="寻找新的未探索区域...")
                
                # 获取未探索的网格点
                unexplored_grids = []
                for r in range(self.grid_rows):
                    for c in range(self.grid_cols):
                        if (r, c) not in self.explored_regions and self.map_grid[r, c] == 0:
                            # 转换为实际坐标
                            x = r * self.grid_size
                            y = c * self.grid_size
                            if self.is_within_map_boundary((x, y)) and min_valid_x <= x <= max_valid_x and min_valid_y <= y <= max_valid_y:
                                unexplored_grids.append((x, y))
                
                # 如果找到了未探索的区域，选择最近的一个
                if unexplored_grids:
                    robot_x, robot_y = self.robot_position
                    nearest = min(unexplored_grids, key=lambda p: math.sqrt((p[0]-robot_x)**2 + (p[1]-robot_y)**2))
                    return nearest
                
                return None
            
            # 过滤掉距离墙壁太近的前沿点 - 降低安全距离要求
            safety_distance = self.robot_radius * 1.2  # 降低安全距离要求
            safe_frontiers = []
            
            for frontier in valid_frontiers:
                too_close = False
                for wall in self.wall_segments:
                    if self.distance_point_to_segment(frontier, wall[0], wall[1]) <= safety_distance:
                        too_close = True
                        break
                if not too_close:
                    safe_frontiers.append(frontier)
            
            # 如果没有安全的前沿点，则尝试使用原始前沿点
            if not safe_frontiers and valid_frontiers:
                # 找出最不靠近墙壁的前沿点
                furthest_frontier = max(valid_frontiers, 
                                      key=lambda f: min(self.distance_point_to_segment(f, w[0], w[1]) 
                                                       for w in self.wall_segments))
                safe_frontiers = [furthest_frontier]
                
            # 记录已访问区域的热力图
            if not hasattr(self, 'visit_heatmap'):
                self.visit_heatmap = np.zeros((self.grid_rows, self.grid_cols))
                self.global_exploration_count = 0
                self.exploration_phases = ['bfs_explore', 'target_edges', 'revisit']
                self.current_phase = 'bfs_explore'
                
            # 更新当前位置的访问热力图
            current_x, current_y = self.robot_position
            current_grid_x = int(current_y / self.grid_size)
            current_grid_y = int(current_x / self.grid_size)
            visit_radius = 2  # 影响半径
            for dr in range(-visit_radius, visit_radius + 1):
                for dc in range(-visit_radius, visit_radius + 1):
                    r, c = current_grid_y + dr, current_grid_x + dc
                    if 0 <= r < self.grid_rows and 0 <= c < self.grid_cols:
                        self.visit_heatmap[r, c] += 1
            
            # 增加全局探索计数
            self.global_exploration_count += 1
            
                        # 检查是否已找到出口点，如果已找到出口但未探索完毕，调整探索策略
            if hasattr(self, 'exit_found_time') and not hasattr(self, 'exploration_complete'):
                # 检查发现出口后的时间是否已经超过一定阈值，确保不会过早结束探索
                # 根据速度因子调整时间阈值，速度越快，阈值越小
                time_threshold = max(5.0, 10.0 / self.speed_factor)
                if time.time() - self.exit_found_time > time_threshold:  # 切换到更全面的探索策略
                    self.current_phase = 'revisit'  # 直接切换到重新访问策略，以确保探索完整
                    self.status_label.config(text="已找到出口，正在完成地图探索...")
            
            # 根据当前探索阶段设置探索策略
            next_target = None
            if self.current_phase == 'bfs_explore':
                next_target = self.bfs_exploration_strategy(safe_frontiers)

                            # 最后确保所选目标点在有效边界内
            if next_target:
                tx, ty = next_target
                if not (min_valid_x <= tx <= max_valid_x and min_valid_y <= ty <= max_valid_y):
                    print(f"选择的目标点 ({tx:.1f}, {ty:.1f}) 超出有效边界，尝试调整")
                    # 调整到最近的有效边界点，同时留出安全余量
                    safety_margin = 1.0  # 增加安全余量
                    adjusted_tx = max(min_valid_x + safety_margin, min(max_valid_x - safety_margin, tx))
                    adjusted_ty = max(min_valid_y + safety_margin, min(max_valid_y - safety_margin, ty))
                    
                    # 确保调整后的点在地图边界内
                    if not self.is_within_map_boundary((adjusted_tx, adjusted_ty)):
                        # 进一步调整，确保在地图边界内
                        adjusted_tx = max(1.0, min(self.max_row - 1.0, adjusted_tx))
                        adjusted_ty = max(1.0, min(self.max_col - 1.0, adjusted_ty))
                    
                    next_target = (adjusted_tx, adjusted_ty)
                    print(f"目标点已调整为 ({adjusted_tx:.1f}, {adjusted_ty:.1f})")
            
            return next_target
                
        except Exception as e:
            print(f"选择前沿点错误: {e}")
            # 出错时使用简单的最近前沿点策略
            if frontiers:
                # 过滤出有效边界内的前沿点
                if hasattr(self, 'valid_map_bounds'):
                    min_valid_x, max_valid_x, min_valid_y, max_valid_y = self.valid_map_bounds
                    valid_frontiers = [f for f in frontiers if 
                                     min_valid_x <= f[0] <= max_valid_x and 
                                     min_valid_y <= f[1] <= max_valid_y]
                else:
                    valid_frontiers = [f for f in frontiers if self.is_within_map_boundary(f)]
                
                if valid_frontiers:
                    robot_x, robot_y = self.robot_position
                    nearest_frontier = min(valid_frontiers, 
                                         key=lambda f: math.sqrt((f[0]-robot_x)**2 + (f[1]-robot_y)**2))
                    return nearest_frontier
            return None
    
    def bfs_exploration_strategy(self, frontiers):
        """基于BFS的探索策略，系统地探索未知区域"""
        try:
            # 当前机器人位置
            robot_x, robot_y = self.robot_position
            
            # 将机器人位置转换为网格坐标
            start_grid_x = int(robot_y / self.grid_size)
            start_grid_y = int(robot_x / self.grid_size)
            
            # 创建距离图，用于BFS
            distance_map = np.full((self.grid_rows, self.grid_cols), np.inf)
            distance_map[start_grid_y, start_grid_x] = 0
            
            # BFS队列
            queue = deque([(start_grid_y, start_grid_x)])
            
            # 执行BFS，计算从机器人到每个可达网格的距离
            while queue:
                current_y, current_x = queue.popleft()
                current_dist = distance_map[current_y, current_x]
                
                # 探索八个方向，增加对角线方向
                for dy, dx in [(-1, 0), (0, 1), (1, 0), (0, -1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    next_y, next_x = current_y + dy, current_x + dx
                    
                    # 检查是否在地图范围内且是可通行区域
                    if (0 <= next_y < self.grid_rows and 0 <= next_x < self.grid_cols and
                        self.map_grid[next_y, next_x] != 1):  # 不是障碍物
                        
                        # 如果这个位置还没被访问过或有更短的路径
                        if distance_map[next_y, next_x] == np.inf:
                            # 对角线移动距离为1.414
                            move_cost = 1.414 if dx != 0 and dy != 0 else 1.0
                            distance_map[next_y, next_x] = current_dist + move_cost
                            queue.append((next_y, next_x))
            
            # 对每个前沿点计算分数
            frontier_scores = []
            
            # 首先检查哪些前沿点位于未探索区域
            unexplored_frontiers = []
            other_frontiers = []
            
            for frontier in frontiers:
                frontier_x, frontier_y = frontier
                # 转换为网格坐标
                grid_f_x = int(frontier_y / self.grid_size)
                grid_f_y = int(frontier_x / self.grid_size)
                
                # 确保坐标在有效范围内
                if not (0 <= grid_f_y < self.grid_rows and 0 <= grid_f_x < self.grid_cols):
                    continue
                
                # 检查该点周围是否有未探索区域
                has_unexplored = False
                for dr in range(-2, 3):
                    for dc in range(-2, 3):
                        nr, nc = grid_f_y + dr, grid_f_x + dc
                        if 0 <= nr < self.grid_rows and 0 <= nc < self.grid_cols:
                            if self.map_grid[nr, nc] == -1:  # 未知区域
                                has_unexplored = True
                                break
                    if has_unexplored:
                        break
                
                if has_unexplored:
                    unexplored_frontiers.append(frontier)
                else:
                    other_frontiers.append(frontier)
            
            # 优先考虑未探索区域的前沿点
            target_frontiers = unexplored_frontiers if unexplored_frontiers else frontiers
            
            for frontier_x, frontier_y in target_frontiers:
                # 转换为网格坐标
                grid_f_x = int(frontier_y / self.grid_size)
                grid_f_y = int(frontier_x / self.grid_size)
                
                # 确保坐标在有效范围内
                if not (0 <= grid_f_y < self.grid_rows and 0 <= grid_f_x < self.grid_cols):
                    continue
                
                # 如果这个前沿点不可达（距离为无穷大），跳过
                if distance_map[grid_f_y, grid_f_x] == np.inf:
                    continue
                
                # 计算到该前沿点的BFS距离
                bfs_distance = distance_map[grid_f_y, grid_f_x]
                
                # 获取访问频率
                visit_frequency = 0
                for dr in range(-2, 3):
                    for dc in range(-2, 3):
                        r, c = grid_f_y + dr, grid_f_x + dc
                        if 0 <= r < self.grid_rows and 0 <= c < self.grid_cols:
                            visit_frequency += self.visit_heatmap[r, c]
                
                # 计算潜在的信息增益
                info_gain = self.estimate_information_gain(frontier_x, frontier_y)
                
                # 检查是否在卡住区域附近
                in_stuck_region = any(
                    math.sqrt((frontier_x-sx)**2 + (frontier_y-sy)**2) < 2.0
                    for sx, sy in self.local_stuck_regions
                )
                
                # 检查该点周围是否有未探索区域，未探索区域优先级高
                has_unexplored = False
                for dr in range(-3, 4):
                    for dc in range(-3, 4):
                        nr, nc = grid_f_y + dr, grid_f_x + dc
                        if 0 <= nr < self.grid_rows and 0 <= nc < self.grid_cols:
                            if self.map_grid[nr, nc] == -1:  # 未知区域
                                has_unexplored = True
                                break
                    if has_unexplored:
                        break
                
                # 计算综合分数
                if in_stuck_region:
                    score = 1000 + bfs_distance  # 非常低的优先级
                else:
                    # 未探索区域的前沿点优先级更高
                    exploration_bonus = -50.0 if has_unexplored else 0.0
                    
                    # BFS距离是主要因素，但减小权重以优先考虑未探索区域
                    base_score = bfs_distance * 3.0
                    
                    # 访问频率惩罚
                    frequency_penalty = min(20, visit_frequency) * 1.0
                    
                    # 信息增益奖励，提高权重
                    info_gain_reward = -info_gain * 5.0
                    
                    # 随机因素，减小随机性
                    random_factor = random.uniform(-5.0, 5.0)
                    
                    # 最终分数
                    score = base_score + frequency_penalty + info_gain_reward + random_factor + exploration_bonus
                
                frontier_scores.append((score, (frontier_x, frontier_y)))
            
            # 选择分数最低的前沿点
            if frontier_scores:
                frontier_scores.sort()
                return frontier_scores[0][1]
            
            return None
        except Exception as e:
            print(f"BFS探索策略错误: {e}")
            # 异常情况下使用简单策略
            if frontiers:
                return random.choice(frontiers)
            return None
    
    def normalize_angle(self, angle):
        """将角度标准化到[-pi, pi]范围内"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
             
    def is_frontier_edge_tip(self, x, y):
        """检测指定位置是否是边缘尖端，但不再显示标记"""
        try:
            # 获取网格坐标
            grid_x = int(y / self.grid_size)
            grid_y = int(x / self.grid_size)
            
            # 检查是否超出边界
            if not (0 <= grid_y < self.grid_rows and 0 <= grid_x < self.grid_cols):
                return False
                
            # 如果不是已知空闲区域，则不是尖端
            if self.map_grid[grid_y, grid_x] != 0:
                return False
                
            # 检查周围8个方向
            neighbor_count = 0
            unknown_count = 0
            
            # 定义8个相邻方向的偏移量
            neighbors = [
                (-1, -1), (-1, 0), (-1, 1),
                (0, -1),           (0, 1),
                (1, -1),  (1, 0),  (1, 1)
            ]
            
            for dy, dx in neighbors:
                ny, nx = grid_y + dy, grid_x + dx
                if 0 <= ny < self.grid_rows and 0 <= nx < self.grid_cols:
                    if self.map_grid[ny, nx] == 0:  # 已知空闲
                        neighbor_count += 1
                    elif self.map_grid[ny, nx] == -1:  # 未知区域
                        unknown_count += 1
            
            # 如果周围有未知区域且已知空闲区域不多，则可能是尖端
            is_tip = unknown_count > 0 and neighbor_count <= 4
            
            if is_tip:
                # 计算信息增益
                info_gain = self.estimate_information_gain(x, y)
                # 不再显示标记
                return info_gain > 0
            
            return False
        except Exception as e:
            print(f"检测边缘尖端错误: {e}")
            return False
            
    def estimate_information_gain(self, x, y):
        """估计从某个点可以看到的未知区域数量"""
        try:
            # 转换为网格坐标
            grid_x = int(y / self.grid_size)
            grid_y = int(x / self.grid_size)
            
            # 模拟从该点进行雷达扫描，计算可能看到的未知区域
            unknown_count = 0
            scan_range = int(self.lidar_range / self.grid_size)
            
            # 简化的雷达模型：检查视线可及的未知区域数量
            for angle in np.linspace(0, 2*math.pi, 16):  # 16个方向
                # 沿射线检查单元格
                for dist in range(1, scan_range + 1):
                    dx = int(dist * math.cos(angle))
                    dy = int(dist * math.sin(angle))
                    
                    r, c = grid_y + dy, grid_x + dx
                    if 0 <= r < self.grid_rows and 0 <= c < self.grid_cols:
                        if self.map_grid[r, c] == -1:  # 未知区域
                            unknown_count += 1
                        elif self.map_grid[r, c] == 1:  # 障碍物，视线被阻挡
                            break
            
            return unknown_count
        except Exception as e:
            print(f"估计信息增益错误: {e}")
            return 0
       
    def verify_path_safety(self, path):
        """验证整个路径的安全性并可视化碰撞检测"""
        if len(path) < 2:
            return True
            
        # 首先检查机器人当前位置到第一个路径点
        current_x, current_y = self.robot_position
        first_point = path[0]
        
        # 检查当前位置到第一个点的路径是否安全
        for wall in self.wall_segments:
            if self.line_segments_intersect((current_x, current_y), first_point, wall[0], wall[1]):
                return False
        
        return True
    
    def min_distance_segment_to_segment(self, p1, p2, p3, p4):
        """计算两线段之间的最小距离"""
        try:
            # 检查线段是否相交
            if self.line_segments_intersect(p1, p2, p3, p4):
                return 0.0
            
            # 计算点到线段的距离
            d1 = self.distance_point_to_segment(p1, p3, p4)
            d2 = self.distance_point_to_segment(p2, p3, p4)
            d3 = self.distance_point_to_segment(p3, p1, p2)
            d4 = self.distance_point_to_segment(p4, p1, p2)
            
            # 返回最小距离
            return min(d1, d2, d3, d4)
        except Exception as e:
            print(f"计算线段间距离错误: {e}")
            return 0.0  # 错误时返回0表示不安全
    
    # def find_intersection_point(self, p1, p2, p3, p4):
    #     """计算两线段的交点，如果相交则返回交点坐标，否则返回None"""
    #     try:
    #         x1, y1 = p1
    #         x2, y2 = p2
    #         x3, y3 = p3
    #         x4, y4 = p4
            
    #         # 线段的方向向量
    #         dx1 = x2 - x1
    #         dy1 = y2 - y1
    #         dx2 = x4 - x3
    #         dy2 = y4 - y3
            
    #         # 计算分母
    #         denominator = dy2 * dx1 - dx2 * dy1
            
    #         # 如果分母为零，则线段平行或共线
    #         if abs(denominator) < 1e-10:
    #             return None
                
    #         # 计算参数t和u
    #         t = ((x3 - x1) * dy2 - (y3 - y1) * dx2) / denominator
    #         u = ((x3 - x1) * dy1 - (y3 - y1) * dx1) / denominator
            
    #         # 检查参数是否在[0,1]范围内，即线段是否相交
    #         if 0 <= t <= 1 and 0 <= u <= 1:
    #             # 计算交点坐标
    #             intersect_x = x1 + t * dx1
    #             intersect_y = y1 + t * dy1
    #             return (intersect_x, intersect_y)
                
    #         return None
    #     except Exception as e:
    #         print(f"计算交点错误: {e}")
    #         return None
    
    def plan_path(self, target):
        """使用A*算法进行路径规划，确保生成最短且不与墙壁相交的路径"""
        try:
            # 清除之前的路径可视化
            if hasattr(self, 'path_lines') and self.path_lines:
                for line in self.path_lines:
                    self.canvas.delete(line)
            self.path_lines = []
            
            # 清除之前的碰撞可视化
            self.canvas.delete("collision")
            
            if not target:
                return []
            
            # 获取有效地图边界
            if hasattr(self, 'valid_map_bounds'):
                min_valid_x, max_valid_x, min_valid_y, max_valid_y = self.valid_map_bounds
            else:
                # 如果未定义边界，使用保守估计
                min_valid_x, min_valid_y = 0, 0
                max_valid_x, max_valid_y = self.max_row, self.max_col
                
            # 检查目标点是否在有效地图边界内
            target_x, target_y = target
            if not (min_valid_x <= target_x <= max_valid_x and min_valid_y <= target_y <= max_valid_y):
                print(f"目标点 ({target_x:.1f}, {target_y:.1f}) 超出有效地图边界，调整目标点")
                # 调整目标点到有效边界内
                adjusted_target_x = max(min_valid_x, min(max_valid_x, target_x))
                adjusted_target_y = max(min_valid_y, min(max_valid_y, target_y))
                target = (adjusted_target_x, adjusted_target_y)
                print(f"目标点已调整为 ({adjusted_target_x:.1f}, {adjusted_target_y:.1f})")
                
            # 检查目标点是否在地图边界内
            if not self.is_within_map_boundary(target):
                print(f"目标点 {target} 超出地图边界，无法规划路径")
                return []
                
            # 将机器人位置和目标转换为网格坐标
            current_x, current_y = self.robot_position
            target_x, target_y = target
            
            # 检查目标点是否与墙体距离过近，如果是则调整位置
            safety_distance = self.robot_radius * 3.0  # 增加安全距离
            adjusted_target = target
            
            for wall in self.wall_segments:
                if self.distance_point_to_segment(target, wall[0], wall[1]) <= safety_distance:
                    # 目标点太靠近墙壁，尝试调整
                    wall_center = ((wall[0][0] + wall[1][0])/2, (wall[0][1] + wall[1][1])/2)
                    dir_from_wall = (target_x - wall_center[0], target_y - wall_center[1])
                    dist = math.sqrt(dir_from_wall[0]**2 + dir_from_wall[1]**2)
                    
                    if dist > 0:
                        # 归一化方向
                        dir_from_wall = (dir_from_wall[0]/dist, dir_from_wall[1]/dist)
                        # 向远离墙壁方向调整
                        adjust_distance = safety_distance * 1.2  # 额外安全距离
                        adjusted_x = target_x + dir_from_wall[0] * adjust_distance
                        adjusted_y = target_y + dir_from_wall[1] * adjust_distance
                        
                        # 确保调整后的点在有效地图边界内
                        adjusted_x = max(min_valid_x, min(max_valid_x, adjusted_x))
                        adjusted_y = max(min_valid_y, min(max_valid_y, adjusted_y))
                        
                        # 确保调整后的点在地图内
                        if self.is_within_map_boundary((adjusted_x, adjusted_y)):
                            # 确保调整后的点不会靠近其他墙壁
                            is_safe = True
                            for other_wall in self.wall_segments:
                                if other_wall != wall and self.distance_point_to_segment((adjusted_x, adjusted_y), 
                                                                                 other_wall[0], other_wall[1]) <= safety_distance:
                                    is_safe = False
                                    break
                            
                            if is_safe:
                                adjusted_target = (adjusted_x, adjusted_y)
                                target_x, target_y = adjusted_target
                                # 更新状态
                                self.status_label.config(text=f"目标点已调整以远离墙壁: ({adjusted_x:.1f}, {adjusted_y:.1f})")
                            else:
                                # 如果调整后的点靠近其他墙壁，尝试另一个方向
                                # 尝试找到远离所有墙壁的方向
                                best_direction = None
                                max_min_dist = 0
                                
                                # 尝试8个方向
                                for angle in range(0, 360, 45):
                                    rad_angle = math.radians(angle)
                                    test_x = target_x + math.cos(rad_angle) * safety_distance * 1.2
                                    test_y = target_y + math.sin(rad_angle) * safety_distance * 1.2
                                    
                                    # 确保测试点在有效地图边界内
                                    test_x = max(min_valid_x, min(max_valid_x, test_x))
                                    test_y = max(min_valid_y, min(max_valid_y, test_y))
                                    
                                    # 检查该方向是否安全
                                    if not self.is_within_map_boundary((test_x, test_y)):
                                        continue
                                    
                                    # 计算到所有墙壁的最小距离
                                    min_dist = float('inf')
                                    for w in self.wall_segments:
                                        dist = self.distance_point_to_segment((test_x, test_y), w[0], w[1])
                                        min_dist = min(min_dist, dist)
                                    
                                    if min_dist > max_min_dist:
                                        max_min_dist = min_dist
                                        best_direction = (test_x, test_y)
                                
                                if best_direction and max_min_dist > self.robot_radius * 1.5:
                                    adjusted_target = best_direction
                                    target_x, target_y = adjusted_target
                                    self.status_label.config(text=f"目标点已调整到最安全位置: ({target_x:.1f}, {target_y:.1f})")
                        break
            
            # 转换为网格坐标
            grid_current_x = int(current_y / self.grid_size)
            grid_current_y = int(current_x / self.grid_size)
            grid_target_x = int(target_y / self.grid_size)
            grid_target_y = int(target_x / self.grid_size)
            
            # 检查起点和终点是否在地图内
            if not (0 <= grid_current_y < self.grid_rows and 0 <= grid_current_x < self.grid_cols):
                print(f"起点 {(grid_current_y, grid_current_x)} 超出地图范围")
                return []
                
            if not (0 <= grid_target_y < self.grid_rows and 0 <= grid_target_x < self.grid_cols):
                print(f"终点 {(grid_target_y, grid_target_x)} 超出地图范围")
                return []
            
            # 检查终点是否是障碍物，但不再直接判断终点网格的值
            # 而是检查终点周围是否有足够的安全空间
            has_space = False
            search_radius = int(self.robot_radius / self.grid_size) + 1
            
            for dr in range(-search_radius, search_radius + 1):
                for dc in range(-search_radius, search_radius + 1):
                    ny, nx = grid_target_y + dr, grid_target_x + dc
                    if (0 <= ny < self.grid_rows and 0 <= nx < self.grid_cols and 
                        self.map_grid[ny, nx] == 0):  # 如果周围有已知空闲区域
                        has_space = True
                        break
                if has_space:
                    break
            
            if not has_space and self.map_grid[grid_target_y, grid_target_x] == 1:
                print("终点周围没有足够的安全空间，无法规划路径")
                return []
            
            # 创建安全距离地图 - 预先计算每个格子到最近墙壁的距离
            safety_map = np.ones((self.grid_rows, self.grid_cols)) * float('inf')
            for r in range(self.grid_rows):
                for c in range(self.grid_cols):
                    if self.map_grid[r, c] == 1:  # 墙壁
                        safety_map[r, c] = 0
                        # 向周围扩散安全距离信息
                        spread_radius = int(safety_distance / self.grid_size) + 1
                        for dr in range(-spread_radius, spread_radius + 1):
                            for dc in range(-spread_radius, spread_radius + 1):
                                nr, nc = r + dr, c + dc
                                if 0 <= nr < self.grid_rows and 0 <= nc < self.grid_cols:
                                    # 计算到墙壁的距离
                                    dist = math.sqrt(dr**2 + dc**2) * self.grid_size
                                    if dist < safety_map[nr, nc]:
                                        safety_map[nr, nc] = dist
            
            # A*算法的开放列表和关闭列表
            open_list = []
            closed_set = set()
            
            # 使用优先队列实现开放列表
            heapq.heappush(open_list, (0, (grid_current_y, grid_current_x)))
            
            # 记录从起点到每个节点的代价
            g_score = {(grid_current_y, grid_current_x): 0}
            
            # 记录每个节点的父节点，用于重建路径
            parent = {}
            
            # 定义8个方向（上、右、下、左、右上、右下、左下、左上）
            directions = [
                (-1, 0), (0, 1), (1, 0), (0, -1),
                (-1, 1), (1, 1), (1, -1), (-1, -1)
            ]
            
            # A*算法主循环
            while open_list:
                # 获取当前代价最小的节点
                current_f, (current_y, current_x) = heapq.heappop(open_list)
                
                # 如果到达目标，重建并返回路径
                if (current_y, current_x) == (grid_target_y, grid_target_x):
                    path = self.reconstruct_path(parent, (current_y, current_x), (grid_current_y, grid_current_x))
                    
                    # 将网格坐标转换回实际坐标
                    real_path = []
                    for y, x in path:
                        real_x = y * self.grid_size
                        real_y = x * self.grid_size
                        real_path.append((real_x, real_y))
                    
                    # 使用调整后的目标点替换最后一个点
                    if real_path and adjusted_target != target:
                        real_path[-1] = adjusted_target
                    
                    # 平滑路径
                    smoothed_path = self.smooth_path(real_path)
                    
                    # 验证平滑后的路径安全性
                    if not self.verify_path_safety(smoothed_path):
                        self.status_label.config(text="平滑路径后发现碰撞，尝试使用原始路径...")
                        
                        # 如果平滑路径不安全，尝试使用原始路径
                        if self.verify_path_safety(real_path):
                            smoothed_path = real_path
                            self.status_label.config(text="使用原始A*路径")
                        else:
                            self.status_label.config(text="无法找到无碰撞路径，尝试增加网格细度...")
                            
                            # 如果仍然不安全，可能需要更精细的网格或者其他方法
                            # 这里我们尝试使用更安全的路径点间距，重新生成路径
                            safer_path = self.generate_safer_path(current_x, current_y, target_x, target_y)
                            
                            if safer_path:
                                smoothed_path = safer_path
                                self.status_label.config(text="使用安全路径生成算法")
                            else:
                                # 如果仍然失败，返回一个空路径，让系统重新选择目标
                                self.status_label.config(text="所有路径规划方法失败，需要重新选择目标")
                                return []
                    
                    # 可视化规划的路径
                    self.visualize_planned_path(smoothed_path)
                    
                    return smoothed_path
                
                # 将当前节点加入关闭列表
                closed_set.add((current_y, current_x))
                
                # 检查所有相邻节点
                for i, (dy, dx) in enumerate(directions):
                    neighbor_y, neighbor_x = current_y + dy, current_x + dx
                    
                    # 检查是否在地图范围内
                    if not (0 <= neighbor_y < self.grid_rows and 0 <= neighbor_x < self.grid_cols):
                        continue
                    
                    # 转换为实际坐标
                    real_x = neighbor_y * self.grid_size
                    real_y = neighbor_x * self.grid_size
                    
                    # 检查是否在有效地图边界内
                    if not (min_valid_x <= real_x <= max_valid_x and min_valid_y <= real_y <= max_valid_y):
                        continue
                    
                    # 检查是否在地图边界内
                    if not self.is_within_map_boundary((real_x, real_y)):
                        continue
                    
                    # 检查是否是障碍物或已经在关闭列表中
                    if self.map_grid[neighbor_y, neighbor_x] == 1 or (neighbor_y, neighbor_x) in closed_set:
                        continue
                    
                    # 检查安全距离
                    if safety_map[neighbor_y, neighbor_x] < safety_distance * 0.8:  # 稍微放宽A*搜索的安全距离
                        continue  # 太靠近墙壁，不安全
                    
                    # 对角线移动时，检查两个相邻的格子是否有障碍物（避免穿墙）
                    if i >= 4:  # 对角线方向
                        if self.map_grid[current_y, neighbor_x] == 1 or self.map_grid[neighbor_y, current_x] == 1:
                            continue
                    
                    # 新增：检查从当前点到邻居点的线段是否与任何json墙壁线段有交点
                    from_pt = (current_y * self.grid_size, current_x * self.grid_size)
                    to_pt = (neighbor_y * self.grid_size, neighbor_x * self.grid_size)
                    has_intersection = False
                    for wall in self.wall_segments:
                        if self.line_segments_intersect(from_pt, to_pt, wall[0], wall[1]):
                            has_intersection = True
                            break
                    if has_intersection:
                        continue  # 这条边不可行，跳过
                    
                    # 计算从起点到该相邻节点的代价
                    # 对角线移动的代价为1.414（√2），直线移动的代价为1
                    move_cost = 1.414 if i >= 4 else 1
                    
                    # 增加靠近墙壁的代价
                    wall_penalty = 1.0
                    if safety_map[neighbor_y, neighbor_x] < safety_distance * 1.5:
                        # 离墙越近，惩罚越大
                        wall_penalty = 1.0 + (safety_distance * 1.5 - safety_map[neighbor_y, neighbor_x]) / safety_distance * 3.0
                    
                    # 应用惩罚到移动代价
                    move_cost *= wall_penalty
                    
                    tentative_g = g_score.get((current_y, current_x), float('inf')) + move_cost
                    
                    # 如果找到了更好的路径，或者该节点还未探索
                    if tentative_g < g_score.get((neighbor_y, neighbor_x), float('inf')):
                        # 更新父节点
                        parent[(neighbor_y, neighbor_x)] = (current_y, current_x)
                        
                        # 更新代价
                        g_score[(neighbor_y, neighbor_x)] = tentative_g
                        
                        # 计算启发式代价（使用曼哈顿距离）
                        h = self.heuristic((neighbor_y, neighbor_x), (grid_target_y, grid_target_x))
                        
                        # 计算总代价
                        f = tentative_g + h
                        
                        # 添加到开放列表
                        heapq.heappush(open_list, (f, (neighbor_y, neighbor_x)))
            
            # 如果开放列表为空但未找到路径，则无法到达目标
            print(f"无法规划到目标 ({target_x:.1f}, {target_y:.1f}) 的路径")
            return []
        except Exception as e:
            print(f"路径规划错误: {e}")
            return []  # 返回空列表表示无法规划路径
    
    def heuristic(self, node, goal):
        """A*算法的启发式函数，使用曼哈顿距离"""
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
    
    def reconstruct_path(self, parent, current, start):
        """从父节点字典中重建路径"""
        path = [current]
        while current != start:
            current = parent[current]
            path.append(current)
        path.reverse()  # 反转路径从起点到终点
        return path
    
    def smooth_path(self, path):
        """平滑路径，消除不必要的拐点，确保不与墙壁相交并保持足够安全距离"""
        try:
            if len(path) < 3:
                return path
            
            # 检测是否在狭窄通道内
            is_narrow_passage = self.detect_narrow_passage(path)
            
            # 根据是否在狭窄通道内调整安全距离
            if is_narrow_passage:
                safety_distance = self.robot_radius * 2.0  # 在狭窄通道中减小安全距离
                print("检测到狭窄通道，减小安全距离")
            else:
                safety_distance = self.robot_radius * 3.0  # 正常安全距离
                
            # 创建一个新的平滑路径，初始包含起点
            smoothed_path = [path[0]]
            
            # 检查整个路径与墙体的安全距离，调整过近的点
            safe_path = []
            for point in path:
                # 检查该点是否安全
                min_wall_dist = float('inf')
                closest_wall = None
                
                for wall in self.wall_segments:
                    dist = self.distance_point_to_segment(point, wall[0], wall[1])
                    if dist < min_wall_dist:
                        min_wall_dist = dist
                        closest_wall = wall
                
                # 如果该点太靠近墙体，调整它
                if min_wall_dist < safety_distance and closest_wall:
                    # 计算从墙壁到点的方向向量
                    wall_center = ((closest_wall[0][0] + closest_wall[1][0])/2, 
                                  (closest_wall[0][1] + closest_wall[1][1])/2)
                    dir_from_wall = (point[0] - wall_center[0], point[1] - wall_center[1])
                    dist = math.sqrt(dir_from_wall[0]**2 + dir_from_wall[1]**2)
                    
                    if dist > 0:
                        # 归一化方向向量
                        dir_from_wall = (dir_from_wall[0]/dist, dir_from_wall[1]/dist)
                        
                        # 在狭窄通道中减小调整量
                        if is_narrow_passage:
                            adjust_distance = max(0.1, safety_distance - min_wall_dist) * 0.7  # 减少70%的调整
                        else:
                            adjust_distance = safety_distance - min_wall_dist + 0.2  # 正常调整
                        
                        # 调整点的位置，使其与墙体保持足够距离
                        adjusted_x = point[0] + dir_from_wall[0] * adjust_distance
                        adjusted_y = point[1] + dir_from_wall[1] * adjust_distance
                        adjusted_point = (adjusted_x, adjusted_y)
                        
                        # 检查调整后的点是否仍然安全 (不会因为调整而更靠近另一个墙壁)
                        is_safer = True
                        for other_wall in self.wall_segments:
                            if other_wall != closest_wall:
                                orig_dist = self.distance_point_to_segment(point, other_wall[0], other_wall[1])
                                new_dist = self.distance_point_to_segment(adjusted_point, other_wall[0], other_wall[1])
                                if new_dist < orig_dist * 0.7:  # 如果调整后距离另一墙壁更近了，可能导致卡住
                                    is_safer = False
                                    break
                        
                        # 确保调整后的点在地图边界内且调整是有益的
                        if self.is_within_map_boundary(adjusted_point) and is_safer:
                            safe_path.append(adjusted_point)
                        else:
                            # 如果调整后更糟或超出边界，保留原点
                            safe_path.append(point)
                    else:
                        safe_path.append(point)
                else:
                    safe_path.append(point)  # 点已经足够安全
            
            # 替换原始路径为安全路径
            path = safe_path
            
            # 跳过中间点，尝试直接连接
            i = 0
            smoothed_path = [path[0]]  # 重新初始化平滑路径
            
            while i < len(path) - 1:
                # 当前平滑路径的最后一点
                current = smoothed_path[-1]
                
                # 尝试跳过尽可能多的中间点
                for j in range(len(path) - 1, i, -1):
                    # 检查直接连接是否与任何墙壁相交
                    path_is_clear = True
                    for wall in self.wall_segments:
                        # 检查线段是否与墙壁相交
                        if self.line_segments_intersect(current, path[j], wall[0], wall[1]):
                            path_is_clear = False
                            break
                        
                        # 检查线段与墙壁的最小距离是否小于安全距离
                        line_dist = self.min_distance_segment_to_segment(current, path[j], wall[0], wall[1])
                        if line_dist < safety_distance:
                            path_is_clear = False
                            break
                    
                    if path_is_clear:
                        # 可以安全连接
                        smoothed_path.append(path[j])
                        i = j
                        break
                
                # 如果无法跳过任何点，添加下一个点
                if smoothed_path[-1] == current and i < len(path) - 1:
                    smoothed_path.append(path[i + 1])
                    i += 1
            
            # 添加终点（如果还没有）
            if smoothed_path[-1] != path[-1]:
                smoothed_path.append(path[-1])
            
            return smoothed_path
        except Exception as e:
            print(f"路径平滑错误: {e}")
            return path
    
    def detect_narrow_passage(self, path):
        """检测路径是否在狭窄通道内"""
        try:
            if len(path) < 2:
                return False
                
            # 计算路径上每个点附近的墙壁距离
            narrow_point_count = 0
            narrow_threshold = self.robot_radius * 2.5  # 墙距小于这个值认为是狭窄
            
            for point in path:
                min_dist1 = float('inf')
                min_dist2 = float('inf')
                closest_wall = None
                
                # 找出最近的墙壁
                for wall in self.wall_segments:
                    dist = self.distance_point_to_segment(point, wall[0], wall[1])
                    if dist < min_dist1:
                        min_dist2 = min_dist1
                        min_dist1 = dist
                        closest_wall = wall
                    elif dist < min_dist2:
                        min_dist2 = dist
                
                # 检测是否在两面墙之间的狭窄区域
                if min_dist1 < narrow_threshold and min_dist2 < narrow_threshold * 1.5:
                    narrow_point_count += 1
            
            # 如果路径上超过一半的点在狭窄区域，认为是狭窄通道
            return narrow_point_count >= len(path) * 0.3
            
        except Exception as e:
            print(f"检测狭窄通道错误: {e}")
            return False
    
    # def generate_safer_path(self, start_x, start_y, end_x, end_y):
        """生成一条从起点到终点的安全路径，使用更细致的碰撞检测和路径生成算法"""
        try:
            # 首先计算方向向量
            dx = end_x - start_x
            dy = end_y - start_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # 如果距离太小，直接返回终点
            if distance < 0.5:
                return [(end_x, end_y)]
            
            # 尝试使用中间点方法
            # 1. 首先尝试直线连接，检查是否有碰撞
            direct_path = [(start_x, start_y), (end_x, end_y)]
            
            # 检查直线路径是否安全
            has_collision = False
            for wall in self.wall_segments:
                if self.line_segments_intersect((start_x, start_y), (end_x, end_y), wall[0], wall[1]):
                    has_collision = True
                    # 找出碰撞点
                    collision_point = self.find_intersection_point((start_x, start_y), (end_x, end_y), wall[0], wall[1])
                    break
            
            if not has_collision:
                return direct_path  # 直线路径安全
            
            # 2. 如果直线不安全，尝试找一些中间点
            # 首先归一化方向向量
            if distance > 0:
                dx /= distance
                dy /= distance
            
            # 计算垂直方向
            perp_dx = -dy
            perp_dy = dx
            
            # 尝试不同的偏移距离
            for side in [1, -1]:  # 尝试两侧
                for offset_dist in [1.0, 1.5, 2.0, 2.5, 3.0]:
                    # 计算中间点
                    mid_x = (start_x + end_x) / 2 + side * perp_dx * offset_dist
                    mid_y = (start_y + end_y) / 2 + side * perp_dy * offset_dist
                    
                    # 检查中间点是否安全
                    if not self.is_within_map_boundary((mid_x, mid_y)):
                        continue
                    
                    is_safe = True
                    for wall in self.wall_segments:
                        if (self.distance_point_to_segment((mid_x, mid_y), wall[0], wall[1]) < self.robot_radius * 1.5 or
                            self.line_segments_intersect((start_x, start_y), (mid_x, mid_y), wall[0], wall[1]) or
                            self.line_segments_intersect((mid_x, mid_y), (end_x, end_y), wall[0], wall[1])):
                            is_safe = False
                            break
                    
                    if is_safe:
                        # 找到安全中间点，构建路径
                        path = [(start_x, start_y), (mid_x, mid_y), (end_x, end_y)]
                        return path
            
            # 3. 如果简单的中间点方法失败，尝试使用更多的中间点
            # 使用一系列等间隔的点
            num_points = 5  # 尝试4个中间点
            safer_path = [(start_x, start_y)]
            
            for i in range(1, num_points):
                # 在直线上生成等间隔的点
                ratio = i / num_points
                point_x = start_x + dx * distance * ratio
                point_y = start_y + dy * distance * ratio
                
                # 检查从上一个点到当前点是否安全
                last_point = safer_path[-1]
                is_safe = True
                for wall in self.wall_segments:
                    if self.line_segments_intersect(last_point, (point_x, point_y), wall[0], wall[1]):
                        is_safe = False
                        
                        # 尝试调整这个点，避开墙体
                        for side in [1, -1]:
                            for adj_dist in [0.5, 1.0, 1.5, 2.0]:
                                adj_x = point_x + side * perp_dx * adj_dist
                                adj_y = point_y + side * perp_dy * adj_dist
                                
                                if self.is_within_map_boundary((adj_x, adj_y)):
                                    adj_is_safe = True
                                    for w in self.wall_segments:
                                        if (self.line_segments_intersect(last_point, (adj_x, adj_y), w[0], w[1]) or
                                            self.distance_point_to_segment((adj_x, adj_y), w[0], w[1]) < self.robot_radius * 1.5):
                                            adj_is_safe = False
                                            break
                                    
                                    if adj_is_safe:
                                        point_x, point_y = adj_x, adj_y
                                        is_safe = True
                                        break
                            
                            if is_safe:
                                break
                        
                        if not is_safe:
                            # 无法调整这个点，跳过
                            continue
                
                if is_safe:
                    safer_path.append((point_x, point_y))
            
            # 添加终点
            last_point = safer_path[-1]
            is_end_safe = True
            for wall in self.wall_segments:
                if self.line_segments_intersect(last_point, (end_x, end_y), wall[0], wall[1]):
                    is_end_safe = False
                    break
            
            if is_end_safe:
                safer_path.append((end_x, end_y))
                return safer_path
            
            # 如果以上方法都失败，返回空路径，表示无法找到安全路径
            return []
            
        except Exception as e:
            print(f"生成安全路径错误: {e}")
            return []
    
    def visualize_planned_path(self, path):
        """可视化规划的路径"""
        try:
            # 清除之前的路径线段
            if hasattr(self, 'path_lines') and self.path_lines:
                for line in self.path_lines:
                    self.canvas.delete(line)
            self.path_lines = []
            
            # 清除之前的碰撞点标记
            self.canvas.delete("collision")
            
            # 如果路径为空，直接返回
            if not path:
                return
            
            # 使用绿色显示规划路径
            for i in range(len(path) - 1):
                # 转换为画布坐标
                start_x = path[i][1] * self.cell_size + self.margin
                start_y = path[i][0] * self.cell_size + self.margin
                end_x = path[i+1][1] * self.cell_size + self.margin
                end_y = path[i+1][0] * self.cell_size + self.margin
                
                # 绘制线段
                line = self.canvas.create_line(start_x, start_y, end_x, end_y, fill="green", width=2)
                self.path_lines.append(line)
                
                # 标记路径点
                point_marker = self.canvas.create_oval(start_x-3, start_y-3, start_x+3, start_y+3, 
                                                    fill="green", outline="green")
                self.path_lines.append(point_marker)
            
            # 标记最后一个点
            if len(path) > 0:
                last_x = path[-1][1] * self.cell_size + self.margin
                last_y = path[-1][0] * self.cell_size + self.margin
                last_marker = self.canvas.create_oval(last_x-3, last_y-3, last_x+3, last_y+3, 
                                                    fill="green", outline="green")
                self.path_lines.append(last_marker)
            
            # 使用蓝色显示机器人当前位置到第一个路径点的线段
            if path:
                robot_x = self.robot_position[1] * self.cell_size + self.margin
                robot_y = self.robot_position[0] * self.cell_size + self.margin
                first_x = path[0][1] * self.cell_size + self.margin
                first_y = path[0][0] * self.cell_size + self.margin
                
                line = self.canvas.create_line(robot_x, robot_y, first_x, first_y, fill="blue", width=2)
                self.path_lines.append(line)
                
                # 验证路径安全性并可视化
                self.verify_path_safety(path)
        except Exception as e:
            print(f"可视化路径错误: {e}")
    
    def move_robot(self, target_position):
        """直接移动机器人到目标位置，不显示轨迹"""
        try:
            # 计算目标方向
            current_x, current_y = self.robot_position
            target_x, target_y = target_position
            
            # 检查是否在狭窄通道中
            in_narrow_passage = self.is_in_narrow_passage(current_x, current_y)
            
            # 检查目标点是否在有效地图边界和地图边界内
            if not self.is_within_map_boundary((target_x, target_y)):
                # 获取有效边界，尝试调整目标点
                if hasattr(self, 'valid_map_bounds'):
                    min_valid_x, max_valid_x, min_valid_y, max_valid_y = self.valid_map_bounds
                    
                    # 设置更大的安全余量
                    safety_margin = 1.0
                    adjusted_x = max(min_valid_x + safety_margin, min(max_valid_x - safety_margin, target_x))
                    adjusted_y = max(min_valid_y + safety_margin, min(max_valid_y - safety_margin, target_y))
                    
                    # 确保调整后的点在地图边界内
                    adjusted_x = max(0.5, min(self.max_row - 0.5, adjusted_x))
                    adjusted_y = max(0.5, min(self.max_col - 0.5, adjusted_y))
                    
                    # 如果调整后的点仍在有效边界内，使用它
                    if self.is_within_map_boundary((adjusted_x, adjusted_y)):
                        target_x, target_y = adjusted_x, adjusted_y
                        target_position = (target_x, target_y)
                        self.status_label.config(text=f"目标点已调整到边界内: ({target_x:.1f}, {target_y:.1f})")
                    else:
                        self.status_label.config(text="目标点超出地图边界，无法调整，重新规划...")
                        return False
                else:
                    # 如果没有定义有效边界，使用地图边界
                    adjusted_x = max(0.5, min(self.max_row - 0.5, target_x))
                    adjusted_y = max(0.5, min(self.max_col - 0.5, target_y))
                    
                    if abs(adjusted_x - target_x) > 0.1 or abs(adjusted_y - target_y) > 0.1:
                        target_x, target_y = adjusted_x, adjusted_y
                        target_position = (target_x, target_y)
                        self.status_label.config(text=f"目标点已调整到地图边界内: ({target_x:.1f}, {target_y:.1f})")
                    else:
                        self.status_label.config(text="目标点超出地图边界，重新规划...")
                        return False
            
            # 检查目标点和当前点之间是否有路径障碍
            has_obstacle = False
            for wall in self.wall_segments:
                if self.line_segments_intersect((current_x, current_y), (target_x, target_y), wall[0], wall[1]):
                    has_obstacle = True
                    break
                    
            # 如果有障碍，尝试找一个中间点
            if has_obstacle:
                # 尝试找一个无障碍的中间点
                self.status_label.config(text="直接路径有障碍，尝试寻找中间点...")
                
                # 计算向量方向
                dir_x = target_x - current_x
                dir_y = target_y - current_y
                dir_len = math.sqrt(dir_x**2 + dir_y**2)
                
                if dir_len > 0.01:
                    # 归一化向量
                    dir_x /= dir_len
                    dir_y /= dir_len
                    
                    # 尝试不同的偏移角度
                    found_path = False
                    for offset in [30, -30, 60, -60, 90, -90]:
                        # 计算偏移角度对应的方向
                        rad_offset = math.radians(offset)
                        rot_x = dir_x * math.cos(rad_offset) - dir_y * math.sin(rad_offset)
                        rot_y = dir_x * math.sin(rad_offset) + dir_y * math.cos(rad_offset)
                        
                        # 尝试不同的距离
                        for dist in [0.5, 1.0, 1.5, 2.0]:
                            mid_x = current_x + rot_x * dist
                            mid_y = current_y + rot_y * dist
                            
                            # 检查中间点是否安全
                            if self.is_within_map_boundary((mid_x, mid_y)):
                                path_safe = True
                                for wall in self.wall_segments:
                                    if self.distance_point_to_segment((mid_x, mid_y), wall[0], wall[1]) < self.robot_radius * 1.5:
                                        path_safe = False
                                        break
                                        
                                    # 检查从当前位置到中间点的路径是否安全
                                    if self.line_segments_intersect((current_x, current_y), (mid_x, mid_y), wall[0], wall[1]):
                                        path_safe = False
                                        break
                                
                                if path_safe:
                                    # 找到安全中间点，移向它
                                    target_x, target_y = mid_x, mid_y
                                    target_position = (target_x, target_y)
                                    found_path = True
                                    self.status_label.config(text=f"找到安全中间点: ({mid_x:.1f}, {mid_y:.1f})")
                                    break
                        
                        if found_path:
                            break
                            
                    if not found_path:
                        self.status_label.config(text="无法找到安全路径，尝试另一种方式...")
                        return False
            
            # 避免除以零错误
            if abs(target_x - current_x) < 1e-10 and abs(target_y - current_y) < 1e-10:
                return True  # 已经在目标点，无需移动
            
            # 计算目标方向
            target_heading = math.atan2(target_y - current_y, target_x - current_x)
            
            # 平滑调整朝向
            angle_diff = self.normalize_angle(target_heading - self.robot_heading)
            max_turn = 0.5  # 最大转向角度
            turn_amount = max(-max_turn, min(max_turn, angle_diff))
            self.robot_heading = target_heading  # 直接朝向目标
            
                            # 计算移动距离，大幅提高移动速度，同时在狭窄通道中保持谨慎
            distance_to_target = self.distance_to_point(target_position)
            
            # 如果在狭窄通道中，使用较小的步长以确保安全
            if in_narrow_passage:
                if distance_to_target < 0.5:
                    move_distance = min(0.1, distance_to_target)  # 在狭窄通道近距离时用小步长
                else:
                    move_distance = min(0.25, distance_to_target)  # 在狭窄通道中适度提高速度
                self.status_label.config(text="狭窄通道中谨慎移动")
            else:
                # 使用全局速度因子控制移动速度
                if distance_to_target < 0.5:
                    move_distance = min(0.3 * self.speed_factor, distance_to_target)  # 近距离时用中等步长
                else:
                    move_distance = min(0.5 * self.speed_factor, distance_to_target)  # 根据速度因子调整移动速度
            
            # 计算新位置
            new_x = current_x + move_distance * math.cos(self.robot_heading)
            new_y = current_y + move_distance * math.sin(self.robot_heading)
            
            # 检查新位置是否在地图边界内
            if not self.is_within_map_boundary((new_x, new_y)):
                # 尝试调整到边界内
                if hasattr(self, 'valid_map_bounds'):
                    min_valid_x, max_valid_x, min_valid_y, max_valid_y = self.valid_map_bounds
                    new_x = max(min_valid_x, min(max_valid_x, new_x))
                    new_y = max(min_valid_y, min(max_valid_y, new_y))
                    
                    # 如果调整后仍然不在有效边界内
                    if not self.is_within_map_boundary((new_x, new_y)):
                        self.status_label.config(text="移动将超出地图边界，重新规划...")
                        return False
                    
                    self.status_label.config(text="调整移动到边界内...")
                else:
                    self.status_label.config(text="移动将超出地图边界，重新规划...")
                    return False
            
            # 碰撞检测 - 先检查是否会直接碰撞墙壁
            direct_collision = False
            min_wall_distance = float('inf')
            closest_wall = None
            
            for wall in self.wall_segments:
                # 检查线段是否相交
                if self.line_segments_intersect(
                    (current_x, current_y), (new_x, new_y),
                    wall[0], wall[1]
                ):
                    direct_collision = True
                    break
                
                # 记录最近的墙体
                distance = self.distance_point_to_segment((new_x, new_y), wall[0], wall[1])
                if distance < min_wall_distance:
                    min_wall_distance = distance
                    closest_wall = wall
            
            # 如果会直接碰撞墙壁，尝试避开
            if direct_collision:
                # 尝试多个不同方向，找到最佳避障方向
                best_direction = None
                best_distance = -1
                
                # 尝试8个方向
                for angle_offset in [0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5, math.pi]:
                    test_angle = target_heading + angle_offset
                    test_x = current_x + move_distance * 0.5 * math.cos(test_angle)
                    test_y = current_y + move_distance * 0.5 * math.sin(test_angle)
                    
                    # 检查测试点是否在有效地图边界内
                    if not self.is_within_map_boundary((test_x, test_y)):
                        continue
                    
                    # 检查是否会碰撞
                    will_collide = False
                    min_distance = float('inf')
                    
                    for wall in self.wall_segments:
                        if self.line_segments_intersect(
                            (current_x, current_y), (test_x, test_y),
                            wall[0], wall[1]
                        ):
                            will_collide = True
                            break
                        
                        distance = self.distance_point_to_segment((test_x, test_y), wall[0], wall[1])
                        min_distance = min(min_distance, distance)
                    
                    # 如果不会碰撞且距离墙体足够远，记录这个方向
                    if not will_collide and min_distance > self.robot_radius and min_distance > best_distance:
                        best_distance = min_distance
                        best_direction = (test_x, test_y)
                
                # 如果找到了安全方向，使用它
                if best_direction:
                    new_x, new_y = best_direction
                    self.robot_heading = math.atan2(new_y - current_y, new_x - current_x)
                    self.status_label.config(text="避开障碍物...")
                else:
                    # 如果找不到安全方向，报告失败
                    self.status_label.config(text="无法找到安全路径，重新规划...")
                    return False
            # 如果太靠近墙壁，尝试调整位置
            elif min_wall_distance <= self.robot_radius * 1.5:
                # 墙体中心点
                if closest_wall:
                    wall_center = ((closest_wall[0][0] + closest_wall[1][0])/2, (closest_wall[0][1] + closest_wall[1][1])/2)
                    
                    # 计算从墙体到机器人的方向向量
                    dir_from_wall = (new_x - wall_center[0], new_y - wall_center[1])
                    dist = math.sqrt(dir_from_wall[0]**2 + dir_from_wall[1]**2)
                    
                    # 如果能计算有效方向
                    if dist > 0:
                        # 归一化方向向量
                        dir_from_wall = (dir_from_wall[0]/dist, dir_from_wall[1]/dist)
                        
                        # 计算调整距离
                        adjust_dist = (self.robot_radius * 1.5) - min_wall_distance + 0.1
                        
                        # 计算调整后的位置
                        adjusted_x = new_x + dir_from_wall[0] * adjust_dist
                        adjusted_y = new_y + dir_from_wall[1] * adjust_dist
                        
                        # 确保调整后的位置在有效地图边界内
                        if not self.is_within_map_boundary((adjusted_x, adjusted_y)):
                            if hasattr(self, 'valid_map_bounds'):
                                min_valid_x, max_valid_x, min_valid_y, max_valid_y = self.valid_map_bounds
                                adjusted_x = max(min_valid_x, min(max_valid_x, adjusted_x))
                                adjusted_y = max(min_valid_y, min(max_valid_y, adjusted_y))
                                
                                # 如果调整后仍然不在有效边界内
                                if not self.is_within_map_boundary((adjusted_x, adjusted_y)):
                                    self.status_label.config(text="调整后位置超出地图边界，尝试其他方向...")
                                    # 转入后续代码尝试其他方向
                                else:
                                    # 检查调整后的位置是否安全
                                    conflict = False
                                    for wall in self.wall_segments:
                                        if wall != closest_wall and self.distance_point_to_segment(
                                            (adjusted_x, adjusted_y), wall[0], wall[1]) <= self.robot_radius:
                                            conflict = True
                                            break
                                    
                                    if not conflict:
                                        new_x, new_y = adjusted_x, adjusted_y
                                        self.status_label.config(text="调整位置以远离墙壁...")
                                        # 更新机器人位置并返回成功
                                        self.robot_position = (new_x, new_y)
                                        self.last_position = (new_x, new_y)
                                        if self.robot_marker:
                                            self.canvas.delete(self.robot_marker)
                                        self.robot_marker = self.draw_robot()
                                        if hasattr(self, 'position_history'):
                                            self.position_history.append((new_x, new_y))
                                        else:
                                            self.position_history = deque([(new_x, new_y)], maxlen=20)
                                        return True
                            else:
                                self.status_label.config(text="调整后位置超出地图边界，尝试其他方向...")
                        
                        # 检查调整后的位置是否安全
                        conflict = False
                        for wall in self.wall_segments:
                            if wall != closest_wall and self.distance_point_to_segment(
                                (adjusted_x, adjusted_y), wall[0], wall[1]) <= self.robot_radius:
                                conflict = True
                                break
                        
                        if not conflict:
                            new_x, new_y = adjusted_x, adjusted_y
                            self.status_label.config(text="调整位置以远离墙壁...")
                        else:
                            # 如果调整后的位置不安全，尝试其他方向
                            safe_direction_found = False
                            for angle in range(0, 360, 45):
                                rad_angle = math.radians(angle)
                                test_x = current_x + move_distance * 0.3 * math.cos(rad_angle)
                                test_y = current_y + move_distance * 0.3 * math.sin(rad_angle)
                                
                                # 确保测试位置在有效地图边界内
                                if not self.is_within_map_boundary((test_x, test_y)):
                                    continue
                                
                                # 检查是否安全
                                is_safe = True
                                for w in self.wall_segments:
                                    if self.distance_point_to_segment((test_x, test_y), w[0], w[1]) <= self.robot_radius:
                                        is_safe = False
                                        break
                                
                                if is_safe:
                                    new_x, new_y = test_x, test_y
                                    self.robot_heading = rad_angle
                                    safe_direction_found = True
                                    self.status_label.config(text="找到安全方向...")
                                    break
                            
                            if not safe_direction_found:
                                # 如果没有找到安全方向，报告失败
                                self.status_label.config(text="无法找到安全移动方向...")
                                return False
            
            # 更新机器人位置
            self.robot_position = (new_x, new_y)
            self.last_position = (new_x, new_y)
            
            # 不记录机器人轨迹
            
            # 更新机器人显示
            if self.robot_marker:
                self.canvas.delete(self.robot_marker)
            self.robot_marker = self.draw_robot()
            
            # 检查是否达到一个前沿点，更新位置历史
            if hasattr(self, 'position_history'):
                self.position_history.append((new_x, new_y))
            else:
                self.position_history = deque([(new_x, new_y)], maxlen=20)
            
            return True
        except Exception as e:
            print(f"移动机器人错误: {e}")
            return False  # 移动失败
    
    def line_segments_intersect(self, p1, p2, p3, p4):
        """检查两条线段是否相交"""
        try:
            # 线段1: p1 -> p2
            # 线段2: p3 -> p4
            
            # 计算行列式
            def cross_product(p1, p2, p3):
                return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])
            
            # 检查线段p1p2是否跨越了p3p4
            d1 = cross_product(p3, p4, p1)
            d2 = cross_product(p3, p4, p2)
            
            # 检查线段p3p4是否跨越了p1p2
            d3 = cross_product(p1, p2, p3)
            d4 = cross_product(p1, p2, p4)
            
            # 如果两段线都跨越了对方，则它们相交
            if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
                return True
                
            # 检查共线情况
            if d1 == 0 and self.point_on_segment(p1, p3, p4):
                return True
            if d2 == 0 and self.point_on_segment(p2, p3, p4):
                return True
            if d3 == 0 and self.point_on_segment(p3, p1, p2):
                return True
            if d4 == 0 and self.point_on_segment(p4, p1, p2):
                return True
                
            return False
        except Exception as e:
            print(f"线段相交检测错误: {e}")
            return False
    
    def point_on_segment(self, p, segment_start, segment_end):
        """检查点是否在线段上"""
        try:
            return (min(segment_start[0], segment_end[0]) <= p[0] <= max(segment_start[0], segment_end[0]) and
                    min(segment_start[1], segment_end[1]) <= p[1] <= max(segment_start[1], segment_end[1]))
        except Exception as e:
            print(f"点在线段上检测错误: {e}")
            return False
    
    def draw_robot(self):
        """绘制机器人"""
        try:
            x, y = self.robot_position
            canvas_x = y * self.cell_size + self.margin
            canvas_y = x * self.cell_size + self.margin
            
            # 绘制机器人本体（红色圆圈）
            robot = self.canvas.create_oval(canvas_x-7, canvas_y-7, canvas_x+7, canvas_y+7, fill="red", outline="")
            
            return robot
        except Exception as e:
            print(f"绘制机器人错误: {e}")
            return None
    
    def is_in_narrow_passage(self, x, y):
        """检测机器人是否在狭窄通道中"""
        try:
            position = (x, y)
            # 检查周围墙壁的距离
            distances = []
            directions = []
            
            for wall in self.wall_segments:
                dist = self.distance_point_to_segment(position, wall[0], wall[1])
                if dist < self.robot_radius * 4:  # 只关注较近的墙壁
                    distances.append(dist)
                    
                    # 计算墙壁的方向向量
                    wall_dir = (wall[1][0] - wall[0][0], wall[1][1] - wall[0][1])
                    length = math.sqrt(wall_dir[0]**2 + wall_dir[1]**2)
                    if length > 0:
                        wall_dir = (wall_dir[0]/length, wall_dir[1]/length)
                        directions.append(wall_dir)
            
            if len(distances) >= 2:
                # 找出最近的两面墙
                distances = sorted(distances)
                
                # 如果两面最近的墙都很近，且距离相近，可能是在狭窄通道中
                if (distances[0] < self.robot_radius * 2.5 and 
                    distances[1] < self.robot_radius * 3.0 and
                    abs(distances[0] - distances[1]) < self.robot_radius * 1.5):
                    
                    # 如果有方向信息，可以进一步判断这两面墙是否大致平行
                    if len(directions) >= 2:
                        # 检查任意两面墙是否大致平行
                        for i in range(len(directions)):
                            for j in range(i+1, len(directions)):
                                dot_product = abs(directions[i][0]*directions[j][0] + directions[i][1]*directions[j][1])
                                # 如果两面墙大致平行 (点积接近1或-1)
                                if dot_product > 0.7:
                                    return True
                    
                    # 没有足够的方向信息时，仅基于距离判断
                    return True
            
            return False
        except Exception as e:
            print(f"狭窄通道检测出错: {e}")
            return False

    def exploration_step(self):
        """执行一步探索"""
        try:
            if not self.is_exploring:
                return
            
            # 检查是否已完成探索且需要返回起点
            if self.exploration_complete and self.returning_home:
                # 如果正在返回起点，检查是否已到达
                if self.distance_to_point(self.start_point) < self.grid_size * 2:
                    print("已返回起点，探索任务完成！")
                    self.is_exploring = False
                    self.status_label.config(text="状态: 探索完成，已返回起点")
                    
                    # 标记出口
                    self.mark_segment_exits()
                    # 新增：只保留A*路径规划和移动到出口
                    exits = self.find_exits_from_segments()
                    if exits:
                        # 选择最近的出口
                        robot_pos = self.robot_position
                        exit_target = min(exits, key=lambda p: self.distance_between_points(robot_pos, p))
                        self.current_target = exit_target
                        self.exploration_path = self.plan_path(exit_target)
                        if self.exploration_path:
                            self.status_label.config(text=f"前往出口: {exit_target}")
                            # 关闭所有探索、扫描、前沿点等功能，只保留路径规划和移动
                            self.is_exploring = False
                            self.returning_home = False
                            self.exploration_complete = False
                            # 只循环移动到出口
                            self.move_to_exit_only()
                    return
                    
                # 继续沿着返回起点的路径移动
                if not self.exploration_path or len(self.exploration_path) == 0:
                    # 如果没有路径，重新规划到起点的路径
                    self.current_target = self.start_point
                    self.exploration_path = self.plan_path(self.start_point)
                    if not self.exploration_path:
                        print("无法规划到起点的路径，尝试紧急返回")
                        # 如果无法规划路径，尝试直接移动
                        if self.move_robot(self.start_point):
                            print("直接移动到起点成功")
                            self.is_exploring = False
                            self.status_label.config(text="状态: 探索完成，已返回起点")
                            return
                        else:
                            # 如果直接移动也失败，尝试紧急逃脱
                            self.emergency_escape()
                            # 重新规划路径
                            self.exploration_path = self.plan_path(self.start_point)
                
                # 移动机器人沿路径返回
                self.move_robot_along_path()
                
                # 重新绘制机器人
                self.draw_robot()
                
                # 继续下一步
                delay = int(max(10, 100 / self.speed_factor))
                self.root.after(delay, self.exploration_step)
                return
                
            # 如果尚未标记为完成探索，检查探索完成度
            if not self.exploration_complete:
                # 每次迭代都检查探索完成度
                is_complete, coverage = self.check_exploration_completion()
                
                # 显示当前探索状态和覆盖率
                self.robot_status_label.config(text=f"状态: 探索中 (墙体覆盖率: {coverage:.1%})")
                
                if is_complete:
                    print(f"探索完成! 墙体覆盖率: {coverage:.2%}")
                    self.exploration_complete = True
                    self.returning_home = True
                    self.status_label.config(text=f"状态: 探索完成，正在返回起点 (覆盖率: {coverage:.2%})")
                    
                    # 规划返回起点的路径
                    self.current_target = self.start_point
                    self.exploration_path = self.plan_path(self.start_point)
                    if self.exploration_path:
                        # 继续下一步
                        delay = int(max(10, 100 / self.speed_factor))
                        self.root.after(delay, self.exploration_step)
                        return
            
            # 执行雷达扫描
            scan_results = self.lidar_scan()
            
            # 更新地图
            self.update_map(scan_results)
            
            # 如果有当前路径，检查路径是否穿墙
            if self.exploration_path and len(self.exploration_path) > 1:
                path_is_valid = True
                # 检查机器人到第一个路径点是否穿墙
                current_x, current_y = self.robot_position
                next_x, next_y = self.exploration_path[0]
                
                # 检查当前位置到第一个路径点是否穿墙
                for wall in self.wall_segments:
                    if self.line_segments_intersect(
                        (current_x, current_y), (next_x, next_y),
                        wall[0], wall[1]):
                        path_is_valid = False
                        self.status_label.config(text="检测到路径穿墙，重新规划...")
                        break
                
                # 检查路径中相邻点之间是否穿墙
                if path_is_valid:
                    for i in range(len(self.exploration_path) - 1):
                        p1 = self.exploration_path[i]
                        p2 = self.exploration_path[i + 1]
                        
                        for wall in self.wall_segments:
                            if self.line_segments_intersect(p1, p2, wall[0], wall[1]):
                                path_is_valid = False
                                self.status_label.config(text="检测到路径穿墙，重新规划...")
                                break
                        
                        if not path_is_valid:
                            break
                
                # 如果路径穿墙，清空当前路径并安排重新规划
                if not path_is_valid:
                    self.exploration_path = []
                    self.current_target = None
            
            # 卡住计数器
            if not hasattr(self, 'stuck_counter'):
                self.stuck_counter = 0
            
            # 记录当前位置，用于检测卡住情况
            if not hasattr(self, 'last_positions'):
                self.last_positions = deque(maxlen=10)
            
            # 添加当前位置到历史记录
            self.last_positions.append(self.robot_position)
            
            # 检查是否卡住（位置几乎不变）
            if len(self.last_positions) >= 5:
                # 计算最近几个位置的移动距离总和
                total_movement = 0
                for i in range(1, len(self.last_positions)):
                    total_movement += self.distance_between_points(
                        self.last_positions[i], self.last_positions[i-1]
                    )
                
                # 如果总移动距离小于阈值，认为卡住了
                if total_movement < 0.5:
                    self.stuck_counter += 5  # 大幅增加卡住计数
                    self.status_label.config(text=f"检测到机器人严重卡住! 移动距离: {total_movement:.2f}")
                    
                    # 立即进行紧急移动
                    if self.stuck_counter > 8:
                        self.emergency_escape()
                        # 重置卡住计数器
                        self.stuck_counter = 0
                elif total_movement < 1.0:
                    # 轻微卡住
                    self.stuck_counter += 2
                    self.status_label.config(text=f"检测到机器人轻微卡住 (移动: {total_movement:.2f})")
                else:
                    # 正常移动，减少卡住计数
                    self.stuck_counter = max(0, self.stuck_counter - 1)
                
            # 探索完成标志 - 新增变量用于跟踪是否已完成探索
            if not hasattr(self, 'exploration_complete'):
                self.exploration_complete = False
                
            # 记录是否发现了出口
            if self.exit_points and not hasattr(self, 'exit_found_time'):
                self.exit_found_time = time.time()
                self.status_label.config(text=f"发现出口点: ({self.exit_points[0][0]:.1f}, {self.exit_points[0][1]:.1f})，继续探索...")
            
            # 如果没有当前目标或已经完成所有路径点
            if not self.current_target or not self.exploration_path:
                # 检测前沿点
                self.frontier_points = self.detect_frontiers()
                
                # 如果前沿点很少，提高扫描频率来寻找更多可能的前沿点
                if len(self.frontier_points) < 3:
                    self.status_label.config(text="前沿点太少，执行360°扫描...")
                    # 进行一次360度的扫描
                    original_heading = self.robot_heading
                    scan_angles = np.linspace(0, 2*math.pi, 24)  # 增加到24个方向以提高扫描精度
                    for angle in scan_angles:
                        self.robot_heading = angle
                        scan_results = self.lidar_scan()
                        self.update_map(scan_results)
                    self.robot_heading = original_heading
                    # 重新检测前沿点
                    self.frontier_points = self.detect_frontiers()
                    
                    # 如果仍然找不到足够的前沿点，尝试移动到新位置再扫描
                    if len(self.frontier_points) < 2:
                        # 尝试随机移动一小段距离
                        self.status_label.config(text="尝试移动到新位置以寻找前沿点...")
                        current_x, current_y = self.robot_position
                        for angle in range(0, 360, 45):
                            rad_angle = math.radians(angle)
                            dx = math.cos(rad_angle) * 0.5
                            dy = math.sin(rad_angle) * 0.5
                            new_x = current_x + dx
                            new_y = current_y + dy
                            
                            if self.is_within_map_boundary((new_x, new_y)):
                                # 检查是否安全
                                is_safe = True
                                for wall in self.wall_segments:
                                    if self.distance_point_to_segment((new_x, new_y), wall[0], wall[1]) <= self.robot_radius * 1.5:
                                        is_safe = False
                                        break
                                
                                if is_safe:
                                    if self.move_robot((new_x, new_y)):
                                        # 移动成功，执行新的扫描
                                        scan_results = self.lidar_scan()
                                        self.update_map(scan_results)
                                        self.frontier_points = self.detect_frontiers()
                                        break
                
                # 选择下一个要探索的前沿点
                self.current_target = self.select_next_frontier(self.frontier_points)
                
                # 如果仍然没有目标，随机选择一个方向进行探索
                if not self.current_target and self.is_exploring:
                    self.status_label.config(text="没有找到前沿点，随机选择探索方向...")
                    current_x, current_y = self.robot_position
                    
                    # 在各个方向上尝试一定距离，看哪个方向有更多的未知区域
                    best_dir = None
                    max_unknown = -1
                    
                    for angle in range(0, 360, 30):
                        rad_angle = math.radians(angle)
                        dx = math.cos(rad_angle)
                        dy = math.sin(rad_angle)
                        
                        # 检查2个单位距离的点
                        check_x = current_x + dx * 2.0
                        check_y = current_y + dy * 2.0
                        
                        if self.is_within_map_boundary((check_x, check_y)):
                            # 检查路径安全性
                            path_safe = True
                            for wall in self.wall_segments:
                                if self.line_segments_intersect(
                                    (current_x, current_y), (check_x, check_y),
                                    wall[0], wall[1]):
                                    path_safe = False
                                    break
                            
                            if path_safe:
                                # 计算该方向上未知区域的数量
                                unknown_count = self.estimate_information_gain(check_x, check_y)
                                if unknown_count > max_unknown:
                                    max_unknown = unknown_count
                                    best_dir = (check_x, check_y)
                    
                    if best_dir:
                        self.current_target = best_dir
                        self.status_label.config(text=f"选择随机方向探索: ({best_dir[0]:.1f}, {best_dir[1]:.1f})")
                
                # 规划路径
                if self.current_target:
                    # 规划前先清理路径
                    self.exploration_path = []
                    # 尝试规划路径，最多尝试5次（增加尝试次数）
                    for attempt in range(5):
                        self.exploration_path = self.plan_path(self.current_target)
                        if self.exploration_path:
                            break
                        # 如果规划失败，先执行一次完整扫描，更新地图
                        if attempt == 1:
                            self.status_label.config(text="规划失败，执行完整扫描...")
                            original_heading = self.robot_heading
                            scan_angles = np.linspace(0, 2*math.pi, 24)
                            for angle in scan_angles:
                                self.robot_heading = angle
                                scan_results = self.lidar_scan()
                                self.update_map(scan_results)
                            self.robot_heading = original_heading
                        
                        # 然后尝试不同的方向寻找新目标
                        current_x, current_y = self.robot_position
                        target_x, target_y = self.current_target
                        
                        # 计算从机器人到目标的方向
                        dx = target_x - current_x
                        dy = target_y - current_y
                        dist = math.sqrt(dx*dx + dy*dy)
                        
                        if dist > 0.01:
                            # 归一化方向向量
                            dx /= dist
                            dy /= dist
                            
                            # 计算垂直方向
                            perp_dx = -dy
                            perp_dy = dx
                            
                            # 向垂直方向调整，避开障碍物
                            adjust_range = 0.5 + attempt * 0.5  # 随着尝试次数增加调整范围
                            side = 1 if random.random() > 0.5 else -1  # 随机选择一侧
                            
                            new_x = target_x + side * perp_dx * adjust_range
                            new_y = target_y + side * perp_dy * adjust_range
                            
                            # 确保新目标在地图边界内
                            if self.is_within_map_boundary((new_x, new_y)):
                                self.current_target = (new_x, new_y)
                            else:
                                # 如果超出边界，尝试另一侧
                                new_x = target_x - side * perp_dx * adjust_range
                                new_y = target_y - side * perp_dy * adjust_range
                                
                                if self.is_within_map_boundary((new_x, new_y)):
                                    self.current_target = (new_x, new_y)
                                else:
                                    # 如果两侧都不行，尝试沿原方向后退
                                    new_x = target_x - dx * adjust_range
                                    new_y = target_y - dy * adjust_range
                                    
                                    if self.is_within_map_boundary((new_x, new_y)):
                                        self.current_target = (new_x, new_y)
                                    else:
                                        # 最后随机调整
                                        self.current_target = (
                                            target_x + random.uniform(-adjust_range, adjust_range),
                                            target_y + random.uniform(-adjust_range, adjust_range)
                                        )
                    
                    # 检查规划结果
                    if self.exploration_path:
                        self.status_label.config(text=f"前往新的探索点: ({self.current_target[0]:.1f}, {self.current_target[1]:.1f})")
                        self.path_label.config(text=f"规划路径长度: {len(self.exploration_path)} 点")
                        # 重置卡住计数器
                        self.stuck_counter = 0
                    else:
                        # 如果多次尝试后仍无法规划路径，重新选择目标
                        self.status_label.config(text="无法规划到当前目标的路径，重新选择目标")
                        self.current_target = None
                        # 安排下一步探索后直接返回
                        self.root.after(30, self.exploration_step)
                        return
                
                # 如果没有前沿点了，可能表示探索完成
                if not self.current_target:
                    # 检查探索完成情况
                    is_complete, coverage = self.check_exploration_completion()
                    
                    if is_complete:
                        # 探索完成标志设置为真
                        self.exploration_complete = True
                        
                        # 标记为返回起点状态
                        self.returning_home = True
                        self.status_label.config(text=f"探索完成！墙体覆盖率: {coverage:.2%}，开始返回起点")
                        
                        # 规划到起点的路径
                        self.current_target = self.start_point
                        home_path = self.plan_path(self.start_point)
                        if home_path:
                            self.exploration_path = home_path
                            self.path_label.config(text=f"返回起点的路径长度: {len(home_path)} 点")
                        else:
                            # 如果无法规划回起点的路径，先尝试去出口点（如果有的话）
                            if self.exit_points:
                                sorted_exits = sorted(self.exit_points, key=lambda p: self.distance_to_point(p))
                                closest_exit = sorted_exits[0]
                                self.status_label.config(text=f"无法回到起点，尝试前往最近出口: ({closest_exit[0]:.1f}, {closest_exit[1]:.1f})")
                                exit_path = self.plan_path(closest_exit)
                                if exit_path:
                                    self.current_target = closest_exit
                                    self.exploration_path = exit_path
                                    self.path_label.config(text=f"到出口的路径长度: {len(exit_path)} 点")
                                else:
                                    # 如果无法规划到出口，结束探索
                                    self.status_label.config(text="无法规划返回路径，探索结束")
                                    self.is_exploring = False
                                    return
                            else:
                                # 如果没有出口点且无法回到起点，结束探索
                                self.status_label.config(text="地图探索完成！但无法返回起点")
                                self.is_exploring = False
                                return
                    else:
                        # 如果探索还未完成但没有前沿点，可能需要进行360度扫描
                        self.status_label.config(text="没有前沿点但探索未完成(覆盖率: {:.2%})，执行360°扫描".format(coverage))
                        
                        # 执行360度扫描
                        original_heading = self.robot_heading
                        for angle in range(0, 360, 15):  # 每15度扫描一次
                            self.robot_heading = math.radians(angle)
                            scan_results = self.lidar_scan()
                            self.update_map(scan_results)
                        self.robot_heading = original_heading
                        
                        # 重新检查探索完成情况
                        is_complete, coverage = self.check_exploration_completion()
                        if is_complete:
                            # 如果扫描后确认探索完成
                            self.exploration_complete = True
                            self.returning_home = True
                            self.status_label.config(text=f"探索完成！墙体覆盖率: {coverage:.2%}，开始返回起点")
                            
                            # 规划到起点的路径
                            self.current_target = self.start_point
                            home_path = self.plan_path(self.start_point)
                            if home_path:
                                self.exploration_path = home_path
                                self.path_label.config(text=f"返回起点的路径长度: {len(home_path)} 点")
                            else:
                                self.status_label.config(text="无法规划返回起点的路径，探索结束")
                                self.is_exploring = False
                                return
                        else:
                            # 如果仍未完成探索且没有前沿点，尝试随机探索
                            self.status_label.config(text="仍有未探索区域但无前沿点，随机选择方向")
                            # 随机选择方向的代码保留原逻辑...
            
            # 移动机器人
            if self.exploration_path:
                next_point = self.exploration_path[0]
                
                # 移动机器人
                success = self.move_robot(next_point)
                
                # 如果移动失败，重新规划路径或寻找新目标
                if not success:
                    # 增加卡住计数器
                    self.stuck_counter += 1
                    
                    # 检查是否在狭窄通道中
                    in_narrow = self.is_in_narrow_passage(*self.robot_position)
                    
                    # 如果连续多次失败，放弃当前目标
                    # 在狭窄通道中给予更多尝试次数和特殊处理
                    stuck_threshold = 8 if in_narrow else 5
                    
                    if self.stuck_counter > stuck_threshold:
                        # 将当前位置添加到卡住区域
                        self.local_stuck_regions.add(self.robot_position)
                        
                        # 在狭窄通道中尝试特殊移动策略
                        if in_narrow:
                            self.status_label.config(text="在狭窄通道中卡住，尝试特殊移动策略...")
                            
                            # 检查机器人当前朝向
                            current_x, current_y = self.robot_position
                            heading_rad = self.robot_heading
                            
                            # 尝试多个小步移动方向
                            narrow_escape_success = False
                            
                            # 尝试前进方向
                            for step_size in [0.3, 0.2, 0.1]:
                                if narrow_escape_success:
                                    break
                                
                                # 前进和后退方向
                                for direction in [1, -1]:
                                    move_x = current_x + direction * math.cos(heading_rad) * step_size
                                    move_y = current_y + direction * math.sin(heading_rad) * step_size
                                    
                                    # 检查移动是否安全
                                    is_safe = True
                                    if self.is_within_map_boundary((move_x, move_y)):
                                        for wall in self.wall_segments:
                                            if self.distance_point_to_segment((move_x, move_y), wall[0], wall[1]) < self.robot_radius * 0.8:
                                                is_safe = False
                                                break
                                        
                                        if is_safe:
                                            # 尝试移动
                                            self.robot_position = (move_x, move_y)
                                            if self.robot_marker:
                                                self.canvas.delete(self.robot_marker)
                                            self.robot_marker = self.draw_robot()
                                            narrow_escape_success = True
                                            self.status_label.config(text=f"在狭窄通道中{'前进' if direction > 0 else '后退'}一小步")
                                            self.stuck_counter = 0
                                            break
                            
                            # 如果小步移动成功，直接返回继续探索
                            if narrow_escape_success:
                                self.root.after(30, self.exploration_step)
                                return
                        
                        # 如果之前已找到出口且卡住，可能是在尝试去出口的路上卡住了
                        if self.exploration_complete and self.exit_points:
                            # 尝试选择不同的出口点
                            available_exits = [exit_p for exit_p in self.exit_points 
                                              if exit_p != self.current_target]
                            
                            if available_exits:
                                # 选择另一个出口点
                                alt_exit = min(available_exits, key=lambda p: self.distance_to_point(p))
                                self.status_label.config(text=f"尝试前往另一个出口点: ({alt_exit[0]:.1f}, {alt_exit[1]:.1f})")
                                self.current_target = alt_exit
                                self.exploration_path = self.plan_path(alt_exit)
                                self.stuck_counter = 0
                                
                                if self.exploration_path:
                                    # 安排下一步探索后直接返回
                                    self.root.after(30, self.exploration_step)
                                    return
                        
                        # 尝试随机移动来摆脱卡住状态
                        self.status_label.config(text="机器人卡住了，尝试随机移动...")
                        
                        # 尝试十六个方向的随机移动，增加方向数量以提高成功率
                        escape_success = False
                        directions = []
                        for angle in range(0, 360, 22):  # 每22.5度一个方向，共16个方向
                            rad = math.radians(angle)
                            directions.append((math.cos(rad), math.sin(rad)))
                        
                        random.shuffle(directions)
                        
                        current_x, current_y = self.robot_position
                        # 尝试不同的移动距离，从大到小
                        move_distances = [1.0, 0.7, 0.5, 0.3]
                        
                        for move_dist in move_distances:
                            if escape_success:
                                break
                                
                            for dx, dy in directions:
                                # 计算尝试移动的位置
                                new_x = current_x + dx * move_dist
                                new_y = current_y + dy * move_dist
                                
                                # 检查新位置是否安全
                                if self.is_within_map_boundary((new_x, new_y)):
                                    # 检查该位置是否离墙太近
                                    is_safe = True
                                    for wall in self.wall_segments:
                                        if self.distance_point_to_segment((new_x, new_y), wall[0], wall[1]) <= self.robot_radius * 1.5:
                                            is_safe = False
                                            break
                                            
                                    if is_safe:
                                        # 设置新的朝向
                                        self.robot_heading = math.atan2(dy, dx)
                                        # 尝试移动
                                        if self.move_robot((new_x, new_y)):
                                            escape_success = True
                                            self.status_label.config(text=f"成功逃离卡住区域（距离：{move_dist:.1f}）")
                                            break
                        
                        # 如果仍然无法移动，尝试更激进的策略
                        if not escape_success:
                            # 尝试"瞬移"到附近一个安全位置
                            for r in range(1, 11):  # 搜索半径从1到10
                                if escape_success:
                                    break
                                    
                                # 在圆形区域内随机选择点
                                for _ in range(20):  # 每个半径尝试20个随机点
                                    angle = random.uniform(0, 2*math.pi)
                                    dist = random.uniform(0, r)
                                    new_x = current_x + dist * math.cos(angle)
                                    new_y = current_y + dist * math.sin(angle)
                                    
                                    # 检查新位置是否安全
                                    if self.is_within_map_boundary((new_x, new_y)):
                                        is_safe = True
                                        for wall in self.wall_segments:
                                            if self.distance_point_to_segment((new_x, new_y), wall[0], wall[1]) <= self.robot_radius * 1.2:
                                                is_safe = False
                                                break
                                                
                                        if is_safe:
                                            # 强制设置位置，直接跳出卡住区域
                                            self.robot_position = (new_x, new_y)
                                            self.robot_heading = angle
                                            if self.robot_marker:
                                                self.canvas.delete(self.robot_marker)
                                            self.robot_marker = self.draw_robot()
                                            self.position_history.append((new_x, new_y))
                                            escape_success = True
                                            self.status_label.config(text="紧急逃离卡住状态")
                                            break
                        
                        # 如果随机移动失败，尝试紧急逃脱
                        if not escape_success:
                            self.status_label.config(text="随机移动失败，尝试紧急逃脱...")
                            
                            # 执行紧急逃脱
                            if self.emergency_escape():
                                self.status_label.config(text="成功通过紧急逃脱脱困!")
                                escape_success = True
                            else:
                                # 如果紧急逃脱也失败，尝试最后一招：直接移动到大范围内的随机安全位置
                                current_x, current_y = self.robot_position
                                max_attempts = 50  # 大量尝试点
                                
                                for _ in range(max_attempts):
                                    # 随机选择范围内一点
                                    rand_angle = random.uniform(0, 2*math.pi)
                                    rand_dist = random.uniform(3.0, 10.0)  # 大范围
                                    rand_x = current_x + rand_dist * math.cos(rand_angle)
                                    rand_y = current_y + rand_dist * math.sin(rand_angle)
                                    
                                    if self.is_within_map_boundary((rand_x, rand_y)):
                                        # 检查该点是否安全
                                        is_safe = True
                                        grid_x = int(rand_y / self.grid_size)
                                        grid_y = int(rand_x / self.grid_size)
                                        
                                        # 只考虑已知空闲区域
                                        if (0 <= grid_y < self.grid_rows and 
                                            0 <= grid_x < self.grid_cols and
                                            self.map_grid[grid_y, grid_x] == 0):
                                                
                                            # 检查距离墙体是否安全
                                            for wall in self.wall_segments:
                                                if self.distance_point_to_segment((rand_x, rand_y), wall[0], wall[1]) <= self.robot_radius * 1.5:
                                                    is_safe = False
                                                    break
                                            
                                            if is_safe:
                                                # 直接跳到这个位置
                                                self.robot_position = (rand_x, rand_y)
                                                self.robot_heading = rand_angle
                                                if self.robot_marker:
                                                    self.canvas.delete(self.robot_marker)
                                                self.robot_marker = self.draw_robot()
                                                self.status_label.config(text=f"紧急大范围传送到: ({rand_x:.1f}, {rand_y:.1f})")
                                                escape_success = True
                                                break
                        
                        # 如果所有方法都失败，最终放弃当前目标
                        if not escape_success:
                            self.status_label.config(text="所有脱困方法失败，重新规划全局目标...")
                            
                            # 记录当前区域为严重卡住区域，以后避开
                            rx, ry = self.robot_position
                            self.local_stuck_regions.add((rx, ry))
                            
                            # 如果卡住区域太多，可能是地图有问题，重新扫描周围环境
                            if len(self.local_stuck_regions) > 5:
                                self.status_label.config(text="检测到多处卡住区域，执行完整环境扫描...")
                                
                                # 执行360度扫描
                                original_heading = self.robot_heading
                                for angle in range(0, 360, 10):  # 每10度扫描一次
                                    self.robot_heading = math.radians(angle)
                                    scan_results = self.lidar_scan()
                                    self.update_map(scan_results)
                                self.robot_heading = original_heading
                        
                        # 重置状态
                        self.current_target = None
                        self.exploration_path = []
                        self.stuck_counter = 0
                        # 安排下一步探索后直接返回
                        self.root.after(30, self.exploration_step)
                        return
                    
                    # 先清空当前路径
                    self.exploration_path = []
                    # 尝试规划新路径，最多尝试2次(减少尝试次数以加快速度)
                    self.status_label.config(text="移动失败，重新规划路径...")
                    
                    for _ in range(2):
                        # 如果当前目标点靠近墙壁，略微调整目标位置
                        if self.current_target:
                            target_x, target_y = self.current_target
                            adjusted_target = (target_x, target_y)
                            
                            # 检查当前目标是否靠近墙壁
                            for wall in self.wall_segments:
                                if self.distance_point_to_segment(adjusted_target, wall[0], wall[1]) <= self.robot_radius * 2.5:
                                    # 目标太靠近墙壁，尝试调整
                                    adjusted_target = (target_x + random.uniform(-0.5, 0.5), 
                                                      target_y + random.uniform(-0.5, 0.5))
                                    break
                            
                            # 用调整后的目标规划路径
                            self.current_target = adjusted_target
                            self.exploration_path = self.plan_path(self.current_target)
                            
                            if self.exploration_path:
                                # 成功规划路径
                                self.status_label.config(text="路径重新规划成功")
                                self.visualize_planned_path(self.exploration_path)
                                break
                    
                    # 如果多次尝试后仍然无法规划路径
                    if not self.exploration_path:
                        self.status_label.config(text="无法规划有效路径，寻找新目标...")
                        self.current_target = None
                # 如果移动成功且到达当前路径点
                elif self.distance_to_point(next_point) < 0.3:
                    # 重置卡住计数器
                    self.stuck_counter = 0
                    self.exploration_path.pop(0)
                    # 更新路径可视化
                    if self.exploration_path:
                        self.visualize_planned_path(self.exploration_path)
                        self.path_label.config(text=f"剩余路径点: {len(self.exploration_path)}")
                    else:
                        # 如果路径已完成
                        if self.exploration_complete:
                            # 如果正在返回起点且已经到达
                            if self.returning_home and self.distance_to_point(self.start_point) < self.grid_size * 2:
                                self.status_label.config(text="探索完成！已成功返回起点")
                                self.is_exploring = False

                                return
                            # 如果到达了出口点
                            elif self.current_target in self.exit_points:
                                self.status_label.config(text=f"已到达出口点: ({self.current_target[0]:.1f}, {self.current_target[1]:.1f})")
                                # 如果已经找到出口但还需返回起点
                                if self.returning_home:
                                    # 规划到起点的路径
                                    self.current_target = self.start_point
                                    home_path = self.plan_path(self.start_point)
                                    if home_path:
                                        self.exploration_path = home_path
                                        self.status_label.config(text=f"已到达出口点，现在返回起点")
                                    else:
                                        self.status_label.config(text="已到达出口点，但无法规划回起点的路径")
                                        self.is_exploring = False
                                        return
                                else:
                                    # 如果只是要找出口，完成任务
                                    self.is_exploring = False
                                    return
                            else:
                                # 如果返回起点的路径已经完成，但实际位置还未到达起点，可能是因为路径规划精度问题
                                if self.returning_home and self.distance_to_point(self.start_point) < self.grid_size * 4:
                                    # 直接移动到起点
                                    if self.move_robot(self.start_point):
                                        self.status_label.config(text="探索完成！已成功返回起点")
                                        self.is_exploring = False
                                        return
                                    else:
                                        # 重新规划路径
                                        self.exploration_path = self.plan_path(self.start_point)
                                        if not self.exploration_path:
                                            self.status_label.config(text="无法返回起点，探索结束")
                                            self.is_exploring = False
                                            return
                                else:
                                    self.status_label.config(text="到达目标点，寻找新目标...")
                        else:
                            self.status_label.config(text="到达目标点，寻找新目标...")
            
            # 更新界面
            self.root.update()
            
            # 安排下一步探索，根据是否在返回起点调整延迟
            delay = 5 if self.returning_home else 10  # 返回起点时使用更小的延迟以加快速度
            delay = int(max(5, delay / self.speed_factor))  # 根据速度因子调整延迟
            self.root.after(delay, self.exploration_step)
        except Exception as e:
            print(f"探索步骤错误: {e}")
            # 尝试重新安排下一步探索
            if self.is_exploring:
                self.root.after(100, self.exploration_step)

    def distance_to_point(self, point):
        """计算到指定点的距离"""
        try:
            if not point:
                return float('inf')
                
            x, y = self.robot_position
            target_x, target_y = point
            return math.sqrt((x - target_x)**2 + (y - target_y)**2)
        except Exception as e:
            print(f"计算点距离错误: {e}")
            return float('inf')
    
    def start_exploration(self):
        """开始SLAM探索"""
        try:
            if self.is_exploring:
                return
            
            # 重置状态
            self.reset()
            
            # 开始探索
            self.is_exploring = True
            self.status_label.config(text="开始SLAM探索...")
            self.robot_status_label.config(text="状态: 正在探索地图", fg="green")
            
            # 执行第一步探索
            self.exploration_step()
        except Exception as e:
            print(f"开始探索错误: {e}")
            self.status_label.config(text=f"探索出错: {e}")
            self.robot_status_label.config(text="状态: 探索出错", fg="red")
    
    def reset(self):
        """重置状态"""
        try:
            self.is_exploring = False
            self.robot_position = self.start_point
            self.robot_heading = 0.0
            self.robot_trajectory = [self.robot_position]
            self.map_grid = np.full((self.grid_rows, self.grid_cols), -1)
            self.current_target = None
            self.exploration_path = []
            self.exploration_stuck_counter = 0
            self.last_position = self.start_point
            self.last_heading = 0.0
            self.position_history.clear()
            self.local_stuck_regions = set()
            self.exit_points = []
            self.has_found_exit = False
            self.exploration_complete = False  # 重置探索完成标志
            self.returning_home = False  # 重置返回起点标志
            self.frontier_points = []  # 重置前沿点列表
            
            # 清除存储的探索历史数据
            if hasattr(self, 'visit_heatmap'):
                self.visit_heatmap = np.zeros((self.grid_rows, self.grid_cols))
                self.global_exploration_count = 0
            if hasattr(self, 'explored_regions'):
                self.explored_regions = set()
            if hasattr(self, 'exit_found_time'):
                delattr(self, 'exit_found_time')
            if hasattr(self, 'stuck_counter'):
                self.stuck_counter = 0
            if hasattr(self, 'last_positions'):
                self.last_positions = deque(maxlen=10)
                
            # 清除路径相关
            if hasattr(self, 'path_lines') and self.path_lines:
                for line in self.path_lines:
                    self.canvas.delete(line)
                self.path_lines = []
                
            # 清除扫描线
            if hasattr(self, 'scan_lines') and self.scan_lines:
                for line in self.scan_lines:
                    self.canvas.delete(line)
                self.scan_lines = []
                
            # 清除机器人标记
            if self.robot_marker:
                self.canvas.delete(self.robot_marker)
                self.robot_marker = None
                
            # 清除画布并重绘
            self.canvas.delete("all")
            self.init_display()
            
            # 重置状态标签
            self.status_label.config(text="准备开始SLAM探索")
            self.robot_status_label.config(text="状态: 待命中", fg="blue")
            self.path_label.config(text="")
        except Exception as e:
            print(f"重置错误: {e}")
    
    def check_exploration_completion(self):
        """检查地图探索是否完成（基于每条json墙壁线段被SLAM墙格覆盖的比例）"""
        try:
            # 1. 统计json墙壁总长度
            total_length = 0
            covered_length = 0
            for seg in self.segments:
                x1, y1 = seg['start']
                x2, y2 = seg['end']
                seg_len = math.hypot(x2 - x1, y2 - y1)
                total_length += seg_len
                # 采样该线段上的点，判断是否被SLAM墙格覆盖
                sample_n = max(2, int(seg_len / self.grid_size * 2))
                covered_n = 0
                for i in range(sample_n + 1):
                    t = i / sample_n
                    sx = x1 + (x2 - x1) * t
                    sy = y1 + (y2 - y1) * t
                    grid_r = int(sx / self.grid_size)
                    grid_c = int(sy / self.grid_size)
                    if 0 <= grid_r < self.grid_rows and 0 <= grid_c < self.grid_cols:
                        if self.map_grid[grid_r, grid_c] == 1:
                            covered_n += 1
                covered_length += seg_len * (covered_n / (sample_n + 1))
            coverage_ratio = covered_length / total_length if total_length > 0 else 0
            if coverage_ratio >= self.wall_coverage_threshold:
                print(f"墙体覆盖率达到阈值: {coverage_ratio:.2%} >= {self.wall_coverage_threshold:.2%}")
                return True, coverage_ratio
            return False, coverage_ratio
        except Exception as e:
            print(f"检查探索完成度错误: {e}")
            return False, 0.0
            
    def update_speed(self, value):
        """更新机器人移动速度"""
        try:
            self.speed_factor = float(value)
            self.speed_value_label.config(text=f"速度: {self.speed_factor}x")
        except Exception as e:
            print(f"更新速度错误: {e}")
    
    def run(self):
        try:
            self.root.mainloop()
        except Exception as e:
            print(f"运行错误: {e}")
    
    def emergency_escape(self):
        """紧急逃脱函数，当机器人卡住时执行"""
        try:
            self.status_label.config(text="执行紧急逃脱程序...")
            current_x, current_y = self.robot_position
            
            # 获取当前已知的空闲空间
            free_spaces = []
            for r in range(self.grid_rows):
                for c in range(self.grid_cols):
                    if self.map_grid[r, c] == 0:  # 空闲区域
                        # 转换为实际坐标
                        space_x = r * self.grid_size
                        space_y = c * self.grid_size
                        
                        # 计算到当前位置的距离
                        dist = math.sqrt((space_x - current_x)**2 + (space_y - current_y)**2)
                        
                        # 只考虑一定范围内的空闲区域
                        if 1.0 <= dist <= 5.0:
                            # 检查该位置是否安全（不靠近墙体）
                            is_safe = True
                            for wall in self.wall_segments:
                                if self.distance_point_to_segment((space_x, space_y), wall[0], wall[1]) <= self.robot_radius * 1.5:
                                    is_safe = False
                                    break
                            
                            if is_safe:
                                free_spaces.append((space_x, space_y, dist))
            
            # 按距离对空闲空间进行排序
            free_spaces.sort(key=lambda x: x[2])
            
            # 尝试前往最近的几个空闲空间
            for space_x, space_y, _ in free_spaces[:10]:  # 尝试最近的10个点
                # 直接设置机器人位置，跳过正常移动逻辑
                self.robot_position = (space_x, space_y)
                if self.robot_marker:
                    self.canvas.delete(self.robot_marker)
                self.robot_marker = self.draw_robot()
                
                # 执行雷达扫描
                scan_results = self.lidar_scan()
                self.update_map(scan_results)
                
                self.status_label.config(text=f"紧急传送到: ({space_x:.1f}, {space_y:.1f})")
                return True
            
            # 如果没有找到合适的空闲空间，随机移动一段距离
            if not free_spaces:
                for angle in range(0, 360, 45):
                    rad_angle = math.radians(angle)
                    dx = math.cos(rad_angle) * 2.0  # 尝试移动2个单位
                    dy = math.sin(rad_angle) * 2.0
                    new_x = current_x + dx
                    new_y = current_y + dy
                    
                    # 检查新位置是否安全
                    if self.is_within_map_boundary((new_x, new_y)):
                        is_safe = True
                        for wall in self.wall_segments:
                            if self.distance_point_to_segment((new_x, new_y), wall[0], wall[1]) <= self.robot_radius * 1.5:
                                is_safe = False
                                break
                        
                        if is_safe:
                            # 直接设置位置
                            self.robot_position = (new_x, new_y)
                            self.robot_heading = rad_angle
                            if self.robot_marker:
                                self.canvas.delete(self.robot_marker)
                            self.robot_marker = self.draw_robot()
                            self.status_label.config(text=f"紧急移动到方向: {angle}°")
                            return True
            
            # 如果所有尝试都失败，返回失败
            self.status_label.config(text="紧急逃脱失败，尝试重新规划...")
            return False
            
        except Exception as e:
            print(f"紧急逃脱错误: {e}")
            return False

    def is_within_map_boundary(self, position):
        """检查位置是否在地图边界内和有效边界内"""
        x, y = position
        
        # 首先检查是否在地图的整体边界内（增加余量以确保不会超出边界）
        margin = 0.5
        in_map = -margin <= x <= self.max_row + margin and -margin <= y <= self.max_col + margin
        
        # 如果已定义有效边界，再进行更精确的检查
        if hasattr(self, 'valid_map_bounds'):
            min_valid_x, max_valid_x, min_valid_y, max_valid_y = self.valid_map_bounds
            # 给边界增加一些余量，确保机器人不会因为微小的计算误差而被判定为出界
            buffer = 0.5
            in_valid_bounds = (min_valid_x - buffer) <= x <= (max_valid_x + buffer) and \
                             (min_valid_y - buffer) <= y <= (max_valid_y + buffer)
            return in_map and in_valid_bounds
        
        return in_map

    def move_robot_along_path(self):
        """沿着规划的路径移动机器人"""
        try:
            if not self.exploration_path:
                return False
                
            # 获取当前位置和下一个路径点
            current_position = self.robot_position
            next_point = self.exploration_path[0]
            
            # 计算从当前位置到下一个点的距离
            distance = self.distance_between_points(current_position, next_point)
            
            # 如果已经非常接近下一个点，移除它并继续下一个点
            if distance < 0.3:
                self.exploration_path.pop(0)
                # 如果还有路径点，更新可视化
                if self.exploration_path:
                    self.visualize_planned_path(self.exploration_path)
                    self.path_label.config(text=f"剩余路径点: {len(self.exploration_path)}")
                    return self.move_robot_along_path()  # 递归调用处理下一个点
                return True  # 路径结束
            
            # 移动机器人向下一个点
            success = self.move_robot(next_point)
            
            # 返回移动是否成功
            return success
            
        except Exception as e:
            print(f"沿路径移动错误: {e}")
            return False

    def is_point_on_segment(self, pt, seg_start, seg_end):
        """判断整数点pt是否在seg_start到seg_end的线段上（含端点）"""
        x, y = pt
        x1, y1 = seg_start
        x2, y2 = seg_end
        # 先判断是否共线
        cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
        if cross != 0:
            return False
        # 再判断是否在端点之间
        if min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2):
            return True
        return False

    def find_exits_from_segments(self):
        """修正：遍历边界所有整数点，只有没有被任何线段覆盖的边界点（且不是起点）才是出口"""
        all_points = set()
        for seg in self.segments:
            all_points.add(tuple(seg['start']))
            all_points.add(tuple(seg['end']))
        min_row = min(p[0] for p in all_points)
        max_row = max(p[0] for p in all_points)
        min_col = min(p[1] for p in all_points)
        max_col = max(p[1] for p in all_points)
        exits = []
        # 上下边界
        for c in range(min_col, max_col+1):
            for r in [min_row, max_row]:
                pt = (r, c)
                if pt == tuple(self.start_point):
                    continue
                # 检查是否被任何线段覆盖
                covered = False
                for seg in self.segments:
                    if self.is_point_on_segment(pt, tuple(seg['start']), tuple(seg['end'])):
                        covered = True
                        break
                if not covered:
                    exits.append(pt)
        # 左右边界
        for r in range(min_row+1, max_row):
            for c in [min_col, max_col]:
                pt = (r, c)
                if pt == tuple(self.start_point):
                    continue
                covered = False
                for seg in self.segments:
                    if self.is_point_on_segment(pt, tuple(seg['start']), tuple(seg['end'])):
                        covered = True
                        break
                if not covered:
                    exits.append(pt)
        self.segment_exits = exits
        return exits

    def mark_segment_exits(self):
        """在画布上标记用segments分析得到的出口"""
        if not hasattr(self, 'segment_exits'):
            self.find_exits_from_segments()
        for p in self.segment_exits:
            x = p[1] * self.cell_size + self.margin
            y = p[0] * self.cell_size + self.margin
            # 用紫色大圆圈标记
            self.canvas.create_oval(x-10, y-10, x+10, y+10, fill="purple", outline="yellow", width=3)

    def move_to_exit_only(self):
        """只执行路径移动到出口，不再进行任何探索、扫描等操作"""
        if not self.exploration_path or not self.current_target:
            return
        if self.distance_to_point(self.current_target) < self.grid_size * 2:
            self.status_label.config(text=f"已到达出口: {self.current_target}")
            return
        # 只移动到下一个路径点
        next_point = self.exploration_path[0]
        success = self.move_robot(next_point)
        if success and self.distance_to_point(next_point) < 0.3:
            self.exploration_path.pop(0)
        self.root.after(30, self.move_to_exit_only)

if __name__ == "__main__":
    try:
        maze_explorer = MazeSLAMExplorer("3.json")
        maze_explorer.run()
    except Exception as e:
        print(f"主程序错误: {e}") 