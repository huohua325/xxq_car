"""
实时地图可视化模块
支持占据栅格地图、机器人位姿、雷达数据、路径等的实时显示
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from typing import Tuple, List, Optional, Callable
from collections import deque
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
import config


class MapVisualizer:
    """实时地图可视化器
    
    功能：
    - 显示占据栅格地图（灰度图）
    - 显示机器人位置和朝向
    - 显示雷达扫描点云
    - 显示规划路径
    - 显示前沿点
    - 显示机器人轨迹
    - 支持鼠标交互
    - 支持键盘快捷键
    
    Attributes:
        map_obj: OccupancyGridMap对象
        fig: matplotlib图形对象
        ax_map: 地图子图
        
    Example:
        >>> from src.slam.occupancy_map import OccupancyGridMap
        >>> map_obj = OccupancyGridMap()
        >>> visualizer = MapVisualizer(map_obj)
        >>> visualizer.update(robot_pose=(0, 0, 0))
        >>> plt.show()
    """
    
    def __init__(self, map_obj, 
                 figsize: Optional[Tuple[int, int]] = None,
                 enable_multi_window: Optional[bool] = None):
        """初始化可视化器
        
        Args:
            map_obj: OccupancyGridMap对象
            figsize: 图形大小（宽, 高）单位英寸，None则使用config.VISUALIZE_WINDOW_SIZE
            enable_multi_window: 是否启用多窗口布局，None则使用config.VISUALIZE_MULTI_WINDOW
        """
        self.map_obj = map_obj
        
        # ✅ 从config读取参数（允许外部覆盖）
        if figsize is None:
            figsize = config.VISUALIZE_WINDOW_SIZE
        if enable_multi_window is None:
            enable_multi_window = config.VISUALIZE_MULTI_WINDOW
        
        self.enable_multi_window = enable_multi_window
        
        # 创建图形窗口
        if enable_multi_window:
            self.fig, self.axes = plt.subplots(2, 2, figsize=figsize)
            self.ax_map = self.axes[0, 0]
            self.ax_radar = self.axes[0, 1]
            self.ax_velocity = self.axes[1, 0]
            self.ax_pose = self.axes[1, 1]
        else:
            self.fig, self.ax_map = plt.subplots(figsize=figsize)
            self.axes = None
        
        # 设置标题
        self.fig.suptitle('xxq Robot - Real-time SLAM Visualization System', fontsize=16, fontweight='bold')
        
        # ✅ 配置（从config读取）
        self.show_lidar = config.SHOW_LIDAR_POINTS
        self.show_path = config.SHOW_PATH
        self.show_frontiers = config.SHOW_FRONTIERS
        self.show_trajectory = config.SHOW_TRAJECTORY
        self.show_grid = False  # 网格线（默认关闭，影响性能）
        
        # 地图显示
        self.img_map = None
        self._init_map_display()
        
        # 机器人显示
        self.robot_marker = None
        self.robot_arrow = None
        
        # 雷达点云显示
        self.lidar_scatter = None
        
        # 路径显示
        self.path_line = None
        
        # 前沿点显示
        self.frontier_scatter = None
        
        # ✅ 轨迹显示（从config读取缓冲区大小）
        self.trajectory_line = None
        self.trajectory_history = deque(maxlen=config.TRAJECTORY_HISTORY_SIZE)
        
        # ✅ 速度历史（用于多窗口模式，从config读取）
        self.velocity_history = deque(maxlen=config.VELOCITY_HISTORY_SIZE)
        
        # ✅ 位姿历史（从config读取）
        self.pose_history = deque(maxlen=config.POSE_HISTORY_SIZE)
        
        # 交互回调
        self.on_click_callback: Optional[Callable] = None
        self._setup_interaction()
        
        # 状态栏文本
        self.status_text = None
        self._setup_status_bar()
        
        # 性能统计
        self.frame_count = 0
        self.last_update_time = None
        
        print("[可视化] MapVisualizer初始化完成")
    
    def _init_map_display(self):
        """初始化地图显示"""
        # 显示地图（使用gray_r颜色映射：白=空闲，黑=占用，灰=未知）
        self.img_map = self.ax_map.imshow(
            self.map_obj.grid,
            cmap='gray_r',
            vmin=0.0,
            vmax=1.0,
            origin='lower',
            interpolation='nearest'
        )
        
        # 添加颜色条
        cbar = self.fig.colorbar(self.img_map, ax=self.ax_map, fraction=0.046, pad=0.04)
        cbar.set_label('Occupancy Probability', rotation=270, labelpad=15)
        
        # 设置坐标轴（世界坐标）
        self.ax_map.set_xlabel('X (meters)', fontsize=10)
        self.ax_map.set_ylabel('Y (meters)', fontsize=10)
        self.ax_map.set_title('Occupancy Grid Map', fontsize=12, fontweight='bold')
        
        # 设置刻度（转换为世界坐标）
        self._update_map_ticks()
        
        # 图例（稍后添加元素后再绘制）
        self.legend_handles = []
    
    def _update_map_ticks(self):
        """更新坐标轴刻度为世界坐标"""
        # 栅格坐标范围
        width, height = self.map_obj.config.width, self.map_obj.config.height
        
        # 生成刻度位置（每隔50格）
        tick_step = 50
        x_ticks = np.arange(0, width, tick_step)
        y_ticks = np.arange(0, height, tick_step)
        
        # 转换为世界坐标标签
        x_labels = [(x - self.map_obj.config.origin_x) * self.map_obj.config.resolution 
                    for x in x_ticks]
        y_labels = [(y - self.map_obj.config.origin_y) * self.map_obj.config.resolution 
                    for y in y_ticks]
        
        self.ax_map.set_xticks(x_ticks)
        self.ax_map.set_yticks(y_ticks)
        self.ax_map.set_xticklabels([f'{x:.1f}' for x in x_labels], fontsize=8)
        self.ax_map.set_yticklabels([f'{y:.1f}' for y in y_labels], fontsize=8)
        
        # 网格线
        if self.show_grid:
            self.ax_map.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    
    def _setup_interaction(self):
        """设置交互功能"""
        # 鼠标点击事件
        self.fig.canvas.mpl_connect('button_press_event', self._on_mouse_click)
        
        # 键盘事件
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)
    
    def _setup_status_bar(self):
        """设置状态栏"""
        # 在地图下方添加状态文本
        self.status_text = self.fig.text(
            0.02, 0.02, '', 
            fontsize=9, 
            family='monospace',
            verticalalignment='bottom',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        )
    
    def _on_mouse_click(self, event):
        """鼠标点击事件处理"""
        if event.inaxes == self.ax_map and event.button == 1:  # 左键
            # 获取点击的栅格坐标
            grid_x = int(event.xdata)
            grid_y = int(event.ydata)
            
            # 转换为世界坐标
            world_x, world_y = self.map_obj.grid_to_world(grid_x, grid_y)
            
            print(f"[交互] 点击位置: 栅格({grid_x}, {grid_y}), 世界({world_x:.2f}, {world_y:.2f})")
            
            # 调用回调函数
            if self.on_click_callback:
                self.on_click_callback(world_x, world_y)
    
    def _on_key_press(self, event):
        """键盘按键事件处理"""
        if event.key == 's':
            # 保存地图
            filename = f'data/maps/map_{self.frame_count:04d}.png'
            self.map_obj.save_map(filename)
            print(f"[交互] 地图已保存: {filename}")
        
        elif event.key == 'r':
            # 重置视图
            self.ax_map.set_xlim(0, self.map_obj.config.width)
            self.ax_map.set_ylim(0, self.map_obj.config.height)
            print("[交互] 视图已重置")
        
        elif event.key == 'g':
            # 切换网格显示
            self.show_grid = not self.show_grid
            if self.show_grid:
                self.ax_map.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
            else:
                self.ax_map.grid(False)
            print(f"[交互] 网格显示: {self.show_grid}")
        
        elif event.key == 'l':
            # 切换雷达点云显示
            self.show_lidar = not self.show_lidar
            print(f"[交互] 雷达显示: {self.show_lidar}")
        
        elif event.key == 't':
            # 切换轨迹显示
            self.show_trajectory = not self.show_trajectory
            print(f"[交互] 轨迹显示: {self.show_trajectory}")
        
        elif event.key == 'h':
            # 显示帮助
            help_text = """
快捷键说明：
  s - 保存地图
  r - 重置视图
  g - 切换网格显示
  l - 切换雷达点云显示
  t - 切换轨迹显示
  h - 显示帮助
  q - 退出
            """
            print(help_text)
        
        elif event.key == 'q':
            # 退出
            plt.close(self.fig)
            print("[交互] 退出可视化")
        
        self.fig.canvas.draw()
    
    def update(self, 
               robot_pose: Optional[Tuple[float, float, float]] = None,
               lidar_points: Optional[List[Tuple[float, float]]] = None,
               path: Optional[List[Tuple[float, float]]] = None,
               frontiers: Optional[List[Tuple[float, float]]] = None,
               velocity: Optional[Tuple[float, float]] = None):
        """更新显示
        
        Args:
            robot_pose: 机器人位姿 (x, y, theta)
            lidar_points: 雷达点云列表 [(x1, y1), (x2, y2), ...]
            path: 规划路径 [(x1, y1), (x2, y2), ...]
            frontiers: 前沿点列表 [(x1, y1), (x2, y2), ...]
            velocity: 机器人速度 (v, omega)
        """
        # 更新地图
        self.img_map.set_data(self.map_obj.grid)
        
        # 更新机器人位姿
        if robot_pose is not None:
            self._update_robot(robot_pose)
            self.pose_history.append(robot_pose)
        
        # 更新雷达点云
        if lidar_points is not None and self.show_lidar:
            self._update_lidar(lidar_points)
        
        # 更新路径
        if path is not None and self.show_path:
            self._update_path(path)
        
        # 更新前沿点
        if frontiers is not None and self.show_frontiers:
            self._update_frontiers(frontiers)
        
        # 更新轨迹
        if robot_pose is not None and self.show_trajectory:
            self._update_trajectory(robot_pose)
        
        # 更新速度历史（多窗口模式）
        if velocity is not None and self.enable_multi_window:
            self.velocity_history.append(velocity)
            self._update_velocity_plot()
        
        # 更新状态栏
        self._update_status_bar(robot_pose, velocity)
        
        # 刷新画布
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        self.frame_count += 1
    
    def _update_robot(self, pose: Tuple[float, float, float]):
        """更新机器人显示
        
        Args:
            pose: (x, y, theta) 位姿
        """
        x, y, theta = pose
        gx, gy = self.map_obj.world_to_grid(x, y)
        
        # 删除旧的标记
        if self.robot_marker is not None:
            self.robot_marker.remove()
        if self.robot_arrow is not None:
            self.robot_arrow.remove()
        
        # 绘制机器人位置（蓝色圆圈）
        self.robot_marker = self.ax_map.scatter(
            [gx], [gy],
            c='blue',
            s=200,
            marker='o',
            edgecolors='black',
            linewidths=2,
            zorder=10,
            label='Robot'
        )
        
        # 绘制朝向箭头
        arrow_length = 20  # 栅格单位
        dx = arrow_length * np.cos(theta)
        dy = arrow_length * np.sin(theta)
        
        self.robot_arrow = self.ax_map.arrow(
            gx, gy, dx, dy,
            head_width=8,
            head_length=10,
            fc='red',
            ec='black',
            linewidth=2,
            zorder=11
        )
    
    def _update_lidar(self, points: List[Tuple[float, float]]):
        """更新雷达点云显示
        
        Args:
            points: 点云列表 [(x1, y1), ...]
        """
        if not points:
            return
        
        # 转换为栅格坐标
        grid_points = [self.map_obj.world_to_grid(x, y) for x, y in points]
        gx_list = [p[0] for p in grid_points]
        gy_list = [p[1] for p in grid_points]
        
        # 删除旧的点云
        if self.lidar_scatter is not None:
            self.lidar_scatter.remove()
        
        # 绘制雷达点云（红色小点）
        self.lidar_scatter = self.ax_map.scatter(
            gx_list, gy_list,
            c='red',
            s=10,
            marker='.',
            alpha=0.6,
            zorder=5,
            label='Lidar Points'
        )
    
    def _update_path(self, path: List[Tuple[float, float]]):
        """更新路径显示
        
        Args:
            path: 路径点列表 [(x1, y1), ...]
        """
        if not path:
            return
        
        # 转换为栅格坐标
        grid_path = [self.map_obj.world_to_grid(x, y) for x, y in path]
        gx_list = [p[0] for p in grid_path]
        gy_list = [p[1] for p in grid_path]
        
        # 删除旧的路径
        if self.path_line is not None:
            self.path_line.remove()
        
        # 绘制路径（绿色线）
        self.path_line, = self.ax_map.plot(
            gx_list, gy_list,
            'g-',
            linewidth=2,
            marker='o',
            markersize=4,
            alpha=0.8,
            zorder=7,
            label='Planned Path'
        )
    
    def _update_frontiers(self, frontiers: List[Tuple[float, float]]):
        """更新前沿点显示
        
        Args:
            frontiers: 前沿点列表 [(x1, y1), ...]
        """
        if not frontiers:
            return
        
        # 转换为栅格坐标
        grid_frontiers = [self.map_obj.world_to_grid(x, y) for x, y in frontiers]
        gx_list = [p[0] for p in grid_frontiers]
        gy_list = [p[1] for p in grid_frontiers]
        
        # 删除旧的前沿点
        if self.frontier_scatter is not None:
            self.frontier_scatter.remove()
        
        # 绘制前沿点（绿色叉）
        self.frontier_scatter = self.ax_map.scatter(
            gx_list, gy_list,
            c='lime',
            s=100,
            marker='X',
            edgecolors='black',
            linewidths=1.5,
            zorder=8,
            label='Frontiers'
        )
    
    def _update_trajectory(self, pose: Tuple[float, float, float]):
        """更新轨迹显示
        
        Args:
            pose: 当前位姿 (x, y, theta)
        """
        x, y, _ = pose
        self.trajectory_history.append((x, y))
        
        if len(self.trajectory_history) < 2:
            return
        
        # 转换为栅格坐标
        grid_traj = [self.map_obj.world_to_grid(x, y) for x, y in self.trajectory_history]
        gx_list = [p[0] for p in grid_traj]
        gy_list = [p[1] for p in grid_traj]
        
        # 删除旧的轨迹线
        if self.trajectory_line is not None:
            self.trajectory_line.remove()
        
        # 绘制轨迹（黄色虚线）
        self.trajectory_line, = self.ax_map.plot(
            gx_list, gy_list,
            'y--',
            linewidth=1.5,
            alpha=0.7,
            zorder=6,
            label='Trajectory'
        )
    
    def _update_velocity_plot(self):
        """更新速度曲线（多窗口模式）"""
        if not self.enable_multi_window or len(self.velocity_history) == 0:
            return
        
        # 提取速度数据
        v_list = [v[0] for v in self.velocity_history]
        omega_list = [v[1] for v in self.velocity_history]
        time_list = list(range(len(v_list)))
        
        # 清空子图
        self.ax_velocity.clear()
        
        # 绘制速度曲线
        self.ax_velocity.plot(time_list, v_list, 'b-', label='Linear Velocity (m/s)', linewidth=2)
        self.ax_velocity.plot(time_list, omega_list, 'r-', label='Angular Velocity (rad/s)', linewidth=2)
        
        self.ax_velocity.set_xlabel('Time Steps')
        self.ax_velocity.set_ylabel('Velocity')
        self.ax_velocity.set_title('Velocity Profile')
        self.ax_velocity.legend(loc='upper right')
        self.ax_velocity.grid(True, alpha=0.3)
    
    def _update_status_bar(self, 
                          robot_pose: Optional[Tuple[float, float, float]] = None,
                          velocity: Optional[Tuple[float, float]] = None):
        """更新状态栏
        
        Args:
            robot_pose: 机器人位姿
            velocity: 机器人速度
        """
        # 获取地图统计
        stats = self.map_obj.get_statistics()
        
        # 构建状态文本
        status_lines = [
            f"Frame: {self.frame_count}",
            f"Explored: {stats['explored_ratio']*100:.1f}%",
            f"Occupied: {stats['occupied_cells']}",
        ]
        
        if robot_pose:
            x, y, theta = robot_pose
            status_lines.append(f"Pose: ({x:.2f}, {y:.2f}, {np.rad2deg(theta):.1f}deg)")
        
        if velocity:
            v, omega = velocity
            status_lines.append(f"Vel: v={v:.2f}m/s, w={omega:.2f}rad/s")
        
        status_text = " | ".join(status_lines)
        self.status_text.set_text(status_text)
    
    def set_click_callback(self, callback: Callable[[float, float], None]):
        """设置鼠标点击回调函数
        
        Args:
            callback: 回调函数 callback(x, y)
        """
        self.on_click_callback = callback
    
    def show(self):
        """显示窗口（阻塞模式）"""
        plt.show()
    
    def close(self):
        """关闭窗口"""
        plt.close(self.fig)
    
    def save_figure(self, filename: str):
        """保存当前图形
        
        Args:
            filename: 文件路径
        """
        from pathlib import Path
        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"[可视化] 图形已保存: {filename}")


class AnimatedVisualizer(MapVisualizer):
    """动画可视化器（自动刷新）
    
    使用matplotlib的FuncAnimation实现自动刷新
    
    Example:
        >>> visualizer = AnimatedVisualizer(map_obj, update_func)
        >>> visualizer.start(interval=100)  # 100ms刷新一次
        >>> plt.show()
    """
    
    def __init__(self, map_obj, update_func: Callable, **kwargs):
        """初始化动画可视化器
        
        Args:
            map_obj: OccupancyGridMap对象
            update_func: 更新函数，返回 (robot_pose, lidar_points, path, frontiers, velocity)
        """
        super().__init__(map_obj, **kwargs)
        self.update_func = update_func
        self.animation = None
    
    def _animation_update(self, frame):
        """动画更新函数"""
        # 调用用户提供的更新函数
        data = self.update_func()
        
        if data is None:
            return
        
        # 解包数据
        robot_pose = data.get('robot_pose')
        lidar_points = data.get('lidar_points')
        path = data.get('path')
        frontiers = data.get('frontiers')
        velocity = data.get('velocity')
        
        # 更新显示
        self.update(robot_pose, lidar_points, path, frontiers, velocity)
    
    def start(self, interval: Optional[int] = None):
        """启动动画
        
        Args:
            interval: 刷新间隔（毫秒），None则根据config.VISUALIZE_RATE自动计算
        """
        # ✅ 从config计算刷新间隔
        if interval is None:
            interval = int(1000 / config.VISUALIZE_RATE)  # fps -> ms
        
        self.animation = FuncAnimation(
            self.fig,
            self._animation_update,
            interval=interval,
            blit=False
        )
        print(f"[可视化] 动画已启动，刷新间隔: {interval}ms ({config.VISUALIZE_RATE}fps)")
    
    def stop(self):
        """停止动画"""
        if self.animation:
            self.animation.event_source.stop()
            print("[可视化] 动画已停止")
