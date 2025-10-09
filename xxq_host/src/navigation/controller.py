"""
主控制器模块
集成SLAM、Frontier探索、路径规划和DWA避障，实现完整的自主探索
"""

import numpy as np
from typing import Tuple, List, Optional
from enum import Enum
import time


class RobotState(Enum):
    """机器人状态枚举"""
    IDLE = 0          # 空闲
    EXPLORING = 1     # 探索中
    NAVIGATING = 2    # 导航中
    STUCK = 3         # 卡住
    COMPLETED = 4     # 完成


class RobotController:
    """主控制器
    
    集成所有模块，实现完整的自主探索流程：
    1. 更新地图（SLAM）
    2. 检测前沿点（Frontier）
    3. 规划全局路径（A*）
    4. 局部避障导航（DWA）
    5. 实时可视化
    
    Attributes:
        map_obj: OccupancyGridMap对象
        frontier_detector: FrontierDetector对象
        path_planner: PathPlanner对象
        dwa: DWA对象
        visualizer: MapVisualizer对象（可选）
        
    Example:
        >>> controller = RobotController(comm)
        >>> controller.run()
    """
    
    def __init__(self, 
                 comm=None,
                 map_obj=None,
                 visualizer=None,
                 enable_visualization=True):
        """初始化控制器
        
        Args:
            comm: RobotComm通信对象（可选）
            map_obj: OccupancyGridMap对象（可选，None则自动创建）
            visualizer: MapVisualizer对象（可选）
            enable_visualization: 是否启用可视化
        """
        # 通信模块
        self.comm = comm
        
        # SLAM模块
        if map_obj is None:
            from src.slam.occupancy_map import OccupancyGridMap, MapConfig
            config = MapConfig(width=200, height=200, resolution=0.1)
            self.map = OccupancyGridMap(config)
        else:
            self.map = map_obj
        
        # Frontier探索模块
        from src.slam.frontier_detector import FrontierDetector
        self.frontier_detector = FrontierDetector(
            self.map, 
            min_frontier_size=5,
            cluster_distance=0.5
        )
        
        # 路径规划模块
        from src.navigation.path_planner import PathPlanner, PathPlannerConfig
        planner_config = PathPlannerConfig(
            obstacle_threshold=0.7,
            inflation_radius=2,
            allow_diagonal=True
        )
        self.path_planner = PathPlanner(self.map, planner_config)
        
        # DWA避障模块
        from src.navigation.dwa import DWA, DWAConfig
        dwa_config = DWAConfig(
            max_speed=1.0,
            max_yaw_rate=np.deg2rad(40.0),
            predict_time=2.0
        )
        self.dwa = DWA(dwa_config)
        
        # 可视化模块
        self.enable_visualization = enable_visualization
        self.visualizer = visualizer
        
        # 机器人状态
        self.robot_pose = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.robot_velocity = [0.0, 0.0]   # [v, omega]
        self.state = RobotState.IDLE
        
        # 导航状态
        self.current_target = None
        self.current_path = []
        self.path_index = 0
        
        # 卡住检测
        self.stuck_counter = 0
        self.stuck_threshold = 50  # 50次迭代未移动则判定卡住
        self.last_pose = [0.0, 0.0, 0.0]
        
        # 统计信息
        self.exploration_steps = 0
        self.total_distance = 0.0
        self.start_time = None
        
        print("[Controller] 主控制器初始化完成")
    
    def run_exploration(self, max_steps=500, save_map=True):
        """运行完整探索流程
        
        Args:
            max_steps: 最大探索步数
            save_map: 是否保存地图
            
        Returns:
            是否成功完成探索
        """
        print("\n" + "="*60)
        print("   开始自主探索")
        print("="*60 + "\n")
        
        self.start_time = time.time()
        self.state = RobotState.EXPLORING
        
        for step in range(max_steps):
            self.exploration_steps = step + 1
            
            # 主控制循环
            success = self._control_step()
            
            # 检查是否完成
            if self.state == RobotState.COMPLETED:
                print(f"\n✅ 探索完成！总步数: {step + 1}")
                break
            
            # 检查是否卡住
            if self.state == RobotState.STUCK:
                print(f"\n⚠️  机器人卡住，尝试恢复...")
                self._handle_stuck()
            
            # 定期输出进度
            if step % 50 == 0:
                self._print_progress()
        else:
            print(f"\n⚠️  达到最大步数 {max_steps}，探索未完成")
        
        # 保存地图
        if save_map:
            self._save_final_map()
        
        # 输出统计信息
        self._print_statistics()
        
        return self.state == RobotState.COMPLETED
    
    def _control_step(self) -> bool:
        """单步控制循环
        
        Returns:
            是否成功
        """
        # 1. 检测前沿点
        frontiers = self.frontier_detector.find_frontiers()
        
        if not frontiers:
            print("[Controller] 未找到前沿点，探索完成")
            self.state = RobotState.COMPLETED
            return True
        
        # 2. 选择目标
        if self.current_target is None:
            self.current_target = self.frontier_detector.select_best_frontier(
                frontiers, 
                tuple(self.robot_pose),
                strategy='nearest'
            )
            print(f"[Controller] 新目标: ({self.current_target[0]:.2f}, {self.current_target[1]:.2f})")
        
        # 3. 规划路径（如果还没有路径）
        if not self.current_path:
            self.current_path = self.path_planner.plan_path(
                tuple(self.robot_pose[:2]),
                self.current_target,
                inflate_obstacles=True,
                smooth=True
            )
            
            if not self.current_path:
                print("[Controller] 路径规划失败，重新选择目标")
                self.current_target = None
                return False
            
            self.path_index = 0
        
        # 4. 跟随路径（DWA）
        if self.path_index < len(self.current_path):
            local_goal = self.current_path[self.path_index]
            
            # 检查是否到达当前路径点
            dist = np.hypot(
                self.robot_pose[0] - local_goal[0],
                self.robot_pose[1] - local_goal[1]
            )
            
            if dist < 0.3:  # 到达阈值
                self.path_index += 1
                if self.path_index >= len(self.current_path):
                    # 到达目标
                    print(f"[Controller] 到达目标")
                    self.current_target = None
                    self.current_path = []
                    self.path_index = 0
                    return True
            
            # DWA规划
            robot_state = tuple(self.robot_pose + self.robot_velocity)
            v, omega = self.dwa.plan(robot_state, local_goal, obstacles=[])
            
            # 更新机器人状态（简化的运动学模型）
            self._update_robot_state(v, omega, dt=0.1)
        
        # 5. 卡住检测
        self._check_stuck()
        
        # 6. 可视化
        if self.enable_visualization and self.visualizer:
            self._update_visualization(frontiers)
        
        return True
    
    def _update_robot_state(self, v: float, omega: float, dt: float = 0.1):
        """更新机器人状态
        
        Args:
            v: 线速度
            omega: 角速度
            dt: 时间步长
        """
        # 保存上一次位姿
        self.last_pose = self.robot_pose.copy()
        
        # 更新位姿
        self.robot_pose[0] += v * np.cos(self.robot_pose[2]) * dt
        self.robot_pose[1] += v * np.sin(self.robot_pose[2]) * dt
        self.robot_pose[2] += omega * dt
        self.robot_pose[2] = np.arctan2(np.sin(self.robot_pose[2]), np.cos(self.robot_pose[2]))
        
        # 更新速度
        self.robot_velocity = [v, omega]
        
        # 更新总距离
        self.total_distance += v * dt
    
    def _check_stuck(self):
        """检测机器人是否卡住"""
        # 计算位移
        displacement = np.hypot(
            self.robot_pose[0] - self.last_pose[0],
            self.robot_pose[1] - self.last_pose[1]
        )
        
        if displacement < 0.01:  # 几乎没有移动
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        
        if self.stuck_counter >= self.stuck_threshold:
            self.state = RobotState.STUCK
    
    def _handle_stuck(self):
        """处理卡住情况"""
        # 简单的恢复策略：后退 + 转向
        print("[Controller] 执行恢复动作...")
        
        # 后退
        for _ in range(10):
            self._update_robot_state(-0.5, 0.0, dt=0.1)
        
        # 随机转向
        turn_angle = np.random.uniform(-np.pi/2, np.pi/2)
        for _ in range(10):
            self._update_robot_state(0.0, turn_angle/10, dt=0.1)
        
        # 重置状态
        self.stuck_counter = 0
        self.current_target = None
        self.current_path = []
        self.state = RobotState.EXPLORING
        
        print("[Controller] 恢复完成，继续探索")
    
    def _update_visualization(self, frontiers):
        """更新可视化
        
        Args:
            frontiers: 前沿点列表
        """
        if self.visualizer:
            self.visualizer.update(
                robot_pose=tuple(self.robot_pose),
                path=self.current_path,
                frontiers=frontiers
            )
    
    def _print_progress(self):
        """打印进度信息"""
        elapsed = time.time() - self.start_time if self.start_time else 0
        stats = self.map.get_statistics()
        
        print(f"\n[进度] 步数: {self.exploration_steps}, "
              f"时间: {elapsed:.1f}s, "
              f"探索率: {stats['explored_ratio']*100:.1f}%, "
              f"距离: {self.total_distance:.1f}m")
    
    def _save_final_map(self):
        """保存最终地图"""
        import os
        os.makedirs('data/maps', exist_ok=True)
        
        filename = f'data/maps/exploration_{int(time.time())}.pkl'
        self.map.save_map(filename)
        print(f"[保存] 地图保存到: {filename}")
    
    def _print_statistics(self):
        """打印统计信息"""
        elapsed = time.time() - self.start_time if self.start_time else 0
        stats = self.map.get_statistics()
        
        print("\n" + "="*60)
        print("   探索统计")
        print("="*60)
        print(f"总步数: {self.exploration_steps}")
        print(f"总时间: {elapsed:.1f}s")
        print(f"总距离: {self.total_distance:.1f}m")
        print(f"探索率: {stats['explored_ratio']*100:.1f}%")
        print(f"障碍物数: {stats['obstacle_cells']}")
        print(f"空闲区域: {stats['free_cells']}")
        print("="*60 + "\n")
    
    def simulate_lidar_scan(self):
        """模拟雷达扫描（用于测试）
        
        Returns:
            模拟的雷达数据
        """
        class MockLidarData:
            def __init__(self):
                self.sectors = []
                for angle_deg in [0, 45, 90, 135, 180, 225, 270, 315]:
                    self.sectors.append({
                        'angle_center': angle_deg,
                        'count': 10,
                        'min_dist': 3.0,
                        'avg_dist': 3.0
                    })
        
        return MockLidarData()
    
    def update_map_from_lidar(self, lidar_data=None):
        """从雷达数据更新地图
        
        Args:
            lidar_data: 雷达数据，None则使用模拟数据
        """
        if lidar_data is None:
            lidar_data = self.simulate_lidar_scan()
        
        self.map.update_with_lidar(lidar_data, tuple(self.robot_pose))

