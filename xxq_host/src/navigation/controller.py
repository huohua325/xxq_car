"""
主控制器模块
集成SLAM、Frontier探索、路径规划和DWA避障，实现完整的自主探索
"""

import numpy as np
from typing import Tuple, List, Optional
from enum import Enum
import time
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
import config


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
                 enable_visualization=True,
                 visualization_mode='matplotlib',
                 web_port=5000):
        """初始化控制器
        
        Args:
            comm: RobotComm通信对象（可选）
            map_obj: OccupancyGridMap对象（可选，None则自动创建）
            visualizer: MapVisualizer对象（可选）
            enable_visualization: 是否启用可视化
            visualization_mode: 可视化模式 ('matplotlib', 'web', 'none')
            web_port: Web服务器端口（仅当visualization_mode='web'时有效）
        """
        # 通信模块
        self.comm = comm
        
        # SLAM模块（使用配置文件参数）
        if map_obj is None:
            from src.slam.occupancy_map import OccupancyGridMap, MapConfig
            map_config = MapConfig(
                width=config.MAP_WIDTH,
                height=config.MAP_HEIGHT,
                resolution=config.MAP_RESOLUTION,
                origin_x=config.MAP_ORIGIN_X,
                origin_y=config.MAP_ORIGIN_Y,
                free_threshold=config.MAP_FREE_THRESHOLD,
                occupied_threshold=config.MAP_OCCUPIED_THRESHOLD,
                prob_occupied=config.MAP_PROB_OCCUPIED,
                prob_free=config.MAP_PROB_FREE
            )
            self.map = OccupancyGridMap(map_config)
        else:
            self.map = map_obj
        
        # Frontier探索模块（使用配置文件参数）
        from src.slam.frontier_detector import FrontierDetector
        self.frontier_detector = FrontierDetector(
            self.map, 
            min_frontier_size=config.MIN_FRONTIER_SIZE,
            cluster_distance=config.FRONTIER_CLUSTER_DIST
        )
        
        # 路径规划模块（使用配置文件参数）
        from src.navigation.path_planner import PathPlanner, PathPlannerConfig
        planner_config = PathPlannerConfig(
            obstacle_threshold=config.PATH_OBSTACLE_THRESHOLD,
            inflation_radius=config.PATH_INFLATION_RADIUS,
            allow_diagonal=config.PATH_ALLOW_DIAGONAL,
            diagonal_cost=config.PATH_DIAGONAL_COST,
            smoothing_tolerance=config.PATH_SMOOTHING_TOLERANCE
        )
        self.path_planner = PathPlanner(self.map, planner_config)
        
        # DWA避障模块（使用配置文件参数）
        from src.navigation.dwa import DWA, DWAConfig
        dwa_config = DWAConfig(
            max_speed=config.DWA_MAX_SPEED,
            min_speed=0.0,  # 最小速度固定为0
            max_yaw_rate=np.deg2rad(config.DWA_MAX_YAW_RATE),
            max_accel=config.DWA_MAX_ACCEL,
            max_yaw_accel=np.deg2rad(config.DWA_MAX_YAW_ACCEL),
            v_resolution=config.DWA_V_RESOLUTION,
            yaw_rate_resolution=np.deg2rad(config.DWA_YAW_RESOLUTION),
            predict_time=config.DWA_PREDICT_TIME,
            dt=config.DWA_DT,
            heading_weight=config.DWA_WEIGHT_HEADING,
            distance_weight=config.DWA_WEIGHT_CLEARANCE,
            velocity_weight=config.DWA_WEIGHT_VELOCITY,
            robot_radius=config.DWA_OBSTACLE_RADIUS,
            safety_margin=config.DWA_MIN_CLEARANCE
        )
        self.dwa = DWA(dwa_config)
        
        # 可视化模块
        self.enable_visualization = enable_visualization
        self.visualization_mode = visualization_mode if enable_visualization else 'none'
        self.visualizer = visualizer
        
        # 如果启用可视化但未提供visualizer，则自动创建
        if self.enable_visualization and self.visualizer is None:
            if self.visualization_mode == 'web':
                from src.visualization.web_visualizer import create_web_visualizer
                self.visualizer = create_web_visualizer(self.map, port=web_port, auto_start=True)
                print(f"[控制器] ✅ Web可视化已启动: http://localhost:{web_port}")
            elif self.visualization_mode == 'matplotlib':
                from src.visualization.map_visualizer import MapVisualizer
                self.visualizer = MapVisualizer(
                    self.map,
                    enable_multi_window=config.VISUALIZE_MULTI_WINDOW
                )
                print("[控制器] ✅ Matplotlib可视化已启动")
            else:
                self.enable_visualization = False
                print("[控制器] ⚠️  未知的可视化模式，已禁用可视化")
        
        # 机器人状态
        self.robot_pose = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.robot_velocity = [0.0, 0.0]   # [v, omega]
        self.state = RobotState.IDLE
        
        # 导航状态
        self.current_target = None
        self.current_path = []
        self.path_index = 0
        
        # 卡住检测（使用配置文件参数）
        self.stuck_counter = 0
        self.stuck_threshold = config.STUCK_THRESHOLD
        self.last_pose = [0.0, 0.0, 0.0]
        
        # 统计信息
        self.exploration_steps = 0
        self.total_distance = 0.0
        self.start_time = None
        
        # ✅ 硬件POSE集成
        if self.comm:
            print("[Controller] 硬件模式：使用STM32 POSE数据")
            self.comm.on_pose_update = self._on_hardware_pose_update
            self.comm.on_lidar_update = self._on_hardware_lidar_update
        else:
            print("[Controller] ⚠️ 警告：无通信对象，位姿数据将不会更新")
        
        print("[Controller] 主控制器初始化完成")
    
    def _on_hardware_pose_update(self, pose_data):
        """接收STM32的POSE数据（硬件位姿）
        
        Args:
            pose_data: PoseData对象，包含x, y, theta
        """
        # 保存上次位姿（用于卡住检测和距离统计）
        self.last_pose = self.robot_pose.copy()
        
        # 使用硬件POSE数据
        self.robot_pose[0] = pose_data.x
        self.robot_pose[1] = pose_data.y
        self.robot_pose[2] = np.deg2rad(pose_data.theta)  # 度 → 弧度
        
        # 更新总距离（基于实际位移）
        displacement = np.hypot(
            self.robot_pose[0] - self.last_pose[0],
            self.robot_pose[1] - self.last_pose[1]
        )
        self.total_distance += displacement
        
        # 定期输出（避免刷屏）
        if self.exploration_steps % 50 == 0 and self.exploration_steps > 0:
            print(f"[硬件POSE] x={pose_data.x:.3f}m, "
                  f"y={pose_data.y:.3f}m, "
                  f"θ={pose_data.theta:.1f}°")
    
    def _on_hardware_lidar_update(self, lidar_data):
        """接收STM32的雷达数据（硬件雷达）
        
        Args:
            lidar_data: LidarData对象，包含sectors扇区数据
        """
        # 使用雷达数据更新地图
        self.map.update_with_lidar(lidar_data, tuple(self.robot_pose))
        
        # 定期输出（避免刷屏）
        if self.exploration_steps % 50 == 0 and self.exploration_steps > 0:
            print(f"[硬件雷达] 收到{lidar_data.total_points}个点")
    
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
        
        # ✅ 启动前请求一次雷达扫描
        if self.comm:
            print("[启动] 请求初始雷达扫描...")
            self.comm.request_lidar_scan()
            time.sleep(0.5)  # 等待雷达数据
        
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
        # ✅ 定期请求雷达扫描（每10步）
        if self.comm and self.exploration_steps % 10 == 0:
            self.comm.request_lidar_scan()
        
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
                strategy=config.FRONTIER_SELECTION
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
            
            # 发送速度指令到硬件
            self._update_robot_state(v, omega, dt=0.1)
            
            # 控制循环延时（避免指令发送过快）
            time.sleep(0.1)  # 10Hz控制频率
        
        # 5. 卡住检测
        self._check_stuck()
        
        # 6. 可视化
        if self.enable_visualization and self.visualizer:
            self._update_visualization(frontiers)
        
        return True
    
    def _update_robot_state(self, v: float, omega: float, dt: float = 0.1):
        """发送速度指令到机器人（硬件模式）
        
        位姿由STM32的POSE数据更新，这里只负责发送速度指令。
        
        Args:
            v: 线速度 (m/s)
            omega: 角速度 (rad/s)
            dt: 时间步长 (未使用，保留接口兼容性)
        """
        # 更新速度记录
        self.robot_velocity = [v, omega]
        
        # 发送速度指令到STM32
        if self.comm:
            # 差速运动学逆解：将 (v, ω) 转换为左右轮速
            wheel_base = config.WHEEL_BASE  # 轮距 (m)
            wheel_radius = config.WHEEL_RADIUS  # 轮半径 (m)
            
            # 左右轮线速度
            v_left = v - omega * wheel_base / 2.0
            v_right = v + omega * wheel_base / 2.0
            
            # 转换为轮速 (RPS - 每秒转数)
            left_rps = v_left / (2.0 * np.pi * wheel_radius)
            right_rps = v_right / (2.0 * np.pi * wheel_radius)
            
            # 发送指令
            self.comm.send_speed_command(left_rps, right_rps)
        else:
            print("[警告] 无通信对象，无法发送速度指令")
        
        # 注意：位姿更新由 _on_hardware_pose_update() 处理
    
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
        """处理卡住情况（硬件模式）"""
        import time
        
        print("[Controller] 🚨 检测到卡住，执行恢复动作...")
        
        # 策略1: 后退 0.5m/s，持续1秒
        print("  → 后退中...")
        self._update_robot_state(-0.5, 0.0)
        time.sleep(1.0)  # 等待实际执行
        
        # 停止
        self._update_robot_state(0.0, 0.0)
        time.sleep(0.2)
        
        # 策略2: 随机转向
        turn_direction = np.random.choice([-1, 1])  # 左或右
        turn_speed = turn_direction * 0.5  # rad/s
        print(f"  → {'左' if turn_direction < 0 else '右'}转中...")
        self._update_robot_state(0.0, turn_speed)
        time.sleep(0.8)  # 转约45度
        
        # 停止
        self._update_robot_state(0.0, 0.0)
        time.sleep(0.2)
        
        # 重置状态
        self.stuck_counter = 0
        self.current_target = None
        self.current_path = []
        self.path_index = 0
        self.state = RobotState.EXPLORING
        
        print("[Controller] ✅ 恢复完成，继续探索")
    
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

