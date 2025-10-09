"""
DWA (Dynamic Window Approach) 局部避障算法
实现动态窗口避障、轨迹预测和评分
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class DWAConfig:
    """DWA配置参数"""
    # 机器人速度限制
    max_speed: float = 1.0  # 最大线速度 (m/s)
    min_speed: float = 0.0  # 最小线速度 (m/s)
    max_yaw_rate: float = np.deg2rad(40.0)  # 最大角速度 (rad/s)
    
    # 机器人加速度限制
    max_accel: float = 0.5  # 最大线加速度 (m/s²)
    max_yaw_accel: float = np.deg2rad(80.0)  # 最大角加速度 (rad/s²)
    
    # 速度分辨率
    v_resolution: float = 0.1  # 线速度分辨率 (m/s)
    yaw_rate_resolution: float = np.deg2rad(5.0)  # 角速度分辨率 (rad/s)
    
    # 预测时间
    predict_time: float = 2.0  # 轨迹预测时间 (s)
    dt: float = 0.1  # 预测步长 (s)
    
    # 评分权重
    heading_weight: float = 0.5  # 朝向得分权重
    distance_weight: float = 0.3  # 距离得分权重
    velocity_weight: float = 0.2  # 速度得分权重
    
    # 安全参数
    robot_radius: float = 0.2  # 机器人半径 (m)
    safety_margin: float = 0.1  # 安全裕度 (m)


class DWA:
    """动态窗口避障算法
    
    实现局部路径规划和实时避障：
    1. 计算动态窗口（速度空间）
    2. 预测多条轨迹
    3. 评分并选择最优轨迹
    4. 输出速度指令
    
    Attributes:
        config: DWAConfig配置对象
        
    Example:
        >>> dwa = DWA()
        >>> v, omega = dwa.plan(robot_state, goal, obstacles)
        >>> print(f"线速度: {v}, 角速度: {omega}")
    """
    
    def __init__(self, config: DWAConfig = None):
        """初始化DWA
        
        Args:
            config: DWA配置，None则使用默认配置
        """
        self.config = config if config else DWAConfig()
        
        print(f"[DWA] 初始化完成 "
              f"(max_v={self.config.max_speed}, "
              f"max_w={np.rad2deg(self.config.max_yaw_rate):.1f}°/s)")
    
    def plan(self,
             robot_state: Tuple[float, float, float, float, float],
             goal: Tuple[float, float],
             obstacles: List[Tuple[float, float]] = None) -> Tuple[float, float]:
        """DWA规划
        
        Args:
            robot_state: 机器人状态 (x, y, theta, v, omega)
                - x, y: 位置 (m)
                - theta: 航向角 (rad)
                - v: 线速度 (m/s)
                - omega: 角速度 (rad/s)
            goal: 目标位置 (x, y)
            obstacles: 障碍物列表 [(x1, y1), (x2, y2), ...]
            
        Returns:
            (v, omega): 最优线速度和角速度
        """
        x, y, theta, v, omega = robot_state
        
        # 计算动态窗口
        v_range, omega_range = self._calc_dynamic_window(v, omega)
        
        # 遍历速度空间，评分
        best_v = 0.0
        best_omega = 0.0
        max_score = -float('inf')
        
        # 生成速度候选
        v_samples = np.arange(v_range[0], v_range[1], self.config.v_resolution)
        omega_samples = np.arange(omega_range[0], omega_range[1], self.config.yaw_rate_resolution)
        
        for v_test in v_samples:
            for omega_test in omega_samples:
                # 预测轨迹
                trajectory = self._predict_trajectory(x, y, theta, v_test, omega_test)
                
                # 检查碰撞
                if obstacles and self._check_collision(trajectory, obstacles):
                    continue
                
                # 评分
                score = self._evaluate_trajectory(trajectory, goal, v_test)
                
                if score > max_score:
                    max_score = score
                    best_v = v_test
                    best_omega = omega_test
        
        return best_v, best_omega
    
    def _calc_dynamic_window(self, 
                            current_v: float, 
                            current_omega: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """计算动态窗口
        
        考虑速度限制和加速度限制
        
        Args:
            current_v: 当前线速度
            current_omega: 当前角速度
            
        Returns:
            (v_range, omega_range): 速度范围
                - v_range: (min_v, max_v)
                - omega_range: (min_omega, max_omega)
        """
        # 速度限制
        v_min = self.config.min_speed
        v_max = self.config.max_speed
        omega_min = -self.config.max_yaw_rate
        omega_max = self.config.max_yaw_rate
        
        # 加速度限制（动态窗口）
        v_min_dyn = current_v - self.config.max_accel * self.config.dt
        v_max_dyn = current_v + self.config.max_accel * self.config.dt
        omega_min_dyn = current_omega - self.config.max_yaw_accel * self.config.dt
        omega_max_dyn = current_omega + self.config.max_yaw_accel * self.config.dt
        
        # 取交集
        v_min = max(v_min, v_min_dyn)
        v_max = min(v_max, v_max_dyn)
        omega_min = max(omega_min, omega_min_dyn)
        omega_max = min(omega_max, omega_max_dyn)
        
        return (v_min, v_max), (omega_min, omega_max)
    
    def _predict_trajectory(self,
                           x: float,
                           y: float,
                           theta: float,
                           v: float,
                           omega: float) -> List[Tuple[float, float, float]]:
        """预测轨迹
        
        使用运动学模型预测未来轨迹
        
        Args:
            x, y: 起始位置
            theta: 起始航向角
            v: 线速度
            omega: 角速度
            
        Returns:
            轨迹点列表 [(x1, y1, theta1), (x2, y2, theta2), ...]
        """
        trajectory = []
        time = 0.0
        
        while time <= self.config.predict_time:
            # 运动学模型
            x += v * np.cos(theta) * self.config.dt
            y += v * np.sin(theta) * self.config.dt
            theta += omega * self.config.dt
            
            # 归一化角度到[-π, π]
            theta = np.arctan2(np.sin(theta), np.cos(theta))
            
            trajectory.append((x, y, theta))
            time += self.config.dt
        
        return trajectory
    
    def _check_collision(self,
                        trajectory: List[Tuple[float, float, float]],
                        obstacles: List[Tuple[float, float]]) -> bool:
        """检查轨迹是否与障碍物碰撞
        
        Args:
            trajectory: 轨迹点列表
            obstacles: 障碍物列表
            
        Returns:
            是否碰撞
        """
        safe_distance = self.config.robot_radius + self.config.safety_margin
        
        for x, y, _ in trajectory:
            for obs_x, obs_y in obstacles:
                dist = np.hypot(x - obs_x, y - obs_y)
                if dist < safe_distance:
                    return True  # 碰撞
        
        return False  # 无碰撞
    
    def _evaluate_trajectory(self,
                            trajectory: List[Tuple[float, float, float]],
                            goal: Tuple[float, float],
                            v: float) -> float:
        """评估轨迹得分
        
        综合考虑：
        1. heading: 朝向目标的程度
        2. distance: 接近目标的程度
        3. velocity: 速度大小（鼓励快速移动）
        
        Args:
            trajectory: 轨迹点列表
            goal: 目标位置
            v: 线速度
            
        Returns:
            得分（越高越好）
        """
        if not trajectory:
            return 0.0
        
        # 轨迹终点
        end_x, end_y, end_theta = trajectory[-1]
        
        # 1. 朝向得分
        goal_angle = np.arctan2(goal[1] - end_y, goal[0] - end_x)
        angle_diff = abs(np.arctan2(np.sin(end_theta - goal_angle), 
                                    np.cos(end_theta - goal_angle)))
        heading_score = np.pi - angle_diff  # 角度差越小，得分越高
        
        # 2. 距离得分
        dist_to_goal = np.hypot(end_x - goal[0], end_y - goal[1])
        distance_score = 1.0 / (dist_to_goal + 1.0)  # 距离越近，得分越高
        
        # 3. 速度得分
        velocity_score = v / self.config.max_speed  # 速度越快，得分越高
        
        # 综合得分
        total_score = (self.config.heading_weight * heading_score +
                      self.config.distance_weight * distance_score +
                      self.config.velocity_weight * velocity_score)
        
        return total_score
    
    def predict_trajectory_to_goal(self,
                                   robot_state: Tuple[float, float, float, float, float],
                                   goal: Tuple[float, float],
                                   max_steps: int = 100) -> List[Tuple[float, float]]:
        """使用DWA预测到达目标的完整轨迹
        
        Args:
            robot_state: 机器人状态
            goal: 目标位置
            max_steps: 最大步数
            
        Returns:
            轨迹点列表
        """
        trajectory = []
        x, y, theta, v, omega = robot_state
        
        for _ in range(max_steps):
            trajectory.append((x, y))
            
            # 检查是否到达目标
            dist = np.hypot(x - goal[0], y - goal[1])
            if dist < 0.2:  # 到达阈值
                break
            
            # DWA规划下一步
            current_state = (x, y, theta, v, omega)
            v, omega = self.plan(current_state, goal)
            
            # 更新状态
            x += v * np.cos(theta) * self.config.dt
            y += v * np.sin(theta) * self.config.dt
            theta += omega * self.config.dt
            theta = np.arctan2(np.sin(theta), np.cos(theta))
        
        trajectory.append(goal)
        return trajectory
    
    def calculate_stopping_time(self, current_v: float) -> float:
        """计算停车时间
        
        Args:
            current_v: 当前速度
            
        Returns:
            停车所需时间（秒）
        """
        if current_v <= 0:
            return 0.0
        
        return current_v / self.config.max_accel
    
    def calculate_stopping_distance(self, current_v: float) -> float:
        """计算停车距离
        
        Args:
            current_v: 当前速度
            
        Returns:
            停车所需距离（米）
        """
        if current_v <= 0:
            return 0.0
        
        # s = v²/(2a)
        return current_v * current_v / (2 * self.config.max_accel)

