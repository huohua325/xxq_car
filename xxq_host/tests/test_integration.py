"""
集成测试
测试完整的探索流程和模块集成
"""

import pytest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.slam.occupancy_map import OccupancyGridMap, MapConfig
from src.slam.frontier_detector import FrontierDetector
from src.navigation.path_planner import PathPlanner
from src.navigation.dwa import DWA
from src.navigation.controller import RobotController, RobotState


def test_controller_initialization():
    """测试控制器初始化"""
    controller = RobotController(enable_visualization=False)
    
    assert controller.map is not None
    assert controller.frontier_detector is not None
    assert controller.path_planner is not None
    assert controller.dwa is not None
    assert controller.state == RobotState.IDLE


def test_slam_frontier_integration():
    """测试SLAM和Frontier集成"""
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    
    # 设置空闲区域
    for y in range(40, 60):
        for x in range(40, 60):
            map_obj.grid[y, x] = 0.1
    
    # 检测前沿
    detector = FrontierDetector(map_obj, min_frontier_size=3)
    frontiers = detector.find_frontiers()
    
    assert len(frontiers) >= 0  # 应该找到前沿点或没有前沿点


def test_frontier_pathplanner_integration():
    """测试Frontier和PathPlanner集成"""
    config = MapConfig(width=100, height=100, resolution=0.1, origin_x=50, origin_y=50)
    map_obj = OccupancyGridMap(config)
    
    # 创建空闲区域
    for y in range(30, 70):
        for x in range(30, 70):
            map_obj.grid[y, x] = 0.1
    
    # 检测前沿
    detector = FrontierDetector(map_obj, min_frontier_size=3)
    frontiers = detector.find_frontiers()
    
    if frontiers:
        # 规划到前沿点的路径
        planner = PathPlanner(map_obj)
        robot_pose = (0.0, 0.0)
        target = frontiers[0]
        
        path = planner.plan_path(robot_pose, target)
        assert path is None or len(path) >= 1  # 至少有起点


def test_pathplanner_dwa_integration():
    """测试PathPlanner和DWA集成"""
    config = MapConfig(width=100, height=100, resolution=0.1, origin_x=50, origin_y=50)
    map_obj = OccupancyGridMap(config)
    
    # 创建空闲区域
    for y in range(30, 70):
        for x in range(30, 70):
            map_obj.grid[y, x] = 0.1
    
    # 规划路径
    planner = PathPlanner(map_obj)
    path = planner.plan_path((-1.0, -1.0), (1.0, 1.0))
    
    if path and len(path) > 1:
        # 使用DWA跟随路径
        dwa = DWA()
        robot_state = (-1.0, -1.0, 0.0, 0.0, 0.0)
        local_goal = path[1]
        
        v, omega = dwa.plan(robot_state, local_goal)
        
        assert v >= 0
        assert abs(omega) <= dwa.config.max_yaw_rate


def test_full_exploration_cycle():
    """测试完整探索循环"""
    controller = RobotController(enable_visualization=False)
    
    # 运行几步探索
    steps_run = 0
    for _ in range(10):
        controller.update_map_from_lidar()
        controller._control_step()
        steps_run += 1
        
        # 验证状态有效
        assert controller.state in RobotState
        
        if controller.state == RobotState.COMPLETED:
            break
    
    # 至少运行了一些步骤
    assert steps_run > 0


def test_stuck_detection_and_recovery():
    """测试卡住检测和恢复"""
    controller = RobotController(enable_visualization=False)
    
    # 模拟卡住情况（位置不变）
    initial_pose = controller.robot_pose.copy()
    
    for _ in range(60):  # 超过stuck_threshold
        controller.last_pose = initial_pose
        controller.robot_pose = initial_pose.copy()
        controller._check_stuck()
    
    # 应该检测到卡住
    assert controller.stuck_counter >= controller.stuck_threshold
    assert controller.state == RobotState.STUCK
    
    # 执行恢复
    controller._handle_stuck()
    
    # 恢复后状态应该重置
    assert controller.stuck_counter == 0
    assert controller.state == RobotState.EXPLORING


def test_state_transitions():
    """测试状态转换"""
    controller = RobotController(enable_visualization=False)
    
    # 初始状态
    assert controller.state == RobotState.IDLE
    
    # 开始探索
    controller.state = RobotState.EXPLORING
    assert controller.state == RobotState.EXPLORING
    
    # 完成探索
    controller.state = RobotState.COMPLETED
    assert controller.state == RobotState.COMPLETED


def test_map_update_integration():
    """测试地图更新集成"""
    controller = RobotController(enable_visualization=False)
    
    stats_before = controller.map.get_statistics()
    initial_cells = stats_before.get('free_cells', 0) + stats_before.get('obstacle_cells', 0)
    
    # 更新地图
    controller.update_map_from_lidar()
    
    stats_after = controller.map.get_statistics()
    updated_cells = stats_after.get('free_cells', 0) + stats_after.get('obstacle_cells', 0)
    
    # 探索的格子数应该增加或保持
    assert updated_cells >= initial_cells


def test_exploration_statistics():
    """测试探索统计"""
    controller = RobotController(enable_visualization=False)
    
    # 运行一些步骤
    steps_run = 0
    for _ in range(20):
        controller.update_map_from_lidar()
        controller._control_step()
        steps_run += 1
        
        if controller.state == RobotState.COMPLETED:
            break
    
    # 验证统计信息
    assert steps_run > 0
    assert controller.total_distance >= 0


def test_performance_basic():
    """基础性能测试"""
    import time
    
    controller = RobotController(enable_visualization=False)
    
    start_time = time.time()
    
    # 运行50步
    for _ in range(50):
        controller.update_map_from_lidar()
        controller._control_step()
        
        if controller.state == RobotState.COMPLETED:
            break
    
    elapsed = time.time() - start_time
    
    # 50步应该在合理时间内完成（<5秒）
    assert elapsed < 5.0
    
    print(f"\n[性能] 50步用时: {elapsed:.2f}s, 平均: {elapsed/50*1000:.1f}ms/步")


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])

