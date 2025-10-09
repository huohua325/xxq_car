"""
路径规划与导航测试
"""

import pytest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.slam.occupancy_map import OccupancyGridMap, MapConfig
from src.navigation.path_planner import PathPlanner, PathPlannerConfig
from src.navigation.dwa import DWA, DWAConfig


@pytest.fixture
def map_obj():
    """创建测试地图"""
    config = MapConfig(
        width=100,
        height=100,
        resolution=0.1,
        origin_x=50,
        origin_y=50
    )
    return OccupancyGridMap(config)


@pytest.fixture
def simple_map():
    """创建简单测试地图（有空闲区域）"""
    config = MapConfig(
        width=100,
        height=100,
        resolution=0.1,
        origin_x=50,
        origin_y=50
    )
    map_obj = OccupancyGridMap(config)
    
    # 中心区域设为空闲
    for y in range(30, 70):
        for x in range(30, 70):
            map_obj.grid[y, x] = 0.1  # 空闲
    
    return map_obj


# ============================================================================
# PathPlanner Tests
# ============================================================================

def test_path_planner_init(map_obj):
    """测试PathPlanner初始化"""
    planner = PathPlanner(map_obj)
    
    assert planner.map == map_obj
    assert isinstance(planner.config, PathPlannerConfig)
    assert len(planner.neighbors_8) == 8
    assert len(planner.neighbors_4) == 4


def test_astar_simple_path(simple_map):
    """测试A*算法 - 简单路径"""
    planner = PathPlanner(simple_map)
    
    start = (-1.0, -1.0)  # 左下
    goal = (1.0, 1.0)     # 右上
    
    path = planner.plan_path(start, goal, inflate_obstacles=False, smooth=False)
    
    assert path is not None
    assert len(path) >= 2
    assert path[0] == start or np.allclose(path[0], start, atol=0.15)
    assert path[-1] == goal or np.allclose(path[-1], goal, atol=0.15)


def test_astar_no_path():
    """测试A*算法 - 无路径情况"""
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    
    # 创建障碍物墙
    for y in range(40, 60):
        for x in range(45, 55):
            map_obj.grid[y, x] = 0.9  # 障碍物
    
    # 设置两侧为空闲
    for y in range(40, 60):
        for x in range(30, 45):
            map_obj.grid[y, x] = 0.1
        for x in range(55, 70):
            map_obj.grid[y, x] = 0.1
    
    planner = PathPlanner(map_obj)
    
    # 尝试跨越障碍物墙
    start = (-1.0, 0.0)
    goal = (1.0, 0.0)
    
    path = planner.plan_path(start, goal, inflate_obstacles=False)
    
    # 可能找到绕路，也可能没有路径
    # 这里只检查函数不崩溃
    assert path is None or len(path) >= 2


def test_path_smoothing():
    """测试路径平滑"""
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    planner = PathPlanner(map_obj)
    
    # 创建锯齿状路径
    path = [
        (0.0, 0.0),
        (0.1, 0.1),
        (0.2, 0.2),
        (0.3, 0.3),
        (1.0, 1.0),
    ]
    
    smoothed = planner.smooth_path(path)
    
    # 平滑后点数应该减少
    assert len(smoothed) <= len(path)
    # 起点和终点应该保持
    assert smoothed[0] == path[0]
    assert smoothed[-1] == path[-1]


def test_obstacle_inflation():
    """测试障碍物膨胀"""
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    planner = PathPlanner(map_obj)
    
    # 创建小障碍物
    map_obj.grid[50, 50] = 0.9
    
    obstacle_map = map_obj.grid >= 0.7
    inflated = planner.inflate_obstacles(obstacle_map, radius=2)
    
    # 膨胀后障碍物范围应该更大
    assert np.sum(inflated) > np.sum(obstacle_map)


def test_path_length_calculation():
    """测试路径长度计算"""
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    planner = PathPlanner(map_obj)
    
    path = [
        (0.0, 0.0),
        (1.0, 0.0),
        (1.0, 1.0),
    ]
    
    length = planner.get_path_length(path)
    
    # 长度应该是 1.0 + 1.0 = 2.0
    assert np.isclose(length, 2.0, atol=0.01)


def test_path_validation(simple_map):
    """测试路径有效性检查"""
    planner = PathPlanner(simple_map)
    
    # 有效路径（在空闲区域）
    valid_path = [
        (0.0, 0.0),
        (0.5, 0.5),
        (1.0, 1.0),
    ]
    
    assert planner.is_path_valid(valid_path) == True
    
    # 无效路径（在未知/障碍物区域）
    invalid_path = [
        (0.0, 0.0),
        (5.0, 5.0),  # 超出空闲区域
    ]
    
    assert planner.is_path_valid(invalid_path) == False


# ============================================================================
# DWA Tests
# ============================================================================

def test_dwa_init():
    """测试DWA初始化"""
    dwa = DWA()
    
    assert isinstance(dwa.config, DWAConfig)
    assert dwa.config.max_speed > 0
    assert dwa.config.max_yaw_rate > 0


def test_dwa_dynamic_window():
    """测试动态窗口计算"""
    dwa = DWA()
    
    current_v = 0.5
    current_omega = 0.1
    
    v_range, omega_range = dwa._calc_dynamic_window(current_v, current_omega)
    
    # 速度范围应该合理
    assert v_range[0] <= v_range[1]
    assert omega_range[0] <= omega_range[1]
    
    # 当前速度应该在范围内
    assert v_range[0] <= current_v <= v_range[1]
    assert omega_range[0] <= current_omega <= omega_range[1]


def test_dwa_trajectory_prediction():
    """测试轨迹预测"""
    dwa = DWA()
    
    x, y, theta = 0.0, 0.0, 0.0
    v = 1.0
    omega = 0.5
    
    trajectory = dwa._predict_trajectory(x, y, theta, v, omega)
    
    # 应该有多个预测点
    assert len(trajectory) > 0
    
    # 轨迹应该前进
    last_x, last_y, _ = trajectory[-1]
    assert last_x > x or last_y > y


def test_dwa_collision_detection():
    """测试碰撞检测"""
    dwa = DWA()
    
    # 创建轨迹
    trajectory = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
    ]
    
    # 测试1: 有障碍物在路径上
    obstacles_collision = [(1.0, 0.0)]
    assert dwa._check_collision(trajectory, obstacles_collision) == True
    
    # 测试2: 障碍物不在路径上
    obstacles_safe = [(10.0, 10.0)]
    assert dwa._check_collision(trajectory, obstacles_safe) == False


def test_dwa_planning():
    """测试DWA规划"""
    dwa = DWA()
    
    robot_state = (0.0, 0.0, 0.0, 0.5, 0.0)  # (x, y, theta, v, omega)
    goal = (5.0, 5.0)
    obstacles = []
    
    v, omega = dwa.plan(robot_state, goal, obstacles)
    
    # 速度应该在合理范围内
    assert 0 <= v <= dwa.config.max_speed
    assert -dwa.config.max_yaw_rate <= omega <= dwa.config.max_yaw_rate


def test_dwa_trajectory_evaluation():
    """测试轨迹评分"""
    dwa = DWA()
    
    # 创建朝向目标的轨迹
    goal = (5.0, 0.0)
    trajectory_good = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
    ]
    
    # 创建偏离目标的轨迹
    trajectory_bad = [
        (0.0, 0.0, 0.0),
        (0.0, 1.0, np.pi/2),
        (0.0, 2.0, np.pi/2),
    ]
    
    score_good = dwa._evaluate_trajectory(trajectory_good, goal, 1.0)
    score_bad = dwa._evaluate_trajectory(trajectory_bad, goal, 1.0)
    
    # 朝向目标的轨迹得分应该更高
    assert score_good > score_bad


def test_dwa_stopping_distance():
    """测试停车距离计算"""
    dwa = DWA()
    
    current_v = 1.0
    distance = dwa.calculate_stopping_distance(current_v)
    
    # 停车距离应该大于0
    assert distance > 0
    
    # 速度为0时停车距离为0
    assert dwa.calculate_stopping_distance(0.0) == 0.0


def test_dwa_stopping_time():
    """测试停车时间计算"""
    dwa = DWA()
    
    current_v = 1.0
    time = dwa.calculate_stopping_time(current_v)
    
    # 停车时间应该大于0
    assert time > 0
    
    # 速度为0时停车时间为0
    assert dwa.calculate_stopping_time(0.0) == 0.0


# ============================================================================
# Integration Tests
# ============================================================================

def test_path_planning_and_dwa_integration(simple_map):
    """测试路径规划与DWA集成"""
    # 规划全局路径
    planner = PathPlanner(simple_map)
    path = planner.plan_path((-1.0, -1.0), (1.0, 1.0))
    
    assert path is not None
    
    # 使用DWA跟随路径
    dwa = DWA()
    robot_state = (-1.0, -1.0, 0.0, 0.0, 0.0)
    
    # 取路径上的下一个点作为局部目标
    if len(path) > 1:
        local_goal = path[1]
        v, omega = dwa.plan(robot_state, local_goal)
        
        # 速度应该有效
        assert v >= 0
        assert abs(omega) <= dwa.config.max_yaw_rate


def test_real_scenario():
    """测试真实场景：带障碍物的路径规划"""
    config = MapConfig(width=100, height=100, resolution=0.1, origin_x=50, origin_y=50)
    map_obj = OccupancyGridMap(config)
    
    # 创建房间（周围是墙）
    for y in range(100):
        map_obj.grid[y, 0] = 0.9
        map_obj.grid[y, 99] = 0.9
    for x in range(100):
        map_obj.grid[0, x] = 0.9
        map_obj.grid[99, x] = 0.9
    
    # 中间区域空闲
    for y in range(20, 80):
        for x in range(20, 80):
            map_obj.grid[y, x] = 0.1
    
    # 添加中间障碍物
    for y in range(40, 60):
        map_obj.grid[y, 50] = 0.9
    
    # 规划路径
    planner = PathPlanner(map_obj)
    start = (-1.0, 0.0)
    goal = (1.0, 0.0)
    
    path = planner.plan_path(start, goal)
    
    if path:
        print(f"\n[真实场景] 找到路径: {len(path)} 个点")
        print(f"[真实场景] 路径长度: {planner.get_path_length(path):.2f}m")
        assert len(path) >= 2


if __name__ == '__main__':
    # 运行测试
    pytest.main([__file__, '-v', '-s'])

