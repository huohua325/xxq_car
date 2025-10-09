"""
Frontier检测算法测试
"""

import pytest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.slam.occupancy_map import OccupancyGridMap, MapConfig
from src.slam.frontier_detector import FrontierDetector


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
    """创建简单测试地图（有明确的前沿）"""
    config = MapConfig(
        width=100,
        height=100,
        resolution=0.1,
        origin_x=50,
        origin_y=50
    )
    map_obj = OccupancyGridMap(config)
    
    # 创建一个已探索的矩形区域
    # 中心区域设为空闲
    for y in range(40, 60):
        for x in range(40, 60):
            map_obj.grid[y, x] = 0.1  # 空闲
    
    # 周围保持未知（0.5）
    # 这样边界处就是前沿点
    
    return map_obj


def test_frontier_detector_init(map_obj):
    """测试Frontier检测器初始化"""
    detector = FrontierDetector(map_obj)
    
    assert detector.map == map_obj
    assert detector.min_frontier_size == 5
    assert detector.cluster_distance == 0.3
    assert len(detector.neighbors) == 8


def test_is_frontier_basic(simple_map):
    """测试基础前沿点判断"""
    detector = FrontierDetector(simple_map)
    
    # 边界点应该是前沿点
    # (40, 40)是空闲区域边界，周围有未知区域
    assert detector._is_frontier(40, 40) == True
    
    # 中心点不是前沿点（周围都是空闲）
    assert detector._is_frontier(50, 50) == False
    
    # 完全未知区域不是前沿点
    assert detector._is_frontier(10, 10) == False


def test_find_frontiers_simple(simple_map):
    """测试前沿点查找"""
    detector = FrontierDetector(simple_map, min_frontier_size=1)
    
    frontiers = detector.find_frontiers()
    
    # 应该找到前沿点
    assert len(frontiers) > 0
    
    # 前沿点应该在世界坐标范围内
    for fx, fy in frontiers:
        assert -5.0 <= fx <= 5.0
        assert -5.0 <= fy <= 5.0


def test_frontier_clustering():
    """测试前沿点聚类"""
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    detector = FrontierDetector(map_obj, cluster_distance=0.5)
    
    # 创建两组距离较远的点
    frontiers = [
        (0.0, 0.0), (0.1, 0.1), (0.2, 0.0),  # 第一组
        (2.0, 2.0), (2.1, 2.1), (2.0, 2.2),  # 第二组
    ]
    
    clusters = detector._cluster_frontiers(frontiers)
    
    # 应该分成两个簇
    assert len(clusters) == 2
    
    # 每个簇应该有3个点
    assert len(clusters[0]) == 3
    assert len(clusters[1]) == 3


def test_select_nearest_frontier():
    """测试选择最近前沿点"""
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    detector = FrontierDetector(map_obj)
    
    frontiers = [
        (1.0, 1.0),
        (2.0, 2.0),
        (0.5, 0.5),
    ]
    
    robot_pose = (0.0, 0.0, 0.0)
    
    target = detector.select_best_frontier(frontiers, robot_pose, strategy='nearest')
    
    # 应该选择(0.5, 0.5)，因为它最近
    assert target == (0.5, 0.5)


def test_find_frontiers_with_info(simple_map):
    """测试查找前沿点详细信息"""
    detector = FrontierDetector(simple_map, min_frontier_size=1)
    
    frontiers_info = detector.find_frontiers_with_info()
    
    assert len(frontiers_info) > 0
    
    for info in frontiers_info:
        assert 'center' in info
        assert 'size' in info
        assert 'points' in info
        assert isinstance(info['center'], tuple)
        assert isinstance(info['size'], int)
        assert isinstance(info['points'], list)


def test_select_best_frontier_advanced():
    """测试高级前沿点选择"""
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    detector = FrontierDetector(map_obj)
    
    frontiers_info = [
        {'center': (1.0, 1.0), 'size': 10, 'points': []},
        {'center': (2.0, 2.0), 'size': 20, 'points': []},
        {'center': (0.5, 0.5), 'size': 5, 'points': []},
    ]
    
    robot_pose = (0.0, 0.0, 0.0)
    
    # 测试nearest策略
    target = detector.select_best_frontier_advanced(
        frontiers_info, robot_pose, strategy='nearest'
    )
    assert target == (0.5, 0.5)
    
    # 测试largest策略
    target = detector.select_best_frontier_advanced(
        frontiers_info, robot_pose, strategy='largest'
    )
    assert target == (2.0, 2.0)  # size=20最大


def test_information_gain():
    """测试信息增益计算"""
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    detector = FrontierDetector(map_obj)
    
    # 设置一些未知区域
    for y in range(45, 55):
        for x in range(45, 55):
            map_obj.grid[y, x] = 0.5  # 未知
    
    frontier = (0.0, 0.0)
    robot_pose = (0.0, 0.0, 0.0)
    
    gain = detector.calculate_information_gain(frontier, robot_pose)
    
    # 应该返回一个数值
    assert isinstance(gain, (int, float))


def test_empty_map_no_frontiers():
    """测试空地图没有前沿点"""
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    # 全部是未知区域（0.5）
    
    detector = FrontierDetector(map_obj)
    frontiers = detector.find_frontiers()
    
    # 全未知地图应该没有前沿点
    assert len(frontiers) == 0


def test_fully_explored_no_frontiers():
    """测试完全探索的地图没有前沿点"""
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    
    # 全部设为空闲
    map_obj.grid[:, :] = 0.1
    
    detector = FrontierDetector(map_obj)
    frontiers = detector.find_frontiers()
    
    # 完全探索的地图应该没有前沿点
    assert len(frontiers) == 0


def test_real_scenario():
    """测试真实场景：部分探索的房间"""
    config = MapConfig(
        width=100,
        height=100,
        resolution=0.1,
        origin_x=50,
        origin_y=50
    )
    map_obj = OccupancyGridMap(config)
    
    # 创建一个部分探索的房间
    # 机器人在中心探索了一小块区域
    robot_x, robot_y = 0.0, 0.0
    robot_gx, robot_gy = map_obj.world_to_grid(robot_x, robot_y)
    
    # 设置探索区域为空闲
    for dy in range(-10, 10):
        for dx in range(-10, 10):
            gx, gy = robot_gx + dx, robot_gy + dy
            if map_obj.is_valid_grid(gx, gy):
                map_obj.grid[gy, gx] = 0.1
    
    # 添加一些障碍物
    for dx in range(-5, 5):
        gx, gy = robot_gx + dx, robot_gy + 5
        if map_obj.is_valid_grid(gx, gy):
            map_obj.grid[gy, gx] = 0.9
    
    # 检测前沿
    detector = FrontierDetector(map_obj, min_frontier_size=3)
    frontiers = detector.find_frontiers()
    
    # 应该找到前沿点
    assert len(frontiers) > 0
    print(f"\n[真实场景] 找到 {len(frontiers)} 个前沿点")
    
    # 选择目标
    robot_pose = (robot_x, robot_y, 0.0)
    target = detector.select_best_frontier(frontiers, robot_pose)
    
    assert target is not None
    print(f"[真实场景] 选择目标: ({target[0]:.2f}, {target[1]:.2f})")


if __name__ == '__main__':
    # 运行测试
    pytest.main([__file__, '-v', '-s'])

