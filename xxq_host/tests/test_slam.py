"""
SLAM模块单元测试
测试占据栅格地图的核心功能
"""

import sys
sys.path.insert(0, 'src')

import pytest
import numpy as np
from slam.occupancy_map import OccupancyGridMap, MapConfig
from communication.protocol import LidarData


class TestOccupancyGridMap:
    """占据栅格地图测试"""
    
    def setup_method(self):
        """测试前准备"""
        self.config = MapConfig(
            width=100,
            height=100,
            resolution=0.1,
            origin_x=50,
            origin_y=50
        )
        self.map = OccupancyGridMap(self.config)
    
    def test_initialization(self):
        """测试地图初始化"""
        assert self.map.grid.shape == (100, 100)
        assert np.all(self.map.grid == 0.5)  # 初始未知
        assert self.map.update_count == 0
    
    def test_world_to_grid_conversion(self):
        """测试世界坐标到栅格坐标的转换"""
        # 原点应该映射到(50, 50)
        gx, gy = self.map.world_to_grid(0.0, 0.0)
        assert gx == 50
        assert gy == 50
        
        # 测试正向偏移
        gx, gy = self.map.world_to_grid(1.0, 0.0)  # 向右1米
        assert gx == 60  # 1m / 0.1m = 10格
        assert gy == 50
        
        # 测试负向偏移
        gx, gy = self.map.world_to_grid(-0.5, -0.5)
        assert gx == 45  # -0.5 / 0.1 = -5
        assert gy == 45
    
    def test_grid_to_world_conversion(self):
        """测试栅格坐标到世界坐标的转换"""
        # 测试原点
        x, y = self.map.grid_to_world(50, 50)
        assert abs(x - 0.0) < 1e-6
        assert abs(y - 0.0) < 1e-6
        
        # 测试其他点
        x, y = self.map.grid_to_world(60, 50)
        assert abs(x - 1.0) < 1e-6
        assert abs(y - 0.0) < 1e-6
    
    def test_round_trip_conversion(self):
        """测试坐标转换的往返一致性"""
        test_points = [
            (0.0, 0.0),
            (1.5, 2.3),
            (-0.8, 1.2),
            (3.0, -1.5)
        ]
        
        for wx, wy in test_points:
            gx, gy = self.map.world_to_grid(wx, wy)
            wx2, wy2 = self.map.grid_to_world(gx, gy)
            
            # 由于整数化，允许一定误差
            assert abs(wx - wx2) < self.config.resolution
            assert abs(wy - wy2) < self.config.resolution
    
    def test_is_valid_grid(self):
        """测试栅格坐标有效性检查"""
        # 有效坐标
        assert self.map.is_valid_grid(0, 0) == True
        assert self.map.is_valid_grid(50, 50) == True
        assert self.map.is_valid_grid(99, 99) == True
        
        # 无效坐标
        assert self.map.is_valid_grid(-1, 0) == False
        assert self.map.is_valid_grid(0, -1) == False
        assert self.map.is_valid_grid(100, 0) == False
        assert self.map.is_valid_grid(0, 100) == False
    
    def test_ray_tracing(self):
        """测试Bresenham射线追踪算法"""
        # 水平射线
        cells = self.map._ray_trace(0, 0, 5, 0)
        assert len(cells) == 6  # 0,1,2,3,4,5
        assert cells[0] == (0, 0)
        assert cells[-1] == (5, 0)
        
        # 垂直射线
        cells = self.map._ray_trace(0, 0, 0, 5)
        assert len(cells) == 6
        assert cells[0] == (0, 0)
        assert cells[-1] == (0, 5)
        
        # 对角射线
        cells = self.map._ray_trace(0, 0, 3, 3)
        assert cells[0] == (0, 0)
        assert cells[-1] == (3, 3)
        
        # 反向射线
        cells = self.map._ray_trace(5, 5, 0, 0)
        assert len(cells) == 6
        assert cells[0] == (5, 5)
        assert cells[-1] == (0, 0)
    
    def test_update_cell(self):
        """测试单个栅格的概率更新"""
        # 初始值应该是0.5（未知）
        assert abs(self.map.grid[50, 50] - 0.5) < 0.01
        
        # 多次标记为占用
        for _ in range(5):
            self.map._update_cell(50, 50, is_occupied=True)
        
        # 占用概率应该增加
        assert self.map.grid[50, 50] > 0.7
        
        # 多次标记为空闲
        for _ in range(10):
            self.map._update_cell(60, 60, is_occupied=False)
        
        # 占用概率应该减少
        assert self.map.grid[60, 60] < 0.3
    
    def test_update_with_lidar(self):
        """测试雷达数据更新地图"""
        # 创建模拟雷达数据
        lidar_data = LidarData(
            timestamp=1000,
            total_points=100,
            angle_coverage=360.0,
            sectors=[
                {'sector_id': 0, 'angle_center': 0, 'count': 10, 'min_dist': 1.0, 'avg_dist': 1.0},
                {'sector_id': 1, 'angle_center': 45, 'count': 10, 'min_dist': 1.5, 'avg_dist': 1.5},
                {'sector_id': 2, 'angle_center': 90, 'count': 10, 'min_dist': 2.0, 'avg_dist': 2.0},
            ]
        )
        
        # 机器人在原点，朝向0度
        robot_pose = (0.0, 0.0, 0.0)
        
        # 更新地图
        self.map.update_with_lidar(lidar_data, robot_pose)
        
        # 检查更新计数
        assert self.map.update_count == 1
        
        # 检查机器人位置附近应该是空闲
        assert self.map.is_free(0.0, 0.0) or self.map.grid[50, 50] < 0.5
        
        # 检查障碍物位置应该是占用
        # 扇区0：0度，1米处应该有障碍物
        assert self.map.is_occupied(1.0, 0.0) or self.map.grid[50, 60] > 0.5
    
    def test_get_binary_map(self):
        """测试二值化地图"""
        # 手动设置一些格子
        self.map.grid[10, 10] = 0.2  # 空闲
        self.map.grid[20, 20] = 0.8  # 占用
        self.map.grid[30, 30] = 0.5  # 未知
        
        binary = self.map.get_binary_map(threshold=0.7)
        
        assert binary[10, 10] == 0  # 空闲
        assert binary[20, 20] == 1  # 占用
        assert binary[30, 30] == 0  # 未知
    
    def test_get_free_space_map(self):
        """测试空闲空间地图"""
        self.map.grid[10, 10] = 0.2  # 空闲
        self.map.grid[20, 20] = 0.8  # 占用
        self.map.grid[30, 30] = 0.5  # 未知
        
        free_map = self.map.get_free_space_map()
        
        assert free_map[10, 10] == 1  # 空闲
        assert free_map[20, 20] == 0  # 占用
        assert free_map[30, 30] == 0  # 未知
    
    def test_is_occupied_is_free(self):
        """测试占用/空闲查询"""
        # 设置测试区域
        gx, gy = self.map.world_to_grid(1.0, 1.0)
        self.map.grid[gy, gx] = 0.9  # 占用
        
        gx2, gy2 = self.map.world_to_grid(-1.0, -1.0)
        self.map.grid[gy2, gx2] = 0.1  # 空闲
        
        # 测试查询
        assert self.map.is_occupied(1.0, 1.0) == True
        assert self.map.is_free(1.0, 1.0) == False
        
        assert self.map.is_occupied(-1.0, -1.0) == False
        assert self.map.is_free(-1.0, -1.0) == True
        
        # 测试边界外
        assert self.map.is_occupied(100.0, 100.0) == True  # 边界外视为占用
        assert self.map.is_free(100.0, 100.0) == False
    
    def test_get_statistics(self):
        """测试统计信息"""
        # 初始状态
        stats = self.map.get_statistics()
        assert stats['total_cells'] == 100 * 100
        assert stats['update_count'] == 0
        assert stats['path_length'] == 0
        
        # 更新后
        lidar_data = LidarData(
            timestamp=1000,
            total_points=10,
            angle_coverage=360.0,
            sectors=[
                {'sector_id': 0, 'angle_center': 0, 'count': 10, 'min_dist': 1.0, 'avg_dist': 1.0}
            ]
        )
        
        self.map.update_with_lidar(lidar_data, (0.0, 0.0, 0.0))
        
        stats = self.map.get_statistics()
        assert stats['update_count'] == 1
        assert stats['path_length'] == 1
        assert stats['explored_ratio'] > 0  # 应该有一些探索过的区域
    
    def test_robot_path_tracking(self):
        """测试机器人轨迹记录"""
        assert len(self.map.robot_path) == 0
        
        # 模拟多次更新
        lidar_data = LidarData(
            timestamp=1000,
            total_points=10,
            angle_coverage=360.0,
            sectors=[
                {'sector_id': 0, 'angle_center': 0, 'count': 10, 'min_dist': 1.0, 'avg_dist': 1.0}
            ]
        )
        
        poses = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
        
        for pose in poses:
            self.map.update_with_lidar(lidar_data, pose)
        
        assert len(self.map.robot_path) == 3
        assert self.map.robot_path[0] == (0.0, 0.0)
        assert self.map.robot_path[1] == (1.0, 0.0)
        assert self.map.robot_path[2] == (2.0, 0.0)
    
    def test_save_and_load_map(self, tmp_path):
        """测试地图保存和加载"""
        # 修改地图
        self.map.grid[50, 50] = 0.9
        self.map.grid[60, 60] = 0.1
        
        # 保存
        map_file = tmp_path / "test_map.npy"
        self.map.save_map(str(map_file))
        
        assert map_file.exists()
        
        # 加载到新地图
        new_map = OccupancyGridMap(self.config)
        new_map.load_map(str(map_file))
        
        # 验证一致性
        assert np.allclose(self.map.grid, new_map.grid)
        assert new_map.grid[50, 50] == 0.9
        assert new_map.grid[60, 60] == 0.1


def test_map_config():
    """测试地图配置"""
    config = MapConfig(
        width=200,
        height=300,
        resolution=0.05
    )
    
    assert config.width == 200
    assert config.height == 300
    assert config.resolution == 0.05


if __name__ == '__main__':
    # 运行测试
    pytest.main([__file__, '-v'])

