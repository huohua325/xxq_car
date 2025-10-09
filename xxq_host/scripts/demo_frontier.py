"""
Frontier探索算法演示脚本
展示前沿点检测和目标选择功能
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 非交互式后端
import matplotlib.pyplot as plt

from src.slam.occupancy_map import OccupancyGridMap, MapConfig
from src.slam.frontier_detector import FrontierDetector


def create_exploration_scenario():
    """创建探索场景：部分探索的房间"""
    config = MapConfig(
        width=200,
        height=200,
        resolution=0.1,
        origin_x=100,
        origin_y=100
    )
    map_obj = OccupancyGridMap(config)
    
    print("[场景] 创建部分探索的房间...")
    
    # 模拟机器人从中心开始探索
    robot_poses = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (2.0, 1.0, np.pi/2),
        (1.0, 1.0, np.pi),
    ]
    
    # 模拟雷达扫描更新地图
    for robot_pose in robot_poses:
        # 创建模拟雷达数据
        lidar_data = create_mock_lidar_scan(robot_pose, obstacles=[])
        map_obj.update_with_lidar(lidar_data, robot_pose)
    
    # 添加一些障碍物
    for x in np.linspace(-3, 3, 30):
        add_obstacle_to_map(map_obj, x, 3.0)
        add_obstacle_to_map(map_obj, x, -3.0)
    
    for y in np.linspace(-3, 3, 30):
        add_obstacle_to_map(map_obj, -3.0, y)
        add_obstacle_to_map(map_obj, 3.0, y)
    
    return map_obj


def create_mock_lidar_scan(robot_pose, obstacles):
    """创建模拟雷达扫描数据"""
    class MockLidarData:
        def __init__(self):
            self.sectors = []
            
            # 8个方向的扇区
            for angle_deg in [0, 45, 90, 135, 180, 225, 270, 315]:
                self.sectors.append({
                    'angle_center': angle_deg,
                    'count': 10,
                    'min_dist': 2.5,  # 固定距离
                    'avg_dist': 2.5
                })
    
    return MockLidarData()


def add_obstacle_to_map(map_obj, wx, wy):
    """在地图中添加障碍物点"""
    gx, gy = map_obj.world_to_grid(wx, wy)
    if map_obj.is_valid_grid(gx, gy):
        map_obj.grid[gy, gx] = 0.9


def demo_basic_frontier_detection():
    """演示1: 基础前沿点检测"""
    print("\n=== 演示1: 基础前沿点检测 ===")
    
    map_obj = create_exploration_scenario()
    
    # 创建Frontier检测器
    detector = FrontierDetector(map_obj, min_frontier_size=3, cluster_distance=0.5)
    
    # 查找前沿点
    frontiers = detector.find_frontiers()
    
    print(f"[结果] 找到 {len(frontiers)} 个前沿簇")
    
    # 可视化
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # 显示地图
    ax.imshow(map_obj.grid, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    
    # 显示前沿点
    if frontiers:
        frontier_grid = [map_obj.world_to_grid(fx, fy) for fx, fy in frontiers]
        fx_list = [p[0] for p in frontier_grid]
        fy_list = [p[1] for p in frontier_grid]
        ax.scatter(fx_list, fy_list, c='lime', s=200, marker='X', 
                  edgecolors='black', linewidths=2, label='Frontiers', zorder=10)
    
    # 显示机器人初始位置
    robot_gx, robot_gy = map_obj.world_to_grid(0, 0)
    ax.scatter([robot_gx], [robot_gy], c='blue', s=200, marker='o',
              edgecolors='black', linewidths=2, label='Robot Start', zorder=10)
    
    ax.set_title('Basic Frontier Detection', fontsize=14, fontweight='bold')
    ax.set_xlabel('X (grid)')
    ax.set_ylabel('Y (grid)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    os.makedirs('data/test_outputs', exist_ok=True)
    plt.savefig('data/test_outputs/demo_frontier_basic.png', dpi=150, bbox_inches='tight')
    print("[保存] data/test_outputs/demo_frontier_basic.png")
    plt.close()


def demo_target_selection():
    """演示2: 目标选择策略"""
    print("\n=== 演示2: 目标选择策略 ===")
    
    map_obj = create_exploration_scenario()
    detector = FrontierDetector(map_obj, min_frontier_size=3)
    
    frontiers_info = detector.find_frontiers_with_info()
    
    if not frontiers_info:
        print("[警告] 没有找到前沿点")
        return
    
    robot_pose = (0.0, 0.0, 0.0)
    
    # 测试不同策略
    strategies = ['nearest', 'largest', 'balanced']
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    for idx, strategy in enumerate(strategies):
        ax = axes[idx]
        
        # 选择目标
        target = detector.select_best_frontier_advanced(
            frontiers_info, robot_pose, strategy=strategy
        )
        
        # 显示地图
        ax.imshow(map_obj.grid, cmap='gray_r', origin='lower', vmin=0, vmax=1)
        
        # 显示所有前沿点
        for info in frontiers_info:
            cx, cy = info['center']
            cgx, cgy = map_obj.world_to_grid(cx, cy)
            ax.scatter([cgx], [cgy], c='lightgreen', s=100, marker='X',
                      alpha=0.5, zorder=8)
        
        # 显示选中的目标
        if target:
            tgx, tgy = map_obj.world_to_grid(target[0], target[1])
            ax.scatter([tgx], [tgy], c='red', s=300, marker='*',
                      edgecolors='black', linewidths=2,
                      label=f'Selected ({strategy})', zorder=10)
        
        # 显示机器人
        robot_gx, robot_gy = map_obj.world_to_grid(robot_pose[0], robot_pose[1])
        ax.scatter([robot_gx], [robot_gy], c='blue', s=200, marker='o',
                  edgecolors='black', linewidths=2, label='Robot', zorder=10)
        
        ax.set_title(f'Strategy: {strategy}', fontsize=12, fontweight='bold')
        ax.set_xlabel('X (grid)')
        ax.set_ylabel('Y (grid)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('data/test_outputs/demo_frontier_strategies.png', dpi=150, bbox_inches='tight')
    print("[保存] data/test_outputs/demo_frontier_strategies.png")
    plt.close()


def demo_exploration_simulation():
    """演示3: 探索过程模拟"""
    print("\n=== 演示3: 探索过程模拟 ===")
    
    config = MapConfig(width=200, height=200, resolution=0.1, origin_x=100, origin_y=100)
    map_obj = OccupancyGridMap(config)
    detector = FrontierDetector(map_obj, min_frontier_size=5, cluster_distance=0.5)
    
    # 创建房间边界
    for x in np.linspace(-5, 5, 50):
        add_obstacle_to_map(map_obj, x, 5.0)
        add_obstacle_to_map(map_obj, x, -5.0)
    for y in np.linspace(-5, 5, 50):
        add_obstacle_to_map(map_obj, -5.0, y)
        add_obstacle_to_map(map_obj, 5.0, y)
    
    # 模拟探索过程
    robot_pose = [0.0, 0.0, 0.0]
    exploration_path = [(0.0, 0.0)]
    
    max_steps = 8
    step = 0
    
    fig, axes = plt.subplots(2, 4, figsize=(20, 10))
    axes = axes.flatten()
    
    while step < max_steps:
        # 更新地图（模拟扫描）
        lidar_data = create_mock_lidar_scan(tuple(robot_pose), obstacles=[])
        map_obj.update_with_lidar(lidar_data, tuple(robot_pose))
        
        # 查找前沿
        frontiers = detector.find_frontiers()
        
        # 可视化当前状态
        ax = axes[step]
        ax.imshow(map_obj.grid, cmap='gray_r', origin='lower', vmin=0, vmax=1)
        
        # 显示探索路径
        if len(exploration_path) > 1:
            path_grid = [map_obj.world_to_grid(x, y) for x, y in exploration_path]
            px = [p[0] for p in path_grid]
            py = [p[1] for p in path_grid]
            ax.plot(px, py, 'b-', linewidth=2, alpha=0.6, label='Path')
        
        # 显示前沿点
        if frontiers:
            fg = [map_obj.world_to_grid(fx, fy) for fx, fy in frontiers]
            ax.scatter([p[0] for p in fg], [p[1] for p in fg],
                      c='lime', s=100, marker='X', alpha=0.7, zorder=8)
        
        # 显示机器人
        rgx, rgy = map_obj.world_to_grid(robot_pose[0], robot_pose[1])
        ax.scatter([rgx], [rgy], c='blue', s=150, marker='o',
                  edgecolors='black', linewidths=2, zorder=10)
        
        ax.set_title(f'Step {step+1}', fontsize=10, fontweight='bold')
        ax.set_xlabel('X (grid)', fontsize=8)
        ax.set_ylabel('Y (grid)', fontsize=8)
        ax.grid(True, alpha=0.3)
        
        # 选择下一个目标
        if frontiers:
            target = detector.select_best_frontier(frontiers, tuple(robot_pose))
            if target:
                # 移动到目标（简化：直接跳到目标）
                robot_pose[0] = target[0]
                robot_pose[1] = target[1]
                exploration_path.append((target[0], target[1]))
                
                # 显示目标
                tgx, tgy = map_obj.world_to_grid(target[0], target[1])
                ax.scatter([tgx], [tgy], c='red', s=200, marker='*',
                          edgecolors='black', linewidths=2, zorder=9)
        else:
            print(f"[Step {step+1}] 没有前沿点，探索完成")
            break
        
        step += 1
    
    plt.tight_layout()
    plt.savefig('data/test_outputs/demo_frontier_exploration.png', dpi=150, bbox_inches='tight')
    print("[保存] data/test_outputs/demo_frontier_exploration.png")
    plt.close()
    
    print(f"[完成] 探索了 {step} 步")


def demo_frontier_clustering():
    """演示4: 前沿点聚类"""
    print("\n=== 演示4: 前沿点聚类 ===")
    
    config = MapConfig(width=200, height=200, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    
    # 创建多个分离的探索区域
    regions = [
        (50, 50, 15),   # (中心x, 中心y, 半径)
        (150, 50, 12),
        (50, 150, 10),
        (150, 150, 13),
    ]
    
    for cx, cy, radius in regions:
        for dy in range(-radius, radius):
            for dx in range(-radius, radius):
                if dx*dx + dy*dy < radius*radius:
                    x, y = cx + dx, cy + dy
                    if 0 <= x < 200 and 0 <= y < 200:
                        map_obj.grid[y, x] = 0.1  # 空闲
    
    # 检测前沿
    detector = FrontierDetector(map_obj, min_frontier_size=3, cluster_distance=0.5)
    frontiers_info = detector.find_frontiers_with_info()
    
    print(f"[结果] 找到 {len(frontiers_info)} 个前沿簇")
    
    # 可视化
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.imshow(map_obj.grid, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    
    # 使用不同颜色显示不同的簇
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan']
    
    for idx, info in enumerate(frontiers_info):
        color = colors[idx % len(colors)]
        
        # 显示簇中心
        cx, cy = info['center']
        cgx, cgy = map_obj.world_to_grid(cx, cy)
        ax.scatter([cgx], [cgy], c=color, s=300, marker='*',
                  edgecolors='black', linewidths=2,
                  label=f'Cluster {idx+1} (size={info["size"]})', zorder=10)
        
        # 显示簇内的点
        if info['points']:
            points_grid = [map_obj.world_to_grid(px, py) for px, py in info['points'][:20]]  # 最多显示20个点
            px = [p[0] for p in points_grid]
            py = [p[1] for p in points_grid]
            ax.scatter(px, py, c=color, s=20, alpha=0.5, zorder=8)
    
    ax.set_title('Frontier Clustering', fontsize=14, fontweight='bold')
    ax.set_xlabel('X (grid)')
    ax.set_ylabel('Y (grid)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    plt.savefig('data/test_outputs/demo_frontier_clustering.png', dpi=150, bbox_inches='tight')
    print("[保存] data/test_outputs/demo_frontier_clustering.png")
    plt.close()


def main():
    """主函数"""
    print("=" * 60)
    print("   Frontier探索算法演示")
    print("=" * 60)
    
    demos = [
        ("基础前沿点检测", demo_basic_frontier_detection),
        ("目标选择策略", demo_target_selection),
        ("探索过程模拟", demo_exploration_simulation),
        ("前沿点聚类", demo_frontier_clustering),
    ]
    
    for name, demo_func in demos:
        try:
            demo_func()
            print(f"[成功] {name} 演示完成\n")
        except Exception as e:
            print(f"[失败] {name} 演示失败: {e}\n")
            import traceback
            traceback.print_exc()
    
    print("=" * 60)
    print("所有演示完成！")
    print("输出文件保存在: data/test_outputs/")
    print("=" * 60)


if __name__ == '__main__':
    main()

