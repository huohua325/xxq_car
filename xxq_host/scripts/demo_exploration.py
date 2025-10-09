"""
完整探索演示
演示主控制器的自主探索流程
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from src.slam.occupancy_map import OccupancyGridMap, MapConfig
from src.navigation.controller import RobotController


def create_test_environment():
    """创建测试环境地图"""
    config = MapConfig(width=200, height=200, resolution=0.1, origin_x=100, origin_y=100)
    map_obj = OccupancyGridMap(config)
    
    # 创建房间边界
    for x in np.linspace(-8, 8, 160):
        add_obstacle(map_obj, x, 8.0)
        add_obstacle(map_obj, x, -8.0)
    for y in np.linspace(-8, 8, 160):
        add_obstacle(map_obj, -8.0, y)
        add_obstacle(map_obj, 8.0, y)
    
    # 添加内部障碍物
    # 中间竖墙
    for y in np.linspace(-3, 3, 60):
        add_obstacle(map_obj, 0.0, y)
    
    # 左侧障碍
    for x in np.linspace(-6, -4, 20):
        add_obstacle(map_obj, x, -2.0)
    
    # 右侧障碍
    for x in np.linspace(4, 6, 20):
        add_obstacle(map_obj, x, 2.0)
    
    return map_obj


def add_obstacle(map_obj, wx, wy):
    """添加障碍物"""
    gx, gy = map_obj.world_to_grid(wx, wy)
    if map_obj.is_valid_grid(gx, gy):
        map_obj.grid[gy, gx] = 0.9


def demo_simple_exploration():
    """演示1: 简单环境探索"""
    print("\n=== 演示1: 简单环境探索 ===")
    
    # 创建简单地图（只有边界）
    config = MapConfig(width=150, height=150, resolution=0.1, origin_x=75, origin_y=75)
    map_obj = OccupancyGridMap(config)
    
    # 边界
    for x in np.linspace(-5, 5, 100):
        add_obstacle(map_obj, x, 5.0)
        add_obstacle(map_obj, x, -5.0)
    for y in np.linspace(-5, 5, 100):
        add_obstacle(map_obj, -5.0, y)
        add_obstacle(map_obj, 5.0, y)
    
    # 创建控制器
    controller = RobotController(map_obj=map_obj, enable_visualization=False)
    
    # 运行探索（模拟）
    trajectory = []
    for step in range(100):
        # 更新地图
        controller.update_map_from_lidar()
        
        # 控制步骤
        controller._control_step()
        
        # 记录轨迹
        trajectory.append(tuple(controller.robot_pose[:2]))
        
        # 检查完成
        if controller.state.value == 4:  # COMPLETED
            break
    
    print(f"[结果] 探索步数: {step + 1}")
    
    # 可视化
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(map_obj.grid, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    
    # 显示轨迹
    if trajectory:
        traj_grid = [map_obj.world_to_grid(x, y) for x, y in trajectory]
        tx = [p[0] for p in traj_grid]
        ty = [p[1] for p in traj_grid]
        ax.plot(tx, ty, 'b-', linewidth=2, alpha=0.7, label='Trajectory')
        ax.scatter(tx[0], ty[0], c='green', s=200, marker='o', label='Start', zorder=10)
        ax.scatter(tx[-1], ty[-1], c='red', s=200, marker='*', label='End', zorder=10)
    
    ax.set_title('Simple Exploration', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    os.makedirs('data/test_outputs', exist_ok=True)
    plt.savefig('data/test_outputs/demo_simple_exploration.png', dpi=150, bbox_inches='tight')
    print("[保存] data/test_outputs/demo_simple_exploration.png")
    plt.close()


def demo_complex_exploration():
    """演示2: 复杂环境探索"""
    print("\n=== 演示2: 复杂环境探索 ===")
    
    # 创建复杂环境
    map_obj = create_test_environment()
    
    # 创建控制器
    controller = RobotController(map_obj=map_obj, enable_visualization=False)
    
    # 运行探索
    trajectory = []
    paths = []
    
    for step in range(200):
        # 更新地图
        controller.update_map_from_lidar()
        
        # 控制步骤
        controller._control_step()
        
        # 记录轨迹和路径
        trajectory.append(tuple(controller.robot_pose[:2]))
        if controller.current_path:
            paths.append(controller.current_path.copy())
        
        # 检查完成
        if controller.state.value == 4:  # COMPLETED
            break
    
    print(f"[结果] 探索步数: {step + 1}")
    
    # 可视化
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.imshow(map_obj.grid, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    
    # 显示规划路径（半透明）
    if paths:
        for path in paths[-5:]:  # 只显示最后5条
            path_grid = [map_obj.world_to_grid(x, y) for x, y in path]
            px = [p[0] for p in path_grid]
            py = [p[1] for p in path_grid]
            ax.plot(px, py, 'g--', linewidth=1, alpha=0.3)
    
    # 显示实际轨迹
    if trajectory:
        traj_grid = [map_obj.world_to_grid(x, y) for x, y in trajectory]
        tx = [p[0] for p in traj_grid]
        ty = [p[1] for p in traj_grid]
        ax.plot(tx, ty, 'b-', linewidth=2, label='Actual Trajectory')
        ax.scatter(tx[0], ty[0], c='green', s=200, marker='o', 
                  edgecolors='black', linewidths=2, label='Start', zorder=10)
        ax.scatter(tx[-1], ty[-1], c='red', s=200, marker='*',
                  edgecolors='black', linewidths=2, label='End', zorder=10)
    
    stats = map_obj.get_statistics()
    ax.set_title(f'Complex Exploration (探索率: {stats["explored_ratio"]*100:.1f}%)', 
                fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    plt.savefig('data/test_outputs/demo_complex_exploration.png', dpi=150, bbox_inches='tight')
    print("[保存] data/test_outputs/demo_complex_exploration.png")
    plt.close()


def demo_state_machine():
    """演示3: 状态机转换"""
    print("\n=== 演示3: 状态机转换 ===")
    
    config = MapConfig(width=100, height=100, resolution=0.1)
    map_obj = OccupancyGridMap(config)
    
    controller = RobotController(map_obj=map_obj, enable_visualization=False)
    
    state_history = []
    
    for step in range(50):
        controller.update_map_from_lidar()
        controller._control_step()
        state_history.append(controller.state.name)
        
        if controller.state.value == 4:
            break
    
    # 统计状态分布
    from collections import Counter
    state_counts = Counter(state_history)
    
    print(f"[状态分布]")
    for state, count in state_counts.items():
        print(f"  {state}: {count} 次 ({count/len(state_history)*100:.1f}%)")


def main():
    """主函数"""
    print("=" * 60)
    print("   完整探索演示")
    print("=" * 60)
    
    demos = [
        ("简单环境探索", demo_simple_exploration),
        ("复杂环境探索", demo_complex_exploration),
        ("状态机转换", demo_state_machine),
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

