"""
路径规划算法演示脚本
展示A*路径规划、DWA避障和路径平滑功能
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 非交互式后端
import matplotlib.pyplot as plt

from src.slam.occupancy_map import OccupancyGridMap, MapConfig
from src.navigation.path_planner import PathPlanner, PathPlannerConfig
from src.navigation.dwa import DWA, DWAConfig


def create_test_map():
    """创建测试地图（带障碍物的房间）"""
    config = MapConfig(
        width=200,
        height=200,
        resolution=0.1,
        origin_x=100,
        origin_y=100
    )
    map_obj = OccupancyGridMap(config)
    
    # 创建房间边界
    for x in np.linspace(-5, 5, 100):
        add_obstacle(map_obj, x, 5.0)
        add_obstacle(map_obj, x, -5.0)
    for y in np.linspace(-5, 5, 100):
        add_obstacle(map_obj, -5.0, y)
        add_obstacle(map_obj, 5.0, y)
    
    # 设置内部为空闲
    for y in range(50, 150):
        for x in range(50, 150):
            map_obj.grid[y, x] = 0.1
    
    return map_obj


def add_obstacle(map_obj, wx, wy):
    """在地图中添加障碍物点"""
    gx, gy = map_obj.world_to_grid(wx, wy)
    if map_obj.is_valid_grid(gx, gy):
        map_obj.grid[gy, gx] = 0.9


def demo_astar_basic():
    """演示1: 基础A*路径规划"""
    print("\n=== 演示1: 基础A*路径规划 ===")
    
    map_obj = create_test_map()
    planner = PathPlanner(map_obj)
    
    # 规划路径
    start = (-3.0, -3.0)
    goal = (3.0, 3.0)
    
    path = planner.plan_path(start, goal, inflate_obstacles=False, smooth=False)
    
    if not path:
        print("[警告] 未找到路径")
        return
    
    print(f"[结果] 找到路径: {len(path)} 个点")
    print(f"[结果] 路径长度: {planner.get_path_length(path):.2f}m")
    
    # 可视化
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # 显示地图
    ax.imshow(map_obj.grid, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    
    # 显示路径
    path_grid = [map_obj.world_to_grid(x, y) for x, y in path]
    px = [p[0] for p in path_grid]
    py = [p[1] for p in path_grid]
    ax.plot(px, py, 'b-', linewidth=3, label='Path', zorder=8)
    ax.scatter(px, py, c='blue', s=30, zorder=9)
    
    # 显示起点和终点
    start_grid = map_obj.world_to_grid(*start)
    goal_grid = map_obj.world_to_grid(*goal)
    ax.scatter([start_grid[0]], [start_grid[1]], c='green', s=200, marker='o',
              edgecolors='black', linewidths=2, label='Start', zorder=10)
    ax.scatter([goal_grid[0]], [goal_grid[1]], c='red', s=200, marker='*',
              edgecolors='black', linewidths=2, label='Goal', zorder=10)
    
    ax.set_title('A* Path Planning (Basic)', fontsize=14, fontweight='bold')
    ax.set_xlabel('X (grid)')
    ax.set_ylabel('Y (grid)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    os.makedirs('data/test_outputs', exist_ok=True)
    plt.savefig('data/test_outputs/demo_astar_basic.png', dpi=150, bbox_inches='tight')
    print("[保存] data/test_outputs/demo_astar_basic.png")
    plt.close()


def demo_path_smoothing():
    """演示2: 路径平滑"""
    print("\n=== 演示2: 路径平滑 ===")
    
    map_obj = create_test_map()
    planner = PathPlanner(map_obj)
    
    start = (-3.0, -3.0)
    goal = (3.0, 3.0)
    
    # 原始路径（不平滑）
    path_raw = planner.plan_path(start, goal, inflate_obstacles=False, smooth=False)
    
    # 平滑路径
    path_smooth = planner.smooth_path(path_raw) if path_raw else None
    
    if not path_raw or not path_smooth:
        print("[警告] 未找到路径")
        return
    
    print(f"[原始] 路径点数: {len(path_raw)}, 长度: {planner.get_path_length(path_raw):.2f}m")
    print(f"[平滑] 路径点数: {len(path_smooth)}, 长度: {planner.get_path_length(path_smooth):.2f}m")
    
    # 可视化
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    
    for idx, (path, title) in enumerate([(path_raw, 'Raw Path'), (path_smooth, 'Smoothed Path')]):
        ax = axes[idx]
        
        # 显示地图
        ax.imshow(map_obj.grid, cmap='gray_r', origin='lower', vmin=0, vmax=1)
        
        # 显示路径
        path_grid = [map_obj.world_to_grid(x, y) for x, y in path]
        px = [p[0] for p in path_grid]
        py = [p[1] for p in path_grid]
        ax.plot(px, py, 'b-', linewidth=3, label=f'{len(path)} points', zorder=8)
        ax.scatter(px, py, c='blue', s=30, zorder=9)
        
        # 起点和终点
        start_grid = map_obj.world_to_grid(*start)
        goal_grid = map_obj.world_to_grid(*goal)
        ax.scatter([start_grid[0]], [start_grid[1]], c='green', s=200, marker='o',
                  edgecolors='black', linewidths=2, label='Start', zorder=10)
        ax.scatter([goal_grid[0]], [goal_grid[1]], c='red', s=200, marker='*',
                  edgecolors='black', linewidths=2, label='Goal', zorder=10)
        
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.set_xlabel('X (grid)')
        ax.set_ylabel('Y (grid)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('data/test_outputs/demo_path_smoothing.png', dpi=150, bbox_inches='tight')
    print("[保存] data/test_outputs/demo_path_smoothing.png")
    plt.close()


def demo_obstacle_avoidance():
    """演示3: 障碍物避障"""
    print("\n=== 演示3: 障碍物避障 ===")
    
    map_obj = create_test_map()
    
    # 添加中间障碍物
    for y in np.linspace(-2, 2, 40):
        add_obstacle(map_obj, 0.0, y)
    
    planner = PathPlanner(map_obj)
    
    start = (-3.0, 0.0)
    goal = (3.0, 0.0)
    
    # 不膨胀障碍物
    path_no_inflation = planner.plan_path(start, goal, inflate_obstacles=False, smooth=True)
    
    # 膨胀障碍物
    path_with_inflation = planner.plan_path(start, goal, inflate_obstacles=True, smooth=True)
    
    # 可视化
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    
    for idx, (path, title, inflate) in enumerate([
        (path_no_inflation, 'Without Inflation', False),
        (path_with_inflation, 'With Inflation', True)
    ]):
        ax = axes[idx]
        
        # 准备显示的地图
        display_map = map_obj.grid.copy()
        if inflate:
            obstacle_map = map_obj.grid >= 0.7
            inflated = planner.inflate_obstacles(obstacle_map, radius=2)
            display_map = np.where(inflated, 0.9, display_map)
        
        # 显示地图
        ax.imshow(display_map, cmap='gray_r', origin='lower', vmin=0, vmax=1)
        
        # 显示路径
        if path:
            path_grid = [map_obj.world_to_grid(x, y) for x, y in path]
            px = [p[0] for p in path_grid]
            py = [p[1] for p in path_grid]
            ax.plot(px, py, 'b-', linewidth=3, label=f'Path ({len(path)} pts)', zorder=8)
        
        # 起点和终点
        start_grid = map_obj.world_to_grid(*start)
        goal_grid = map_obj.world_to_grid(*goal)
        ax.scatter([start_grid[0]], [start_grid[1]], c='green', s=200, marker='o',
                  edgecolors='black', linewidths=2, label='Start', zorder=10)
        ax.scatter([goal_grid[0]], [goal_grid[1]], c='red', s=200, marker='*',
                  edgecolors='black', linewidths=2, label='Goal', zorder=10)
        
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.set_xlabel('X (grid)')
        ax.set_ylabel('Y (grid)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('data/test_outputs/demo_obstacle_avoidance.png', dpi=150, bbox_inches='tight')
    print("[保存] data/test_outputs/demo_obstacle_avoidance.png")
    plt.close()


def demo_dwa_local_planning():
    """演示4: DWA局部规划"""
    print("\n=== 演示4: DWA局部规划 ===")
    
    dwa = DWA()
    
    # 模拟机器人移动到目标
    robot_state = [0.0, 0.0, 0.0, 0.0, 0.0]  # [x, y, theta, v, omega]
    goal = (5.0, 5.0)
    
    trajectory = []
    max_steps = 50
    
    for step in range(max_steps):
        x, y, theta, v, omega = robot_state
        trajectory.append((x, y))
        
        # 检查是否到达目标
        dist = np.hypot(x - goal[0], y - goal[1])
        if dist < 0.2:
            break
        
        # DWA规划
        v_new, omega_new = dwa.plan(tuple(robot_state), goal)
        
        # 更新状态
        robot_state[0] += v_new * np.cos(theta) * dwa.config.dt
        robot_state[1] += v_new * np.sin(theta) * dwa.config.dt
        robot_state[2] += omega_new * dwa.config.dt
        robot_state[2] = np.arctan2(np.sin(robot_state[2]), np.cos(robot_state[2]))
        robot_state[3] = v_new
        robot_state[4] = omega_new
    
    print(f"[结果] DWA轨迹: {len(trajectory)} 个点，{step+1} 步到达")
    
    # 可视化
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # 显示轨迹
    tx = [p[0] for p in trajectory]
    ty = [p[1] for p in trajectory]
    ax.plot(tx, ty, 'b-', linewidth=3, label='DWA Trajectory', zorder=8)
    ax.scatter(tx, ty, c='blue', s=20, alpha=0.6, zorder=9)
    
    # 显示起点和终点
    ax.scatter([0.0], [0.0], c='green', s=300, marker='o',
              edgecolors='black', linewidths=3, label='Start', zorder=10)
    ax.scatter([goal[0]], [goal[1]], c='red', s=300, marker='*',
              edgecolors='black', linewidths=3, label='Goal', zorder=10)
    
    # 显示方向箭头
    for i in range(0, len(trajectory), 5):
        if i >= len(trajectory) - 1:
            break
        dx = trajectory[i+1][0] - trajectory[i][0]
        dy = trajectory[i+1][1] - trajectory[i][1]
        ax.arrow(trajectory[i][0], trajectory[i][1], dx, dy,
                head_width=0.2, head_length=0.15, fc='orange', ec='orange',
                alpha=0.6, zorder=7)
    
    ax.set_title('DWA Local Planning', fontsize=14, fontweight='bold')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend(loc='upper left', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.savefig('data/test_outputs/demo_dwa_planning.png', dpi=150, bbox_inches='tight')
    print("[保存] data/test_outputs/demo_dwa_planning.png")
    plt.close()


def demo_integrated_navigation():
    """演示5: A* + DWA集成导航"""
    print("\n=== 演示5: A* + DWA集成导航 ===")
    
    # 创建地图
    map_obj = create_test_map()
    
    # 添加障碍物
    for x in np.linspace(-1, 1, 20):
        add_obstacle(map_obj, x, 0.5)
    
    # A*全局规划
    planner = PathPlanner(map_obj)
    start = (-3.0, -3.0)
    goal = (3.0, 3.0)
    global_path = planner.plan_path(start, goal, inflate_obstacles=True, smooth=True)
    
    if not global_path:
        print("[警告] 未找到全局路径")
        return
    
    print(f"[A*] 全局路径: {len(global_path)} 个点")
    
    # DWA局部跟随（简化版本）
    dwa = DWA()
    robot_state = list(start) + [0.0, 0.0, 0.0]  # [x, y, theta, v, omega]
    actual_trajectory = [start]
    
    path_index = 1
    max_steps = 100
    
    for step in range(max_steps):
        x, y = robot_state[0], robot_state[1]
        
        # 检查是否到达全局路径的当前目标点
        if path_index < len(global_path):
            local_goal = global_path[path_index]
            dist = np.hypot(x - local_goal[0], y - local_goal[1])
            
            if dist < 0.3:
                path_index += 1
            
            # DWA规划到局部目标
            v, omega = dwa.plan(tuple(robot_state), local_goal)
            
            # 更新状态
            theta = robot_state[2]
            robot_state[0] += v * np.cos(theta) * dwa.config.dt
            robot_state[1] += v * np.sin(theta) * dwa.config.dt
            robot_state[2] += omega * dwa.config.dt
            robot_state[2] = np.arctan2(np.sin(robot_state[2]), np.cos(robot_state[2]))
            robot_state[3] = v
            robot_state[4] = omega
            
            actual_trajectory.append((robot_state[0], robot_state[1]))
        else:
            break
    
    print(f"[DWA] 实际轨迹: {len(actual_trajectory)} 个点")
    
    # 可视化
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # 显示地图
    ax.imshow(map_obj.grid, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    
    # 显示全局路径
    global_path_grid = [map_obj.world_to_grid(x, y) for x, y in global_path]
    gpx = [p[0] for p in global_path_grid]
    gpy = [p[1] for p in global_path_grid]
    ax.plot(gpx, gpy, 'g--', linewidth=2, label='A* Global Path', alpha=0.7, zorder=7)
    ax.scatter(gpx, gpy, c='green', s=50, alpha=0.5, zorder=8)
    
    # 显示实际轨迹
    actual_grid = [map_obj.world_to_grid(x, y) for x, y in actual_trajectory]
    atx = [p[0] for p in actual_grid]
    aty = [p[1] for p in actual_grid]
    ax.plot(atx, aty, 'b-', linewidth=3, label='DWA Actual Trajectory', zorder=9)
    
    # 起点和终点
    start_grid = map_obj.world_to_grid(*start)
    goal_grid = map_obj.world_to_grid(*goal)
    ax.scatter([start_grid[0]], [start_grid[1]], c='cyan', s=300, marker='o',
              edgecolors='black', linewidths=3, label='Start', zorder=10)
    ax.scatter([goal_grid[0]], [goal_grid[1]], c='red', s=300, marker='*',
              edgecolors='black', linewidths=3, label='Goal', zorder=10)
    
    ax.set_title('Integrated Navigation (A* + DWA)', fontsize=14, fontweight='bold')
    ax.set_xlabel('X (grid)')
    ax.set_ylabel('Y (grid)')
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.savefig('data/test_outputs/demo_integrated_navigation.png', dpi=150, bbox_inches='tight')
    print("[保存] data/test_outputs/demo_integrated_navigation.png")
    plt.close()


def main():
    """主函数"""
    print("=" * 60)
    print("   路径规划算法演示")
    print("=" * 60)
    
    demos = [
        ("基础A*路径规划", demo_astar_basic),
        ("路径平滑", demo_path_smoothing),
        ("障碍物避障", demo_obstacle_avoidance),
        ("DWA局部规划", demo_dwa_local_planning),
        ("A* + DWA集成导航", demo_integrated_navigation),
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

