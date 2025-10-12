"""
可视化系统演示脚本
展示MapVisualizer的各种功能
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
from src.slam.occupancy_map import OccupancyGridMap, MapConfig
from src.visualization.map_visualizer import MapVisualizer
import config
import time


def create_test_map():
    """创建测试地图"""
    map_config = MapConfig(
        width=500,
        height=500,
        resolution=0.1,
        origin_x=250,
        origin_y=250
    )
    map_obj = OccupancyGridMap(map_config)
    
    # 模拟一个方形房间（5m x 5m）
    print("[演示] 创建测试环境...")
    
    # 添加四面墙
    wall_points = []
    
    # 上墙 (y=2.5)
    for x in np.linspace(-2.5, 2.5, 50):
        wall_points.append((x, 2.5))
    
    # 下墙 (y=-2.5)
    for x in np.linspace(-2.5, 2.5, 50):
        wall_points.append((x, -2.5))
    
    # 左墙 (x=-2.5)
    for y in np.linspace(-2.5, 2.5, 50):
        wall_points.append((-2.5, y))
    
    # 右墙 (x=2.5)
    for y in np.linspace(-2.5, 2.5, 50):
        wall_points.append((2.5, y))
    
    # 添加中间的障碍物
    for x in np.linspace(-1.0, 1.0, 20):
        wall_points.append((x, 0.0))
    
    # 模拟雷达扫描更新地图
    robot_pose = (0.0, 0.0, 0.0)  # 起始位置
    
    # 创建模拟的LidarData对象
    class MockLidarData:
        def __init__(self, obstacles):
            self.sectors = []
            
            # 将障碍物点转换为8个扇区
            for angle_deg in [0, 45, 90, 135, 180, 225, 270, 315]:
                angle_rad = np.deg2rad(angle_deg)
                
                # 查找该方向最近的障碍物
                min_dist = 10.0
                for ox, oy in obstacles:
                    dist = np.hypot(ox - robot_pose[0], oy - robot_pose[1])
                    # 计算角度差
                    point_angle = np.arctan2(oy - robot_pose[1], ox - robot_pose[0])
                    angle_diff = abs(np.rad2deg(point_angle - angle_rad))
                    
                    if angle_diff < 22.5 and dist < min_dist:
                        min_dist = dist
                
                if min_dist < 10.0:
                    self.sectors.append({
                        'angle_center': angle_deg,
                        'count': 10,
                        'min_dist': min_dist,
                        'avg_dist': min_dist
                    })
    
    lidar_data = MockLidarData(wall_points)
    map_obj.update_with_lidar(lidar_data, robot_pose)
    
    print(f"[演示] 地图创建完成，障碍物点数: {len(wall_points)}")
    return map_obj, wall_points


def demo_basic_visualization():
    """演示1: 基础可视化功能"""
    print("\n=== 演示1: 基础可视化功能 ===")
    
    map_obj, obstacles = create_test_map()
    
    # ✅ 创建可视化器（使用config参数）
    visualizer = MapVisualizer(map_obj)
    
    # 机器人位姿
    robot_pose = (0.0, 0.0, np.deg2rad(45))
    
    # 模拟雷达点云
    lidar_points = obstacles[:20]  # 取部分点
    
    # 模拟路径
    path = [
        (-2.0, -2.0),
        (-1.0, -1.0),
        (0.0, 0.0),
        (1.0, 1.0),
        (2.0, 2.0)
    ]
    
    # 模拟前沿点
    frontiers = [
        (1.5, 1.5),
        (-1.5, 1.5),
        (1.5, -1.5),
        (-1.5, -1.5)
    ]
    
    # 更新显示
    visualizer.update(
        robot_pose=robot_pose,
        lidar_points=lidar_points,
        path=path,
        frontiers=frontiers,
        velocity=(0.5, 0.2)
    )
    
    # 设置点击回调
    def on_click(x, y):
        print(f"[回调] 用户点击: ({x:.2f}, {y:.2f})")
    
    visualizer.set_click_callback(on_click)
    
    print("[演示] 基础可视化完成，按任意键关闭...")
    print("提示: 按 'h' 查看快捷键帮助")
    
    plt.show()


def demo_trajectory_animation():
    """演示2: 轨迹动画"""
    print("\n=== 演示2: 轨迹动画 ===")
    
    map_obj, obstacles = create_test_map()
    
    # ✅ 创建可视化器（使用config参数）
    visualizer = MapVisualizer(map_obj)
    
    # 模拟机器人沿圆形轨迹运动
    print("[演示] 开始轨迹动画...")
    
    radius = 1.5
    num_steps = 100
    
    plt.ion()  # 交互模式
    
    for i in range(num_steps):
        angle = 2 * np.pi * i / num_steps
        
        robot_x = radius * np.cos(angle)
        robot_y = radius * np.sin(angle)
        robot_theta = angle + np.pi/2
        
        robot_pose = (robot_x, robot_y, robot_theta)
        
        # 模拟雷达点云（取附近的障碍物）
        lidar_points = []
        for ox, oy in obstacles:
            dist = np.hypot(ox - robot_x, oy - robot_y)
            if dist < 3.0:
                lidar_points.append((ox, oy))
        
        # 更新显示
        visualizer.update(
            robot_pose=robot_pose,
            lidar_points=lidar_points[:30],
            velocity=(0.3, 0.1)
        )
        
        plt.pause(0.05)
    
    plt.ioff()
    print("[演示] 轨迹动画完成")
    plt.show()


def demo_multi_window():
    """演示3: 多窗口布局"""
    print("\n=== 演示3: 多窗口布局 ===")
    
    map_obj, obstacles = create_test_map()
    
    # ✅ 创建多窗口可视化器（使用config参数，但演示时临时覆盖）
    visualizer = MapVisualizer(map_obj, figsize=(15, 12), enable_multi_window=True)
    
    print("[演示] 多窗口布局创建完成")
    
    # 模拟数据更新
    plt.ion()
    
    for i in range(50):
        angle = 2 * np.pi * i / 50
        robot_x = 1.5 * np.cos(angle)
        robot_y = 1.5 * np.sin(angle)
        robot_theta = angle + np.pi/2
        
        robot_pose = (robot_x, robot_y, robot_theta)
        velocity = (0.3 + 0.1*np.sin(angle), 0.2*np.cos(angle))
        
        visualizer.update(
            robot_pose=robot_pose,
            velocity=velocity
        )
        
        plt.pause(0.1)
    
    plt.ioff()
    plt.show()


def demo_interactive_features():
    """演示4: 交互功能"""
    print("\n=== 演示4: 交互功能 ===")
    print("""
交互说明：
  - 左键点击地图设置目标点
  - 按 's' 保存地图
  - 按 'r' 重置视图
  - 按 'g' 切换网格显示
  - 按 'l' 切换雷达显示
  - 按 't' 切换轨迹显示
  - 按 'h' 显示帮助
  - 按 'q' 退出
    """)
    
    map_obj, obstacles = create_test_map()
    visualizer = MapVisualizer(map_obj)
    
    # 添加点击回调
    target_points = []
    
    def on_target_click(x, y):
        target_points.append((x, y))
        print(f"[目标] 设置目标点: ({x:.2f}, {y:.2f})")
        
        # 更新显示
        visualizer.update(
            robot_pose=(0, 0, 0),
            path=target_points
        )
    
    visualizer.set_click_callback(on_target_click)
    
    # 初始显示
    visualizer.update(robot_pose=(0, 0, 0))
    
    print("[演示] 交互模式启动，请尝试交互功能...")
    plt.show()


def demo_performance_test():
    """演示5: 性能测试"""
    print("\n=== 演示5: 性能测试 ===")
    
    map_obj, obstacles = create_test_map()
    visualizer = MapVisualizer(map_obj)
    
    print("[演示] 开始性能测试（100帧）...")
    
    plt.ion()
    
    start_time = time.time()
    num_frames = 100
    
    for i in range(num_frames):
        angle = 2 * np.pi * i / num_frames
        
        robot_pose = (
            1.5 * np.cos(angle),
            1.5 * np.sin(angle),
            angle + np.pi/2
        )
        
        # 大量雷达点（性能测试）
        lidar_points = obstacles[:100]
        
        visualizer.update(
            robot_pose=robot_pose,
            lidar_points=lidar_points,
            velocity=(0.5, 0.2)
        )
        
        plt.pause(0.001)  # 最小延迟
    
    elapsed = time.time() - start_time
    fps = num_frames / elapsed
    
    plt.ioff()
    
    print(f"[性能] 总帧数: {num_frames}")
    print(f"[性能] 总耗时: {elapsed:.2f}秒")
    print(f"[性能] 平均帧率: {fps:.1f} FPS")
    print(f"[性能] {'✓ 通过' if fps >= 20 else '✗ 未达标'} (目标: 20+ FPS)")
    
    plt.show()


def main():
    """主函数"""
    print("=" * 60)
    print("   xxq智能小车 - 可视化系统演示")
    print("=" * 60)
    
    demos = {
        '1': ('基础可视化功能', demo_basic_visualization),
        '2': ('轨迹动画', demo_trajectory_animation),
        '3': ('多窗口布局', demo_multi_window),
        '4': ('交互功能', demo_interactive_features),
        '5': ('性能测试', demo_performance_test),
    }
    
    print("\n请选择演示：")
    for key, (name, _) in demos.items():
        print(f"  {key}. {name}")
    print("  0. 全部运行")
    print("  q. 退出")
    
    choice = input("\n请输入选择 [1-5/0/q]: ").strip()
    
    if choice == 'q':
        print("退出演示")
        return
    
    if choice == '0':
        # 运行所有演示
        for name, func in demos.values():
            func()
    elif choice in demos:
        # 运行单个演示
        _, func = demos[choice]
        func()
    else:
        print("无效选择")


if __name__ == '__main__':
    main()

