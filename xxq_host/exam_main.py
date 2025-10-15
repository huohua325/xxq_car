#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
考试专用探索脚本（基于maze适配器）
从Entrance到Exit的目标导向探索 + 返回起点
"""

import sys
import time
import argparse
import numpy as np
from typing import Tuple

# 添加项目根目录到路径
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.adapters.maze_adapter import MazeAdapter
from src.slam.occupancy_map import OccupancyGridMap
from src.slam.frontier_detector import FrontierDetector
from src.navigation.path_planner import PathPlanner
import config


def cell_to_world(cell_x: int, cell_y: int) -> Tuple[float, float]:
    """
    格子坐标转世界坐标
    
    Args:
        cell_x, cell_y: 格子坐标 (0-3)
    
    Returns:
        (world_x, world_y): 世界坐标（米）
    """
    # 格子中心 = (索引 + 0.5) * 格子尺寸
    world_x = (cell_x + 0.5) * config.CELL_SIZE
    world_y = (cell_y + 0.5) * config.CELL_SIZE
    return (world_x, world_y)


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='考试探索系统（maze适配器版）')
    parser.add_argument('--entrance-x', type=int, required=True, help='入口X (0-3)')
    parser.add_argument('--entrance-y', type=int, required=True, help='入口Y (0-3)')
    parser.add_argument('--exit-x', type=int, required=True, help='出口X (0-3)')
    parser.add_argument('--exit-y', type=int, required=True, help='出口Y (0-3)')
    parser.add_argument('--max-steps', type=int, default=50, help='最大探索步数')
    args = parser.parse_args()
    
    # 转换坐标
    entrance = cell_to_world(args.entrance_x, args.entrance_y)
    exit_pos = cell_to_world(args.exit_x, args.exit_y)
    
    print("="*80)
    print("🎓 考试探索系统启动（maze适配器版）")
    print("="*80)
    print(f"📍 入口格子: ({args.entrance_x}, {args.entrance_y}) → 世界坐标: ({entrance[0]:.2f}, {entrance[1]:.2f})m")
    print(f"🎯 出口格子: ({args.exit_x}, {args.exit_y}) → 世界坐标: ({exit_pos[0]:.2f}, {exit_pos[1]:.2f})m")
    print(f"🔢 最大步数: {args.max_steps}")
    print("="*80)
    
    # 1. 初始化适配器
    print("\n[1/4] 初始化maze适配器...")
    adapter = MazeAdapter()
    
    if not adapter.connect():
        print("❌ BLE连接失败")
        return
    print("✅ BLE已连接")
    time.sleep(1)
    
    # 2. 初始化SLAM组件
    print("\n[2/4] 初始化SLAM组件...")
    # 创建地图
    map_obj = OccupancyGridMap(
        width=config.MAP_WIDTH,
        height=config.MAP_HEIGHT,
        resolution=config.MAP_RESOLUTION
    )
    
    # Frontier检测器
    frontier_detector = FrontierDetector(
        map_obj,
        min_frontier_size=config.MIN_FRONTIER_SIZE,
        cluster_distance=config.FRONTIER_CLUSTER_DIST
    )
    
    # 路径规划器
    path_planner = PathPlanner(map_obj)
    
    print("✅ SLAM组件初始化完成")
    
    # 3. 探索到Exit
    print("\n[3/4] 开始探索（Entrance → Exit）...")
    print("🎯 目标：探索到Exit格子")
    input("▶️  按Enter开始探索...")
    
    start_time = time.time()
    step_count = 0
    exploration_path = []
    
    try:
        while step_count < args.max_steps:
            step_count += 1
            print(f"\n{'='*70}")
            print(f"🔍 探索步骤 {step_count}/{args.max_steps}")
            print(f"{'='*70}")
            
            # 获取当前位姿
            current_pose = adapter.get_pose()
            print(f"📍 位姿: x={current_pose[0]:.3f}m, y={current_pose[1]:.3f}m, θ={current_pose[2]:.2f}rad")
            exploration_path.append(current_pose)
            
            # 检查是否到达Exit
            dist_to_exit = np.hypot(exit_pos[0] - current_pose[0], exit_pos[1] - current_pose[1])
            print(f"📏 距离Exit: {dist_to_exit:.3f}m")
            
            if dist_to_exit < config.GOAL_REACHED_THRESHOLD:
                print("\n🎉 到达Exit！")
                break
            
            # 雷达扫描
            print("🔄 雷达扫描中...")
            scan = adapter.scan_lidar(timeout=config.LIDAR_SCAN_TIMEOUT)
            
            if not scan:
                print("⚠️  扫描超时，继续...")
                adapter.move_forward(0.26)
                time.sleep(0.5)
                continue
            
            print(f"✅ 扫描成功，点数: {len(scan)}")
            
            # 更新地图
            angles = np.deg2rad([p[0] for p in scan])
            distances = np.array([p[1] for p in scan])
            map_obj.update_with_lidar(angles, distances, current_pose)
            
            # 检测frontiers
            frontiers = frontier_detector.find_frontiers()
            print(f"🎯 检测到 {len(frontiers)} 个frontier点")
            
            if not frontiers:
                print("⚠️ 无frontier，直接前进")
                adapter.move_forward(0.26)
                time.sleep(0.5)
                continue
            
            # 目标导向选择frontier
            best_frontier = frontier_detector.select_best_frontier_goal_directed(
                frontiers,
                current_pose,
                exit_pos,
                alpha=config.EXPLORATION_ALPHA
            )
            
            if best_frontier is None:
                print("⚠️ 无法选择frontier，直接前进")
                adapter.move_forward(0.26)
                time.sleep(0.5)
                continue
            
            print(f"📍 选择frontier: ({best_frontier[0]:.2f}, {best_frontier[1]:.2f})")
            
            # 规划路径
            start_grid = map_obj.world_to_grid(current_pose[0], current_pose[1])
            goal_grid = map_obj.world_to_grid(best_frontier[0], best_frontier[1])
            
            path = path_planner.plan_path(start_grid, goal_grid)
            
            if path is None or len(path) < 2:
                print("⚠️ 路径规划失败，直接前进")
                adapter.move_forward(0.26)
                time.sleep(0.5)
                continue
            
            print(f"🛤️  规划路径，点数: {len(path)}")
            
            # 简化：只执行第一步（朝frontier前进）
            adapter.move_forward(0.26)
            time.sleep(0.5)
        
        if step_count >= args.max_steps:
            print(f"\n⚠️ 达到最大步数 {args.max_steps}")
    
    except KeyboardInterrupt:
        print("\n⏹️  手动停止")
    
    exploration_time = time.time() - start_time
    adapter.stop()
    
    print(f"\n✅ 探索阶段完成")
    print(f"⏱️  用时: {exploration_time:.1f}秒")
    print(f"📏 总步数: {step_count}")
    print(f"📍 最终位姿: ({current_pose[0]:.3f}, {current_pose[1]:.3f})")
    
    # 4. 返回起点（简化版）
    input("\n▶️  按Enter开始返回起点...")
    print("\n[4/4] 返回起点（Entrance）...")
    
    start_time = time.time()
    return_step = 0
    max_return_steps = 30
    
    try:
        while return_step < max_return_steps:
            return_step += 1
            print(f"\n{'='*70}")
            print(f"🏠 返回步骤 {return_step}/{max_return_steps}")
            print(f"{'='*70}")
            
            current_pose = adapter.get_pose()
            dist_to_entrance = np.hypot(entrance[0] - current_pose[0], entrance[1] - current_pose[1])
            print(f"📏 距离起点: {dist_to_entrance:.3f}m")
            
            if dist_to_entrance < config.GOAL_REACHED_THRESHOLD:
                print("\n🏁 返回起点成功！")
                break
            
            # 简化：直接朝起点前进
            adapter.move_forward(0.26)
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\n⏹️  手动停止")
    
    return_time = time.time() - start_time
    adapter.stop()
    
    print(f"\n✅ 返回阶段完成")
    print(f"⏱️  用时: {return_time:.1f}秒")
    
    # 最终统计
    print("\n" + "="*80)
    print("🎓 考试完成！")
    print("="*80)
    print(f"⏱️  探索用时: {exploration_time:.1f}秒")
    print(f"⏱️  返回用时: {return_time:.1f}秒")
    print(f"⏱️  总用时: {exploration_time + return_time:.1f}秒")
    print(f"📏 探索步数: {step_count}")
    print(f"📏 返回步数: {return_step}")
    print("="*80)
    
    # 断开连接
    print("\n🔵 断开连接...")
    adapter.disconnect()
    print("✅ 完成")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 手动退出")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()


