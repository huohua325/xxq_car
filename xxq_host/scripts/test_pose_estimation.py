"""
位姿估计测试脚本
测试POSE数据接收和可视化
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from src.communication.robot_comm import RobotComm, setup_comm_logging

class PoseVisualizer:
    """位姿可视化器"""
    
    def __init__(self):
        self.poses = []
        self.max_points = 500
        
        # 创建图形
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # 左图：轨迹
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.set_title('机器人轨迹')
        self.ax1.grid(True)
        self.ax1.axis('equal')
        self.line, = self.ax1.plot([], [], 'b-', linewidth=2, label='轨迹')
        self.current_pos, = self.ax1.plot([], [], 'ro', markersize=10, label='当前位置')
        self.ax1.legend()
        
        # 右图：位姿随时间变化
        self.ax2.set_xlabel('时间 (s)')
        self.ax2.set_ylabel('值')
        self.ax2.set_title('位姿随时间变化')
        self.ax2.grid(True)
        self.line_x, = self.ax2.plot([], [], 'r-', label='X (m)')
        self.line_y, = self.ax2.plot([], [], 'g-', label='Y (m)')
        self.line_theta, = self.ax2.plot([], [], 'b-', label='θ (deg)')
        self.ax2.legend()
        
        self.start_time = time.time()
    
    def add_pose(self, x, y, theta):
        """添加位姿数据点"""
        elapsed = time.time() - self.start_time
        self.poses.append((elapsed, x, y, theta))
        
        # 限制点数
        if len(self.poses) > self.max_points:
            self.poses.pop(0)
    
    def update(self, frame):
        """更新图形"""
        if not self.poses:
            return self.line, self.current_pos, self.line_x, self.line_y, self.line_theta
        
        # 提取数据
        times, xs, ys, thetas = zip(*self.poses)
        
        # 更新轨迹图
        self.line.set_data(xs, ys)
        self.current_pos.set_data([xs[-1]], [ys[-1]])
        
        # 自动调整范围
        if len(xs) > 1:
            x_range = max(xs) - min(xs)
            y_range = max(ys) - min(ys)
            margin = max(0.5, max(x_range, y_range) * 0.1)
            
            self.ax1.set_xlim(min(xs) - margin, max(xs) + margin)
            self.ax1.set_ylim(min(ys) - margin, max(ys) + margin)
        
        # 更新时间图
        self.line_x.set_data(times, xs)
        self.line_y.set_data(times, ys)
        thetas_deg = [np.rad2deg(t) for t in thetas]
        self.line_theta.set_data(times, thetas_deg)
        
        if len(times) > 1:
            self.ax2.set_xlim(times[0], times[-1])
            
            # 自动调整Y轴
            all_values = list(xs) + list(ys) + thetas_deg
            self.ax2.set_ylim(min(all_values) * 1.1, max(all_values) * 1.1)
        
        return self.line, self.current_pos, self.line_x, self.line_y, self.line_theta


def test_pose_basic(port='COM5'):
    """基础POSE测试：接收并打印数据"""
    print("\n" + "="*60)
    print("   位姿估计基础测试")
    print("="*60 + "\n")
    
    setup_comm_logging()
    comm = RobotComm(port=port)
    
    pose_count = 0
    
    def on_pose_update(pose_data):
        nonlocal pose_count
        pose_count += 1
        
        print(f"[{pose_count:4d}] "
              f"X: {pose_data.x:7.3f}m, "
              f"Y: {pose_data.y:7.3f}m, "
              f"θ: {pose_data.theta:6.3f}rad ({np.rad2deg(pose_data.theta):6.1f}°)")
    
    comm.on_pose_update = on_pose_update
    
    if not comm.start():
        print("❌ 通信启动失败")
        return
    
    print("✅ 开始接收POSE数据（按Ctrl+C停止）\n")
    print("提示：")
    print("  - 发送 MODE,1 让机器人前进")
    print("  - 发送 MODE,0 停止机器人")
    print("  - 发送 RESET,0,0,0 重置位姿\n")
    
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n停止测试")
    finally:
        comm.stop()


def test_pose_visualization(port='COM5'):
    """可视化测试：实时绘制轨迹"""
    print("\n" + "="*60)
    print("   位姿估计可视化测试")
    print("="*60 + "\n")
    
    setup_comm_logging()
    comm = RobotComm(port=port)
    visualizer = PoseVisualizer()
    
    def on_pose_update(pose_data):
        visualizer.add_pose(pose_data.x, pose_data.y, pose_data.theta)
    
    comm.on_pose_update = on_pose_update
    
    if not comm.start():
        print("❌ 通信启动失败")
        return
    
    print("✅ 开始可视化POSE数据")
    print("提示：发送控制命令让机器人移动，观察轨迹变化\n")
    
    # 启动动画
    ani = FuncAnimation(visualizer.fig, visualizer.update, 
                       interval=100, blit=True, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n停止测试")
    finally:
        comm.stop()


def test_pose_accuracy(port='COM5'):
    """精度测试：评估位姿估计质量"""
    print("\n" + "="*60)
    print("   位姿估计精度测试")
    print("="*60 + "\n")
    
    setup_comm_logging()
    comm = RobotComm(port=port)
    
    poses = []
    
    def on_pose_update(pose_data):
        poses.append((pose_data.x, pose_data.y, pose_data.theta))
    
    comm.on_pose_update = on_pose_update
    
    if not comm.start():
        print("❌ 通信启动失败")
        return
    
    # 测试1：静止漂移测试
    print("\n📍 测试1：静止漂移测试（30秒）")
    print("   请保持机器人静止...\n")
    
    poses.clear()
    time.sleep(30)
    
    if len(poses) > 10:
        xs, ys, thetas = zip(*poses)
        x_drift = max(xs) - min(xs)
        y_drift = max(ys) - min(ys)
        theta_drift = np.rad2deg(max(thetas) - min(thetas))
        
        print(f"   X轴漂移: {x_drift*1000:.2f} mm")
        print(f"   Y轴漂移: {y_drift*1000:.2f} mm")
        print(f"   角度漂移: {theta_drift:.2f}°")
        
        if x_drift < 0.01 and y_drift < 0.01 and theta_drift < 2:
            print("   ✅ 静止测试通过")
        else:
            print("   ⚠️  静止漂移较大，建议校准IMU")
    
    # 测试2：直线精度测试
    print("\n📍 测试2：直线精度测试")
    print("   请发送 MODE,1 让机器人前进，然后在1米处停止（MODE,0）")
    print("   按Enter开始记录起点...")
    input()
    
    # 重置位姿
    comm.reset_pose(0, 0, 0)
    time.sleep(0.5)
    
    print("   开始记录，请让机器人移动...")
    print("   到达目标后按Enter...")
    input()
    
    if poses:
        final_x = poses[-1][0]
        final_y = poses[-1][1]
        actual_distance = np.hypot(final_x, final_y)
        
        print(f"\n   测量结果：")
        print(f"   X: {final_x:.3f}m, Y: {final_y:.3f}m")
        print(f"   总距离: {actual_distance:.3f}m")
        print(f"   请用卷尺测量实际距离并输入（米）: ", end='')
        
        try:
            real_distance = float(input())
            error = abs(actual_distance - real_distance)
            error_percent = (error / real_distance) * 100
            
            print(f"\n   误差: {error*100:.1f}cm ({error_percent:.1f}%)")
            
            if error_percent < 5:
                print("   ✅ 精度测试通过")
            else:
                print("   ⚠️  误差较大，建议检查编码器参数")
        except ValueError:
            print("   跳过精度评估")
    
    comm.stop()
    print("\n测试完成！")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='位姿估计测试工具')
    parser.add_argument('--port', default='COM5', help='串口号（Windows: COM5, Linux: /dev/ttyUSB0）')
    parser.add_argument('--test', choices=['basic', 'viz', 'accuracy'], 
                       default='basic', help='测试类型')
    
    args = parser.parse_args()
    
    if args.test == 'basic':
        test_pose_basic(args.port)
    elif args.test == 'viz':
        test_pose_visualization(args.port)
    elif args.test == 'accuracy':
        test_pose_accuracy(args.port)


if __name__ == '__main__':
    main()

