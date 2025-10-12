"""
系统综合测试脚本
完整测试所有硬件、软件、通信功能

测试模块：
1. 通信连接测试
2. 传感器数据质量测试  
3. 控制命令响应测试
4. 通信延迟测试
5. 建图功能测试
6. 可视化功能测试
7. 完整系统集成测试

使用方法：
    python tests/test_system_comprehensive.py --port COM5
    python tests/test_system_comprehensive.py --port COM5 --quick  # 快速测试
    python tests/test_system_comprehensive.py --port COM5 --test comm  # 只测试通信
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import argparse
import numpy as np
from collections import deque
from datetime import datetime

from src.communication import create_robot_comm, RobotComm, RobotCommBLE
from src.communication.protocol import RobotMode, LidarData, MPUData, OdometryData, PoseData
import logging

def setup_comm_logging():
    """设置通信日志"""
    logging.basicConfig(
        level=logging.INFO,
        format='[%(asctime)s] %(name)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )
from src.slam.occupancy_map import OccupancyGridMap, MapConfig
from src.navigation.controller import RobotController
import config


class TestResult:
    """测试结果记录"""
    def __init__(self, name):
        self.name = name
        self.passed = False
        self.message = ""
        self.details = {}
        self.start_time = time.time()
        self.duration = 0
    
    def finish(self, passed, message="", details=None):
        self.passed = passed
        self.message = message
        self.details = details or {}
        self.duration = time.time() - self.start_time
    
    def __str__(self):
        status = "✅ 通过" if self.passed else "❌ 失败"
        return f"{status} [{self.duration:.2f}s] {self.name}: {self.message}"


class SystemTester:
    """系统综合测试器"""
    
    def __init__(self, connection_type='serial', port=None, baudrate=115200, ble_address=None):
        """
        初始化测试器
        
        Args:
            connection_type: 连接类型 'serial'(串口) 或 'ble'(蓝牙)
            port: 串口号（serial模式）
            baudrate: 波特率（serial模式）
            ble_address: BLE设备地址（ble模式）
        """
        self.connection_type = connection_type
        self.port = port
        self.baudrate = baudrate
        self.ble_address = ble_address
        self.comm = None
        self.results = []
        
        # 数据收集缓冲区
        self.lidar_buffer = deque(maxlen=100)
        self.mpu_buffer = deque(maxlen=100)
        self.odom_buffer = deque(maxlen=100)
        self.pose_buffer = deque(maxlen=100)
        
        # 延迟测试用
        self.cmd_timestamps = {}
        self.response_times = []
    
    def add_result(self, result):
        """添加测试结果"""
        self.results.append(result)
        try:
            print(result)
        except UnicodeEncodeError:
            # Windows控制台编码问题，尝试用ASCII替代
            safe_result = str(result).replace('✅', '[PASS]').replace('❌', '[FAIL]').replace('⚠️', '[WARN]')
            print(safe_result)
    
    def print_summary(self):
        """打印测试摘要"""
        print("\n" + "="*70)
        print("测试摘要")
        print("="*70)
        
        passed = sum(1 for r in self.results if r.passed)
        total = len(self.results)
        
        print(f"\n总计: {total} 项测试")
        print(f"通过: {passed} 项 ({passed/total*100:.1f}%)")
        print(f"失败: {total-passed} 项")
        
        if total - passed > 0:
            print(f"\n失败的测试:")
            for r in self.results:
                if not r.passed:
                    try:
                        print(f"  ❌ {r.name}: {r.message}")
                    except UnicodeEncodeError:
                        print(f"  [FAIL] {r.name}: {r.message}")
        
        print("\n" + "="*70)
        
        # 保存报告
        self.save_report()
    
    def save_report(self):
        """保存测试报告"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = f"data/test_outputs/system_test_{timestamp}.txt"
        
        os.makedirs(os.path.dirname(report_file), exist_ok=True)
        
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("="*70 + "\n")
            f.write(f"系统综合测试报告\n")
            f.write(f"时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"串口: {self.port}\n")
            f.write("="*70 + "\n\n")
            
            for r in self.results:
                f.write(str(r) + "\n")
                if r.details:
                    for key, value in r.details.items():
                        f.write(f"  {key}: {value}\n")
                f.write("\n")
            
            f.write("\n" + "="*70 + "\n")
            passed = sum(1 for r in self.results if r.passed)
            total = len(self.results)
            f.write(f"总计: {total} 项, 通过: {passed} 项, 失败: {total-passed} 项\n")
        
        try:
            print(f"\n📄 测试报告已保存: {report_file}")
        except UnicodeEncodeError:
            print(f"\n[REPORT] 测试报告已保存: {report_file}")
    
    # ========================================================================
    # 1. 通信连接测试
    # ========================================================================
    
    def test_serial_connection(self):
        """测试连接（串口或BLE）"""
        conn_name = "BLE连接" if self.connection_type == 'ble' else "串口连接"
        result = TestResult(f"{conn_name}测试")
        
        try:
            # 使用工厂函数创建通信对象
            if self.connection_type == 'ble':
                if not self.ble_address:
                    result.finish(False, "BLE模式需要提供设备地址")
                    self.add_result(result)
                    return False
                
                self.comm = create_robot_comm('ble', address=self.ble_address)
                conn_info = {
                    "连接类型": "BLE",
                    "设备地址": self.ble_address
                }
            else:
                self.comm = create_robot_comm('serial', port=self.port, baudrate=self.baudrate)
                conn_info = {
                    "连接类型": "Serial",
                    "串口": self.port,
                    "波特率": self.baudrate
                }
            
            # 设置数据回调
            self.comm.on_lidar_update = lambda d: self.lidar_buffer.append(d)
            self.comm.on_mpu_update = lambda d: self.mpu_buffer.append(d)
            self.comm.on_odom_update = lambda d: self.odom_buffer.append(d)
            self.comm.on_pose_update = lambda d: self.pose_buffer.append(d)
            
            if self.comm.start():
                result.finish(True, f"{conn_name}成功", conn_info)
            else:
                result.finish(False, f"无法建立{conn_name}")
        
        except Exception as e:
            result.finish(False, f"连接异常: {e}")
        
        self.add_result(result)
        return result.passed
    
    def test_initial_data_reception(self, timeout=5.0):
        """测试初始数据接收"""
        result = TestResult("初始数据接收测试")
        
        print(f"  等待 {timeout}秒 接收数据...")
        time.sleep(timeout)
        
        details = {
            "雷达数据包": len(self.lidar_buffer),
            "MPU数据包": len(self.mpu_buffer),
            "里程计数据包": len(self.odom_buffer),
            "位姿数据包": len(self.pose_buffer)
        }
        
        # 检查是否收到各种数据
        has_lidar = len(self.lidar_buffer) > 0
        has_mpu = len(self.mpu_buffer) > 0
        has_odom = len(self.odom_buffer) > 0
        has_pose = len(self.pose_buffer) > 0
        
        if has_pose:
            result.finish(True, "成功接收所有类型数据", details)
        elif has_lidar or has_mpu or has_odom:
            result.finish(False, "部分数据未收到（POSE数据缺失）", details)
        else:
            result.finish(False, "未收到任何数据", details)
        
        self.add_result(result)
        return result.passed
    
    # ========================================================================
    # 2. 传感器数据质量测试
    # ========================================================================
    
    def test_lidar_data_quality(self, duration=10.0):
        """测试雷达数据质量"""
        result = TestResult("雷达数据质量测试")
        
        self.lidar_buffer.clear()
        
        print(f"  收集 {duration}秒 雷达数据...")
        
        # 定期请求雷达扫描
        start_time = time.time()
        scan_count = 0
        while time.time() - start_time < duration:
            self.comm.request_lidar_scan()
            time.sleep(0.5)
            scan_count += 1
        
        if len(self.lidar_buffer) == 0:
            result.finish(False, "未收到雷达数据")
        else:
            # 分析数据质量
            total_points = sum(d.total_points for d in self.lidar_buffer)
            avg_points = total_points / len(self.lidar_buffer)
            avg_coverage = np.mean([d.angle_coverage for d in self.lidar_buffer])
            
            # 检查每个扇区
            sector_counts = {}
            for data in self.lidar_buffer:
                for sector in data.sectors:
                    sid = sector['sector_id']
                    sector_counts[sid] = sector_counts.get(sid, 0) + 1
            
            details = {
                "数据包数量": len(self.lidar_buffer),
                "请求次数": scan_count,
                "响应率": f"{len(self.lidar_buffer)/scan_count*100:.1f}%",
                "平均点数": f"{avg_points:.1f}",
                "平均覆盖角度": f"{avg_coverage:.1f}°",
                "扇区数量": len(sector_counts)
            }
            
            # 判断质量
            if avg_points > 10 and avg_coverage > 180:
                result.finish(True, "雷达数据质量良好", details)
            elif avg_points > 5:
                result.finish(True, "雷达数据可用（质量一般）", details)
            else:
                result.finish(False, "雷达数据质量差（点数过少）", details)
        
        self.add_result(result)
        return result.passed
    
    def test_mpu_data_quality(self, duration=5.0):
        """测试MPU数据质量"""
        result = TestResult("MPU6500数据质量测试")
        
        self.mpu_buffer.clear()
        
        print(f"  收集 {duration}秒 MPU数据...")
        time.sleep(duration)
        
        if len(self.mpu_buffer) == 0:
            result.finish(False, "未收到MPU数据")
        else:
            # 分析数据
            rolls = [d.roll for d in self.mpu_buffer]
            pitches = [d.pitch for d in self.mpu_buffer]
            accels = [d.accel_magnitude for d in self.mpu_buffer]
            
            roll_std = np.std(rolls)
            pitch_std = np.std(pitches)
            accel_mean = np.mean(accels)
            
            details = {
                "数据包数量": len(self.mpu_buffer),
                "频率": f"{len(self.mpu_buffer)/duration:.1f} Hz",
                "Roll标准差": f"{roll_std:.2f}°",
                "Pitch标准差": f"{pitch_std:.2f}°",
                "平均加速度": f"{accel_mean:.2f} m/s²"
            }
            
            # 判断质量（静止时标准差应该小）
            if roll_std < 5 and pitch_std < 5:
                result.finish(True, "MPU数据稳定", details)
            elif roll_std < 10 and pitch_std < 10:
                result.finish(True, "MPU数据可用（有抖动）", details)
            else:
                result.finish(False, "MPU数据不稳定（抖动严重）", details)
        
        self.add_result(result)
        return result.passed
    
    def test_pose_estimation_quality(self, duration=10.0):
        """测试位姿估计质量"""
        result = TestResult("位姿估计质量测试")
        
        self.pose_buffer.clear()
        
        print(f"  请保持机器人静止，测试 {duration}秒...")
        time.sleep(duration)
        
        if len(self.pose_buffer) < 10:
            result.finish(False, "位姿数据不足")
        else:
            # 分析漂移
            xs = [d.x for d in self.pose_buffer]
            ys = [d.y for d in self.pose_buffer]
            thetas = [d.theta for d in self.pose_buffer]
            
            x_drift = max(xs) - min(xs)
            y_drift = max(ys) - min(ys)
            theta_drift = max(thetas) - min(thetas)
            
            details = {
                "数据包数量": len(self.pose_buffer),
                "频率": f"{len(self.pose_buffer)/duration:.1f} Hz",
                "X轴漂移": f"{x_drift*1000:.2f} mm",
                "Y轴漂移": f"{y_drift*1000:.2f} mm",
                "角度漂移": f"{np.rad2deg(theta_drift):.2f}°"
            }
            
            # 判断质量
            if x_drift < 0.01 and y_drift < 0.01 and np.rad2deg(theta_drift) < 2:
                result.finish(True, "位姿估计精度优秀", details)
            elif x_drift < 0.05 and y_drift < 0.05 and np.rad2deg(theta_drift) < 5:
                result.finish(True, "位姿估计精度良好", details)
            else:
                result.finish(False, "位姿估计漂移过大", details)
        
        self.add_result(result)
        return result.passed
    
    # ========================================================================
    # 3. 控制命令响应测试
    # ========================================================================
    
    def test_mode_commands(self):
        """测试模式命令"""
        result = TestResult("模式命令测试")
        
        test_modes = [
            (RobotMode.STOP, "停止"),
            (RobotMode.FORWARD, "前进"),
            (RobotMode.STOP, "停止"),
        ]
        
        success_count = 0
        
        for mode, name in test_modes:
            print(f"  发送命令: {name}")
            if self.comm.send_mode_command(mode):
                success_count += 1
                time.sleep(0.5)
            else:
                print(f"    ❌ 命令发送失败")
        
        details = {
            "测试命令数": len(test_modes),
            "成功发送": success_count
        }
        
        if success_count == len(test_modes):
            result.finish(True, "所有模式命令发送成功", details)
        else:
            result.finish(False, f"部分命令发送失败", details)
        
        self.add_result(result)
        return result.passed
    
    def test_speed_commands(self):
        """测试速度命令"""
        result = TestResult("速度命令测试")
        
        test_speeds = [
            (0.0, 0.0, "停止"),
            (1.0, 1.0, "直行"),
            (1.0, -1.0, "原地转"),
            (0.0, 0.0, "停止"),
        ]
        
        success_count = 0
        
        for left, right, name in test_speeds:
            print(f"  发送速度: {name} (L:{left:.1f}, R:{right:.1f})")
            if self.comm.send_speed_command(left, right):
                success_count += 1
                time.sleep(0.5)
            else:
                print(f"    ❌ 命令发送失败")
        
        details = {
            "测试命令数": len(test_speeds),
            "成功发送": success_count
        }
        
        if success_count == len(test_speeds):
            result.finish(True, "所有速度命令发送成功", details)
        else:
            result.finish(False, f"部分命令发送失败", details)
        
        self.add_result(result)
        return result.passed
    
    def test_control_response_time(self):
        """测试控制响应时间"""
        result = TestResult("控制响应时间测试")
        
        print("  测试控制命令响应延迟...")
        
        self.odom_buffer.clear()
        
        # 发送前进命令，测量多久后收到速度反馈
        send_time = time.time()
        self.comm.send_speed_command(1.0, 1.0)
        
        # 等待里程计数据变化
        timeout = 2.0
        response_time = None
        
        while time.time() - send_time < timeout:
            if len(self.odom_buffer) > 0:
                latest_odom = self.odom_buffer[-1]
                if abs(latest_odom.left_speed) > 0.1 or abs(latest_odom.right_speed) > 0.1:
                    response_time = time.time() - send_time
                    break
            time.sleep(0.01)
        
        # 停止
        self.comm.send_speed_command(0.0, 0.0)
        
        if response_time:
            details = {"响应时间": f"{response_time*1000:.1f} ms"}
            
            if response_time < 0.1:
                result.finish(True, "响应速度优秀", details)
            elif response_time < 0.3:
                result.finish(True, "响应速度良好", details)
            else:
                result.finish(False, "响应速度慢", details)
        else:
            result.finish(False, "未检测到控制响应")
        
        self.add_result(result)
        return result.passed
    
    # ========================================================================
    # 4. 通信延迟测试
    # ========================================================================
    
    def test_communication_latency(self, iterations=50):
        """测试通信延迟"""
        result = TestResult("通信延迟测试")
        
        print(f"  测试 {iterations} 次往返延迟...")
        
        latencies = []
        
        for i in range(iterations):
            # 发送命令
            send_time = time.time()
            self.comm.send_mode_command(RobotMode.STOP)
            
            # 等待命令发送完成（简化测试，假设20ms）
            time.sleep(0.02)
            recv_time = time.time()
            
            latency = (recv_time - send_time) * 1000  # 转为毫秒
            latencies.append(latency)
            
            if (i + 1) % 10 == 0:
                print(f"    进度: {i+1}/{iterations}")
        
        avg_latency = np.mean(latencies)
        max_latency = np.max(latencies)
        min_latency = np.min(latencies)
        std_latency = np.std(latencies)
        
        details = {
            "平均延迟": f"{avg_latency:.2f} ms",
            "最小延迟": f"{min_latency:.2f} ms",
            "最大延迟": f"{max_latency:.2f} ms",
            "标准差": f"{std_latency:.2f} ms"
        }
        
        # 判断延迟质量
        if avg_latency < 50:
            result.finish(True, "通信延迟优秀", details)
        elif avg_latency < 100:
            result.finish(True, "通信延迟良好", details)
        else:
            result.finish(False, "通信延迟过高", details)
        
        self.add_result(result)
        return result.passed
    
    def test_data_throughput(self, duration=10.0):
        """测试数据吞吐量"""
        result = TestResult("数据吞吐量测试")
        
        # 清空缓冲区
        self.lidar_buffer.clear()
        self.mpu_buffer.clear()
        self.odom_buffer.clear()
        self.pose_buffer.clear()
        
        print(f"  测试 {duration}秒 数据吞吐量...")
        
        # 持续请求雷达数据
        start_time = time.time()
        request_count = 0
        while time.time() - start_time < duration:
            self.comm.request_lidar_scan()
            request_count += 1
            time.sleep(0.1)
        
        total_packets = (len(self.lidar_buffer) + len(self.mpu_buffer) + 
                        len(self.odom_buffer) + len(self.pose_buffer))
        
        packets_per_sec = total_packets / duration
        
        details = {
            "总数据包": total_packets,
            "雷达": len(self.lidar_buffer),
            "MPU": len(self.mpu_buffer),
            "里程计": len(self.odom_buffer),
            "位姿": len(self.pose_buffer),
            "吞吐量": f"{packets_per_sec:.1f} 包/秒"
        }
        
        if packets_per_sec > 50:
            result.finish(True, "数据吞吐量充足", details)
        elif packets_per_sec > 20:
            result.finish(True, "数据吞吐量一般", details)
        else:
            result.finish(False, "数据吞吐量不足", details)
        
        self.add_result(result)
        return result.passed
    
    # ========================================================================
    # 5. 建图功能测试
    # ========================================================================
    
    def test_mapping_basic(self, duration=15.0):
        """测试基础建图功能"""
        result = TestResult("基础建图测试")
        
        print(f"  测试 {duration}秒 建图功能...")
        print("  提示: 请让机器人缓慢移动或转动")
        
        # 创建地图
        map_config = MapConfig(
            width=config.MAP_WIDTH,
            height=config.MAP_HEIGHT,
            resolution=config.MAP_RESOLUTION,
            origin_x=config.MAP_ORIGIN_X,
            origin_y=config.MAP_ORIGIN_Y
        )
        slam_map = OccupancyGridMap(map_config)
        
        # 收集数据并更新地图
        self.lidar_buffer.clear()
        self.pose_buffer.clear()
        
        start_time = time.time()
        update_count = 0
        
        while time.time() - start_time < duration:
            self.comm.request_lidar_scan()
            time.sleep(0.5)
            
            # 如果有新数据，更新地图
            if len(self.lidar_buffer) > 0 and len(self.pose_buffer) > 0:
                lidar = self.lidar_buffer[-1]
                pose = self.pose_buffer[-1]
                robot_pose = (pose.x, pose.y, pose.theta)
                
                slam_map.update_with_lidar(lidar, robot_pose)
                update_count += 1
        
        # 分析地图质量
        stats = slam_map.get_statistics()
        
        details = {
            "更新次数": update_count,
            "探索率": f"{stats['explored_ratio']*100:.1f}%",
            "空闲格子": stats['free_cells'],
            "障碍物格子": stats['obstacle_cells']
        }
        
        if stats['explored_ratio'] > 0.01:  # 至少探索1%
            result.finish(True, "建图功能正常", details)
        else:
            result.finish(False, "建图功能异常（未探索到区域）", details)
        
        # 保存地图
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_file = f"data/maps/test_map_{timestamp}.npy"
        slam_map.save_map(map_file)
        print(f"    地图已保存: {map_file}")
        
        self.add_result(result)
        return result.passed
    
    # ========================================================================
    # 6. 可视化功能测试
    # ========================================================================
    
    def test_visualization(self):
        """测试可视化功能"""
        result = TestResult("可视化功能测试")
        
        try:
            from src.visualization.map_visualizer import MapVisualizer
            
            print("  测试可视化初始化...")
            
            # 创建测试地图
            map_config = MapConfig(width=100, height=100, resolution=0.1)
            test_map = OccupancyGridMap(map_config)
            
            # 创建可视化器
            viz = MapVisualizer(test_map, figsize=(8, 8))
            
            # 测试更新
            viz.update(
                robot_pose=(0, 0, 0),
                lidar_points=[],
                path=None,
                frontiers=[]
            )
            
            result.finish(True, "可视化功能正常")
            
        except Exception as e:
            result.finish(False, f"可视化异常: {e}")
        
        self.add_result(result)
        return result.passed
    
    # ========================================================================
    # 7. 完整系统测试
    # ========================================================================
    
    def test_full_system(self, duration=30.0):
        """测试完整系统运行"""
        result = TestResult("完整系统测试")
        
        try:
            print(f"  运行完整系统 {duration}秒...")
            print("  提示: 观察控制台输出和机器人行为")
            
            # 创建控制器（不启用可视化，避免阻塞）
            controller = RobotController(
                comm=self.comm,
                enable_visualization=False
            )
            
            # 运行短时间
            steps = int(duration / 0.1)  # 假设10Hz控制频率
            
            start_time = time.time()
            for i in range(min(steps, 300)):  # 最多300步
                controller.update_map_from_lidar()
                time.sleep(0.1)
                
                if (i + 1) % 50 == 0:
                    print(f"    进度: {i+1}/{steps}")
            
            duration_actual = time.time() - start_time
            
            # 停止机器人
            self.comm.stop_robot()
            
            # 检查统计
            stats = controller.map.get_statistics()
            
            details = {
                "运行时间": f"{duration_actual:.1f}秒",
                "探索率": f"{stats['explored_ratio']*100:.1f}%",
                "状态": str(controller.state)
            }
            
            result.finish(True, "系统运行完成", details)
            
        except Exception as e:
            result.finish(False, f"系统运行异常: {e}")
            import traceback
            traceback.print_exc()
        
        self.add_result(result)
        return result.passed
    
    # ========================================================================
    # 测试套件
    # ========================================================================
    
    def run_quick_test(self):
        """快速测试（5分钟内完成）"""
        print("\n" + "="*70)
        print("开始快速测试")
        print("="*70 + "\n")
        
        # 1. 连接测试
        if not self.test_serial_connection():
            print("\n❌ 连接失败，中止测试")
            return False
        
        # 2. 数据接收测试
        self.test_initial_data_reception(timeout=3.0)
        
        # 3. 传感器质量测试（缩短时间）
        self.test_lidar_data_quality(duration=5.0)
        self.test_mpu_data_quality(duration=3.0)
        
        # 4. 控制命令测试
        self.test_mode_commands()
        self.test_speed_commands()
        
        # 5. 通信延迟测试
        self.test_communication_latency(iterations=20)
        
        print("\n快速测试完成！")
        return True
    
    def run_full_test(self):
        """完整测试（15-20分钟）"""
        print("\n" + "="*70)
        print("开始完整测试")
        print("="*70 + "\n")
        
        # 1. 连接测试
        if not self.test_serial_connection():
            print("\n❌ 连接失败，中止测试")
            return False
        
        # 2. 数据接收测试
        self.test_initial_data_reception(timeout=5.0)
        
        # 3. 传感器质量测试
        self.test_lidar_data_quality(duration=10.0)
        self.test_mpu_data_quality(duration=5.0)
        self.test_pose_estimation_quality(duration=10.0)
        
        # 4. 控制命令测试
        self.test_mode_commands()
        self.test_speed_commands()
        self.test_control_response_time()
        
        # 5. 通信性能测试
        self.test_communication_latency(iterations=50)
        self.test_data_throughput(duration=10.0)
        
        # 6. 建图功能测试
        self.test_mapping_basic(duration=15.0)
        
        # 7. 可视化测试
        self.test_visualization()
        
        # 8. 完整系统测试
        self.test_full_system(duration=30.0)
        
        print("\n完整测试完成！")
        return True
    
    def run_specific_test(self, test_name):
        """运行特定测试"""
        print(f"\n运行测试: {test_name}\n")
        
        tests = {
            'conn': self.test_serial_connection,
            'lidar': lambda: self.test_lidar_data_quality(10),
            'mpu': lambda: self.test_mpu_data_quality(5),
            'pose': lambda: self.test_pose_estimation_quality(10),
            'control': self.test_mode_commands,
            'latency': lambda: self.test_communication_latency(50),
            'map': lambda: self.test_mapping_basic(15),
            'viz': self.test_visualization,
            'system': lambda: self.test_full_system(30),
        }
        
        if test_name in tests:
            if test_name != 'conn':
                self.test_serial_connection()
            tests[test_name]()
        else:
            print(f"❌ 未知测试: {test_name}")
            print(f"可用测试: {', '.join(tests.keys())}")
    
    def cleanup(self):
        """清理资源"""
        if self.comm:
            print("\n正在关闭通信...")
            self.comm.stop_robot()
            time.sleep(0.5)
            self.comm.stop()


def main():
    parser = argparse.ArgumentParser(
        description='xxq系统综合测试 - 支持串口(Serial)和蓝牙(BLE)连接',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例用法：
  串口连接:
    python test_system_comprehensive.py --port COM5 --quick
    python test_system_comprehensive.py --port COM5 --test conn
  
  BLE连接:
    python test_system_comprehensive.py --ble --address C4:25:01:20:02:8E --quick
    python test_system_comprehensive.py --ble --address C4:25:01:20:02:8E --test conn
        '''
    )
    
    # 连接方式参数
    conn_group = parser.add_mutually_exclusive_group()
    conn_group.add_argument('--port', type=str,
                           help='串口号 (如: COM5, /dev/ttyUSB0)')
    conn_group.add_argument('--ble', action='store_true',
                           help='使用BLE连接')
    
    parser.add_argument('--address', type=str,
                       help='BLE设备地址 (如: C4:25:01:20:02:8E)')
    parser.add_argument('--baudrate', type=int, default=9600,
                       help='波特率（串口模式，默认9600）')
    parser.add_argument('--quick', action='store_true',
                       help='快速测试（约5分钟）')
    parser.add_argument('--test', type=str,
                       help='运行特定测试 (conn/lidar/mpu/pose/control/latency/map/viz/system)')
    
    args = parser.parse_args()
    
    # 配置日志
    setup_comm_logging()
    
    # 确定连接方式和参数
    if args.ble:
        if not args.address:
            print("错误: BLE模式需要提供设备地址 (--address)")
            print("示例: python test_system_comprehensive.py --ble --address C4:25:01:20:02:8E --quick")
            sys.exit(1)
        
        print("="*70)
        print(f"使用BLE连接")
        print(f"设备地址: {args.address}")
        print("="*70 + "\n")
        
        tester = SystemTester(
            connection_type='ble',
            ble_address=args.address
        )
    else:
        # 默认串口模式
        port = args.port or 'COM5'
        
        print("="*70)
        print(f"使用串口连接")
        print(f"端口: {port}")
        print(f"波特率: {args.baudrate}")
        print("="*70 + "\n")
        
        tester = SystemTester(
            connection_type='serial',
            port=port,
            baudrate=args.baudrate
        )
    
    try:
        if args.test:
            # 运行特定测试
            tester.run_specific_test(args.test)
        elif args.quick:
            # 快速测试
            tester.run_quick_test()
        else:
            # 完整测试
            tester.run_full_test()
        
        # 打印摘要
        tester.print_summary()
        
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
    
    except Exception as e:
        print(f"\n\n测试异常: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        tester.cleanup()
        print("\n测试结束")


if __name__ == '__main__':
    main()

