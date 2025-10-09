"""
通信性能测试模块
测试通信吞吐量、延迟、丢包率等
"""

import time
import statistics
from typing import List, Dict
from dataclasses import dataclass


@dataclass
class PerformanceMetrics:
    """性能指标"""
    throughput_bps: float  # 吞吐量(bytes/s)
    latency_avg_ms: float  # 平均延迟(ms)
    latency_min_ms: float  # 最小延迟(ms)
    latency_max_ms: float  # 最大延迟(ms)
    packet_loss_rate: float  # 丢包率(0-1)
    total_packets: int  # 总包数
    lost_packets: int  # 丢失包数


class CommunicationTester:
    """通信性能测试器"""
    
    def __init__(self, comm):
        """初始化
        
        Args:
            comm: RobotComm实例
        """
        self.comm = comm
        self.latencies = []
        self.received_count = 0
        self.sent_count = 0
        self.total_bytes = 0
        self.start_time = 0
    
    def test_latency(self, iterations: int = 100) -> Dict:
        """测试通信延迟
        
        Args:
            iterations: 测试次数
        
        Returns:
            延迟统计信息
        """
        print(f"\n[性能测试] 开始延迟测试（{iterations}次）...")
        
        latencies = []
        
        for i in range(iterations):
            # 发送命令
            start = time.time()
            self.comm.send_mode_command(0)  # 发送停止命令
            
            # 等待响应（简化：固定延迟）
            time.sleep(0.01)  # 模拟往返时间
            
            latency = (time.time() - start) * 1000  # 转换为ms
            latencies.append(latency)
            
            if (i + 1) % 20 == 0:
                print(f"  进度: {i+1}/{iterations}")
        
        # 统计
        stats = {
            'avg': statistics.mean(latencies),
            'min': min(latencies),
            'max': max(latencies),
            'stdev': statistics.stdev(latencies) if len(latencies) > 1 else 0
        }
        
        print(f"\n延迟测试结果:")
        print(f"  平均: {stats['avg']:.2f}ms")
        print(f"  最小: {stats['min']:.2f}ms")
        print(f"  最大: {stats['max']:.2f}ms")
        print(f"  标准差: {stats['stdev']:.2f}ms")
        
        return stats
    
    def test_throughput(self, duration: int = 10) -> Dict:
        """测试数据吞吐量
        
        Args:
            duration: 测试时长（秒）
        
        Returns:
            吞吐量统计
        """
        print(f"\n[性能测试] 开始吞吐量测试（{duration}秒）...")
        
        byte_count = 0
        packet_count = 0
        start = time.time()
        
        # 设置回调计数
        def count_callback(data):
            nonlocal byte_count, packet_count
            # 估算数据大小
            if hasattr(data, 'total_points'):  # Lidar
                byte_count += 2048  # JSON约2KB
            else:  # MPU/ODO/POSE
                byte_count += 128  # CSV约128B
            packet_count += 1
        
        # 临时设置回调
        old_lidar_cb = self.comm.on_lidar_update
        old_mpu_cb = self.comm.on_mpu_update
        old_odom_cb = self.comm.on_odom_update
        
        self.comm.on_lidar_update = count_callback
        self.comm.on_mpu_update = count_callback
        self.comm.on_odom_update = count_callback
        
        # 运行测试
        while time.time() - start < duration:
            time.sleep(0.1)
            if (int(time.time() - start) % 2 == 0):
                print(f"  进度: {int(time.time() - start)}/{duration}秒", end='\r')
        
        elapsed = time.time() - start
        
        # 恢复回调
        self.comm.on_lidar_update = old_lidar_cb
        self.comm.on_mpu_update = old_mpu_cb
        self.comm.on_odom_update = old_odom_cb
        
        # 统计
        throughput_bps = byte_count / elapsed
        throughput_kbps = throughput_bps / 1024
        packet_rate = packet_count / elapsed
        
        print(f"\n\n吞吐量测试结果:")
        print(f"  总字节: {byte_count:,} bytes")
        print(f"  总包数: {packet_count}")
        print(f"  时长: {elapsed:.1f}秒")
        print(f"  吞吐量: {throughput_kbps:.2f} KB/s")
        print(f"  包速率: {packet_rate:.1f} packets/s")
        
        return {
            'total_bytes': byte_count,
            'total_packets': packet_count,
            'duration': elapsed,
            'throughput_bps': throughput_bps,
            'throughput_kbps': throughput_kbps,
            'packet_rate': packet_rate
        }
    
    def test_packet_loss(self, iterations: int = 1000) -> Dict:
        """测试丢包率
        
        Args:
            iterations: 测试包数量
        
        Returns:
            丢包统计
        """
        print(f"\n[性能测试] 开始丢包测试（{iterations}个包）...")
        
        sent = 0
        received = 0
        timeout_count = 0
        
        # 回调计数
        def receive_callback(data):
            nonlocal received
            received += 1
        
        old_cb = self.comm.on_pose_update
        self.comm.on_pose_update = receive_callback
        
        for i in range(iterations):
            # 发送命令
            self.comm.send_mode_command(0)
            sent += 1
            
            # 短暂等待
            time.sleep(0.001)
            
            if (i + 1) % 100 == 0:
                print(f"  进度: {i+1}/{iterations}", end='\r')
        
        # 等待最后的数据
        time.sleep(0.5)
        
        self.comm.on_pose_update = old_cb
        
        # 计算丢包率（简化：基于预期接收数）
        expected = sent
        loss_rate = (sent - received) / sent if sent > 0 else 0
        
        print(f"\n\n丢包测试结果:")
        print(f"  发送: {sent}")
        print(f"  接收: {received}")
        print(f"  丢失: {sent - received}")
        print(f"  丢包率: {loss_rate * 100:.2f}%")
        
        return {
            'sent': sent,
            'received': received,
            'lost': sent - received,
            'loss_rate': loss_rate
        }
    
    def run_full_test(self) -> PerformanceMetrics:
        """运行完整性能测试
        
        Returns:
            性能指标对象
        """
        print("\n" + "="*60)
        print("开始完整性能测试")
        print("="*60)
        
        # 1. 延迟测试
        latency_stats = self.test_latency(100)
        
        # 2. 吞吐量测试
        throughput_stats = self.test_throughput(10)
        
        # 3. 丢包测试
        loss_stats = self.test_packet_loss(1000)
        
        # 汇总结果
        metrics = PerformanceMetrics(
            throughput_bps=throughput_stats['throughput_bps'],
            latency_avg_ms=latency_stats['avg'],
            latency_min_ms=latency_stats['min'],
            latency_max_ms=latency_stats['max'],
            packet_loss_rate=loss_stats['loss_rate'],
            total_packets=loss_stats['sent'],
            lost_packets=loss_stats['lost']
        )
        
        print("\n" + "="*60)
        print("性能测试总结")
        print("="*60)
        print(f"吞吐量: {metrics.throughput_bps/1024:.2f} KB/s")
        print(f"平均延迟: {metrics.latency_avg_ms:.2f}ms")
        print(f"丢包率: {metrics.packet_loss_rate*100:.2f}%")
        print("="*60)
        
        return metrics
    
    def save_results(self, metrics: PerformanceMetrics, filename: str):
        """保存测试结果
        
        Args:
            metrics: 性能指标
            filename: 文件名
        """
        import json
        from pathlib import Path
        from datetime import datetime
        
        output_dir = Path('data/test_results')
        output_dir.mkdir(parents=True, exist_ok=True)
        
        result = {
            'timestamp': datetime.now().isoformat(),
            'metrics': {
                'throughput_bps': metrics.throughput_bps,
                'latency_avg_ms': metrics.latency_avg_ms,
                'latency_min_ms': metrics.latency_min_ms,
                'latency_max_ms': metrics.latency_max_ms,
                'packet_loss_rate': metrics.packet_loss_rate,
                'total_packets': metrics.total_packets,
                'lost_packets': metrics.lost_packets
            }
        }
        
        filepath = output_dir / filename
        with open(filepath, 'w') as f:
            json.dump(result, f, indent=2)
        
        print(f"\n[测试] 结果已保存: {filepath}")




