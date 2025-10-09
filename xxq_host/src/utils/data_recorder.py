"""
数据记录与回放模块
记录机器人运行数据，支持离线分析和回放
"""

import json
import time
import pickle
from pathlib import Path
from typing import Optional, List, Dict
from datetime import datetime


class DataRecorder:
    """数据记录器
    
    功能：
    - 记录雷达、MPU、里程计、位姿数据
    - 保存为JSON或pickle格式
    - 支持数据回放
    
    Example:
        >>> recorder = DataRecorder()
        >>> recorder.start_recording('test_run.pkl')
        >>> recorder.record_lidar(lidar_data)
        >>> recorder.record_mpu(mpu_data)
        >>> recorder.stop_recording()
        >>> 
        >>> # 回放
        >>> recorder.load_recording('test_run.pkl')
        >>> for frame in recorder.replay():
        >>>     print(frame)
    """
    
    def __init__(self, data_dir: str = 'data/recordings'):
        """初始化数据记录器
        
        Args:
            data_dir: 数据保存目录
        """
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
        self.recording = False
        self.current_file = None
        self.start_time = 0
        
        # 数据缓冲区
        self.frames = []
        self.current_frame = {}
        
        # 统计信息
        self.stats = {
            'lidar_count': 0,
            'mpu_count': 0,
            'odom_count': 0,
            'pose_count': 0,
            'duration': 0
        }
    
    def start_recording(self, filename: str, format: str = 'pickle'):
        """开始记录
        
        Args:
            filename: 文件名
            format: 'pickle' 或 'json'
        """
        if self.recording:
            print("[RECORDER] 已在记录中，请先停止")
            return False
        
        # 添加时间戳到文件名
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base_name = Path(filename).stem
        ext = '.pkl' if format == 'pickle' else '.json'
        
        self.current_file = self.data_dir / f"{base_name}_{timestamp}{ext}"
        self.format = format
        self.recording = True
        self.start_time = time.time()
        self.frames = []
        
        # 重置统计
        for key in self.stats:
            self.stats[key] = 0
        
        print(f"[RECORDER] 开始记录: {self.current_file}")
        return True
    
    def record_lidar(self, data):
        """记录雷达数据"""
        if not self.recording:
            return
        
        frame = {
            'type': 'lidar',
            'timestamp': time.time() - self.start_time,
            'data': {
                'timestamp': data.timestamp,
                'total_points': data.total_points,
                'angle_coverage': data.angle_coverage,
                'sectors': data.sectors
            }
        }
        self.frames.append(frame)
        self.stats['lidar_count'] += 1
    
    def record_mpu(self, data):
        """记录MPU数据"""
        if not self.recording:
            return
        
        frame = {
            'type': 'mpu',
            'timestamp': time.time() - self.start_time,
            'data': {
                'timestamp': data.timestamp,
                'roll': data.roll,
                'pitch': data.pitch,
                'accel': data.accel,
                'gyro': data.gyro
            }
        }
        self.frames.append(frame)
        self.stats['mpu_count'] += 1
    
    def record_odom(self, data):
        """记录里程计数据"""
        if not self.recording:
            return
        
        frame = {
            'type': 'odom',
            'timestamp': time.time() - self.start_time,
            'data': {
                'timestamp': data.timestamp,
                'left_speed': data.left_speed,
                'right_speed': data.right_speed,
                'left_count': data.left_count,
                'right_count': data.right_count
            }
        }
        self.frames.append(frame)
        self.stats['odom_count'] += 1
    
    def record_pose(self, data):
        """记录位姿数据"""
        if not self.recording:
            return
        
        frame = {
            'type': 'pose',
            'timestamp': time.time() - self.start_time,
            'data': {
                'timestamp': data.timestamp,
                'x': data.x,
                'y': data.y,
                'theta': data.theta
            }
        }
        self.frames.append(frame)
        self.stats['pose_count'] += 1
    
    def stop_recording(self):
        """停止记录并保存"""
        if not self.recording:
            print("[RECORDER] 未在记录中")
            return False
        
        self.recording = False
        self.stats['duration'] = time.time() - self.start_time
        
        # 保存数据
        data_to_save = {
            'version': '1.0',
            'start_time': self.start_time,
            'duration': self.stats['duration'],
            'stats': self.stats,
            'frames': self.frames
        }
        
        if self.format == 'pickle':
            with open(self.current_file, 'wb') as f:
                pickle.dump(data_to_save, f)
        else:  # json
            with open(self.current_file, 'w') as f:
                json.dump(data_to_save, f, indent=2)
        
        print(f"[RECORDER] 记录完成: {self.current_file}")
        print(f"  - 时长: {self.stats['duration']:.1f}秒")
        print(f"  - 雷达: {self.stats['lidar_count']}帧")
        print(f"  - MPU: {self.stats['mpu_count']}帧")
        print(f"  - 里程计: {self.stats['odom_count']}帧")
        print(f"  - 位姿: {self.stats['pose_count']}帧")
        
        return True
    
    def load_recording(self, filename: str):
        """加载录制文件
        
        Args:
            filename: 文件路径
        """
        filepath = Path(filename)
        if not filepath.exists():
            # 尝试在data_dir中查找
            filepath = self.data_dir / filename
        
        if not filepath.exists():
            print(f"[RECORDER] 文件不存在: {filename}")
            return False
        
        try:
            if filepath.suffix == '.pkl':
                with open(filepath, 'rb') as f:
                    data = pickle.load(f)
            else:  # json
                with open(filepath, 'r') as f:
                    data = json.load(f)
            
            self.frames = data['frames']
            self.stats = data['stats']
            
            print(f"[RECORDER] 加载成功: {filepath}")
            print(f"  - 时长: {self.stats['duration']:.1f}秒")
            print(f"  - 总帧数: {len(self.frames)}")
            
            return True
            
        except Exception as e:
            print(f"[RECORDER] 加载失败: {e}")
            return False
    
    def replay(self, speed: float = 1.0, start_time: float = 0, end_time: float = None):
        """回放数据
        
        Args:
            speed: 回放速度倍率（1.0=正常，2.0=2倍速）
            start_time: 开始时间（秒）
            end_time: 结束时间（秒，None=全部）
        
        Yields:
            数据帧
        """
        if not self.frames:
            print("[RECORDER] 没有数据可回放")
            return
        
        if end_time is None:
            end_time = float('inf')
        
        last_time = start_time
        
        for frame in self.frames:
            frame_time = frame['timestamp']
            
            # 跳过开始时间之前的帧
            if frame_time < start_time:
                continue
            
            # 结束时间之后停止
            if frame_time > end_time:
                break
            
            # 模拟时间流逝
            delay = (frame_time - last_time) / speed
            if delay > 0:
                time.sleep(delay)
            
            last_time = frame_time
            
            yield frame
    
    def get_statistics(self):
        """获取统计信息"""
        return self.stats.copy()
    
    def export_csv(self, output_file: str, data_type: str = 'all'):
        """导出为CSV格式
        
        Args:
            output_file: 输出文件名
            data_type: 'lidar', 'mpu', 'odom', 'pose', 'all'
        """
        import csv
        
        output_path = self.data_dir / output_file
        
        with open(output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # 根据类型写入不同的头部
            if data_type == 'mpu' or data_type == 'all':
                writer.writerow(['Time', 'Type', 'Roll', 'Pitch', 'Accel_X', 'Accel_Y', 'Accel_Z', 
                               'Gyro_X', 'Gyro_Y', 'Gyro_Z'])
                
                for frame in self.frames:
                    if frame['type'] == 'mpu':
                        d = frame['data']
                        writer.writerow([
                            frame['timestamp'], 'MPU',
                            d['roll'], d['pitch'],
                            d['accel'][0], d['accel'][1], d['accel'][2],
                            d['gyro'][0], d['gyro'][1], d['gyro'][2]
                        ])
        
        print(f"[RECORDER] CSV导出完成: {output_path}")

