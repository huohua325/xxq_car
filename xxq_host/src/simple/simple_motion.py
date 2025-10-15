#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
运动控制包装器
标准化运动命令，支持45度和90度转向
基于运动参数标定结果.md的标定数据
"""

import time


class SimpleMotionController:
    """简单运动控制器（基于标定数据）"""
    
    # 转向参数（90度）
    TURN_LEFT_90 = {
        'direction': 0,
        'left_pwm': 0.85,
        'right_pwm': 0.18,
        'duration': 1.0,
        'angle': 90
    }
    
    TURN_RIGHT_90 = {
        'direction': 1,
        'left_pwm': 0.50,
        'right_pwm': 0.45,
        'duration': 0.73,
        'angle': 90
    }
    
    # 转向参数（45度）
    TURN_LEFT_45 = {
        'direction': 0,
        'left_pwm': 0.85,
        'right_pwm': 0.18,
        'duration': 0.52,
        'angle': 45
    }
    
    TURN_RIGHT_45 = {
        'direction': 1,
        'left_pwm': 0.50,
        'right_pwm': 0.45,
        'duration': 0.47,  # 从0.365增加到0.40
        'angle': 45
    }
    
    # 前进参数
    FORWARD_MEDIUM = {
        'duration': 0.8,
        'distance': 0.26,  # 恢复为实际物理距离26cm
        'speed': 0.325,    # 对应速度
        'mode': 1
    }
    
    FORWARD_LONG = {
        'duration': 1.0,
        'distance': 0.319,
        'speed': 0.319,
        'mode': 1
    }
    
    def __init__(self, robot):
        """
        初始化运动控制器
        
        Args:
            robot: SimpleBLERobotComm实例
        """
        self.robot = robot
        self.settle_time = 0.15
        
        self.motion_count = {
            'forward': 0,
            'backward': 0,
            'turn_left_45': 0,
            'turn_right_45': 0,
            'turn_left_90': 0,
            'turn_right_90': 0,
            'stop': 0
        }
    
    def stop(self):
        """停止运动"""
        self.robot.send_command('MODE,0\n')
        time.sleep(self.settle_time)
        self.motion_count['stop'] += 1
    
    def forward(self, step_type='medium'):
        """前进一步"""
        cfg = self.FORWARD_MEDIUM if step_type == 'medium' else self.FORWARD_LONG
        
        self.robot.send_command(f"MODE,{cfg['mode']}\n")
        time.sleep(cfg['duration'])
        self.stop()
        
        self.motion_count['forward'] += 1
        return cfg['distance']
    
    def backward(self, duration=0.8):
        """后退"""
        self.robot.send_command('MODE,2\n')
        time.sleep(duration)
        self.stop()
        
        self.motion_count['backward'] += 1
        return duration * 0.32
    
    def turn_left(self, angle=45):
        """左转"""
        if angle == 45:
            cfg = self.TURN_LEFT_45
            count_key = 'turn_left_45'
        else:
            cfg = self.TURN_LEFT_90
            count_key = 'turn_left_90'
        
        self.robot.send_command(
            f"TURN,{cfg['direction']},{cfg['left_pwm']:.2f},{cfg['right_pwm']:.2f}\n"
        )
        time.sleep(cfg['duration'])
        self.stop()
        
        self.motion_count[count_key] += 1
        return cfg['angle']
    
    def turn_right(self, angle=45):
        """右转"""
        if angle == 45:
            cfg = self.TURN_RIGHT_45
            count_key = 'turn_right_45'
        else:
            cfg = self.TURN_RIGHT_90
            count_key = 'turn_right_90'
        
        self.robot.send_command(
            f"TURN,{cfg['direction']},{cfg['left_pwm']:.2f},{cfg['right_pwm']:.2f}\n"
        )
        time.sleep(cfg['duration'])
        self.stop()
        
        self.motion_count[count_key] += 1
        return cfg['angle']
    
    def turn_left_45(self):
        """左转45度（快捷方法）"""
        return self.turn_left(45)
    
    def turn_right_45(self):
        """右转45度（快捷方法）"""
        return self.turn_right(45)
    
    def turn_left_90(self):
        """左转90度（快捷方法）"""
        return self.turn_left(90)
    
    def turn_right_90(self):
        """右转90度（快捷方法）"""
        return self.turn_right(90)
    
    def get_statistics(self):
        """获取运动统计"""
        return {
            'motion_count': self.motion_count.copy(),
            'total_motions': sum(self.motion_count.values())
        }
    
    def reset_statistics(self):
        """重置统计信息"""
        for key in self.motion_count:
            self.motion_count[key] = 0
    
    def __repr__(self):
        """字符串表示"""
        total = sum(self.motion_count.values())
        return f"SimpleMotionController(motions={total})"
