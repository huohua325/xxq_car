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
    
    # ==================== 转向参数（90度） ====================
    # 来自运动参数标定结果.md
    
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
    
    # ==================== 转向参数（45度） ====================
    # 时间为90度的一半
    
    TURN_LEFT_45 = {
        'direction': 0,
        'left_pwm': 0.85,
        'right_pwm': 0.18,
        'duration': 0.5,   # 1.0 / 2
        'angle': 45
    }
    
    TURN_RIGHT_45 = {
        'direction': 1,
        'left_pwm': 0.50,
        'right_pwm': 0.45,
        'duration': 0.365,  # 0.73 / 2
        'angle': 45
    }
    
    # ==================== 前进参数 ====================
    # 来自运动参数标定结果.md
    
    FORWARD_MEDIUM = {
        'duration': 0.8,
        'distance': 0.263,  # 米（26.3 cm）
        'speed': 0.329,     # m/s（32.9 cm/s）
        'mode': 1           # PID速度控制
    }
    
    FORWARD_LONG = {
        'duration': 1.0,
        'distance': 0.319,  # 米（31.9 cm）
        'speed': 0.319,     # m/s（31.9 cm/s）
        'mode': 1
    }
    
    def __init__(self, robot):
        """
        初始化运动控制器
        
        Args:
            robot: SimpleBLERobotComm实例
        """
        self.robot = robot
        
        # 运动参数
        self.settle_time = 0.15  # 稳定时间（秒），运动后等待
        
        # 统计信息
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
        """
        前进一步
        
        Args:
            step_type: 'medium' (26cm, 0.8秒) 或 'long' (32cm, 1.0秒)
        
        Returns:
            float: 实际前进距离（米）
        """
        cfg = self.FORWARD_MEDIUM if step_type == 'medium' else self.FORWARD_LONG
        
        self.robot.send_command(f"MODE,{cfg['mode']}\n")
        time.sleep(cfg['duration'])
        self.stop()
        
        self.motion_count['forward'] += 1
        return cfg['distance']
    
    def backward(self, duration=0.8):
        """
        后退
        
        Args:
            duration: 持续时间（秒）
        
        Returns:
            float: 估计后退距离（米）
        """
        self.robot.send_command('MODE,2\n')
        time.sleep(duration)
        self.stop()
        
        self.motion_count['backward'] += 1
        # 后退速度假设与前进相同
        return duration * 0.32
    
    def turn_left(self, angle=45):
        """
        左转
        
        Args:
            angle: 转向角度，45或90（度）
        
        Returns:
            int: 实际转向角度
        """
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
        """
        右转
        
        Args:
            angle: 转向角度，45或90（度）
        
        Returns:
            int: 实际转向角度
        """
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
        """
        获取运动统计
        
        Returns:
            dict: 统计信息
        """
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


if __name__ == '__main__':
    # 单元测试（需要实际硬件）
    print("="*60)
    print("SimpleMotionController 单元测试")
    print("="*60)
    print("\n⚠️ 此测试需要连接真实硬件")
    print("请确保小车已开机并且BLE已连接")
    print("测试将执行以下动作：")
    print("  1. 前进（medium步长）")
    print("  2. 左转45度")
    print("  3. 前进（medium步长）")
    print("  4. 右转45度")
    print("  5. 前进（long步长）")
    print("\n预期：小车前进 -> 左转 -> 前进 -> 右转 -> 前进\n")
    
    choice = input("是否继续？(y/n): ").strip().lower()
    if choice != 'y':
        print("测试取消")
        exit()
    
    # 导入BLE通信模块
    import sys
    sys.path.insert(0, '..')
    from ble_robot_control import SimpleBLERobotComm
    
    BLE_ADDRESS = "C4:25:01:20:02:8E"
    
    robot = SimpleBLERobotComm(address=BLE_ADDRESS, verbose=False)
    
    try:
        print("\n🔵 连接BLE...")
        if not robot.connect():
            print("❌ 连接失败")
            exit()
        print("✅ BLE已连接\n")
        time.sleep(1)
        
        # 创建运动控制器
        motion = SimpleMotionController(robot)
        
        # 执行测试序列
        print("\n" + "="*60)
        print("开始测试序列")
        print("="*60)
        
        print("\n1️⃣ 前进（medium步长，26cm）")
        dist = motion.forward('medium')
        print(f"   ✅ 完成，前进距离: {dist:.3f}m")
        time.sleep(1)
        
        print("\n2️⃣ 左转45度")
        angle = motion.turn_left_45()
        print(f"   ✅ 完成，转向角度: {angle}°")
        time.sleep(1)
        
        print("\n3️⃣ 前进（medium步长，26cm）")
        dist = motion.forward('medium')
        print(f"   ✅ 完成，前进距离: {dist:.3f}m")
        time.sleep(1)
        
        print("\n4️⃣ 右转45度")
        angle = motion.turn_right_45()
        print(f"   ✅ 完成，转向角度: {angle}°")
        time.sleep(1)
        
        print("\n5️⃣ 前进（long步长，32cm）")
        dist = motion.forward('long')
        print(f"   ✅ 完成，前进距离: {dist:.3f}m")
        
        # 显示统计
        print("\n" + "="*60)
        print("测试统计")
        print("="*60)
        stats = motion.get_statistics()
        print(f"总运动次数: {stats['total_motions']}")
        print("\n详细计数:")
        for action, count in stats['motion_count'].items():
            if count > 0:
                print(f"  {action}: {count}")
        
        print("\n" + "="*60)
        print("✅ 测试完成")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断")
        robot.send_command('MODE,0\n')
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        robot.send_command('MODE,0\n')
    finally:
        robot.disconnect()

