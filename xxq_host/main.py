"""
xxq_host 主程序入口
智能小车主机端控制系统
"""

import sys
import time
import signal
from src.communication.robot_comm import RobotComm, setup_comm_logging
from src.communication.protocol import RobotMode
import config


# 全局通信对象（用于信号处理）
comm = None


def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    print("\n\n[系统] 接收到中断信号，正在安全退出...")
    if comm:
        comm.stop_robot()
        comm.stop()
    print("[系统] 已退出")
    sys.exit(0)


def main():
    """主函数"""
    global comm
    
    # 配置日志
    setup_comm_logging()
    
    print("=" * 70)
    print(" xxq_host - 智能小车主机端控制系统")
    print("=" * 70)
    print(f"\n配置信息:")
    print(f"  串口: {config.SERIAL_PORT}")
    print(f"  波特率: {config.BAUDRATE}")
    print(f"  地图分辨率: {config.MAP_RESOLUTION}m/格")
    print(f"  最大速度: {config.MAX_SPEED}m/s")
    print()
    
    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)
    
    # 创建通信对象
    comm = RobotComm(port=config.SERIAL_PORT, baudrate=config.BAUDRATE)
    
    # 设置回调函数
    def on_lidar(data):
        print(f"[雷达] 收到数据: {data.total_points}点, 覆盖{data.angle_coverage:.1f}°")
        if len(data.sectors) > 0:
            # 显示前方扇区（扇区0）
            front = data.get_sector(0)
            if front:
                print(f"  前方: {front['count']}点, 最近{front['min_dist']:.2f}m")
    
    def on_mpu(data):
        print(f"[MPU] Roll:{data.roll:6.2f}° Pitch:{data.pitch:6.2f}° Accel:{data.accel_magnitude:.2f}m/s²")
    
    def on_odom(data):
        print(f"[里程计] 左轮:{data.left_speed:5.2f} 右轮:{data.right_speed:5.2f} RPS, 速度差:{data.speed_diff:.3f}")
    
    def on_pose(data):
        print(f"[位姿] X:{data.x:6.3f}m Y:{data.y:6.3f}m θ:{data.theta:6.2f}°")
    
    comm.on_lidar_update = on_lidar
    comm.on_mpu_update = on_mpu
    comm.on_odom_update = on_odom
    comm.on_pose_update = on_pose
    
    # 启动通信
    print("[系统] 正在连接机器人...")
    if not comm.start():
        print("[错误] 无法连接串口，请检查:")
        print(f"  1. 串口设备是否存在: ls {config.SERIAL_PORT}")
        print(f"  2. 是否有权限: sudo chmod 666 {config.SERIAL_PORT}")
        print(f"  3. STM32是否已连接并通电")
        sys.exit(1)
    
    print("[系统] 已连接，开始接收数据...")
    print("-" * 70)
    
    try:
        # 简单的交互式控制
        print("\n可用命令:")
        print("  a - 请求雷达扫描")
        print("  s - 停止机器人")
        print("  f - 前进 (1.5 RPS)")
        print("  b - 后退 (1.5 RPS)")
        print("  l - 左转")
        print("  r - 右转")
        print("  q - 退出程序")
        print()
        
        while True:
            # 非阻塞输入（简化版，实际应使用更好的方式）
            cmd = input("[命令] > ").strip().lower()
            
            if cmd == 'a':
                print("→ 请求雷达扫描")
                comm.request_lidar_scan()
                
            elif cmd == 's':
                print("→ 停止机器人")
                comm.send_mode_command(RobotMode.STOP)
                
            elif cmd == 'f':
                print("→ 前进 (1.5 RPS)")
                comm.send_speed_command(1.5, 1.5)
                
            elif cmd == 'b':
                print("→ 后退 (1.5 RPS)")
                comm.send_speed_command(-1.5, -1.5)
                
            elif cmd == 'l':
                print("→ 左转")
                comm.send_mode_command(RobotMode.TURN_LEFT)
                
            elif cmd == 'r':
                print("→ 右转")
                comm.send_mode_command(RobotMode.TURN_RIGHT)
                
            elif cmd == 'q':
                print("→ 退出程序")
                break
                
            else:
                print("未知命令，请输入 a/s/f/b/l/r/q")
    
    except KeyboardInterrupt:
        print("\n[系统] 接收到中断信号")
    
    finally:
        # 清理
        print("\n[系统] 正在关闭...")
        comm.stop_robot()
        time.sleep(0.5)
        comm.stop()
        print("[系统] 已安全退出")


if __name__ == "__main__":
    main()

